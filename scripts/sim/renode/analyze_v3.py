#!/usr/bin/env python3
"""Summarise v3 runs (in-ESC SIM_PROFILE sine), one row per run.

Definitions (all from 25 us virtual-time samples):
  f_meas        command frequency measured from sim_profile_out: the
                frequency (searched within +-2 % of nominal) that maximises the
                least-squares sinusoid amplitude of sim_profile_out. All
                fundamentals below are fitted at f_meas.
  window        sine segment minus its first 0.05 s, trimmed to an integer
                number of cycles at f_meas.
  fit           y = c0 + c1*(t-tm) + A sin(w t) + B cos(w t); amp = hypot(A,B).
                Lags are phase of the command fundamental minus phase of the
                quantity, in degrees, wrapped to [-180, 180) (positive =
                quantity lags the command).
  cmd_amp       amp of sim_profile_out (duty units)
  duty_amp/lag  last_duty_cycle (duty after the ramp limiter, 0..2000)
  rpm_*         physics true mechanical RPM: mean, fundamental amp and 2*amp,
                raw max-min
  idc_*         "DC-equivalent" motor current = (|ia|+|ib|+|ic|)/2 from the
                physics phase currents (the current in the two conducting
                phases); fundamental amp, lag, and mean
  ibus_*        physics bus current (into the bridge) fundamental amp, mean
  tau_*         electromagnetic torque reconstructed as in motor.c:
                tau = ke * sum_p shape(theta_e + p*2pi/3) * i_p,
                ke = 0.5*60/(2 pi kv), theta_e = theta_mech * poles/2,
                shape = AM32 SITL trapezoid. Mean, fundamental amp, lag.
                tau_load_check = damping*w + k*w^2 at the mean speed (the
                baseline tau_mean should match it).
  iph_rms, pcu  sqrt(mean(ia^2+ib^2+ic^2)/3) and 3*Irms^2*0.0772
  *_base        same quantity over the 0.3 s baseline before the sine
  desyncs       desync_happened increase (uint8, wrap-safe) over the sine
  stress_*      PR #438 stress per 50 ms window (host replica, see v2);
                status_or = OR of status events over sine windows
  ticks_per_cyc control-loop ticks per modulation cycle (19607.843/f_meas)
  pwm_per_cyc   PWM periods per cycle, 120 MHz/(mean tim1_arr+1)/f_meas
  comm_per_cyc  commutations per cycle, 6*(poles/2)*rpm/60 / f_meas
"""
import argparse
import csv
import glob
import json
import math
import os

import numpy as np

HAKEA = os.path.expanduser("~/vehicles/hakea")
R_PHASE = 0.0772
LOOP_HZ = 1e6 / 51


def lsq(t, y, f):
    w = 2 * math.pi * f
    tm = t.mean()
    X = np.column_stack([np.ones_like(t), t - tm, np.sin(w * t), np.cos(w * t)])
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    return math.hypot(c[2], c[3]), math.atan2(c[3], c[2]), c[0]


def refine_f(t, y, f0):
    fs = np.linspace(f0 * 0.98, f0 * 1.02, 401)
    amps = [lsq(t, y, f)[0] for f in fs]
    i = int(np.argmax(amps))
    lo, hi = fs[max(i - 1, 0)], fs[min(i + 1, len(fs) - 1)]
    for _ in range(40):  # golden section on the bracket
        a = lo + (hi - lo) * 0.382
        b = lo + (hi - lo) * 0.618
        if lsq(t, y, a)[0] > lsq(t, y, b)[0]:
            hi = b
        else:
            lo = a
    return 0.5 * (lo + hi)


def lag(phc, ph):
    return (math.degrees(phc - ph) + 180) % 360 - 180


def shape(te):
    s = math.pi / 6
    te = np.mod(te, 2 * math.pi)
    out = np.where(te < s, te / s,
          np.where(te < math.pi - s, 1.0,
          np.where(te < math.pi + s, (math.pi - te) / s,
          np.where(te < 2 * math.pi - s, -1.0, (te - 2 * math.pi) / s))))
    return out


def analyse(csv_path, meta):
    rows = list(csv.DictReader(open(csv_path)))
    c = {}
    for k in rows[0]:
        if k == "seg":
            c[k] = np.array([r[k] for r in rows])
        else:
            c[k] = np.array([float(r[k]) if r[k] != "" else np.nan for r in rows])
    mot = meta["model_json"]["motor"]
    pp = mot["poles"] / 2
    ke = 0.5 * 60.0 / (2 * math.pi * mot["kv"])
    ia, ib, ic = c["ia"], c["ib"], c["ic"]
    te = c["theta"] * pp
    tau = ke * (shape(te) * ia + shape(te + 2 * math.pi / 3) * ib
                + shape(te + 4 * math.pi / 3) * ic)
    idc = (np.abs(ia) + np.abs(ib) + np.abs(ic)) / 2
    t = c["t_s"]
    sine = np.where(c["seg"] == "sine")[0]
    base = np.where(c["seg"] == "base")[0]
    t0 = t[sine[0]]
    f0 = meta["freq_requested"]
    w = sine[t[sine] >= t0 + 0.05]
    fm = refine_f(t[w], c["sim_profile_out"][w], f0)
    ncyc = int((t[w[-1]] - t[w[0]]) * fm)
    w = w[t[w] < t[w[0]] + ncyc / fm]
    tw = t[w]

    def fit(y):
        return lsq(tw, y[w], fm)

    ca, cph, _ = fit(c["sim_profile_out"])
    da, dph, _ = fit(c["last_duty_cycle"])
    ra, rph, _ = fit(c["rpm_true"])
    ia_, iph, _ = fit(idc)
    ba, bph, _ = fit(c["ibus_true"])
    ta, tph, _ = fit(tau)

    def rms(idx):
        return math.sqrt(float(np.mean(ia[idx] ** 2 + ib[idx] ** 2 + ic[idx] ** 2)) / 3)

    rpm_mean = float(c["rpm_true"][w].mean())
    om = rpm_mean * 2 * math.pi / 60
    om_b = float(c["rpm_true"][base].mean()) * 2 * math.pi / 60
    tl = lambda o: mot["damping"] * o + mot["load_k_omega2"] * o * o
    ev = c["status_take"][sine]
    ev = ev[~np.isnan(ev)].astype(int)
    st = c["stress_take"][sine]
    st = st[~np.isnan(st)]
    stb = c["stress_take"][base]
    stb = stb[~np.isnan(stb)]
    orv = 0
    for e in ev:
        orv |= int(e)
    dd = np.diff(np.concatenate([[c["desync_happened"][sine[0] - 1]],
                                 c["desync_happened"][sine]]).astype(int))
    dd[dd < 0] += 256
    arr = float(np.nanmean(c["tim1_arr"][w]))
    irms_b, irms_s = rms(base), rms(w)
    duty_mean_b = float(c["last_duty_cycle"][base].mean())
    return {
        "run": meta["name"], "variant": meta["name"].split("_f")[0],
        "elf_sha256": meta["elf_sha256"][:12], "model_sha256": meta["model_sha256"][:12],
        "batch_us": meta["bridge_batch_us"], "f_req": f0, "f_meas": round(fm, 3),
        "amp_req": meta["amp"], "cycles": ncyc,
        "ramp": int(np.nanmax(c["max_ramp_high_rpm"])),
        "cmd_amp": round(ca, 1),
        "duty_amp": round(da, 1), "duty_lag_deg": round(lag(cph, dph), 1),
        "duty_amp_ratio": round(da / ca, 3) if ca else "",
        "duty_mean_shift": round(float(c["last_duty_cycle"][w].mean()) - duty_mean_b, 1),
        "rpm_mean_base": round(float(c["rpm_true"][base].mean()), 1),
        "rpm_mean": round(rpm_mean, 1), "rpm_amp": round(ra, 2), "rpm_pp_fit": round(2 * ra, 2),
        "rpm_pp_raw": round(float(c["rpm_true"][w].max() - c["rpm_true"][w].min()), 1),
        "rpm_lag_deg": round(lag(cph, rph), 1),
        "idc_mean_base": round(float(idc[base].mean()), 2), "idc_mean": round(float(idc[w].mean()), 2),
        "idc_amp": round(ia_, 2), "idc_lag_deg": round(lag(cph, iph), 1),
        "ibus_mean_base": round(float(c["ibus_true"][base].mean()), 2),
        "ibus_mean": round(float(c["ibus_true"][w].mean()), 2), "ibus_amp": round(ba, 2),
        "ibus_lag_deg": round(lag(cph, bph), 1),
        "tau_mean_base_mNm": round(1e3 * float(tau[base].mean()), 2),
        "tau_load_check_base_mNm": round(1e3 * tl(om_b), 2),
        "tau_mean_mNm": round(1e3 * float(tau[w].mean()), 2), "tau_amp_mNm": round(1e3 * ta, 2),
        "tau_lag_deg": round(lag(cph, tph), 1),
        "iph_rms_base": round(irms_b, 2), "iph_rms": round(irms_s, 2),
        "pcu_base_W": round(3 * irms_b ** 2 * R_PHASE, 2), "pcu_W": round(3 * irms_s ** 2 * R_PHASE, 2),
        "desyncs": int(dd.sum()), "status_or": "0x%02X" % orv,
        "n_warn": int(sum(1 for e in ev if e & 0x40)), "n_alert": int(sum(1 for e in ev if e & 0x80)),
        "stress_base_med": float(np.median(stb)) if len(stb) else "",
        "stress_med": float(np.median(st)) if len(st) else "",
        "stress_max": int(st.max()) if len(st) else "",
        "stress_windows": len(st),
        "ticks_per_cyc": round(LOOP_HZ / fm, 2),
        "pwm_khz": round(120e6 / (arr + 1) / 1e3, 2), "pwm_per_cyc": round(120e6 / (arr + 1) / fm, 1),
        "comm_per_cyc": round(6 * pp * rpm_mean / 60 / fm, 2),
        "running_min": int(np.nanmin(c["running"][w])),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default=os.path.join(HAKEA, "notes/sim/renode/v3/raw"))
    ap.add_argument("--out", default=os.path.join(HAKEA, "notes/sim/renode/v3/summary_v3.csv"))
    args = ap.parse_args()
    rows = []
    for mp in sorted(glob.glob(os.path.join(args.raw, "*.json"))):
        meta = json.load(open(mp))
        cp = mp[:-5] + ".csv"
        if os.path.exists(cp):
            rows.append(analyse(cp, meta))
    rows.sort(key=lambda r: (r["batch_us"], r["variant"], r["f_req"], r["amp_req"]))
    with open(args.out, "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    show = ["variant", "batch_us", "f_meas", "cmd_amp", "duty_amp", "duty_lag_deg",
            "duty_mean_shift", "rpm_mean", "rpm_amp", "rpm_pp_fit", "idc_amp", "idc_lag_deg",
            "ibus_amp", "tau_amp_mNm", "tau_lag_deg", "iph_rms", "pcu_W", "desyncs",
            "status_or", "stress_med", "stress_max", "ticks_per_cyc", "pwm_per_cyc", "comm_per_cyc"]
    print("| " + " | ".join(show) + " |")
    print("|" + "---|" * len(show))
    for r in rows:
        print("| " + " | ".join(str(r[k]) for k in show) + " |")
    print("wrote", args.out)


if __name__ == "__main__":
    main()
