#!/usr/bin/env python3
"""Summarise v2 Renode runs (one sine per run, see run_crosscheck_v2.py).

Per run, over the sine segment minus its first 0.1 s (exactly 1.0 s, i.e. an
integer number of cycles at 250 and 300 Hz), and over the last 0.9 s of the
settle segment as a baseline:

  rpm_mean, rpm_amp_f / rpm_pp_f   fundamental at f from the least-squares fit
                                   c0 + c1 t + A sin + B cos; amp = hypot(A,B),
                                   pp = 2*amp
  rpm_pp_raw                       max - min
  duty_*                           same on last_duty_cycle (committed post-ramp
                                   duty, 0..2000); duty_shift = sine mean -
                                   settle mean
  duty_lag_deg / rpm_lag_deg       phase lag behind the commanded sine (the
                                   first ~11 deg at 300 Hz / 9 deg at 250 Hz is
                                   one 100 us sample of measurement latency)
  desyncs_sine / desyncs_run       desync_happened increase (uint8, wraps
                                   at 256 on F421 - handled)
  bemf_to_max                      max bemf_timeout_happened in the segment
  stress_*                         PR #438 commutation stress (0..255) from the
                                   host-side take every window (hakea_stim_v2.py)
  status_or / n_warn / n_alert / n_err
                                   OR of status events over the segment's
                                   windows, and windows holding WARNING (0x40,
                                   desync) / ALERT (0x80, bemf timeout) /
                                   ERROR (0x20, stuck rotor)
  edt_max_comm_stress              what the status frame would carry: max
                                   window stress while armed (>>4 in-frame)
  zc_odd_frac                      fraction of samples whose thiszctime is
                                   <0.75x or >1.25x the segment median (zero
                                   cross displaced by >= 1/4 step)
  ibus_mean_A                      firmware actual_current / 100
  iph_rms_A, pcu_W                 phase current RMS from the physics
                                   (sqrt(mean(ia^2+ib^2+ic^2)/3), sampled every
                                   100 us asynchronously to the electrical
                                   period) and copper loss 3*Irms^2*R_PHASE
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


def fit(t, y, f):
    w = 2 * math.pi * f
    X = np.column_stack([np.ones_like(t), t - t.mean(), np.sin(w * t), np.cos(w * t)])
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    return math.hypot(c[2], c[3]), math.degrees(math.atan2(c[3], c[2]))


def delta(a, bits):
    d = np.diff(a.astype(np.int64))
    d[d < 0] += 1 << bits
    return int(d.sum())


def seg_stats(c, sl, step_s):
    out = {}
    rpm = c["rpm_true"][sl]
    duty = c["last_duty_cycle"][sl]
    out["rpm_mean"] = float(rpm.mean())
    out["rpm_pp_raw"] = float(rpm.max() - rpm.min())
    out["duty_mean"] = float(duty.mean())
    ia, ib, ic = c["ia"][sl], c["ib"][sl], c["ic"][sl]
    irms = math.sqrt(float(np.mean(ia ** 2 + ib ** 2 + ic ** 2)) / 3)
    out["iph_rms_A"] = irms
    out["pcu_W"] = 3 * irms ** 2 * R_PHASE
    out["ibus_mean_A"] = float(np.nanmean(c["actual_current"][sl])) / 100
    st = c["stress_take"][sl]
    st = st[~np.isnan(st)]
    out["stress_max"] = int(st.max()) if len(st) else ""
    out["stress_med"] = float(np.median(st)) if len(st) else ""
    out["stress_mean"] = round(float(st.mean()), 1) if len(st) else ""
    ev = c["status_take"][sl]
    ev = ev[~np.isnan(ev)].astype(int)
    orv = 0
    for e in ev:
        orv |= int(e)
    out["status_or"] = "0x%02X" % orv
    out["n_warn"] = int(sum(1 for e in ev if e & 0x40))
    out["n_alert"] = int(sum(1 for e in ev if e & 0x80))
    out["n_err"] = int(sum(1 for e in ev if e & 0x20))
    z = c["thiszctime"][sl]
    med = np.median(z)
    out["zc_odd_frac"] = float(np.mean((z < 0.75 * med) | (z > 1.25 * med))) if med else ""
    out["bemf_to_max"] = int(np.nanmax(c["bemf_timeout_happened"][sl]))
    return out


def analyse(path, meta):
    rows = list(csv.DictReader(open(path)))
    c = {k: np.array([float(r[k]) if r[k] != "" else np.nan for r in rows]) for k in rows[0]}
    step_s = meta["step_us"] * 1e-6
    f = meta["freq"]
    segs = {s["name"]: s for s in meta["segments"]}
    se = segs["settle"]
    sb = slice(se["end_step"] - int(round(0.9 / step_s)), se["end_step"])
    ss = segs["sine"]
    s0 = ss["start_step"] + int(round(0.1 / step_s))
    ncyc = int(round((ss["end_step"] - s0) * step_s * f))
    s1 = s0 + int(round(ncyc / f / step_s))
    sl = slice(s0, s1)
    base = seg_stats(c, sb, step_s)
    sine = seg_stats(c, sl, step_s)
    t = c["t_s"][sl]
    ra, rph = fit(t, c["rpm_true"][sl], f)
    da, dph = fit(t, c["last_duty_cycle"][sl], f)
    ca, cph = fit(t, c["cmd"][sl], f)
    full = slice(ss["start_step"] - 1, ss["end_step"])
    takes = c["stress_take"][: ss["end_step"]]
    armed = c["armed"][: ss["end_step"]]
    ok = ~np.isnan(takes) & (armed == 1)
    r = {
        "run": meta["name"], "variant": meta.get("variant", ""),
        "elf_sha256": meta["elf_sha256"][:12], "model_sha256": meta["model_sha256"][:12],
        "batch_us": meta["bridge_batch_us"], "freq": f, "amp": meta["amp"], "cycles": ncyc,
        "ramp": int(np.nanmax(c["max_ramp_high_rpm"])),
        "rpm_settle": round(base["rpm_mean"], 1), "rpm_mean": round(sine["rpm_mean"], 1),
        "rpm_amp_f": round(ra, 1), "rpm_pp_f": round(2 * ra, 1),
        "rpm_pp_raw": round(sine["rpm_pp_raw"], 1), "rpm_lag_deg": round((cph - rph) % 360, 1),
        "cmd_amp_f": round(ca, 1),
        "duty_settle": round(base["duty_mean"], 1), "duty_amp_f": round(da, 1),
        "duty_pp_f": round(2 * da, 1),
        "duty_pp_raw": int(np.nanmax(c["last_duty_cycle"][sl]) - np.nanmin(c["last_duty_cycle"][sl])),
        "duty_shift": round(sine["duty_mean"] - base["duty_mean"], 1),
        "duty_lag_deg": round((cph - dph) % 360, 1),
        "desyncs_sine": delta(c["desync_happened"][full], 8),
        "desyncs_run": delta(c["desync_happened"], 8),
        "bemf_to_max": sine["bemf_to_max"],
        "stress_settle_max": base["stress_max"], "stress_settle_med": base["stress_med"],
        "stress_max": sine["stress_max"], "stress_med": sine["stress_med"],
        "stress_mean": sine["stress_mean"],
        "edt_max_comm_stress": int(np.nanmax(takes[ok])) if ok.any() else "",
        "status_or": sine["status_or"], "n_warn": sine["n_warn"],
        "n_alert": sine["n_alert"], "n_err": sine["n_err"],
        "zc_odd_settle": round(base["zc_odd_frac"], 3), "zc_odd_sine": round(sine["zc_odd_frac"], 3),
        "ibus_settle_A": round(base["ibus_mean_A"], 2), "ibus_mean_A": round(sine["ibus_mean_A"], 2),
        "iph_rms_settle_A": round(base["iph_rms_A"], 2), "iph_rms_A": round(sine["iph_rms_A"], 2),
        "pcu_settle_W": round(base["pcu_W"], 2), "pcu_W": round(sine["pcu_W"], 2),
        "running_min": int(np.nanmin(c["running"][sl])),
        "dshot_bad": delta(c["dshot_badcounts"], 16),
    }
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default=os.path.join(HAKEA, "notes/sim/renode/v2/raw"))
    ap.add_argument("--out", default=os.path.join(HAKEA, "notes/sim/renode/v2/summary_v2.csv"))
    args = ap.parse_args()
    rows = []
    for mp in sorted(glob.glob(os.path.join(args.raw, "*.json"))):
        meta = json.load(open(mp))
        cp = mp[:-5] + ".csv"
        if os.path.exists(cp) and "freq" in meta:
            meta.setdefault("variant", meta["name"].split("_f")[0])
            rows.append(analyse(cp, meta))
    rows.sort(key=lambda r: (r["batch_us"], r["variant"], r["freq"], r["amp"]))
    with open(args.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    show = ["variant", "batch_us", "freq", "amp", "rpm_mean", "rpm_amp_f", "rpm_pp_f",
            "duty_amp_f", "duty_pp_f", "duty_shift", "desyncs_sine", "stress_settle_med",
            "stress_med", "stress_max", "status_or", "n_warn", "n_alert", "zc_odd_sine",
            "ibus_mean_A", "iph_rms_A", "pcu_W"]
    print("| " + " | ".join(show) + " |")
    print("|" + "---|" * len(show))
    for r in rows:
        print("| " + " | ".join(str(r[k]) for k in show) + " |")
    print("wrote", args.out)


if __name__ == "__main__":
    main()
