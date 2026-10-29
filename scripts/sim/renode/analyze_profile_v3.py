#!/usr/bin/env python3
"""Analyse the v3 PROFILE_MODE Renode run (run_profile_v3.py output).

Definitions:
  step start     time the firmware entered a programme step, from the 1 ms
                 coarse log: t(first sample in the step) - step_ticks/f_loop,
                 so it is exact to one loop tick, not to 1 ms.
  f_loop         control-loop ticks per virtual second, from a straight-line
                 fit of hakea step_ticks against time inside each step.
  modulation     delivered: (adjusted_duty_cycle - 1) * 2000 / tim1_arr
                 minus last_duty_cycle (the post-ramp base), from the 25 us
                 fine log. (Reconstruction quantum 2000/tim1_arr ~ 0.4.)
                 ~10 % of samples pair an adjusted_duty_cycle computed in the
                 last tick with a tim1_arr the main loop has just changed
                 (variable PWM flips ARR 4200..4999 at this speed); those
                 give spikes (recon_skew_frac). The fundamental fit is
                 insensitive to them; min/max are therefore taken from the
                 firmware's own hakea_profile.c:modulation (fw_mod_*).
  f_meas         frequency maximising the least-squares sinusoid amplitude of
                 the delivered modulation (search +-5 % around the nominal).
  fit            y = c0 + c1 (t - tm) + A sin + B cos at f_meas over an integer
                 number of cycles; amp = hypot(A, B); lag = phase(mod) -
                 phase(y), degrees in [-180, 180), positive = y lags.
  idc            (|ia|+|ib|+|ic|)/2 from the physics phase currents (the
                 current in the two conducting phases)
  tau            ke * sum_p shape(theta_e + p*120deg) * i_p, ke = 0.5*60/(2 pi kv)
                 (same as motor.c; checked against the load torque at settle)
  iph_rms, pcu   sqrt(mean(ia^2+ib^2+ic^2)/3), 3*Irms^2*0.0772
  desyncs        desync_happened increase over the step (uint8 wrap safe)
  stress         PR #438 stress per 50 ms window (host replica), windows
                 fully inside the step
"""
import argparse
import csv
import json
import math
import os

import numpy as np

HAKEA = os.path.expanduser("~/vehicles/hakea")
RAW = os.path.join(HAKEA, "notes/sim/renode/v3/raw")
R_PHASE = 0.0772
PROGRAMME = [(0, 0, 3000), (250, 40, 3000), (0, 0, 1000), (250, 100, 3000), (0, 0, 1000),
             (400, 100, 3000), (0, 0, 1000), (600, 100, 3000), (0, 0, 1000),
             (600, 200, 3000), (0, 0, 1000), (800, 100, 3000), (0, 0, 1000),
             (1000, 100, 3000), (0, 0, 1000)]
HP = "hakea_profile.c:"


def load(path):
    rows = list(csv.DictReader(open(path)))
    out = {}
    for k in rows[0]:
        if k == "phase":
            out[k] = np.array([r[k] for r in rows])
        else:
            out[k] = np.array([float(r[k]) if r[k] not in ("",) else np.nan for r in rows])
    return out


def lsq(t, y, f):
    w = 2 * math.pi * f
    X = np.column_stack([np.ones_like(t), t - t.mean(), np.sin(w * t), np.cos(w * t)])
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    return math.hypot(c[2], c[3]), math.atan2(c[3], c[2]), c[0]


def refine(t, y, f0):
    fs = np.linspace(f0 * 0.95, f0 * 1.05, 801)
    a = [lsq(t, y, f)[0] for f in fs]
    i = int(np.argmax(a))
    lo, hi = fs[max(i - 1, 0)], fs[min(i + 1, len(fs) - 1)]
    for _ in range(40):
        x1 = lo + (hi - lo) * 0.382
        x2 = lo + (hi - lo) * 0.618
        if lsq(t, y, x1)[0] > lsq(t, y, x2)[0]:
            hi = x2
        else:
            lo = x1
    return 0.5 * (lo + hi)


def lag(a, b):
    return (math.degrees(a - b) + 180) % 360 - 180


def shape(te):
    s = math.pi / 6
    te = np.mod(te, 2 * math.pi)
    return np.where(te < s, te / s, np.where(te < math.pi - s, 1.0, np.where(
        te < math.pi + s, (math.pi - te) / s, np.where(te < 2 * math.pi - s, -1.0,
                                                     (te - 2 * math.pi) / s))))


def derived(c, mot):
    pp = mot["poles"] / 2
    ke = 0.5 * 60.0 / (2 * math.pi * mot["kv"])
    te = c["theta"] * pp
    c["tau"] = ke * (shape(te) * c["ia"] + shape(te + 2 * math.pi / 3) * c["ib"]
                     + shape(te + 4 * math.pi / 3) * c["ic"])
    c["idc"] = (np.abs(c["ia"]) + np.abs(c["ib"]) + np.abs(c["ic"])) / 2
    c["isq"] = (c["ia"] ** 2 + c["ib"] ** 2 + c["ic"] ** 2) / 3


def u8delta(a):
    d = np.diff(a[~np.isnan(a)].astype(int))
    d[d < 0] += 256
    return int(d.sum())


def segments(c, phase):
    """contiguous runs of the programme step inside one test phase"""
    idx = np.where(c["phase"] == phase)[0]
    st = c[HP + "step"][idx]
    segs = []
    start = 0
    for i in range(1, len(idx) + 1):
        if i == len(idx) or st[i] != st[start]:
            segs.append((int(st[start]), idx[start], idx[i - 1]))
            start = i
    return segs


def step_table(c, phase):
    out = []
    for s, i0, i1 in segments(c, phase):
        sl = slice(i0, i1 + 1)
        t = c["t_s"][sl]
        tk = c[HP + "step_ticks"][sl]
        ok = c["running"][sl] == 1
        f_loop = float(np.polyfit(t[ok], tk[ok], 1)[0]) if ok.sum() > 10 and np.ptp(tk[ok]) > 0 else float("nan")
        out.append({"step": s, "i0": i0, "i1": i1, "t_first": float(t[0]), "t_last": float(t[-1]),
                    "ticks_first": float(tk[0]), "f_loop": f_loop})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="profile_v1")
    ap.add_argument("--plain", default="stock_pr438_plain")
    ap.add_argument("--outdir", default=os.path.join(HAKEA, "notes/sim/renode/v3"))
    a = ap.parse_args()
    meta = json.load(open(os.path.join(RAW, a.name + ".json")))
    mot = meta["model_json"]["motor"]
    c = load(os.path.join(RAW, a.name + "_coarse.csv"))
    f = load(os.path.join(RAW, a.name + "_fine.csv"))
    derived(c, mot)
    derived(f, mot)
    f["mod"] = (f["adjusted_duty_cycle"] - 1) * 2000.0 / f["tim1_arr"] - f["last_duty_cycle"]
    report = {}

    # ---- loop rate (all programme steps of phase A, ticks/s) ----
    stA = step_table(c, "A")
    floops = [s["f_loop"] for s in stA if not math.isnan(s["f_loop"])]
    f_loop = float(np.median(floops))
    report["f_loop_median"] = f_loop
    report["f_loop_per_step"] = [round(x, 2) for x in floops]

    # ---- gate opening (phase A) ----
    A = np.where(c["phase"] == "A")[0]
    run1 = A[np.argmax(c["running"][A] == 1)]
    tick_on = A[np.argmax((c[HP + "step_ticks"][A] > 0) & (c["running"][A] == 1))]
    t_gate = c["t_s"][tick_on] - c[HP + "step_ticks"][tick_on] / f_loop
    zc_at = np.interp(t_gate, c["t_s"][A], c["zero_crosses"][A])
    report["gate_A"] = {"t_running": float(c["t_s"][run1]), "t_gate_open": float(t_gate),
                        "zero_crosses_at_gate_interp": float(zc_at),
                        "zero_crosses_1ms_before": float(c["zero_crosses"][tick_on - 1]),
                        "zero_crosses_at_first_tick_sample": float(c["zero_crosses"][tick_on])}

    # ---- step timeline ----
    timeline = []
    for s in stA:
        t_start = s["t_first"] - s["ticks_first"] / f_loop if s["step"] < 15 else s["t_first"]
        timeline.append({"step": s["step"], "t_start": t_start})
    for i, row in enumerate(timeline):
        if i + 1 < len(timeline):
            row["duration_s"] = timeline[i + 1]["t_start"] - row["t_start"]
        if row["step"] < 15:
            fz, amp, ms = PROGRAMME[row["step"]]
            row.update({"freq_prog": fz, "amp_prog": amp, "ms_prog": ms,
                        "duration_expected_s": ms / 1000 * 20000 / f_loop})
    report["timeline_A"] = timeline
    timeline[0]["t_start"] = report["gate_A"]["t_gate_open"]
    timeline[0]["duration_s"] = timeline[1]["t_start"] - timeline[0]["t_start"]
    t0 = timeline[0]["t_start"]

    # ---- per-step metrics ----
    rows = []
    base_vals = None
    for k, s in enumerate(stA):
        st = s["step"]
        if st >= 15:
            continue
        fz, amp, ms = PROGRAMME[st]
        tst = timeline[k]["t_start"]
        tend = timeline[k + 1]["t_start"] if k + 1 < len(timeline) else s["t_last"]
        cs = (c["phase"] == "A") & (c["t_s"] > tst) & (c["t_s"] <= tend)
        takes = c["stress_take"][cs]
        takes = takes[~np.isnan(takes)]
        ev = c["status_take"][cs]
        ev = ev[~np.isnan(ev)].astype(int)
        orv = 0
        for e in ev:
            orv |= int(e)
        r = {"step": st, "freq_prog": fz, "amp_prog": amp,
             "t_start_s": round(tst, 5), "t_from_step0_s": round(tst - t0, 5),
             "duration_s": round(timeline[k].get("duration_s", float("nan")), 5),
             "duration_expected_s": round(timeline[k]["duration_expected_s"], 5),
             "f_loop_step": round(s["f_loop"], 2),
             "desyncs": u8delta(c["desync_happened"][cs]),
             "status_or": "0x%02X" % orv, "n_warn": int(sum(1 for e in ev if e & 0x40)),
             "n_alert": int(sum(1 for e in ev if e & 0x80)),
             "stress_med": float(np.median(takes)) if len(takes) else "",
             "stress_max": int(takes.max()) if len(takes) else "",
             "stress_n": len(takes),
             "rpm_mean_coarse": round(float(np.nanmean(c["rpm_true"][cs])), 1),
             "pcu_coarse_W": round(3 * float(np.nanmean(c["isq"][cs])) * R_PHASE, 2)}
        fs = (f["phase"] == "A") & (f[HP + "step"] == st)
        if fs.sum() > 100:
            t = f["t_s"][fs]
            mod = f["mod"][fs]
            fwm = f[HP + "modulation"][fs]
            if amp > 0:
                fm = refine(t, mod, fz * 20000 / f_loop if not math.isnan(f_loop) else fz)
                n = int((t[-1] - t[0]) * fm)
                w = t < t[0] + n / fm
                tw = t[w]
                ma, mph, _ = lsq(tw, mod[w], fm)
                fa, fph, _ = lsq(tw, fwm[w], fm)

                def fit(y):
                    return lsq(tw, y[fs][w], fm)
                ra, rph, _ = fit(f["rpm_true"])
                ia, iph, _ = fit(f["idc"])
                ba, bph, _ = fit(f["ibus_true"])
                ta, tph, _ = fit(f["tau"])
                r.update({"f_meas": round(fm, 3), "f_expected": round(fz * f_loop / 20000, 3),
                          "cycles": n, "mod_amp": round(ma, 2), "fw_modulation_amp": round(fa, 2),
                          "fw_mod_min": int(fwm[w].min()), "fw_mod_max": int(fwm[w].max()),
                          "recon_skew_frac": round(float(np.mean(np.abs(mod[w] - fwm[w]) > 5)), 3),
                          "rpm_amp": round(ra, 2), "rpm_pp_fit": round(2 * ra, 2),
                          "rpm_lag_deg": round(lag(mph, rph), 1),
                          "idc_amp_A": round(ia, 3), "idc_lag_deg": round(lag(mph, iph), 1),
                          "ibus_amp_A": round(ba, 3),
                          "tau_amp_mNm": round(1e3 * ta, 3), "tau_lag_deg": round(lag(mph, tph), 1)})
                sel = w
            else:
                sel = np.ones(fs.sum(), bool)
                r.update({"fw_mod_absmax": int(np.abs(f[HP + "modulation"][fs]).max())})
            r.update({
                "rpm_mean": round(float(f["rpm_true"][fs][sel].mean()), 1),
                "rpm_pp_raw": round(float(np.ptp(f["rpm_true"][fs][sel])), 1),
                "duty_base_mean": round(float(f["last_duty_cycle"][fs][sel].mean()), 1),
                "idc_mean_A": round(float(f["idc"][fs][sel].mean()), 2),
                "ibus_mean_A": round(float(f["ibus_true"][fs][sel].mean()), 2),
                "tau_mean_mNm": round(1e3 * float(f["tau"][fs][sel].mean()), 2),
                "iph_rms_A": round(math.sqrt(float(f["isq"][fs][sel].mean())), 3),
                "pcu_W": round(3 * float(f["isq"][fs][sel].mean()) * R_PHASE, 2),
                "fine_s": round(float(t[-1] - t[0]) if amp > 0 else float(np.ptp(f["t_s"][fs])), 3)})
            if st == 0:
                base_vals = r
                om = r["rpm_mean"] * 2 * math.pi / 60
                r["tau_load_check_mNm"] = round(1e3 * (mot["damping"] * om + mot["load_k_omega2"] * om * om), 2)
        rows.append(r)
    if base_vals:
        for r in rows:
            if "pcu_W" in r:
                r["pcu_vs_settle_W"] = round(r["pcu_W"] - base_vals["pcu_W"], 2)
                r["rpm_shift_vs_settle"] = round(r["rpm_mean"] - base_vals["rpm_mean"], 1)
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(os.path.join(a.outdir, "summary_v3.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    # ---- restarts (phases B and C) ----
    for ph in ("B", "C"):
        idx = np.where(c["phase"] == ph)[0]
        segsP = segments(c, ph)
        info = {"segments": [(s, round(float(c["t_s"][i0]), 4), round(float(c["t_s"][i1]), 4))
                             for s, i0, i1 in segsP]}
        armed = c["armed"][idx]
        info["armed_min"] = float(np.nanmin(armed))
        if ph == "C":
            dis = idx[np.argmax(armed == 0)] if (armed == 0).any() else None
            if dis is not None:
                info["t_disarmed"] = float(c["t_s"][dis])
                re = idx[(c["t_s"][idx] > c["t_s"][dis]) & (armed == 1)]
                info["t_rearmed"] = float(c["t_s"][re[0]]) if len(re) else None
            kk = c["iwdg_kicks"][idx]
            info["iwdg_kicks_dropped_to"] = float(np.nanmin(kk))
            info["tenkhzcounter_min"] = float(np.nanmin(c["tenkhzcounter"][idx]))
        tk = c[HP + "step_ticks"][idx]
        ok = (c["running"][idx] == 1) & (tk > 0) & (c[HP + "step"][idx] == 0)
        if ok.any():
            j = idx[np.argmax(ok)]
            tg = c["t_s"][j] - c[HP + "step_ticks"][j] / f_loop
            info["t_gate_open"] = float(tg)
            info["zero_crosses_at_gate_interp"] = float(np.interp(tg, c["t_s"][idx], c["zero_crosses"][idx]))
            s1 = idx[(c[HP + "step"][idx] == 1) & (c["t_s"][idx] > tg)]
            if len(s1):
                t1 = c["t_s"][s1[0]] - c[HP + "step_ticks"][s1[0]] / f_loop
                info["t_step1_start"] = float(t1)
                info["step0_duration_s"] = float(t1 - tg)
        report["restart_" + ph] = info
    # restart sine check (fine, phase B/C step 1)
    for ph in ("B", "C"):
        fs = (f["phase"] == ph) & (f[HP + "step"] == 1)
        if fs.sum() > 1000:
            t = f["t_s"][fs]
            fm = refine(t, f["mod"][fs], 250 * f_loop / 20000)
            report["restart_%s_step1_f_meas" % ph] = fm
            report["restart_%s_step1_mod_amp" % ph] = lsq(t, f["mod"][fs], fm)[0]

    # ---- timing: missed ticks, watchdog kicks, compare with stock ----
    def kick_gap_ms(cc, sel):
        k = cc["iwdg_kicks"][sel]
        t = cc["t_s"][sel]
        inc = np.where(np.diff(k) != 0)[0]
        gaps = np.diff(t[inc]) if len(inc) > 1 else np.array([np.nan])
        return float(np.nanmax(gaps) * 1e3), float(np.nanmean(np.diff(k)[np.diff(k) > 0]) / 1e-3) if (np.diff(k) > 0).any() else float("nan")

    def tick_rate_tenkhz(cc, sel):
        x = cc["tenkhzcounter"][sel]
        t = cc["t_s"][sel]
        d = np.diff(x)
        good = d > 0
        return float(d[good].sum() / np.diff(t)[good].sum())

    selA = (c["phase"] == "A") & (c["running"] == 1)
    report["timing_profile"] = {"iwdg_max_gap_ms_at_1ms_res": kick_gap_ms(c, selA)[0],
                                "iwdg_kicks_per_s": kick_gap_ms(c, selA)[1],
                                "tick_rate_tenkhzcounter": tick_rate_tenkhz(c, selA),
                                "tick_rate_step_ticks": f_loop}
    pp = os.path.join(RAW, a.plain + "_coarse.csv")
    if os.path.exists(pp):
        p = load(pp)
        derived(p, mot)
        sel = (p["running"] == 1) & (p["t_s"] > p["t_s"][np.argmax(p["running"] == 1)] + 1.0)
        report["timing_stock_pr438"] = {"iwdg_max_gap_ms_at_1ms_res": kick_gap_ms(p, sel)[0],
                                        "iwdg_kicks_per_s": kick_gap_ms(p, sel)[1],
                                        "tick_rate_tenkhzcounter": tick_rate_tenkhz(p, sel),
                                        "rpm_mean": float(p["rpm_true"][sel].mean()),
                                        "pcu_W": float(3 * p["isq"][sel].mean() * R_PHASE),
                                        "desyncs": u8delta(p["desync_happened"][sel])}
    json.dump(report, open(os.path.join(a.outdir, "profile_v3_report.json"), "w"), indent=1,
              default=float)
    print(json.dumps({k: v for k, v in report.items() if k != "timeline_A"}, indent=1, default=float))
    print("timeline:")
    for r in timeline:
        print(r)
    show = ["step", "freq_prog", "amp_prog", "t_from_step0_s", "duration_s", "f_meas", "f_expected",
            "mod_amp", "fw_modulation_amp", "fw_mod_min", "fw_mod_max", "recon_skew_frac", "rpm_mean", "rpm_amp", "rpm_pp_fit",
            "idc_amp_A", "ibus_amp_A", "tau_amp_mNm", "tau_lag_deg", "iph_rms_A", "pcu_W",
            "pcu_vs_settle_W", "desyncs", "status_or", "stress_med", "stress_max"]
    print("| " + " | ".join(show) + " |")
    print("|" + "---|" * len(show))
    for r in rows:
        print("| " + " | ".join(str(r.get(k, "")) for k in show) + " |")


if __name__ == "__main__":
    main()
