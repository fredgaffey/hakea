#!/usr/bin/env python3
"""Summarise Renode cross-check runs.

Reads every <name>.csv + <name>.json pair in the raw directory written by
run_crosscheck.py and, for each run and segment, computes:

  rpm_mean          mean of the physics' true mechanical RPM
  rpm_pp_250        peak-to-peak of the 250 Hz component of RPM: a least
                    squares fit  rpm = c0 + c1*t + A sin(wt) + B cos(wt)
                    over an integer number of 250 Hz cycles, pp = 2*sqrt(A^2+B^2)
  rpm_pp_raw        max - min of RPM over the same window
  rpm_lag_deg       phase lag of the RPM 250 Hz component behind the
                    commanded DShot sine (degrees)
  cmd_pp_250        same fit applied to the commanded DShot value (sanity)
  duty_pp_250       same fit on the firmware's committed duty, last_duty_cycle
                    (0..2000 scale, after the ramp limiter) - the duty swing
                    actually applied.  NOT duty_cycle: tenKhzRoutine() first
                    writes the unlimited setpoint into duty_cycle and clamps
                    it a few instructions later, and a sample taken while the
                    emulation is paused between the two catches the raw
                    setpoint (seen as single-sample spikes in the CSV).
  duty_min/max      raw range of last_duty_cycle in the window
  duty_slew_p99     99th percentile of |change of last_duty_cycle| per second
                    between 100 us samples (shows the ramp limiter's
                    ceiling: 16/tick * 20 kHz = 320000/s).  A 100 us sample
                    holds 2 control ticks nominally, sometimes 3, so the max
                    is noisier than the p99.
  desyncs           increase of the firmware's desync_happened counter
                    during the segment
  bemf_timeouts     max of bemf_timeout_happened during the segment
  dshot_rate_hz     frames decoded per second (dshot_goodcounts delta)
  dshot_bad         dshot_badcounts increase

The analysis window skips the first 0.1 s of each sine segment (25
cycles) so the start-up transient of the sine does not bias the fit.
Writes summary.csv next to the raw directory and prints a markdown table.
"""
import argparse
import csv
import glob
import json
import math
import os

import numpy as np

HAKEA = os.path.expanduser("~/vehicles/hakea")


def fit250(t, y, f=250.0):
    w = 2 * math.pi * f
    X = np.column_stack([np.ones_like(t), t - t.mean(), np.sin(w * t), np.cos(w * t)])
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    amp = math.hypot(c[2], c[3])
    phase = math.degrees(math.atan2(c[3], c[2]))  # y ~ amp*sin(wt + phase)
    return 2 * amp, phase


def counter_delta(a, bits):
    d = np.diff(a.astype(np.int64))
    d[d < 0] += 1 << bits
    return int(d.sum())


def analyse(csv_path, meta):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    cols = {k: np.array([float(r[k]) if r[k] != '' else np.nan for r in rows])
            for k in rows[0].keys()}
    step_s = meta["step_us"] * 1e-6
    out = []
    for seg in meta["segments"]:
        s0, s1 = seg["start_step"], seg["end_step"]
        name = seg["name"]
        if name.startswith("sine"):
            s0 += int(round(0.1 / step_s))
            ncyc = int((s1 - s0) * step_s * 250.0)
            s1 = s0 + int(round(ncyc / 250.0 / step_s))
        else:
            ncyc = 0
        sl = slice(s0, s1)
        t = cols["t_s"][sl]
        rpm = cols["rpm_true"][sl]
        duty = cols["last_duty_cycle"][sl]
        cmd = cols["cmd"][sl]
        r = {"run": meta["name"], "target": meta["target"], "segment": name,
             "t_start": round(float(t[0]), 4), "t_end": round(float(t[-1]), 4),
             "cycles": ncyc,
             "max_ramp_high_rpm": int(np.nanmax(cols["max_ramp_high_rpm"][sl])),
             "rpm_mean": round(float(rpm.mean()), 1),
             "rpm_pp_raw": round(float(rpm.max() - rpm.min()), 1),
             "duty_mean": round(float(duty.mean()), 1),
             "duty_min": int(duty.min()), "duty_max": int(duty.max()),
             "duty_slew_p99_per_s": int(np.percentile(np.abs(np.diff(duty)), 99) / step_s) if len(duty) > 1 else 0,
             "duty_slew_max_per_s": int(np.abs(np.diff(duty)).max() / step_s) if len(duty) > 1 else 0,
             }
        if ncyc:
            rpp, rph = fit250(t, rpm)
            cpp, cph = fit250(t, cmd)
            dpp, dph = fit250(t, duty)
            lag = (cph - rph) % 360.0
            dlag = (cph - dph) % 360.0
            r.update({"rpm_pp_250": round(rpp, 1), "rpm_lag_deg": round(lag, 1),
                      "cmd_pp_250": round(cpp, 1), "duty_pp_250": round(dpp, 1),
                      "duty_lag_deg": round(dlag, 1)})
        else:
            r.update({"rpm_pp_250": "", "rpm_lag_deg": "", "cmd_pp_250": "",
                      "duty_pp_250": "", "duty_lag_deg": ""})
        # counters over the whole (unshortened) segment
        full = slice(seg["start_step"], seg["end_step"])
        bits_desync = 32 if meta["target"].endswith("_CAN") else 8
        r["desyncs"] = counter_delta(
            np.concatenate([[cols["desync_happened"][max(seg["start_step"] - 1, 0)]],
                            cols["desync_happened"][full]]), bits_desync)
        r["bemf_timeouts_max"] = int(np.nanmax(cols["bemf_timeout_happened"][full]))
        r["running_min"] = int(np.nanmin(cols["running"][full]))
        good = cols["dshot_goodcounts"][full]
        r["dshot_rate_hz"] = round(counter_delta(good, 16) / (len(good) * step_s), 0)
        r["dshot_bad"] = counter_delta(cols["dshot_badcounts"][full], 16)
        r["current_mean_cA"] = round(float(np.nanmean(cols["actual_current"][full])), 0)
        out.append(r)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default=os.path.join(HAKEA, "notes/sim/renode/raw"))
    ap.add_argument("--out", default=os.path.join(HAKEA, "notes/sim/renode/summary.csv"))
    args = ap.parse_args()
    allrows = []
    for meta_path in sorted(glob.glob(os.path.join(args.raw, "*.json"))):
        meta = json.load(open(meta_path))
        csv_path = meta_path[:-5] + ".csv"
        if os.path.exists(csv_path):
            allrows += analyse(csv_path, meta)
    if not allrows:
        print("no runs found in", args.raw)
        return
    keys = list(allrows[0].keys())
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(allrows)
    show = ["run", "segment", "max_ramp_high_rpm", "rpm_mean", "rpm_pp_250", "rpm_lag_deg",
            "duty_mean", "duty_pp_250", "duty_min", "duty_max", "duty_slew_p99_per_s",
            "desyncs", "bemf_timeouts_max", "running_min", "dshot_rate_hz", "dshot_bad"]
    print("| " + " | ".join(show) + " |")
    print("|" + "---|" * len(show))
    for r in allrows:
        if r["segment"] in ("settle", "sine200", "sine600", "zero"):
            print("| " + " | ".join(str(r[k]) for k in show) + " |")
    print("wrote", args.out)


if __name__ == "__main__":
    main()
