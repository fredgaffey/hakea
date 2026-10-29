#!/usr/bin/env python3
"""Run Fred's exact PROFILE_MODE v1 programme (Src/hakea_profile.c, built
as SITL with -DHAKEA_PROFILE, no programme override) and measure every
step. Also checks that the programme restarts after the gate drops
(zero throttle, motor stops, back to base).

Sequence (DShot300 unidirectional at 1200 frames/s, like ArduPilot):
  3 s zero (arm), 1 s ramp to base 1048 (= 1500 us), hold base for the
  whole programme (~31.6 s at the real 19,608 Hz loop rate) + 2 s,
  zero for 2 s (stop; gate drops), ramp back to base, hold 6 s
  (programme restarts: settle 3 s + first sine step).

Step boundaries: step k lasts time_ms * 20 ticks of 51 us (= x1.02).
The programme clock starts when the gate opens, found as the first
sample where the delivered duty departs from the base by > amp/3 in step
1, minus the known step-0 length.

usage: program_run.py --out FILE.csv [--set commutation_transfer=0.8] [--ee pwm_frequency=48]
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grid  # noqa: E402
import sitl_run as sr  # noqa: E402
from profile_grid import fit, lag  # noqa: E402

HAKEA = os.path.expanduser('~/vehicles/hakea')
ELF = os.path.join(HAKEA, 'am32-sim-profile/variants/sitl_profile_fred.elf')
# Fred's v1 programme (Src/hakea_profile.c): (freq_hz, amp, time_ms)
PROGRAMME = [(0, 0, 3000), (250, 40, 3000), (0, 0, 1000), (250, 100, 3000), (0, 0, 1000),
             (400, 100, 3000), (0, 0, 1000), (600, 100, 3000), (0, 0, 1000), (600, 200, 3000),
             (0, 0, 1000), (800, 100, 3000), (0, 0, 1000), (1000, 100, 3000), (0, 0, 1000)]
TICK_SCALE = 51.0 / 50.0     # ms per programme ms (loop 19,608 Hz, code assumes 20 kHz)
BASE_DS = 1048


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--vbat', type=float, default=24.0)
    ap.add_argument('--set', action='append', default=[])
    ap.add_argument('--ee', action='append', default=[])
    ap.add_argument('--elf', default=ELF)
    a = ap.parse_args()
    base = json.load(open(grid.BASE_JSON))
    for s in a.set:
        k, v = s.split('=')
        base[k] = float(v)
    model = grid.sitl_model(base, a.vbat)
    kt = 60.0 / (2 * np.pi * base['kv'])
    ee = {}
    for s in a.ee:
        k, v = s.split('=')
        ee[k] = int(v)
    prog_s = sum(p[2] for p in PROGRAMME) / 1000.0 * TICK_SCALE
    st = sr.Stim(period_ns=833_333)
    st.const(0, 3.0)
    st.ramp(48, BASE_DS, 1.0)
    st.const(BASE_DS, prog_s + 2.0)
    t_stop = st.t
    st.const(0, 2.0)
    t_restart = st.t
    st.ramp(48, BASE_DS, 1.0)
    st.const(BASE_DS, 6.0)
    lg = sr.run(st, model, elf=a.elf, eeprom_overrides=ee, dt_ns=20_000)
    t = lg['t']
    arr = np.maximum(lg['arr'], 1)
    deliv = (lg['adj'] - 1) * 2000.0 / arr
    dev = deliv - lg['duty']
    # find the sine steps from the data: rolling rms of (delivered - base)
    # over 20 ms; active where > 10 duty units (step 1 is +-40 -> rms 28)
    steady = (t > 4.5) & (t < t_stop) & (lg['rpm'] > 10000)
    base_duty = float(np.median(deliv[steady]))
    x = np.where(steady, deliv - base_duty, 0.0)
    nwin = max(1, int(round(0.02 / (t[1] - t[0]))))
    rms = np.sqrt(np.convolve(x ** 2, np.ones(nwin) / nwin, mode='same'))
    act = rms > 10
    edges = np.flatnonzero(np.diff(act.astype(int)))
    segs = []
    start = None
    for e in edges:
        if act[e + 1]:
            start = e + 1
        elif start is not None:
            if t[e] - t[start] > 1.0:
                segs.append((t[start], t[e]))
            start = None
    sine_steps = [i for i, p in enumerate(PROGRAMME) if p[0] > 0]
    if len(segs) < len(sine_steps):
        print('found only %d sine segments' % len(segs), segs)
    bounds = {}
    for i, sg in zip(sine_steps, segs):
        bounds[i] = sg
    # rest / settle steps: between the neighbouring sine segments
    t_gate = (segs[0][0] - PROGRAMME[0][2] / 1000.0 * TICK_SCALE) if segs else float('nan')
    for i, p in enumerate(PROGRAMME):
        if i in bounds:
            continue
        prev_end = bounds[i - 1][1] if (i - 1) in bounds else t_gate
        nxt = bounds.get(i + 1, (prev_end + p[2] / 1000.0 * TICK_SCALE,))[0]
        bounds[i] = (prev_end, nxt)
    rows = []
    for i, (fq, amp, ms) in enumerate(PROGRAMME):
        t0, t1 = bounds[i]
        m = (t > t0 + 0.3) & (t < t1 - 0.05)
        row = dict(step=i, freq_nominal=fq, freq_actual=fq / TICK_SCALE, amp=amp, t_start=t0, t_end=t1,
                   duration=t1 - t0, duration_expected=ms / 1000.0 * TICK_SCALE)
        if m.sum() < 50:
            row['error'] = 'no data'
            rows.append(row)
            continue
        tt = t[m] - t0
        row['desync'] = int(lg['desync'][m].max() - lg['desync'][m].min())
        rel = (lg['thetae'][m] - 60.0 * lg['step'][m]) % 360.0
        med = np.angle(np.mean(np.exp(1j * np.radians(rel))), deg=True) % 360
        d = (rel - med + 180) % 360 - 180
        row['slip'] = float(np.mean(np.abs(d) > 60))
        row.update(rpm_mean=float(lg['rpm'][m].mean()), irms=float(np.sqrt(np.mean(lg['irms'][m] ** 2))),
                   pcu=float(lg['pcu'][m].mean()), ibus_mean=float(lg['ibus'][m].mean()),
                   duty_base=float(lg['duty'][m].mean()), duty_min=float(deliv[m].min()), duty_max=float(deliv[m].max()),
                   rpm_pp=float(np.percentile(lg['rpm'][m], 99.5) - np.percentile(lg['rpm'][m], 0.5)))
        if fq > 0:
            f = fq / TICK_SCALE
            _, hd = fit(tt, deliv[m], f)
            _, ht = fit(tt, lg['tau'][m], f)
            _, hr = fit(tt, lg['rpm'][m], f)
            _, hb = fit(tt, lg['ibus_inst'][m], f)
            xd = deliv[m] - base_duty
            zc = np.where((xd[:-1] < 0) & (xd[1:] >= 0))[0]
            f_meas = (len(zc) - 1) / (tt[zc[-1]] - tt[zc[0]]) if len(zc) > 2 else float('nan')
            row.update(f_measured=f_meas, duty_amp=abs(hd), tau_amp=abs(ht), tau_lag_deg=lag(hd, ht),
                       idc_amp=abs(ht) / kt, ibus_amp=abs(hb), rpm_amp=abs(hr), rpm_lag_deg=lag(hd, hr))
        rows.append(row)
    # restart check: after zero throttle and re-spin, the first sine must
    # appear ~settle (3.06 s) after the gate reopens
    xr = np.where((t > t_restart + 1.0) & (lg['rpm'] > 10000), deliv - base_duty, 0.0)
    rr = np.sqrt(np.convolve(xr ** 2, np.ones(nwin) / nwin, mode='same'))
    mr = (t > t_restart + 1.0) & (rr > 10)
    restart = dict(stopped_rpm=float(lg['rpm'][(t > t_restart - 0.2) & (t < t_restart)].mean()),
                   restart_first_modulation_s=float(t[mr][0] - t_restart) if mr.any() else None,
                   expected_s='~1.1 (ramp, gate) + 3.06 (settle) = ~4.2')
    keys = []
    for r in rows:
        for kk in r:
            if kk not in keys:
                keys.append(kk)
    with open(a.out, 'w', newline='') as fo:
        w = csv.DictWriter(fo, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    meta = dict(elf=a.elf, model=model, vbat=a.vbat, ee=ee, t_gate=t_gate, restart=restart,
                end=lg['_end'], desync_total_after_spinup=int(lg['desync'][t > 5].max() - lg['desync'][t > 5].min()))
    json.dump(meta, open(a.out + '.meta.json', 'w'), indent=1)
    for r in rows:
        print(json.dumps({kk: (round(v, 4) if isinstance(v, float) else v) for kk, v in r.items()}))
    print(json.dumps(meta['restart']), 't_gate', round(t_gate, 3), meta['end'])


if __name__ == '__main__':
    main()
