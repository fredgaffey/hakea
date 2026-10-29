#!/usr/bin/env python3
"""In-ESC throttle sine (study-only SIM_PROFILE firmware) on the AM32 SITL,
0-1000 Hz. Predictions for Fred's future PROFILE_MODE tests.

Firmware: am32-sim-pr438 (bdbc75e + study patch, -DSIM_PROFILE,
DEAD_TIME 47). The sine is added to the duty setpoint every control tick
(19,608 Hz, the real F421 loop rate) BEFORE the ramp limiter, so the ramp
setting applies to collective + sine, as it would to a profile generated
in the loop. The collective comes in over DShot (1200 frames/s,
unidirectional), like ArduPilot.

Per run: arm 3 s, ramp to base 1 s, settle 1 s, sine on for SKIP + WIN.
Quantities (fundamentals fitted at f against the firmware's own
modulation signal `mod` as the phase reference; lags positive = behind):
  cmd_amp        commanded modulation amplitude (duty units, 0..2000)
  duty_amp, duty_lag_deg   delivered duty after the ramp limiter
  tau_amp, tau_lag_deg     electromagnetic torque (N m)
  idc_amp        equivalent DC current swing = tau_amp / Kt (A)
  ibus_amp       battery current fundamental (A)
  rpm_amp, rpm_lag_deg, rpm_mean
  irms, pcu      phase current rms (A), copper loss (W, R at 25 C)
  desync, slip   firmware desync increments; ground-truth commutation slip

usage: profile_grid.py --variants 16,255 --freqs ... --amps 5,10,20 [--ee pwm_frequency=48]
                       [--set commutation_transfer=0.8] --out FILE.csv
"""
import argparse
import csv
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grid  # noqa: E402
import sitl_run as sr  # noqa: E402

HAKEA = os.path.expanduser('~/vehicles/hakea')
# variants:
#   fred   Fred's PROFILE_MODE v1 code (am32-sim-profile, -DHAKEA_PROFILE
#          -DSIM_STUDY_PROGRAMME): sine added AFTER the ramp limiter, duty
#          clamped to [180, max]; programme replaced per run via
#          AM32_SITL_PROGRAMME. Frequencies come out x 19608/20000 (the code
#          computes phase steps for 20 kHz; the loop runs at 19,608 Hz)
#   pre16  my study SIM_PROFILE build (am32-sim-pr438): sine added BEFORE the
#          stock ramp limiter (16/tick), exact frequency - a comparison of
#          what the stock ramp would do to an in-loop profile
#   pre255 the same with the ramp opened to 255
ELFS = {
    'fred': os.path.join(HAKEA, 'am32-sim-profile/variants/sitl_profile_study.elf'),
    'pre16': os.path.join(HAKEA, 'am32-sim-pr438/variants/sitl_pr438_prof_ramp16.elf'),
    'pre255': os.path.join(HAKEA, 'am32-sim-pr438/variants/sitl_pr438_prof_ramp255.elf'),
}
LOOP_HZ = 1e6 / 51.0     # real AM32 control loop rate (TMR pr = 50 at 1 MHz)
FREQS = [1, 2, 5, 10, 20, 50, 100, 150, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000]
BASE_DS = 1048          # ArduPilot 1500 us
DUTY_PER_PCT = 20.0     # +-1 % throttle = 20 duty units (0..2000 scale)


def window(f):
    skip = max(0.1, 1.0 / f)
    win = max(0.5, 4.0 / f)
    return skip, win


def fit(t, y, f):
    A = np.column_stack([np.ones_like(t), np.sin(2 * np.pi * f * t), np.cos(2 * np.pi * f * t)])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    return c[0], complex(c[1], c[2])


def lag(h_ref, h):
    """phase lag of h behind h_ref in degrees, wrapped to [-90, 270)"""
    d = (np.degrees(np.angle(h_ref) - np.angle(h))) % 360.0
    return float(d - 360.0 if d >= 270.0 else d)


def run_one(args):
    variant, f, a_pct, model, ee, kt, tag, extra = args
    skip, win = window(f)
    amp = int(round(a_pct * DUTY_PER_PCT))
    st = sr.Stim(period_ns=833_333)
    st.const(0, 3.0)
    st.ramp(48, BASE_DS, 1.0)
    env = {}
    if variant == 'fred':
        # gate opens ~0.1 s into the ramp; 1.5 s settle at base, then the
        # sine; the run is long enough for settle + sine regardless
        f_act = f * LOOP_HZ / 20000.0
        sine_ms = int(round((skip + win + 0.1) * 1000))
        env['AM32_SITL_PROGRAMME'] = '0:0:1500,%d:%d:%d,0:0:60000' % (int(f), amp, sine_ms)
        st.const(BASE_DS, 1.0 + 1.6 + skip + win + 0.3)
    else:
        f_act = f
        t_on = 3.0 + 1.0 + 1.0
        if amp > 0:
            env['AM32_SITL_PROFILE'] = '%d,%.6f,%.6f,%.6f' % (amp, f, t_on, t_on + skip + win)
        st.const(BASE_DS, 1.0 + skip + win + 0.05)
    dt_ns = 20_000 if f >= 50 else 100_000
    lg = sr.run(st, model, elf=ELFS[variant], eeprom_overrides=ee, dt_ns=dt_ns, extra_env=env)
    t = lg['t']
    arr = np.maximum(lg['arr'], 1)
    deliv = (lg['adj'] - 1) * 2000.0 / arr   # duty actually written to the timer
    row = dict(tag=tag, variant=variant, f=f, f_actual=f_act, a_pct=a_pct, amp_cmd=amp, **extra)
    # sine segment start: first tick with non-zero modulation (the study
    # copy of Fred's code exports it as sim_profile_out -> `mod`)
    if variant == 'fred':
        if amp > 0:
            k = np.where((t > 4.0) & (lg['mod'] != 0))[0]
            if len(k) == 0:
                row['error'] = 'sine never started: ' + str(lg['_end'])
                return row
            t_on = t[k[0]] - (t[1] - t[0])
        else:
            t_on = 5.5
    m = (t > t_on + skip) & (t <= t_on + skip + win)
    if m.sum() < 20:
        row['error'] = 'no data: ' + str(lg['_end'])
        return row
    tt = t[m] - t_on
    fr = f_act
    md, hd = fit(tt, deliv[m], fr)
    mb_, hbase = fit(tt, lg['duty'][m].astype(float), fr)
    mt, ht = fit(tt, lg['tau'][m], fr)
    mr, hr = fit(tt, lg['rpm'][m], fr)
    mb, hb = fit(tt, lg['ibus_inst'][m], fr)
    _, ref = fit(tt, lg['mod'][m].astype(float), fr)
    cmd_amp = abs(ref)
    if amp == 0:
        ref = complex(1, 0)
    row.update(cmd_amp=cmd_amp, duty_mean=md, duty_amp=abs(hd), duty_lag_deg=lag(ref, hd) if amp else 0.0,
               duty_ratio=abs(hd) / cmd_amp if amp else 0.0,
               duty_min=float(deliv[m].min()), duty_max=float(deliv[m].max()),
               tau_mean=mt, tau_amp=abs(ht), tau_lag_deg=lag(ref, ht) if amp else 0.0,
               idc_amp=abs(ht) / kt, ibus_mean=float(lg['ibus'][m].mean()), ibus_amp=abs(hb),
               rpm_mean=mr, rpm_amp=abs(hr), rpm_lag_deg=lag(ref, hr) if amp else 0.0,
               rpm_pp=float(np.percentile(lg['rpm'][m], 99.5) - np.percentile(lg['rpm'][m], 0.5)),
               irms=float(np.sqrt(np.mean(lg['irms'][m] ** 2))), pcu=float(lg['pcu'][m].mean()),
               pbr=float(lg['pbr'][m].mean()), vbus_mean=float(lg['vbus'][m].mean()))
    row['desync'] = int(lg['desync'][m].max() - lg['desync'][m].min())
    rel = (lg['thetae'][m] - 60.0 * lg['step'][m]) % 360.0
    med = np.angle(np.mean(np.exp(1j * np.radians(rel))), deg=True) % 360
    dev = (rel - med + 180) % 360 - 180
    row['slip'] = float(np.mean(np.abs(dev) > 60))
    row['end'] = (lg['_end'] or '').split(',')[1] if lg['_end'] else 'none'
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--variants', default='fred,pre16')
    ap.add_argument('--freqs', default=','.join(str(f) for f in FREQS))
    ap.add_argument('--amps', default='0,5,10,20')
    ap.add_argument('--vbat', type=float, default=24.0)
    ap.add_argument('--set', action='append', default=[])
    ap.add_argument('--ee', action='append', default=[])
    ap.add_argument('--tag', default='')
    ap.add_argument('--workers', type=int, default=14)
    ap.add_argument('--out', required=True)
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
    extra = dict(pwm_khz=ee.get('pwm_frequency', 24), ct=base.get('commutation_transfer', 1.0))
    jobs = []
    freqs = [float(x) for x in a.freqs.split(',')]
    for v in a.variants.split(','):
        for amp in [float(x) for x in a.amps.split(',')]:
            for f in (freqs if amp > 0 else [250.0]):
                jobs.append((v, f, amp, model, ee, kt, a.tag, extra))
    rows = []
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        for r in ex.map(run_one, jobs):
            rows.append(r)
            print(json.dumps({k: (round(x, 4) if isinstance(x, float) else x) for k, x in r.items()}))
            sys.stdout.flush()
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(a.out, 'w', newline='') as fo:
        w = csv.DictWriter(fo, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    json.dump(dict(base=base, model=model, vbat=a.vbat, ee=ee, elf={v: ELFS[v] for v in a.variants.split(',')},
                   base_dshot=BASE_DS), open(a.out + '.meta.json', 'w'), indent=1)


if __name__ == '__main__':
    main()
