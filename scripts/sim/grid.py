#!/usr/bin/env python3
"""1P throttle-sine grid on the AM32 SITL (the ramp-limit study).

For each firmware variant (ramp limit), frequency f and amplitude A:
  arm (3 s zero), soft ramp to the base throttle (1 s), settle (1 s),
  sine base +- A at f for SINE_S seconds, hold base 0.3 s.
DShot frames are injected by the SITL itself at 10 kHz of simulated time
(unidirectional DShot300), so the stimulus is exact and host-independent.

Measured over the sine window (after SKIP_S):
  desync      firmware desync_happened increments
  slip        ground truth: fraction of samples where the firmware's
              commutation step disagrees with the rotor electrical angle by
              more than 60 deg from its median relation (0 = in sync)
  rpm_mean, rpm_amp1 (fundamental at f), rpm_pp
  duty_cmd_amp1  commanded duty fundamental (what the throttle asks for)
  duty_amp1   applied duty fundamental (after the ramp limiter), duty_pp
  tau_amp1    electromagnetic torque fundamental (Nm), tau_mean
  ibus_mean, irms (phase current rms), pcu (copper W), pbr (bridge W)

usage:
  grid.py --variants 16,32,64,255 --freqs 50,100,150,200,250,300,400 --amps 5,10,20,30,40,50 --out FILE.csv
  grid.py ... --ee comp_pwm=0 --ee pwm_frequency=48   (ESC setting overrides)
"""
import argparse
import csv
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sitl_run as sr  # noqa: E402

HAKEA = os.path.expanduser('~/vehicles/hakea')
VARIANTS = os.path.join(HAKEA, 'am32-sim/variants')
BASE_JSON = os.path.join(HAKEA, 'notes/sim/sitl_calib/sitl_base_final.json')
SINE_S = 1.0
SKIP_S = 0.2
FRAME_NS = 100_000


def dshot_amp(a_pct):
    return a_pct / 100.0 * 2000.0


def fund(t, y, f):
    A = np.column_stack([np.ones_like(t), np.sin(2 * np.pi * f * t), np.cos(2 * np.pi * f * t)])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    return float(np.hypot(c[1], c[2])), float(c[0])


def one(args):
    variant, f, a_pct, base_ds, model, ee, tag, elf_pattern = args
    st = sr.Stim(period_ns=FRAME_NS)
    st.const(0, 3.0)
    st.ramp(48, base_ds, 1.0)
    st.const(base_ds, 1.0)
    t_s0 = st.t
    amp = dshot_amp(a_pct)
    if a_pct > 0:
        st.sine(base_ds, amp, f, SINE_S)
    else:
        st.const(base_ds, SINE_S)
    t_s1 = st.t
    st.const(base_ds, 0.3)
    elf = elf_pattern % variant
    lg = sr.run(st, model, elf=elf, eeprom_overrides=ee, dt_ns=50_000)
    t = lg['t']
    m = (t > t_s0 + SKIP_S) & (t <= t_s1)
    row = dict(tag=tag, variant=variant, f=f, a_pct=a_pct, base_dshot=base_ds, end=(lg['_end'] or '').split(',')[1] if lg['_end'] else 'none')
    if m.sum() < 10:
        row['error'] = 'no data'
        return row
    tt = t[m] - t_s0
    # commanded duty: AM32 map of the commanded DShot value (no ramp limit)
    ds_cmd = np.clip(base_ds + (amp * np.sin(2 * np.pi * f * tt) if a_pct > 0 else 0 * tt), 48, 2047)
    duty_cmd = 40 + (ds_cmd - 47) * 1960 / 2000
    fr = f if a_pct > 0 else 100.0
    row['desync'] = int(lg['desync'][m].max() - lg['desync'][m].min())
    row['desync_total'] = int(lg['desync'][t > 4.0].max() - lg['desync'][t > 4.0].min()) if (t > 4.0).any() else -1
    # ground truth sync check: electrical angle relative to commutation step
    rel = (lg['thetae'][m] - 60.0 * lg['step'][m]) % 360.0
    med = np.angle(np.mean(np.exp(1j * np.radians(rel))), deg=True) % 360
    dev = (rel - med + 180) % 360 - 180
    row['slip'] = float(np.mean(np.abs(dev) > 60))
    row['angle_spread_deg'] = float(np.std(dev))
    row['running_min'] = int(lg['running'][m].min())
    row['rpm_mean'] = float(lg['rpm'][m].mean())
    row['rpm_amp1'], _ = fund(tt, lg['rpm'][m], fr)
    row['rpm_pp'] = float(np.percentile(lg['rpm'][m], 99.5) - np.percentile(lg['rpm'][m], 0.5))
    row['duty_cmd_amp1'], _ = fund(tt, duty_cmd, fr)
    row['duty_amp1'], row['duty_mean'] = fund(tt, lg['duty'][m].astype(float), fr)
    row['duty_pp'] = float(lg['duty'][m].max() - lg['duty'][m].min())
    row['tau_amp1'], row['tau_mean'] = fund(tt, lg['tau'][m], fr)
    row['ibus_mean'] = float(lg['ibus'][m].mean())
    row['ibus_min'] = float(lg['ibus'][m].min())
    row['irms'] = float(np.sqrt(np.mean(lg['irms'][m] ** 2)))
    row['pcu'] = float(lg['pcu'][m].mean())
    row['pbr'] = float(lg['pbr'][m].mean())
    row['vbus_mean'] = float(lg['vbus'][m].mean())
    row['vbus_min'] = float(lg['vbus'][m].min())
    return row


def sitl_model(base, vbat):
    mp = dict(kv=base['kv'], R_phase=base['R_phase'], L_phase=base['L_phase'], J=base['J_m'] + base['J_p'],
              kq=base['kq'], T0=base['T0'], b=base['b'], Vbat=vbat, Rb=base['Rb'], rds_on=base['rds_on'],
              commutation_transfer=base.get('commutation_transfer', 1.0))
    return sr.model_from_params(mp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--variants', default='16,32,64,255')
    ap.add_argument('--freqs', default='50,100,150,200,250,300,400')
    ap.add_argument('--amps', default='0,5,10,20,30,40,50')
    ap.add_argument('--base-dshot', type=int, default=1048, help='1048 = ArduPilot 1500 us')
    ap.add_argument('--vbat', type=float, default=24.3)
    ap.add_argument('--base', default=BASE_JSON, help='calibrated SITL parameter json')
    ap.add_argument('--set', action='append', default=[], help='override a SITL model parameter, e.g. L_phase=2e-5')
    ap.add_argument('--ee', action='append', default=[], help='eeprom override name=value')
    ap.add_argument('--tag', default='')
    ap.add_argument('--elf-pattern', default=os.path.join(VARIANTS, 'sitl_ramp%s.elf'), help='%%s = variant')
    ap.add_argument('--workers', type=int, default=14)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    base = json.load(open(a.base))
    for s in a.set:
        k, v = s.split('=')
        base[k] = float(v)
    model = sitl_model(base, a.vbat)
    ee = {}
    for s in a.ee:
        k, v = s.split('=')
        ee[k] = int(v)
    jobs = []
    for v in a.variants.split(','):
        for f in [float(x) for x in a.freqs.split(',')]:
            for amp in [float(x) for x in a.amps.split(',')]:
                if amp == 0 and f != float(a.freqs.split(',')[0]):
                    continue  # one baseline per variant
                jobs.append((v, f, amp, a.base_dshot, model, ee, a.tag, a.elf_pattern))
    rows = []
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        for row in ex.map(one, jobs):
            rows.append(row)
            print(json.dumps({k: (round(x, 3) if isinstance(x, float) else x) for k, x in row.items()}))
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
    meta = dict(base=base, vbat=a.vbat, ee=ee, base_dshot=a.base_dshot, model=model, frame_ns=FRAME_NS,
                sine_s=SINE_S, skip_s=SKIP_S, elfs={v: a.elf_pattern % v for v in a.variants.split(',')})
    json.dump(meta, open(a.out + '.meta.json', 'w'), indent=1)


if __name__ == '__main__':
    main()
