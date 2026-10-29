#!/usr/bin/env python3
"""Averaged-model and analytic predictions for the in-ESC throttle sine
(Hakea PROFILE_MODE v1: sine added after the ramp limiter, duty clamped to
[180, 2000]), 0-1000 Hz, at base 1500 us, 24 V.

  analytic  linearised averaged motor (R, L, Ke, J, prop damping, battery)
            around the operating point: torque/current/rpm per unit duty,
            amplitude and phase vs the delivered duty
  avg       nonlinear averaged simulation (avgmodel.py) with the modulation
            added per 20 kHz tick after the ramp limiter

Frequencies are the ACTUAL ones produced by Fred's v1 code (nominal x
19608/20000), so they line up with the SITL and the hardware.

usage: profile_avg.py --params P.json --out FILE.csv [--L 36e-6] [--R 0.158]
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import avgmodel as am  # noqa: E402
import calibrate as cal  # noqa: E402

FREQS = [1, 2, 5, 10, 20, 50, 100, 150, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000]
AMPS = [5, 10, 20, 30]
LOOP_HZ = 1e6 / 51.0
VBAT = 24.0
BASE_PWM = 1500


def fit(t, y, f):
    A = np.column_stack([np.ones_like(t), np.sin(2 * np.pi * f * t), np.cos(2 * np.pi * f * t)])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    return c[0], complex(c[1], c[2])


def lagdeg(href, h):
    d = np.degrees(np.angle(href) - np.angle(h)) % 360.0
    return float(d - 360.0 if d >= 270.0 else d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--params', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--L', type=float, default=36e-6, help='line-line inductance (2 x 18 uH phase prior)')
    ap.add_argument('--R', type=float, default=None, help='override loop resistance (ohm)')
    ap.add_argument('--freqs', default=','.join(str(f) for f in FREQS))
    ap.add_argument('--amps', default=','.join(str(a) for a in AMPS))
    a = ap.parse_args()
    p = json.load(open(a.params))
    q = cal.params_for(p, 'p34')
    q.update(V0=VBAT, V1=0.0, delay=0.0, L=a.L)
    if a.R:
        q['R'] = a.R
    ke = 60 / (2 * np.pi * am.KV_NOMINAL)
    ds0 = int(am.pwm_to_dshot(BASE_PWM))
    d0 = (am.MIN_DUTY + (ds0 - 47) * (2000 - am.MIN_DUTY) // 2000) / 2000.0
    w0 = am.steady_state(q, d0, VBAT)
    g0 = am.g_of_rpm(q, w0 * 60 / (2 * np.pi))
    i0 = (q['kq'] * w0 ** 2 + q['b'] * w0 + q['T0']) / ke
    vb = VBAT - q['Rb'] * g0 * d0 * i0
    pcu0 = i0 ** 2 * q['R'] * am.RW_FRAC
    rows = []
    for fn in [float(x) for x in a.freqs.split(',')]:
        f = fn * LOOP_HZ / 20000.0
        H = am.linear_response(q, w0, [f], d0=d0, vbus=vb)[0]      # rad/s per unit duty
        s = 2j * np.pi * f
        Z = q['L'] * s + q['R'] + q['Rb'] * (g0 * d0) ** 2
        Hi = (g0 * vb - ke * H) / Z                                 # A per unit duty
        for A in [float(x) for x in a.amps.split(',')]:
            amp = A * 20.0                                          # duty units
            du = amp / 2000.0
            lin_i = abs(Hi) * du
            lin = dict(an_idc_amp=lin_i, an_tau_amp=ke * lin_i, an_tau_lag_deg=lagdeg(1 + 0j, Hi),
                       an_rpm_amp=abs(H) * du * 60 / (2 * np.pi), an_rpm_lag_deg=lagdeg(1 + 0j, H),
                       an_pcu=pcu0 * (1 + 0.5 * (lin_i / i0) ** 2))
            # nonlinear averaged simulation: settle 0.6 s, then the sine
            skip, win = max(0.1, 1.0 / f), max(0.5, 4.0 / f)
            t_on = 0.6
            t1 = t_on + skip + win
            n = int(round(t1 / am.TICK))
            tt = np.arange(n) * am.TICK
            ds = np.full(n, ds0)
            r = am.simulate(q, tt, ds, 0.0, t1, out_dt=am.TICK if f > 50 else 2e-4, dshot_cmd=ds, w0=w0,
                            duty_mod=lambda x: np.where(x >= t_on, amp * np.sin(2 * np.pi * f * (x - t_on)), 0.0))
            m = r['t'] > t_on + skip
            tm = r['t'][m] - t_on
            _, hd = fit(tm, r['duty'][m] * 2000, f)
            mi, hi = fit(tm, r['i'][m], f)
            mr, hr = fit(tm, r['rpm'][m], f)
            irms_dc = float(np.sqrt(np.mean(r['i'][m] ** 2)))
            row = dict(f_nominal=fn, f_actual=f, a_pct=A, amp=amp, rpm0=w0 * 60 / (2 * np.pi), i0=i0,
                       pcu0=pcu0, **lin,
                       avg_duty_amp=abs(hd), avg_idc_amp=abs(hi), avg_tau_amp=ke * abs(hi),
                       avg_tau_lag_deg=lagdeg(hd, hi), avg_rpm_amp=abs(hr), avg_rpm_lag_deg=lagdeg(hd, hr),
                       avg_rpm_mean=mr, avg_i_mean=mi,
                       avg_pcu=float(np.mean(r['pcu'][m])), avg_pcu_ratio=float(np.mean(r['pcu'][m])) / pcu0,
                       avg_idc_rms=irms_dc)
            rows.append(row)
            print(json.dumps({k: round(v, 4) if isinstance(v, float) else v for k, v in row.items()}))
    with open(a.out, 'w', newline='') as fo:
        w = csv.DictWriter(fo, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    main()
