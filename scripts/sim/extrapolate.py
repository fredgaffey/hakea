#!/usr/bin/env python3
"""Simple-physics extrapolation of 1P throttle pulsing to 50-400 Hz, to set
beside the SITL grid.

Three levels, all from the calibrated averaged model (params JSON):
  1. analytic linear: small-signal transfer duty -> current/torque/rpm of
     the averaged motor (L, R, Ke, J, prop damping) around the operating
     point, no ramp limit;
  2. analytic with ramp limit: the AM32 slew limit S = ramp * 20 kHz duty/s
     turns a sine of amplitude A into (at most) a triangle of amplitude
     min(A, S/(4f)); its fundamental is (8/pi^2)*S/(4f) once fully limited
     (smooth interpolation between, from a describing-function estimate);
  3. nonlinear averaged simulation (avgmodel.py, includes the real AM32
     ramp limiter, low-rpm/startup ramp rules and duty clamp).
Copper loss: I_rms^2 * R (loop R at 25 C) and x1.30 for ~100 C windings.

usage: extrapolate.py --params P.json --out FILE.csv
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

FREQS = (50, 100, 150, 200, 250, 300, 400)
AMPS = (5, 10, 20, 30, 40)
BASE_PWM = 1500
VBAT = 24.3


def ramp_fund(a_duty, f, ramp):
    """fundamental amplitude of the slew-limited sine (describing function).
    Exact for the unlimited and fully limited (triangle) cases; numerically
    integrated in between."""
    S = ramp * 20000.0
    t = np.linspace(0, 3 / f, 6000)
    x = a_duty * np.sin(2 * np.pi * f * t)
    y = np.zeros_like(x)
    dt = t[1] - t[0]
    for i in range(1, len(t)):
        dy = np.clip(x[i] - y[i - 1], -S * dt, S * dt)
        y[i] = y[i - 1] + dy
    k = t > 1 / f
    A = np.column_stack([np.sin(2 * np.pi * f * t[k]), np.cos(2 * np.pi * f * t[k])])
    c, *_ = np.linalg.lstsq(A, y[k], rcond=None)
    return float(np.hypot(*c))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--params', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--ramps', default='16,255')
    ap.add_argument('--L', type=float, default=36e-6, help='line-line inductance (H); 36 uH = 2 x the 18 uH phase prior')
    a = ap.parse_args()
    p = json.load(open(a.params))
    q = cal.params_for(p, 'p34')
    q.update(V0=VBAT, V1=0.0, delay=0.0, L=a.L)
    ke = 60 / (2 * np.pi * am.KV_NOMINAL)
    ds0 = am.pwm_to_dshot(BASE_PWM)
    d0 = (am.MIN_DUTY + (ds0 - 47) * (2000 - am.MIN_DUTY) // 2000) / 2000.0
    w0 = am.steady_state(q, d0, VBAT)
    g0 = am.g_of_rpm(q, w0 * 60 / (2 * np.pi))
    # steady current and bus voltage at the operating point
    i0 = (q['kq'] * w0 ** 2 + q['b'] * w0 + q['T0']) / ke
    vb = VBAT - q['Rb'] * g0 * d0 * i0
    rows = []
    for ramp in [int(x) for x in a.ramps.split(',')]:
        for f in FREQS:
            for A in AMPS:
                a_ds = A / 100 * 2000
                a_duty = a_ds * (2000 - am.MIN_DUTY) / 2000  # duty units
                # 1. linear
                H = am.linear_response(q, w0, [f], d0=d0, vbus=vb)[0]  # rad/s per unit duty
                s = 2j * np.pi * f
                Z = q['L'] * s + q['R'] + q['Rb'] * (g0 * d0) ** 2
                # current per unit duty: (g V - ke * H) / Z
                Hi = (g0 * vb - ke * H) / Z
                lin_rpm = abs(H) * a_duty / 2000 * 60 / (2 * np.pi)
                lin_i = abs(Hi) * a_duty / 2000
                # 2. with ramp limit
                af = ramp_fund(a_duty, f, ramp)
                rl_rpm = abs(H) * af / 2000 * 60 / (2 * np.pi)
                rl_i = abs(Hi) * af / 2000
                rl_tau = ke * rl_i
                # 3. nonlinear averaged sim: settle 1 s then 1 s of sine
                dt = 1e-4
                tt = np.arange(0, 2.0, dt)
                ds = np.where(tt < 1.0, ds0, np.clip(np.round(ds0 + a_ds * np.sin(2 * np.pi * f * (tt - 1.0))), 48, 2047))
                r = am.simulate(q, tt, ds, 0.0, 2.0, ramp_high=ramp, out_dt=1e-4, dshot_cmd=ds,
                                w0=w0)
                m = r['t'] > 1.2
                tm = r['t'][m] - 1.0
                Af = np.column_stack([np.ones_like(tm), np.sin(2 * np.pi * f * tm), np.cos(2 * np.pi * f * tm)])
                cr, *_ = np.linalg.lstsq(Af, r['rpm'][m], rcond=None)
                ci, *_ = np.linalg.lstsq(Af, r['i'][m], rcond=None)
                cd, *_ = np.linalg.lstsq(Af, r['duty'][m] * 2000, rcond=None)
                irms = float(np.sqrt(np.mean(r['i'][m] ** 2)))
                rows.append(dict(ramp=ramp, f=f, a_pct=A, rpm0=w0 * 60 / (2 * np.pi), i0=i0,
                                 lin_rpm_amp=lin_rpm, lin_i_amp=lin_i, lin_tau_amp=ke * lin_i,
                                 duty_fund_rampest=af, rl_rpm_amp=rl_rpm, rl_tau_amp=rl_tau,
                                 sim_rpm_mean=cr[0], sim_rpm_amp=float(np.hypot(cr[1], cr[2])),
                                 sim_duty_amp=float(np.hypot(cd[1], cd[2])), sim_duty_mean=cd[0],
                                 sim_i_amp=float(np.hypot(ci[1], ci[2])), sim_i_mean=ci[0], sim_irms=irms,
                                 pcu_25C=irms ** 2 * q['R'], pcu_100C=irms ** 2 * q['R'] * 1.295,
                                 pcu_ratio_vs_steady=irms ** 2 / i0 ** 2,
                                 analytic_ratio=1 + 0.5 * (float(np.hypot(ci[1], ci[2])) / i0) ** 2))
                print(json.dumps({k: round(v, 3) if isinstance(v, float) else v for k, v in rows[-1].items()}))
    with open(a.out, 'w', newline='') as fo:
        w = csv.DictWriter(fo, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    main()
