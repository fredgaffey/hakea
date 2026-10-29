#!/usr/bin/env python3
"""Which mechanism could make the bench's big sweep (1500+-400) respond
less per us at 10-40 Hz than the small sweep (1300+-200)?

Runs the averaged model on synthetic bench.lua sweeps and prints the
rpm/pwm gain at a few frequencies for:
  base/amplitude combinations (operating point vs amplitude),
  higher battery/bus resistance (dynamic sag the 10 Hz BAT log can't see),
  current-dependent torque constant (Isat),
  winding heating (thermal R(T)).
usage: compression_hypotheses.py PARAMS.json
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import avgmodel as am  # noqa: E402
import calibrate as cal  # noqa: E402

FREQS = (3, 8, 15, 25, 40)


def sweep_gain(q, base, amp, warm=None):
    dt = 1 / 430.0
    t = np.arange(0, 60, dt)
    f = 50 ** (t / 60)
    ph = np.cumsum(2 * np.pi * f * dt)
    pwm = np.floor(base + amp * np.sin(ph))
    tt = np.concatenate([np.arange(-2, 0, dt), t])
    pp = np.concatenate([np.full(int(round(2 / dt)), float(base)), pwm])
    qq = dict(q, delay=0.0, V1=0.0)
    if warm is not None:
        qq['Tw0'] = warm
    r = am.simulate(qq, tt, pp, -2.0, 60.0, out_dt=0.0005)
    r['t'] = r['t'] - 2.0
    m = r['t'] > 0
    fm = np.interp(r['t'][m], t, f)
    h = cal.sweep_response(r['t'][m], r['rpm'][m], fm, t, pwm)
    out = []
    for fc in FREQS:
        j = np.argmin(abs(h[:, 0] - fc))
        out.append(h[j, 1])
    tw = r.get('tw')
    return out, (float(tw[m].max()) if tw is not None else None), float(np.mean(r['rpm'][m]))


def main():
    p = json.load(open(sys.argv[1]))
    q = cal.params_for(p, 'p34')
    q['V0'] = 24.6
    cases = [
        ('as fitted, 1300+-200', q, 1300, 200),
        ('as fitted, 1500+-400', q, 1500, 400),
        ('as fitted, 1500+-200', q, 1500, 200),
        ('as fitted, 1300+-400', q, 1300, 400),
        ('Rb x5,   1300+-200', dict(q, Rb=q['Rb'] * 5), 1300, 200),
        ('Rb x5,   1500+-400', dict(q, Rb=q['Rb'] * 5), 1500, 400),
        ('Isat 30A, 1300+-200', dict(q, Isat=30.0), 1300, 200),
        ('Isat 30A, 1500+-400', dict(q, Isat=30.0), 1500, 400),
        ('thermal C15 R4, 1300+-200', dict(q, Cth=15.0, Rth=4.0), 1300, 200),
        ('thermal C15 R4, 1500+-400', dict(q, Cth=15.0, Rth=4.0), 1500, 400),
    ]
    print('gain rpm/us at', FREQS, ' (max winding temp, mean rpm)')
    for name, qq, base, amp in cases:
        g, tw, mr = sweep_gain(qq, base, amp)
        print('%-24s %s  Tw %s  mean %.0f' % (name, ' '.join('%5.1f' % x for x in g), tw if tw is None else '%.0f' % tw, mr))


if __name__ == '__main__':
    main()
