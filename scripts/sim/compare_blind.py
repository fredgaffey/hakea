#!/usr/bin/env python3
"""Blind validation "bench2": extract the pre-registered quantities from a
bench log and compare them with the hashed predictions.

The SAME analysis function (analyze) is used on the simulated run when
making the predictions (predict_blind.py) and on the real log here, so
both sides are measured identically.

Log format: BNCH messages (fields test, pwm, f, rpm), test ids
  11 = A steps (1150, 1450, 1750 us, 4 s each)
  12 = B coast-down (1650 us for 4 s, then 1000 us for 5 s)
  13 = C sines 1600 +- 250 us at f = 5, 15, 35, 50 Hz (8 s each)
  14 = D sine 1400 +- 150 us at 25 Hz (8 s)

Definitions (identical for prediction and measurement):
  A  rpm_mean       mean rpm over the last 1.0 s of each step
  B  rpm_cut        mean rpm over the 0.5 s before the cut
     rpm_tX         median rpm of samples within +-20 ms of cut + X s
                    (X = 0.25, 0.5, 1, 2); 0 = stopped / no telemetry
     t_half         time from cut until rpm first < rpm_cut / 2 (s)
  C/D (window = last 6 s of each 8 s sine, i.e. skipping 2 s settling)
     rpm_mean       mean rpm in the window
     amp1           fundamental rpm amplitude (least squares sine fit
                    at f), i.e. half of the fundamental peak-to-peak
     pp             mean over cycles of (max - min) of the rpm samples
                    (includes harmonics and telemetry noise)
     lag_deg        phase lag of the rpm fundamental behind the pwm
                    fundamental, degrees, INCLUDING the measurement
                    chain delay (Lua -> DShot -> ESC -> telemetry -> Lua)
     gain           amp1 / pwm amplitude (rpm per us)

usage:
  compare_blind.py LOG.BIN [--pred ~/vehicles/hakea/notes/sim/blind/predictions.csv]
"""
import argparse
import csv
import os

import numpy as np

BLIND = os.path.expanduser('~/vehicles/hakea/notes/sim/blind')


def read_bnch(path):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    rows = []
    while True:
        msg = m.recv_match(type='BNCH')
        if msg is None:
            break
        rows.append((msg.TimeUS * 1e-6, msg.test, msg.pwm, msg.f, msg.rpm))
    a = np.array(rows, float)
    return dict(t=a[:, 0], test=a[:, 1], pwm=a[:, 2], f=a[:, 3], rpm=a[:, 4])


def _fit(t, y, f):
    A = np.column_stack([np.ones_like(t), np.sin(2 * np.pi * f * t), np.cos(2 * np.pi * f * t)])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    return c[0], c[1] + 1j * c[2]


def analyze(d):
    """d: dict of arrays t, test, pwm, f, rpm. Returns list of rows
    (quantity, value)."""
    out = []
    t, test, pwm, f, rpm = d['t'], d['test'], d['pwm'], d['f'], d['rpm']
    # A: steps
    for p in (1150, 1450, 1750):
        k = (test == 11) & (pwm == p)
        if k.any():
            te = t[k][-1]
            m = k & (t > te - 1.0)
            out.append(('A_%d_rpm_mean' % p, float(rpm[m].mean())))
    # B: coast-down
    k = test == 12
    if k.any():
        tk, pk, rk = t[k], pwm[k], rpm[k]
        on = np.where(pk >= 1600)[0]
        if len(on):
            after = np.where((pk <= 1000) & (np.arange(len(pk)) > on[0]))[0]
            if len(after):
                tc = tk[after[0]]
                pre = (tk > tc - 0.5) & (tk < tc)
                r0 = float(rk[pre].mean())
                out.append(('B_rpm_cut', r0))
                for x in (0.25, 0.5, 1.0, 2.0):
                    m = (tk > tc + x - 0.02) & (tk < tc + x + 0.02)
                    out.append(('B_rpm_t%g' % x, float(np.median(rk[m])) if m.any() else float('nan')))
                post = (tk >= tc) & (rk < r0 / 2)
                out.append(('B_t_half', float(tk[post][0] - tc) if post.any() else float('nan')))
    # C/D sines
    for tid, label in ((13, 'C'), (14, 'D')):
        k = test == tid
        if not k.any():
            continue
        for fr in np.unique(f[k]):
            if fr <= 0:
                continue
            kk = k & (f == fr)
            tt = t[kk]
            m = kk & (t > tt[-1] - 6.0)
            # whole number of cycles
            ncyc = int(np.floor((t[m][-1] - t[m][0]) * fr))
            t0 = t[m][-1] - ncyc / fr
            m = m & (t >= t0)
            mean, hr = _fit(t[m], rpm[m], fr)
            _, hp = _fit(t[m], pwm[m], fr)
            lag = np.degrees(np.angle(hp) - np.angle(hr))
            lag = (lag + 360) % 360
            if lag > 300:
                lag -= 360
            cyc = np.floor((t[m] - t0) * fr)
            pps = [rpm[m][cyc == c].max() - rpm[m][cyc == c].min() for c in np.unique(cyc) if (cyc == c).sum() > 3]
            tag = '%s_%gHz' % (label, fr)
            out += [(tag + '_rpm_mean', float(rpm[m].mean())), (tag + '_amp1', float(abs(hr))),
                    (tag + '_pp', float(np.mean(pps))), (tag + '_lag_deg', float(lag)),
                    (tag + '_gain', float(abs(hr) / abs(hp)))]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--pred', default=os.path.join(BLIND, 'predictions.csv'))
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    meas = dict(analyze(read_bnch(a.log)))
    pred = {}
    with open(a.pred) as f:
        for row in csv.DictReader(f):
            pred[row['quantity']] = row
    lines = ['| quantity | predicted | range | measured | error | within? |', '|---|---|---|---|---|---|']
    n_in = n_tot = 0
    for q, row in pred.items():
        pv, pu = float(row['predicted']), float(row['plus_minus'])
        lo = float(row['lo']) if row.get('lo') not in (None, '') else pv - pu
        hi = float(row['hi']) if row.get('hi') not in (None, '') else pv + pu
        mv = meas.get(q, float('nan'))
        err = mv - pv
        ok = (lo <= mv <= hi) if np.isfinite(mv) else None
        if ok is not None:
            n_tot += 1
            n_in += ok
        lines.append('| %s | %.4g | %.4g .. %.4g | %.4g | %+.3g | %s |' % (q, pv, lo, hi, mv, err, {True: 'yes', False: 'NO', None: '-'}[ok]))
    lines.append('')
    lines.append('%d of %d measured quantities within the predicted range' % (n_in, n_tot))
    txt = '\n'.join(lines)
    print(txt)
    if a.out:
        open(a.out, 'w').write(txt + '\n')


if __name__ == '__main__':
    main()
