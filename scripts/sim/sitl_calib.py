#!/usr/bin/env python3
"""Calibrate / verify the AM32 SITL motor model against bench logs 33/34.

Each bench.lua test segment (1 steps, 2 coast-downs, 3 small sweep,
4 big sweep) is replayed separately through the SITL (3 s of zero
throttle for arming first, then the logged pwm as bidirectional
DShot300 at 1200 frames/s like ArduPilot), several in parallel. The
battery is a constant voltage source in the SITL, set per segment from
the FC-measured BAT voltage (mean over the segment, plus the internal
resistance sag the model adds itself).

usage:
  sitl_calib.py eval --model-params P.json [--set L_phase=2e-5 ...] [--tag T]
  sitl_calib.py grid  ... (see main)
"""
import argparse
import itertools
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calibrate as cal  # noqa: E402
import sitl_run as sr  # noqa: E402

OUT = os.path.join(cal.HOME, 'notes/sim/sitl_calib')
ELF = os.path.join(cal.HOME, 'am32-sim/variants/sitl_ramp16.elf')
ARM_S = 3.0
SEGMENTS = [('np33', 1), ('np33', 2), ('np33', 3), ('np33', 4),
            ('p34', 1), ('p34', 2), ('p34', 3), ('p34', 4)]
FIT_TESTS = (1, 2, 3)
BIDIR = False


def seg_range(d, test):
    k = d['test'] == test
    t0 = d['ts'][k][0] - 0.3
    t1 = d['ts'][k][-1] + (1.0 if test == 2 else 0.05)
    return t0, t1


def sitl_params(base, name, d, t0, t1, over):
    """SITL model dict for one segment"""
    prop = cal.DATASETS[name][4]
    p = dict(base)
    p.update(over)
    mb = (d['tb'] >= t0) & (d['tb'] <= t1)
    vbat = float(np.max(d['vb'][mb])) + p.get('v_offset', 0.0)  # least-loaded reading ~ open circuit
    mp = dict(kv=p['kv'], R_phase=p['R_phase'], L_phase=p['L_phase'],
              J=p['J_m'] + (p['J_p'] if prop else 0.0), kq=p['kq'] if prop else 0.0,
              T0=p['T0'], b=p['b'], Vbat=vbat, Rb=p['Rb'], rds_on=p['rds_on'],
              commutation_transfer=p.get('commutation_transfer', 1.0))
    return sr.model_from_params(mp)


def run_segment(args):
    name, test, base, over, elf, ee = args
    d = cal.run_data(name)
    t0, t1 = seg_range(d, test)
    model = sitl_params(base, name, d, t0, t1, over)
    m = (d['tc'] >= t0) & (d['tc'] <= t1)
    tc, pw = d['tc'][m], d['pwm'][m]
    # start the replay at the first non-zero command; before that the
    # SITL motor is brought up gently (arm, ramp from DShot 48 over 1 s,
    # hold 0.5 s). AM32 startup straight into a high throttle is
    # unreliable in the SITL with this light rotor (see report), and the
    # bench start-up transient is excluded from the comparison anyway
    k0 = np.argmax(pw > 1000)
    t_first = tc[k0]
    if test == 2:
        # coast-downs: replay the first hold and cut only
        cut = tc[k0:][np.argmax(pw[k0:] <= 1000)]
        t1 = cut + 1.0
    m2 = (tc >= t_first) & (tc <= t1)
    ds = sr.pwm_to_dshot(pw[m2])
    st = sr.Stim(period_ns=833_333, bidir=BIDIR)
    st.const(0, ARM_S)
    st.ramp(48, ds[0], 1.0)
    st.const(ds[0], 0.5)
    t_rep = st.t
    st.samples(np.concatenate([tc[m2] - t_first, [t1 - t_first]]), np.concatenate([ds, [ds[-1]]]))
    lg = sr.run(st, model, elf=elf, eeprom_overrides=ee, dt_ns=200_000)
    tb = lg['t'] - t_rep + t_first + base.get('delay', 0.005)
    t0 = t_first
    keep = tb > t0
    res = dict(name=name, test=test, t0=t0, t1=t1, end=lg['_end'],
               desync=int(lg['desync'][keep].max() - lg['desync'][keep].min()) if keep.any() else -1)
    sel = d['use'] & (d['test'] == test) & (d['ts'] < tb[-1]) & (d['ts'] > t0)
    e = np.interp(d['ts'][sel], tb, lg['rpm']) - d['rpm'][sel]
    res['rpm_rms'] = float(np.sqrt(np.mean(e ** 2))) if len(e) else None
    res['rpm_mean_err'] = float(np.mean(e)) if len(e) else None
    if test == 1:
        steps = []
        pw_s = np.interp(d['ts'], d['tc'], d['pwm'])
        for pwm in range(1100, 1900, 100):
            k = (d['test'] == 1) & (pw_s == pwm)
            if not k.any():
                continue
            te = d['ts'][k][-1]
            mb = k & (d['ts'] > te - 1.0)
            ms = (tb > te - 1.0) & (tb < te)
            steps.append((pwm, float(d['rpm'][mb].mean()), float(lg['rpm'][ms].mean()),
                          float(lg['ibus'][ms].mean()), float(lg['irms'][ms].mean()), float(lg['vbus'][ms].mean())))
        res['steps'] = steps
    if test in (3, 4):
        mm = (d['ts'] >= t0) & (d['ts'] <= t1) & (d['test'] == test)
        mc = (d['tc'] >= t0) & (d['tc'] <= t1)
        hb = cal.sweep_response(d['ts'][mm], d['rpm'][mm], d['f'][mm], d['tc'][mc], d['pwm'][mc])
        ms = (tb >= d['ts'][mm][0]) & (tb <= d['ts'][mm][-1])
        fs = np.interp(tb[ms], d['ts'][mm], d['f'][mm])
        hs = cal.sweep_response(tb[ms], lg['rpm'][ms], fs, d['tc'][mc], d['pwm'][mc])
        res['bode'] = [[float(x) for x in hb[i, :5]] + [float(x) for x in hs[np.argmin(abs(hs[:, 0] - hb[i, 0])), 1:5]]
                       for i in range(len(hb))]
        # gain error in log terms, over 3-50 Hz
        b = np.array(res['bode'])
        k = b[:, 0] >= 3
        res['gain_log_rms'] = float(np.sqrt(np.mean(np.log(b[k, 5] / b[k, 1]) ** 2)))
        res['gain_log_mean'] = float(np.mean(np.log(b[k, 5] / b[k, 1])))
    res['trace'] = dict(t=tb[::5].tolist(), rpm=lg['rpm'][::5].tolist())
    return res


def evaluate(base, over=None, segments=SEGMENTS, elf=ELF, ee=None, workers=8):
    over = over or {}
    jobs = [(n, t, base, over, elf, ee) for n, t in segments]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        res = list(ex.map(run_segment, jobs))
    return res


def score(res, tests=FIT_TESTS):
    """combined fit score: rpm rms (in 300 rpm units) plus sweep gain
    log-error (in 10 % units), over the fit tests"""
    s = []
    for r in res:
        if r['test'] not in tests or r['rpm_rms'] is None:
            continue
        s.append((r['rpm_rms'] / 300.0) ** 2)
        if 'gain_log_rms' in r:
            s.append((r['gain_log_rms'] / 0.1) ** 2)
    return float(np.sqrt(np.mean(s)))


def summary(res):
    lines = []
    for r in res:
        line = '%-5s test %d: rpm rms %6.0f mean %+6.0f desync %d' % (r['name'], r['test'], r['rpm_rms'] or -1, r['rpm_mean_err'] or 0, r['desync'])
        if 'gain_log_rms' in r:
            line += ' | gain log err rms %.3f mean %+.3f' % (r['gain_log_rms'], r['gain_log_mean'])
        lines.append(line)
        if 'steps' in r:
            lines.append('      steps ' + ' '.join('%d:%+.1f%%' % (s[0], 100 * (s[2] / s[1] - 1)) for s in r['steps']))
    return '\n'.join(lines)


def base_from_avg(fn, L_phase=20e-6, rds=0.002):
    p = json.load(open(fn))
    return dict(kv=1300.0, R_phase=(p['R'] - 2 * rds) / 2, L_phase=L_phase, J_m=p['J_m'], J_p=p['J_p'],
                kq=p['kq'], T0=min(p['T0'], 0.003), b=p['b'], Rb=p['Rb'], rds_on=rds, delay=p['delay'])


def parse_sets(sets):
    over = {}
    for s in sets:
        k, v = s.split('=')
        over[k] = float(v)
    return over


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['eval', 'grid'])
    ap.add_argument('--avg-params', default=os.path.join(cal.OUT, 'params_v2.json'))
    ap.add_argument('--base', help='SITL base param json (overrides --avg-params)')
    ap.add_argument('--set', action='append', default=[])
    ap.add_argument('--grid', action='append', default=[], help='name=v1,v2,v3')
    ap.add_argument('--tag', default='')
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--elf', default=ELF)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    base = json.load(open(a.base)) if a.base else base_from_avg(a.avg_params)
    base.update(parse_sets(a.set))
    if a.cmd == 'eval':
        res = evaluate(base, elf=a.elf, workers=a.workers)
        print(summary(res))
        print('score (fit tests) %.3f  holdout (test 4) %.3f' % (score(res), score(res, (4,))))
        json.dump(dict(base=base, results=res, score=score(res), score_holdout=score(res, (4,))),
                  open(os.path.join(OUT, 'sitl_eval%s.json' % a.tag), 'w'))
    else:
        names, vals = [], []
        for g in a.grid:
            k, v = g.split('=')
            names.append(k)
            vals.append([float(x) for x in v.split(',')])
        rows = []
        segs = [s for s in SEGMENTS if s[1] in FIT_TESTS]
        for combo in itertools.product(*vals):
            over = dict(zip(names, combo))
            res = evaluate(dict(base, **over), segments=segs, elf=a.elf, workers=a.workers)
            sc = score(res)
            rows.append((over, sc))
            print(json.dumps(over), 'score %.3f' % sc)
            print(summary(res))
            sys.stdout.flush()
        rows.sort(key=lambda r: r[1])
        json.dump(dict(base=base, rows=rows), open(os.path.join(OUT, 'sitl_grid%s.json' % a.tag), 'w'), indent=1)
        print('best', rows[0])


if __name__ == '__main__':
    main()
