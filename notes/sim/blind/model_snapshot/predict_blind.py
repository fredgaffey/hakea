#!/usr/bin/env python3
"""Make the pre-registered predictions for the blind "bench2" test.

The planned test sequence (prop ON, log-34 hardware/settings) is
simulated with the calibrated averaged model (avgmodel.py + a params
JSON from calibrate.py), sampled like the Lua log (~430 Hz, with the
fitted measurement-chain delay), and analysed with exactly the same
code that will analyse the real log (compare_blind.analyze).

Battery: the open-circuit voltage at the time of the test is unknown;
predictions use V_nominal and the spread from V_nominal +- dV is folded
into the uncertainty.

usage: predict_blind.py --params P.json [--vbat 24.4] [--dv 0.5] [--out DIR]
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import avgmodel as am  # noqa: E402
import calibrate as cal  # noqa: E402
import compare_blind as cb  # noqa: E402


def plan():
    """(t, test, pwm, f) breakpoints of the bench2 plan, 1 ms steps"""
    segs = []
    t = 0.0

    def add(test, dur, pwm_fn, f=0.0):
        nonlocal t
        tt = np.arange(0, dur, 0.001)
        segs.append(np.column_stack([t + tt, np.full_like(tt, test), pwm_fn(tt), np.full_like(tt, f)]))
        t += dur

    zero = lambda tt: np.full_like(tt, 1000.0)  # noqa: E731
    add(0, 5, zero)
    for p in (1150, 1450, 1750):
        add(11, 4, lambda tt, p=p: np.full_like(tt, float(p)))
    add(0, 5, zero)
    add(12, 4, lambda tt: np.full_like(tt, 1650.0))
    add(12, 5, zero)
    add(0, 5, zero)
    for fr in (5, 15, 35, 50):
        add(13, 8, lambda tt, fr=fr: np.floor(1600 + 250 * np.sin(2 * np.pi * fr * tt)), f=fr)
    add(0, 5, zero)
    add(14, 8, lambda tt: np.floor(1400 + 150 * np.sin(2 * np.pi * 25 * tt)), f=25)
    add(0, 2, zero)
    return np.vstack(segs)


def simulate_plan(q, pl, lua_hz=430.0):
    """run the plan through the averaged model, return a BNCH-like log"""
    # the Lua script updates the output every ~2.3 ms: zero-order hold
    tl = np.arange(0, pl[-1, 0], 1.0 / lua_hz)
    idx = np.searchsorted(pl[:, 0], tl, side='right') - 1
    pwm_l = pl[idx, 2]
    r = am.simulate(dict(q, delay=0.0), tl, pwm_l, 0.0, pl[-1, 0], out_dt=0.0005)
    # logged rpm at Lua time t is the motor rpm at t - delay (chain delay)
    rpm = np.interp(tl - q['delay'], r['t'], r['rpm'], left=0)
    # telemetry: with zero throttle AM32 stops tracking (running = 0) once
    # it loses the back-EMF, and then reports 0 rpm (seen in log 33 at
    # ~1.4-2k rpm). Emulate: zero command and rpm below RPM_STOP -> 0
    rpm = np.where((pwm_l <= 1000) & (rpm < am.RPM_STOP), 0.0, rpm)
    return dict(t=tl, test=pl[idx, 1], pwm=pwm_l, f=pl[idx, 3], rpm=rpm), r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--params', required=True)
    ap.add_argument('--vbat', type=float, default=24.8, help='battery open-circuit volts at the start (assumed)')
    ap.add_argument('--vdrift', type=float, default=-0.004, help='V/s battery drift during the run (log 34: ~-0.004)')
    ap.add_argument('--dv', type=float, default=0.6, help='+- battery uncertainty for the spread')
    ap.add_argument('--json-out', default=None)
    a = ap.parse_args()
    p = json.load(open(a.params))
    q = cal.params_for(p, 'p34')
    pl = plan()
    res = {}
    for tag, v in (('nom', a.vbat), ('lo', a.vbat - a.dv), ('hi', a.vbat + a.dv)):
        qq = dict(q, V0=v, V1=a.vdrift)
        d, _ = simulate_plan(qq, pl)
        res[tag] = dict(cb.analyze(d))
    out = {k: dict(nom=res['nom'][k], lo=res['lo'][k], hi=res['hi'][k]) for k in res['nom']}
    print(json.dumps(out, indent=1))
    if a.json_out:
        json.dump(dict(params_file=a.params, vbat=a.vbat, dv=a.dv, values=out), open(a.json_out, 'w'), indent=1)


if __name__ == '__main__':
    main()


def sitl_plan_run(sitl_base, vbat, elf=None, lua_hz=430.0, delay=0.005):
    """Secondary prediction: the same plan through the AM32 SITL (calibrated
    SITL motor model, unidirectional DShot at 1200 frames/s). Returns a
    BNCH-like log sampled at the Lua rate."""
    import sitl_calib as sc
    import sitl_run as sr
    pl = plan()
    tl = np.arange(0, pl[-1, 0], 1.0 / lua_hz)
    idx = np.searchsorted(pl[:, 0], tl, side='right') - 1
    pwm_l = pl[idx, 2]
    ds = sr.pwm_to_dshot(pwm_l)
    st = sr.Stim(period_ns=833_333, bidir=False)
    st.const(0, 3.0)
    st.samples(np.concatenate([tl, [pl[-1, 0]]]), np.concatenate([ds, [0]]))
    p = dict(sitl_base)
    mp = dict(kv=p['kv'], R_phase=p['R_phase'], L_phase=p['L_phase'], J=p['J_m'] + p['J_p'], kq=p['kq'],
              T0=p['T0'], b=p['b'], Vbat=vbat, Rb=p['Rb'], rds_on=p['rds_on'],
              commutation_transfer=p.get('commutation_transfer', 1.0))
    lg = sr.run(st, sr.model_from_params(mp), elf=elf or sc.ELF, dt_ns=200_000)
    ts = lg['t'] - 3.0
    rpm = np.interp(tl - delay, ts, lg['rpm'], left=0)
    run = np.interp(tl - delay, ts, lg['running'], left=0)
    rpm = np.where((pwm_l <= 1000) & (run < 0.5), 0.0, rpm)
    return dict(t=tl, test=pl[idx, 1], pwm=pwm_l, f=pl[idx, 3], rpm=rpm), lg
