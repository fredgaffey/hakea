#!/usr/bin/env python3
"""End-to-end test of motorid.py on synthetic dataflash logs.

1. Get a test plan (throttle, test id, frequency at ~430 Hz): from the
   real motorid.lua run against the binding mock (if the local Lua build
   .luabuild/runlua exists), else from a Python copy of a short plan.
2. Simulate a "true" motor with motorid_model (known parameters, R
   deliberately different from the default prior), add noise, and write
   two .BIN logs (prop off / prop on) with PARM, ARM, BAT, MIDC, MIDT and
   the optional thrust-stand messages LOAD (Fz, Mz) and BPWR (Volt, Curr).
3. Run motorid.py on them and check:
   a. with thrust stand + bench current: the absolute scale, R, J, kq and
      the thrust coefficient are recovered;
   b. rpm + battery voltage only: rpm is still predicted well, and the
      parameters are off by one common scale factor s exactly as the
      identifiability argument says (J/J_true = kq/kq_true = R_true/R).

usage: test_motorid.py [--keep]   (outputs in tests/_out)
"""
import argparse
import json
import os
import shutil
import struct
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import motorid as mi  # noqa: E402
import motorid_model as mm  # noqa: E402

OUT = os.path.join(HERE, '_out')

TRUE = dict(g0=1.03, g1=1.03, g2=1.04, g3=1.04, g4=1.0, R=0.10, L=30e-6, T0=0.002, b=4e-6, J_m=3.0e-6,
            delay=0.0055, Rb=0.05)
TRUE_PROP = dict(J_p=4.0e-6, kq=1.2e-8, kT=1.5e-6)

# ------------------------------------------------------------ dataflash


class DFWriter(object):
    """Just enough of the ArduPilot dataflash format for pymavlink to read."""
    SIZES = dict(Q=8, B=1, H=2, h=2, f=4, I=4, i=4, N=16, Z=64, n=4, b=1)
    PACK = dict(Q='Q', B='B', H='H', h='h', f='f', I='I', i='i', N='16s', Z='64s', n='4s', b='b')

    def __init__(self):
        self.types = {}
        self.records = []   # (time_us, bytes)

    def define(self, name, fmt, cols):
        mid = 129 + len(self.types)
        st = '<' + ''.join(self.PACK[c] for c in fmt)
        length = 3 + struct.calcsize(st)
        self.types[name] = (mid, fmt, st)
        hdr = struct.pack('<BBB', 0xA3, 0x95, 128)
        body = struct.pack('<BB4s16s64s', mid, length, name.encode(), fmt.encode(), cols.encode())
        self.records.append((-1, hdr + body))

    def write(self, name, t_s, *vals):
        mid, fmt, st = self.types[name]
        vals = [int(round(t_s * 1e6))] + [v.encode() if isinstance(v, str) else v for v in vals]
        self.records.append((int(round(t_s * 1e6)), struct.pack('<BBB', 0xA3, 0x95, mid) + struct.pack(st, *vals)))

    def save(self, path):
        fmts = [r for r in self.records if r[0] < 0]
        msgs = sorted((r for r in self.records if r[0] >= 0), key=lambda r: r[0])
        with open(path, 'wb') as f:
            for _, b in fmts + msgs:
                f.write(b)

# ----------------------------------------------------------------- plan


def lua_plan(tmpdir):
    """MIDT plan from the real Lua script (short settings), or None"""
    runlua = os.path.join(os.path.dirname(HERE), '.luabuild', 'runlua')
    if not os.path.exists(runlua):
        return None
    csv = os.path.join(tmpdir, 'plan.csv')
    r = subprocess.run([runlua, os.path.join(HERE, 'test_motorid_lua.lua'), 'short', csv],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        raise RuntimeError('Lua plan generation failed')
    a = np.genfromtxt(csv, delimiter=',', names=True)
    return dict(t=a['t'], test=a['test'], pwm=a['pwm'], f=a['f'])


def python_plan():
    """the same short plan as the Lua 'short' scenario, without Lua"""
    segs, t = [], 0.0

    def add(test, dur, fn):
        nonlocal t
        tt = np.arange(0, dur, 1 / 430.0)
        p, f = fn(tt)
        segs.append(np.column_stack([t + tt, np.full_like(tt, test), p, f]))
        t += dur
    idle = lambda tt: (np.full_like(tt, 1000.0), 0 * tt)  # noqa: E731
    add(0, 2, idle)
    for p in range(1100, 1801, 100):
        add(1, 1.5, lambda tt, p=p: (np.full_like(tt, float(p)), 0 * tt))
    add(0, 2, idle)
    for _ in range(2):
        add(2, 2, lambda tt: (np.full_like(tt, 1500.0), 0 * tt))
        add(2, 2, idle)
    for test, base, amp in ((3, 1300, 200), (4, 1500, 400)):
        add(0, 2, idle)
        k = np.log(50.0)
        add(test, 20, lambda tt, b=base, a=amp: (np.floor(b + a * np.sin(2 * np.pi * 20 / k * (np.exp(k * tt / 20) - 1))),
                                                 np.exp(k * tt / 20)))
    add(0, 3, idle)
    a = np.vstack(segs)
    return dict(t=a[:, 0] + 1.0, test=a[:, 1], pwm=a[:, 2], f=a[:, 3])

# ------------------------------------------------------------ synthesis


def make_log(path, plan, prop, esc, seed, stand=True):
    rng = np.random.default_rng(seed)
    t0 = 100.0
    tc = plan['t'] + t0
    q = dict(TRUE)
    q['J'] = TRUE['J_m'] + (TRUE_PROP['J_p'] if prop else 0.0)
    q['kq'] = TRUE_PROP['kq'] if prop else 0.0
    q['V0'], q['V1'] = 25.2, -0.003
    t_end = tc[-1] + 0.5
    r = mm.simulate(q, esc, tc - t0 + 1.0, plan['pwm'], 0.0, t_end - t0 + 1.0, out_dt=0.0005)
    r['t'] = r['t'] + t0 - 1.0
    # telemetry rpm at the Lua sample times (the delay is already in the input path)
    rpm = np.interp(tc, r['t'], r['rpm_telem']) + rng.normal(0, 40, len(tc))
    rpm = np.where(np.interp(tc, r['t'], r['running']) > 0.5, np.maximum(rpm, 0), 0.0)
    df = DFWriter()
    df.define('PARM', 'QNf', 'TimeUS,Name,Value')
    df.define('ARM', 'QB', 'TimeUS,ArmState')
    df.define('BAT', 'QBff', 'TimeUS,Inst,Volt,Curr')
    df.define('MIDC', 'QBHhBHBHHHff', 'TimeUS,ver,run,prop,chan,func,poles,mask,pwmin,pwmax,gap,dur')
    df.define('MIDT', 'QBHff', 'TimeUS,test,pwm,f,rpm')
    df.define('LOAD', 'QBffffff', 'TimeUS,I,Fx,Fy,Fz,Mx,My,Mz')
    df.define('BPWR', 'QBff', 'TimeUS,I,Volt,Curr')
    for k, v in dict(MOT_PWM_MIN=1000, MOT_PWM_MAX=2000, SERVO_BLH_POLES=14, SERVO_DSHOT_RATE=3,
                     SCHED_LOOP_RATE=400, SERVO1_FUNCTION=33).items():
        df.write('PARM', 1.0, k, float(v))
    df.write('ARM', tc[0] - 0.01, 1)
    df.write('MIDC', tc[0] - 0.005, 1, 1, prop, 1, 33, 14, 15, 1000, 1900, 2.0, float(tc[-1] - tc[0]))
    for i in range(len(tc)):
        df.write('MIDT', tc[i], int(plan['test'][i]), int(plan['pwm'][i]), float(plan['f'][i]), float(rpm[i]))
    df.write('ARM', tc[-1] + 0.01, 0)
    # FC battery monitor: 10 Hz, 100 ms average of the bus voltage, phantom current
    for tb in np.arange(t0, t_end, 0.1):
        m = (r['t'] > tb - 0.1) & (r['t'] <= tb)
        if m.any():
            df.write('BAT', tb, 0, float(r['vbus'][m].mean() + rng.normal(0, 0.01)), 69.0)
    if stand:
        # thrust stand at 50 Hz (load cell averages over its 20 ms sample)
        for tl in np.arange(t0, t_end, 0.02):
            m = (r['t'] > tl - 0.02) & (r['t'] <= tl)
            if not m.any():
                continue
            w = r['w'][m]
            Fz = TRUE_PROP['kT'] * np.mean(w * w) if prop else 0.0
            Mz = np.mean(r['torque'][m])     # stator reaction torque
            # stand convention under test: thrust positive, torque NEGATIVE
            # (motorid flips the sign itself)
            df.write('LOAD', tl, 0, rng.normal(0, 0.02), rng.normal(0, 0.02), Fz + rng.normal(0, 0.03) + 0.2,
                     rng.normal(0, 0.001), rng.normal(0, 0.001), -(Mz + rng.normal(0, 0.0005)) - 0.003)
            df.write('BPWR', tl, 0, float(np.mean(r['vbus'][m])), float(np.mean(r['ibus'][m]) + 0.3 + rng.normal(0, 0.05)))
    df.save(path)


def run(argv):
    print('$ motorid.py ' + ' '.join(argv))
    mi.main(argv)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--keep', action='store_true')
    a = ap.parse_args()
    if os.path.exists(OUT):
        shutil.rmtree(OUT)
    os.makedirs(OUT)
    esc = mm.Am32Settings()
    esc.minimum_duty_cycle = 40
    plan = lua_plan(OUT)
    print('plan from', 'motorid.lua (mock)' if plan is not None else 'Python copy')
    if plan is None:
        plan = python_plan()
    f_off, f_on = os.path.join(OUT, 'synth_off.BIN'), os.path.join(OUT, 'synth_on.BIN')
    make_log(f_off, plan, 0, esc, 1)
    make_log(f_on, plan, 1, esc, 2)
    fails = []

    def check(cond, what):
        print('  %s %s' % ('ok  ' if cond else 'FAIL', what))
        if not cond:
            fails.append(what)

    common = [f_off, f_on, '--esc', 'minimum_duty_cycle=40', '--max-nfev', '60', '--no-plots', '--prop-diameter', '0.127',
              '--verbose', '1']
    # a. with thrust stand + bench current
    run(['fit'] + common + ['--out', os.path.join(OUT, 'stand')])
    m = json.load(open(os.path.join(OUT, 'stand', 'motor_model.json')))
    sc = m['identifiability']['scale']
    mot, pr = m['model']['motor_fit'], m['model']['props']['1']
    print('  scale', json.dumps({k: v for k, v in sc.items() if k != 'sources'}),
          [(x['source'], round(np.exp(x['log_s']), 3)) for x in sc.get('sources', [])])
    check(sc['method'] == 'measured', 'scale fixed by measurements')
    check(abs(mot['R'] / TRUE['R'] - 1) < 0.1, 'R %.4f vs true %.4f' % (mot['R'], TRUE['R']))
    check(abs(mot['J_m'] / TRUE['J_m'] - 1) < 0.15, 'J_m %.3g vs true %.3g' % (mot['J_m'], TRUE['J_m']))
    check(abs(pr['kq'] / TRUE_PROP['kq'] - 1) < 0.1, 'kq %.3g vs true %.3g' % (pr['kq'], TRUE_PROP['kq']))
    check(abs(pr['J_p'] / TRUE_PROP['J_p'] - 1) < 0.2, 'J_p %.3g vs true %.3g' % (pr['J_p'], TRUE_PROP['J_p']))
    kT = pr['thrust_stand']['kT']
    check(abs(kT / TRUE_PROP['kT'] - 1) < 0.03, 'kT %.4g vs true %.4g' % (kT, TRUE_PROP['kT']))
    check(pr['thrust_stand'].get('torque_sign_flipped') is True, 'negative torque convention detected')
    hold = [v['holdout_rpm_rms'] for v in m['metrics'].values() if 'holdout_rpm_rms' in v]
    check(max(hold) < 150, 'hold-out rpm rms %s < 150' % ['%.0f' % h for h in hold])
    check(all(r['prop_source'] == 'MIDC context record' for lg in m['inputs']['logs'] for r in lg['runs']),
          'prop state read from MIDC')

    # b. rpm + battery voltage only: scale set by the (wrong) default R prior
    run(['fit'] + common + ['--channels', 'rpm,bat_volt', '--out', os.path.join(OUT, 'rpm_only')])
    m2 = json.load(open(os.path.join(OUT, 'rpm_only', 'motor_model.json')))
    mot2, pr2 = m2['model']['motor_fit'], m2['model']['props']['1']
    s_R = TRUE['R'] / mot2['R']
    print('  implied scale from R: %.3f' % s_R)
    check(m2['identifiability']['scale']['method'] == 'prior', 'scale from prior only')
    check(abs(mot2['J_m'] / TRUE['J_m'] / s_R - 1) < 0.15, 'J_m/J_true = %.3f ~ s' % (mot2['J_m'] / TRUE['J_m']))
    check(abs(pr2['kq'] / TRUE_PROP['kq'] / s_R - 1) < 0.1, 'kq/kq_true = %.3f ~ s' % (pr2['kq'] / TRUE_PROP['kq']))
    hold2 = [v['holdout_rpm_rms'] for v in m2['metrics'].values() if 'holdout_rpm_rms' in v]
    check(max(hold2) < 150, 'rpm still predicted: hold-out %s' % ['%.0f' % h for h in hold2])
    if not a.keep:
        shutil.rmtree(OUT)
    print('PASS' if not fails else 'FAILED: %s' % fails)
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
