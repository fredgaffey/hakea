#!/usr/bin/env python3
"""Calibrate the averaged model (avgmodel.py) against Fred's bench logs.

Data (CSV made by bench_extract.py from the dataflash logs):

  NP  log 33, prop OFF, bench.lua (message BNCH: test, pwm, f, rpm ~430 Hz)
        test 1 steps 1100..1800 us, test 2 coast-downs (1500 us -> 0),
        test 3 sweep 1300+-200 us 1->50 Hz, test 4 sweep 1500+-400 us.
  P   log 32, prop ON, Lua sweep 1500+-400 us 1->50 Hz (message SWP).
        run 2 (598-658 s) is used for fitting, run 1 (540-591 s) is
        held out for verification.
  BAT.Volt (FC-measured battery voltage, 100 ms averages) for both.

The motor-only parameters are shared between the two logs; the prop
adds inertia J_p and the aerodynamic torque coefficient kq:

  shared : g0..g4 (effective voltage gain vs rpm), R, T0, b, delay, Rb
  NP     : J_m, battery V0/V1
  P      : J_p (prop inertia, added to J_m), kq, battery V0/V1

Scale degeneracy: with only speed and battery-voltage measurements the
model is invariant under J -> s*J, R -> R/s, kq -> s*kq, Rb -> Rb/s, T0,b
-> s*T0, s*b (all rpm predictions identical). A prior on R (scaled from
the T-Motor V2307 datasheet resistance) fixes s. RPM predictions do not
depend on it; absolute torque/current do.

usage:
  calibrate.py fit [--prior R=0.16,0.04] [--tag X]
  calibrate.py verify --params FILE
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import avgmodel as am  # noqa: E402

HOME = os.path.expanduser('~/vehicles/hakea')
BENCH = os.path.join(HOME, 'notes/sim/bench')
OUT = os.path.join(HOME, 'notes/sim/calib')

# name: (log, message, t0, t1, prop)
DATASETS = {
    'np33': ('00000033', 'BNCH', 316.8, 504.9, False),   # prop off, bench.lua
    'p34': ('00000034', 'BNCH', 249.0, 446.1, True),     # prop on, bench.lua (2nd arm)
    'p32r2': ('00000032', 'SWP', 597.95, 657.95, True),  # secondary check only
    'p32r1': ('00000032', 'SWP', 540.40, 590.85, True),  # secondary check only
}
FIT_SETS = ['np33', 'p34']
HOLDOUT = ['p32r2', 'p32r1']
# bench.lua test ids used for fitting; test 4 (big sweep 1500+-400) is
# held out for verification. SWP logs (32) count as test 4.
FIT_TESTS = tuple(int(x) for x in os.environ.get('HAKEA_FIT_TESTS', '1,2,3').split(','))

SHARED = ['g0', 'g1', 'g2', 'g3', 'g4', 'R', 'T0', 'b', 'Rb', 'delay']
if os.environ.get('HAKEA_ISAT'):
    SHARED.append('Isat')
if os.environ.get('HAKEA_THERMAL'):
    SHARED += ['Rth', 'Cth']
PER = {'np33': ['J_m', 'V0_np33', 'V1_np33'],
       'p34': ['J_p', 'kq', 'V0_p34', 'V1_p34']}
FIXED = dict(L=30e-6)
X0 = dict(Rth=3.0, Cth=15.0, Isat=80.0, g0=1.0, g1=1.0, g2=1.0, g3=1.02, g4=1.02, R=0.16, T0=0.003, b=1e-6, Rb=0.02,
          delay=0.005, J_m=3e-6, J_p=5e-6, kq=1.5e-8,
          V0_np33=25.25, V1_np33=-0.002, V0_p34=24.9, V1_p34=-0.008)
LOG_SPACE = {'Rth', 'Cth', 'Isat', 'R', 'T0', 'b', 'Rb', 'J_m', 'J_p', 'kq'}

SIG_RPM = 300.0   # rpm, expected time-domain fit error
SIG_V = 0.03      # V, 100 ms battery-voltage averages
PRIORS = {}       # name: (value, sigma)


def fit_names():
    names = list(SHARED)
    for ds in FIT_SETS:
        names += PER[ds]
    return names


def load(name):
    return np.genfromtxt(os.path.join(BENCH, name + '.csv'), delimiter=',', names=True)


def run_data(name):
    log, msg, t0, t1, prop = DATASETS[name]
    s = load(log + '_' + msg)
    b = load(log + '_BAT')
    mc = (s['t'] >= t0 - 0.2) & (s['t'] <= t1)
    ms = (s['t'] >= t0) & (s['t'] <= t1)
    mb = (b['t'] >= t0 + 0.1) & (b['t'] <= t1)
    d = dict(name=name, t0=t0, t1=t1, prop=prop, tc=s['t'][mc], pwm=s['pwm'][mc],
             ts=s['t'][ms], rpm=s['rpm'][ms], f=s['f'][ms],
             tb=b['t'][mb], vb=b['Volt'][mb])
    d['test'] = s['test'][ms] if 'test' in s.dtype.names else np.full(ms.sum(), 4.0)
    # samples used in the fit: motor commanded and settled at least 0.5 s
    # after a start from rest, or coasting/braking above 3500 rpm (the
    # telemetry holds stale values below that)
    pw = d['pwm']
    on = np.interp(d['ts'], d['tc'], (pw > 1000).astype(float), left=0) > 0.5
    starts = d['tc'][1:][(pw[1:] > 1000) & (pw[:-1] <= 1000)]
    if len(starts) == 0 or starts[0] > d['t0'] + 0.1:
        starts = np.concatenate([[d['t0']], starts])
    since = np.full(len(d['ts']), 1e9)
    for st in starts:
        k = d['ts'] >= st
        since[k] = np.minimum(since[k], d['ts'][k] - st)
    d['use'] = (on & (since > 0.5)) | (~on & (d['rpm'] > 3500))
    return d


def params_for(p, name):
    """global parameter dict -> the parameter set of one dataset"""
    q = dict(FIXED)
    for k in SHARED:
        q[k] = p[k]
    q['J'] = p['J_m'] + (p['J_p'] if DATASETS[name][4] else 0.0)
    q['kq'] = p['kq'] if DATASETS[name][4] else 0.0
    q['V0'] = p.get('V0_' + name, p.get('V0_p34'))
    q['V1'] = p.get('V1_' + name, p.get('V1_p34'))
    return q


def unpack(x, names):
    return {k: (float(np.exp(v)) if k in LOG_SPACE else float(v)) for k, v in zip(names, x)}


def pack(p, names):
    return np.array([np.log(p[k]) if k in LOG_SPACE else p[k] for k in names])


def sim_run(q, d, ramp_high=am.RAMP_HIGH_DEFAULT, out_dt=0.0005):
    r = am.simulate(q, d['tc'] - d['t0'], d['pwm'], 0.0, d['t1'] - d['t0'],
                    out_dt=out_dt, ramp_high=ramp_high)
    r['t'] = r['t'] + d['t0']
    return r


def bat_avg(r, tb):
    """model bus voltage averaged over the 100 ms before each BAT sample"""
    c = np.concatenate([[0], np.cumsum(r['vbus'])])
    i1 = np.searchsorted(r['t'], tb)
    i0 = np.searchsorted(r['t'], tb - 0.1)
    n = np.maximum(i1 - i0, 1)
    return (c[i1] - c[i0]) / n


def residuals(x, names, data):
    p = unpack(x, names)
    out = []
    for d in data:
        q = params_for(p, d['name'])
        r = sim_run(q, d)
        m = d['use'] & np.isin(d['test'], FIT_TESTS)
        rpm_m = np.interp(d['ts'][m], r['t'], r['rpm'])
        res_rpm = (rpm_m - d['rpm'][m])[::2] / SIG_RPM
        res_v = (bat_avg(r, d['tb']) - d['vb']) / SIG_V
        n = len(res_rpm)
        out += [res_rpm, res_v * np.sqrt(n / len(res_v))]
    ntot = sum(len(o) for o in out)
    for k, (v, sg) in PRIORS.items():
        out.append(np.array([(p[k] - v) / sg * np.sqrt(ntot / 4)]))
    return np.concatenate(out)


def fit_battery_state(q, d):
    """V0/V1 differ from run to run (state of charge, temperature):
    nuisance parameters, refit from BAT.Volt with everything else
    frozen (used for held-out data)."""
    def res(x):
        return bat_avg(sim_run(dict(q, V0=x[0], V1=x[1]), d), d['tb']) - d['vb']
    sol = least_squares(res, [d['vb'][:5].mean() + 0.3, 0.0], diff_step=1e-4)
    return dict(q, V0=float(sol.x[0]), V1=float(sol.x[1]))


def metrics(q, d):
    r = sim_run(q, d)
    vm = bat_avg(r, d['tb'])
    out = dict(vbat_rms=float(np.sqrt(np.mean((vm - d['vb']) ** 2))))
    for test in np.unique(d['test']):
        if test == 0:
            continue
        m = d['use'] & (d['test'] == test)
        if not m.any():
            continue
        e = np.interp(d['ts'][m], r['t'], r['rpm']) - d['rpm'][m]
        out['test%d' % test] = dict(rpm_rms=float(np.sqrt(np.mean(e ** 2))),
                                    rpm_mean_err=float(np.mean(e)),
                                    role='fit' if (test in FIT_TESTS and d['name'] in FIT_SETS) else 'holdout')
    out['rpm_rms'] = float(np.sqrt(np.mean([v['rpm_rms'] ** 2 for k, v in out.items() if k.startswith('test')])))
    return out, r


def sweep_response(t, x, f, tref, pwm, cycles=2.0, fmin=1.0, fmax=50.0):
    """Gain/phase of x relative to pwm along a sweep: in windows of
    `cycles` periods, least-squares fit both to [1, sin, cos] at the
    local frequency and divide the phasors. Returns rows
    (f, gain, phase_deg, x_pp, x_mean)."""
    out = []
    for fc in np.unique(np.round(np.geomspace(fmin, fmax, 40), 3)):
        k = np.searchsorted(f, fc)
        if k <= 0 or k >= len(f):
            continue
        tc = t[k]
        half = cycles / fc / 2
        m = (t > tc - half) & (t < tc + half)
        mp = (tref > tc - half) & (tref < tc + half)
        if m.sum() < 8 or mp.sum() < 8:
            continue

        def phasor(tt, yy):
            A = np.column_stack([np.ones_like(tt), np.sin(2 * np.pi * fc * tt), np.cos(2 * np.pi * fc * tt)])
            c, *_ = np.linalg.lstsq(A, yy, rcond=None)
            return c[1] + 1j * c[2], c[0]

        hx, mx = phasor(t[m], x[m])
        hp, _ = phasor(tref[mp], pwm[mp])
        out.append((fc, abs(hx / hp), np.degrees(np.angle(hx / hp)), abs(hx) * 2, mx))
    return np.array(out)


def sweep_segments(d):
    """(test, t_start, t_end) of each sweep inside a dataset"""
    if DATASETS[d['name']][1] == 'BNCH':
        segs = []
        for test in (3, 4):
            k = d['test'] == test
            if k.any():
                segs.append((test, d['ts'][k][0], d['ts'][k][-1]))
        return segs
    return [(0, d['t0'], d['t1'])]


def plot_dataset(q, d, tag):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    mt, r = metrics(q, d)
    segs = sweep_segments(d)
    fig, ax = plt.subplots(4, 1, figsize=(13, 13))
    ax[0].plot(d['ts'], d['rpm'], lw=.3, label='bench')
    ax[0].plot(r['t'], r['rpm'], lw=.3, label='model', alpha=.7)
    ax[0].set_ylabel('rpm'); ax[0].legend()
    ax[0].set_title('%s  rpm rms err %.0f (fit samples)' % (d['name'], mt['rpm_rms']))
    t_end = segs[-1][2]
    for a, (lo, hi) in zip(ax[1:3], [(segs[-1][1] + 1.0, segs[-1][1] + 3.0), (t_end - 0.2, t_end - 0.05)]):
        m = (d['ts'] > lo) & (d['ts'] < hi)
        mr = (r['t'] > lo) & (r['t'] < hi)
        a.plot(d['ts'][m], d['rpm'][m], '.-', ms=2, lw=.6, label='bench')
        a.plot(r['t'][mr], r['rpm'][mr], lw=1, label='model')
        a.set_ylabel('rpm'); a.legend()
    ax[3].plot(d['tb'], d['vb'], '.-', ms=2, label='BAT.Volt (bench)')
    ax[3].plot(d['tb'], bat_avg(r, d['tb']), '.-', ms=2, label='model (100 ms avg)')
    ax[3].set_ylabel('V'); ax[3].legend(); ax[3].set_title('vbat rms err %.3f V' % mt['vbat_rms'])
    plt.tight_layout(); plt.savefig(os.path.join(OUT, 'timeseries_%s%s.png' % (d['name'], tag)), dpi=75); plt.close()
    bodes = []
    for test, lo, hi in segs:
        m = (d['ts'] >= lo) & (d['ts'] <= hi)
        mc = (d['tc'] >= lo - 0.5) & (d['tc'] <= hi)
        hb = sweep_response(d['ts'][m], d['rpm'][m], d['f'][m], d['tc'][mc], d['pwm'][mc])
        mr = (r['t'] >= lo) & (r['t'] <= hi)
        fm = np.interp(r['t'][mr], d['ts'][m], d['f'][m])
        hm = sweep_response(r['t'][mr], r['rpm'][mr], fm, d['tc'][mc], d['pwm'][mc])
        bodes.append((test, hb, hm))
    return mt, r, bodes


def plot_bode(bodes, name, tag):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
    for i, (test, hb, hm) in enumerate(bodes):
        lab = 'test %d' % test if test else ''
        ax[0].loglog(hb[:, 0], hb[:, 1], 'o-', color='C%d' % (2 * i), label='bench ' + lab)
        ax[0].loglog(hm[:, 0], hm[:, 1], 's--', color='C%d' % (2 * i + 1), label='model ' + lab)
        ax[1].semilogx(hb[:, 0], np.unwrap(np.radians(hb[:, 2])) * 180 / np.pi, 'o-', color='C%d' % (2 * i))
        ax[1].semilogx(hm[:, 0], np.unwrap(np.radians(hm[:, 2])) * 180 / np.pi, 's--', color='C%d' % (2 * i + 1))
    ax[0].set_ylabel('rpm per us of pwm'); ax[1].set_ylabel('phase deg (incl. chain delay)')
    ax[1].set_xlabel('Hz'); ax[0].legend(); ax[0].set_title('rpm/pwm along sweep: %s' % name)
    for a in ax:
        a.grid(True, which='both', alpha=.3)
    plt.tight_layout(); plt.savefig(os.path.join(OUT, 'bode_%s%s.png' % (name, tag)), dpi=75); plt.close()
    rows = []
    for test, hb, hm in bodes:
        for row in hb:
            j = np.argmin(abs(hm[:, 0] - row[0]))
            rows.append([test, row[0], row[1], row[2], row[3], row[4], hm[j, 1], hm[j, 2], hm[j, 3], hm[j, 4]])
    np.savetxt(os.path.join(OUT, 'bode_%s%s.csv' % (name, tag)), np.array(rows), delimiter=',',
               header='test,f_hz,bench_gain_rpm_per_us,bench_phase_deg,bench_rpm_pp,bench_rpm_mean,'
                      'model_gain,model_phase_deg,model_rpm_pp,model_rpm_mean', fmt='%.5g', comments='')


def steps_table(q, d):
    """steady no-prop steps (test 1): bench vs model mean rpm"""
    rows = []
    r = sim_run(q, d)
    pw_s = np.interp(d['ts'], d['tc'], d['pwm'])
    for pwm in range(1100, 1900, 100):
        k = (d['test'] == 1) & (pw_s == pwm)
        if not k.any():
            continue
        t1 = d['ts'][k][-1]
        m = k & (d['ts'] > t1 - 1.2)
        mr = (r['t'] > t1 - 1.2) & (r['t'] < t1)
        rows.append((pwm, float(d['rpm'][m].mean()), float(r['rpm'][mr].mean())))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['fit', 'verify'])
    ap.add_argument('--params')
    ap.add_argument('--prior', action='append', default=[], help='name=value,sigma')
    ap.add_argument('--tag', default='')
    ap.add_argument('--max-nfev', type=int, default=300)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    for pr in a.prior:
        k, v = pr.split('=')
        PRIORS[k] = tuple(float(z) for z in v.split(','))
    if a.cmd == 'fit':
        names = fit_names()
        data = [run_data(n) for n in FIT_SETS]
        x0 = pack(X0, names)
        sol = least_squares(residuals, x0, args=(names, data), x_scale='jac', diff_step=1e-3,
                            max_nfev=a.max_nfev, verbose=1)
        p = unpack(sol.x, names)
        try:
            J = sol.jac
            cov = np.linalg.inv(J.T @ J) * np.sum(sol.fun ** 2) / max(1, len(sol.fun) - len(x0))
            p['_sigma_fit_space'] = {k: float(s) for k, s in zip(names, np.sqrt(np.diag(cov)))}
        except np.linalg.LinAlgError:
            pass
        p['_priors'] = PRIORS
        p['_cost'] = float(sol.cost)
        p['_fit_sets'] = FIT_SETS
        fn = os.path.join(OUT, 'params%s.json' % a.tag)
        json.dump(p, open(fn, 'w'), indent=1)
        print(json.dumps({k: v for k, v in p.items() if not k.startswith('_')}, indent=1))
        print('wrote', fn)
        a.params = fn
    p = json.load(open(a.params))
    summary = {}
    for name in FIT_SETS + HOLDOUT:
        d = run_data(name)
        q = params_for(p, name)
        if name in HOLDOUT:
            q = fit_battery_state(q, d)
        mt, r, bodes = plot_dataset(q, d, a.tag)
        plot_bode(bodes, name, a.tag)
        summary[name] = mt
        print(name, json.dumps(mt))
        if DATASETS[name][1] == 'BNCH':
            rows = steps_table(q, d)
            for row in rows:
                print('  step pwm %d: bench %.0f rpm, model %.0f rpm (%+.1f%%)' % (row[0], row[1], row[2], 100 * (row[2] / row[1] - 1)))
            summary[name]['steps'] = rows
    json.dump(summary, open(os.path.join(OUT, 'metrics%s.json' % a.tag), 'w'), indent=1)


if __name__ == '__main__':
    main()
