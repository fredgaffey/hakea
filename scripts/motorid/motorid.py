#!/usr/bin/env python3
"""motorid: identify a motor + ESC + prop model from ArduPilot bench logs.

Reads dataflash .BIN logs from bench runs of motorid.lua (messages MIDC
context + MIDT excitation), of the older bench.lua (BNCH) or of a plain
sweep script (SWP), fits the commutation-averaged AM32 ESC + BLDC motor
model in motorid_model.py, and writes a model JSON (averaged-model
parameters + ESCSim SITL model sections, with provenance), a markdown
report and plots.

Workflow (see README.md):
  1. prop-off runs  -> motor/ESC parameters (stage 1)
  2. prop-on runs   -> prop parameters, motor fixed (stage 2)
  3. optional joint refinement of everything (--joint)
  4. held-out data (default: test 4, the big sweep, of every run; and
     whole logs given with --validate) -> validation errors

Identifiability: with only rpm and battery voltage the model is
invariant under J, kq, b, T0 -> s*(...), R, Rb, L -> (...)/s, so one
absolute quantity must be supplied: by default a prior on the loop
resistance R (--r-phase-phase or --prior R=value,sigma); when a bench
current sensor or a torque cell is logged, they fix s instead.

Measured channels (--channels): by default only the excitation rpm and
the FC battery voltage (BAT.Volt), plus thrust-stand messages (LOAD,
BPWR) when present. The FC battery current, ESC-telemetry voltage and
current are never used unless named explicitly.

usage:
  motorid.py info LOG.BIN ...
  motorid.py fit LOG.BIN ... [--validate LOG.BIN] [--prop LOG=off|on|N]
                 [--eeprom esc.bin] [--r-phase-phase OHM] [--out DIR] ...
Run with -h for all options.
"""
import argparse
import datetime
import hashlib
import json
import os
import re
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import motorid_model as mm  # noqa: E402

TOOL_VERSION = '0.1.0'
EXCITATION_MSGS = ('MIDT', 'BNCH', 'SWP')   # preferred first
MOTOR_FUNCTIONS = set(range(33, 41)) | set(range(82, 86))  # Motor1..12

# measurement channels: name -> description
CHANNELS = {
    'rpm': 'excitation-record rpm (ESC telemetry via Lua)',
    'bat_volt': 'FC battery monitor voltage BAT.Volt',
    'bat_curr': 'FC battery monitor current BAT.Curr (off by default)',
    'esc_volt': 'ESC telemetry voltage ESC.Volt (off by default)',
    'esc_curr': 'ESC telemetry current ESC.Curr (off by default)',
    'load_thrust': 'thrust stand LOAD.Fz',
    'load_torque': 'thrust stand LOAD.Mz',
    'bench_volt': 'bench power monitor BPWR.Volt',
    'bench_curr': 'bench power monitor BPWR.Curr',
}
AUTO_CHANNELS = ('rpm', 'bat_volt', 'load_thrust', 'load_torque', 'bench_volt', 'bench_curr')

# parameters fitted in log space (positive, wide dynamic range)
LOG_SPACE = {'R', 'T0', 'b', 'Rb', 'J_m', 'J_p', 'kq', 'Isat', 'Rth', 'Cth', 'L'}

# allowed range of the log-space parameters (SI units); generous, only
# there to stop unidentifiable ones running away
LOG_BOUNDS = dict(R=(1e-3, 10.0), T0=(1e-6, 1.0), b=(1e-10, 1e-2), Rb=(1e-4, 2.0), J_m=(1e-8, 1e-2),
                  J_p=(1e-9, 1e-1), kq=(1e-12, 1e-3), Isat=(1.0, 1e4), Rth=(0.01, 100.0), Cth=(0.1, 1e4),
                  L=(1e-7, 1e-2))

# starting values; J and kq are replaced by data-driven guesses
X0 = dict(R=0.16, T0=0.003, b=1e-6, Rb=0.03, delay=0.005, J_m=3e-6, J_p=3e-6, kq=1.5e-8,
          L=30e-6, Isat=80.0, Rth=3.0, Cth=15.0)

# ===================================================================
# reading logs


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def log_short_name(path):
    """'.../00000033.BIN' -> '33'; other names: the file stem"""
    stem = os.path.splitext(os.path.basename(path))[0]
    return str(int(stem)) if stem.isdigit() else re.sub(r'[^A-Za-z0-9_]', '_', stem)


def read_log(path, msg=None, esc_instance=None, bat_instance=0):
    """Read everything motorid needs from one dataflash log.

    Returns a dict: path, name, sha256, msg (excitation message used),
    params, exc (t, test, pwm, f, rpm), bat (t, volt, curr), arm
    (t, state), midc (list of dicts), midp, load, bpwr, esc (arrays).
    """
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    want = ['PARM', 'BAT', 'ARM', 'MIDC', 'MIDP', 'LOAD', 'BPWR', 'ESC'] + list(EXCITATION_MSGS)
    exc = {k: [] for k in EXCITATION_MSGS}
    params, bat, arm, midc, midp, load, bpwr, esc = {}, [], [], [], [], [], [], {}
    while True:
        x = m.recv_match(type=want)
        if x is None:
            break
        t = x.get_type()
        ts = x.TimeUS * 1e-6
        if t == 'PARM':
            params[x.Name] = x.Value
        elif t in exc:
            d = x.to_dict()
            exc[t].append((ts, d.get('test', np.nan), d['pwm'], d.get('f', 0.0), d['rpm']))
        elif t == 'BAT':
            d = x.to_dict()
            if d.get('Inst', d.get('Instance', 0)) == bat_instance:
                bat.append((ts, d['Volt'], d.get('Curr', np.nan)))
        elif t == 'ARM':
            arm.append((ts, x.ArmState))
        elif t == 'MIDC':
            d = x.to_dict(); d['t'] = ts; midc.append(d)
        elif t == 'MIDP':
            d = x.to_dict(); d['t'] = ts; midp.append(d)
        elif t == 'LOAD':
            d = x.to_dict()
            load.append((ts, d.get('Fz', np.nan), d.get('Mz', np.nan), d.get('Fx', np.nan), d.get('Fy', np.nan),
                         d.get('Mx', np.nan), d.get('My', np.nan)))
        elif t == 'BPWR':
            d = x.to_dict()
            bpwr.append((ts, d.get('Volt', np.nan), d.get('Curr', np.nan)))
        elif t == 'ESC':
            d = x.to_dict()
            esc.setdefault(d.get('Instance', 0), []).append((ts, d.get('RPM', np.nan), d.get('Volt', np.nan),
                                                             d.get('Curr', np.nan), d.get('Temp', np.nan)))
    if msg is None:
        msg = next((k for k in EXCITATION_MSGS if exc[k]), None)
    if msg is None or not exc.get(msg):
        raise ValueError('%s: no excitation records (%s) found' % (path, '/'.join(EXCITATION_MSGS)))
    a = np.array(exc[msg], float)
    rpm = a[:, 4].copy()
    rpm[rpm < 0] = np.nan  # Lua logs -1 when there is no telemetry
    out = dict(path=os.path.abspath(path), name=log_short_name(path), sha256=sha256_file(path), msg=msg,
               params=params, midc=midc, midp=midp,
               exc=dict(t=a[:, 0], test=a[:, 1], pwm=a[:, 2], f=a[:, 3], rpm=rpm),
               bat=_arr(bat, ('t', 'volt', 'curr')), arm=_arr(arm, ('t', 'state')),
               load=_arr(load, ('t', 'Fz', 'Mz', 'Fx', 'Fy', 'Mx', 'My')), bpwr=_arr(bpwr, ('t', 'volt', 'curr')))
    if esc_instance is None:
        esc_instance = 0
        if midc:
            esc_instance = int(midc[-1].get('chan', 1)) - 1
    out['esc'] = _arr(esc.get(esc_instance, []), ('t', 'rpm', 'volt', 'curr', 'temp'))
    out['esc_instance'] = esc_instance
    return out


def _arr(rows, names):
    if not rows:
        return None
    a = np.array(rows, float)
    return {k: a[:, i] for i, k in enumerate(names)}


def split_runs(log, min_len=5.0, window=None):
    """Split the excitation records into runs: a new run starts at an
    arm event, at a MIDC context record, or after a gap > 1 s.
    Returns a list of (t0, t1, midc-or-None)."""
    t = log['exc']['t']
    if window:
        k = (t >= window[0]) & (t <= window[1])
        t = t[k]
    if len(t) == 0:
        return []
    cuts = set(np.where(np.diff(t) > 1.0)[0] + 1)
    marks = []
    if log['arm'] is not None:
        marks += list(log['arm']['t'])
    marks += [c['t'] for c in log['midc']]
    for tm in marks:
        i = np.searchsorted(t, tm)
        if 0 < i < len(t):
            cuts.add(int(i))
    edges = [0] + sorted(cuts) + [len(t)]
    runs = []
    for i0, i1 in zip(edges[:-1], edges[1:]):
        if i1 - i0 < 10 or t[i1 - 1] - t[i0] < min_len:
            continue
        t0, t1 = t[i0], t[i1 - 1]
        ctx = None
        for c in log['midc']:
            if t0 - 2.0 <= c['t'] <= t0 + 1.0:
                ctx = c
        runs.append((float(t0), float(t1), ctx))
    return runs


def steady_segments(ts, pwm, rpm, pwm_min, min_len=1.2, avg_len=1.0):
    """Constant-throttle holds of at least min_len seconds: list of
    (pwm, t_start, t_end, mask of the last avg_len seconds, or of the
    second half for holds shorter than 2 avg_len)."""
    out = []
    if len(ts) == 0:
        return out
    change = np.concatenate([[0], np.where(np.diff(pwm) != 0)[0] + 1, [len(pwm)]])
    for i0, i1 in zip(change[:-1], change[1:]):
        if pwm[i0] <= pwm_min or ts[i1 - 1] - ts[i0] < min_len:
            continue
        m = np.zeros(len(ts), bool)
        m[i0:i1] = ts[i0:i1] > ts[i1 - 1] - min(avg_len, 0.5 * (ts[i1 - 1] - ts[i0]))
        m &= np.isfinite(rpm)
        if m.sum() >= 5:
            out.append((float(pwm[i0]), float(ts[i0]), float(ts[i1 - 1]), m))
    return out


def make_run(log, t0, t1, ctx, cfg, idx, nruns):
    """Cut one run out of a log and prepare the arrays used for fitting."""
    e = log['exc']
    pmin = float(log['params'].get('MOT_PWM_MIN', 1000.0) or 1000.0)
    pmax = float(log['params'].get('MOT_PWM_MAX', 2000.0) or 2000.0)
    if pmax <= pmin:
        pmin, pmax = 1000.0, 2000.0
    # dead start: throttle commanded but the motor did not turn (ESC not
    # ready yet after arming, seen in log 32). Start the run just before
    # the motor actually starts, otherwise the model spins up too early.
    k = (e['t'] >= t0) & (e['t'] <= t1)
    tk, pk, rk = e['t'][k], e['pwm'][k], e['rpm'][k]
    first_on = np.argmax(pk > pmin) if np.any(pk > pmin) else None
    if first_on is not None:
        spinning = np.where(np.nan_to_num(rk[first_on:]) > 0)[0]
        if len(spinning) and tk[first_on + spinning[0]] - tk[first_on] > 0.5:
            t0 = float(tk[first_on + spinning[0]] - 0.2)
            print('NOTE: %s: throttle commanded from %.2f s but no rotation until %.2f s; run starts at %.2f s'
                  % (log['name'], tk[first_on], tk[first_on + spinning[0]], t0))
            k = (e['t'] >= t0) & (e['t'] <= t1)
    test = e['test'][k].copy()
    if np.all(np.isnan(test)):
        test[:] = cfg.untagged_test  # plain sweep logs (SWP) have no test field
    r = dict(log=log['name'], log_path=log['path'], t0=float(t0), t1=float(t1), ctx=ctx,
             ts=e['t'][k], pwm=e['pwm'][k], f=e['f'][k], rpm=e['rpm'][k], test=test,
             pwm_min=pmin, pwm_max=pmax, suffix=('r%d' % (idx + 1)) if nruns > 1 else '')
    # voltage measurement (first available of the enabled channels)
    r['vsrc'] = None
    for ch, src, key in (('bench_volt', 'bpwr', 'volt'), ('esc_volt', 'esc', 'volt'), ('bat_volt', 'bat', 'volt')):
        if ch in cfg.channels and log.get(src) is not None:
            s = log[src]
            mb = (s['t'] >= t0 + 0.1) & (s['t'] <= t1) & np.isfinite(s[key])
            if mb.sum() >= 3:
                r['tb'], r['vb'], r['vsrc'] = s['t'][mb], s[key][mb], ch
                dtb = np.median(np.diff(r['tb'])) if mb.sum() > 1 else 0.1
                r['vb_win'] = float(np.clip(dtb, 0.005, 0.1))
                break
    if r['vsrc'] is None:
        r['tb'], r['vb'], r['vb_win'] = np.zeros(0), np.zeros(0), 0.1
    # current measurement (only used to fix the scale s)
    r['isrc'] = None
    for ch, src, key in (('bench_curr', 'bpwr', 'curr'), ('esc_curr', 'esc', 'curr'), ('bat_curr', 'bat', 'curr')):
        if ch in cfg.channels and log.get(src) is not None:
            s = log[src]
            mb = (s['t'] >= t0) & (s['t'] <= t1) & np.isfinite(s[key])
            if mb.sum() >= 3:
                r['ti'], r['ib'], r['isrc'] = s['t'][mb], s[key][mb], ch
                break
    # thrust stand
    r['load'] = None
    if log.get('load') is not None and ({'load_thrust', 'load_torque'} & set(cfg.channels)):
        s = log['load']
        mb = (s['t'] >= t0) & (s['t'] <= t1)
        if mb.sum() >= 3:
            r['load'] = {kk: v[mb] for kk, v in s.items()}
    # samples used for fitting: motor commanded and settled at least
    # 0.5 s after a start from rest, or coasting/braking above
    # coast_min_rpm (below that the telemetry is stale or drops out)
    on = r['pwm'] > pmin
    starts = r['ts'][1:][on[1:] & ~on[:-1]]
    if on[0]:
        starts = np.concatenate([[r['ts'][0]], starts])
    since = np.full(len(r['ts']), 1e9)
    for st in starts:
        kk = r['ts'] >= st
        since[kk] = np.minimum(since[kk], r['ts'][kk] - st)
    valid = np.isfinite(r['rpm'])
    r['use'] = valid & ((on & (since > cfg.settle_s)) | (~on & (r['rpm'] > cfg.coast_min_rpm)))
    r['w0'] = float(r['rpm'][0] * 2 * np.pi / 60) if valid[0] and r['rpm'][0] > cfg.esc.rpm_stop else 0.0
    return r


def classify_prop(run, cfg):
    """Decide prop off / on for a run.

    Order: --prop override for the log, then the MIDC context record,
    then a heuristic: at steady throttle a motor without prop runs
    within a few % of its no-load speed Kv * duty * Vbat (ratio ~1.0;
    measured ~1.02 for Hakea's 2307), a prop loads it down (~0.92 at
    the highest throttle). Uses the highest-duty holds (duty >= 0.4).
    Returns (prop_id, source, details); prop_id 0 = off, >= 1 = prop type.
    """
    ov = cfg.prop_override.get(run['log'])
    if ov is not None:
        return ov, 'command line', {}
    if run['ctx'] is not None and 'prop' in run['ctx']:
        p = int(run['ctx']['prop'])
        if p >= 0:
            return p, 'MIDC context record', {}
    rows = []
    for pwm, ta, tb_, m in steady_segments(run['ts'], run['pwm'], run['rpm'], run['pwm_min']):
        duty = float(mm.static_duty(mm.pwm_to_dshot([pwm], run['pwm_min'], run['pwm_max']), cfg.esc)[0])
        rpm = float(np.mean(run['rpm'][m]))
        if len(run['tb']) == 0 or rpm < cfg.esc.rpm_stop:
            continue  # no voltage, or the motor was not turning
        mv = (run['tb'] > tb_ - 1.0) & (run['tb'] <= tb_ + 0.1)
        v = float(np.mean(run['vb'][mv])) if mv.any() else float(np.interp(tb_, run['tb'], run['vb']))
        rows.append((duty, rpm / (cfg.kv * duty * v), pwm))
    det = dict(holds=[dict(pwm=p, duty=round(d, 3), ratio=round(r, 4)) for d, r, p in rows])
    # the prop's load grows ~ rpm^2, so the separation is clearest at high
    # duty: Hakea 2307 at duty 0.8 (pwm 1800) 1.01 off vs 0.89 on, but at
    # duty 0.5 (pwm 1500) only 1.02 vs 0.96
    for dmin, (r_off, r_on) in ((0.6, (cfg.ratio_off, cfg.ratio_on)), (0.4, (0.99, 0.95))):
        hi = [r for d, r, p in rows if d >= dmin]
        if hi:
            break
    if not hi:
        return None, 'heuristic: no steady holds at duty >= 0.4, use --prop', det
    ratio = float(np.median(hi))
    det['median_ratio'] = round(ratio, 4)
    if ratio >= r_off:
        return 0, 'heuristic (rpm/no-load speed %.3f >= %.2f at duty >= %.1f)' % (ratio, r_off, dmin), det
    if ratio <= r_on:
        return 1, 'heuristic (rpm/no-load speed %.3f <= %.2f at duty >= %.1f)' % (ratio, r_on, dmin), det
    return None, 'heuristic ambiguous (rpm/no-load speed %.3f at duty >= %.1f), use --prop' % (ratio, dmin), det


# ===================================================================
# model evaluation


class Config(object):
    """Everything that controls a fit (filled from the command line)."""

    def __init__(self, **kw):
        self.esc = mm.Am32Settings()
        self.kv = 1300.0
        self.g_knots = mm.G_KNOTS
        self.step_us = mm.DEFAULT_STEP_US
        self.channels = list(AUTO_CHANNELS)
        self.fit_tests = None            # None = every non-zero test id not held out
        self.holdout_tests = (4,)
        self.untagged_test = 4
        self.settle_s = 0.5
        self.coast_min_rpm = 3500.0
        self.sig_rpm = 300.0
        self.sig_v = 0.03
        self.priors = {}                 # name: (value, sigma)
        self.fixed = dict(L=30e-6)
        self.extra_free = []
        self.prop_override = {}
        self.ratio_off = 0.96   # rpm / (Kv duty V) above this at duty >= 0.6: no prop
        self.ratio_on = 0.94    # below this: prop on
        self.max_nfev = 300
        self.restarts = 3                # extra least-squares passes from the solution
        self.diff_step = 1e-2            # relative finite-difference step for the Jacobian
        self.verbose = 1
        self.__dict__.update(kw)

    def is_fit_test(self, test):
        test = np.asarray(test)
        ok = (test != 0) & ~np.isin(test, self.holdout_tests)
        if self.fit_tests is not None:
            ok &= np.isin(test, self.fit_tests)
        return ok


def params_for(P, run):
    """global parameter dict -> the parameter set of one run"""
    q = {k: v for k, v in P.items() if '@' not in k}
    pid = run['prop']
    q['J'] = P['J_m'] + (P.get('J_p@%d' % pid, 0.0) if pid else 0.0)
    q['kq'] = P.get('kq@%d' % pid, 0.0) if pid else 0.0
    q['V0'] = P['V0@' + run['name']]
    q['V1'] = P['V1@' + run['name']]
    return q


def sim_run(cfg, q, run, out_dt=0.0005):
    r = mm.simulate(q, cfg.esc, run['ts'] - run['t0'], run['pwm'], 0.0, run['t1'] - run['t0'] + 0.01,
                    w0=run['w0'], pwm_min=run['pwm_min'], pwm_max=run['pwm_max'], out_dt=out_dt,
                    step_us=cfg.step_us, g_knots=cfg.g_knots)
    r['t'] = r['t'] + run['t0']
    return r


def window_avg(r, key, tq, win):
    """model signal averaged over the `win` seconds before each time"""
    c = np.concatenate([[0], np.cumsum(r[key])])
    i1 = np.searchsorted(r['t'], tq)
    i0 = np.searchsorted(r['t'], tq - win)
    n = np.maximum(i1 - i0, 1)
    return (c[i1] - c[i0]) / n


def unpack(x, names):
    return {k: (float(np.exp(v)) if k.split('@')[0] in LOG_SPACE else float(v)) for k, v in zip(names, x)}


def pack(P, names):
    return np.array([np.log(P[k]) if k.split('@')[0] in LOG_SPACE else P[k] for k in names])


def residuals(x, names, base, runs, cfg):
    P = dict(base)
    P.update(unpack(x, names))
    out = []
    for run in runs:
        r = sim_run(cfg, params_for(P, run), run)
        m = run['use'] & cfg.is_fit_test(run['test'])
        rpm_m = np.interp(run['ts'][m], r['t'], r['rpm'])
        res_rpm = (rpm_m - run['rpm'][m])[::2] / cfg.sig_rpm
        out.append(res_rpm)
        if len(run['tb']):
            res_v = (window_avg(r, 'vbus', run['tb'], run['vb_win']) - run['vb']) / cfg.sig_v
            out.append(res_v * np.sqrt(max(len(res_rpm), 1) / len(res_v)))
    ntot = sum(len(o) for o in out)
    for k, (v, sg) in cfg.priors.items():
        if k in P:
            # the prior competes with ~ntot correlated samples; weight as in
            # Hakea's calibrate.py so it reliably pins the scale
            out.append(np.array([(P[k] - v) / sg * np.sqrt(ntot / 4.0)]))
    return np.concatenate(out)


def fit_stage(label, names, P, runs, cfg):
    """Least-squares fit of the parameters `names`, everything else in P fixed."""
    from scipy.optimize import least_squares
    if not names:
        return P, {}
    t_start = time.time()
    x0 = pack(P, names)
    if cfg.verbose:
        print('[%s] fitting %d parameters on %d run(s): %s' % (label, len(names), len(runs),
                                                               ', '.join(r['name'] for r in runs)))
    # keep log-space parameters within a sane range: a parameter the data
    # cannot see (e.g. T0 next to b) otherwise drifts towards exp(-inf)
    lo = np.array([np.log(LOG_BOUNDS.get(k.split('@')[0], (1e-12, 1e12))[0]) if k.split('@')[0] in LOG_SPACE
                   else -np.inf for k in names])
    hi = np.array([np.log(LOG_BOUNDS.get(k.split('@')[0], (1e-12, 1e12))[1]) if k.split('@')[0] in LOG_SPACE
                   else np.inf for k in names])
    x0 = np.clip(x0, lo + 1e-9, hi - 1e-9)
    # The cost surface has long flat valleys (near-degenerate parameter
    # combinations) where the trust region shrinks and the solver stops
    # early; restarting from the solution resets it. Repeat while the cost
    # still drops by more than 0.2 %.
    nfev, sol, passes = 0, None, 0
    while passes < cfg.restarts + 1 and nfev < cfg.max_nfev:
        new = least_squares(residuals, x0 if sol is None else sol.x, args=(names, P, runs, cfg), x_scale='jac',
                            diff_step=cfg.diff_step, bounds=(lo, hi), max_nfev=cfg.max_nfev - nfev,
                            verbose=1 if cfg.verbose > 1 else 0)
        nfev += new.nfev
        passes += 1
        improved = sol is None or new.cost < sol.cost * 0.998
        if sol is None or new.cost < sol.cost:
            sol = new
        if not improved:
            break
    Pn = dict(P)
    Pn.update(unpack(sol.x, names))
    info = dict(stage=label, cost=float(sol.cost), nfev=int(nfev), passes=passes, status=int(sol.status),
                n_residuals=int(len(sol.fun)), seconds=round(time.time() - t_start, 1), params=list(names))
    try:
        J = sol.jac
        cov = np.linalg.pinv(J.T @ J) * np.sum(sol.fun ** 2) / max(1, len(sol.fun) - len(x0))
        info['sigma_fit_space'] = {k: float(s) for k, s in zip(names, np.sqrt(np.abs(np.diag(cov))))}
    except (np.linalg.LinAlgError, ValueError):
        pass
    if cfg.verbose:
        print('[%s] cost %.4g after %d evaluations (%d passes), %.0f s' % (label, sol.cost, nfev, passes, info['seconds']))
    return Pn, info


def fit_battery_only(P, run, cfg):
    """Refit only V0/V1 of a run (validation logs: battery state is a nuisance)."""
    names = ['V0@' + run['name'], 'V1@' + run['name']]
    c = Config(**cfg.__dict__)
    c.priors = {}
    c.holdout_tests = ()
    c.fit_tests = [-12345]  # no rpm samples: voltage residuals only
    c.verbose = 0
    Pn, _ = fit_stage('battery ' + run['name'], names, P, [run], c)
    return Pn


def initial_guess(runs, cfg, P):
    """Battery start values from the data, J and kq from simple physics."""
    for run in runs:
        vb = run['vb']
        if len(vb):
            P.setdefault('V0@' + run['name'], float(np.max(vb[:20])) + 0.2)
            dv = (np.median(vb[-20:]) - np.median(vb[:20])) / max(run['t1'] - run['t0'], 1.0)
            P.setdefault('V1@' + run['name'], float(min(dv, 0.0)))
        else:
            # no voltage measurement: V0 cannot be separated from g, keep a
            # nominal 6S value (g then absorbs the difference)
            P.setdefault('V0@' + run['name'], 24.0)
            P.setdefault('V1@' + run['name'], 0.0)
    R = P.get('R', X0['R'])
    ke = 60.0 / (2 * np.pi * cfg.kv)
    # mechanical time constant tau = J R / ke^2 ~ 15 ms for a small motor
    P.setdefault('J_m', 0.015 * ke * ke / R)
    for run in runs:
        pid = run['prop']
        if not pid or ('kq@%d' % pid) in P:
            continue
        # kq from the highest steady hold: torque = ke * I, I from the
        # speed shortfall vs. no-load speed
        best = None
        for pwm, ta, tb_, m in steady_segments(run['ts'], run['pwm'], run['rpm'], run['pwm_min']):
            if best is None or pwm > best[0]:
                best = (pwm, m)
        kq = X0['kq']
        if best is not None and len(run['vb']):
            duty = float(mm.static_duty(mm.pwm_to_dshot([best[0]], run['pwm_min'], run['pwm_max']), cfg.esc)[0])
            w = float(np.mean(run['rpm'][best[1]])) * 2 * np.pi / 60
            v = float(np.median(run['vb']))
            i_est = max((duty * v - ke * w) / R, 1.0)
            kq = ke * i_est / (w * w)
        P['kq@%d' % pid] = kq
        P['J_p@%d' % pid] = P['J_m']
    for k in ('R', 'T0', 'b', 'Rb', 'delay', 'L'):
        P.setdefault(k, X0[k])
    # optional sub-models are OFF (0) unless they are fitted or fixed
    for k in ('Isat', 'Rth', 'Cth'):
        if k in cfg.extra_free:
            P.setdefault(k, X0[k])
        else:
            P.setdefault(k, 0.0)
    for i in range(len(cfg.g_knots)):
        P.setdefault('g%d' % i, 1.0)
    return P

# ===================================================================
# analysis


def metrics(P, run, cfg, r=None):
    if r is None:
        r = sim_run(cfg, params_for(P, run), run)
    out = {}
    if len(run['tb']):
        vm = window_avg(r, 'vbus', run['tb'], run['vb_win'])
        out['volt_rms'] = float(np.sqrt(np.mean((vm - run['vb']) ** 2)))
    fit_m, hold_m = [], []
    for test in np.unique(run['test']):
        if test == 0:
            continue
        m = run['use'] & (run['test'] == test)
        if not m.any():
            continue
        e = np.interp(run['ts'][m], r['t'], r['rpm']) - run['rpm'][m]
        role = 'fit' if (run['role'] == 'fit' and cfg.is_fit_test([test])[0]) else 'holdout'
        out['test%d' % test] = dict(rpm_rms=float(np.sqrt(np.mean(e ** 2))), rpm_mean_err=float(np.mean(e)),
                                    rpm_rel_rms=float(np.sqrt(np.mean(e ** 2)) / max(np.mean(run['rpm'][m]), 1.0)),
                                    n=int(m.sum()), role=role)
        (fit_m if role == 'fit' else hold_m).append(e)
    if fit_m:
        out['fit_rpm_rms'] = float(np.sqrt(np.mean(np.concatenate(fit_m) ** 2)))
    if hold_m:
        out['holdout_rpm_rms'] = float(np.sqrt(np.mean(np.concatenate(hold_m) ** 2)))
    return out, r


def steady_table(run, r):
    rows = []
    for pwm, ta, tb_, m in steady_segments(run['ts'], run['pwm'], run['rpm'], run['pwm_min']):
        mr = (r['t'] > run['ts'][m][0]) & (r['t'] <= tb_)
        rows.append(dict(pwm=pwm, t=round(tb_, 2), test=int(np.median(run['test'][m])),
                         rpm_bench=float(np.mean(run['rpm'][m])), rpm_model=float(np.mean(r['rpm'][mr]))))
    return rows


def _phasor(t, y, fc):
    A = np.column_stack([np.ones_like(t), np.sin(2 * np.pi * fc * t), np.cos(2 * np.pi * fc * t)])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    return c[1] + 1j * c[2], c[0]


def frequency_response(run, r, cycles=2.0, npts=40):
    """Gain/phase of rpm vs pwm along sweeps (f varying) and at fixed
    sines, for bench and model, analysed identically at the logged sample
    times. Returns a list of dicts (test, kind, rows) with rows
    [f, gain_bench, phase_bench, gain_model, phase_model]."""
    t, f, test = run['ts'], run['f'], run['test']
    rpm_b = run['rpm']
    rpm_m = np.interp(t, r['t'], r['rpm'])
    out = []
    active = (f > 0) & np.isfinite(rpm_b)
    if not active.any():
        return out
    edges = np.where(np.diff(np.concatenate([[0], active.astype(int), [0]])) != 0)[0]
    for i0, i1 in zip(edges[::2], edges[1::2]):
        if t[i1 - 1] - t[i0] < 1.0:
            continue
        seg = slice(i0, i1)
        ts, fs = t[seg], f[seg]
        tid = int(np.median(test[seg]))
        # split a run of constant-frequency sines into separate dwells
        if fs.max() / fs.min() < 1.05 or np.all(np.diff(fs) == 0) or len(np.unique(fs)) < 20:
            for fc in np.unique(fs):
                kk = fs == fc
                tt = ts[kk]
                if tt[-1] - tt[0] < 1.0:
                    continue
                m = kk & (ts > tt[-1] - 0.75 * (tt[-1] - tt[0]))
                hp, _ = _phasor(ts[m], run['pwm'][seg][m], fc)
                hb, _ = _phasor(ts[m], rpm_b[seg][m], fc)
                hm, _ = _phasor(ts[m], rpm_m[seg][m], fc)
                out.append(dict(test=tid, kind='sine', rows=[[float(fc), abs(hb / hp), np.degrees(np.angle(hb / hp)),
                                                              abs(hm / hp), np.degrees(np.angle(hm / hp))]]))
            continue
        rows = []
        for fc in np.unique(np.round(np.geomspace(max(fs.min(), 0.5), fs.max(), npts), 3)):
            k = np.searchsorted(fs, fc) if np.all(np.diff(fs) >= 0) else np.argmin(abs(fs - fc))
            if k <= 0 or k >= len(fs):
                continue
            half = cycles / fc / 2
            m = (ts > ts[k] - half) & (ts < ts[k] + half)
            if m.sum() < 8:
                continue
            hp, _ = _phasor(ts[m], run['pwm'][seg][m], fc)
            hb, _ = _phasor(ts[m], rpm_b[seg][m], fc)
            hm, _ = _phasor(ts[m], rpm_m[seg][m], fc)
            rows.append([float(fc), abs(hb / hp), np.degrees(np.angle(hb / hp)), abs(hm / hp), np.degrees(np.angle(hm / hp))])
        if rows:
            out.append(dict(test=tid, kind='sweep', rows=rows))
    return out


def bandwidth(P, cfg, pid, pwm, vbus):
    """Model small-signal -3 dB bandwidth (Hz, without ramp limit and
    delay) and equivalent first-order time constant at a throttle."""
    q = {k: v for k, v in P.items() if '@' not in k}
    q['J'] = P['J_m'] + (P.get('J_p@%d' % pid, 0.0) if pid else 0.0)
    q['kq'] = P.get('kq@%d' % pid, 0.0) if pid else 0.0
    duty = float(mm.static_duty(mm.pwm_to_dshot([pwm]), cfg.esc)[0])
    w0 = float(mm.steady_state(q, duty, vbus, cfg.kv, cfg.g_knots))
    fr = np.geomspace(0.1, 500, 400)
    h = np.abs(mm.linear_response(q, w0, fr, cfg.kv, duty, vbus, cfg.g_knots))
    k = np.where(h < h[0] / np.sqrt(2))[0]
    f3 = float(fr[k[0]]) if len(k) else float('nan')
    return w0 * 60 / (2 * np.pi), f3, 1.0 / (2 * np.pi * f3)

# ===================================================================
# thrust stand and absolute scale


def _linfit(x, y):
    """y = a x + c, returns a, c, sigma_a"""
    A = np.column_stack([x, np.ones_like(x)])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ c
    dof = max(len(y) - 2, 1)
    cov = np.linalg.pinv(A.T @ A) * np.sum(res ** 2) / dof
    return float(c[0]), float(c[1]), float(np.sqrt(abs(cov[0, 0])))


def thrust_stand_fit(P, runs, sims, cfg, prop_diameter=None, rho=1.225):
    """Thrust and torque coefficients from LOAD (steady holds), and the
    scale factor s from measured torque / current. Returns a dict."""
    out = dict(props={}, scale_sources=[])
    tq_x, tq_y, cur_x, cur_y = [], [], [], []
    for run in runs:
        r = sims[run['name']]
        segs = steady_segments(run['ts'], run['pwm'], run['rpm'], run['pwm_min'])
        L = run.get('load')
        for pwm, ta, tb_, m in segs:
            w = float(np.mean(run['rpm'][m])) * 2 * np.pi / 60
            ta_avg = run['ts'][m][0]         # start of the averaging window
            mr = (r['t'] > ta_avg) & (r['t'] <= tb_)
            if L is not None:
                ml = (L['t'] > ta_avg) & (L['t'] <= tb_)
                if ml.sum() >= 3:
                    d = out['props'].setdefault(run['prop'], dict(w2=[], Fz=[], Mz=[]))
                    d['w2'].append(w * w)
                    d['Fz'].append(float(np.nanmean(L['Fz'][ml])))
                    d['Mz'].append(float(np.nanmean(L['Mz'][ml])))
                    if 'load_torque' in cfg.channels and np.isfinite(d['Mz'][-1]):
                        tq_x.append(float(np.mean(r['torque'][mr])))
                        tq_y.append(d['Mz'][-1])
            if run.get('isrc'):
                mi = (run['ti'] > ta_avg) & (run['ti'] <= tb_)
                if mi.sum() >= 2:
                    cur_x.append(float(np.mean(r['ibus'][mr])))
                    cur_y.append(float(np.mean(run['ib'][mi])))
        # zero-throttle tare points help the offset
    for pid, d in list(out['props'].items()):
        w2, Fz, Mz = (np.array(d[k]) for k in ('w2', 'Fz', 'Mz'))
        res = dict(n_holds=int(len(w2)))
        if 'load_thrust' in cfg.channels and len(w2) >= 3 and np.all(np.isfinite(Fz)):
            if np.median(Fz) < 0:
                Fz = -Fz
                res['thrust_sign_flipped'] = True
            kT, c, s_kT = _linfit(w2, Fz)
            res.update(kT=kT, kT_sigma=s_kT, thrust_offset=c)
        if 'load_torque' in cfg.channels and len(w2) >= 3 and np.all(np.isfinite(Mz)):
            if np.median(Mz) < 0:
                Mz = -Mz
                res['torque_sign_flipped'] = True
            kQ, c, s_kQ = _linfit(w2, Mz)
            res.update(kQ_direct=kQ, kQ_direct_sigma=s_kQ, torque_offset=c)
        if prop_diameter and pid:
            D = prop_diameter
            if 'kT' in res:
                res['Ct'] = res['kT'] * 4 * np.pi ** 2 / (rho * D ** 4)
            if 'kQ_direct' in res:
                res['Cq'] = res['kQ_direct'] * 4 * np.pi ** 2 / (rho * D ** 5)
                res['Cp'] = 2 * np.pi * res['Cq']
        out['props'][pid] = res
    # scale factor estimates (log s, sigma) from each source
    if len(tq_x) >= 3:
        tq_y = np.abs(tq_y)
        a, c, sa = _linfit(np.array(tq_x), tq_y)
        if a > 0:
            out['scale_sources'].append(dict(source='load_torque', s=a, log_s=float(np.log(a)), sigma_log=sa / a, n=len(tq_x)))
    if len(cur_x) >= 3:
        a, c, sa = _linfit(np.array(cur_x), np.array(cur_y))
        if a > 0:
            out['scale_sources'].append(dict(source=runs[0].get('isrc') or 'current', s=a, log_s=float(np.log(a)),
                                             sigma_log=sa / a, offset_A=c, n=len(cur_x)))
    return out


def resolve_scale(P, stand, cfg):
    """Combine the scale estimates (inverse variance in log s) with the
    R prior (if any) and apply the rpm-invariant scale transformation."""
    src = list(stand['scale_sources'])
    if not src:
        return P, dict(method='prior', s=1.0, note='no current/torque data: absolute scale set by the priors')
    if 'R' in cfg.priors:
        v, sg = cfg.priors['R']
        src.append(dict(source='R prior', log_s=float(np.log(P['R'] / v)), sigma_log=sg / v))
    wts = np.array([1.0 / max(s_['sigma_log'], 1e-6) ** 2 for s_ in src])
    ls = float(np.sum(wts * np.array([s_['log_s'] for s_ in src])) / np.sum(wts))
    s = float(np.exp(ls))
    return mm.scale_params(P, s), dict(method='measured', s=s, sigma_log=float(1 / np.sqrt(np.sum(wts))), sources=src)

# ===================================================================
# outputs


def plot_run(run, r, fr, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    hold = (run['test'] != 0)
    fig, ax = plt.subplots(4, 1, figsize=(12, 12))
    ax[0].plot(run['ts'], run['rpm'], lw=.3, label='bench')
    ax[0].plot(r['t'], r['rpm_telem'], lw=.3, alpha=.7, label='model')
    ax[0].set_ylabel('rpm'); ax[0].legend(loc='upper left')
    ax[0].set_title('%s  (prop %s)  rpm: bench vs model' % (run['name'], run['prop']))
    # two zooms into the last sweep (largest test id with f > 0)
    sw = (run['f'] > 0) & hold
    if sw.any():
        tid = run['test'][sw].max()
        k = (run['test'] == tid) & sw
        ta, tb_ = run['ts'][k][0], run['ts'][k][-1]
        zooms = [(ta + 1.0, ta + 3.0), (tb_ - 0.25, tb_ - 0.05)]
    else:
        zooms = [(run['t0'] + 5, run['t0'] + 8), (run['t1'] - 3, run['t1'])]
    for a, (lo, hi) in zip(ax[1:3], zooms):
        m = (run['ts'] > lo) & (run['ts'] < hi)
        mr = (r['t'] > lo) & (r['t'] < hi)
        a.plot(run['ts'][m], run['rpm'][m], '.-', ms=2, lw=.6, label='bench')
        a.plot(r['t'][mr], r['rpm'][mr], lw=1, label='model')
        a.set_ylabel('rpm'); a.legend(loc='upper left'); a.set_title('zoom %.1f-%.1f s' % (lo, hi))
    if len(run['tb']):
        ax[3].plot(run['tb'], run['vb'], '.-', ms=2, label='%s (bench)' % run['vsrc'])
        ax[3].plot(run['tb'], window_avg(r, 'vbus', run['tb'], run['vb_win']), '.-', ms=2, label='model')
    ax[3].set_ylabel('V'); ax[3].set_xlabel('log time (s)'); ax[3].legend(loc='upper right')
    plt.tight_layout(); plt.savefig(path, dpi=70); plt.close()


def plot_bode(run, fr, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ci = 0
    for seg in fr:
        a = np.array(seg['rows'])
        lab = 'test %d %s' % (seg['test'], seg['kind'])
        st = 'o-' if seg['kind'] == 'sweep' else 'o'
        ax[0].loglog(a[:, 0], a[:, 1], st, color='C%d' % ci, ms=4, label='bench ' + lab)
        ax[0].loglog(a[:, 0], a[:, 3], 's--' if seg['kind'] == 'sweep' else 's', color='C%d' % ci, ms=3, alpha=.6,
                     label='model ' + lab)
        ax[1].semilogx(a[:, 0], np.degrees(np.unwrap(np.radians(a[:, 2]))), st, color='C%d' % ci, ms=4)
        ax[1].semilogx(a[:, 0], np.degrees(np.unwrap(np.radians(a[:, 4]))), 's--' if seg['kind'] == 'sweep' else 's',
                       color='C%d' % ci, ms=3, alpha=.6)
        ci += 1
    ax[0].set_ylabel('gain (rpm per us of pwm)'); ax[1].set_ylabel('phase (deg, incl. chain delay)')
    ax[1].set_xlabel('frequency (Hz)'); ax[0].legend(fontsize=8)
    ax[0].set_title('rpm/pwm frequency response: %s' % run['name'])
    for a in ax:
        a.grid(True, which='both', alpha=.3)
    plt.tight_layout(); plt.savefig(path, dpi=70); plt.close()


def plot_steady(all_rows, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 1, figsize=(8, 5))
    for i, (name, rows) in enumerate(all_rows.items()):
        rows = [x for x in rows if x['test'] != 0]
        if not rows:
            continue
        p = np.array([x['pwm'] for x in rows])
        o = np.argsort(p)
        ax.plot(p[o], np.array([x['rpm_bench'] for x in rows])[o], 'o', color='C%d' % i, label=name + ' bench')
        ax.plot(p[o], np.array([x['rpm_model'] for x in rows])[o], 'x--', color='C%d' % i, label=name + ' model')
    ax.set_xlabel('throttle pwm (us)'); ax.set_ylabel('steady rpm (mean of last 1 s of a hold)')
    ax.grid(alpha=.3); ax.legend(fontsize=8)
    plt.tight_layout(); plt.savefig(path, dpi=70); plt.close()


def fmt(v, sig=None):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return '-'
    s = '%.4g' % v
    if sig is not None:
        s += ' (+-%s)' % sig
    return s


def sigma_text(info_list, k, v):
    """human-readable 1-sigma from the stage fits ('12%' for log-space)"""
    base = k.split('@')[0]
    if base in LOG_BOUNDS and v is not None and v <= LOG_BOUNDS[base][0] * 2.0:
        return 'at lower bound (not identifiable)'
    for info in reversed(info_list):
        sg = info.get('sigma_fit_space', {}).get(k)
        if sg is not None:
            if k.split('@')[0] in LOG_SPACE:
                return '%.2g%%' % (100 * sg) if sg < 5 else 'unconstrained'
            return '%.2g' % sg
    return None

# ===================================================================
# main driver


def parse_prop(s):
    """'off' -> 0, 'on' -> 1, 'N' -> prop type N; None if not understood"""
    s = s.strip().lower()
    if s == 'off':
        return 0
    if s == 'on':
        return 1
    return int(s) if s.isdigit() else None


def build_config(a):
    cfg = Config()
    if a.eeprom:
        cfg.esc = mm.Am32Settings.from_eeprom(open(a.eeprom, 'rb').read())
    for kv_ in a.esc or []:
        k, v = kv_.split('=')
        if not hasattr(cfg.esc, k):
            raise SystemExit('unknown ESC setting %s' % k)
        setattr(cfg.esc, k, float(v) if '.' in v else int(v))
    cfg.kv = float(a.kv or cfg.esc.motor_kv)
    if a.g_knots:
        cfg.g_knots = tuple(float(x) for x in a.g_knots.split(','))
    cfg.step_us = a.step_us or mm.DEFAULT_STEP_US
    if a.channels and a.channels != 'auto':
        cfg.channels = [c.strip() for c in a.channels.split(',')]
        bad = [c for c in cfg.channels if c not in CHANNELS]
        if bad:
            raise SystemExit('unknown channel(s) %s; known: %s' % (bad, ', '.join(CHANNELS)))
    cfg.holdout_tests = tuple(int(x) for x in a.holdout_tests.split(',') if x.strip()) if a.holdout_tests else ()
    if a.fit_tests:
        cfg.fit_tests = tuple(int(x) for x in a.fit_tests.split(','))
    cfg.untagged_test = a.untagged_test
    cfg.max_nfev = a.max_nfev
    cfg.diff_step = a.diff_step
    cfg.verbose = a.verbose
    # the absolute-scale prior (see module docstring)
    if a.r_phase_phase:
        rpp = a.r_phase_phase + 2 * a.rds_on
        cfg.priors['R'] = (rpp, max(0.1 * rpp, 0.005))
    for pr in a.prior or []:
        k, v = pr.split('=')
        cfg.priors[k] = tuple(float(z) for z in v.split(','))
    if not any(k in cfg.priors for k in ('R', 'J_m', 'kq', 'Rb', 'b')):
        cfg.priors['R'] = (0.16, 0.04)
        cfg.default_prior = True
    for fx in a.fix or []:
        k, v = fx.split('=')
        cfg.fixed[k] = float(v)
    cfg.init = {}
    for it in a.init or []:
        k, v = it.split('=')
        cfg.init[k] = float(v)
    cfg.extra_free = [x for f in (a.free or []) for x in f.split(',') if x]
    if a.thermal:
        cfg.extra_free += ['Rth', 'Cth']
    for po in a.prop or []:
        k, v = po.split('=')
        val = parse_prop(v)
        if val is None:
            raise SystemExit('--prop %s: use off, on or a prop type number' % po)
        cfg.prop_override[log_short_name(k)] = val
    cfg.windows = {}
    for w in a.window or []:
        k, v = w.split('=')
        t0, t1 = (float(x) for x in v.split(':'))
        cfg.windows[log_short_name(k)] = (t0, t1)
    return cfg


def load_runs(paths, cfg, role, msg=None):
    logs, runs = [], []
    for p in paths:
        log = read_log(p, msg=msg)
        logs.append(log)
        segs = split_runs(log, window=cfg.windows.get(log['name']))
        log['runs'] = []
        for i, (t0, t1, ctx) in enumerate(segs):
            run = make_run(log, t0, t1, ctx, cfg, i, len(segs))
            pid, src, det = classify_prop(run, cfg)
            run.update(prop=pid, prop_source=src, prop_details=det, role=role)
            run['name'] = ('np' if pid == 0 else 'p') + log['name'] + run['suffix']
            log['runs'].append(run)
            if pid is None:
                print('WARNING: %s run %.1f-%.1f s: prop state unknown (%s); skipped' % (log['name'], t0, t1, src))
                continue
            runs.append(run)
    return logs, runs


def describe(logs):
    for log in logs:
        print('%s  (%s, sha256 %s...)  excitation %s, %d records' % (log['name'], log['path'], log['sha256'][:12],
                                                                      log['msg'], len(log['exc']['t'])))
        pr = log['params']
        print('  params: ' + ', '.join('%s=%g' % (k, pr[k]) for k in
                                       ('MOT_PWM_MIN', 'MOT_PWM_MAX', 'SERVO_BLH_POLES', 'SERVO_DSHOT_RATE',
                                        'SCHED_LOOP_RATE', 'SERVO1_FUNCTION', 'MOT_PWM_TYPE') if k in pr))
        for c in log['midc']:
            print('  MIDC at %.1f s: %s' % (c['t'], {k: v for k, v in c.items() if k not in ('mavpackettype', 'TimeUS', 't')}))
        print('  LOAD: %s, BPWR: %s' % ('yes' if log['load'] is not None else 'no', 'yes' if log['bpwr'] is not None else 'no'))
        for run in log['runs']:
            tests = sorted(set(int(x) for x in np.unique(run['test'])))
            print('  run %-8s %7.1f-%7.1f s  prop=%s (%s)  tests %s  voltage: %s' % (
                run.get('name', '?'), run['t0'], run['t1'], run['prop'], run['prop_source'], tests, run['vsrc']))
            if run['prop_details'].get('holds'):
                print('     steady holds rpm/(Kv*duty*V): ' + ', '.join('%d:%.3f' % (h['pwm'], h['ratio'])
                                                                       for h in run['prop_details']['holds']))


def run_fit(a):
    t_all = time.time()
    cfg = build_config(a)
    if not mm.HAVE_NUMBA:
        print('NOTE: numba not installed: pure-Python model with a %.0f us step (slower, ~1%% rpm error)' % cfg.step_us)
    logs, runs = load_runs(a.logs, cfg, 'fit', a.msg)
    vlogs, vruns = load_runs(a.validate or [], cfg, 'validate', a.msg)
    describe(logs + vlogs)
    if getattr(cfg, 'default_prior', False):
        print('NOTE: no absolute-scale prior given; using R = 0.16 +- 0.04 ohm (Hakea 2307 default). '
              'Measure the phase-to-phase resistance and pass --r-phase-phase.')
    off = [r for r in runs if r['prop'] == 0]
    on = [r for r in runs if r['prop'] and r['prop'] > 0]
    P = dict(cfg.fixed)
    P.update(cfg.init)
    infos = []
    props_done = False
    motor_names = ['g%d' % i for i in range(len(cfg.g_knots))] + ['R', 'T0', 'b', 'Rb', 'delay', 'J_m']
    motor_names += [k for k in cfg.extra_free if k in ('Isat', 'L', 'Rth', 'Cth') and k not in motor_names]
    motor_names = [k for k in motor_names if k not in cfg.fixed]
    if a.motor:
        prev = json.load(open(a.motor))
        P.update(prev['model']['motor_fit'])
        print('motor parameters taken from %s' % a.motor)
    initial_guess(runs + vruns, cfg, P)
    if not a.motor:
        if off:
            names = motor_names + [k + '@' + r['name'] for r in off for k in ('V0', 'V1')]
            c1 = Config(**cfg.__dict__)
            # battery+wiring resistance is barely visible at prop-off
            # currents: a weak prior keeps it sane, stage 2 refits it
            c1.priors = dict(cfg.priors)
            c1.priors.setdefault('Rb', (0.03, 0.05))
            P, info = fit_stage('stage 1: motor (prop off)', names, P, off, c1)
            infos.append(info)
        elif on:
            print('WARNING: no prop-off runs: motor and prop fitted together; only the total inertia is identifiable')
            for r in on:
                P['J_p@%d' % r['prop']] = 0.0
            names = motor_names + sorted({'kq@%d' % r['prop'] for r in on}) + \
                [k + '@' + r['name'] for r in on for k in ('V0', 'V1')]
            P, info = fit_stage('stage 1+2: motor and prop (prop on only)', names, P, on, cfg)
            infos.append(info)
            props_done = True
    if on and not props_done:
        names = sorted({'J_p@%d' % r['prop'] for r in on} | {'kq@%d' % r['prop'] for r in on})
        names += ['Rb'] if 'Rb' not in cfg.fixed else []
        names += [k for k in cfg.extra_free if k in ('Rth', 'Cth') and k not in names and k not in motor_names]
        names += [k + '@' + r['name'] for r in on for k in ('V0', 'V1')]
        P, info = fit_stage('stage 2: prop (motor fixed)', names, P, on, cfg)
        infos.append(info)
    if a.joint and off and on:
        names = motor_names + sorted({'J_p@%d' % r['prop'] for r in on} | {'kq@%d' % r['prop'] for r in on})
        names += [k + '@' + r['name'] for r in off + on for k in ('V0', 'V1')]
        P, info = fit_stage('stage 3: joint refinement', names, P, off + on, cfg)
        infos.append(info)
    # held-out logs: battery state only
    for r in vruns:
        P = fit_battery_only(P, r, cfg)
    # absolute scale from current / torque, if logged
    sims = {r['name']: sim_run(cfg, params_for(P, r), r) for r in runs}
    stand = thrust_stand_fit(P, runs, sims, cfg, a.prop_diameter)
    P, scale = resolve_scale(P, stand, cfg)
    if scale['method'] == 'measured':
        print('absolute scale from measurements: s = %.4f (sources: %s)' % (
            scale['s'], ', '.join(s_['source'] for s_ in scale['sources'])))
        sims = {r['name']: sim_run(cfg, params_for(P, r), r) for r in runs}
        stand = thrust_stand_fit(P, runs, sims, cfg, a.prop_diameter)
    # evaluation
    allruns = runs + vruns
    results = {}
    for r in allruns:
        sim = sims.get(r['name']) or sim_run(cfg, params_for(P, r), r)
        mt, _ = metrics(P, r, cfg, sim)
        results[r['name']] = dict(metrics=mt, steady=steady_table(r, sim), freq=frequency_response(r, sim), sim=sim)
        print('%-8s %s' % (r['name'], json.dumps({k: (round(v, 1) if isinstance(v, float) else
                                                     {kk: round(vv, 1) if isinstance(vv, float) else vv
                                                      for kk, vv in v.items()}) for k, v in mt.items()})))
    write_outputs(a, cfg, logs, vlogs, runs, vruns, P, infos, scale, stand, results)
    print('done in %.0f s' % (time.time() - t_all))


def write_outputs(a, cfg, logs, vlogs, runs, vruns, P, infos, scale, stand, results):
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    tag = a.tag or ''
    props = sorted({r['prop'] for r in runs + vruns if r['prop']})
    motor_keys = ['g%d' % i for i in range(len(cfg.g_knots))] + ['R', 'L', 'T0', 'b', 'J_m', 'delay', 'Rb']
    motor_keys += [k for k in ('Isat', 'Rth', 'Cth') if P.get(k)]  # only when in use
    motor = {k: P[k] for k in motor_keys if k in P}
    prop_d = {str(p): {'J_p': P.get('J_p@%d' % p), 'kq': P.get('kq@%d' % p)} for p in props}
    for p in props:
        if p in stand['props']:
            prop_d[str(p)]['thrust_stand'] = stand['props'][p]
        if a.prop_diameter:
            prop_d[str(p)]['Cq_from_model'] = P['kq@%d' % p] * 4 * np.pi ** 2 / (1.225 * a.prop_diameter ** 5)
    batt = {r['name']: dict(V0=P['V0@' + r['name']], V1=P['V1@' + r['name']], t0=r['t0'], log=r['log'],
                            voltage_source=r['vsrc']) for r in runs + vruns}
    # flat parameter file compatible with Hakea's scripts/sim/calibrate.py
    # (params_for / predict_blind.py): first prop type only
    compat = {k: v for k, v in motor.items()}
    if props:
        compat['J_p'] = P['J_p@%d' % props[0]]
        compat['kq'] = P['kq@%d' % props[0]]
    for name, b in batt.items():
        compat['V0_' + name] = b['V0']
        compat['V1_' + name] = b['V1']
    compat['_generated_by'] = 'motorid %s' % TOOL_VERSION
    # ESCSim SITL model per prop state
    escsim = {}
    v_nom = {}
    for r in runs + vruns:
        v_nom.setdefault(r['prop'], P['V0@' + r['name']])
    for pid in sorted(v_nom):
        key = 'prop_off' if pid == 0 else 'prop_%d' % pid
        escsim[key] = mm.escsim_model(motor, prop_d.get(str(pid)), round(v_nom[pid], 2), a.rds_on,
                                      int(cfg.esc.motor_poles), cfg.kv)
    gmid = float(np.mean([P['g%d' % i] for i in range(1, len(cfg.g_knots) - 1)]))
    sigmas = {}
    for k in sorted(P):
        s_ = sigma_text(infos, k, P[k])
        if s_:
            sigmas[k] = s_
    fw = None
    if a.firmware_hash:
        if os.path.exists(os.path.expanduser(a.firmware_hash)):
            fp = os.path.expanduser(a.firmware_hash)
            fw = dict(path=os.path.abspath(fp), sha256=sha256_file(fp), content=open(fp).read()[:4000])
        else:
            fw = dict(value=a.firmware_hash)
    metrics_all = {name: res['metrics'] for name, res in results.items()}
    fit_e = [m['fit_rpm_rms'] for m in metrics_all.values() if 'fit_rpm_rms' in m]
    hold_e = [m['holdout_rpm_rms'] for m in metrics_all.values() if 'holdout_rpm_rms' in m]
    model = {
        'tool': dict(name='motorid', version=TOOL_VERSION, date=datetime.datetime.now().isoformat(timespec='seconds'),
                     command=' '.join([os.path.basename(sys.argv[0])] + sys.argv[1:]),
                     numba=mm.HAVE_NUMBA, step_us=cfg.step_us),
        'inputs': dict(
            logs=[dict(path=l_['path'], sha256=l_['sha256'], excitation_message=l_['msg'], role=role,
                       runs=[dict(name=r.get('name'), t0=r['t0'], t1=r['t1'], prop=r['prop'],
                                  prop_source=r['prop_source']) for r in l_['runs']])
                  for role, ls in (('fit', logs), ('validate', vlogs)) for l_ in ls],
            eeprom=dict(path=os.path.abspath(a.eeprom), sha256=sha256_file(a.eeprom)) if a.eeprom else None,
            firmware=fw, reference=os.path.abspath(a.reference) if a.reference else None),
        'esc_settings': cfg.esc.to_dict(),
        'config': dict(channels=cfg.channels, fit_tests=cfg.fit_tests, holdout_tests=cfg.holdout_tests,
                       untagged_test=cfg.untagged_test, sig_rpm=cfg.sig_rpm, sig_v=cfg.sig_v, settle_s=cfg.settle_s,
                       coast_min_rpm=cfg.coast_min_rpm, fixed=cfg.fixed, extra_free=cfg.extra_free,
                       priors={k: list(v) for k, v in cfg.priors.items()}, joint=bool(a.joint),
                       g_knots=list(cfg.g_knots), kv=cfg.kv, rds_on=a.rds_on),
        'identifiability': dict(
            note=('rpm and battery voltage alone leave one scale factor s free: J, kq, b, T0, Cth, Isat -> s*x and '
                  'R, Rb, L, Rth -> x/s give identical rpm. Torque, current and heating depend on s.'),
            scale=scale),
        'model': dict(
            description='commutation-averaged AM32 + BLDC model, see motorid_model.py',
            kv_nominal=cfg.kv, poles=int(cfg.esc.motor_poles), g_knots_rpm=list(cfg.g_knots),
            kv_effective_hint=cfg.kv * gmid,
            motor_fit=motor, props=prop_d, battery=dict(Rb=P['Rb'], runs=batt), sigma=sigmas),
        'avgmodel_params': compat,
        'escsim': escsim,
        'escsim_notes': ('R_phase = (R - 2 rds_on)/2, inductance = L/2, inertia = J_m (+ J_p), load_k_omega2 = kq, '
                         'static_friction = T0, damping = b, battery voltage = fitted open-circuit V0 of the '
                         'first run in that prop state, resistance = Rb. kv is nominal; the averaged model\'s '
                         'effective gain g(rpm) (mean %.3f) has no SITL parameter.' % gmid),
        'thrust_stand': {str(k): v for k, v in stand['props'].items()} or None,
        'fit': dict(stages=infos, fit_rpm_rms=fit_e, holdout_rpm_rms=hold_e),
        'metrics': metrics_all,
        'steady': {name: res['steady'] for name, res in results.items()},
        'frequency_response': {name: res['freq'] for name, res in results.items()},
    }
    fn = os.path.join(out, 'motor_model%s.json' % tag)
    with open(fn, 'w') as f:
        json.dump(model, f, indent=1, default=_json_default)
    with open(os.path.join(out, 'avgmodel_params%s.json' % tag), 'w') as f:
        json.dump(compat, f, indent=1, default=_json_default)
    for key, m_ in escsim.items():
        with open(os.path.join(out, 'escsim_%s%s.json' % (key, tag)), 'w') as f:
            json.dump(m_, f, indent=1)
    print('wrote', fn)
    plots = []
    if not a.no_plots:
        for r in runs + vruns:
            res = results[r['name']]
            p1 = 'timeseries_%s%s.png' % (r['name'], tag)
            plot_run(r, res['sim'], res['freq'], os.path.join(out, p1))
            plots.append(p1)
            if res['freq']:
                p2 = 'freqresp_%s%s.png' % (r['name'], tag)
                plot_bode(r, res['freq'], os.path.join(out, p2))
                plots.append(p2)
        p3 = 'steady%s.png' % tag
        plot_steady({n: res['steady'] for n, res in results.items()}, os.path.join(out, p3))
        plots.append(p3)
    write_report(os.path.join(out, 'report%s.md' % tag), a, cfg, model, runs + vruns, results, P, plots, props)


def _json_default(o):
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, tuple):
        return list(o)
    raise TypeError(type(o))


def write_report(path, a, cfg, model, runs, results, P, plots, props):
    L = []
    w = L.append
    w('# Motor identification report\n')
    w('Generated by motorid %s on %s.  ' % (TOOL_VERSION, model['tool']['date']))
    w('Command: `%s`\n' % model['tool']['command'])
    w('## Inputs\n')
    w('| log | role | sha256 | excitation | run | window (s) | prop | how classified |')
    w('|---|---|---|---|---|---|---|---|')
    for lg in model['inputs']['logs']:
        for r in lg['runs']:
            w('| %s | %s | `%s...` | %s | %s | %.1f-%.1f | %s | %s |' % (
                os.path.basename(lg['path']), lg['role'], lg['sha256'][:12], lg['excitation_message'], r['name'],
                r['t0'], r['t1'], {0: 'off', None: '?'}.get(r['prop'], 'on (type %s)' % r['prop']), r['prop_source']))
    ee = model['inputs']['eeprom']
    w('\nESC settings: %s%s. Derived: min duty %d/2000, ramp (startup/low/high) %d/%d/%d per 20 kHz tick, '
      'low-rpm limit %d..%d k-erpm, comp_pwm %d, brake_on_zero_throttle %d.\n' % (
          cfg.esc.source, (' `%s` (sha256 `%s...`)' % (ee['path'], ee['sha256'][:12])) if ee else '',
          *(cfg.esc.derived()[k] for k in ('min_duty', 'ramp_startup', 'ramp_low', 'ramp_high', 'low_rpm_level',
                                            'high_rpm_level')), cfg.esc.comp_pwm, cfg.esc.brake_on_zero_throttle))
    if model['inputs']['firmware']:
        fw = model['inputs']['firmware']
        w('Firmware: %s\n' % (('`%s` (sha256 `%s...`)' % (fw['path'], fw['sha256'][:12])) if 'path' in fw else fw['value']))
    w('Measured channels used: %s. Never used unless asked: FC battery current, ESC telemetry voltage/current, '
      'IMU.\n' % ', '.join(sorted({r['vsrc'] for r in runs if r['vsrc']} | {'rpm'} |
                                  ({r['isrc'] for r in runs if r.get('isrc')}) |
                                  ({'LOAD'} if model['thrust_stand'] else set()))))
    w('## Identifiability\n')
    w('With only rpm and battery voltage, the model predicts *identical rpm* for any scale factor s applied as '
      'J, kq, b, T0 -> s x and R, Rb, L -> x / s (the motor current scales by s). Torque, current and heating do '
      'depend on s, so one absolute quantity must be known.\n')
    sc = model['identifiability']['scale']
    if sc['method'] == 'prior':
        pr = ', '.join('%s = %g +- %g' % (k, v[0], v[1]) for k, v in cfg.priors.items())
        w('**This fit fixes s with the prior(s): %s.** %s Absolute torque, current, inertia and loss numbers below '
          'are only as good as that prior; rpm predictions are not affected.\n' % (
              pr, '(This is the built-in default for Hakea\'s 2307; measure the phase-to-phase resistance and pass '
                  '`--r-phase-phase`.)' if getattr(cfg, 'default_prior', False) else ''))
    else:
        w('Scale fixed by measurements: s = %.4f (1-sigma %.1f%%), from %s.\n' % (
            sc['s'], 100 * sc['sigma_log'], ', '.join('%s (s=%.3f)' % (x['source'], np.exp(x['log_s'])) for x in sc['sources'])))
    w('## Parameters\n')
    sg = model['model']['sigma']
    desc = dict(R='loop resistance 2 R_phase + 2 Rds_on (ohm, 25 C)', L='loop inductance (H, fixed)',
                T0='Coulomb friction torque (N m)', b='viscous friction (N m s/rad)', J_m='rotor inertia (kg m^2)',
                delay='measurement chain delay (s)', Rb='battery + wiring resistance (ohm)',
                Isat='torque-constant roll-off current (A)', Rth='winding thermal resistance (K/W)',
                Cth='winding heat capacity (J/K)')
    w('| parameter | value | 1-sigma | meaning |')
    w('|---|---|---|---|')
    for k, v in model['model']['motor_fit'].items():
        if k.startswith('g'):
            continue
        w('| %s | %.4g | %s | %s |' % (k, v, sg.get(k, 'fixed' if k in cfg.fixed else '-'), desc.get(k, '')))
    gk = cfg.g_knots
    w('| g(rpm) | %s | | effective voltage gain at %s rpm |' % (
        ', '.join('%.3f' % P['g%d' % i] for i in range(len(gk))), '/'.join('%g' % x for x in gk)))
    for p in props:
        w('| J_p (prop %d) | %.4g | %s | prop + adapter inertia (kg m^2) |' % (p, P['J_p@%d' % p], sg.get('J_p@%d' % p, '-')))
        w('| kq (prop %d) | %.4g | %s | prop torque coefficient Q = kq w^2 (N m s^2) |' % (p, P['kq@%d' % p], sg.get('kq@%d' % p, '-')))
    w('\nKt = Ke = %.5f N m/A (nominal Kv %g); effective Kv ~ %.0f rpm/V (Kv x mean g). Uncertainties are from the '
      'fit Jacobian (relative, %% for log-space parameters) and are optimistic: residuals are strongly correlated '
      'in time.\n' % (60 / (2 * np.pi * cfg.kv), cfg.kv, model['model']['kv_effective_hint']))
    w('Battery (open-circuit voltage V0 + V1 t, per run): ' + '; '.join(
        '%s %.2f V %+.4f V/s' % (n, b['V0'], b['V1']) for n, b in model['model']['battery']['runs'].items()) + '\n')
    if model['thrust_stand']:
        w('### Thrust stand\n')
        for pid, d in model['thrust_stand'].items():
            w('- prop %s: %s' % (pid, ', '.join('%s = %.4g' % (k, v) for k, v in d.items() if isinstance(v, float))))
        w('')
    w('## Fit and hold-out errors (rpm)\n')
    w('Fit samples: motor commanded and settled >= %.1f s after a start, or coasting above %d rpm. '
      'Held out: test(s) %s of every run, and all of the validation logs (battery state refitted only).\n' % (
          cfg.settle_s, cfg.coast_min_rpm, ', '.join(str(x) for x in cfg.holdout_tests) or 'none'))
    w('| run | test | role | rpm rms | mean error | rms / mean rpm | samples |')
    w('|---|---|---|---|---|---|---|')
    for r in runs:
        mt = results[r['name']]['metrics']
        for k, v in sorted(mt.items()):
            if k.startswith('test'):
                w('| %s | %s | %s | %.0f | %+.0f | %.1f%% | %d |' % (r['name'], k[4:], v['role'], v['rpm_rms'],
                                                                    v['rpm_mean_err'], 100 * v['rpm_rel_rms'], v['n']))
        if 'volt_rms' in mt:
            w('| %s | voltage | | %.3f V rms | | | |' % (r['name'], mt['volt_rms']))
    fe, he = model['fit']['fit_rpm_rms'], model['fit']['holdout_rpm_rms']
    if fe:
        w('\nOverall: fit %.0f rpm rms, hold-out %s rpm rms.\n' % (
            np.sqrt(np.mean(np.square(fe))), ('%.0f' % np.sqrt(np.mean(np.square(he)))) if he else '-'))
    w('## Steady rpm vs throttle\n')
    w('Mean rpm over the end of every constant-throttle hold (test 1 = steps, 2 = coast-down holds).\n')
    w('| run | test | pwm | bench rpm | model rpm | error |')
    w('|---|---|---|---|---|---|')
    for r in runs:
        for x in results[r['name']]['steady']:
            if x['test'] in (0,):
                continue
            w('| %s | %d | %d | %.0f | %.0f | %+.1f%% |' % (r['name'], x['test'], x['pwm'], x['rpm_bench'],
                                                          x['rpm_model'], 100 * (x['rpm_model'] / x['rpm_bench'] - 1)))
    w('\n## Frequency response (rpm per us of pwm, phase incl. chain delay)\n')
    w('| run | test | f (Hz) | bench gain | model gain | bench phase | model phase |')
    w('|---|---|---|---|---|---|---|')
    for r in runs:
        for seg in results[r['name']]['freq']:
            a_ = np.array(seg['rows'])
            pick = [np.argmin(abs(a_[:, 0] - f)) for f in (1, 3, 10, 20, 35, 50)] if seg['kind'] == 'sweep' else range(len(a_))
            for i in sorted(set(pick)):
                w('| %s | %d | %.3g | %.1f | %.1f | %.0f | %.0f |' % (r['name'], seg['test'], *a_[i, [0, 1, 3, 2, 4]]))
    w('\n## Derived: speed and bandwidth vs throttle (model, steady state)\n')
    w('Rotation frequency = rpm/60 is the 1st motor harmonic (useful for notch planning). Bandwidth is the '
      'small-signal duty->speed -3 dB frequency without ramp limit or delay; tau = 1/(2 pi f3dB).\n')
    w('| prop | V | pwm | rpm | rotation Hz | f3dB (Hz) | tau (ms) |')
    w('|---|---|---|---|---|---|---|')
    for pid in sorted({r['prop'] for r in runs}):
        v = np.median([P['V0@' + r['name']] for r in runs if r['prop'] == pid])
        for pwm in (1200, 1300, 1400, 1500, 1600, 1700, 1800):
            rpm, f3, tau = bandwidth(P, cfg, pid, pwm, v)
            w('| %s | %.1f | %d | %.0f | %.0f | %.1f | %.1f |' % ('off' if pid == 0 else pid, v, pwm, rpm, rpm / 60, f3, 1e3 * tau))
    if a.reference:
        ref = json.load(open(a.reference))
        w('\n## Comparison with reference `%s`\n' % a.reference)
        w('| parameter | this fit | reference | ratio |')
        w('|---|---|---|---|')
        comp = model['avgmodel_params']
        for k in sorted(set(comp) & set(ref)):
            if k.startswith('_') or not isinstance(ref[k], (int, float)):
                continue
            rv, cv = ref[k], comp[k]
            w('| %s | %.4g | %.4g | %s |' % (k, cv, rv, ('%.3f' % (cv / rv)) if rv else '-'))
    w('\n## Files\n')
    w('- `%s`: full model + provenance; `avgmodel_params*.json`: flat parameters for Hakea\'s scripts/sim '
      '(calibrate.py / predict_blind.py layout); `escsim_*.json`: ESCSim SITL model files.' %
      os.path.basename(path).replace('report', 'motor_model').replace('.md', '.json'))
    for p in plots:
        w('- ![%s](%s)' % (p, p))
    with open(path, 'w') as f:
        f.write('\n'.join(L) + '\n')
    print('wrote', path)


def make_parser():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog='See README.md for the workflow and motorid_model.py for the model.')
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('info', 'fit'):
        p = sub.add_parser(name)
        p.add_argument('logs', nargs='+', help='dataflash .BIN logs used for fitting')
        p.add_argument('--validate', action='append', help='log used only for validation (repeatable)')
        p.add_argument('--prop', action='append', help='LOG=off|on|N: prop state override (N = prop type id)')
        p.add_argument('--window', action='append', help='LOG=t0:t1: only use this time window of a log (s)')
        p.add_argument('--msg', help='excitation message name (default: first of %s found)' % '/'.join(EXCITATION_MSGS))
        p.add_argument('--eeprom', help='AM32 EEPROM image (.bin, 184/192 bytes) with the ESC settings')
        p.add_argument('--esc', action='append', help='override an ESC setting, e.g. --esc max_ramp=160')
        p.add_argument('--kv', type=float, help='nominal Kv (default: from the EEPROM, else 1300)')
        p.add_argument('--channels', default='auto', help='comma list of %s, or auto (default: %s when present)' % (
            ','.join(CHANNELS), ','.join(AUTO_CHANNELS)))
        p.add_argument('--untagged-test', type=int, default=4, help='test id for records without a test field (SWP)')
        p.add_argument('--verbose', type=int, default=1)
    p = sub.choices['fit']
    p.add_argument('--out', default='.', help='output directory')
    p.add_argument('--tag', default='', help='suffix for output file names')
    p.add_argument('--r-phase-phase', type=float, help='measured phase-to-phase resistance (ohm): prior on R = '
                   'this + 2 rds_on, 10%% sigma')
    p.add_argument('--rds-on', type=float, default=0.002, help='ESC FET on-resistance (ohm) for the R mapping')
    p.add_argument('--prior', action='append', help='NAME=value,sigma (e.g. R=0.16,0.04). Default R=0.16,0.04')
    p.add_argument('--fix', action='append', help='NAME=value: hold a parameter fixed (default L=30e-6)')
    p.add_argument('--init', action='append', help='NAME=value: starting value for a fitted parameter')
    p.add_argument('--free', action='append', help='extra parameters to fit: Isat, Rth, Cth, L')
    p.add_argument('--thermal', action='store_true', help='fit the winding thermal model (Rth, Cth)')
    p.add_argument('--joint', action='store_true', help='after the sequential fit, refine everything jointly')
    p.add_argument('--motor', help='motor_model.json from an earlier fit: reuse its motor, fit only props')
    p.add_argument('--holdout-tests', default='4', help='test ids held out for validation (default 4, "" = none)')
    p.add_argument('--fit-tests', help='only fit these test ids (default: all non-zero not held out)')
    p.add_argument('--g-knots', help='rpm knots of the voltage-gain table (default %s)' % ','.join('%g' % x for x in mm.G_KNOTS))
    p.add_argument('--step-us', type=float, help='integration step (default 50 with numba, 1000 without)')
    p.add_argument('--diff-step', type=float, default=1e-2, help='relative Jacobian finite-difference step '
                   '(large enough to step over the ESC logic\'s small discontinuities)')
    p.add_argument('--max-nfev', type=int, default=300, help='function evaluation budget per stage')
    p.add_argument('--prop-diameter', type=float, help='prop diameter (m) for Ct/Cq/Cp')
    p.add_argument('--firmware-hash', help='file (e.g. SHA256SUMS) or string recorded as firmware provenance')
    p.add_argument('--reference', help='calibrate.py-style params JSON to compare against')
    p.add_argument('--no-plots', action='store_true')
    return ap


def main(argv=None):
    a = make_parser().parse_args(argv)
    if a.cmd == 'info':
        cfg = Config()
        if a.eeprom:
            cfg.esc = mm.Am32Settings.from_eeprom(open(a.eeprom, 'rb').read())
        cfg.kv = float(a.kv or cfg.esc.motor_kv)
        cfg.windows = {}
        cfg.untagged_test = a.untagged_test
        for po in a.prop or []:
            k, v = po.split('=')
            cfg.prop_override[log_short_name(k)] = parse_prop(v)
        for w in a.window or []:
            k, v = w.split('=')
            cfg.windows[log_short_name(k)] = tuple(float(x) for x in v.split(':'))
        logs, _ = load_runs(a.logs + (a.validate or []), cfg, 'info', a.msg)
        describe(logs)
        return
    run_fit(a)


if __name__ == '__main__':
    main()
