#!/usr/bin/env python3
"""Replay a bench log's throttle commands through the AM32 SITL and
compare the simulated rpm with the measured rpm.

The bench pwm (as logged by the Lua script) is converted to DShot the
way ArduPilot does it and sent as bidirectional DShot300 at 1200
frames/s (SCHED_LOOP_RATE 400 x SERVO_DSHOT_RATE 3), after 3 s of zero
throttle for AM32 arming. The SITL rpm is compared with the bench rpm
delayed by the measured chain delay (Lua -> loop -> DShot -> ESC ->
telemetry -> Lua), which is a fit parameter of the averaged model.

usage:
  sitl_replay.py --model models/hakea_sitl.json --dataset np33 [--elf X] [--tag T]
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calibrate as cal  # noqa: E402
import sitl_run as sr  # noqa: E402

ARM_S = 3.0


def replay(dataset, model, elf=sr.DEFAULT_ELF, frame_hz=1200.0, bidir=True, ee=None, t_range=None):
    d = cal.run_data(dataset)
    t0, t1 = (d['t0'], d['t1']) if t_range is None else t_range
    m = (d['tc'] >= t0) & (d['tc'] <= t1)
    tc = d['tc'][m] - t0
    ds = sr.pwm_to_dshot(d['pwm'][m])
    st = sr.Stim(period_ns=int(round(1e9 / frame_hz)), bidir=bidir)
    st.const(0, ARM_S)
    st.samples(np.concatenate([tc, [t1 - t0]]), np.concatenate([ds, [ds[-1]]]))
    lg = sr.run(st, model, elf=elf, eeprom_overrides=ee)
    lg['t_bench'] = lg['t'] - ARM_S + t0
    return d, lg


def compare(d, lg, delay, name, tag, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    tb = lg['t_bench'] + delay
    use = d['use'] & (d['ts'] < tb[-1])
    rpm_s = np.interp(d['ts'][use], tb, lg['rpm'])
    e = rpm_s - d['rpm'][use]
    res = dict(rpm_rms=float(np.sqrt(np.mean(e ** 2))), rpm_mean_err=float(np.mean(e)),
               desync=int(lg['desync'].max()), end=lg['_end'])
    # battery: SITL vbus 100 ms averages vs BAT.Volt (battery Voc is a
    # model constant in the SITL, so only the sag pattern is comparable)
    fig, ax = plt.subplots(3, 1, figsize=(13, 10))
    ax[0].plot(d['ts'], d['rpm'], lw=.3, label='bench')
    ax[0].plot(tb, lg['rpm'], lw=.3, alpha=.7, label='SITL')
    ax[0].legend(); ax[0].set_title('%s SITL replay: rpm rms err %.0f, desyncs %d' % (name, res['rpm_rms'], res['desync']))
    segs = cal.sweep_segments(d)
    bodes = []
    for test, lo, hi in segs:
        mm = (d['ts'] >= lo) & (d['ts'] <= hi)
        mc = (d['tc'] >= lo - 0.5) & (d['tc'] <= hi)
        hb = cal.sweep_response(d['ts'][mm], d['rpm'][mm], d['f'][mm], d['tc'][mc], d['pwm'][mc])
        ms = (tb >= lo) & (tb <= hi)
        fs = np.interp(tb[ms], d['ts'][mm], d['f'][mm])
        hs = cal.sweep_response(tb[ms], lg['rpm'][ms], fs, d['tc'][mc], d['pwm'][mc])
        bodes.append((test, hb, hs))
    lo, hi = segs[-1][2] - 0.2, segs[-1][2] - 0.05
    mm = (d['ts'] > lo) & (d['ts'] < hi)
    ms = (tb > lo) & (tb < hi)
    ax[1].plot(d['ts'][mm], d['rpm'][mm], '.-', ms=2, label='bench')
    ax[1].plot(tb[ms], lg['rpm'][ms], label='SITL'); ax[1].legend()
    for i, (test, hb, hs) in enumerate(bodes):
        ax[2].loglog(hb[:, 0], hb[:, 1], 'o-', color='C%d' % (2 * i), label='bench test %d' % test)
        ax[2].loglog(hs[:, 0], hs[:, 1], 's--', color='C%d' % (2 * i + 1), label='SITL test %d' % test)
    ax[2].set_xlabel('Hz'); ax[2].set_ylabel('rpm per us'); ax[2].legend(); ax[2].grid(True, which='both', alpha=.3)
    plt.tight_layout(); plt.savefig(os.path.join(out, 'sitl_replay_%s%s.png' % (name, tag)), dpi=75); plt.close()
    rows = []
    for test, hb, hs in bodes:
        for row in hb:
            j = np.argmin(abs(hs[:, 0] - row[0]))
            rows.append([test, row[0], row[1], row[2], row[3], row[4], hs[j, 1], hs[j, 2], hs[j, 3], hs[j, 4]])
    np.savetxt(os.path.join(out, 'sitl_bode_%s%s.csv' % (name, tag)), np.array(rows), delimiter=',',
               header='test,f_hz,bench_gain,bench_phase_deg,bench_rpm_pp,bench_rpm_mean,'
                      'sitl_gain,sitl_phase_deg,sitl_rpm_pp,sitl_rpm_mean', fmt='%.5g', comments='')
    if name.startswith('np'):
        steps = []
        pw_s = np.interp(d['ts'], d['tc'], d['pwm'])
        for pwm in range(1100, 1900, 100):
            k = (d['test'] == 1) & (pw_s == pwm)
            if not k.any():
                continue
            te = d['ts'][k][-1]
            mb = k & (d['ts'] > te - 1.2)
            ms = (tb > te - 1.2) & (tb < te)
            steps.append((pwm, float(d['rpm'][mb].mean()), float(lg['rpm'][ms].mean()),
                          float(lg['ibus'][ms].mean()), float(lg['irms'][ms].mean())))
        res['steps'] = steps
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--dataset', default='np33')
    ap.add_argument('--elf', default=sr.DEFAULT_ELF)
    ap.add_argument('--delay', type=float, default=0.005)
    ap.add_argument('--tag', default='')
    ap.add_argument('--save-log', action='store_true')
    a = ap.parse_args()
    out = os.path.join(cal.HOME, 'notes/sim/sitl_calib')
    os.makedirs(out, exist_ok=True)
    d, lg = replay(a.dataset, a.model, elf=a.elf)
    res = compare(d, lg, a.delay, a.dataset, a.tag, out)
    res['model'] = json.load(open(a.model))
    res['elf'] = a.elf
    json.dump(res, open(os.path.join(out, 'sitl_replay_%s%s.json' % (a.dataset, a.tag)), 'w'), indent=1)
    if a.save_log:
        k = slice(None, None, 10)
        np.savetxt(os.path.join(out, 'sitl_log_%s%s.csv.gz' % (a.dataset, a.tag)),
                   np.column_stack([lg['t_bench'][k], lg['rpm'][k], lg['vbus'][k], lg['ibus'][k], lg['irms'][k], lg['duty'][k], lg['desync'][k]]),
                   delimiter=',', header='t_bench,rpm,vbus,ibus,irms,duty,desync', fmt='%.5g', comments='')
    print(json.dumps({k: v for k, v in res.items() if k != 'model'}, indent=1))


if __name__ == '__main__':
    main()
