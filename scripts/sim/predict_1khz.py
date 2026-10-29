#!/usr/bin/env python3
"""Assemble the pre-registered 0-1000 Hz PROFILE_MODE predictions from the
SITL grids (profile_grid.py), the averaged/analytic model (profile_avg.py)
and the exact-programme SITL run (program_run.py), and draw the graphs.

Writes, in notes/sim/predict-1khz/:
  predictions.csv            one row per (source, f, amplitude, ramp, pwm)
  predictions_programme.csv  Fred's v1 programme, one row per step
  torque_vs_freq.png, phase_lag_vs_freq.png, copper_loss_vs_freq.png,
  desync_map.png, rpm_ripple_vs_freq.png
"""
import csv
import json
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

D = os.path.expanduser('~/vehicles/hakea/notes/sim/predict-1khz')
RAW = os.path.join(D, 'raw')
LOOP_HZ = 1e6 / 51.0
PWM_HZ = 24000.0
RPM0 = 14800.0
COMM_HZ = RPM0 / 60 * 7 * 6         # commutation steps per second at base


def load(fn):
    return list(csv.DictReader(open(os.path.join(RAW, fn))))


def regime(f):
    """which physics dominates the torque response at this frequency"""
    notes = []
    if f < 15:
        notes.append('mechanical: rpm follows, back-EMF cancels drive (torque << high-f value)')
    elif f < 400:
        notes.append('inertia-dominated: torque ~ flat, rpm ripple ~ 1/f')
    else:
        notes.append('electrical L/R lag: torque falls, phase lag grows (corner ~700 Hz for R 0.16 ohm, L 36 uH line)')
    tpc, ppc, cpc = LOOP_HZ / f, PWM_HZ / f, COMM_HZ / f
    if tpc < 40 or ppc < 40 or cpc < 20:
        notes.append('resolution: %.0f control ticks, %.0f PWM periods, %.0f commutations per cycle' % (tpc, ppc, cpc))
    return '; '.join(notes), tpc, ppc, cpc


def main():
    rows = []
    sources = [('sitl_pwm24_ct1.csv', 'SITL nominal', 24), ('sitl_pwm24_ct0.8.csv', 'SITL pessimistic-demag', 24),
               ('sitl_pwm48_ct1.csv', 'SITL nominal', 48),
               ('sitl_pwm24_ct1_big.csv', 'SITL nominal', 24), ('sitl_pwm24_ct0.8_big.csv', 'SITL pessimistic-demag', 24)]
    for fn, label, pwm in sources:
        if not os.path.exists(os.path.join(RAW, fn)):
            continue
        g = load(fn)
        gb = load(fn.replace('_big', '')) if '_big' in fn else g
        base = {v: [r for r in gb if r['variant'] == v and float(r['a_pct']) == 0] for v in set(r['variant'] for r in g)}
        for r in g:
            if float(r['a_pct']) == 0 or 'error' in r and r['error']:
                continue
            b = base[r['variant']][0] if base[r['variant']] else None
            fa = float(r['f_actual'])
            note, tpc, ppc, cpc = regime(fa)
            rows.append(dict(
                source=label, firmware='Fred PROFILE v1 (after ramp, clamp 180)' if r['variant'] == 'fred'
                else 'study: sine before stock ramp 16', ramp='bypass' if r['variant'] == 'fred' else 'stock 16',
                pwm_khz=pwm, f_nominal=float(r['f']), f_actual=fa, amp_pct=float(r['a_pct']),
                amp_duty=float(r['a_pct']) * 20,
                delivered_duty_amp=float(r['duty_amp']), delivered_ratio=float(r['duty_amp']) / (float(r['a_pct']) * 20),
                current_swing_A=float(r['idc_amp']), torque_swing_Nm=float(r['tau_amp']),
                torque_lag_deg=float(r['tau_lag_deg']), rpm_ripple_amp=float(r['rpm_amp']),
                rpm_ripple_pp=float(r['rpm_pp']), rpm_mean=float(r['rpm_mean']),
                desync=int(r['desync']), slip=float(r['slip']),
                copper_W=float(r['pcu']), copper_ratio=float(r['pcu']) / float(b['pcu']) if b else '',
                regime=note, ticks_per_cycle=round(tpc, 1), pwm_periods_per_cycle=round(ppc, 1),
                commutations_per_cycle=round(cpc, 1)))
    avg = load('avg_L36.csv')
    for r in avg:
        fa = float(r['f_actual'])
        note, tpc, ppc, cpc = regime(fa)
        for src, pre in (('averaged model (nonlinear)', 'avg_'), ('analytic linear', 'an_')):
            rows.append(dict(
                source=src, firmware='Fred PROFILE v1 (after ramp, clamp 180)', ramp='bypass', pwm_khz=24,
                f_nominal=float(r['f_nominal']), f_actual=fa, amp_pct=float(r['a_pct']), amp_duty=float(r['amp']),
                delivered_duty_amp=float(r['avg_duty_amp']) if pre == 'avg_' else float(r['amp']),
                delivered_ratio=float(r['avg_duty_amp']) / float(r['amp']) if pre == 'avg_' else 1.0,
                current_swing_A=float(r[pre + 'idc_amp']), torque_swing_Nm=float(r[pre + 'tau_amp']),
                torque_lag_deg=float(r[pre + 'tau_lag_deg']), rpm_ripple_amp=float(r[pre + 'rpm_amp']),
                rpm_ripple_pp=2 * float(r[pre + 'rpm_amp']), rpm_mean=float(r['avg_rpm_mean']) if pre == 'avg_' else float(r['rpm0']),
                desync='', slip='',
                copper_W=float(r['avg_pcu']) if pre == 'avg_' else float(r['an_pcu']),
                copper_ratio=float(r['avg_pcu_ratio']) if pre == 'avg_' else float(r['an_pcu']) / float(r['pcu0']),
                regime=note, ticks_per_cycle=round(tpc, 1), pwm_periods_per_cycle=round(ppc, 1),
                commutations_per_cycle=round(cpc, 1)))
    keys = list(rows[0].keys())
    with open(os.path.join(D, 'predictions.csv'), 'w', newline='') as fo:
        w = csv.DictWriter(fo, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 5) if isinstance(v, float) else v) for k, v in r.items()})

    # programme table
    prog = []
    for fn, label in (('programme_v1_ct1.csv', 'SITL nominal'), ('programme_v1_ct0.8.csv', 'SITL pessimistic-demag')):
        if not os.path.exists(os.path.join(RAW, fn)):
            continue
        g = load(fn)
        settle = [r for r in g if r['step'] in ('2', '4', '6')]
        p0 = np.mean([float(r['pcu']) for r in settle])
        for r in g:
            prog.append(dict(source=label, step=int(r['step']), freq_nominal=float(r['freq_nominal']),
                             freq_actual=float(r['freq_actual']), amp_duty=float(r['amp']),
                             duration_s=float(r['duration']), rpm_mean=float(r['rpm_mean']),
                             delivered_duty_amp=float(r.get('duty_amp') or 0), duty_min=float(r['duty_min']),
                             duty_max=float(r['duty_max']), torque_swing_Nm=float(r.get('tau_amp') or 0),
                             torque_lag_deg=float(r.get('tau_lag_deg') or 0), current_swing_A=float(r.get('idc_amp') or 0),
                             rpm_ripple_amp=float(r.get('rpm_amp') or 0), rpm_pp=float(r['rpm_pp']),
                             desync=int(r['desync']), slip=float(r['slip']), copper_W=float(r['pcu']),
                             copper_ratio=float(r['pcu']) / p0))
    with open(os.path.join(D, 'predictions_programme.csv'), 'w', newline='') as fo:
        w = csv.DictWriter(fo, fieldnames=list(prog[0].keys()))
        w.writeheader()
        for r in prog:
            w.writerow({k: (round(v, 5) if isinstance(v, float) else v) for k, v in r.items()})

    # ---- graphs
    def sel(src, firm=None, pwm=24, amp=None):
        out = [r for r in rows if r['source'] == src and r['pwm_khz'] == pwm and (amp is None or r['amp_pct'] == amp)
               and (firm is None or r['ramp'] == firm)]
        return sorted(out, key=lambda r: r['f_actual'])

    colors = {5: 'C0', 10: 'C1', 20: 'C2', 30: 'C3', 40: 'C4', 50: 'C5'}
    fig, ax = plt.subplots(figsize=(10, 6.5))
    for A in (5, 10, 20, 30):
        r = sel('SITL nominal', 'bypass', 24, A)
        ax.loglog([x['f_actual'] for x in r], [x['torque_swing_Nm'] for x in r], 'o-', color=colors[A], label='SITL, PROFILE v1 (ramp bypassed), +-%d %%' % A)
        r = sel('SITL nominal', 'stock 16', 24, A)
        ax.loglog([x['f_actual'] for x in r], [x['torque_swing_Nm'] for x in r], 's--', color=colors[A], alpha=.6, label='SITL, sine before stock ramp 16, +-%d %%' % A)
        r = sel('averaged model (nonlinear)', 'bypass', 24, A)
        ax.loglog([x['f_actual'] for x in r], [x['torque_swing_Nm'] for x in r], ':', color=colors[A], lw=2, label='averaged model, +-%d %%' % A)
    r = sel('SITL nominal', 'bypass', 48, 10)
    ax.loglog([x['f_actual'] for x in r], [x['torque_swing_Nm'] for x in r], 'x-.', color='k', label='SITL, PWM 48 kHz, +-10 %')
    ax.axhline(0.056, color='gray', lw=.8)
    ax.text(1.1, 0.059, 'mean hover torque 0.056 N m', color='gray', fontsize=8)
    ax.axvspan(500, 1000, color='orange', alpha=.08)
    ax.text(520, 0.0035, 'L/R lag region\n(corner ~700 Hz)', fontsize=8)
    ax.set_xlabel('modulation frequency (Hz, actual)'); ax.set_ylabel('torque swing, fundamental amplitude (N m)')
    ax.set_title('Predicted torque swing, base 1500 us, 24 V, 5" prop rig')
    ax.grid(True, which='both', alpha=.3); ax.legend(fontsize=7, ncol=2); ax.set_ylim(3e-3, 0.5)
    plt.tight_layout(); plt.savefig(os.path.join(D, 'torque_vs_freq.png'), dpi=90); plt.close()

    fig, ax = plt.subplots(figsize=(10, 5.5))
    for src, st in (('SITL nominal', 'o-'), ('averaged model (nonlinear)', ':'), ('analytic linear', '--')):
        for A in (10, 30):
            r = sel(src, 'bypass', 24, A)
            if r:
                ax.semilogx([x['f_actual'] for x in r], [x['torque_lag_deg'] for x in r], st, color=colors[A], label='%s, +-%d %%' % (src, A))
    ax.set_xlabel('Hz'); ax.set_ylabel('torque lag behind delivered duty (deg)'); ax.grid(True, which='both', alpha=.3)
    ax.set_title('Phase lag of motor torque vs commanded (delivered) duty; negative = torque leads (low f, back-EMF)')
    ax.legend(fontsize=8); plt.tight_layout(); plt.savefig(os.path.join(D, 'phase_lag_vs_freq.png'), dpi=90); plt.close()

    fig, ax = plt.subplots(figsize=(10, 5.5))
    for A in (5, 10, 20, 30):
        r = sel('SITL nominal', 'bypass', 24, A)
        ax.semilogx([x['f_actual'] for x in r], [x['copper_W'] for x in r], 'o-', color=colors[A], label='SITL PROFILE v1 +-%d %%' % A)
        r = sel('analytic linear', 'bypass', 24, A)
        ax.semilogx([x['f_actual'] for x in r], [x['copper_W'] for x in r], ':', color=colors[A], label='analytic 1+1/2(dI/I0)^2, +-%d %%' % A)
    b = [x for x in load('sitl_pwm24_ct1.csv') if x['variant'] == 'fred' and float(x['a_pct']) == 0]
    if b:
        ax.axhline(float(b[0]['pcu']), color='gray', lw=.8)
        ax.text(1.1, float(b[0]['pcu']) * 1.05, 'no pulsing', color='gray', fontsize=8)
    ax.set_xlabel('Hz'); ax.set_ylabel('motor copper loss (W, R at 25 C; x1.3 at 100 C)')
    ax.set_title('Copper loss vs modulation frequency (scales ~1/R; assumed R 0.16 ohm, L 18 uH/phase)')
    ax.grid(True, which='both', alpha=.3); ax.legend(fontsize=8)
    plt.tight_layout(); plt.savefig(os.path.join(D, 'copper_loss_vs_freq.png'), dpi=90); plt.close()

    fig, ax = plt.subplots(figsize=(10, 5.5))
    for A in (5, 10, 20, 30):
        r = sel('SITL nominal', 'bypass', 24, A)
        ax.loglog([x['f_actual'] for x in r], [x['rpm_ripple_amp'] for x in r], 'o-', color=colors[A], label='SITL +-%d %%' % A)
        r = sel('averaged model (nonlinear)', 'bypass', 24, A)
        ax.loglog([x['f_actual'] for x in r], [x['rpm_ripple_amp'] for x in r], ':', color=colors[A])
    ax.set_xlabel('Hz'); ax.set_ylabel('rpm ripple, fundamental amplitude'); ax.grid(True, which='both', alpha=.3)
    ax.set_title('RPM ripple (dotted: averaged model)'); ax.legend(fontsize=8)
    plt.tight_layout(); plt.savefig(os.path.join(D, 'rpm_ripple_vs_freq.png'), dpi=90); plt.close()

    fig, axs = plt.subplots(1, 3, figsize=(15, 4.8))
    panels = [('SITL nominal', 'bypass', 'PROFILE v1, nominal demag'),
              ('SITL pessimistic-demag', 'bypass', 'PROFILE v1, pessimistic demag'),
              ('SITL pessimistic-demag', 'stock 16', 'sine before stock ramp, pessimistic')]
    for ax, (src, firm, title) in zip(axs, panels):
        r = sel(src, firm, 24)
        fs = sorted(set(x['f_nominal'] for x in r))
        As = sorted(set(x['amp_pct'] for x in r))
        M = np.full((len(As), len(fs)), np.nan)
        for x in r:
            M[As.index(x['amp_pct']), fs.index(x['f_nominal'])] = x['desync'] + 100 * x['slip']
        im = ax.imshow(M, origin='lower', aspect='auto', cmap='Reds', vmin=0, vmax=5)
        ax.set_xticks(range(len(fs))); ax.set_xticklabels([str(int(f)) for f in fs], rotation=90, fontsize=7)
        ax.set_yticks(range(len(As))); ax.set_yticklabels(['+-%d %%' % a for a in As])
        for i in range(len(As)):
            for j in range(len(fs)):
                if not np.isnan(M[i, j]):
                    ax.text(j, i, '%d' % M[i, j], ha='center', va='center', fontsize=7)
        ax.set_title(title, fontsize=9); ax.set_xlabel('Hz (nominal)')
    fig.suptitle('Desync map (firmware desync count + % commutation slip), base 1500 us, 24 kHz PWM')
    plt.tight_layout(); plt.savefig(os.path.join(D, 'desync_map.png'), dpi=90); plt.close()
    print('wrote', len(rows), 'rows,', len(prog), 'programme rows')


if __name__ == '__main__':
    main()
