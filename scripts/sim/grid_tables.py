#!/usr/bin/env python3
"""Pivot a grid.py CSV into markdown tables (one per metric)."""
import csv
import sys
from collections import defaultdict

fn = sys.argv[1]
metrics = sys.argv[2].split(',') if len(sys.argv) > 2 else [
    'desync', 'slip', 'rpm_amp1', 'duty_ratio', 'tau_amp1', 'rpm_mean', 'irms', 'pcu']
rows = list(csv.DictReader(open(fn)))
for r in rows:
    try:
        r['duty_ratio'] = '%.2f' % (float(r['duty_amp1']) / float(r['duty_cmd_amp1'])) if float(r['a_pct']) > 0 else '-'
    except (KeyError, ValueError, ZeroDivisionError):
        r['duty_ratio'] = '-'
variants = sorted(set(r['variant'] for r in rows), key=lambda v: int(v))
amps = sorted(set(float(r['a_pct']) for r in rows if float(r['a_pct']) > 0))
freqs = sorted(set(float(r['f']) for r in rows))
for m in metrics:
    print('\n### %s\n' % m)
    print('| ramp | f Hz | ' + ' | '.join('+-%g %%' % a for a in amps) + ' |')
    print('|---|---|' + '---|' * len(amps))
    for v in variants:
        base = [r for r in rows if r['variant'] == v and float(r['a_pct']) == 0]
        for f in freqs:
            cells = []
            for a in amps:
                x = [r for r in rows if r['variant'] == v and float(r['f']) == f and float(r['a_pct']) == a]
                if not x:
                    cells.append('')
                    continue
                val = x[0].get(m, '')
                try:
                    fv = float(val)
                    val = ('%.0f' % fv) if abs(fv) >= 100 else ('%.3g' % fv)
                except ValueError:
                    pass
                cells.append(val)
            print('| %s | %g | %s |' % (v, f, ' | '.join(cells)))
        if base and m in base[0]:
            print('| %s | base (A=0) | %s |' % (v, base[0][m]))
