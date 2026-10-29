#!/usr/bin/env python3
"""Turn predict_blind.py output into the pre-registered predictions.csv
(one row per quantity with a 90 % range). The range combines the battery
voltage spread (simulated at V0 +- dV) with the model error factors
below, which come from the calibration/holdout residuals (see
notes/sim/am32-ramp-study.md, "Blind validation")."""
import csv
import json
import sys

src, out = sys.argv[1], sys.argv[2]
v = json.load(open(src))['values']

# multiplicative model-error factors (lo, hi) per quantity type / frequency.
# Sine amplitudes: the model over-predicts the rpm swing of large throttle
# swings at >= 10 Hz (bench big sweep holdout: bench/model 0.6-0.85), so
# the ranges are skewed low.
AMP = {'C_5Hz': (0.88, 1.08), 'C_15Hz': (0.75, 1.10), 'C_35Hz': (0.60, 1.10),
       'C_50Hz': (0.55, 1.15), 'D_25Hz': (0.80, 1.15)}
LAG = {'C_5Hz': 8, 'C_15Hz': 12, 'C_35Hz': 20, 'C_50Hz': 25, 'D_25Hz': 15}
NOISE_PP = 300.0  # rpm: telemetry noise adds to a measured max-min

rows = []
for q, x in v.items():
    nom, lo, hi = x['nom'], min(x['lo'], x['hi']), max(x['lo'], x['hi'])
    note = ''
    if q.startswith('A_'):
        k = 0.05 if '1150' in q else 0.03
        lo, hi = lo * (1 - k), hi * (1 + k)
        unit = 'rpm'
    elif q == 'B_rpm_cut':
        lo, hi, unit = lo * 0.97, hi * 1.03, 'rpm'
    elif q.startswith('B_rpm_t'):
        lo, hi, unit = 0.0, 1500.0, 'rpm'
        note = 'motor braked to a stop within ~0.05 s; AM32 then reports 0 rpm'
    elif q == 'B_t_half':
        lo, hi, unit = 0.008, 0.045, 's'
        note = 'includes ~2.3 ms Lua sampling'
    else:
        seg = q.rsplit('_', 2)[0] if not q.endswith('rpm_mean') else q[:-len('_rpm_mean')]
        seg = '_'.join(q.split('_')[:2])
        kind = q[len(seg) + 1:]
        if kind == 'rpm_mean':
            lo, hi, unit = lo * 0.96, hi * 1.04, 'rpm'
        elif kind in ('amp1', 'gain'):
            f0, f1 = AMP[seg]
            lo, hi = lo * f0, hi * f1
            unit = 'rpm' if kind == 'amp1' else 'rpm/us'
            note = 'model known to over-predict large swings at high f'
        elif kind == 'pp':
            f0, f1 = AMP[seg]
            nom = nom + NOISE_PP
            lo, hi = lo * f0, hi * f1 + 2 * NOISE_PP
            unit = 'rpm'
            note = 'max-min per cycle incl. ~300 rpm telemetry noise'
        elif kind == 'lag_deg':
            lo, hi, unit = nom - LAG[seg], nom + LAG[seg], 'deg'
            note = 'includes the fitted 5.0 ms measurement-chain delay'
    rows.append(dict(quantity=q, predicted=round(nom, 4), plus_minus=round((hi - lo) / 2, 4),
                     lo=round(lo, 4), hi=round(hi, 4), unit=unit, note=note))
rows.append(dict(quantity='desync_events', predicted=0, plus_minus=0, lo=0, hi=0, unit='count',
                 note='no desync expected in any segment (not directly logged by BNCH; seen as rpm dropouts / EDT2 warnings)'))
with open(out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=['quantity', 'predicted', 'plus_minus', 'lo', 'hi', 'unit', 'note'])
    w.writeheader()
    w.writerows(rows)
print(open(out).read())
