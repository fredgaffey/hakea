# v10m stiffen sweep (nominal, presets re-solved iters=6; t mm / L x / cross deg). No point feasible; no v11 saved.
Goodman = util fatigue_stress in T0.7/1/1.4 order. At current t it reads 2.21/1.60/1.04 vs verify md 1.064/0.852/0.618, so scale or labels differ; treat as relative.
t | L | ang | cr_pl | res_margin (limiting mode) | Goodman T0.7/1/1.4 | ltb T0.7/1/1.4
0 | 1 | 120 | 0.128 | 0.073 (pitch antisym+hub 0.95/rev) | 2.21/1.60/1.04 | 0.25/0.25/0.27
0 | 1 | 105 | 0.074 | 0.120 (pitch antisym+hub 0.92/rev) | 2.36/1.70/1.16 | 0.30/0.29/0.30
0 | 0.85 | 120 | 0.146 | 0.042 (pitch antisym+hub 0.97/rev) | 2.61/1.81/1.21 | 0.33/0.34/0.31
0 | 0.85 | 105 | 0.105 | 0.053 (pitch antisym+hub 0.96/rev) | 2.78/1.91/1.27 | 0.34/0.37/0.36
0 | 0.7 | 120 | 0.184 | 0.024 (pitch antisym+hub 1.01/rev) | 2.07/2.12/1.26 | 0.33/0.38/0.41
0 | 0.7 | 105 | 0.188 | 0.024 (pitch antisym+hub 1.01/rev) | 2.03/2.23/1.32 | 0.32/0.39/0.44
0.20 | 1 | 120 | 0.129 | 0.078 (pitch antisym+hub 1.08/rev) | 0.29/0.30/0.32 | 0.11/0.14/0.17
0.20 | 1 | 105 | 0.124 | 0.086 (pitch antisym+hub 1.08/rev) | 0.27/0.28/0.29 | 0.12/0.15/0.19
0.20 | 0.85 | 120 | 0.084 | 0.135 (pitch antisym+hub 1.13/rev) | 0.26/0.25/0.24 | 0.09/0.12/0.15
0.20 | 0.85 | 105 | 0.082 | 0.144 (pitch antisym+hub 1.14/rev) | 0.25/0.24/0.22 | 0.09/0.13/0.16
0.20 | 0.7 | 120 | 0.061 | 0.197 (pitch antisym+hub 1.20/rev) | 0.24/0.23/0.20 | 0.07/0.09/0.11
0.20 | 0.7 | 105 | 0.059 | 0.205 (pitch antisym+hub 1.20/rev) | 0.24/0.22/0.20 | 0.07/0.09/0.11
0.25 | 1 | 120 | 0.050 | 0.209 (pitch antisym+hub 1.21/rev) | 0.20/0.20/0.18 | 0.07/0.09/0.12
0.25 | 1 | 105 | 0.049 | 0.223 (pitch antisym+hub 1.22/rev) | 0.17/0.21/0.19 | 0.07/0.09/0.12
0.25 | 0.85 | 120 | 0.038 | 0.103 (pitch sym 0.92/rev) | 0.18/0.20/0.18 | 0.05/0.07/0.09
0.25 | 0.85 | 105 | 0.037 | 0.106 (pitch sym 0.92/rev) | 0.18/0.20/0.19 | 0.05/0.07/0.09
0.25 | 0.7 | 120 | 0.029 | 0.016 (pitch sym 0.99/rev) | 0.35/0.19/0.18 | 0.04/0.05/0.06
0.25 | 0.7 | 105 | 0.029 | 0.017 (pitch sym 0.99/rev) | 0.33/0.19/0.19 | 0.04/0.05/0.06
Finding: t=0.10 fails Goodman at every L/angle, and shortening L worsens res. t=0.20 fixes Goodman (<0.31) but cr drops: L1 cr 0.124-0.129 res 0.078-0.086; L0.85 res 0.135-0.144 cr 0.082-0.084 with harmonic_jitter viol 0.13-0.19; L0.7 res 0.20 cr 0.06 jitter viol 0.41. t=0.25 cr<=0.05.
No point has res>=0.15, Goodman<=0.95 and cr>0.0715, so SA not run and v11 not saved. Angle 105 vs 120 barely matters at t>=0.2.
