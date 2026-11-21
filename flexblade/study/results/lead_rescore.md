# Lead rescore on the current model (2026-11-22)

Scorer: `study/i34tools/sa.py file` (control_quality_cal continuous + Goodman + ground_res, SA_WORKERS=2).
Model includes: harmonic_jitter hard, section stress recovery, strip LTB, zero-stress presets.

| design | cr_pl | feasible | worst mech margin | resid phase ° | resid gain |
|---|---|---|---|---|---|
| 10in v8 (lead) | 0.0715 | yes | 0.036 | 6.2 | 0.098 |
| 7in v6 (Goodman) | 0.065 | yes | 0.074 | 2.9 | 0.128 |
| 10in v7 | 0.0641 | yes | 0.034 | 8.2 | 0.118 |
| 10in v6 (I33) | 0.056 | yes | 0.045 | 9.9 | 0.158 |
| 10in v5 | 0.0538 | yes | 0.064 | 6.8 | 0.150 |
| 7in CRM2204 (coax half, 0.18 kg) | 0.110 | NO (gain resid 0.49, mode 1.08/rev in 1P band) | 0.038 | 12.6 | 0.487 |

v8 stays the lead. 7in v6 has the most robust margin (0.074) and lowest phase residual.
