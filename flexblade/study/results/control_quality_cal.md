# control_quality_cal: post-calibration residual (study/control_quality.py cal)

Build tolerance (nominal + 4 CQ_PERTURB corners) is calibrated out per airframe (FC schedule of cyclic gain+phase per throttle at measured J). Residual = LINEAR worst-case stack of one-at-a-time runtime deviations about each calibrated (throttle, build, J) node: rpm +-5 % (thrust ~ rpm^2), temp +20 C (op.ambient_C 45: printed parts derate; steel strip E -0.6 %) / -20 C (strip E +0.6 % only, = 0.3 x creep, model has no stiffening below 25 C), creep strip k -2 %, J error +-10 %. Strip E applied as strip width (thin strip: EI/EA/GJ linear in w). Aramid strap: no T dependence in the model (skipped). Limits: phase <= 15 deg, gain <= 25 %; calibratability: |G_b| >= 0.5 |G_nom|, mechanism-mode 1P margin >= 0.03 on the nominal side, every build sample. Other modes keep the hard 0.15 margin (from the nominal-build points). J axis: 7" {0.92,2.5,5.49}e-6, 10" {2.5,3.76,5.49}e-6.

| design | margin: feas / cr_pl | CQ: feas, phase±, gain± | CAL: feas | resid phase° | resid gain % | groups (rpm/temp/creep/J) ph°/gain% | authority min | cr_pl ratio min | mech margin min | CAL viol |
|---|---|---|---|---|---|---|---|---|---|---|
| 10in_v5 | True / 0.0822 | False, 20.6, 78 % | False | 19.6 (T0.7:p0:J2.5e-06) | 34.3 (T0.7:p0:J3.76e-06) | rpm 15.1/26 temp 2.2/5 creep 1.1/2 J 1.8/4 | 0.75 | 0.74 | 0.064 | {'cqc_phase': 0.3055, 'cqc_gain': 0.3735} |
| 7in_v6 | True / 0.0650 | False, 17.9, 54 % | True | 8.6 (T0.7:p0:J9.2e-07) | 19.9 (T1.4:p1:J5.49e-06) | rpm 6.6/12 temp 1.3/5 creep 0.3/2 J 1.4/9 | 0.75 | 0.00 | 0.074 | {} |
| 7in_v5 | False / 0.0604 | False, 20.0, 47 % | True | 7.8 (T1:p0:J9.2e-07) | 24.3 (T1.4:p0:J9.2e-07) | rpm 5.0/16 temp 2.0/5 creep 0.9/2 J 1.4/9 | 0.70 | 0.00 | 0.055 | {} |
| 7in_v6+ballast | False / 0.0803 | False, 109.0, 91 % | False | 27.8 (T0.7:p0:J2.5e-06) | 39.1 (T1.4:p2:J9.2e-07) | rpm 18.6/26 temp 3.4/10 creep 1.2/2 J 11.9/16 | 0.58 | 0.00 | 0.001 FLIP | {'cqc_phase': 0.8542, 'cqc_gain': 0.5634, 'cqc_mech_margin': 0.9778, 'cqc_mech_side': 1.0} |
| 10in_v4 | False / 0.0954 | False, 84.6, 93 % | False | 36.8 (T0.7:p0:J5.49e-06) | 37.9 (T1:p0:J5.49e-06) | rpm 25.6/26 temp 3.7/6 creep 1.3/3 J 6.4/5 | 0.51 | 0.71 | 0.003 FLIP | {'cqc_phase': 1.4538, 'cqc_gain': 0.5158, 'cqc_mech_margin': 0.8968, 'cqc_mech_side': 1.0} |

margin-criterion hard violations: 10in_v5: {}; 7in_v6: {}; 7in_v5: {'resonance': 0.3539}; 7in_v6+ballast: {'resonance': 0.7664}; 10in_v4: {'resonance': 0.4945}

## Continuous eRPM schedule (rpm_schedule="continuous")

FC has continuous eRPM: gain/phase calibrated at n nodes equally spaced in rpm over throttle 0.7-1.4 (rpm ~ sqrt thrust), linearly interpolated in ln rpm. rpm residual = worst interpolation error at 2 test points per interval + |dG/dln rpm| x 0.5 % noise; temp/creep/J groups unchanged. Test points carry tag ':c' and are excluded from cr_pl / hard checks. Limits phase <= 15 deg, gain <= 25 %.

| design | nodes | resid phase deg | resid gain % | rpm group ph/gain% | CAL feas |
|---|---|---|---|---|---|
| 10in_v5 | 5 | 6.8 | 15.0 | 2.3/6.2 | True |
| 7in_v6 | 5 | 3.6 | 15.2 | 0.9/3.6 | True |
| 7in_v5 | 5 | 5.1 | 13.5 | 1.1/3.5 | True |
| 10in_v5 | 8 | 6.0 | 13.5 | 1.5/4.6 | True |
| 7in_v6 | 8 | 3.5 | 14.8 | 0.9/2.9 | True |
| 7in_v5 | 8 | 4.7 | 12.5 | 0.7/2.4 | True |

## steelx_v5zs skew retune scan (v7 attempt; skew_add on 60 deg, presets re-resolved, Goodman, J10)

| skew add | mech margin (worst corner) | phase res deg | gain res | cr_pl | feasible | hard viol |
|---|---|---|---|---|---|---|
| +4 | 0.019 | 6.9 | 0.080 | 0.099 | no | cqc_mech_margin (<0.03) |
| +5 | 0.033 | 6.2 | 0.086 | 0 | no | pulse_infeasible |
| +5.4 | 0.038 | 5.7 | 0.088 | 0 | no | pulse_infeasible |
| +5.7 | 0.043 | 5.4 | 0.089 | 0 | no | pulse_infeasible, aeroelastic_stability 0.0002 |
| +6 | 0.047 | 5.0 | 0.090 | 0 | no | pulse_infeasible, aeroelastic_stability 0.0088 |
| +8 | 0.076 | 3.2 | 0.091 | 0 | no | pulse_infeasible, aeroelastic_stability 0.060 |

Cliff between +4 and +5: margin reaches 0.03 at about +4.8 but pulse_infeasible appears (pulse scale 0 at some case). No v7 saved.

## I36 partial preset: v5 + alpha x (resolved v5zs slot presets), no re-resolve (study/i34tools/sa.py)

Scored with control_quality_cal (continuous, 5 nodes) + Goodman (`strip_fatigue_mean="goodman"` set explicitly) + ground_resonance (FLEXBLADE_GROUND_RES=1). NOTE: the earlier sc.py / rt.py skew scan above set NEITHER Goodman nor ground_resonance (v5zs json has strip_fatigue_mean "none"), so its cr_pl values are amplitude-only fatigue. alpha = 0 is v5 (CAL worst-corner mech margin 0.064).

| alpha | mech mode /rev (nominal T0.7..1.4, all builds) | worst-corner mech margin | resid phase deg / gain % | binding lamina sm MPa (T0.7 / T1.0, J3.76e-6) | cr_pl | CAL feasible |
|---|---|---|---|---|---|---|
| 0.3 | 0.92 nominal; 0.876-0.966 | 0.034 | 8.2 / 11.8 | 219 / 318 | 0.0818 | YES (no hard viol) |
| 0.4 | 0.93 nominal; 0.887-0.978 | 0.022 | 8.5 / 10.3 | 215 / 288 | 0.0895 | no: cqc_mech_margin |
| 0.6 | 0.95 nominal; 0.905-0.996 | 0.004 | 7.3 / 8.4 | 220 / 226 | 0.108 | no: cqc_mech_margin |
| 0.8 | 0.97 nominal; 0.916-1.008 | 0.001 FLIP | 7.0 / 7.4 | 228 / 265 | 0.126 | no: mech margin + side flip |

Binding check at every alpha: fatigue (strain/stress) at T0.7 (pulse scale 0.29-0.46). The worst corner moves ~-0.105/rev per unit alpha up to 0.4, then saturates near 1P. Linear interpolation alpha=0 (0.064) to 0.4 (0.022) puts margin 0.03-0.035 at alpha 0.28-0.32; alpha 0.3 scored 0.034 -> saved as designs/10in_tt_strap_steelx_v7.json (slot angles in "notes"; +22 % cr_pl vs v5 with Goodman 0.067). Under the old 0.15 hard 1P band v7 fails (pitch antisym+hub 0.92/rev, margin 0.125); under CAL it is feasible.

Tolerance check (study/i34tools/tol.py; nominal + 8 random builds, t/w/L/hinge_radius each U(0.95,1.05), printed presets FIXED; T0.7/1.0/1.4 at J 3.76e-6, margin metrics): cr_pl 0.076-0.092 (nominal 0.082); mech mode 0.90-0.94/rev; the ONLY hard violation on any build is the legacy 0.15 'resonance' band on that mechanism mode (exempt under CAL); no fatigue, ground_resonance or stability violation.

### Skew +5 deg cliff diagnosed (s5 design rescored with sa.py)

pulse_infeasible comes from `harmonic_jitter` (n>=2 resonant-crossing jitter, rotor.py / forced.py). In run_study.pulse_util_fns its u(s) = j*max(s,1) / max(JITTER_MAX_DEG=0.3, 0.10*pitch_1P*s): at vanishing pulse the allowable is the fixed 0.3 deg floor and the jitter does not scale down (max(s,1)), so if nP jitter > 0.3 deg at a point, pulse_scale returns 0. At skew +5 it flips at NOMINAL T1.0, J 2.5e-6 and 3.76e-6 (u0 1.027 / 1.007, i.e. jitter ~0.30-0.31 deg, just over the floor) and on stiff build corner p1 (t,w,h +5 %, L -5 %) at T0.7/T1.0/T1.4 (u0 up to 1.48). At +4 it is just below the floor -> a cliff, not a trend. Note the function is non-monotone (allowable grows with s; for s>=3/pitch_1P u = j/(0.1 pitch_1P), constant), so pulse_scale's "u(1e-9)>1 -> infeasible" shortcut can declare infeasible a point that is feasible at a normal pulse amplitude; the same flip appears on alpha 0.8 at T1.4:p1 (u0 3.7). Fix candidates: treat jitter as a hard check at the operating pulse (s=1) instead of in pulse_scale, or search s over the whole range rather than testing s->0.

## I36 v8 search: skew add + alpha x v5zs presets (sa.py, SA_SKEW_ADD env; CAL continuous + Goodman + ground_res)

Base 10in v5 with skew_deg + add. v7 reference (skew +0, alpha 0.3): margin 0.034, cr_pl 0.0818. Mode /rev was not logged (sa.py prints only the summary); worst-corner margin is the CAL metric.

| skew add deg | alpha | mode /rev | corner margin | resid phase deg / gain | sigma_m T0.7 MPa | cr_pl | feasible |
|---|---|---|---|---|---|---|---|
| +2 | 0.5 | n/a | 0.041 | 6.1 / 10.3 % | n/a | 0.0881 | yes |
| +4 | 0.7 | n/a | 0.056 | 4.3 / 9.2 % | n/a | 0.0778 | no: cqc_mech_side (side flip) |
| +3 | 0.6 | n/a | 0.047 | 5.0 / 9.7 % | n/a | 0.0833 | no: cqc_mech_side (side flip) |
| +2 | 0.55 | n/a | 0.036 | 6.2 / 9.8 % | 219 (binding lamina) | 0.0911 | yes |

Best: skew +2, alpha 0.55, cr_pl 0.0911 (+11 % vs v7), saved as designs/10in_tt_strap_steelx_v8.json. Higher alpha flips the mechanism mode across 1P at some corner even with extra skew, so the side flip, not margin, caps alpha near 0.55.
