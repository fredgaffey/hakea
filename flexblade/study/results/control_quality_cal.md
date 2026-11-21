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
