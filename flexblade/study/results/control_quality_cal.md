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

## 10in v6

I33 directed scan from v5 under control_quality_cal (continuous, 5 nodes, J 10" axis) + all other hard checks (other modes 0.15 1P margin, stability, ground_resonance on, fatigue). Script `study/i33_10in_v6.py` -> `results/i33_10in_v6.json`. CQ ratio = min(15°/phase, 25 %/gain, mech_min/0.03).

| variant (from v5) | cr_pl | feas | phase° | gain % | mech min | CQ ratio | viol |
|---|---|---|---|---|---|---|---|
| v5 (L 25.12, skew 60, hinge_r 21.5) | 0.0822 | ✓ | 6.8 | 15.0 | 0.064 | 1.67 | |
| skew 63 | 0.0627 | ✓ | 3.2 | 12.2 | 0.103 | 2.05 | |
| hinge_r −5 % | 0.0751 | ✓ | 4.9 | 13.9 | 0.083 | 1.80 | |
| **skew 58.5 = v6** | **0.0960** | ✓ | 9.9 | 15.8 | 0.045 | 1.49 | |
| skew 58 | 0.1004 | ✓ | 10.8 | 15.6 | 0.039 | 1.29 | |
| skew 57 | 0.1041 | ✗ | 12.2 | 14.8 | 0.026 | 0.88 | mech |
| L 23 | 0.0936 | ✗ | 11.7 | 16.9 | 0.027 | 0.90 | mech |
| L 22 | 0.1003 | ✗ | 14.5 | 18.0 | 0.010 | 0.33 | mech |
| L 21 | 0.1084 | ✗ | 15.0 | 20.0 | 0.004 FLIP | 0 | mech, side |
| skew 56 + hinge_r −5 % | 0.1011 | ✗ | 11.1 | 15.8 | 0.034 | 1.14 | aeroelastic stab, flexure_inside_hub |
| skew 55 + hinge_r −5 % | 0.1048 | ✗ | 12.2 | 15.1 | 0.023 | 0.77 | + mech |
| v4 L 22.0 (w 10) | 0.0900 | ✗ | 16.1 | 22.6 | 0.006 | 0.20 | phase, mech |
| v4 L 22.7 | 0.0855 | ✗ | 14.4 | 20.1 | 0.019 | 0.64 | mech |
| v4 L 23.3 | 0.0821 | ✓ | 12.7 | 19.8 | 0.031 | 1.03 | |

- Pareto: along every knob cr_pl rises as the worst build corner approaches 1P, and the residual (mostly phase) grows with it. The calibratability margin (mech min ≥ 0.03) binds first, phase second (~12° at the boundary). SKEW is the efficient knob: Δcr_pl/Δmech ≈ −0.58 (skew) vs −0.31 (strip L). The v4 width-10 family is dominated (boundary L ≈ 23.2 mm gives only cr_pl 0.082 at ratio 1.0). hinge_r −5 % moves the mode away from 1P but costs cr_pl; combined with lower skew it trips aeroelastic stability + hub fit.
- Boundary (mech min = 0.03): skew ≈ 57.4° (cr_pl ≈ 0.102) / v5 L ≈ 23.2 mm (≈ 0.094) / v4 L ≈ 23.2 mm (0.082).
- **v6 = v5 with skew 58.5°** (designs/10in_tt_strap_steelx_v6.json): cr_pl 0.096 (+17 %), residual 9.9°/15.8 %, mech min 0.045, authority 0.72, ≥1.49× on all three CQ limits; other modes 0.169 (pitch sym+hub 1.17/rev); no hard violation.
- Tolerance (nominal + 8 random, verify_v5.perturbed seed 11, full CQ-cal continuous each): 7/9 pass, cr_pl 0.088–0.096. random06 fails calibratability (mech 0.012), random07 fails the OTHER-mode 0.15 margin (pitch sym 1.14/rev, 0.141). The random ±5 % / ±1° skew draws push the mode both ways; v6 sits in a narrow skew band (mech needs skew ≳ 57.4°, the pitch-sym margin shrinks as the mode moves the other way). Build note: hold skew to ±0.5° and check the 1P sweep on the bench. v5 was not re-run through this tolerance set, so this is not a like-for-like comparison.
- System check (study/system_check_10in_v6.py -> results/system_check_10in_v6.json; twin mount, arms 120/150/200 Hz, rev-avg governor 20 Hz τ 0.5/2 ms, I_t ×1/×2, 3 J): min ζ 0.0139/0.0114/0.0115 (T0.7/1.0/1.4) vs v5 0.0140/0.0117/0.0089; no divergence; G = free spin (1.000, 0.0°); tilt ≤ 0.12°, lateral 40–44 µm, imbalance 22–26 µm @ 2 g·mm. Same verdict as v5.
