# 10in_tt_strap_steelx_v5: placing the cyclic (pitch antisym+hub) mode away from 1P
Script: study/steelx_10in_v5.py (decomp | knobs | pl | tol | save). Raw: study/results/steelx_10in_v5.json.
Design: study/designs/10in_tt_strap_steelx_v5.json = v4 with strip free length 21.29 -> 25.12 mm (x1.18), width 10 -> 9 mm.
Everything else is unchanged (8 x 0.10 mm 1095, cross 105 deg, skew 60 deg, hinge r 21.5 mm, EA 0.33, balance pins).

## 1. 2-DOF model of the cyclic mode (v4, J_hub 3.76e-6)
Coordinates: hub lag psi plus antisymmetric blade pitch/lag. w^2 = (k_struct + k_cf Omega^2)/I, so
/rev = sqrt(k_struct/(I Omega^2) + k_cf/I).
Rayleigh split of the model mode (real shape x scaled to hub swing psi = 1 rad, so I is in kg m^2 and k in N m/rad):
k_struct = x'Re(K_el)x, k_cf Omega^2 = x'Kgeo x, I = x'Mx. The modes come from the undamped structural solver
(rotor.modes, no aero stiffness).

| throttle | rpm | /rev (model) | /rev (Rayleigh) | k_struct N m/rad | k_cf N m s^2 | I kg m^2 | struct share |
|---|---|---|---|---|---|---|---|
| 0.7 | 6525 | 1.011 | 1.011 | 10.7 | 4.61e-5 | 6.74e-5 | 0.33 |
| 1.0 | 7751 | 0.990 | 0.991 | 15.4 | 4.97e-5 | 7.46e-5 | 0.32 |
| 1.4 | 9145 | 0.988 | 0.988 | 18.1 | 5.76e-5 | 7.92e-5 | 0.26 |

Least-squares fit of w^2 = a + b Omega^2 over the 3 points: a = k_struct/I = 3.98e4 s^-2 (sqrt = 199 rad/s = 32 Hz),
b = k_cf/I = 0.929 (rel. rms 0.6 %). The CF term alone puts the mode at sqrt(0.929) = 0.96/rev. The structural
share (0.26-0.33) only adds about +0.03..+0.05/rev, and that addition is largest at T0.7. The mode shape changes with
the trim deflection (k_struct and I both rise with rpm), so the 2-DOF constants hold only roughly.

Important: the pulse-limited checker uses damped modes with aero stiffness, and those sit lower and fall faster with
rpm. For v4 that gives about 0.92/0.86/0.83 per rev at T0.7/1.0/1.4, against 1.01/0.99/0.99 structural. So the T0.7
problem comes partly from the aero (thrust-dependent) stiffness, and only partly from the k_struct share.

## 2. Knob ranking: d(/rev) per +10 % of the knob, structural model, T0.7 / T1.0 / T1.4
| rank | knob | T0.7 | T1.0 | T1.4 | type |
|---|---|---|---|---|---|
| 1 | skew_deg (60) | -0.046 | -0.042 | -0.040 | uniform (lag->pitch coupling) |
| 2 | hinge_radius (21.5 mm) | +0.041 | +0.044 | +0.044 | uniform (CF lever, k_cf/I) |
| 3 | elastic_axis_chord_frac (0.33) | +0.005 | -0.021 | -0.041 | tilts the slope (aero/propeller) |
| 4 | strip length (21.3 mm) | -0.024 | -0.010 | -0.005 | k_struct: acts mainly at T0.7 |
| 5 | strip width (10 mm) | -0.005 | -0.014 | -0.017 | mixed |
| 6 | cross_angle (105) | -0.006 | +0.004 | +0.005 | small |
| 7 | strap length / strap diameter | -0.005 / +0.004 | | | small (k_cf through the strap) |
| 8 | balance pins (r, normal offset, mass) | <= 0.005 | | | negligible: I_eff is dominated by the blade |

Larger steps (structural /rev at T0.7/T1.4): hinge r x0.75 gives 0.90/0.82, skew 72 gives 0.91/0.89, hinge r x1.35
gives 1.13/1.11, and hinge r x1.2 with skew 52 gives 1.14/1.12.

## 3. Which side of 1P? (pulse-limited, throttle {0.7,1,1.4} x J {3.76, 2.5, 5.49}e-6; 34 robust evals)
- **Above 1P** (hinge r up, skew down): feasible at hinge r x1.45 with skew 44 (1.16/rev), but cr_pl is only 0.032.
  At hinge r x1.35 with skew 52 the mode is only at 1.07/rev: the aero stiffness pulls it back toward 1P.
  Lower skew also costs lag->pitch authority. Rejected.
- **Below 1P with hinge r down** (x0.75-0.9): trips geom:flexure_inside_hub (hub radius 12 mm), and a sym+hub mode
  near 2P makes the pulse infeasible (harmonic_jitter). Skew 66 is feasible but scores only 0.062.
  Skew 72 and above: aeroelastic_stability fails and the pulse becomes infeasible.
- **Below 1P via strip length** (the k_struct knob that acts at T0.7): this is the winner. A longer strip lowers T0.7
  most, and a slightly narrower strip lowers the margin at high throttle less, so the three margins come out nearly
  equal at 0.16-0.20.
- Moving the elastic axis forward (EA 0.28-0.31) raises cr_pl at T1.4 by moving it closer to resonance, but at T0.7
  the margin falls below 0.15 in every combination tried.
- Pareto (cr_pl vs min 1P margin): v4 0.095 / 0.076; L1.1 + EA 0.30 0.099 / 0.095; L1.1 + w0.9 0.091 / 0.130;
  L1.15 + w0.9 0.085 / 0.152; **L1.18 + w0.9 0.082 / 0.163**; L1.15 0.076 / 0.173. cr_pl at T1.4 is
  resonance gain, so it trades almost linearly against margin.

## 4. v5 result (L x1.18 = 25.12 mm, w 9.0 mm)
| throttle | J_hub | cr | cr_pl | pulse scale | binding | 1P margin |
|---|---|---|---|---|---|---|
| 0.7 | 3.76e-6 | 0.297 | 0.180 | 0.607 | fatigue_stress | 0.177 |
| 0.7 | 2.5e-6 | 0.318 | 0.180 | 0.566 | fatigue_stress | 0.163 |
| 0.7 | 5.49e-6 | 0.271 | 0.180 | 0.664 | fatigue_stress | 0.196 |
| 1.0 | 3.76e-6 | 0.145 | 0.121 | 0.831 | fatigue_stress | 0.188 |
| 1.0 | 2.5e-6 | 0.153 | 0.121 | 0.791 | fatigue_stress | 0.177 |
| 1.0 | 5.49e-6 | 0.136 | 0.121 | 0.888 | fatigue_stress | 0.205 |
| 1.4 | 3.76e-6 | 0.075 | 0.086 | 1.150 | motor_torque_1p | 0.173 |
| 1.4 | 2.5e-6 | 0.077 | 0.089 | 1.150 | motor_torque_1p | 0.178 |
| 1.4 | 5.49e-6 | 0.072 | 0.082 | 1.150 | motor_torque_1p | 0.167 |

**cr_pl 0.082** (T1.4, J 5.49e-6), feasible on all hard checks. The cyclic mode sits at 0.90/rev, and the minimum 1P
margin is 0.163 (T0.7, J 2.5e-6). This costs 14 % of v4's cr_pl (0.095), which was infeasible.

Tolerance (nominal + 8 random +-5 % on t, w, L, hinge r, rho, strap; skew +-1 deg; EA +-0.01; J 3.76e-6): **8/9 pass**,
with cr_pl 0.066-0.092. random07 fails on resonance (margin 0.144).

Not re-run for v5: ccx, Floquet/airframe and the fit stages (verify_10in_v4.py with the v5 name). The longer strip
needs a new build of the fittings.
