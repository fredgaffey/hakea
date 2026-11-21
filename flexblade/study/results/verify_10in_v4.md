# Verification: 10in_tt_strap_steelx_v4 (8 x 0.10 mm 1095 laminae, 10 mm wide, cross 105 deg, hinge r 21.5 mm)
Script: study/verify_10in_v4.py (stages J | pl | ccx | floq | fit); raw: study/results/verify_10in_tt_strap_steelx_v4.json; parts: study/build/10in_v4 (build_10in_v4.py).

## 1. Pulse-limited score (throttle {0.7,1.0,1.4} x J_hub {itemised, 2.5e-6, 5.49e-6})
Itemised 10in J_hub (kg m^2): printed hub assembly STL 17.61 g PA6-CF, Izz 3.07e-6; 4 hub-side steel lands (2 blades x 2 strips, 0.79 g each) 0.55e-6; 4 x M3x6 screws at 8 mm 0.13e-6; thread-locker/knots 0.01e-6; total **3.76e-6** (not the 7in ~0.9e-6: the 10in hub is 2.5x heavier and wider).

| throttle | J_hub | cr (unscaled) | cr_pl | pulse scale | cap frac | binding | 1P margin (need 0.15) | feasible |
|---|---|---|---|---|---|---|---|---|
| 0.7 | 3.76e-6 | 0.595 | 0.290 | 0.487 | 0.463 | fatigue_stress | 0.088 | no |
| 0.7 | 2.5e-6 | 0.674 | 0.290 | 0.430 | 0.409 | fatigue_stress | 0.076 | no |
| 0.7 | 5.49e-6 | 0.505 | 0.290 | 0.574 | 0.546 | fatigue_stress | 0.106 | no |
| 1.0 | 3.76e-6 | 0.198 | 0.171 | 0.864 | 0.821 | fatigue_stress | 0.164 | yes |
| 1.0 | 2.5e-6 | 0.216 | 0.171 | 0.793 | 0.754 | fatigue_stress | 0.149 | no (by 0.001) |
| 1.0 | 5.49e-6 | 0.178 | 0.171 | 0.964 | 0.916 | fatigue_stress | 0.184 | yes |
| 1.4 | 3.76e-6 | 0.090 | 0.103 | 1.151 | 1.093 | motor_torque_1p | 0.202 | yes |
| 1.4 | 2.5e-6 | 0.095 | 0.110 | 1.151 | 1.093 | motor_torque_1p | 0.188 | yes |
| 1.4 | 5.49e-6 | 0.083 | 0.095 | 1.151 | 1.093 | motor_torque_1p | 0.222 | yes |

cr_pl (min) = **0.0954** (T1.4, J 5.49e-6). **feasible_pl = False**: the hard 'resonance' check (pitch antisym+hub mode at 0.94/rev vs 1P, need margin >= 0.15) fails at T0.7 for all J (0.076-0.106) and at T1.0 J 2.5e-6 (0.149). FC pulse schedule (fraction of motor pulse capability): T0.7 about 0.41-0.55, T1.0 about 0.75-0.92, T1.4 about 1.09 (capped by motor_torque_1p); lower J_hub means a smaller allowed pulse.

Fix attempt (4 evals, width x0.75/0.85/1.15/1.3): the T0.7 margin stays 0.08-0.11; wider helps only T1/T1.4 and trips aeroelastic_stability / pulse infeasible. No minimal fix found, v4b not saved. A real fix needs the hub/arm-mount mode placed away from 1P, not a strip change.

## 2. ccx vs beam (ccx.compare_strip_flexure)
ccx meshes ONE bonded strip of the member thickness (a 0.1 mm lamina); the beam model carries 8 unbonded laminae (8x stiffness). Raw ccx/beam: k_hinge 0.119, k_cross 0.135, k_flap 0.136, kx 0.134, ky 0.127, kz 0.129; axis error 0.001 deg. Against the analytic unbonded stack (x8): 0.95, 1.08, 1.08, 1.07, 1.01, 1.03, within 8 %. The stack itself (lamina contact/slip) is not modelled in ccx.

## 3. Floquet + airframe (f_y = 1.15 f_x; J_hub itemised/2.5e-6/5.49e-6 give the same minima; f_x sweep 60-320 Hz, mount 190/225/260 Hz)
- zeta_af 0.02 (rigid arm): air-resonance zeta slightly NEGATIVE (-0.0026 at T1/T1.4 at 60 Hz, -0.0033 at T0.7 at 100 Hz).
- zeta_af 0.05: min +0.0085 (T1, 60 Hz), +0.0023 (T1.4, 60 Hz), T0.7 +0.0076..+0.017.
- TPU mount (G 6.5 MPa, eta 0.15): min +0.0107 (T1.4, 260 Hz), +0.0137 (T1), +0.0172 (T0.7). Pass.
- Structural zeta_min 0.0083 (T0.7) / 0.0084 (T1) / 0.0103 (T1.4) vs min_damping 0.005. The sweep is coarse (6 frequencies).

## 4. Fit
- Stack 8 x 0.1 = 0.80 mm; slot 0.85 x 10.2 mm (0.05 mm allowance), land 12.65 mm, free span 21.29 mm, jaw R 1.0 mm, wall 1.2 mm. 0.05 mm on a stack of 8 laminae plus epoxy film is tight: test-fit a spare stack.
- Clearance sweep (rigid, lag/pitch): +-5/8 deg 6.50 mm; +-8/12 6.06 mm; +-12/16 4.64 mm; rest 6.61 mm. No interference.
- STLs watertight, single shell: hub 14672 mm3, blade1 8216, blade2 8770. Hub assembly 17.61 g, root fitting 2.67 g, hub fitting 2.62 g per blade.
- Fitting mass vs model: fittings.py mass properties equal trimesh of the exported STLs (root 2.671 g, COM and inertia identical).

## Verdict
Fails the pulse-limited resonance hard check (1P margin below 0.15 at T0.7, marginally at T1 J 2.5e-6). Rigid arm at zeta_af 0.02 is slightly unstable; passes with the TPU mount. ccx and fit pass. cr_pl 0.095 (T1.4).
