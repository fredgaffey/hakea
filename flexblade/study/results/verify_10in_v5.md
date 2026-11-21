# Verification: 10in_tt_strap_steelx_v5 (8 x 0.10 mm 1095 laminae, w 9 mm, free L 25.12 mm, cross 105 deg, hinge r 21.5 mm)
Script: study/verify_10in_v5.py (stages J | pl | ccx | floq | fit; copy of v4 script, J_hub for pl/floq = {2.5, 3.76, 5.49}e-6); summary helper study/verify_md_10in_v5.py; raw: study/results/verify_10in_tt_strap_steelx_v5.json; parts: study/build/10in_v5 (build_10in_v5.py + finish_build_10in_v5.py + make_mount.py); mount numbers: study/mount_calc_10in.py.

## 1. Pulse-limited score (throttle {0.7,1.0,1.4} x J_hub {2.5, 3.76, 5.49}e-6)
**cr_pl (min) = 0.0822 (T1.4, J 5.49e-6), feasible_pl = True**, no hard violations; resonance (pitch antisym+hub 0.90/rev vs 1P) worst margin 0.163 (need 0.15). Tolerance 8/9 (steelx_10in_v5.md; random07 resonance margin 0.144). Per-point table is in build/10in_v5/BUILD.md (pulse schedule): cr_pl T0.7 0.180, T1 0.121, T1.4 0.082-0.089; binding fatigue_stress at T0.7/T1, motor_torque_1p at T1.4; pulse scale 0.57-1.15 x 0.95 capability.
Itemised J_hub (v5, direct-clamp): hub STL 17.73 g Izz 3.01e-6 + lands 0.44e-6 + screws 0.13e-6 + misc 0.01e-6 = **3.59e-6** (between the 2.5 and 3.76 columns).

## 2. ccx vs beam (ccx.compare_strip_flexure, one 0.1 mm lamina vs 8-lamina beam)
Raw ccx/beam: k_hinge 0.118, k_cross 0.131, k_flap 0.131, kx 0.129, ky 0.124, kz 0.126; axis error 0.0008 deg. Per-lamina x 8: **0.94, 1.05, 1.05, 1.03, 0.99, 1.01** (within 6 %). Lamina slip/contact is not modelled in ccx.

## 3. Floquet + airframe (f_y = 1.15 f_x; f_x sweep 60-320 Hz; mount 190/225/260 Hz; T 0.7/1/1.4 x 3 J values)
Minimum air-resonance zeta over the sweep:
| case | T0.7 | T1 | T1.4 | min |
|---|---|---|---|---|
| rigid arm, zeta_af 0.02 | -0.0043 (100 Hz, J 2.5e-6) | +0.0107 | +0.0089 | **-0.0043 (slightly unstable)** |
| rigid arm, zeta_af 0.05 | +0.0028 (260 Hz, J 5.49e-6) | +0.0115 | +0.0063 | **+0.0028** (stable, thin) |
| TPU mount (G 6.5 MPa, eta 0.15, ri4/ro9/t10) | +0.0129 | +0.0114 | +0.0088 | **+0.0088** |

Structural zeta_min 0.0063 (T0.7), 0.0087 (T1), 0.0126 (T1.4) vs min_damping 0.005. The f_x sweep is coarse (6 frequencies, 3 for the mount). The TPU mount is required (as in v4): rigid arm at 0.02 is unstable at T0.7.

Mount sizing (mount_calc_10in.py; STLs mount_tube_clamp / mount_tpu_washer / mount_carrier watertight): rotating mass 124 g, arm modal mass 0.30 kg (0.25 x 1.697 kg minus 124 g), k_m 53 kN/m: hub mode 99-102 Hz, zeta 0.068-0.072, arm mode 238-317 Hz. Tilt 1.3 deg at 0.0975 N m, sag 92 um at 16.65 N. **Concern:** the 10in 1P is 108 / 128 / 152 Hz at T0.7 / T1 / T1.4, so the mount hub mode (100 Hz) sits just under 1P at low throttle: 1P hub amplitude for 2 g mm unbalance 106 / 45 / 30 um. Balance the rotor (< 0.5 g mm). A softer washer (k x 0.4, hub mode 66 Hz, 25 um at 108 Hz) is an option but was NOT Floquet-checked.

## 4. Fit (longer strip)
- Stack 8 x 0.1 = 0.80 mm; slot 0.85 x 9.2 mm (0.05 mm allowance, tight: test-fit a spare stack), land 12.0 mm, free span 25.12 mm, jaw R 1.0 mm.
- Clearance sweep (rigid): +-5/8 deg 6.50 mm; +-8/12 6.44 mm; +-12/16 6.00 mm; rest 6.61 mm. No interference (v4 was 6.50 / 6.06 / 4.64 mm).
- STLs watertight, single shell: hub 14773 mm3 (17.73 g), blade1 8152, blade2 8578; mount carrier/clamp/washer watertight. fittings.py mass/inertia equal trimesh of the exported STLs (root 2.584 g).
- Pitch preset (pre_lag auto, T1): +3.976 deg per blade.

## Verdict
No disqualifier. feasible_pl True, cr_pl 0.0822 (v4 failed the resonance hard check). ccx per-lamina x 8 within 6 %. Rigid arm at zeta_af 0.02 is slightly unstable at T0.7 (as v4); min zeta +0.0028 at 0.05 and +0.0088 with the TPU mount. Caveats: resonance margin 0.163 vs 0.15 need and tolerance 8/9 are thin; mount hub mode close to 1P at T0.7.

## Soft mount v5b (decoupled twin TPU washers; study/mount_soft_10in.py)
Problem: single washer (ri4/ro9/t10, 53 kN/m) gave hub mode 99-102 Hz, 0.93 x 1P at T0.7. Fix: 2 washers ri 4 / ro 5.8 / t 12 mm, 40 % infill, G 6.5 MPa, eta 0.15, centres 30 mm apart, preloaded.
- k_lat 24.0 kN/m (arm-coupled hub mode 68.8-69.5 Hz at f_arm 190-260, zeta 0.072-0.074; 0.65 x 1P(T0.7) = 108 Hz; 1P at T1/T1.4 128/152 Hz is further above).
- Sag: 0.051 mm per 1g lateral, 0.152 mm at 3g, axial 0.204 mm at 16.65 N (both washers preloaded; 0.41 mm if only one carried thrust).
- Tilt: k_tilt 19.4 N m/rad -> 0.50 deg at 0.17 N m, 0.89 deg at 0.30 N m. Single soft washer (ro 6.5, t 10, 21 kN/m): k_tilt 1.06 -> 9.2-16.2 deg, rejected. Spacing 20 mm: 9.2 N m/rad, 1.06-1.87 deg (marginal).
- 1P imbalance at 2 g mm: 27.5 um (108 Hz), 22.9 (128), 20.4 (152) vs 106/45/30 um before.
- Floquet (J 3.76e-6, f_y = 1.15 f_x, zeta_arm 0.02, f_arm 190/225/260): min zeta +0.0063 (T0.7), +0.0087 (T1), +0.0127 (T1.4), identical across arm freq; previous TPU mount +0.0088 at worst. Air-resonance zeta 0.0129 / 0.0114 / 0.0088.
- Verdict: hub mode in the 60-75 Hz window, tilt <1 deg, sag <0.3 mm; stability stays positive but T0.7 min zeta falls to 0.0063 (thin). Floquet model has translation only (no tilt DOF); tilt mode ~75 Hz estimate needs a bench check. Not re-run: J 2.5e-6 / 5.49e-6.
