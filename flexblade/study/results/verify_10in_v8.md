# Verification: 10in_tt_strap_steelx_v8 (v5 + skew +2 deg, 0.55 x zero-stress slot presets, cr_pl 0.0911)
Tools: study/i34tools/tol.py (tolerance), study/verify_10in_v5.py ccx 10in_tt_strap_steelx_v8 (ccx); raw: results/verify_10in_tt_strap_steelx_v8.json

## 1. Tolerance (nominal + 8 random builds, t/w/L/hinge_r U(0.95,1.05), presets fixed; T0.7/1/1.4 at J 3.76e-6)
- cr_pl range 0.0826 - 0.1014 (nominal 0.0923). Resonance mode (pitch antisym + hub) 0.90 - 0.93 /rev vs 1P.
- Resonance margin range 0.095 - 0.163 (need 0.15); nominal itself is 0.131 (FAILS the 0.15 need).
- Violations: 6 of 9 builds (incl. nominal) have a hard 'resonance' violation (0.019 - 0.367); only 3/9 clean
  (builds 3, 4, 5, all with margin >= 0.152). Only violation type is resonance. v5 was 8/9 at margin 0.163.
- Trend: thicker/wider/larger hinge radius (stiffer) builds push the mode toward 1P and fail.

## 2. ccx vs beam (ccx.compare_strip_flexure, one 0.1 mm lamina vs 8-lamina beam, unloaded)
- ccx path IGNORES flexure.strip_preset_deg: strips are meshed straight at the member frame, hub face fixed,
  blade face rigid; it returns the linear 6x6 stiffness. The slot-clamped preset state is NOT modelled.
- Linear stiffness does not depend on the preset, so this check still tests the strip model.
  ccx/beam raw, x8 laminae: k_cross 1.11, k_flap 1.06, kx 1.05, ky 1.03, kz 1.04 (all within 11 %).
- k_hinge (~0.5 N m/rad beam, ~0 ccx) is axis-noise: beam principal axis is 0.45 deg off the design axis
  (ccx 3e-6 deg; v5 was 0.0008 deg), and 440 N m/rad cross stiffness x sin^2(0.45 deg) ~ 0.03 N m/rad. Ratio not meaningful.
  The 0.45 deg beam axis offset is new in v8 (skew 62 deg); not investigated.
- Hover static deflection/stress with presets: NOT compared (needs a loaded ccx strip model with preset-rotated
  slot frames; not built). Beam PRESET_MODE="slot" hover stress remains unverified by ccx.

## Verdict
Stiffness pass (<= 11 %, 20 % limit). Preset/hover pass/fail NOT established. Tolerance: v8 is worse than v5 on resonance.

## 3. Loaded preset strip (study/ccx_preset_strip.py; one 0.1 mm lamina, hex20 40x12x2, NLGEOM)
- Clamp frames = R_node @ Q_slot from the beam hover solve (preset included); relative blade-clamp motion imposed.
  Binding strip b2.flex0 (preset hub 4.38 / blade -3.87 deg): hover u_z -2.81/-2.92 mm, theta_y 12.2/12.7 deg (T0.7/T1).
- All-6-DOF imposition is ill-conditioned (axial misfit of ~7 um -> -58 MPa membrane, past Euler; ccx 3-10x beam).
  Used mixed BCs: u_z, theta_x, theta_y imposed; u_x, u_y, theta_z free (beam N ~ 0). Membrane then agrees (<1 MPa).
- Beam sigma_m 219 MPa (T0.7) is the strip max at an INTERIOR element end; at the clamp edges beam sm(xKt 1.05) is 150 hub / 79 blade.
- Clamp-edge surface stress, ccx median over width vs beam out-of-plane bending (in-plane, free in ccx, is 31-38 hub / 1-5 blade):
  T0.7 hub 110 vs 112 (-2 %), blade 80 vs 72 (+11 %); T1.0 hub 113 vs 187 (-40 %), blade 84 vs 111 (-25 %).
  vs full beam sm: T0.7 0.77 / 1.05, T1.0 0.50 / 0.75 (hub / blade).
- Verdict: T0.7 PASS (out-of-plane, <=11 %); T1.0 FAIL (beam 25-40 % HIGHER than ccx, conservative). ccx stress barely moves
  T0.7->T1 (end kinematics nearly equal); the beam's jump and its interior 219/265 MPa peak are not reproduced -> look at
  the beam strip discretisation/slot-frame stress recovery (not investigated). Beam in-plane part unchecked.
- Principal-axis offset (axis stage, beam flexure_only_model): v8 0.453 deg, v8 presets=0 0.0008 deg, v8 skew60 0.456 deg,
  skew60 presets=0 0.0008 deg -> caused by the (blade-asymmetric) strip presets, not by the skew.
