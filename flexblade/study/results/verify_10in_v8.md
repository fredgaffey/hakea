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

## 4. Beam strip stress recovery audit (b2.flex0: 6 elements, 8-lamina stack, fibres z +-0.05 / y +-4.5 mm)
- 219 MPa (T0.7) = (190 out-of-plane + 19 in-plane) x Kt 1.05 at el56a, mid-span; axial 0-1 MPa; torsion (T 22 -> -17
  mN m) is not in sigma. Along the strip: M2 (oop) -15 hub / +25 mid-span plateau / -9 blade mN m; M3 (in-plane) 340 -> 54.
  T1.0: M2 -24 / +30 / -14, M3 414 -> 14 -> 265 MPa.
- Interior peak is PHYSICAL: the stiff strip carries a large in-plane moment M3. Rod equilibrium in the section frame,
  dM2/ds = -V3 + k1*M3 and dT/ds = -k2*M3 (k1 twist rate ~12 rad/m, k2 oop curvature): lateral-torsional coupling
  turns ~15 mN m of M3 into M2 per element and builds the mid-span plateau (clamped M_cr,LTB ~ 2pi/L sqrt(EIy GJ) ~ 0.39
  N m vs M3 0.34-0.41). M2 and M3 both scale ~+22 % T0.7->T1. Presets are not double-counted (slot mode).
- ccx misses it: mixed BCs free u_x/u_y/theta_z -> M3 = 0, no coupling; fair check must impose theta_z/u_y (or apply M3).
- Recovery ARTEFACT found and fixed (beam.element_end_forces(e, c, st)): legacy K12.d7 moments are log-rotation conjugates
  in the CR frame, off by ~1/2 theta x M; with M3 ~ 0.3 N m and theta ~ 1.5 deg that leaks ~5 mN m (~40 MPa) into M2 and
  makes M2/T jump at nodes. Now Bt.f7 (spatial nodal action) in the end-section frame, continuous at nodes
  (tests/test_strip_stress_recovery.py). Mid-span 207 vs 208 MPa (unchanged); hub clamp 175 vs 143 (T0.7), 270 vs 225 (T1),
  so T1 binding sigma_m moves to the hub clamp, 284 (x Kt) vs 265. rotor static/mean uses it; amplitudes (_lin) still CR.
- Re-score (sa.py file v8): cr_pl 0.0883 (was 0.0911), feasible, mech margin 0.036, phase 6.2 deg / gain 9.8 % (unchanged).
- Amplitude recovery now consistent: beam.element_end_forces_lin(e, c, du, st) = central FD (h = 1e-6/max|du|) of the
  corrected Bt.f7 end-section recovery about the hover state (real/imag parts separately); flag beam.LIN_RECOVERY
  ("section" default | "legacy" K12.d7) or mode= kwarg. Test: amplitude M2/M3 continuous at nodes (equilibrated tip-moment
  increment), legacy jumps. Full suite 297 passed.
- Re-score v8 after: cr_pl 0.0883 -> 0.0715, feas true (res_margin 0.177), mech 0.036, phase 6.19 deg / gain 9.8 % (unchanged).
  Binding Goodman lamina (T0.7, J3.76e-6, hub-side group): sa 1011 MPa, sm 183 MPa (fat util 2.30 on the unfixed-flexure group; the
  strip group binding sa=404 MPa, sm=104 MPa, util 0.87); before-values of sa not logged (legacy path not rerun).
