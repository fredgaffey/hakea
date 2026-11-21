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
