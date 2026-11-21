# k_cross ccx/beam vs cross_angle (10in steelx v8/v10m)
k_cross = 1/(s.C_rr.s) at the hinge point, where s = y x a (in the x-z plane, perpendicular to the hinge axis a), from
ccx.compare_strip_flexure. For crossed strips (width along a) rotation about s = torsion (cos(c/2)) + edgewise bending (sin(c/2)).

## Cause: preset artefact in the beam side of the check, not the crossing angle
- v10m differs from v8 ONLY in cross_angle (105 -> 120) and re-solved strip_preset_deg.
- ccx ignores presets (straight strips, stress-free). The beam `FlexureOnly.stiffness_at(state=None)` evaluates the
  tangent at the reference state. With pre-angled slots (PRESET_MODE "slot") that state is NOT in equilibrium:
  slot misfit -> element forces -> spurious Kgeo plus a mis-posed CR tangent.
- Beam k_cross at the hinge point (8 laminae): v8 440 with presets / 469 without; v10m 324 with presets / 556 without.
  ccx (x8) v10m 583: 583/324 = 1.80 (the reported failure), 583/556 = 1.05.
- The same artefact explains the "new in v8" 0.45 / 0.59 deg beam soft-axis offset. Without presets: 0.001 deg.

## Sweep (v8, presets removed on the beam side; ccx x8 / beam)
| cross deg | 60 | 75 | 90 | 105 | 120 |
|---|---|---|---|---|---|
| k_cross | 1.02 | 1.03 | 1.04 | 1.05 | 1.05 |
| k_flap  | 1.02 | 1.03 | 1.04 | 1.05 | 1.05 |
Smooth and flat. kx/ky/kz 0.98-1.05. The beam strip model (torsion + edgewise) holds up to 120 deg, so no correction
factor is needed and the DE bounds can stay as they are.

## Fix
- ccx.compare_strip_flexure now clears strip_preset_deg on the beam copy, so both sides model stress-free straight strips.
- Test: tests/test_ccx.py::test_strip_flexure_cross_angle_presets (v10m presets, 60 and 120 deg, ccx/beam in 0.8-1.2, axis < 0.1 deg).
- DE impact: none from this. Rotor dynamics/trim use the equilibrated hover state, where Kgeo is consistent. The v10m
  +73 % cr_pl is not explained by this artefact (it must be judged on its other verify results).
- Open: fact.check_layout and rotor.flexure_stiffness(ss=None) use the same unequilibrated reference when presets are set.
  A trial Newton solve of the self-stressed unloaded state (blade fitting free) gave much softer k_cross (~40, v8). Not
  verified (convergence unchecked); the preset misfit may load the strips axially close to Euler (cf. ccx_preset_strip notes).
