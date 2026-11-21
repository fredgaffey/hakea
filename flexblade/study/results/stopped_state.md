# Stopped/assembled rotor with slot presets (kcross_angle.md item 7): v8, v10m SAFE
Tool: fact.stopped_state (flexure-only assembly, both blades, fittings free, no CF/aero/gravity; beam.solve_static
per preset step with a 1e-4 N probe for an absolute residual scale). Script: study/stopped_state.py. Tests: test_stopped_state.py.
- Converged and path-independent: 4 and 16 preset steps (0->100 %) give identical results (4 digits).
| | v8 | v10m |
|---|---|---|
| strip axial N (Pcr clamped 81.8 N) | +0.12..+0.52 N (N/Pcr <= 0.006, tension) | +0.12..+0.53 N (<= 0.007) |
| buckling multiplier of misfit state | 30.3 | 28.3 |
| max strip stress (x Kt 1.05) | 67 MPa (b2.flex0 hub) | 55 MPa |
| vs 0.6 Xt = 840 / fatigue amp 560 MPa | 0.08 / 0.12 | 0.07 / 0.10 |
| fitting rotation, b1 / b2 | 0.40 / 1.21 deg (mostly about the hinge) | 0.84 / 0.93 deg |
| k_2nd principal (no preset 275 / 185) | b1 67 / b2 39 N m/rad | 45 / 33 |
| k_cross fixed axis (no preset 468 / 555) | 114 / 67 | 135 / 97 |
- ccx (study/ccx_preset_strip.py stopped; one lamina, hex20, NLGEOM, beam end motion imposed, mixed BCs): b2.flex0 hub
  surface 64 beam vs 72 ccx median (74 peak), blade 36 vs 40; b2.flex1 hub 59 vs 65, blade 40 vs 44. ccx +11..16 %, still < 90 MPa.
- The axial safety rests on the blade-end radial slider (axial_release): WITHOUT it the misfit compresses the strips
  (-4.5..-10 N, 0.12 Pcr), multiplier 1.5 (< 2) and 198 MPa. Keep the slider free (low friction) or, if it is ever
  dropped, split the preset between hub and blade slots / assemble in a jig at the hover angle. No check_layout gate added.
- The ~40 trial k_cross was the unconverged path; the converged stopped tangent is 4-7x softer than unstressed in cross/flap
  (re-solved FD agrees with the tangent, Km alone gives the same; Kgeo is negligible). Cause not identified (geometric,
  slider/strap kinematics suspected). Bench-only: in flight the presets cancel at hover; do not use stopped k for hover checks.
