# Verification of 7in_tt_strap_steelx_v5 (crossed 1095 shim, cross 105.16 deg)

Scripts: `study/verify_v5.py` (ccx, floq), `study/build/build_7in_v5.py`. Raw: `verify_7in_v5.json`, `study/build/7in_v5/parts.json`.

## 1. ccx (hex20 strip pair, no strap) vs beam model, at the hinge point
| term | ratio ccx/beam |
|---|---|
| k_hinge (about hinge axis, the "pitch/hinge" term) | 0.950 |
| k_cross | 1.028 |
| k_flap | 1.029 |
| kx / ky / kz | 1.008 / 0.960 / 0.972 |

Principal rotation axis vs beam: 0.0015 deg. No term is outside +-20 % (worst -5 %); v3 was 0.94-1.02.

## 1b. Fitting mass/inertia (fittings.py model vs trimesh of the exported geometry, PA6-CF)
Per-blade fittings, density from materials.py, inertia about the centroid:
- hub fitting 1.492 g vs 1.492 g; root fitting 1.558 g vs 1.558 g; diagonal inertia and COM agree to 1e-12 relative (hub Ixx/Iyy/Izz 1.13e-7 / 4.83e-8 / 1.30e-7 kg m^2, root 1.17e-7 / 6.35e-8 / 1.42e-7).
- The exported hub STL holds the whole hub assembly (7.05 g, hub plate + both blade fittings), so the per-blade comparison uses the same CSG solid (keepout cut) as the export; the root STL is the root fitting plus the airfoil, compared on the fitting solid.
- Fitting_balance_mass is 0 in v5 (no pins added). Passes.

## 2. Floquet + airframe stability (airframe f_y = 1.15 f_x, f_x in 50..400 Hz, T = 0.7/1.0/1.4 x weight)
| zeta_af | T | min air-resonance zeta | at f_x (Hz) | min zeta_min (all modes) |
|---|---|---|---|---|
| 0.02 | 0.7 | +0.0086 | 160 | 0.0181 |
| 0.02 | 1.0 | **-0.0034** | 200 | 0.0199 |
| 0.02 | 1.4 | **-0.0032** | 250 | 0.0219 |
| 0.05 | 0.7 | +0.0120 | 400 | 0.0181 |
| 0.05 | 1.0 | +0.0172 | 80 | 0.0199 |
| 0.05 | 1.4 | +0.0227 | 250 | 0.0219 |

Rotor modes themselves are stable everywhere (min 0.018). With a 2 % airframe damping there is a slightly negative air-resonance zeta (about -0.003) at f_x ~ 200-250 Hz at T1 / T1.4, i.e. when the arm resonance sits near the pitch antisym+hub mode (185 / 219 Hz). It disappears at zeta_af 0.05 (min +0.012). So v5 needs an airframe that is isotropic and damped (zeta_af >= ~0.03), or an arm resonance away from 190-260 Hz. Coarse grid (8 frequencies); the sharp minima could be a little deeper between grid points.

## 3. Build package (study/build/7in_v5/)
All three STLs watertight, one shell each, no fragments (blade1 3312 mm^3, blade2 3390 mm^3). Strip 6.97 mm wide, 0.15 mm, free 14.92 mm, cut length in BUILD.md; cross angle 105.16 deg; clearance min gap 3.82 mm (rest 4.08). Pitch preset resolves to -0.115 deg per blade (effectively none; v4 was +2.86). Predicted modes (rest and hover) are in BUILD.md: rest pitch sym 34.5 Hz, pitch antisym+hub 49 Hz, flap 249/255 Hz; hover (T1, 13130 rpm) 131 / 185 / 271 / 351 Hz. BUILD.md also holds the isotropy requirement and the nP calibration measurement list.

## Verdict
Nothing disqualifies v5 on the geometry/FE side. The one caution is air-resonance margin at low airframe damping (see 2).
