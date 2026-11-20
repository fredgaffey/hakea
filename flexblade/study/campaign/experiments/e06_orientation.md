# E06 print orientation, install and forming (torsion bar, 2307, 0.6 kg)

Bar 1.5 mm CF rod, TPU 95A plate 2.5 mm, skew 45 deg, bar length 12 mm x D/5. Fails = print:* checks with utilisation > 1 (first 3 listed). il = worst interlayer util (blade+flexure). Bed 270 mm cube.

| D | variant | ctrl | axis err deg | il | insert err deg | overhang deg | print max util | fails |
|---|---|---|---|---|---|---|---|---|
| 5 | legacy (no print_setup) | 0.030 | 7.7 | 0.25 | nan | nan | nan | none |
| 5 | flat, bar paused | 0.001 | 45.3 | 5.37 | 45.0 | 45 | 9.00 | insert_in_layer(9.0) |
| 5 | on_edge, bar paused | 0.030 | 7.7 | 0.25 | 0.0 | 45 | 0.90 | none |
| 5 | hinge_in_plane, bar paused | 0.029 | 7.5 | 0.81 | 0.0 | 0 | 0.34 | none |
| 5 | tilted 45, bar paused | 0.029 | 7.5 | 0.81 | 0.0 | 0 | 0.34 | none |
| 5 | drilled_torsion_bar (flat) | 0.030 | 7.7 | 0.80 | 0.0 | 45 | 1.00 | none |
| 5 | formed_torsion_bar (rod mandrel) | 0.030 | 7.7 | 0.25 | 0.0 | 45 | 0.91 | none |
| 6 | legacy (no print_setup) | 0.097 | 10.9 | 0.34 | nan | nan | nan | none |
| 6 | flat, bar paused | 0.010 | 40.4 | 1.53 | 45.0 | 45 | 9.00 | insert_in_layer(9.0) |
| 6 | on_edge, bar paused | 0.097 | 10.9 | 0.34 | 0.0 | 45 | 0.90 | none |
| 6 | hinge_in_plane, bar paused | 0.094 | 10.8 | 0.92 | 0.0 | 0 | 0.41 | none |
| 6 | tilted 45, bar paused | 0.094 | 10.8 | 0.92 | 0.0 | 0 | 0.41 | none |
| 6 | drilled_torsion_bar (flat) | 0.097 | 10.9 | 0.90 | 0.0 | 45 | 1.00 | none |
| 6 | formed_torsion_bar (rod mandrel) | 0.097 | 10.9 | 0.34 | 0.0 | 45 | 0.91 | none |

## Reading

- flat (build = rotor axis z) with a laid-in bar: the bar lies 45 deg out of the layer plane (insert_in_layer util 9.0) -> infeasible; the strength model agrees (bar fibres cross-layer: ctrl 0.001-0.01, interlayer util 5.4 at 5in, 1.5 at 6in).
- on_edge (build = y): bar in-plane, interlayer unchanged (0.25 / 0.34), but the plate lies horizontal in mid-air: feasible only as a bridge (plate length <= 10 mm, counted 0.9 x the 50 deg limit); a longer plate needs supports.
- hinge_in_plane == tilted 45 (build = (cos skew, 0, sin skew), tilt 45 deg for skew 45): bar in-plane, 0 deg overhang, no supports -- cleanest print, but the blade span crosses the layers at 45 deg: interlayer util 0.81 (5in) / 0.92 (6in) vs 0.25 / 0.34, and ctrl ~3 % lower.
- drilled_torsion_bar (flat print, bar drilled after): no layer constraint, fibres along the rod regardless of build -> ctrl/axis error as on_edge; interlayer 0.80-0.90 (plate printed edge-on). Wall (2.5 mm plate, 1.5 mm hole -> 0.5 mm) is exactly at the limit (util 1.00); drill length passes (12 d = 18 mm).
- formed_torsion_bar (flat, rod first as forming mandrel, twist 90 deg about the rod, then bend 45 deg about y at 8.75 mm radius): insert in-plane as printed, twist axis == rod axis, TPU 95A forming 100 C < rod Tg 110 C (assumed), bend radius >= 3 t. Interlayer util 0.25/0.34 (best), ctrl as legacy. PA6-CF (160 C) would need the rod drilled after forming.

Caveats: forming = rigid rotation of region orientation (residual stress not modelled); blade k>1 strength uses blade-1 orientations (printability checks use true per-blade axes); bridging (<= 10 mm), bed seating (1 mm), forming temperatures and rod Tg are assumptions; hinge-axis error (~8-11 deg) is not changed by print setup.

