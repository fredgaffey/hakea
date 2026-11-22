# BUILD: 10in_easy_steelx (2-blade, crossed 8x0.075 mm 1095 stacks, aramid strand, printed stops)
Files: `hub_fitting_assembly.stl` (1x), `blade1_with_root_fitting.stl`, `blade2_with_root_fitting.stl` (1x each; blade 2 is a different part, flexure mirrored in z), `drawing.png`, `parts.json`. Generator `../build_10in_easy.py` (`flexblade/fittings.py`, `strip_stop=True`). All STLs watertight, one shell each; clearance min gap 6.5 mm.
Hub frame, z up = build direction = rotor axis. Strips cross at 105 deg, skew 62 deg.

## BOM
| Item | Spec | Qty |
|---|---|---|
| Feeler-gauge roll | 12.7 mm x 0.003" (0.075 mm) 1095 spring steel | 1 roll |
| Laminae | 8 per stack, 4 stacks (2 per blade) = 32 pieces of 12.7 x 50.6 mm; cut 40 (8 spare) | 40 |
| PA-CF (pa6_cf) filament | hub fitting + 2 root fittings (about 3.1 + 3.2 g per blade) | 1 spool |
| PETG | the two blades (Ender 3 S1) | 1 spool |
| Aramid strand 2.6 mm | about 40 mm per blade | 2 |
| Dowels | 3 mm, one per slot (datum), cut flush | 8 |
| Glue | thin CA to wick, 5 min epoxy to seal | |

## Print settings
- Ender 3 S1: PETG blades, 0.4 nozzle, 0.12 layers, 4 perimeters, 100 % infill, no supports, z up.
- X1 Carbon: PA-CF hub and root fittings, dried 6+ h at 80 C, 0.4 hardened nozzle, 0.12 layers, 6 perimeters, 100 % infill, no supports, z up.
- Slot is 0.85 mm x 12.9 mm (8 x 0.075 + 0.25 clearance): check a dry stack drops in; do not drill.

## Assembly (hand work +-1 mm is fine)
1. Print all three parts; check a dry 8-lamina stack drops into every slot to its stop.
2. Snip laminae over-length from the roll, grain along the strip, 12.7 mm width as bought; deburr, IPA wipe.
3. Stack 8 laminae per strip with thin CA between, and drop each stack into its slot to the stop (hub end, then blade end).
4. Press a 3 mm dowel into each dowel hole, wick CA then epoxy into the lands.
5. Thread the aramid strand through the channel, knot in the well, strand just taut, seal.
6. Repeat for blade 2; pin-balance the blade pair, then the rotor.

## Critical dimensions
- The printed stop sets the free length: stops sit 14.40 mm past each jaw exit (land 14.25 + 0.15), so a strip cut to 50.6 mm (22.11 free + 2 x 14.25) bottoms with free length 22.11 + 0.3 mm (tested to +-0.05 mm of the model, `tests/test_fittings_easy.py`).
- Shorter is the dangerous side: model sensitivity is about -1 mm free length = resonance violation (infeasible), +1 mm = marginal. Keep the stack hard against the stop; never trim it short, and never let a stack sit back from the stop.
- Hinge radius +-0.2 mm printed is inside the allowance. Gates pass in the model only; no bench data.
