# BUILD: 5in_v8s_2307_m245_301 (2-blade, crossed 9x0.05 mm 301 FH stacks, one-piece breakaway jig print)
Files: `rotor_jig.stl` (1x: hub + both blades as ONE body, 127 x 23 x 23 mm, ~5 g), `drawing.png`, `parts.json`. Generator `../build_5in_301.py`; test `tests/test_fittings_5in301.py`. STL watertight, one shell; min clearance gap 2.7 mm (lag +-5, pitch +-8 deg).
Design: 5 in, 2307 1300 kV, design AUW 0.245 kg (retune if the built AUW is outside 0.20-0.29 kg). Hub and blades are both PA-CF (pa6_cf), same as the design blade material.
MOTOR INTERFACE IS A PLACEHOLDER: no 2307 shaft/bolt pattern exists in the design or fittings.py; the STL has a plain 5 mm bore on the axis. Replace with the real shaft/bolt pattern before flying.
Each blade is joined to the hub by 2 breakaway sprues (1.2 mm square, necked to 0.8 mm, 8.6 mm long). They hold the free length at 10.05 mm and the strip cross/pitch presets.

## BOM
- Strip: 301 full-hard stainless, 0.05 mm x 4 mm slit strip, 1 roll. Cut 9 laminae per stack, 4 stacks = 36; cut 45 (9 spare), each 26.1 mm, over-length +-1 mm, grain along the strip.
- PA-CF filament (1 spool), aramid strand 1.5 mm (about 25 mm per blade), thick SLOW epoxy (30 min+), PTFE dry-film spray, flush-cut pliers, small file.

## Print (X1 Carbon, one body)
- PA-CF dried 6+ h at 80 C, 0.4 hardened nozzle, 0.12 layers, 6 perimeters, 100 % infill, z up, no supports.
- Print slowly (outer wall 60 mm/s). If a sprue shows a layer break, scrap the part (the pose is no longer held).
- Slot is 0.70 x 4.2 mm (9 x 0.05 + 0.25 thick, 4.0 + 0.2 wide). Check a dry stack drops in; do not drill.

## Assembly (+-1 mm hand work is fine)
1. Do NOT cut the sprues yet. Check a dry 9-lamina stack drops into every slot.
2. Cut laminae, deburr, IPA wipe.
3. GLUE RULES (critical): stack the 9 laminae DRY, no glue between laminae (they must slide in the free span, else the hinge is far stiffer). Mist the free span with PTFE dry film first. NO thin CA anywhere near the strips.
4. Insert each stack from the hub side across the free span until it touches the blade slot bottom, then withdraw 3 mm so both lands (8 mm) are engaged.
5. Thick slow epoxy on the LANDS only (at the slot mouths from the outer ends); keep it off the jaw exits and free span; wipe any creep.
6. Thread the strand through the channel, knot in the well, just taut.
7. Cure 24 h. Then snip the sprues one at a time, alternating sides, and file the stubs flush.
8. POST-CURE CHECK: bend each strip by hand; the laminae must fan slightly and the free span flex easily. If it feels like a solid plate, glue got in: scrap that stack.
9. Pin-balance the blade pair, then the rotor.

## Critical dimensions
- Free length 10.05 mm (set by the print), hinge radius 10.75 mm, land 8 mm, slot depth in blade = land + 3 mm (over-deep, no stops).
- Do not remove the sprues before the epoxy has cured.
- Gates pass in the model only; no bench data.
