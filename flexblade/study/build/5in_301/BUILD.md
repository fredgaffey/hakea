# BUILD: 5in_v8s_2307_m245_301 (2-blade, crossed 9x0.05 mm 301 FH stacks, one-piece breakaway jig print)
Files: `rotor_jig.stl` (1x: hub + both blades as ONE body, 127 x 23 x 23 mm, ~5 g), `drawing.png`, `parts.json`. Generator `../build_5in_301.py`; test `tests/test_fittings_5in301.py`. STL watertight, one shell; min clearance gap 2.7 mm (lag +-5, pitch +-8 deg).
Design: 5 in, 2307 1300 kV, design AUW 0.245 kg (retune if the built AUW is outside 0.20-0.29 kg). Hub and blades are both PA-CF (pa6_cf), same as the design blade material.
Motor interface (2306/2307, M5 prop shaft): 5.1 mm bore on the axis, hub boss OD 10 mm x 12 mm tall (z +-6), flat washer faces top and bottom (Ø10; hub is flat to Ø12). Boss clears the strip keep-out by 0.33 mm; strip slots stay open through it. Stack on the shaft, bottom to top: motor bell, M5 flat steel washer, hub, M5 flat steel washer, prop nut. PA-CF creeps under nut preload, so the washers spread the load. Needs about 12 + 2 x 1 + nut height of exposed M5 thread above the bell (about 18 mm with a 4 mm nut): check the shaft length before printing. Use a locking (nyloc or nut plus threadlock) nut.
Slot mouths carry a 0.5 mm 45 deg chamfer (width and thickness) at every jaw exit, hub and blade side, to soften the clamp-edge stress concentration. It starts at the exit face, so the free length (10.05 mm) is still measured from that face (chamfer root is 0.5 mm further in, not used as datum).
Each blade is joined to the hub by 2 breakaway sprues (1.2 mm square, necked to 0.8 mm, 8.6 mm long). They hold the free length at 10.05 mm and the strip cross/pitch presets.

## BOM
- Strip: 301 full-hard stainless, 0.05 mm x 4 mm slit strip, 1 roll. Cut 9 laminae per stack, 4 stacks = 36; cut 45 (9 spare), each 26.1 mm, over-length +-1 mm, grain along the strip.
- 2 x M5 flat steel washer (10 mm OD), 1 x M5 prop nut (locking), PA-CF filament (1 spool), aramid strand 1.5 mm (about 25 mm per blade), thick SLOW epoxy (30 min+), PTFE dry-film spray, flush-cut pliers, small file.

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
