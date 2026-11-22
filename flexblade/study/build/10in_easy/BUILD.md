# BUILD: 10in_easy_steelx (2-blade, crossed 8x0.075 mm 1095 stacks, aramid strand, one-piece breakaway jig print)
Files: `rotor_jig.stl` (1x: hub + both blades printed as ONE body), `drawing.png`, `parts.json`. Generator `../build_10in_easy.py` (`flexblade/fittings.py`, `strip_print_parts(..., breakaway_jig=True)`). STL watertight, one shell; clearance min gap 6.5 mm (checked on the fittings without the bridges).
Hub frame, z up = build direction = rotor axis. Strips cross at 105 deg, skew 62 deg.
The print itself fixes the jaw-exit spacing: each blade (with its root fitting) is joined to the hub fitting by 2 thin breakaway sprues (1.2 x 1.2 mm, necked to 0.8 mm for 0.8 mm at both joints) that hold the root fitting at its assembled pose (free length 22.11 mm, pitch presets and strip cross geometry). The sprues lie outside the strip paths and the strand channel. Tested to 0.05 mm of the model (`tests/test_fittings_easy.py`).

## BOM
| Item | Spec | Qty |
|---|---|---|
| Feeler-gauge roll | 12.7 mm x 0.003" (0.075 mm) 1095 spring steel | 1 roll |
| Laminae | 8 per stack, 4 stacks (2 per blade) = 32 pieces of 12.7 x 50.6 mm; cut 40 (8 spare) | 40 |
| PA-CF (pa6_cf) filament | the whole rotor (about 31 g incl. sprues; design material for hub and blades) | 1 spool |
| Aramid strand 2.6 mm | about 40 mm per blade | 2 |
| Dowels | 3 mm, one per slot (datum), cut flush | 8 |
| Glue | slow (30 min+) gel/thick epoxy for the lands ONLY; NO thin CA anywhere near the strips | |
| Dry lube | PTFE dry-film spray or a drop of light oil for the free span | |
| Flush-cut pliers, small file | to remove the sprues | |

## Print settings
- X1 Carbon: PA-CF, dried 6+ h at 80 C, 0.4 hardened nozzle, 0.12 layers, 6 perimeters, 100 % infill, z up, no supports.
- The part is 254 mm long (bed 256): rotate it about 20 deg on the plate (or diagonal) so it clears the bed edges and the purge line.
- Sprues lean 62 deg from horizontal (no support needed) but are 16 mm long and 1.2 mm thin: print slowly (outer wall 60 mm/s), cooling on, and do not knock the part until the strips are glued in. If a sprue shows a layer break, the part is scrap (the pose is no longer held).
- Slot is 0.85 mm x 12.9 mm (8 x 0.075 + 0.25 clearance): check a dry stack drops in; do not drill. Hub slots run through the back; blade slots are over-deep (land + 3 mm), no stops.

## Assembly (hand work +-1 mm is fine)
1. Print the rotor; do NOT cut the sprues yet. Check a dry 8-lamina stack drops into every slot.
2. Snip laminae from the roll to 50.6 mm +-1, grain along the strip, 12.7 mm width as bought; deburr, IPA wipe.
3. Stack 8 laminae per strip DRY (no glue between them: the laminae must slide over each other in the free span, or the hinge becomes up to ~60x stiffer and the rotor is out of tune). Mist the free-span region with PTFE dry film first. Insert each stack from the hub back through the hub jaw and across the free span into the blade slot until it touches the slot bottom, then withdraw it 3 mm (use a 3 mm drill shank as a spacer) so both lands are fully engaged.
4. Press a 3 mm dowel into each dowel hole. Butter thick epoxy into the slot mouths at the LANDS only (from the hub back and the blade slot end), never at the jaw exits facing the free span; wipe any that creeps toward the free span.
5. Thread the aramid strand through the channel, knot in the well, strand just taut, seal.
6. Let everything cure fully (epoxy 24 h). Then snip the sprues (flush-cut pliers, one at a time, alternating sides) and file the stubs flush with the web faces.
7. Pin-balance the blade pair, then the rotor.

## Critical dimensions
- The print fixes the free length (22.11 mm), so a +-1 mm strip cut length only changes how deep the ends sit in the over-deep slots (>= land - 1 mm engaged); it does not change the free length.
- Free span must stay glue-free: k_pitch was verified (CalculiX, verify_5in_v8s.md) only for freely sliding laminae. Check after cure: each strip's free span should flex easily and the laminae should fan slightly when bent.
- Do not remove the sprues before the epoxy has cured: the sprues are what holds the free length.
- Hinge radius +-0.2 mm printed is inside the allowance. Gates pass in the model only; no bench data.
