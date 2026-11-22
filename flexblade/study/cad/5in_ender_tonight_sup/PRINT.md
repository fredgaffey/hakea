# Designed-support versions (same parts as ../5in_ender_tonight)
- Files: `hub_sup.stl`, `blade_a_sup.stl`, `blade_b_sup.stl`. Each file is the part plus its breakaway support grid, already in print orientation (lowest point at z = 0). The hub prints upside down; the blades print z-up.
- Slicer: **turn slicer supports OFF**. Use **0.2 mm layers, including the first layer**. The support tops stop exactly 0.2 mm under the part, so the gap is one bridged layer and the supports snap off. Any other layer height may fuse them.
- Supports are 0.8 mm walls on a 3.5 mm grid, with nothing within 1 mm of the slots or glue lands.
- About 7–28 mm² per part is still unsupported: the small areas next to the slot keep-outs, plus a few short bridges inside the part. These print as short bridges.
- In a few places a support wall runs right beside a vertical face (0.02 mm away). If one fuses there, cut it with a knife rather than levering it.
- Generator: `../build_5in_ender_sup.py` (`overrides["supports"]=True` in flexblade/cad/solidcraft.py). Assembly and safety: `../5in_ender_tonight/BUILD_TONIGHT.md`.
