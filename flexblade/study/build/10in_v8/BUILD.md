# BUILD SHEET: 10in_tt_strap_steelx_v8 (lead; 2 blades, crossed 1095 strips, aramid strand)
Files: `hub_fitting_assembly.stl` (1x), `blade1_with_root_fitting.stl`, `blade2_with_root_fitting.stl` (1x each; blade 2 flexure/fitting is z-mirrored, different part), `drawing.png`, `parts.json` (all watertight, 1 shell). Generator `../build_10in_v8.py` (flexblade/fittings.py CSG). mm, hub frame, z up = build dir = rotor axis.
Mount (twin TPU washers, carrier, clamp): reuse `../10in_v5/mount_*.stl` (unchanged, see v5 BUILD.md).
Params: strips crossed 105 deg, skew **62 deg**, hinge radius **21.5 mm**, cross gap 2.71 mm, free span 25.12 mm, land 12.0, slot 0.85 x 9.2, jaw exit R 1.00. Rest gap 6.64 mm, min 6.53 mm over lag +-5 / pitch +-8 deg.

## NOT IN THE STLs (manual, do before printing/assembly)
1. **Slot preset angles** (v8 = 0.55 x zero-stress presets; fittings.py does not apply them, the slots are cut at the nominal member frame). Either re-angle slots in CAD or shim the strips: hub-end/blade-end th2 (deg): b1 strip1 -2.45/+2.17, strip2 +2.73/-2.67; b2 strip1 +4.38/-3.87, strip2 -3.14/+2.61 (th3 <= 0.24, ignore). Full table: design JSON `notes`. Without them the stopped state is unpreset (stiffer, no misfit stress).
2. **Datum dowels**: not modelled. Drill/ream a 3.0 mm steel dowel hole across each hub slot exit (axis on the jaw exit radius centre line, parallel to the strip width) so the exit radius is the dowel (hinge_tolerance.md); same for the root exit. The only depth stop is the blind root end; make a seating gauge.

## BOM
| Item | Spec | Qty |
|---|---|---|
| 1095 spring-steel shim | **0.10 mm (0.004") x 9.00 mm wide x 49.1 mm long** (25.12 free + 2 x 12.0 land); 4 strips x 8 laminae | 32 (cut 40) |
| Damper | VHB 9469 0.13 mm + 0.05 mm shim, 9.00 x 23.1 mm, 1 mm short of jaw exits | 4 |
| Aramid strand | 1.5 mm, 40 mm per blade, one strand hub to blade | 2 |
| Dowel pins | 3.0 mm hardened steel, ~12 mm long (hub exits, root exits) | 4 + 2 spare |
| Radial slider | PTFE sleeve/film + PTFE grease on the blade-end radial slider; MUST slide freely (stopped_state.md: without it strips reach 198 MPa, buckling multiplier 1.5) | 2 |
| Epoxy / CA / oil | structural epoxy on lands only; 3-in-1 oil between laminae on free span only | |
| Balance | no designed masses (fitting_balance_mass 0): pin-balance blades to < 0.05 g tip difference, then whole rotor | |
| Gauges | 2.7 mm cross-gap shim, gauge pin for r_h | |
Printed mass (PA6-CF): hub fitting 2.54 g/blade (assembly 17.75 g), root fitting 2.58 g.

## Print
PA6-CF dried 6+ h at 80 C, z up as exported, 0.4 nozzle, 0.12 layers, 6 perimeters, 100 % infill, no supports. Slot ~1.7 layers: test with a spare strip, scrape or ream (do not drill).

## Assembly
1. Prep laminae: cut along rolling direction, file to 9.00 +-0.05 mm (k ~ width), matched within 0.05 mm, one sheet. Deburr all four edges (400 grit lengthwise, break ~0.1 mm), IPA, scuff lands only.
2. Fit dowels; dry test-fit stacks (0.80 mm in 0.85 slot).
3. Per blade: oil laminae lightly over free span only, push each stack through the hub jaw into the blind root slot (seated hard against the end), strips crossing at 105 deg.
4. Epoxy on lands only (oil wiped off), hold the 2.7 +-0.05 mm shim between strips until cured.
5. Check r_h with the gauge pin before the epoxy sets; trim with a 0.1 mm shim under the root fitting (+0.1 mm = +0.0009 mech/rev).
6. Damper layers, thread strand, stopper knots (just taut), CA then epoxy seal; PTFE and grease on the slider.
7. Blade 2 likewise; pitch preset matched +-0.3 deg (inclinometer); balance pair then rotor.

## Critical dimensions
- Hinge radius r_h 21.5 mm to **+-1.5 % (+-0.33 mm)** worst case, RSS +-0.75 %; dln(p)/dln(r_h) = 0.43. Datum on dowels, not printed edges.
- Jaw exit radius **>= 1.0 mm** (0.5 acceptable, forbid < 0.5): clamp_edge.md FE K <= 1.03 for r >= 1 mm, sharp 1.28.
- **Deburred strip edges** (burrs cut fatigue life to 0.5-0.6x; not modelled).
- Layers <= 0.12, cross-gap +-0.05 mm, strip width +-0.05 mm.

## Arm / mount spec (arm_vertical.md)
- Mount mass <= 126 g; twin TPU washers as v5 (k_t 145 N m/rad).
- f_x, f_y >= 120 Hz (forbidden 50-95 Hz) AND **f_z >= 210 Hz at precone 0**, or f_z >= 170 Hz with 1.3 deg precone (1P vertical force 11 N -> 1.55 N). Tip > 50 um bands: 80-203 Hz (precone 0), 106-165 Hz (precone 1.3).
- A round tube has f_z = f_x: 22 x 20 CF at 200 mm gives 136 Hz (fails 210 Hz). Use 25 x 23 at <= 180 mm (194 Hz, still < 210) or precone 1.3 deg with >= 170 Hz (25 x 23 at 200 mm = 166, marginal; use <= 190 mm). Tap-test x, y, z (>= 1 kHz).
- 2P lateral 9.75 N (precone 1.3, unverified): keep f_x, f_y out of 205-339 Hz.
- ESC governor on rev-averaged eRPM or none; FC cyclic schedule continuous in eRPM, calibrate every build (v5 BUILD.md).
