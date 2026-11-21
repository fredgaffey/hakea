# BUILD SHEET: 7in_tt_strap_steelx_v4 (2-blade, crossed 1095 strips, aramid strand)

Files (this folder): `hub_fitting_assembly.stl` (print 1x), `blade_with_root_fitting.stl` (print 2x; blade 2 is the same part rotated 180 deg about the rotor axis), `drawing.png`; generator `../build_7in_v4.py`.
Coordinates mm, hub frame, z up = build direction = rotor axis. Strips crossed at 151.27 deg, skew 59.7 deg.

## Cut list (bought parts)
| Item | Spec | Qty |
|---|---|---|
| Spring-steel shim strip, 1095, 0.15 mm | **9.0 x 28.7 mm** = 9.0 wide x (18.7 free + 2 x 5.0 clamp lands); cut 6 (4 + 2 spare) | 4 |
| Constrained-layer damper (design assumes it): 3M VHB 9469 0.13 mm + 0.05 mm shim on each strip free length, ending 1 mm short of the lands | 9.0 x 16.7 mm | 4 |
| Aramid strand 1.5 mm | **40 mm** per blade: 4 free + 2 x 8 channel + 2 x 8 knot/tail allowance; one continuous strand hub side to blade side | 2 |
| Glue | Strips: structural epoxy, thin film (0.05 mm bondline). CA only to tack (brittle, creeps on steel/PA). Knot wells: thin CA then epoxy seal | |
| Balance | Balance blade pair on a pin balancer; add putty/CA at tip or file the root until the blades hold level; tip mass difference < 0.05 g | |

Strip prep: tin snips, with the strip LONG axis parallel to the rolling (grain) direction, so bend lines are perpendicular to the rolling direction. Cut ~9.3 wide, file to 9.0. Deburr ALL four edges (400 grit, stroke along the length, break corners ~0.1 mm); a burr at the clamp exit defeats the radiused jaw and starts the fatigue crack. Degrease with IPA, scuff only the glued lands.

## Slots and radii (in the STLs)
- Strip slot 0.20 mm (0.15 shim + 0.05 glue/press allowance) x 9.2 mm (0.1 mm per side); clamp land 5.0 mm per strip end.
- Jaw exits RADIUSED R = 1.5 mm both faces (design `strip_clamp: radiused`; `strip_clamp_radius` should be set to 0.0015 in the design JSON).
- Free-span clearance window 3.2 mm (strip +-1.6) x 9.2 mm over the rest of each strip span in both fittings.
- Strand channel dia 1.8 mm, 8 mm deep each side of the crossing, ending in a dia 4.0 mm knot well open to +z.
- Crossing gap 1.9 mm (strand + 0.4 mm) is set by the strip positions in the model; hold it with a 1.9 mm shim while the epoxy cures.

## Print settings
- PA6-CF (design `pa6_cf`), dried 6+ h at 80 C. PETG-CF/PC-CF for a first fit test only.
- Orientation as exported (z up): raster lies in XY, so the blade pull along the rotor-plane x axis runs along the raster lines; no layer interface is loaded in peel at the slot.
- 0.4 nozzle, 0.12 mm layers, 6 perimeters (>= 2.4 mm walls), 100 % infill, no supports (knot well is an open vertical hole).
- The 0.20 mm slot is ~1.7 layers: check with a spare strip; scrape with a fresh blade if tight, do not drill.
- Pauses: none (nothing embedded; strips are glued in afterwards).

## Assembly
1. Dry test-fit deburred strips in both slots (light finger pressure).
2. Per blade, strip end 0 goes into the hub fitting and end 1 into the blade root fitting, strips crossing at 151.27 deg (see drawing.png).
3. Thin epoxy film on the 5 mm lands, insert, hold straight with the 1.9 mm shim between the crossed strips until cured, then remove the shim.
4. Fit damper layers on the free lengths, 1 mm short of the lands.
5. Thread the strand through both channels, stopper knot in each well, strand just taut at rest, CA the knots then epoxy seal.
6. Repeat for blade 2; balance the blade pair, then the assembled rotor.

## Pre-flight checks
- Static pitch preset: design `pre_lag auto` on the pitch axis resolves to +2.86 deg per blade (model, resolve_pre_lag). Measure each blade at rest with a phone inclinometer on the chord at 0.5R vs the hub plane; match the blades within +-0.3 deg.
- Free lag swing: each blade swings freely by hand and returns smoothly, no rubbing on strand or fitting walls, same feel on both blades.
- Tap/pluck: clamp the hub, tap a blade, record with a phone spectrum app. Analytic clamped-clamped bending of the 18.7 mm free span is ~2 kHz (E x1.105, rho x1.44 with damper). The model's hinge/pitch mode is a low rigid-body mode (a few per rev); compare the lowest tonal peak (< ~200 Hz) with the model value from the analysis output. More than 15 % off means a loose clamp or wrong strip dimensions. (I did not run the model to extract that number.)
- Hardware rule (study/campaign/LOG.md): make the motor-arm mount in-plane stiffness ISOTROPIC (x ~ y) or soft-damped. A 2-blade rotor on an anisotropic support has a hub-dominated 2/rev parametric air-resonance window that strip damping cannot cure. Tap-test the arm in x and y (tones within ~5 %), add a rubber/TPU damped mount if not.

## Approximations and limits
- Fittings are the lofted envelope solids from geometry.py plus boolean slot/window/channel cuts. The CAD hub and root fitting solids overlap by ~62 mm3 (loft artefact): verify the fit and file as needed.
- Strap anchor `pin_wrap` (6 mm pin) replaced by a stopper-knot well; knot anchor strength is not modelled.
- STL files have a few pinch edges where thin loft slivers touch (see the report); slicers normally repair this silently.
