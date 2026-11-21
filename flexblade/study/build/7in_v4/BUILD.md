# BUILD SHEET: 7in_tt_strap_steelx_v4 (2-blade, crossed 1095 strips, aramid strand)

Files (this folder): `hub_fitting_assembly.stl` (print 1x), `blade1_with_root_fitting.stl` and `blade2_with_root_fitting.stl` (print 1x each: blade 2's flexure and root fitting are MIRRORED in z, its airfoil is not, so it is a different part), `drawing.png`, `parts.json` (dims, masses, clearance); generator `../build_7in_v4.py` (parts from `flexblade/fittings.py`). All STLs watertight, one shell each.
Coordinates mm, hub frame, z up = build direction = rotor axis. Strips crossed at 151.27 deg, skew 59.7 deg.

## Cut list (bought parts)
| Item | Spec | Qty |
|---|---|---|
| Spring-steel shim strip, 1095, 0.15 mm | **9.0 x 42.7 mm** = 9.0 wide x (18.73 free + 2 x 12.0 clamp lands); cut 6 (4 + 2 spare) | 4 |
| Constrained-layer damper (design assumes it): 3M VHB 9469 0.13 mm + 0.05 mm shim on each strip free length, ending 1 mm short of the jaw exits | 9.0 x 16.7 mm | 4 |
| Aramid strand 1.5 mm | **40 mm** per blade: 4 free + 2 x 8 channel + 2 x 8 knot/tail allowance; one continuous strand hub side to blade side | 2 |
| Glue | Strips: structural epoxy, thin film (0.05 mm bondline). CA only to tack (brittle, creeps on steel/PA). Knot wells: thin CA then epoxy seal | |
| Balance | Balance blade pair on a pin balancer; add putty/CA at tip or file the root until the blades hold level; tip mass difference < 0.05 g | |

Strip prep: tin snips, with the strip LONG axis parallel to the rolling (grain) direction, so bend lines are perpendicular to the rolling direction. Cut ~9.3 wide, file to 9.0. Deburr ALL four edges (400 grit, stroke along the length, break corners ~0.1 mm); a burr at the clamp exit defeats the radiused jaw and starts the fatigue crack. Degrease with IPA, scuff only the glued lands.

## Slots and radii (in the STLs; flexblade/fittings.py)
- Jaw exit faces sit exactly on the modelled strip end points: free span 18.73 mm between hub and root jaws.
- Clamp land 12.0 mm per strip end (max(4 sqrt(w_mm), 5)); slot 0.20 mm (0.15 shim + 0.05 glue/press) x 9.2 mm (0.1 per side).
- Crossing edge gap is 1.9 mm, less than two 1.2 mm walls, so each jaw is a U jaw: slot open on the crossing edge, strip edge overhangs the jaw by 0.5 mm there. Both faces are clamped over the full land.
- Jaw exits RADIUSED R = 1.5 mm (10 t, design `strip_clamp: radiused`, `strip_clamp_radius` 0 -> default 10 t) on both faces; jaw walls >= 1.2 mm beyond the radius.
- Hub slots run THROUGH the back of the jaw (strip inserted from behind); root-fitting slots are blind (land + 0.3 mm).
- Webs stay > 2 mm beyond the strip exit plane; the strand posts stay 0.5 mm clear of both strip planes. A strip +-0.5 mm keep-out is cut from both fittings over the free span.
- Strand channel dia 1.8 mm, 8 mm from each strand end, ending in a dia 3.75 mm knot well open to +z (hub: inside the hub at r = 5.2 mm; blade: at r = 25.2 mm through the root web/shank).
- Clearance (rigid sweep of the root fitting, lag +-5 deg about the hinge axis x pitch +-8 deg about x): rest gap 2.9 mm, minimum 1.50 mm at lag +5 / pitch +8.

## Print settings
- PA6-CF (design `pa6_cf`), dried 6+ h at 80 C. PETG-CF/PC-CF for a first fit test only.
- Orientation as exported (z up): raster lies in XY, so the blade pull along the rotor-plane x axis runs along the raster lines; no layer interface is loaded in peel at the slot.
- 0.4 nozzle, 0.12 mm layers, 6 perimeters (>= 2.4 mm walls), 100 % infill, no supports (knot well is an open vertical hole).
- The 0.20 mm slot is ~1.7 layers: check with a spare strip; scrape with a fresh blade if tight, do not drill.
- Pauses: none (nothing embedded; strips are glued in afterwards).

## Assembly
1. Dry test-fit deburred strips in both slots (light finger pressure).
2. Per blade, push each strip from behind through the hub jaw (through slot), across the free span, into the blind root slot (end 1), strips crossing at 151.27 deg (see drawing.png).
3. Thin epoxy film on the 12 mm lands, insert, hold straight with the 1.9 mm shim between the crossed strips until cured, then remove the shim.
4. Fit damper layers on the free lengths, 1 mm short of the lands.
5. Thread the strand through both channels, stopper knot in each well, strand just taut at rest, CA the knots then epoxy seal.
6. Repeat for blade 2; balance the blade pair, then the assembled rotor.

## Pre-flight checks
- Static pitch preset: design `pre_lag auto` on the pitch axis resolves to +2.86 deg per blade (model, resolve_pre_lag). Measure each blade at rest with a phone inclinometer on the chord at 0.5R vs the hub plane; match the blades within +-0.3 deg.
- Free lag swing: each blade swings freely by hand and returns smoothly, no rubbing on strand or fitting walls, same feel on both blades.
- Tap/pluck: clamp the hub, tap a blade, record with a phone spectrum app. Analytic clamped-clamped bending of the 18.7 mm free span is ~2 kHz (E x1.105, rho x1.44 with damper). The model's hinge/pitch mode is a low rigid-body mode (a few per rev); compare the lowest tonal peak (< ~200 Hz) with the model value from the analysis output. More than 15 % off means a loose clamp or wrong strip dimensions. (I did not run the model to extract that number.)
- Hardware rule (study/campaign/LOG.md): make the motor-arm mount in-plane stiffness ISOTROPIC (x ~ y) or soft-damped. A 2-blade rotor on an anisotropic support has a hub-dominated 2/rev parametric air-resonance window that strip damping cannot cure. Tap-test the arm in x and y (tones within ~5 %), add a rubber/TPU damped mount if not.

## Approximations and limits
- Printed fitting masses (PA6-CF): hub fitting 3.4 g per blade, root fitting 3.5 g per blade (jaws + web + post; shank/airfoil extra). The rotor model carries only a 0.08 g shank beam for the blade-side hardware and no hub-fitting mass: the real blade root/hub inertia is much higher than modelled, so re-run the Campbell / stability analysis with these masses (or slim the webs) before flying.
- Strap anchor `pin_wrap` (6 mm pin) replaced by a stopper-knot well; knot anchor strength is not modelled.
- Clearance is a rigid-body sweep of the fittings; strip S-bending and strand slack are not included.
