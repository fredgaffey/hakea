# BUILD SHEET: 7in_tt_strap_steelx_v5 (2-blade, crossed 1095 strips, aramid strand)

Files (this folder): `hub_fitting_assembly.stl` (print 1x), `blade1_with_root_fitting.stl` and `blade2_with_root_fitting.stl` (print 1x each; blade 2's flexure and root fitting are MIRRORED in z, its airfoil is not, so it is a different part), `drawing.png`, `parts.json`; generator `../build_7in_v5.py` (parts from `flexblade/fittings.py`). All STLs watertight, one shell each (parts.json).
Coordinates mm, hub frame, z up = build direction = rotor axis. Strips crossed at **105.16 deg**, skew 59.91 deg.

## Cut list (bought parts)
| Item | Spec | Qty |
|---|---|---|
| Spring-steel shim strip, 1095, 0.15 mm | **6.97 wide x 36.0 mm long** = 14.92 free + 2 x 10.6 clamp lands; cut 6 (4 + 2 spare) | 4 |
| Constrained-layer damper (the design assumes it, eta 0.0298): 3M VHB 9469 0.13 mm + 0.05 mm shim on each strip free length, ending 1 mm short of the jaw exits | 6.97 x 12.9 mm | 4 |
| Aramid strand 1.5 mm | **40 mm** per blade: 4 free + 2 x 8 channel + 2 x 8 knot/tail; one continuous strand hub side to blade side | 2 |
| Glue | Strips: structural epoxy, thin film. CA only to tack. Knot wells: thin CA then epoxy seal | |
| Balance | Pin-balance the blade pair; tip mass difference < 0.05 g | |

Strip prep: tin snips, strip LONG axis parallel to the rolling (grain) direction. Cut ~0.3 mm wide, file to 6.97 (the strip is narrower than v4: measure with calipers, tolerance +-0.05 mm; width is the most sensitive dimension of the flexure stiffness, k_hinge ~ w). Deburr all four edges (400 grit along the length, break ~0.1 mm). Degrease with IPA, scuff only the glued lands.

## Slots and radii (in the STLs)
- Free span 14.92 mm between hub and root jaw exits; clamp land 10.6 mm per strip end; slot 0.20 mm x 7.2 mm; jaw exits radiused R = 1.50 mm.
- Cross gap 1.9 mm between the two strip planes: hold this with a 1.9 mm shim while the epoxy cures.
- Clearance (rigid sweep, lag +-5 deg x pitch +-8 deg): rest gap 4.08 mm, minimum 3.82 mm.
- Printed fitting mass (PA6-CF, per blade, in the rotor model): hub fitting 1.49 g, root fitting 1.56 g (fitting_slim webs). Hub assembly STL total 7.05 g.

## Print settings
PA6-CF dried 6+ h at 80 C; z up as exported; 0.4 nozzle, 0.12 layers, 6 perimeters, 100 % infill, no supports. The slot is ~1.7 layers: check with a spare strip, scrape with a fresh blade, do not drill.

## Assembly
1. Dry test-fit deburred strips in both slots.
2. Per blade, push each strip from behind through the hub jaw, across the free span, into the blind root slot, strips crossing at 105.16 deg (drawing.png).
3. Thin epoxy on the lands, hold the 1.9 mm shim between the strips until cured, remove it.
4. Fit damper layers 1 mm short of the lands.
5. Thread the strand, stopper knots, strand just taut at rest, CA then epoxy seal.
6. Repeat for blade 2; balance the blade pair, then the assembled rotor.

## Pitch preset (static, per blade)
`pre_lag auto` on the pitch axis resolves (resolve_pre_lag, T1 hover) to **-0.115 deg per blade**, i.e. essentially zero: no deliberate preset is needed in v5 (v4 needed +2.86 deg). Build both blades at 0 +- 0.3 deg: phone inclinometer on the chord at 0.5R vs the hub plane, blades matched to +-0.3 deg. If one blade is off by more than that, re-seat the root jaw before gluing, do not bend the strips.

## Predicted modes for the pluck / tap test (Hz; model, nominal strips, damper, real fitting mass)
At rest (0 rpm, aero off; hub treated as the model hub, tap with the arm clamped and the hub as stiff as you can make it):

| Mode | T0.7 | T1 | T1.4 |
|---|---|---|---|
| pitch sym | 34, 963, 1092 | 34, 963, 1092 | 34, 963, 1092 |
| pitch antisym | 1038 | 1038 | 1038 |
| pitch antisym+hub | 49 | 49 | 49 |
| flap sym | 249 | 249 | 249 |
| flap antisym | 255 | 255 | 255 |

At hover rpm (aero on, trim rpm per thrust level T = 0.7 / 1.0 / 1.4 x weight, centrifugal stiffening included; only observable by an accelerometer or a mic during a spin-up, or by the throttle sweep below):

| Mode | T0.7 (11005 rpm) | T1 (13132 rpm) | T1.4 (15684 rpm) |
|---|---|---|---|
| pitch sym | 111, 612, 1003, 1106 | 131, 626, 828, 1125 | 155, 314, 628, 1147 |
| pitch antisym | - | - | 671 |
| pitch antisym+hub | 156 | 185 | 219 |
| flap sym | 333 | 351 | 343 |
| flap antisym | 259 | 271 | - |

Notes for the bench test: the 'pitch' modes are the flexure hinge / torsion modes of the crossed strips (the skewed hinge couples pitch and lag, so there is no separate low lag mode: the hinge mode IS the 'pitch sym' line). 'pitch antisym+hub' is the differential mode that also moves the hub in-plane, so it depends on the arm mount stiffness. Flap modes at rest (~250 Hz) are the strip bending on the free span plus blade root. Tolerance: model values assume 0.15 mm / 6.97 mm strips; more than 15 % off in the pitch-sym line (~35 Hz) means a loose clamp or wrong width. Frequency scales as sqrt(k): width +5 % = +2.4 % frequency. Pluck the blade tip chordwise (excites pitch) and flapwise (excites flap), record 5 s with a phone spectrum app, read the lowest peak.

## Airframe isotropy requirement
Make the motor-arm mount in-plane stiffness ISOTROPIC (x ~ y within 5 %) or soft-damped. A 2-blade rotor on an anisotropic support has a hub-dominated 2/rev parametric air-resonance window that strip damping cannot cure. Tap-test the arm in x and y (tones within ~5 %); fly with a rubber/TPU damped mount if not. The v5 Floquet check (verify_7in_v5.md) is with f_y = 1.15 f_x as a worst realistic anisotropy; the airframe damping assumed is 0.02 minimum.

## What to measure to calibrate the nP aero forcing (blade-root accelerometer, slow throttle sweep)
The forced-response criterion (forced.py) assumes torque harmonics 3 % at 2P, 1 % at 3P, aero nP loads ~5-10 % of mean lift moment at 2P falling ~1/n^2. These are assumptions; measure:
1. Mount a 3-axis MEMS accelerometer (>= 4 kHz sample rate, e.g. ADXL372 or a logging IMU in the blade-root fitting, or on the arm near the hub as a fall-back) on one blade root; log with a telemetry-free logger on the rotor or a slip-ring.
2. Slow throttle sweep: ramp 40 % to 100 % of hover throttle over >= 60 s with the vehicle restrained (thrust stand or tethered), so every nP line crosses every mode slowly (rpm range ~ 11000-15700). Log rpm from ESC telemetry (or the 1P line itself) and motor current.
3. Spectrogram vs rpm: read the 1P, 2P, 3P, 4P amplitudes, and the amplification peaks where nP meets a mode of the table above (predicted: e.g. 2P at 438 Hz at T1 rpm against the pitch antisym+hub line at 185 Hz).
4. Per crossing record amplitude A_pk (g) and the half-power width of the resonance peak, from which zeta = df/(2 f) (compare with the model zeta). Ratio of the nP acceleration far from any crossing to the 1P acceleration gives the forcing fraction to compare with the 3 % / 1 % assumptions.
5. Repeat with the motor current harmonic spectrum logged (shunt + scope/ADC) to separate the electrical (torque) nP source from aero nP: if the root acceleration at nP follows the current nP amplitude it is torque; if it follows thrust / tilt it is aero. Then test the aero part by tilting the rotor disc (a few degrees) or running near a wall and see the 2P scale.
6. Pitch jitter: if possible a hall/optical pitch pickup on one blade; the criterion limit is pitch jitter in degrees, accelerometer double-integration is not reliable.

## Approximations and limits
- Strap anchor `pin_wrap` replaced by a stopper-knot well; knot strength not modelled. Clearance is a rigid-body sweep; strip S-bending not included.
- ccx cross-check of the strip pair (study/verify_v5.py ccx): all terms within 6 % of the beam model.
