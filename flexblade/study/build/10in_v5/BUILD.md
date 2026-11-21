# BUILD SHEET: 10in_tt_strap_steelx_v5 (2-blade, crossed 1095 strips, aramid strand)

Files (this folder): `mount_tube_clamp.stl`, `mount_tpu_washer.stl`, `mount_carrier.stl` (make_mount.py),  `hub_fitting_assembly.stl` (print 1x), `blade1_with_root_fitting.stl` and `blade2_with_root_fitting.stl` (print 1x each; blade 2's flexure and root fitting are MIRRORED in z, its airfoil is not, so it is a different part), `drawing.png`, `parts.json`; generator `../build_10in_v5.py` (parts from `flexblade/fittings.py`). All STLs watertight, one shell each (parts.json).
Coordinates mm, hub frame, z up = build direction = rotor axis. Strips crossed at **105.00 deg**, skew 60.00 deg.

## Cut list (bought parts)
| Item | Spec | Qty |
|---|---|---|
| Spring-steel shim, 1095, **0.10 mm** (0.004"), laminae of one strip | **9.00 wide x 49.1 mm long** = 25.12 free + 2 x 12.0 clamp lands. 4 strips x **8 laminae = 32 laminae**; cut 40 (8 spare) | 32 |
| Constrained-layer damper (the design assumes it, eta 0.0298): 3M VHB 9469 0.13 mm + 0.05 mm shim on each strip free length, ending 1 mm short of the jaw exits | 9.00 x 23.1 mm | 4 |
| Aramid strand 1.5 mm | **40 mm** per blade: 4 free + 2 x 8 channel + 2 x 8 knot/tail; one continuous strand hub side to blade side | 2 |
| Glue | Strips: structural epoxy, thin film. CA only to tack. Knot wells: thin CA then epoxy seal | |
| Balance | Pin-balance the blade pair; tip mass difference < 0.05 g | |

Strip prep (per lamina): tin snips, strip LONG axis parallel to the rolling (grain) direction. Cut ~0.3 mm wide, file to 9.00 (v5 strip: 9 mm wide, 25.12 mm free; measure with calipers, tolerance +-0.05 mm; width is the most sensitive dimension of the flexure stiffness, k_hinge ~ w). Deburr all four edges (400 grit along the length, break ~0.1 mm). Degrease with IPA, scuff only the glued lands.

## Slots and radii (in the STLs)
- Free span 25.12 mm between hub and root jaw exits; clamp land 12.0 mm per strip end; slot 0.85 mm x 9.2 mm; jaw exits radiused R = 1.00 mm.
- Cross gap 2.7 mm between the two strip planes: hold this with a 2.7 mm shim while the epoxy cures.
- Clearance (rigid sweep, lag +-5 deg x pitch +-8 deg): rest gap 6.61 mm, minimum 6.50 mm.
- Printed fitting mass (PA6-CF, per blade, in the rotor model): hub fitting 2.54 g, root fitting 2.58 g (fitting_slim webs). Hub assembly STL total 17.73 g.

## Print settings
PA6-CF dried 6+ h at 80 C; z up as exported; 0.4 nozzle, 0.12 layers, 6 perimeters, 100 % infill, no supports. The slot is ~1.7 layers: check with a spare strip, scrape with a fresh blade, do not drill.

## Assembly
1. Dry test-fit deburred strips in both slots.
2. Per blade, push each strip from behind through the hub jaw, across the free span, into the blind root slot, strips crossing at 105.00 deg (drawing.png).
3. Thin epoxy on the lands, hold the 2.7 mm shim between the strips until cured, remove it.
4. Fit damper layers 1 mm short of the lands.
5. Thread the strand, stopper knots, strand just taut at rest, CA then epoxy seal.
6. Repeat for blade 2; balance the blade pair, then the assembled rotor.

## Pitch preset (static, per blade)
`pre_lag auto` on the pitch axis resolves (resolve_pre_lag, T1 hover) to **+3.976 deg per blade** (v4 needed +2.86 deg). Build both blades at this value +- 0.3 deg: phone inclinometer on the chord at 0.5R vs the hub plane, blades matched to +-0.3 deg. If one blade is off by more than that, re-seat the root jaw before gluing, do not bend the strips.

## Predicted modes for the pluck / tap test (Hz; model, nominal strips, damper, real fitting mass)
At rest (0 rpm, aero off; hub treated as the model hub, tap with the arm clamped and the hub as stiff as you can make it):

| Mode | T0.7 | T1 | T1.4 |
|---|---|---|---|
| pitch sym | 19, 775, 841 | 19, 775, 841 | 19, 775, 841 |
| pitch antisym | 799 | 799 | 799 |
| pitch antisym+hub | 31 | 31 | 31 |
| flap sym | 189 | 189 | 189 |
| flap antisym | 190 | 190 | 190 |

At hover rpm (aero on, trim rpm per thrust level T = 0.7 / 1.0 / 1.4 x weight, centrifugal stiffening included; only observable by an accelerometer or a mic during a spin-up, or by the throttle sweep below):

| Mode | T0.7 (6456 rpm) | T1 (7687 rpm) | T1.4 (9093 rpm) |
|---|---|---|---|
| pitch sym | 62, 473, 603 | 74, 523 | 88, 177, 508 |
| pitch antisym | - | 560 | 581 |
| pitch antisym+hub | 100 | 119 | 141 |
| flap sym | 144, 218, 589 | 156 | - |
| flap antisym | - | 227 | 234 |

Notes for the bench test: the 'pitch' modes are the flexure hinge / torsion modes of the crossed strips (the skewed hinge couples pitch and lag, so there is no separate low lag mode: the hinge mode IS the 'pitch sym' line). 'pitch antisym+hub' is the differential mode that also moves the hub in-plane, so it depends on the arm mount stiffness. Flap modes at rest (~190 Hz) are the strip bending on the free span plus blade root. Tolerance: model values assume 8 x 0.10 mm / 9.00 mm laminae; more than 15 % off in the pitch-sym line (~19 Hz at rest, 62-88 Hz at hover) means a loose clamp or wrong width. Frequency scales as sqrt(k): width +5 % = +2.4 % frequency. Pluck the blade tip chordwise (excites pitch) and flapwise (excites flap), record 5 s with a phone spectrum app, read the lowest peak.

## Airframe isotropy requirement
Make the motor-arm mount in-plane stiffness ISOTROPIC (x ~ y within 5 %) or soft-damped. A 2-blade rotor on an anisotropic support has a hub-dominated 2/rev parametric air-resonance window that strip damping cannot cure. Tap-test the arm in x and y (tones within ~5 %); fly with a rubber/TPU damped mount if not. The v5 Floquet check (verify_10in_v5.md) is with f_y = 1.15 f_x as a worst realistic anisotropy; the airframe damping assumed is 0.02 minimum.

## What to measure to calibrate the nP aero forcing (blade-root accelerometer, slow throttle sweep)
The forced-response criterion (forced.py) assumes torque harmonics 3 % at 2P, 1 % at 3P, aero nP loads ~5-10 % of mean lift moment at 2P falling ~1/n^2. These are assumptions; measure:
1. Mount a 3-axis MEMS accelerometer (>= 4 kHz sample rate, e.g. ADXL372 or a logging IMU in the blade-root fitting, or on the arm near the hub as a fall-back) on one blade root; log with a telemetry-free logger on the rotor or a slip-ring.
2. Slow throttle sweep: ramp 40 % to 100 % of hover throttle over >= 60 s with the vehicle restrained (thrust stand or tethered), so every nP line crosses every mode slowly (rpm range ~ 6500-9100). Log rpm from ESC telemetry (or the 1P line itself) and motor current.
3. Spectrogram vs rpm: read the 1P, 2P, 3P, 4P amplitudes, and the amplification peaks where nP meets a mode of the table above (predicted: e.g. 2P at 256 Hz at T1 rpm against the pitch antisym+hub line at 119 Hz).
4. Per crossing record amplitude A_pk (g) and the half-power width of the resonance peak, from which zeta = df/(2 f) (compare with the model zeta). Ratio of the nP acceleration far from any crossing to the 1P acceleration gives the forcing fraction to compare with the 3 % / 1 % assumptions.
5. Repeat with the motor current harmonic spectrum logged (shunt + scope/ADC) to separate the electrical (torque) nP source from aero nP: if the root acceleration at nP follows the current nP amplitude it is torque; if it follows thrust / tilt it is aero. Then test the aero part by tilting the rotor disc (a few degrees) or running near a wall and see the 2P scale.
6. Pitch jitter: if possible a hall/optical pitch pickup on one blade; the criterion limit is pitch jitter in degrees, accelerometer double-integration is not reliable.

## Approximations and limits
- Strap anchor `pin_wrap` replaced by a stopper-knot well; knot strength not modelled. Clearance is a rigid-body sweep; strip S-bending not included.
- ccx cross-check of the strip pair (verify_10in_v5.py ccx): per-lamina ccx x 8 within 6 % of the 8-lamina beam model; lamina slip/contact not modelled in ccx.

## Lamina stack (8 x 0.10 mm per strip, UNBONDED in the free span)
The model carries 8 independent laminae (stiffness 8 x one lamina, ccx per-lamina x 8 within 6 %), stacked 0.80 mm in a 0.85 mm slot (0.05 mm allowance: tight, test-fit a spare stack and a spare slot first). Stack the 8 laminae with a thin film of light oil (3-in-1 / sewing-machine oil) between them over the FREE span only; epoxy ONLY on the 12 mm clamp lands (wet the whole stack in the land, clamp flat, wipe oil off the land first). Keep oil off the lands. Do not bond the free span: a bonded stack is 64x stiffer in bending than 8 sliding laminae and will fail the fatigue/pitch design. Damper layer (VHB 0.13 mm + 0.05 mm shim) goes over the stack, 1 mm short of the jaw exits. Cut all laminae from the same sheet, same rolling direction, matched widths +-0.05 mm (a width spread of 5 % is inside the 8/9 tolerance run; width is the most sensitive dimension).

## Damped isotropic motor mount (air-resonance margin)
Parts (make_mount.py, STLs in this folder, all watertight): `mount_tube_clamp.stl` (split clamp for a 10 mm tube, bore 10.1, M3 pinch bolt, 22x24x20 block, post d 7.8 with 18 mm flange, M4 tap), `mount_tpu_washer.stl` (TPU 85A, ri 4 / ro 9 / t 10 mm, 40 % gyroid so the in-plane shear stiffness is 53 kN/m; print a trial and trim), `mount_carrier.stl` (30x30x4 plate, motor holes M3 on 16x16 and 19x19, central d 9 hole, collar lip d 23; the 3115 motor pattern is 19x19). Stack: clamp flange / washer / carrier; M4 screw with large washer 1 mm above the carrier is the bump stop, not preloaded. Washer axis = motor axis so k_x = k_y by construction. The 10 mm tube is the 7in clamp: check the 1.7 kg vehicle's arm tube; for a bigger tube scale the clamp block only, not the washer.
10in sizing (study/mount_calc_10in.py): rotating mass on the mount 124 g (motor ~90 + carrier 12 + hub 17.7 + screws), arm modal mass 0.25 x 1.697 kg = 0.424 kg, k_m = 53 kN/m, eta 0.15 (TPU tan-delta 0.1-0.2, VERIFY on a printed sample). Hub mode **99-102 Hz at zeta_af 0.068-0.072** (arm mode 238-317 Hz at 0.023-0.027 for f_arm 190-260). Tap test: motor bell on the carrier rings ~100 Hz, x and y equal within 5 %.
Caveat: the 10in 1P is 108 Hz (T0.7, 6456 rpm), 128 Hz (T1), 152 Hz (T1.4), so the mount hub mode sits just under 1P at low throttle. Hub 1P amplitude for 2 g mm unbalance: 106 um at 108 Hz, 45 um at 128 Hz, 30 um at 152 Hz: BALANCE THE ROTOR to < 0.5 g mm (pin balance + hub spin balance) or the low-throttle 1P hub motion is large. A softer washer (k x 0.4: ro 6.5 mm, same ri/t, or t 25 mm) puts the hub mode at ~66 Hz and cuts the 108 Hz amplitude to 25 um, but that variant is not Floquet-checked.
**v5b UPDATE (supersedes the 53 kN/m single washer above, which put the hub mode at 99-102 Hz, 0.93 x 1P).** Decoupled twin-washer mount: TWO TPU 85A washers, each ri 4 / ro 5.8 / t 12 mm (40 % gyroid), centres 30 mm apart: carrier (`mount_carrier.stl`, plate + 46 mm cup skirt) and post (`mount_post.stl`, rod d 7.8 + two d 12 flanges, 16 mm web) with M4 preload so both washers carry thrust (print 2x `mount_tpu_washer.stl`, 1x `mount_post.stl`, retain with a ring; `mount_tube_clamp.stl` unchanged, its M4 tap takes the post). Numbers (study/mount_soft_10in.py): k_lat 24.0 kN/m, hub mode 69-70 Hz (zeta 0.072), 0.65 x the 108 Hz 1P; tilt k 19.4 N m/rad -> 0.5 deg at 0.17 N m, 0.9 deg at 0.3 N m (a single soft washer ro 6.5 t 10 would tilt 9-16 deg: do not use); sag 0.05 mm per 1g lateral, 0.15 mm at 3g, 0.20 mm axial at 16.65 N hover thrust; 2 g mm unbalance gives 27 / 23 / 20 um at 108 / 128 / 152 Hz; Floquet min zeta +0.0063 (T0.7), +0.0087 (T1), +0.0127 (T1.4). Bench: motor bell rings ~70 Hz, x = y within 5 %; the cup hangs 42 mm below the motor plate so check arm clearance. Tilt mode (rotating inertia ~8e-5 kg m^2) is an estimated ~75 Hz, near the hub mode: verify on the bench. eta 0.15 is unmeasured.
Costs: tilt stiffness ~4.4 N m/rad: the 0.0975 N m cyclic moment tilts the disc ~1.3 deg relative to the arm (steady); axial sag ~92 um at 16.65 N hover thrust.

## Minimum-inertia hub build (J_hub itemised)
Direct-to-bell clamp: the printed hub assembly is screwed straight onto the motor bell (no nut, no adapter). Itemised polar inertia about the rotor axis (bell stays in J_rotor):
| item | mass | radius | J (kg m^2) |
|---|---|---|---|
| printed hub assembly (hub_fitting_assembly.stl, PA6-CF, Izz of the STL) | 17.73 g | - | 3.01e-6 |
| 4 hub-side strip lands, 8 laminae of 0.10 x 9 x 12 mm steel each, 2 per blade side | 4 x 0.68 g | 12.6-22.7 mm (land centre to axis) | 0.44e-6 |
| 4x M3x6 socket screws into the bell | 4 x 0.5 g | 8 mm | 0.13e-6 |
| thread-locker / knots / tape allowance | ~0.2 g | 8 mm | 0.01e-6 |
| total J_hub | | | **3.59e-6** |

The schedule below is computed at J_hub 2.5, 3.76 and 5.49e-6 (the 3.76e-6 column is the v4 itemisation; the v5 itemised value 3.59e-6 lies between the 2.5 and 3.76 columns; use the smaller pulse of the bracketing rows). Not verified below 2.5e-6 or above 5.49e-6.

## Bifilar measurement of J_hub
Hang the motor bell + printed hub (no blades, screws fitted) from two parallel threads of length l, separation 2a (a ~ 30 mm, l ~ 0.5 m); twist about the vertical axis and time 20+ oscillations (period T): J = m g a^2 T^2 / (4 pi^2 l), m = bell + hub mass. Repeat with the bell alone (3115 bell, expected J_rotor ~2.2e-5 per the design motor): J_hub = J(bell+hub) - J(bell). Photogate or slow-mo video. Accept J_hub within the itemised value +- 0.5e-6; the design is feasible 2.5 to 5.49e-6.

## Pluck-test mode table
See "Predicted modes" above (rest and hover tables from the v5 model). Pass criteria: lowest pitch-sym line (~19 Hz at rest) within 15 %, antisym+hub line (31 Hz at rest, mount-dependent: clamp the arm stiffly) within 20 %, flap sym/antisym (~190 Hz) within 10 %; frequency scales as sqrt(k), strip width +5 % = +2.4 %. Pluck the tip chordwise (pitch) and flapwise (flap), 5 s phone spectrum. Hover line to avoid: pitch antisym+hub 100 / 119 / 141 Hz against 1P 108 / 128 / 152 Hz (ratio 0.93, 0.93, 0.93; the resonance check requires |1 - ratio| margin >= 0.15 from the effective frequency including damping; see verify_10in_v5.md).

## FC pulse schedule (cyclic pulse amplitude vs throttle and measured J_hub)
The stored design op pulse is the original; the FC applies this schedule (Lua in-flight calibrated 1P phase; boot reference on the ESC). Pulse scale is relative to 0.95 x the motor's capability pulse at that rpm; values above 1 x mod_frac_max are fine because the motor_torque_1p check on the delivered torque is the real cap. Interpolate linearly in throttle and J_hub between rows; between columns use the smaller pulse of the two bracketing rows. Fatigue binds at low and mid throttle, the motor 1P torque at T1.4. Robust cr_pl (min over all) = **0.0822**; tolerance (nominal + 8 random +-5 % on t, w, L, hinge r, rho, strap, skew +-1 deg, EA +-0.01) **8/9 pass** (study/results/steelx_10in_v5.md; random07 fails the resonance check, margin 0.144).

| throttle (thrust x weight) | J_hub | pulse, x 0.95 capability | pulse, x mod_frac_max | control ratio at that pulse | cr_pl | binding | 1P resonance margin (need 0.15) |
|---|---|---|---|---|---|---|---|
| 0.7 | 3.76e-06 | 0.607 | 0.576 | 0.2968 | 0.1801 | fatigue_stress | 0.177 |
| 0.7 | 2.50e-06 | 0.566 | 0.538 | 0.3181 | 0.1801 | fatigue_stress | 0.163 |
| 0.7 | 5.49e-06 | 0.664 | 0.631 | 0.2711 | 0.1801 | fatigue_stress | 0.196 |
| 1 | 3.76e-06 | 0.831 | 0.790 | 0.1452 | 0.1207 | fatigue_stress | 0.188 |
| 1 | 2.50e-06 | 0.791 | 0.752 | 0.1525 | 0.1207 | fatigue_stress | 0.177 |
| 1 | 5.49e-06 | 0.888 | 0.844 | 0.1359 | 0.1207 | fatigue_stress | 0.205 |
| 1.4 | 3.76e-06 | 1.150 | 1.092 | 0.0748 | 0.0860 | motor_torque_1p | 0.173 |
| 1.4 | 2.50e-06 | 1.150 | 1.092 | 0.0774 | 0.0890 | motor_torque_1p | 0.178 |
| 1.4 | 5.49e-06 | 1.150 | 1.092 | 0.0715 | 0.0822 | motor_torque_1p | 0.167 |

Procedure: measure J_hub (bifilar), pick the matching J column, program the pulse for each throttle.
