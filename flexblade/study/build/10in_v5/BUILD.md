# BUILD SHEET: 10in_tt_strap_steelx_v5 (2-blade, crossed 1095 strips, aramid strand)

Files (this folder): `mount_tube_clamp.stl`, `mount_tpu_washer.stl`, `mount_carrier.stl` (make_mount.py),  `hub_fitting_assembly.stl` (print 1x), `blade1_with_root_fitting.stl` and `blade2_with_root_fitting.stl` (print 1x each; blade 2's flexure and root fitting are MIRRORED in z, its airfoil is not, so it is a different part), `drawing.png`, `parts.json`; generator `../build_10in_v5.py` (parts from `flexblade/fittings.py`). All STLs watertight, one shell each (parts.json).
Coordinates mm, hub frame, z up = build direction = rotor axis. Strips crossed at **105.00 deg**, skew 60.00 deg.

## System requirements (single checklist; all items verified together in study/results/system_check.md)
1. **Flexure stack**: 4 strips x 8 x 0.10 mm unbonded 1095 laminae (9.00 x 49.1 mm), crossed 105.00 deg, skew 60.00 deg, 40 mm aramid strand per blade, VHB 9469 constrained-layer damper (eta about 0.03). 0.80 stack in the 0.85 slot, strip width within 15 % of nominal (pluck table). Pitch preset +3.98 deg/blade.
2. **Mount (required: a rigid mount has air-resonance zeta -0.004)**: twin TPU 85A washers ri 4 / ro 5.8 / t 12 mm, centres **84 mm apart**, both preloaded (M4), mass on the mount <= 126 g. k_lat about 24 kN/m (hub mode about 0.65 x lowest 1P), **k_t 145 N m/rad**. Sizing rule k_t >= 2.5 x Omega_max^2 (I_t - J_p) with Omega_max = 952 rad/s: allowed measured I_t <= 8.9e-5 kg m^2 (estimate 4.1e-5, J_p 2.55e-5). If the measured I_t is higher, lengthen the spacing (k_t goes as d^2) before flying.
3. **Arm**: f_x and f_y (mount locked, motor fitted, no blades) **>= 120 Hz** on both axes; forbidden band 50-95 Hz (ground resonance). The model run at 120/150/200 Hz gave identical min zeta. Tap test as in "Arm stiffness" below. 22 x 20 CF tube at 200 mm.
5. **ESC governor**: PI on **rev-averaged eRPM** (window exactly one mechanical rev, sliding; raw or low-passed eRPM is forbidden), **bandwidth <= 20 Hz** (25 Hz at 0.5 ms latency), **latency <= 2 ms** (loop dead time + L/R + T/2 at the lowest flight rpm); or NO ESC governor (FC closes rpm slowly through the throttle). With this the 1P cyclic gain/phase equal free spin (G 1.000, 0.0 deg) and min zeta is unchanged (0.0089 at T1.4). Without rev_avg, G is 0.3-11x and unstable at 30 Hz/2 ms.
4. **FC cyclic schedule**: cyclic gain and phase are a CONTINUOUS function of measured eRPM (bidirectional DShot), stored at >= 5 calibration nodes spanning the flight rpm range
   (T0.5 to T1.5), interpolated linearly in ln(rpm); not +-5 % bins. Calibrate EVERY build (the strip-k build tolerance moves the 1P phase and gain; it is fixed per airframe).
   Procedure: bolt the airframe down, accelerometer/IMU on the motor can, step the throttle through >= 5 rpm plateaus (hold 3 s each, plus 2 in-between),
   at each log the fixed-frame hub tilt response to a known commanded cyclic pulse and store gain and phase per plateau; repeat with the governor on and off
   (identical with rev_avg; a difference means the governor is not 1P-blind). Re-check after a crash or strip replacement (creep/fatigue drift, about 1 deg per 2 % k).
   Mechanism mode must never cross 1P on any build corner (margin >= 0.05); the model says the continuous schedule leaves about 7 deg / 15 % residual (10 in) and 9 deg / 20 % (7 in).
6. **Bench measurements** (each has its own section below): J_p of the motor+hub (bifilar; the J_hub column of the FC pulse schedule), motor transverse inertia I_t (bifilar about a diameter; compare with k_t rule),
   TPU washer G and eta (dynamic, about 200 Hz: a shear sample or the mount hub-mode tap test vs the predicted frequency), arm tap tests (x and y, mount locked, spec above), pluck test of the flexure modes,
   and the cyclic gain/phase sweep of item 4. Integrated model check: `study/results/system_check.md`.

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

## Damped isotropic motor mount, twin washers, tilt-stiff (air-resonance and tilt-whirl margin)
**Rule: never tilt-soft-mount a 2-blade rotor.** A single soft washer gives rotating stiffness about the blade axis k_t + Omega^2 (J_p - I_t) that goes negative when the motor+hub transverse inertia I_t exceeds the polar J_p: synchronous 1/rev tilt divergence (the 7in v6 single washer was unstable, zeta -0.09...-0.14). Decouple: translation soft (hub mode <= 0.7 x lowest 1P), tilt stiff (k_t >= 2x Omega_max^2 (I_t - J_p), designed 2.5x at 2x the estimated I_t).

Parts (make_mount.py -> ../mount_twin.py, watertight STLs in this folder, parameters in mount_parts.json): `mount_tube_clamp.stl` (split clamp for a 10 mm tube, bore 10.1, M3 pinch bolt, M4 tap), 2x `mount_tpu_washer.stl` (TPU 85A, ri 4 / ro 5.8 / t 12 mm, 40 % gyroid), `mount_carrier.stl` (plate + skirt holding both washers, motor holes M3 on 16x16 and 19x19), `mount_post.stl` (d 7.8 rod with two d 11.6 x 2 flanges on the washer inner faces, M4 at the clamp end), `mount_cap.stl` (retainer ring, M2 x3). Washer centres 84 mm apart. Preload the M4 so BOTH washers carry thrust (then the pair is a stiff tilt joint: tilt k = 2 Ec pi (ro^2-ri^2)/t (d/2)^2 + local term).

Sized (study/mounts_tilt.py): k_lat 24.0 kN/m, k_t 145 N m/rad (rule x2.5 at 2x I_t needs 128), axial k 82 kN/m (sag 0.12 mm per 10 N). Mass on the mount assumed 126 g. Lateral hub mode 68-69 Hz (0.63-0.64 x lowest 1P 108 Hz at T0.7), zeta_af 0.072-0.074. Floquet min zeta over the T x J x arm scan: 0.0089 (estimated I_t), 0.0089 (2x I_t), 0.0089 (3x I_t); all positive. Details: study/results/mounts_tilt.md. TPU properties (G, eta) must be verified on a printed sample (tap test: the motor bell should ring at the hub-mode frequency, x = y within 5 %).

## Bench: transverse inertia I_t of motor + carrier + hub (bifilar about a diameter)
Measure next to J_p. Hang the motor + hub + carrier (no blades) bifilar from two threads (length l ~ 0.5 m, separation 2a ~ 60 mm) with the rotor axis HORIZONTAL (a bar through the carrier holds it) so the oscillation is a twist about a diameter through the centre of mass (use the cup axis point where the washers sit as the tilt centre; add m h^2 for the offset h of the centre of mass from the tilt centre by the parallel-axis theorem): I = m g a^2 T^2 / (4 pi^2 l). Compare with the estimate I_t_est = 4.11e-05 kg m^2 (0.5 J_p + m (15 mm)^2, J_p = 2.55e-05). Then look up the minimum k_t (rule k_t >= margin x Omega_max^2 (I_t - J_p), Omega_max = 952 rad/s = 1P at T1.4, J_p = 2.55e-05); the built mount has k_t = 145 N m/rad:

| measured motor+hub I_t (kg m^2) | min k_t at 2x margin (N m/rad) | at 2.5x |
|---|---|---|
| 2.05e-05 (0.5x est.) | 0 | 0 |
| 4.11e-05 (1x est.) | 28 | 35 |
| 8.22e-05 (2x est.) | 103 | 128 |
| 1.23e-04 (3x est.) | 177 | 222 |
| 1.64e-04 (4x est.) | 252 | 315 |

If the measured I_t needs more than the built k_t at 2x margin, lengthen the washer spacing (k_t ~ d^2) before flying. Measure J_p the same way about the axis (existing bifilar section).

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

## Arm stiffness (ground resonance, tap test)
The regressing lag mode (omega_lag - Omega) crosses the hub-on-mount / arm mode: **forbidden f_arm band 50-95 (45-100 with margin) Hz** (cantilever frequency of the arm with motor + mount, x and y; unstable or zeta < 0.005 there).  **Spec: f_arm >= 120 Hz on both axes** (set by study/results/mounts_tilt.md I32; arm length 180-220 mm is an assumption).
**Vertical (z) arm mode, added** (results/arm_vertical.md): mismatched 1P flapping puts 11 N at 1P (108-152 Hz over T0.7-1.4) on the arm vertically; forbidden f_z band, tip > 50 um at zeta 0.02: **80-203 Hz** (precone 0). With blade precone 1.3 deg the force is 1.55 N and the band shrinks to **106-165 Hz**.
**One arm requirement (10 in): f_x, f_y >= 120 Hz (ground res.) AND f_z >= 210 Hz at precone 0; with precone 1.3 deg f_z >= 170 Hz (or <= 100 Hz, not advised).** A round tube has f_z = f_x, so the round 22 x 20 tube must reach 210 Hz (170 Hz with precone); tap-test z as well as x, y.
(Caveat: 2P lateral 13.8 N with precone is unverified, see arm_vertical.md; if real, f_x,f_y must also avoid 196-349 Hz.)

**Mount-mass limit: <= 126 g on the mount** (the modelled motor-side mass; the 10in limit was not swept, the 7in one is 55 g); more mass moves the band up and the margin disappears.

**Tap test (mount locked, i.e. washers bolted solid / clamped, motor + prop adapter fitted, no blades).**  Phone or accelerometer taped to the motor can; flick the motor can with a fingernail, record >= 2 s at >= 1 kHz (phone: a physics-toolbox accelerometer app or audio of the tap on the arm), FFT, read the lowest peak.  Do it twice, tapping along x and y (x = along the arm axis normal to the rotor axis, y = the other lateral), peak must be >= 120 Hz on both.  Unlocked-washer taps show the hub mode instead (0.63-0.69/rev), which is a different, intended mode: do not confuse them.

Minimum tube sizes (cantilever CF tube, E 70 GPa, rho 1600, tip mass 126 g + 0.24 tube mass; f = sqrt(3EI/L^3/m)/2pi; "OK" means >= 120 Hz):

| tube OD x ID mm | L 180 mm: f_arm Hz (tube g) | L 200 mm: f_arm Hz (tube g) | L 220 mm: f_arm Hz (tube g) |
|---|---|---|---|
| 12 x 10 | 61 NO (10.0) | 52 NO (11.1) | 45 NO (12.2) |
| 14 x 12 | 78 NO (11.8) | 67 NO (13.1) | 58 NO (14.4) |
| 16 x 14 | 97 NO (13.6) | 83 NO (15.1) | 72 NO (16.6) |
| 18 x 16 | 117 NO (15.4) | 99 NO (17.1) | 86 NO (18.8) |
| 20 x 18 | 138 OK (17.2) | 117 NO (19.1) | 101 NO (21.0) |
| 22 x 20 | 160 OK (19.0) | 136 OK (21.1) | 118 NO (23.2) |
| 25 x 23 | 194 OK (21.7) | 166 OK (24.1) | 143 OK (26.5) |

Clamp/joint compliance lowers the real number, so take the tap-test as authoritative; a bonded-in socket shorter than the tube adds a series spring.

### Calibration procedure (per build: continuous cyclic gain/phase vs eRPM, >= 5 nodes)
1. **Logging setup**: airframe bolted down; accelerometer/IMU on the motor can (fixed frame, x/y moment proxy) or a load cell (Mx, My); log at >= 1 kHz: `t, erpm, azimuth_deg` (or `rev`; else integrated from eRPM / pole pairs), `cmd_amp`, `cmd_phase_deg` (the commanded cyclic pulse, torque pulse = amp cos(azimuth - phase)), `mx, my`. Use the same azimuth zero the FC will use in flight.
2. **Sweep**: command a pulse of constant amplitude with the command phase **rotating** at 1-3 Hz (needed to separate G from DC/imbalance), while the throttle ramps slowly from T0.5 to T1.5 (model-validated 0.7 to 1.4) in **>= 20 s** (10 s only passes with amp >= 2x the per-sample IMU noise). Pulse amplitude: **>= 2x the per-sample IMU noise** (>= 1x marginal at 20 s; <= 0.5x fails); keep it inside the pulse-schedule limit above. Repeat with the governor on and off.
3. **Fit**: `python flexblade/study/calib/fit_cyclic.py log.csv --nodes 5 --pole-pairs <pp> --out schedule` (use `--sens` for IMU units per N m). It lock-in demodulates per 1 s rpm window (1P..4P nuisance terms), robustly fits ln|G| and phase at the nodes (piecewise linear in ln eRPM) and writes `schedule.csv/.md` (erpm, gain, phase_deg, gain_ci, phase_ci; 95 % half-widths). Reject a build whose CI exceeds 5 % / 3 deg; re-log with more amplitude or a slower sweep.
4. **Load** the CSV table into the FC schedule (Lua/param table, linear in ln eRPM, cyclic gain and phase); re-do after a crash or strip replacement.
Validated on synthetic logs (10in v5 model, 1 % eRPM jitter, nP vibration, IMU noise): study/calib/synth.py.
