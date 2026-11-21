# BUILD SHEET: 7in_tt_strap_steelx_v6 (2-blade, crossed 1095 strips, aramid strand)

Files (this folder): `hub_fitting_assembly.stl` (print 1x), `blade1_with_root_fitting.stl` and `blade2_with_root_fitting.stl` (print 1x each; blade 2's flexure and root fitting are MIRRORED in z, its airfoil is not, so it is a different part), `drawing.png`, `parts.json`; generator `../build_7in_v6.py` (parts from `flexblade/fittings.py`). All STLs watertight, one shell each (parts.json).
Coordinates mm, hub frame, z up = build direction = rotor axis. Strips crossed at **88.00 deg**, skew 59.91 deg.

## System requirements (single checklist; all items verified together in study/results/system_check.md)
1. **Flexure stack**: the crossed 1095 strip flexure in the Cut list above (88.00 deg crossing, aramid strand, damper per the cut list), strip width/lamina count within 15 % of nominal (pluck table).
2. **Mount (required)**: twin TPU 85A washers ri 4 / ro 8 / t 22 mm, centres **76 mm apart**, both preloaded, mass on the mount <= 55 g. k_lat 35.6 kN/m (hub mode 0.67-0.69 x lowest 1P), **k_t 177 N m/rad**. Rule k_t >= 2.5 x Omega_max^2 (I_t - J_p), Omega_max = 1636 rad/s: allowed measured I_t <= 3.4e-5 kg m^2 (estimate 1.6e-5, J_p 7.6e-6). If I_t is higher, lengthen the spacing. A single washer is UNSTABLE (tilt divergence). Balance to < 1 g mm (the hub mode amplifies imbalance: 40-65 um per 2 g mm).
3. **Arm**: f_x and f_y (mount locked, motor fitted, no blades) **>= 180 Hz** without a governor, **>= 200 Hz with the ESC governor** (system check: at 180 Hz, T1.4, min zeta falls to 0.0025-0.0041 with the governor; at 200 Hz 0.007-0.009). Forbidden band 75-175 Hz. Tap test as in "Arm stiffness" below.
5. **ESC governor**: PI on **rev-averaged eRPM** (one full mechanical rev, sliding; raw or low-passed eRPM is forbidden), **bandwidth <= 27 Hz** (40 Hz at 0.5 ms latency), **latency <= 2 ms** (use 0.5 ms with bw 27 if possible: zeta is higher at T0.7/T1.0 with 2 ms but lower at T1.4), or NO ESC governor. 1P cyclic gain/phase then equal free spin.
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
| Spring-steel shim strip, 1095, 0.15 mm | **5.84 wide x 31.8 mm long** = 12.50 free + 2 x 9.7 clamp lands; cut 6 (4 + 2 spare) | 4 |
| Constrained-layer damper (the design assumes it, eta 0.0298): 3M VHB 9469 0.13 mm + 0.05 mm shim on each strip free length, ending 1 mm short of the jaw exits | 5.84 x 10.5 mm | 4 |
| Aramid strand 1.5 mm | **40 mm** per blade: 4 free + 2 x 8 channel + 2 x 8 knot/tail; one continuous strand hub side to blade side | 2 |
| Glue | Strips: structural epoxy, thin film. CA only to tack. Knot wells: thin CA then epoxy seal | |
| Balance | Pin-balance the blade pair; tip mass difference < 0.05 g | |

Strip prep: tin snips, strip LONG axis parallel to the rolling (grain) direction. Cut ~0.3 mm wide, file to 5.84 (the strip is narrower than v4: measure with calipers, tolerance +-0.05 mm; width is the most sensitive dimension of the flexure stiffness, k_hinge ~ w). Deburr all four edges (400 grit along the length, break ~0.1 mm). Degrease with IPA, scuff only the glued lands.

## Slots and radii (in the STLs)
- Free span 12.50 mm between hub and root jaw exits; clamp land 9.7 mm per strip end; slot 0.20 mm x 6.0 mm; jaw exits radiused R = 1.50 mm.
- Cross gap 1.9 mm between the two strip planes: hold this with a 1.9 mm shim while the epoxy cures.
- Clearance (rigid sweep, lag +-5 deg x pitch +-8 deg): rest gap 4.18 mm, minimum 4.10 mm.
- Printed fitting mass (PA6-CF, per blade, in the rotor model): hub fitting 1.11 g, root fitting 1.18 g (fitting_slim webs). Hub assembly STL total 6.22 g.

## Print settings
PA6-CF dried 6+ h at 80 C; z up as exported; 0.4 nozzle, 0.12 layers, 6 perimeters, 100 % infill, no supports. The slot is ~1.7 layers: check with a spare strip, scrape with a fresh blade, do not drill.

## Assembly
1. Dry test-fit deburred strips in both slots.
2. Per blade, push each strip from behind through the hub jaw, across the free span, into the blind root slot, strips crossing at 88.00 deg (drawing.png).
3. Thin epoxy on the lands, hold the 1.9 mm shim between the strips until cured, remove it.
4. Fit damper layers 1 mm short of the lands.
5. Thread the strand, stopper knots, strand just taut at rest, CA then epoxy seal.
6. Repeat for blade 2; balance the blade pair, then the assembled rotor.

## Pitch preset (static, per blade)
`pre_lag auto` on the pitch axis resolves (resolve_pre_lag, T1 hover) to **+1.590 deg per blade**, (v5: -0.115, v4: +2.86 deg). Build both blades at this value +- 0.3 deg: phone inclinometer on the chord at 0.5R vs the hub plane, blades matched to +-0.3 deg. If one blade is off by more than that, re-seat the root jaw before gluing, do not bend the strips.

## Predicted modes for the pluck / tap test (Hz; model, nominal strips, damper, real fitting mass)
At rest (0 rpm, aero off; hub treated as the model hub, tap with the arm clamped and the hub as stiff as you can make it):

| Mode | T0.7 | T1 | T1.4 |
|---|---|---|---|
| pitch sym | 35, 921, 1058 | 35, 921, 1058 | 35, 921, 1058 |
| pitch antisym | 988 | 988 | 988 |
| pitch antisym+hub | 51 | 51 | 51 |
| flap sym | 247 | 247 | 247 |
| flap antisym | 250 | 250 | 250 |

At hover rpm (aero on, trim rpm per thrust level T = 0.7 / 1.0 / 1.4 x weight, centrifugal stiffening included; only observable by an accelerometer or a mic during a spin-up, or by the throttle sweep below):

| Mode | T0.7 (10957 rpm) | T1 (13044 rpm) | T1.4 (15622 rpm) |
|---|---|---|---|
| pitch sym | 104, 883 | 122, 692, 1106 | 149, 317, 629, 1131 |
| pitch antisym | 943 | 882 | 709 |
| pitch antisym+hub | 151 | 176 | 214 |
| flap sym | 338 | 354 | 377 |
| flap antisym | 328, 1087 | 338 | - |

Notes for the bench test: the 'pitch' modes are the flexure hinge / torsion modes of the crossed strips (the skewed hinge couples pitch and lag, so there is no separate low lag mode: the hinge mode IS the 'pitch sym' line). 'pitch antisym+hub' is the differential mode that also moves the hub in-plane, so it depends on the arm mount stiffness. Flap modes at rest (~250 Hz) are the strip bending on the free span plus blade root. Tolerance: model values assume 0.15 mm / 5.84 mm strips; more than 15 % off in the pitch-sym line (~35 Hz) means a loose clamp or wrong width. Frequency scales as sqrt(k): width +5 % = +2.4 % frequency. Pluck the blade tip chordwise (excites pitch) and flapwise (excites flap), record 5 s with a phone spectrum app, read the lowest peak.

## Airframe isotropy requirement
Make the motor-arm mount in-plane stiffness ISOTROPIC (x ~ y within 5 %) or soft-damped. A 2-blade rotor on an anisotropic support has a hub-dominated 2/rev parametric air-resonance window that strip damping cannot cure. Tap-test the arm in x and y (tones within ~5 %); fly with a rubber/TPU damped mount if not. The v5 Floquet check (verify_7in_v5.md; v6 is the same airframe model) is with f_y = 1.15 f_x as a worst realistic anisotropy; the airframe damping assumed is 0.02 minimum.

## What to measure to calibrate the nP aero forcing (blade-root accelerometer, slow throttle sweep)
The forced-response criterion (forced.py) assumes torque harmonics 3 % at 2P, 1 % at 3P, aero nP loads ~5-10 % of mean lift moment at 2P falling ~1/n^2. These are assumptions; measure:
1. Mount a 3-axis MEMS accelerometer (>= 4 kHz sample rate, e.g. ADXL372 or a logging IMU in the blade-root fitting, or on the arm near the hub as a fall-back) on one blade root; log with a telemetry-free logger on the rotor or a slip-ring.
2. Slow throttle sweep: ramp 40 % to 100 % of hover throttle over >= 60 s with the vehicle restrained (thrust stand or tethered), so every nP line crosses every mode slowly (rpm range ~ 11000-15700). Log rpm from ESC telemetry (or the 1P line itself) and motor current.
3. Spectrogram vs rpm: read the 1P, 2P, 3P, 4P amplitudes, and the amplification peaks where nP meets a mode of the table above (predicted: e.g. 2P at 435 Hz at T1 rpm against the pitch antisym+hub line at 176 Hz).
4. Per crossing record amplitude A_pk (g) and the half-power width of the resonance peak, from which zeta = df/(2 f) (compare with the model zeta). Ratio of the nP acceleration far from any crossing to the 1P acceleration gives the forcing fraction to compare with the 3 % / 1 % assumptions.
5. Repeat with the motor current harmonic spectrum logged (shunt + scope/ADC) to separate the electrical (torque) nP source from aero nP: if the root acceleration at nP follows the current nP amplitude it is torque; if it follows thrust / tilt it is aero. Then test the aero part by tilting the rotor disc (a few degrees) or running near a wall and see the 2P scale.
6. Pitch jitter: if possible a hall/optical pitch pickup on one blade; the criterion limit is pitch jitter in degrees, accelerometer double-integration is not reliable.

## Approximations and limits
- Strap anchor `pin_wrap` replaced by a stopper-knot well; knot strength not modelled. Clearance is a rigid-body sweep; strip S-bending not included.
- ccx cross-check of the strip pair (study/verify_v5.py ccx, v5 strips; v6 not re-run): all terms within 6 % of the beam model.

## Compliant clamp lip and strip free length
`strip_clamp = compliant`: the jaw exits carry a soft lip (TPU, Kt 1.05) so the strip curvature spreads over the lip instead of a hard edge at the jaw exit (this is what keeps the strip fatigue stress inside the limit at 1/rev). In the STLs the jaw exits are radiused R = 1.50 mm; bond a 0.5 mm TPU 95A shim lip (strip width, 2 mm long) on each jaw exit edge with CA, flush with the slot floor. Do not add lip stiffness beyond the exit: the free span is 12.50 mm between the exits.

## Laminated strips (studied, NOT the build)
Studied in study/v6L.py (equal total EI, pulse-limited robust over throttle {0.7,1,1.4} x J_hub {0.92,2.5,5.49}e-6): 2 x 0.13 mm laminae (w 4.49) cr_pl 0.0644, 3 x 0.10 (w 6.57) 0.0613, 4 x 0.10 (w 4.93) 0.0678 but resonance-infeasible; v6 solid 0.0650. Laminates cut the fatigue-bound low-J cases (e.g. T1 J0.92e-6 cr_pl 0.146 -> 0.198 with 2 x 0.13) but the worst case is the motor 1P torque at high J and throttle, which they do not move (<10 %), so the single 0.15 mm strip is built. If you try 2 x 0.13 mm anyway: cut w = 4.49 mm, stack unbonded with a thin film of light oil (e.g. 3-in-1 / sewing-machine oil) between the laminae, epoxy only on the clamp lands, keep oil off the lands, damper layer over the stack.

## Damped isotropic motor mount, twin washers, tilt-stiff (air-resonance and tilt-whirl margin)
**Rule: never tilt-soft-mount a 2-blade rotor.** A single soft washer gives rotating stiffness about the blade axis k_t + Omega^2 (J_p - I_t) that goes negative when the motor+hub transverse inertia I_t exceeds the polar J_p: synchronous 1/rev tilt divergence (the 7in v6 single washer was unstable, zeta -0.09...-0.14). Decouple: translation soft (hub mode <= 0.7 x lowest 1P), tilt stiff (k_t >= 2x Omega_max^2 (I_t - J_p), designed 2.5x at 2x the estimated I_t).

Parts (make_mount.py -> ../mount_twin.py, watertight STLs in this folder, parameters in mount_parts.json): `mount_tube_clamp.stl` (split clamp for a 10 mm tube, bore 10.1, M3 pinch bolt, M4 tap), 2x `mount_tpu_washer.stl` (TPU 85A, ri 4 / ro 8 / t 22 mm, 40 % gyroid), `mount_carrier.stl` (plate + skirt holding both washers, motor holes M3 on 16x16 and 19x19), `mount_post.stl` (d 7.8 rod with two d 16 x 2 flanges on the washer inner faces, M4 at the clamp end), `mount_cap.stl` (retainer ring, M2 x3). Washer centres 76 mm apart. Preload the M4 so BOTH washers carry thrust (then the pair is a stiff tilt joint: tilt k = 2 Ec pi (ro^2-ri^2)/t (d/2)^2 + local term).

Sized (study/mounts_tilt.py): k_lat 35.6 kN/m, k_t 177 N m/rad (rule x2.5 at 2x I_t needs 166), axial k 121 kN/m (sag 0.08 mm per 10 N). Mass on the mount assumed 55 g. Lateral hub mode 122-126 Hz (0.67-0.69 x lowest 1P 183 Hz at T0.7), zeta_af 0.067-0.072. Floquet min zeta over the T x J x arm scan: 0.0043 (estimated I_t), 0.0116 (2x I_t), 0.0123 (3x I_t); all positive. Details: study/results/mounts_tilt.md. TPU properties (G, eta) must be verified on a printed sample (tap test: the motor bell should ring at the hub-mode frequency, x = y within 5 %).

## Bench: transverse inertia I_t of motor + carrier + hub (bifilar about a diameter)
Measure next to J_p. Hang the motor + hub + carrier (no blades) bifilar from two threads (length l ~ 0.5 m, separation 2a ~ 60 mm) with the rotor axis HORIZONTAL (a bar through the carrier holds it) so the oscillation is a twist about a diameter through the centre of mass (use the cup axis point where the washers sit as the tilt centre; add m h^2 for the offset h of the centre of mass from the tilt centre by the parallel-axis theorem): I = m g a^2 T^2 / (4 pi^2 l). Compare with the estimate I_t_est = 1.62e-05 kg m^2 (0.5 J_p + m (15 mm)^2, J_p = 7.64e-06). Then look up the minimum k_t (rule k_t >= margin x Omega_max^2 (I_t - J_p), Omega_max = 1636 rad/s = 1P at T1.4, J_p = 7.64e-06); the built mount has k_t = 177 N m/rad:

| measured motor+hub I_t (kg m^2) | min k_t at 2x margin (N m/rad) | at 2.5x |
|---|---|---|
| 8.10e-06 (0.5x est.) | 2 | 3 |
| 1.62e-05 (1x est.) | 46 | 57 |
| 3.24e-05 (2x est.) | 133 | 166 |
| 4.86e-05 (3x est.) | 219 | 274 |
| 6.48e-05 (4x est.) | 306 | 382 |

If the measured I_t needs more than the built k_t at 2x margin, lengthen the washer spacing (k_t ~ d^2) before flying. Measure J_p the same way about the axis (existing bifilar section).

## Minimum-inertia hub build (J_hub itemised)
Direct-to-bell clamp: the printed hub assembly is screwed straight onto the motor bell (no nut, no adapter). Itemised polar inertia about the rotor axis (the bell stays in J_rotor = 6.99e-6):
| item | mass | radius | J (kg m^2) |
|---|---|---|---|
| printed hub assembly (hub_fitting_assembly.stl, PA6-CF, Izz of the STL) | 6.22 g | - | 0.49e-6 |
| 4x M3x6 socket screws into the bell | 4 x 0.5 g | 8 mm | 0.13e-6 |
| strip clamp lands, 4 strips 0.15 x 5.84 x 9.7 mm steel | 4 x 0.066 g | ~9 mm | 0.02e-6 |
| thread-locker / knots / tape allowance | ~0.1 g | 8 mm | 0.01e-6 |
| total J_hub | | | ~0.65e-6 |

The v6 design is robust over J_hub 0.92 to 5.49e-6, and the itemised direct-clamp value (~0.65e-6 with the lighter v6 hub) sits slightly below the 0.92e-6 design corner; the design is not verified below 0.92e-6, so if the bifilar result is lower, use the 0.92e-6 schedule row and add a few tenths of a gram at 8 mm radius to reach it.

## Bifilar measurement of J_hub
Hang the motor bell + printed hub (no blades, screws fitted) from two parallel threads of length l, separation 2a (a ~ 30 mm, l ~ 0.5 m); twist about the vertical axis and time 20+ oscillations (period T): J = m g a^2 T^2 / (4 pi^2 l), m = bell + hub mass. Repeat with the bell alone (expected J_rotor 6.99e-6): J_hub = J(bell+hub) - J(bell). Use a photogate or slow-mo video. Accept J_hub within the itemised value +- 0.3e-6; the design is feasible anywhere in 0.92 to 5.49e-6.

## Pluck-test mode table
See "Predicted modes" above (rest and hover tables, generated from the v6 model). Pass criteria: lowest pitch-sym line (~35 Hz) within 15 % of the table, flap sym/antisym (~250 Hz) within 10 %; frequency scales as sqrt(k), strip width +5 % = +2.4 %.

## FC pulse schedule (cyclic pulse amplitude vs throttle and measured J_hub)
The stored design op pulse is the original; the FC applies this schedule (Lua in-flight calibrated 1P phase; boot reference on the ESC). Pulse scale is relative to 0.95 x the motor's capability pulse at that rpm; values above 1 x mod_frac_max are fine because the motor_torque_1p check on the delivered torque (speed-locked gain below DC) is the real cap. Interpolate linearly in throttle (thrust) and in J_hub between rows. Fatigue binds at low J and low throttle, the motor 1P torque at high J and high throttle. Robust cr_pl (min over all) = 0.0650; tolerance at J 2.5e-6 9/9 (nominal + 8 random +-5 %).

| throttle (thrust x weight) | J_hub (kg m^2) | pulse, x 0.95 capability | pulse, x mod_frac_max | cr at that pulse | cr_pl | binding check | 1P margin |
|---|---|---|---|---|---|---|---|
| 0.7 | 9.20e-07 | 0.508 | 0.483 | 0.3648 | 0.1854 | fatigue_stress | 0.184 |
| 0.7 | 2.50e-06 | 0.781 | 0.742 | 0.2373 | 0.1854 | fatigue_stress | 0.293 |
| 0.7 | 5.49e-06 | 1.162 | 1.104 | 0.1415 | 0.1645 | motor_torque_1p | 0.557 |
| 1 | 9.20e-07 | 0.735 | 0.698 | 0.1984 | 0.1458 | fatigue_stress | 0.233 |
| 1 | 2.50e-06 | 1.066 | 1.013 | 0.1367 | 0.1458 | fatigue_stress | 0.357 |
| 1 | 5.49e-06 | 1.194 | 1.135 | 0.0856 | 0.1023 | motor_torque_1p | 0.579 |
| 1.4 | 9.20e-07 | 1.005 | 0.955 | 0.1246 | 0.1252 | fatigue_stress | 0.215 |
| 1.4 | 2.50e-06 | 1.232 | 1.170 | 0.0850 | 0.1047 | motor_torque_1p | 0.304 |
| 1.4 | 5.49e-06 | 1.232 | 1.170 | 0.0528 | 0.0650 | motor_torque_1p | 0.306 |

Procedure: measure J_hub (bifilar), pick the matching J column, program the pulse for each throttle; if J falls between columns use the smaller pulse of the two bracketing rows.

## Arm stiffness (ground resonance, tap test)
The regressing lag mode (omega_lag - Omega) crosses the hub-on-mount / arm mode: **forbidden f_arm band 75-175 Hz** (cantilever frequency of the arm with motor + mount, x and y; unstable or zeta < 0.005 there).  **Spec: f_arm >= 180 Hz on both axes** (set by study/results/mounts_tilt.md I31/I32).

**Mount-mass limit: <= 55 g on the mount** (everything moving with the motor side of the washers); more mass moves the band up and the margin disappears.

**Tap test (mount locked, i.e. washers bolted solid / clamped, motor + prop adapter fitted, no blades).**  Phone or accelerometer taped to the motor can; flick the motor can with a fingernail, record >= 2 s at >= 1 kHz (phone: a physics-toolbox accelerometer app or audio of the tap on the arm), FFT, read the lowest peak.  Do it twice, tapping along x and y (x = along the arm axis normal to the rotor axis, y = the other lateral), peak must be >= 180 Hz on both.  Unlocked-washer taps show the hub mode instead (0.63-0.69/rev), which is a different, intended mode: do not confuse them.

Minimum tube sizes (cantilever CF tube, E 70 GPa, rho 1600, tip mass 85 g + 0.24 tube mass; f = sqrt(3EI/L^3/m)/2pi; "OK" means >= 180 Hz):

| tube OD x ID mm | L 120 mm: f_arm Hz (tube g) | L 150 mm: f_arm Hz (tube g) |
|---|---|---|
| 12 x 10 | 137 NO (6.6) | 98 NO (8.3) |
| 14 x 12 | 175 NO (7.8) | 125 NO (9.8) |
| 16 x 14 | 217 OK (9.0) | 155 NO (11.3) |
| 18 x 16 | 261 OK (10.3) | 186 OK (12.8) |
| 20 x 18 | 308 OK (11.5) | 219 OK (14.3) |
| 22 x 20 | 357 OK (12.7) | 254 OK (15.8) |
| 25 x 23 | 435 OK (14.5) | 310 OK (18.1) |

Clamp/joint compliance lowers the real number, so take the tap-test as authoritative; a bonded-in socket shorter than the tube adds a series spring.

### Calibration procedure (per build: continuous cyclic gain/phase vs eRPM, >= 5 nodes)
1. **Logging setup**: airframe bolted down; accelerometer/IMU on the motor can (fixed frame, x/y moment proxy) or a load cell (Mx, My); log at >= 1 kHz: `t, erpm, azimuth_deg` (or `rev`; else integrated from eRPM / pole pairs), `cmd_amp`, `cmd_phase_deg` (the commanded cyclic pulse, torque pulse = amp cos(azimuth - phase)), `mx, my`. Use the same azimuth zero the FC will use in flight.
2. **Sweep**: command a pulse of constant amplitude with the command phase **rotating** at 1-3 Hz (needed to separate G from DC/imbalance), while the throttle ramps slowly from T0.5 to T1.5 (model-validated 0.7 to 1.4) in **>= 20 s** (10 s only passes with amp >= 2x the per-sample IMU noise). Pulse amplitude: **>= 2x the per-sample IMU noise** (>= 1x marginal at 20 s; <= 0.5x fails); keep it inside the pulse-schedule limit above. Repeat with the governor on and off.
3. **Fit**: `python flexblade/study/calib/fit_cyclic.py log.csv --nodes 5 --pole-pairs <pp> --out schedule` (use `--sens` for IMU units per N m). It lock-in demodulates per 1 s rpm window (1P..4P nuisance terms), robustly fits ln|G| and phase at the nodes (piecewise linear in ln eRPM) and writes `schedule.csv/.md` (erpm, gain, phase_deg, gain_ci, phase_ci; 95 % half-widths). Reject a build whose CI exceeds 5 % / 3 deg; re-log with more amplitude or a slower sweep.
4. **Load** the CSV table into the FC schedule (Lua/param table, linear in ln eRPM, cyclic gain and phase); re-do after a crash or strip replacement.
Validated on synthetic logs (10in v5 model, 1 % eRPM jitter, nP vibration, IMU noise): study/calib/synth.py.
