# CRM2204 1400KV coax: flexure-cyclic design (PRELIMINARY; DE still running)

## Specs (one half of the unit unless noted; E = estimate)
| item | value | source |
|---|---|---|
| unit | Dancing Wings Hobby CRM2204 1400KV, 2 motors, needs 2 ESC | Amazon B07D7S2MRG / .ca / .fr listings |
| mass | ~85 g, both halves plus housing | listing |
| cells / ESC | 3S (11.1 V), 2 x 20 A ESC recommended | listing |
| props / thrust | 2 x 8045 supplied (with prop savers); 734 g total @ 11.1 V with 2 x 8040 | listing |
| kV / stator | 1400 / 22x4 mm, 12N14P (E) | name; generic 2204 |
| R | 0.24 ohm bare (generic 2204 1400KV, Furitek), 0.30 effective with ESC (E) | powerhobby Furitek page |
| max current | 15 A per motor (generic 2204 1400KV is 13-15 A) (E) | GoolRC / Furitek listings |
| shaft | 3.0 mm (sister CR23M: 3.0 mm x 13 mm) (E); concentric inner/outer shaft not confirmed | CR23M listing |
| J_rotor | 2.1e-6 kg m2 (scaled from the Hakea 2307 fit, +-40 %) (E) | motor.py `_J(22,4)` |

Preset: `motor.MOTOR_LIBRARY["crm2204_1400kv"]`. Run with `run_study.py --motor crm2204_1400kv --vehicle-mass 0.18`.
Trim (7", 0.18 kg thrust/rotor, i.e. AUW ~360 g, T/W ~2): 5.9k rpm (1P ~98 Hz), Q 0.027 N m, 1P cap 0.078 N m (current-limited, 2.8x hover torque).
At 6": 7.6-8.5k rpm. At 8": 4.2-4.7k rpm.

## Architecture (first principles)
- Shaft torsion (E: 3 mm steel, 25-40 mm free, J 2e-6 motor + 5e-6..3e-5 rotor): k_t 16-25 N m/rad, f_tors 450-650 Hz, i.e. 4.6-6.6P at 98 Hz. This is clear of 1P, but the 5P/6P pulse harmonics may excite it, so keep the pulse sinusoidal.
- Inner-shaft bending: a 25 mm overhang gives about 500 Hz, OK. A 40 mm cantilever gives about 250 Hz with 15 g on top, which is 2.5P and marginal. Measure the overhang.
- Housing coupling: a 0.05 N m 1P reaction on a vehicle with Izz ~3e-4 kg m2 gives 0.025 deg of housing yaw. The other rotor sees this as base rotation, about 1 % of its own 1-2 deg lag swing, so it is negligible. The two drivetrains share only bearings, so there is no torsional path between them.
- Hub: the supplied O-ring prop savers WILL SLIP under the 1P pulse. Use a set-screw or collet hub on the 3 mm shaft. A cross-spring hub (r_hub ~8 mm at 7") fits within the ~27 mm bell.
- Yaw by differential mean torque also shifts mean lag, and so collective, on every flexure rotor (yaw-to-thrust coupling). B has this on one rotor only.
- Counter-rotation: the second rotor is a full mirror (airfoil + flexure). Its pulse phase reverses, so each rotor needs its own phase calibration. A 1P cyclic moment is set in the fixed frame per rotor, so both can add in the same direction.
- Literature: Paulos & Yim ICRA 2015 coax (227 g) put cyclic on one rotor, with yaw from the second (from memory; check the paper). HKU's coax work (Liang 2025) needed flap hinges for vibration.

## Option comparison (A = both rotors flexure, B = upper flexure + rigid lower)
| | A: both flexure | A': both, half-authority each | B: upper flexure, rigid lower |
|---|---|---|---|
| cyclic moment | M_up + ~0.85 M_lo ~ 1.85 M (lower: wake inflow, E) | 1.0 M with pulse ~0.54 per rotor | 1.0 M |
| pulse per rotor / fatigue | full pulse on both; util as single design | sigma_a(pulse) halves, Goodman u ~ sigma_m/Su + 0.5 sigma_a/Se | full pulse on one rotor |
| 1P corner margin | as single design | same unless re-designed thicker (corner margin is NOT set by pulse) | as single design |
| yaw left (I_max - I0 - dI)*Kt | ~0 at full pulse (dI ~11 A, I0 ~4 A, I_max 15 A) | ~0.03 N m per motor | rigid motor 0.075 N m, all of it |
| yaw-to-collective coupling | both rotors | both, half as much | one rotor |
| mass / complexity | 2 flexure hubs, 2 encoders, 2 phase cals | same | 1 flexure hub + stock hub |
| failure modes | 2 x fatigue sites; mismatched 1P flap on both; blade strike | same, lower util | one fatigue site; rigid rotor keeps yaw on a flexure failure |

## Tip clearance (E: 7" R = 89 mm; stock rotor spacing ~15 mm, MEASURE)
Lower rotor cones UP toward the upper one: 3 deg coning gives 4.7 mm, 1P flap up to 2.6 deg (10" b2 value, used as a proxy) gives 4.0 mm, 2P ~0.5 mm, and precone 1.3 deg adds 2.0 mm.
Upper rotor 1P flap down: 4.0 mm.
Sum ~15 mm. With a 1.5x gust margin plus ~5 mm of hub and blade thickness, about 27 mm tip-plane spacing is needed, so a ~10-15 mm spacer (E).
For B the rigid lower rotor has no 1P flap, so about 20 mm is needed.

## Design (TODO, pending DE)
DE: 7in tt_strap_steelx, `--criterion mech_band --mech-margin 0.07 --goodman --objective pulse_limited --robust 0.7,1.0,1.4 --ground-resonance --maxiter 12 --popmult 6 --hosts local:4`, seeded from 7" v6.
Fill in from it: dimensions, cr_pl, binding check, and the 7" 1P flap per blade (this replaces the 10" proxy above).

## Recommendation
Build B first: upper flexure and stock lower rotor. It isolates the mechanism, keeps full yaw authority and needs less spacer.
Move to A' (both rotors, each designed for half authority with thicker strips and more corner margin) only if B's moment is too small. A at full pulse on both rotors leaves no yaw authority.

## Bench measurements on arrival
J_rotor per half (spin-down with a known prop), kV and R (line-line plus ESC), L, inner-shaft torsion and bending frequency (tap test with the hub fitted), stock rotor spacing and overhang, shaft diameter, and whether the shafts are concentric.
