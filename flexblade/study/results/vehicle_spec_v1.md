# Hakea vehicle spec v1: single 5" flexblade rotor + 6 anti-torque vanes (supersedes vehicle_spec_v0.md)
Labels: [MODEL] = simulation estimate, [MEASURED] = from Fred's logs, [EST] = mass or size guess. Nearly everything about the rotor is [MODEL]; no bench data exists yet. `verify_5in_v8s.md` was named as a source but is not in the repo, so its numbers are not used.

## 1. Configuration
- One 5" two-blade flexblade rotor (flexure hinge made of stacked 301 steel foils) on Fred's 2307 1300KV motor, 6S.
- Torque reaction is cancelled by 6 vanes under the rotor. 4 are fixed and 2 sit on one micro servo, for yaw steering.
- FC: Matek H743-Slim V3 (Fred's choice).
- Design point AUW 0.245 kg [EST]. The rotor tune moves with weight, so retune the rotor if the built AUW is outside 0.20-0.29 kg.
- Why single, not coax or tandem: v0 compared them, and single is the only one that is near-feasible with an existing design [MODEL]. Fred's tandem question is open (section 5).

## 2. Parts
| item | spec | status |
|---|---|---|
| Rotor | `study/build/5in_301/`: one PA-CF print (hub + both blades, ~5 g), breakaway sprues, 9 foil laminae x 4 stacks | recommended; motor interface still a placeholder |
| Strip | 301 full-hard stainless, 0.05 mm x 4 mm slit, 1 roll (cut 45 laminae, 26.1 mm each) | recommended; one-off order, OK pending Fred |
| Vanes | 6 vanes, chord 33 mm, 5 % camber, 117 mm OD ring, ~20-22 g printed + rods and links. PETG, or PA-CF if the motor base runs hot | recommended |
| Servo | one micro servo (~4.3 g) in the hub tube, drives the 2 opposite vanes through a double horn | recommended |
| Motor | 2307 1300KV, ~35 g [MEASURED in notes] | decided by Fred |
| FC | Matek H743-Slim V3 | decided by Fred |
| ESC | Foxeer Reaper 65A 4-in-1, AM32 2.21, 6S [MEASURED, log-index] | decided by Fred (he owns it) |
| Battery | 6S 650 mAh, ~112 g [EST, 0.17 g/mAh]; Fred should weigh his own | recommended |
| Frame/body | PA-CF motor mount + stack, CF legs, wiring, receiver, encoder; v0 budget ~15 + 8 + 5 + 8 g [EST]. Vane/servo add 4-6 g over v0's 20 g | recommended; open until weighed |

## 3. Key numbers (all [MODEL] at T1.0, 0.245 kg)
- Control authority: cr_pl **0.169** (T1.25 optimiser run, 9 x 4 mm 301 strip, feasible, no hard violations). The authority needed is **0.039-0.053** (5", gusts + 1 mm CG error).
  Nominal screen at T0.7/1.0/1.25 was 0.439/0.284/0.180. Worst perturbed corner has fatigue stress 0.86 of the limit, so margin is not unlimited.
- Hover: ~13.1k rpm, ~57 W [MODEL, v0]. Rotor tip speed ~87 m/s.
- Flight time: ~12.2 min on 650 mAh (v0, 22.2 V x Ah x 0.8 / P). With camera and 850 mAh (~290 g): ~74 W, ~12 min [EST].
- Vanes (all [MODEL]): fixed set returns ~100 % of rotor torque across T0.7-1.25, so trim needs only +-0.3 deg of servo. Yaw authority is +25 % / -28 % of Q. Yaw rate time constant ~0.5 s.
- Measured motor facts: R loop 0.154 ohm; at 1500/1800 throttle the 5" setup read 15,650/22,480 rpm; ESC temp reached 59 C. There is no thrust data and no usable current data.

## 4. Must-haves
1. **ArduPilot harmonic notch driven by ESC rpm.** The 1P torque reaction (176 mN m [MODEL]) lands at 190-240 Hz and puts ~1 rad/s on gyro z. Without a notch the FC will fight it. AM32 telemetry must give rpm to the H743.
2. **Glue the foils at the lands only.** Stack the 9 laminae dry, with no glue between them, and mist the free span with PTFE spray. Use thick slow epoxy (30 min+) on the 8 mm lands. No thin CA near the strips.
   Check after cure: the foils must fan slightly and flex easily. If it feels like a solid plate, scrap that stack.
   Cure 24 h before snipping the breakaway sprues, then pin-balance the rotor.
3. **Motor interface is a placeholder.** The STL has a plain 5 mm bore only. Still TODO: the real 2307 shaft/bolt pattern, washer faces and boss height.
4. Lean the vanes 15 deg tangentially (spoke style). Radial vanes put the ring torsion mode at 213 Hz, inside the 1P band [MODEL].
5. Weigh all parts and hang-test the CG before flying. Slew-limit throttle changes, because a fast punch makes a spin-up reaction about equal to the yaw authority.

## 5. Open questions for Fred
1. Tandem vs single? v0 recommends single. Tandem needs 2 motors/ESC channels, a stiff boom and a custom mixer, and has no better margin [MODEL].
2. How many 2307s do you have? Single needs 1, and tandem needs 2.
3. Is the 2307 shaft M5? The placeholder bore is 5 mm, and the real pattern is unknown.
4. Is the slit 301 order OK (0.05 x 4 mm, one roll)? 5 mm width tested worse, so 4 mm is recommended.
5. 2307 max current? The model's 40 A is a guess. BAT.Curr and ESC.Curr read 64-90 A even at idle, so those sensors are uncalibrated. A clamp-meter reading is needed. Pulse amplitude at 5" is 21-24 A [MODEL].
