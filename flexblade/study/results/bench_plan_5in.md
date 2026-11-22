# Bench plan: 5in lead rotor (5in_v8s_2307_m245_301), cheapest and safest first
Rotor: `study/designs/5in_v8s_2307_m245_301.json`, prints in `study/build/5in_301/` (BUILD.md). Tools: kitchen scale (1 g), weights, phone slow-mo (240 fps), printed fixture, your 2307 bench with tach/ESC rpm. Labels: [MODEL] = simulation, [MEAS] = your logs. Every band is +-20 % unless stated. No bench data exists yet, so all gates are model-only.

## 0. Setup (10 min, no power)
- Weigh the rotor: expect ~5 g print plus the foils [MODEL]. Do the post-cure fan check (BUILD.md step 8): each stack fans a little and flexes easily. A stack that feels like a plate is glued, so scrap it before anything else.
- Pin-balance it. Check the M5 shaft length (about 18 mm of thread above the bell), washers both sides, locking nut.

## 1. Static pitch stiffness (15 min, safe)
- Clamp the hub on the M5 shaft or in the fixture, blade horizontal. Print a 50 mm arm, perpendicular to the blade at the tip (chord direction), horizontal. Tape a laser pointer to the blade and read the dot on a wall 1 m away (dot shift per metre = angle), or lay a phone protractor on the blade. Weigh the arm and subtract its tare.
- [MODEL] k_pitch = 0.0999 N m/rad = 1.74 mN m/deg (blade 1, unloaded 6x6 from `rotor.flexure_stiffness`, other freedoms free; blade 2 is the mirror and is about 10 % softer in the raw matrix). Expected angle = 0.282 deg per gram at the 50 mm arm.
| weight at arm | expected | pass band |
|---|---|---|
| 5 g | 1.41 deg (dot 25 mm at 1 m) | 1.13-1.69 deg |
| 10 g | 2.82 deg (dot 49 mm) | 2.26-3.39 deg |
| 15 g | 4.23 deg (dot 74 mm) | 3.38-5.08 deg |
- Check both blades and both signs. The response must be linear and return to zero within 0.1 deg.
- Stiff reading (under half the angle): laminae are bonded. A bonded stack is up to 81x stiffer in bending (9^2) [MODEL], so you would see a tiny angle. Scrap it and rebuild with glue on the lands only. Soft reading (over 1.5x): a land has slipped or a stack is short.

## 2. Strap pull test (spare strand, 10 min, safe behind a guard)
- Strand: aramid 1.3 mm in the design file (BUILD.md says 1.5 mm), so test the strand you will use. Termination: the design file says pin wrap (cap 2308 N [MODEL]) but BUILD.md says a knot (cap about 1590 N). Test the termination you will build. strap_audit_5in.md recommends 2 turns round a PETG post of 6 mm or more, no knot.
- Mean strap tension is 75 / 100 / 122 N at T0.7 / 1.0 / 1.25 [MODEL]. Test load = 3 x 122 = 366 N (about 37 kg). Hang the weights (bucket of water, or a lifting scale), hold 60 s, unload.
- Pass: no slip (mark the strand at the post), no fray at the post or knot. Strength is not expected to be the limit (static util 0.05-0.08 [MODEL]); fatigue and slack are the risk.
- Fit the real strap with 20-30 N pretension and check it is still taut after test 4.

## 3. Spin-up thrust vs rpm (30 min, rotor spinning)
- SAFETY: clear lexan guard on all sides. Stand out of the rotor plane, at least 2 m away, with glasses on. Mount the motor so thrust reduces the scale reading (rotor above). Slew-limit the throttle, with no punches.
- Do NOT use 1500 / 1800 throttle as the target. Those gave 15650 / 22480 rpm [MEAS] on the quad prop. This rotor loads the motor differently. Step the throttle up and stop at about 14.7k rpm. 22.5k rpm is 1.7x hover rpm and about 2.9x thrust (about 7 N), far beyond the T1.25 study point.
- Read rpm from ESC telemetry and the tach. Thrust N = scale drop (g) x 9.81e-3. [MODEL] with T proportional to rpm^2 and hover 13.1k rpm = 2.40 N (0.245 kg design AUW):
| point | rpm | thrust |
|---|---|---|
| smoke test | 8k | about 91 g |
| T0.7 | 10.96k | 171 g |
| T1.0 hover | 13.1k | 245 g |
| T1.25 | 14.65k | 306 g |
- Pass: thrust within +-20 % at each rpm, rpm steady within 1 %, no table vibration. Slow-mo of the tips: both blades in one plane within 2 mm, no flutter. Note ESC temp (reached 59 C in your logs [MEAS]). Current data are unusable (uncalibrated sensors), so use a clamp meter if you want power (hover 57 W [MODEL, v0]).
- Thrust under 0.7x or over 1.3x: stop, recheck test 1 and the blade pitch zero.

## 4. Fatigue soak (45 min including cool-downs)
- Hold hover rpm 13.1k behind the same guard, 3 x 10 min with cool-downs. 10 min is about 131k 1P cycles, which covers the 1e5-cycle strap test recommended in strap_audit_5in.md. It is 1.3 % of the 1e7 design life [MODEL], so it is a screen, not proof. Without cyclic code only the steady loads are present.
- Afterwards repeat the section 0 fan check and test 1. Pass: k_pitch change under 10 %, stacks still fanned, no cracks at slot mouths, no fretting dust at the clamps, sprue stubs, strap and nut still tight and the strap taut. Fail: any foil crack, a slack strap, or a stiffer stack (laminae stuck).

## 5. 1P cyclic response (needs ESC cyclic code first)
- Not testable yet: `cyclic_apply()` is a stub, and `angle_from_zc_cnt` has a scaling bug (esc_phase_budget.md section 3). Tip-path tilt per unit command was not extracted from the model for this note [MODEL gap]. What the study has: control power cr_pl 0.169 vs 0.039-0.053 needed, and a phase-error budget of about 1.7 deg rms against a 15 deg gate.
- After the ESC code is fixed: slow-mo of a laser line across the disc (or the tip trace on a screen) at hover, sweeping the pulse phase. Pass is a tilt that follows the commanded phase. Run it only after tests 1-4 pass.
