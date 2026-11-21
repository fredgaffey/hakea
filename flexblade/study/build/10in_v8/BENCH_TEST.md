# BENCH TEST: 10in v8 (model values at J_hub 3.76e-6, nominal strips; T = thrust/weight)
Pass band everywhere: **+-20 % of the model**. Caveat: v8 mech margin is only 0.036 and the stopped tangent is an unconverged-path result (stopped_state.md).

## (a) Pluck/tap of the mech mode, stopped hub
Arm clamped stiff, blades on, no rpm. Flick the blade tip chordwise, phone spectrum, 5 s, lowest peak; both blades (blade 2 is softer).
Model stopped tangent (presets on): k_cross b1 114 / b2 67 N m/rad (unpreset 468 / 555); k_2nd b1 67 / b2 39.
Expected: scale the v5 rest pitch-sym line (19 Hz, unpreset) by sqrt(k): **b1 ~9 Hz, b2 ~7 Hz** (rough, no eigen solve). Pass 7-11 Hz b1, 5.5-8.5 Hz b2.
If the slots were built without presets (BUILD.md), expect the unpreset ~19 Hz (+-15 %) instead: a check of strip width and clamping. Flap lines ~190 Hz (+-10 %).

## (b) Spin-up, encoder OPTIONAL
Throttle ramp T0.5 to T1.4 over >= 20 s on the thrust stand; log eRPM (bidir DShot) and an accelerometer on the motor can. Model trim rpm: T0.7 6480, T1.0 7724, T1.4 9145 (1P 108 / 129 / 152 Hz). Pass: within 20 %.
A magnetic encoder (AS5047/AS5600) is not required: Fred's logs 31/32 show sensorless AM32 with no desync. Use it only as a phase cross-check of the ZC-derived azimuth (lit_review.md: published flying systems all use an encoder); a disagreement > 3 deg means fix the ESC angle, not the model.

## (c) 1P cyclic moment, 2-axis load cell
Step 1, before any cyclic: extend the throttle-sine sweep from 50 Hz toward ~300 Hz (log 32 style, 1500 +-400, then wider) and check for desync (eRPM dropouts, current spikes), especially over the 1P band 108-152 Hz. Stop if it desyncs.
Step 2: ZC-locked 1P pulse (ESC synthesises the sine from its own angle). Hold 3 s plateaus at T0.7/1.0/1.4, lock-in Mx, My at 1P, rotate the command phase, fit gain and phase (study/calib/fit_cyclic.py).
Model (torque pulse at 0.95 x capability to 1P hub moment; phase vs the command):
| T | rpm | torque amp N m | moment N m | moment/torque | phase deg |
|---|---|---|---|---|---|
| 0.7 | 6480 | 0.603 | 0.669 | 1.11 | 78.6 |
| 1.0 | 7724 | 0.509 | 0.393 | 0.772 | 91.7 |
| 1.4 | 9145 | 0.391 | 0.244 | 0.624 | 98.6 |
Moment is linear in pulse: M = (moment/torque) x amp. Start at 25 and 50 % of the amps shown (fatigue / motor-torque caps bind); never exceed the v5 schedule row. Model 2P moment 0.82 / 0.50 / 0.32 N m.

## (d) Pass/fail
- Gain within +-20 % at each T; phase within +-20 deg and smooth in ln(rpm) (model moves 20 deg over T0.7-1.4; residual after CAL 6.2 deg / 9.8 %).
- Mech mode (a) within 20 %; no desync to 300 Hz; no ground/air resonance in spin-up (arm per BUILD.md).
- On fail recheck r_h (+-1.5 %), strip width, presets, and slider friction before touching the model.

## ESC firmware requirements (esc_latency.md)
1. FC sends only slow amplitude/phase trim; the ESC generates the sine from its own rotor angle.
2. Apply at the 20 kHz tick or faster, angle extrapolated theta = theta_zc + omega (t - t_zc), 0.5 us timer capture; fix angle_from_zc_cnt() (x2, not 65536).
3. Bypass or raise the ramp limiter for the cyclic term; amp x f < 50 % of 320k units/s.
4. L/R lag (~36 deg, +-4 over rpm, 5 deg per 60 K): closed iq loop (BW >= 1 kHz) or phi = atan(wL/R(T)) lead. Telemetry echoes applied phase and rpm.
5. Any governor on rev-averaged eRPM, BW <= 20 Hz.
