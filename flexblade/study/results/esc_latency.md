# ESC latency / phase-error budget at 1P (analysis only; no AM32/ArduPilot edits)
Basis: 10" 1P 110-150 Hz (6.6-9k rpm, T0.7-1.4); 7" assumed 180-250 Hz (10.8-15k rpm, not read from a design file). 1 deg of rotor angle = 21 us at 130 Hz, 14 us at 200 Hz.
Motors (motor.py presets): 4010_475kv (10") L=108 uH R=0.12; 2806.5_1100kv (7") L=44 uH R=0.10 (estimate_L, +-30 %). Both assumed 14P (42 commutations/rev, 8.6 deg mech/step, per esc_mini.md). pydantic missing, so figures were hand-computed.
| component | 10" fixed | 10" variable | 7" fixed | 7" variable |
|---|---|---|---|---|
| ZC angle: comparator+filter ~6 us, ZC jitter ~1-2 us | 0.3 | 0.1 (rpm range, temp) | 0.4 | 0.15 |
| Interpolation between ZCs (extrapolating last interval, ~1 % 1P speed ripple) | 0 | <0.1 | 0 | <0.1 |
| ZC shift with cyclic current (demag, advance) and per-step magnet/ZC asymmetry (repeats each rev) | ~0.5 | 0.3 | ~0.5 | 0.3 |
| AM32 20 kHz duty update (50 us tick: mean 25 us hold delay, uniform jitter) | 1.2 | 0.7 rms / +-1.2 pk | 1.8 | 1.0 rms / +-1.8 pk |
| Ramp limiter 16 units/tick (320k units/s): linear while 2*pi*f*amp < 320k | 0 | 0 if amp < ~200 units (10"), < ~130 (7"), else lag grows | 0 | same |
| DShot frame ~53 us (DShot300), CPU decode | 2.5 | 0.3 (frame-edge sync; cut to 0 if FC sends amplitude only) | 3.5 | 0.4 |
| L/R current lag atan(2 pi f L/R): 10" L/R 0.9 ms, 7" 0.45 ms (6-step, open loop) | 36 | +-4 over 110-150 Hz; -5 for +60 K (R +23 %) | 29 | +-4 over 180-250 Hz; -5 for +60 K |
| torque_bandwidth_hz 2 kHz guess (library default) | 3.7 | <0.5 | 6-7 | <0.5 |
FC path: 400 Hz FC loop gives 3 samples per rev at 130 Hz; one loop of jitter (2.5 ms) = 117 deg, DShot at 1 kHz one frame = 47 deg. The FC cannot be in the 1P phase path at all.
Conclusions
- Everything except L/R lag is <= ~2 deg variable at both sizes (dominated by 20 kHz update jitter), so electronics meet <=3 deg ONLY IF the cyclic term is synthesised on the ESC and L/R lag is handled.
- Open-loop 6-step voltage drive fails: L/R lag varies ~+-4 deg with rpm and ~5 deg with winding temperature, i.e. 6-8 deg total at 10" and 7". Needs (a) closed iq/current loop (FOC or current-limit-based, BW >= 1 kHz, which flattens lag to ~atan(f/BW) = 7 deg fixed, ~0.5 deg variable), or (b) a lag model in the ESC: phi = atan(w L / R(T)), R(T) from ESC temperature or measured I/V, then lead the command by phi.
- Calibration (Lua) then trims the residual; the 15 deg CAL limit is fine, but the 6 deg residual of the 10" lead should not be spent on electronics.
ESC firmware requirements
1. FC sends only slow amplitude and phase-trim (FRAME_CYCLIC, dshot.c:329, currently a TODO) at 50-400 Hz; ESC generates the sine from its own rotor angle. Latency in those updates then only delays amplitude.
2. Apply the term as duty/iq reference at the 20 kHz tick or faster (PWM period at 24-48 kHz is better: jitter <= 21 us -> 0.5-1 deg), computed from angle extrapolated to the update instant: theta = theta_zc + omega*(t - t_zc), timer timestamps at 0.5 us. Fix angle_from_zc_cnt() (multiplies by 2, not 65536) and use a hardware timer capture for t_zc, not the main-loop tick.
3. Bypass or raise max_ramp_high_rpm for the cyclic term (apply after the ramp limiter); keep amp*f below 50 % of 320k units/s so it never saturates.
4. Phase trim is per throttle/rpm; add a thermal term (R(T)) or a current loop. Telemetry should echo the applied phase and rpm for CAL.
