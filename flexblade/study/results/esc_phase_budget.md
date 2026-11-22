# ESC cyclic phase budget (AM32 sensorless, 2307 1300KV, 5" lead). Gate 15 deg - 1.8 tolerance use = 13.2 deg left.
Tags: [F] fact (repo/code read), [E] estimate. Rotor-frame mech deg at 1P; 1 us of timing = 0.0786 deg at hover.
## 1. Motor and rates
- Pole count [E]: 2307 is the usual 12N14P (14 magnets, 7 pole pairs); NOT stated in motor.py or 2307_measured.md, Fred to confirm the eeprom motor_poles value. AM32 uses ZC per mech rev = 3*poles = 42 (main.c:980) [F], so 6 ZC per elec rev, one every 8.57 deg mech.
- Hover 13.1k rpm (218 Hz): f_elec 1528 Hz, ZC every 109 us. T0.7 (rpm ~ sqrt(T) = 10.96k, 183 Hz): 1279 Hz, 130 us. T1.25 (14.65k, 244 Hz): 1709 Hz, 97.5 us [E, thrust ~ rpm^2].
- [F] measured (2307_measured.md): rpm 15650@1500, 22480@1800; duty->rpm tau 14.5-23.5 ms, ~6.5 ms delay. No ZC, comparator or phase-lag data exists. Everything below is estimate.
## 2. Error sources
(a) ZC jitter [E]: filter_level 2-12 comparator reads (main.c:963, map(avg_interval...) ~3 at this rate), ~1-2 us, plus 1 us timer tick: 0.1-0.2 deg. PWM-edge sampling worst case +-half period (24 kHz assumed, default not confirmed; ZC every 2.6 periods): +-20 us = +-1.6 deg. Take 0.2 deg typical, 1.6 deg bound.
   Per-index ZC asymmetry (winding/magnet, +-1-2 deg elec) = +-0.15-0.3 deg, repeatable so the Lua calibration can absorb it.
(b) Lag [E]: detection delay is a fixed time tau (filter+comparator, 2-5 us) so phase = 360 f tau: 0.2-0.4 deg at hover, changes +-16 % over T0.7-1.25 = +-0.07 deg. Fixed part absorbed by calibration. Advance (auto 13-23 *0.9375 deg elec, main.c:2283) shifts commutation, not the ZC timestamp, so it does not enter the angle. Slow part: filter_level steps with rpm (+-1-3 us = 0.1-0.2 deg) and current/duty dependence of BEMF ringing/demag (+-0.3 deg guess). Speed ripple at 1P of d% moves ZC prediction by ~4.3 deg*d: 0.04 deg per 1 %.
(c) Index mapping: ZC index gives elec angle only, 6 per elec rev, so the mech phase is P=7-fold ambiguous (51.4 deg apart) until the boot reference picks one. A missed/extra ZC is a permanent 8.57 deg step (57 % of the gate, two exceed it) because zc_index is deliberately never reset; it shows as a sudden constant phase offset in 8.57 deg steps in the Lua calibration. A wrong boot cycle shows as k*51.4 deg.
(d) DShot latency [E]: negligible for rotor phase. Pulses are made in the ESC from the local angle; only amp/phase command changes lag (frame 26 us at DShot600; FC loop 1-4 ms). Cost is control-loop lag in the body frame (360 f_ctrl t: 10 Hz x 2 ms = 7 deg of loop phase), not rotor phase error.
## 3. Code findings for Fred (cyclic.c, not fixed)
- angle_from_zc_cnt(): sizeof(uint16_t) is 2, so angle = zc_cnt*2/steps gives 0..1, not 0..65535. Probably meant zc_cnt*65536u/steps (32-bit math). Also steps-only: no interpolation within the 8.57 deg step.
- Zero-order hold on that angle = sawtooth error +-4.3 deg (the largest term); interpolate with INTERVAL_TIMER_COUNT/commutation_interval.
- unpack_frame phase is 6 bits = 5.6 deg/LSB: +-2.8 deg max, 1.6 deg rms quantisation (second largest). cyclic_apply() is a stub returning duty.
## 4. Total [E]
| term | random (deg) | slow/throttle (deg) |
| ZC jitter + per-index | 0.2 typ, 1.8 bound | - |
| lag + filter + current | - | 0.1-0.5 |
| phase LSB | 1.6 rms, 2.8 max | - |
| interpolation (if done) | 0.1 | - |
RSS typical about 1.7 deg, linear worst case about 5.5 deg (without ZOH): fits in 13.2 deg. With ZOH add 4.3: about 10 deg, still inside but 3 deg left. Sync loss (8.57) alone passes; 2 steps + anything fails. Bottom line: dominant risks are lost sync and unfixed angle code, not hardware jitter.
