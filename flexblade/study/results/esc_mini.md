# ESC for 1P torque pulsing on 2–3" two-blade rotors (11xx, 12N14P)
Question: can an ESC deliver a 1P torque modulation (10–30 % of mean, phase ≤10°) at 1P = 500–1000 Hz?
Short answer: **AM32 6-step can do it on G431/F421/L431-class MCUs, but only with a firmware duty path that bypasses
the ramp limiter and that is referenced to the ZC timer (requirements below; Fred writes it). Stock AM32 throttle
input cannot.** A FOC ESC with a 40 kHz+ current loop and a magnetic encoder is the clean option at 2".

## Requirements (`flexblade/study/esc/requirements.py`; L/R ≈ 8 µH / 0.25 Ω = 32 µs)
| size | rpm | 1P Hz | e-freq | eRPM | comm/rev (°mech) | PWM/rev @24/48 kHz | AM32 20 kHz ticks/rev (°) | loop fc for ≤10° uncomp. | fc if lag calibrated out | L/R lag @1P |
|---|---|---|---|---|---|---|---|---|---|---|
| 2" | 60k | 1000 | 7.0 kHz | 420k | 42 (8.6) | 24 / 48 | 20 (18) | 5.7 kHz | 2.0 kHz | 11° |
| 2.5" | 45k | 750 | 5.25 kHz | 315k | 42 (8.6) | 32 / 64 | 27 (14) | 4.3 kHz | 1.5 kHz | 9° |
| 3" | 30k | 500 | 3.5 kHz | 210k | 42 (8.6) | 48 / 96 | 40 (9) | 2.8 kHz | 1.0 kHz | 6° |
The governor route (drive_rev_avg.md, BW ≈ 0.13/(latency+L/R+T/2)) is a speed loop; it cannot reach 1P. The
modulation must be feed-forward duty (6-step) or an iq reference (FOC), phase-locked per rev.

## AM32 code facts (am32-hakea, read only)
- ZC: comparator ISR `interruptRoutine()` (main.c:950–985) with `filter_level` re-reads; `filter_level` = 2 once
  `commutation_interval < 50` (main.c:2278); interval timer 0.5 µs ticks (main.c:1963). Commutation after
  `waitTime = commutation_interval/2 − advance` on COM_TIMER (main.c:935). No hard max-eRPM clamp.
- Duty: applied only in `tenKhzRoutine()` at `LOOP_FREQUENCY_HZ` 20 kHz (targets.h; main.c:1531 `adjusted_duty_cycle =
  duty_cycle*tim1_arr/2000`, `SET_DUTY_CYCLE_ALL`, main.c:1570), i.e. same duty on all phases, not per step.
  Input → `duty_cycle_setpoint` is slew-limited by `max_ramp_high_rpm = RAMP_SPEED_HIGH_RPM` = 16 /tick
  (main.c:312, targets.h:3589) = 320 000 duty-units/s. ±20 % of 1000 at 1 kHz needs 2π·1000·200 = 1.26 M/s.
- PWM: TIM1 ARR from `pwm_frequency` (main.c:643–649); variable_pwm=2 sets ARR = 100·CPU_MHz/9 at short intervals
  (~94–96 kHz, main.c:2002–2010). 24–48 kHz typical.
- Cyclic primitive: `zc_index_conter` counts ZCs mod `motor_poles*3` = 42 (main.c:979, 1655) → mechanical angle
  with 8.6° granularity, interpolable with the 0.5 µs interval timer. `cyclic.c`: `unpack_frame()` (6-bit phase =
  5.6°, 5-bit amp), `poly_sin()`; **`cyclic_apply()` returns duty unchanged and FRAME_CYCLIC is a TODO
  (dshot.c:329)**, so nothing modulates yet. Note `angle_from_zc_cnt()` multiplies by `sizeof(uint16_t)` (=2), not 65536.
- Worst-case phase quantisation if modulation is applied in the 20 kHz tick: one tick = 18° at 1 kHz (±9°), plus
  CCR preload to next PWM edge (≤1 PWM period: 7.5° at 48 kHz, 1 kHz 1P) → ~±13° raw at 2", ±7° at 3".
  Computing the target angle as "time since last ZC / commutation_interval" and applying it at the ZC/commutation
  ISR (42/rev) instead bounds it to ±4.3° + PWM preload.

## SITL results (unmodified fork, emulated G431 160 MHz; `flexblade/study/esc/am32_sitl_probe.py`, data in `esc/data/`)
Model: kv 11 000, 14 poles, R 0.25 Ω, L 8 µH, J 1.5–3e-7, k_load 3.8e-10 N·m·s², 16 V. Build needed
`SITL_COV_FLAGS=-Wno-error` (cyclic.c unused statics break `-Werror`).
- Sweep: clean commutation to **59.1k rpm / 414k eRPM** (com. interval 48 half-µs = 24 µs, filter 2, 0 new desyncs,
  duty 0.95). Firmware e_rpm matches model within 0.5 %.
  A kv 14 000 / 25 V / light-load sweep ran clean to **125.6k rpm / 879k eRPM** (interval 22 half-µs = 11 µs, i.e.
  0.5 PWM periods per step at 48 kHz), 0 desyncs: SITL finds no ZC/commutation limit in the 2–3" range. Caveat: SITL
  models ISR latency as fixed (`isr_read_ns` 100, `loop_time_ns` 2000), not MCU cycles, so CPU overrun is not tested.
- Startup, not speed, is the weak point: a step to throttle 1050 stalls in polling mode in ~50 % of boots (J ≤ 3e-7).
- 1P throttle modulation at 40k rpm, ±20 % DShot sine (only path in stock code):

| f_mod | setpoint amp | delivered duty amp | duty lag vs setpoint | ibus amp / lag vs duty |
|---|---|---|---|---|
| 300 Hz | 19.7 % | 18.9 % | 13° | 53 % / 2° |
| 500 Hz | 19.6 % | 12.3 % | 52° | 35 % / 3° |
| 750 Hz | 19.4 % | 8.1 % | 72° | 24 % / 5° |
| 1000 Hz | 19.2 % | 5.9 % | 82° | 17 % / 7° |
Ramp limiter → 320 k/s triangle (±51 units predicted at 1 kHz, ±60 measured). Duty→current lag only 2–7°: **the
motor/bridge is not the limit, the AM32 input path is**; DShot also carries no rotor angle.

## Renode
Renode is installed (~/.net/renode via ESCSim) and Fred's round 3 (notes/sim/renode/v3) already ran his PROFILE_MODE
F421 image: duty modulation applied in `tenKhzRoutine` delivered full ±100/±200 at 250–1000 Hz, 0 desyncs, 0 missed
ticks; torque lag vs duty 22° → 59° (250 → 1000 Hz) on the 18 µH hakea motor. Renode (1 instr/cycle, no wait states,
block-granular counters) **cannot give cycle-accurate ISR timing**; not re-run. CPU load at 420k eRPM must be measured on hardware (DWT->CYCCNT, or GPIO toggle).

## AM32 verdict per MCU (assuming a ramp-bypassing, ZC-timed cyclic path)
| MCU | clock | ISR budget per step at 420k eRPM (24 µs) | 2" | 2.5" | 3" | limiter |
|---|---|---|---|---|---|---|
| STM32F051 / G071 / GD32E230 | 48 / 64 / 72 MHz M0/M0+/M23 | 1150–1730 cyc | marginal | go | go | no HW divide for per-step `dt*65536/ci`, + 20 kHz tick + DShot decode in ~1.2k cycles |
| AT32F421 / STM32L431 | 120 / 80 MHz M4 | 2880 / 1920 cyc | go | go | go | F421 tick path Renode-proven |
| STM32G431 | 160 MHz M4 | 3840 cyc | go | go | go | SITL target; also FOC-capable |
Stock (no firmware change): **no-go on all**, limited by `RAMP_SPEED_HIGH_RPM` (main.c:312) and no angle reference.

## Alternatives
- VESC-class FOC (F4/G4, 20–30 kHz, current BW ~1–3 kHz): 500 Hz OK, 1 kHz marginal (20–40° lag, calibratable);
  sensorless observers cap ~150–300k eRPM at 7 kHz e-freq; physically too big for 2".
- Small STM32G4 FOC (e.g. B-G431B-ESC1, ESP32-S3 + DRV) at 40–60 kHz: loop BW ~4–6 kHz → meets 2" with lag compensation.
- Magnetic encoder (AS5047P/MA730, ~2 µs latency = 0.7° at 60k rpm, DAEC): true 1/rev index that ZC lacks (42 edges, no index).

## Recommendation
- **3" (500 Hz):** AM32 on F421/L431/G431, ZC-timed cyclic duty past the ramp limiter; 40 ticks/rev, ≤6° L/R lag.
- **2.5" (750 Hz):** same, prefer G431/F421; apply duty at commutation ISR, 48 kHz+ PWM.
- **2" (1000 Hz):** G431 FOC with encoder (40–60 kHz PWM, ≥5 kHz current loop) is the safe choice; AM32 6-step on
  G431/F421 is feasible (SITL: duty→current lag 7°) only with commutation-ISR application and PWM ≥ 48 kHz.
- Firmware requirements (Fred): offset after the ramp limiter; 1/rev index from `zc_index_conter` aligned at boot; angle
  interpolated by `INTERVAL_TIMER_COUNT/commutation_interval`; CCR update in the commutation ISR; fix `angle_from_zc_cnt` scale.