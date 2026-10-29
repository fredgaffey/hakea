# Blind validation "bench2": pre-registered predictions

Written 2026-10-26, before any log numbered above 00000034 existed or was
read. These predictions are not to be edited; later revisions go in a separate
file. Hashes of this file and predictions.csv are in HASHES.txt.

## Test (as specified)
PROP ON, same hardware, settings and ESC EEPROM as log 34. DShot300
bidirectional at 1200 frames/s, throttle in ArduPilot µs on output 1, each
segment preceded by 5 s at 1000 µs (script ~/vehicles/hakea/scripts/bench2.lua,
sha256 70c8be13…). Segments: A steps 1150/1450/1750 µs, 4 s each; B 1650 µs
for 4 s then 1000 µs for 5 s; C sines 1600 ± 250 µs at 5/15/35/50 Hz, 8 s each;
D sine 1400 ± 150 µs at 25 Hz, 8 s.

Firmware: Fred plans to flash AM32 head + PR #438 (bdbc75e,
firmware/AM32_FOXEER_F421_head-pr438_bdbc75e.elf) before this test. Its
throttle, ramp and commutation code is the same as v2.21 (the calibration
logs). The PR only adds EDT2 status/stress telemetry hooks. If the EDT2
telemetry rate changes the DShot timing, that is not modelled.

## Model used
The commutation-averaged model (scripts/sim/avgmodel.py), fitted by
scripts/sim/calibrate.py on logs 33 (prop off) and 34 (prop on, 249-446 s),
all four bench.lua tests. It was checked against log 32 runs 1 and 2 (same
1500 ± 400 sweep, held out): 440-480 rpm rms.

Parameters (params_v5all.json, sha256 652c852b…; snapshot in model_snapshot/):

| | |
|---|---|
| Kv (nominal, fixed) | 1300 |
| effective voltage gain g(rpm) at 0/8k/14k/20k/30k rpm | 1.088 / 1.029 / 1.082 / 1.050 / 1.001 |
| R (loop, prior 0.16 ± 0.04 Ω) | 0.158 Ω |
| L (fixed) | 30 µH line-line |
| J motor / prop | 4.17e-6 / 2.53e-6 kg m² |
| kq (prop torque = kq ω²) | 1.84e-8 N m s² |
| b, T0 | 5.8e-6 N m s, 0.0008 N m |
| battery R | 0.037 Ω |
| measurement-chain delay | 4.98 ms |

Battery assumption: 24.8 V open circuit at the start, drifting −0.004 V/s
(as in log 34). The predictions were also simulated at ±0.6 V, and that
spread is included in the ranges.

Analysis: the simulated run is sampled like the Lua log (430 Hz, delayed by
the chain delay) and analysed with scripts/sim/compare_blind.py's
`analyze()`. compare_blind.py will analyse the real log with the same
function. Definitions are in its docstring. Phase lags INCLUDE the
measurement-chain delay; the assumed delay is 4.98 ms, fitted from logs
33/34. That is less than the ~6.5 ms estimated earlier from log 32 by a
simpler first-order fit.

## Predictions (90 % ranges)

| quantity | predicted | range |
|---|---|---|
| A 1150 µs steady rpm | 5,360 | 4,980 – 5,760 |
| A 1450 µs steady rpm | 14,470 | 13,740 – 15,220 |
| A 1750 µs steady rpm | 21,770 | 20,690 – 22,860 |
| B rpm at cut (1650 µs) | 19,440 | 18,480 – 20,430 |
| B rpm 0.25 / 0.5 / 1 / 2 s after cut | 0 | 0 – 1,500 |
| B time to half speed after cut | 0.021 s | 0.008 – 0.045 s |
| C 5 Hz: mean / fundamental amp / p-p / lag | 17,940 / 5,480 / 11,280 / 38° | 16,860–19,050 / 4,740–6,010 / 9,510–12,650 / 30–46° |
| C 15 Hz | 17,960 / 3,330 / 6,950 / 91° | 16,880–19,060 / 2,440–3,740 / 4,880–8,070 / 79–103° |
| C 35 Hz | 17,940 / 1,620 / 3,530 / 158° | 16,860–19,040 / 950–1,820 / 1,900–4,240 / 138–178° |
| C 50 Hz | 17,960 / 1,120 / 2,480 / 201° | 16,880–19,060 / 600–1,320 / 1,170–3,160 / 176–226° |
| D 25 Hz (1400 ± 150) | 12,750 / 1,460 / 3,200 / 130° | 11,930–13,580 / 1,140–1,720 / 2,260–4,020 / 115–145° |
| desyncs (any segment) | 0 | 0 |

The amplitude is the fundamental (half the fundamental peak-to-peak). p-p is
the per-cycle max − min of the logged samples, including ~300 rpm of
telemetry noise. Full list with gains (rpm per µs) in predictions.csv.

Coast-down: AM32 with comp_pwm = 1 ramps the duty to 0 while still
commutating, which brakes the motor dynamically. The model predicts the
motor stops in ~50 ms and the telemetry then reads 0.

ESC temperature rise: not predicted. The model has no validated thermal path.

## Known weaknesses (stated in advance)
1. **Large, fast throttle swings.** On the bench big sweep (1500 ± 400 µs), the rpm
   swing at 10-45 Hz is 15-40 % smaller than this model gives. A fit on the
   other tests alone over-predicts it by up to ~60 %. The cause is not
   identified: candidates are heating of the windings, unlogged fast battery
   sag, and torque-constant drop at high current. C's ± 250 µs lies between the
   calibrated ± 200 and ± 400 cases, so the amplitude and gain ranges at
   15-50 Hz are skewed low (down to 0.55-0.75 × the model value).
2. **Absolute current/torque.** These are not identified; they rest on the R
   prior. They do not affect any rpm quantity predicted here.
3. **Battery voltage at test time.** It is unknown. Each volt shifts the steady
   rpm by ~4 %.
4. **Low throttle (1150 µs).** The model's effective gain is least certain here
   (+2-3 % residual in calibration).
