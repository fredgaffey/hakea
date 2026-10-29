# Pre-registered predictions: in-ESC throttle sine, 1-1000 Hz (Hakea PROFILE_MODE)

Written 2026-10-26, before any PROFILE_MODE hardware test. These files are not to be edited. Later revisions go
in a separate file. Hashes are in HASHES.txt.

## What is predicted
The current 5" bench rig (T-Motor 2307 1300 kV, 5" prop on, Foxeer Reaper F421 ESC, Fred's EEPROM
`esc1_config.bin`), base throttle 1500 µs (DShot 1048, duty 1020/2000 ≈ 14,800 rpm), battery 24.0 V.

A sine is added to the duty **inside the ESC's control loop**.

**Main case: Fred's PROFILE_MODE v1.**
- Source: `~/vehicles/hakea/am32-profile`, branch hakea/profile-mode = bdbc75e + uncommitted
  `Src/hakea_profile.c`, sha256 41bc0fcd….
- Hardware image: `firmware/AM32_FOXEER_F421_PROFILE_profile-v1_bdbc75e-dirty.elf`, sha256 6a64484e…bc5f.
- The sine is added **after** the ramp limiter ("ramp bypassed"), and the duty is clamped to
  [180, min(duty_cycle_maximum, 2000)].

**Comparison case: "sine before stock ramp 16".**
- This is a study-only build (`scripts/sim/firmware-profile-study.patch`, SIM_PROFILE): the same sine is added
  before AM32's stock ramp limiter (16 duty/tick).
- It shows what the stock limiter would do to an in-loop profile. It is not a mode anyone plans to run.

**Amplitude** is in % of full throttle: ±A % = ±20·A duty units. ±5 % = ±100, ±10 % = ±200. The trough always
stays above 180, except ±50 %, where the v1 clamp acts.

## Important firmware fact found while doing this
AM32's control loop does **not** run at 20 kHz.
- `TMR14->pr = 1000000/LOOP_FREQUENCY_HZ` = 50 at 1 MHz gives a 51 µs period, so the loop runs at
  **19,608 Hz** (`Mcu/f421/Src/peripherals.c:170`; the SITL emulates the same timer).
- Fred's v1 computes `phase_inc` and the step lengths for 20 kHz. So on hardware (and in all predictions here):
  - every frequency is **×0.980**: 250 → 245.1, 400 → 392.2, 600 → 588.2, 800 → 784.3, 1000 → 980.4 Hz;
  - every step lasts **×1.020** (3 s → 3.06 s).
- The `f_actual` column is that real frequency. The fix in PROFILE_MODE is a one-liner: compute with
  1e6/(1e6/LOOP_FREQUENCY_HZ + 1).

## Sources (all with the same calibrated motor model)
- **SITL nominal:** AM32 SITL built from bdbc75e plus Fred's hakea_profile.c and main.c hooks (study copy
  `~/vehicles/hakea/am32-sim-profile`).
  - The study copy only adds: a programme override for the frequency/amplitude grid
    (`-DSIM_STUDY_PROGRAMME`), and the modulation exported to the log. The diff is
    `scripts/sim/fred-profile-v1-study-copy.diff`.
  - The exact-programme run uses an ELF without those additions (`sitl_profile_fred.elf`).
  - Settings: DEAD_TIME 47 (= F421's 35 at 120 MHz), 24 kHz PWM (Fred's setting), variable PWM, auto advance.
  - Motor model `notes/sim/sitl_calib/sitl_base_final.json` (kv 1380 SITL-effective, R_phase 0.0772 Ω,
    L_phase 18 µH, J 6.7e-6, kq 1.84e-8, battery 0.037 Ω, commutation_transfer 1.0).
- **SITL pessimistic-demag:** as above with commutation_transfer 0.8. This model desynced bench replays that
  were clean on hardware, so it is a pessimistic bound.
- **SITL 48 kHz PWM:** EEPROM pwm_frequency 48.
- **Averaged model (nonlinear):** `scripts/sim/avgmodel.py`, params_v5all (R 0.158 Ω loop, L 36 µH line-line,
  Kv 1300). The modulation is added after the ramp per tick, clamped to [180, 2000].
- **Analytic linear:** the same averaged model, linearised at the operating point. This is the first-order
  extrapolation.
- **Renode** (F421 instruction-level, Fred's exact PROFILE ELF): see the section below.

## Columns of predictions.csv

| column | meaning |
|---|---|
| `delivered_duty_amp`, `delivered_ratio` | fundamental of the duty actually written to the timer, vs the command |
| `current_swing_A` | DC-equivalent winding current swing = torque swing / K_t |
| `torque_swing_Nm` | fundamental amplitude of electromagnetic torque |
| `torque_lag_deg` | phase lag of the torque fundamental behind the delivered duty (negative = leads) |
| `rpm_ripple_amp` / `_pp` | rpm fundamental amplitude / peak-to-peak |
| `desync`, `slip` | firmware desync count; slip = fraction of time the commutation was out of step with the rotor (ground truth) |
| `copper_W`, `copper_ratio` | motor copper loss (R at 25 °C) and its ratio to no pulsing |
| `regime` | which physics dominates |
| `ticks_per_cycle`, `pwm_periods_per_cycle`, `commutations_per_cycle` | resolution flags |

## Headline predictions (SITL nominal, PROFILE v1, 24 kHz PWM)

| f nominal → actual | ±5 % torque N m / lag | ±10 % torque / lag | ±20 % torque / lag | rpm ripple ±10 % (amp) | copper ×, ±10 % / ±20 % |
|---|---|---|---|---|---|
| 1 Hz | 0.010 / −26° | 0.021 / −26° | 0.041 / −26° | ±2,620 | ×1.07 / ×1.31 |
| 10 → 9.8 | 0.035 / −29° | 0.067 / −27° | 0.135 / −27° | ±1,540 | ×1.7 / ×3.9 |
| 50 → 49 | 0.041 / −3° | 0.082 / −3° | 0.166 / −3° | ±380 | ×2.1 / ×5.4 |
| 100 → 98 | 0.041 / +4° | 0.082 / +4° | 0.167 / +4° | ±190 | ×2.1 / ×5.3 |
| 250 → 245 | 0.040 / +18° | 0.080 / +18° | 0.161 / +18° | ±74 | ×2.0 / ×5.0 |
| 300 → 294 | 0.040 / +22° | 0.079 / +21° | 0.158 / +21° | ±61 | ×2.0 / ×4.9 |
| 400 → 392 | 0.038 / +29° | 0.076 / +28° | 0.151 / +28° | ±44 | ×1.9 / ×4.6 |
| **600 → 588** | 0.035 / +42° | **0.068 / +41°** | **0.136 / +40°** | ±26 | ×1.7 / ×3.9 |
| 800 → 784 | 0.032 / +53° | 0.061 / +51° | 0.121 / +50° | ±18 | ×1.6 / ×3.3 |
| 1000 → 980 | 0.028 / +62° | 0.054 / +60° | 0.108 / +59° | ±12.5 | ×1.5 / ×2.8 |

Context and other cases:
- No-pulsing hover: torque 0.056 N m, copper ≈ 10.8 W, phase current 6.8 A rms.
- ±30 %: 0.256 N m at 50-100 Hz, 0.208 at 600 Hz, 0.164 at 1000 Hz.
- Stock-ramp comparison (sine before ramp 16): the delivered fraction is 1.00 / 0.97 / 0.53 / 0.32 at ±10 % for
  250 / 300 / 600 / 1000 Hz. The torque swing saturates at ≈ 0.10 (250 Hz), 0.083 (300), 0.037 (600) and
  0.018 N m (1 kHz), whatever the amplitude.
- 48 kHz PWM: torque −5…−10 % vs 24 kHz, same shape.

**Desync map.**
- Nominal SITL: **0 desyncs and no slip anywhere**, 1-1000 Hz, ±5…±50 %, for PROFILE v1 and the stock-ramp
  variant.
- Pessimistic SITL:
  - 0 up to ±30 % everywhere;
  - at ±40 %, desyncs at 50 Hz and 150-400 Hz (14-43 % slip), but clean at 500-1000 Hz, where L/R keeps the
    current swing smaller;
  - at ±50 % (clamp at 180 active), loss of sync at nearly every f ≥ 10 Hz.
- **Prediction for hardware: no desyncs up to ±30 % at any f. Risk appears at ±40 % between ~50 and 400 Hz.
  At 600 Hz ±10-20 % (Fred's target) no desync is expected in either model.**

## Fred's exact v1 programme (predictions_programme.csv)

SITL nominal, Fred's unmodified hakea_profile.c logic, base 1048 over 1200 Hz DShot, 24 V.

| step | f actual | amp (duty) | duration | torque swing N m / lag | DC-equiv. current swing | rpm ripple amp / p-p | copper W (× rest) | desync |
|---|---|---|---|---|---|---|---|---|
| 0 settle | – | 0 | 3.06 s | – | – | – | – | 0 |
| 1 | 245.1 Hz | ±40 (±2 %) | 3.06 s | 0.017 / +19° | ±2.5 A | ±16 / 46 | 11.3 (×1.04) | 0 |
| 3 | 245.1 | ±100 | 3.06 | 0.040 / +18° | ±5.8 | ±37 / 85 | 13.7 (×1.26) | 0 |
| 5 | 392.2 | ±100 | 3.06 | 0.038 / +29° | ±5.6 | ±22 / 58 | 13.4 (×1.23) | 0 |
| 7 | 588.2 | ±100 | 3.06 | 0.035 / +42° | ±5.1 | ±14 / 39 | 13.0 (×1.19) | 0 |
| 9 | 588.2 | ±200 | 3.06 | 0.068 / +41° | ±9.9 | ±26 / 64 | 18.9 (×1.74) | 0 |
| 11 | 784.3 | ±100 | 3.06 | 0.032 / +53° | ±4.6 | ±9 / 33 | 12.5 (×1.15) | 0 |
| 13 | 980.4 | ±100 | 3.06 | 0.028 / +62° | ±4.1 | ±6.5 / 29 | 12.1 (×1.12) | 0 |
| rests | – | 0 | 1.02 s each | – | – | (p-p 14-20 rpm noise) | 10.8 | 0 |

- **Gate and restart:** the gate opened ~0.42 s into the 1 s spin-up ramp (zero_crosses > 1000). The programme
  then ran on time, with every step 3.06 s / 1.02 s ±0.02 s.
- After 2 s at zero throttle and a re-spin, the first sine appeared 3.48 s after the throttle came back, i.e.
  the programme had restarted from the settle step. The pessimistic model gives the same timeline, also with no
  desyncs.
- Mean rpm stays at 14,790-14,890 throughout (a +0.6 % rise at ±200).
- **What the bench will see in rpm telemetry:** the bidir eRPM at 1200 frames/s cannot resolve rpm ripple at
  245-980 Hz. Expect only aliasing and noise of ~±20-60 rpm; the ripple itself is ±6-37 rpm. The measurable
  1P effect is torque/current: a phase-current sensor ≥ 10 kHz, a load cell, or EDT2 stress.

**Side effect of the gate.** A firmware desync resets zero_crosses to 0. This closes the gate, so the programme
silently restarts from the settle step. On hardware, a programme that "restarts" mid-run is therefore itself a
desync indicator.

## Regimes and where things dominate
- **< ~15 Hz (mechanical):**
  - the rotor speed follows the throttle, so back-EMF cancels most of the drive;
  - torque swing is small and *leads* the duty by 25-40°;
  - rpm ripple is large (±2,600 rpm at 1 Hz, ±10 %).
- **~15-400 Hz (inertia-dominated):** torque swing is flat at ≈ 0.08 N m per ±10 %. rpm ripple falls as 1/f.
- **≳ 400 Hz (electrical L/R):** torque falls and lag grows.
  - The corner f = R/(2πL) ≈ 700 Hz for R 0.16 Ω and L 36 µH line-line.
  - At 600 Hz: −17 % torque, +41° lag. At 1 kHz: −34 %, +60°.
  - **These numbers rest on the assumed L (not measured).**
  - L sensitivity (averaged model, ±30 %, 300 Hz): L_ll 20 µH → 0.324 N m, 36 µH → 0.309, 70 µH → 0.264.
    At 600-1000 Hz the sensitivity roughly doubles.
- **Resolution:** at 1 kHz there are only 19.6 control ticks, 24 PWM periods and ~10 commutation steps per
  cycle (at 14.8k rpm).
  - The SITL captures ticks and PWM preload, and gives ~7-17° more lag than the averaged model at 600-1000 Hz,
    largely the PWM preload and tick latency.
  - The ~10 commutations per cycle mean the torque waveform is visibly stepped. The fundamental is still
    delivered (SITL), with harmonics at 6× the electrical frequency.

## Model disagreements (where to be careful)
- **Torque level:** the averaged and analytic model is **25-35 % higher** than the SITL everywhere above 10 Hz.
  The causes are the SITL's effective kv 1380 (lower K_t) and its explicit commutation and inductance effects.
  Treat **SITL as the central value and the averaged model as an upper bound.**
- **Lag at 600-1000 Hz:** SITL is 7-17° more than averaged, and 3-7° more than analytic. The difference is the
  tick and PWM latency, which the averaged model doesn't include.
- **Desync:** nominal SITL says never; pessimistic says ±40 % at 50-400 Hz. The truth is probably in between
  (see `am32-ramp-study.md` §7.3).

## What depends on unmeasured assumptions
- **Torque, current and copper loss** depend on the assumed resistance (R ≈ 0.16 Ω, from the T-Motor V2307
  datasheet scaled by Kv²; RPM data cannot fix it). They scale roughly as 1/R.
- **Everything at ≳ 300 Hz** (torque fall-off, lag, current, desync margin) also depends on the assumed
  L ≈ 18 µH/phase.
- **rpm ripple** is fixed by the calibration (inertia/torque ratio), but the large-swing effect seen on the
  bench (§5.3 of the study) may make it optimistic by up to ~1.6× for ≥ ±20 %.

**Confidence**

| quantity | confidence |
|---|---|
| frequencies and step timing, programme gating/restart, ramp-limit behaviour | high (firmware arithmetic) |
| rpm ripple | medium |
| torque, current, copper at ≤ 300 Hz | medium-low (±30-50 %) |
| torque, lag at 600-1000 Hz | low-medium (L) |
| desync boundary | low |
| no desync at ±10-20 % | medium |

## Renode cross-check (Fred's exact PROFILE ELF, F421 instruction-level)

Setup:
- Run by the Renode sub-agent; notes in `notes/sim/renode/v3/renode-v3.md`, data in `renode_programme_v1.csv`
  here.
- `AM32_FOXEER_F421_PROFILE_profile-v1_bdbc75e-dirty.elf` run **unmodified** (sha256 6a64484e…bc5f).
- Same model at 24.0 V, 2 µs physics batch, DShot 10 kHz.

**Renode limitation:** ESCSim's basic-timer model uses period = ARR instead of ARR+1, so Renode's loop runs at
exactly 20,000 Hz and the programme frequencies come out exactly 250 … 1000 Hz. **On hardware expect the ×0.980
values (245.1 … 980.4 Hz) and 3.06 s / 1.02 s steps** (tool-issues #19).

Comparison per programme step (torque swing mN m / lag, rpm ripple amp, copper W):

| step | SITL (actual f) | Renode (20 kHz loop) | difference |
|---|---|---|---|
| 1: 250 Hz ±40 | 17.4 / 19° / 16.2 rpm / 11.3 W | 17.1 / 22° / 15.5 / 11.8 | torque −2 % |
| 3: 250 Hz ±100 | 40.2 / 18° / 37.3 / 13.7 | 40.0 / 20° / 36.2 / 14.2 | −1 % |
| 5: 400 Hz ±100 | 38.4 / 29° / 22.2 / 13.4 | 37.1 / 32° / 21.0 / 13.9 | −3 % |
| 7: 600 Hz ±100 | 35.3 / 42° / 13.7 / 13.0 | 31.5 / 47° / 11.8 / 13.0 | −11 %, +5° |
| 9: 600 Hz ±200 | 68.3 / 41° / 26.4 / 18.9 | 64.4 / 40° / 24.3 / 18.8 | −6 % |
| 11: 800 Hz ±100 | 31.9 / 53° / 9.3 / 12.5 | 27.4 / 53° / 7.7 / 12.5 | −14 % |
| 13: 1000 Hz ±100 | 28.2 / 62° / 6.5 / 12.1 | 24.9 / 59° / 5.6 / 12.3 | −12 % |

Other Renode results:
- Gate opened exactly at zero_crosses = 1000. Steps were exact.
- Restart worked after a gate drop (zero throttle) and after a full disarm, firmware reset and re-arm.
- 0 desyncs and no status events in any programme step. The only events were the normal start-up and stop events,
  also seen on the stock image.
- No missed control ticks, and no main-loop starvation (watchdog kicks ~225k/s, the same as stock).
- tenKhzRoutine timing could not be measured in Renode. Statically the profile code adds ~96 instructions
  (≤ ~1 µs per tick estimated). Measure with DWT->CYCCNT on hardware.
- The PR #438 stress metric reads 255 in every window, including settle, in this emulator (false half-step zero
  crosses with the 2 µs coupling). It carries no step-to-step information here.

Code notes from the Renode agent:
- At 1 kHz the 8-bit sine index gives a ~20-point staircase (12.8 table entries per tick). The fundamental is
  unaffected (fits are within 1 % of the amplitude). Harmonics sit at the tick rate.
- At 14.6k rpm AM32's variable PWM flips ARR between ~4200 and 4999 between ticks. This is stock behaviour, not
  PROFILE_MODE.

**Combined prediction for the hardware programme**
- **Torque swing:** midpoint of SITL and Renode, with ±35 % for the R/L assumptions:

  | step | torque swing | range |
  |---|---|---|
  | 250 Hz ±100 | ≈ 40 mN m | 26-54 |
  | 400 Hz ±100 | 38 mN m | – |
  | 600 Hz ±100 | 33 mN m | 21-45 |
  | 600 Hz ±200 | 66 mN m | 43-89 |
  | 800 Hz ±100 | 30 mN m | – |
  | 1000 Hz ±100 | 27 mN m | 17-36 |

- **Lag behind duty:** 20 / 30 / 45 / 41 / 53 / 60° (±10°, more if L differs).
- **No desyncs.**
- **Copper:** +0.5 to +8 W over the ~11 W rest level.
