# Renode round 3: Fred's PROFILE_MODE firmware (profile-v1), run unmodified

Date: 2026-10-26.

| item | value |
|---|---|
| image | `firmware/AM32_FOXEER_F421_PROFILE_profile-v1_bdbc75e-dirty.elf`, sha256 `6a64484e5ee677c89d77d1ffb147cd92b0af2cbc79e820dea34184135e9ebc5f` (matches `firmware/SHA256SUMS`), loaded unmodified on ESCSim's FOXEER_F421 platform |
| model | `hakea_calibrated_prop_24v0.json`, sha256 `c92a64cee9d2bb367d1cf1d42aac69203101aef7cfe44ca28c64e9be2988fd05` (24.0 V, L 18 µH; copy in `model_used.json`) |
| EEPROM | `~/Downloads/esc1_config.bin`, sha256 `a1ce48e4…f355` |
| throttle | DShot300, unidirectional, **10 kHz** (one frame per 100 µs of virtual time), base 1048 |
| bridge batch | 2 µs |
| stock comparison | `firmware/AM32_FOXEER_F421_head-pr438_bdbc75e.elf`, sha256 `35215759…8688`, same operating point, no programme |

What was written into firmware RAM: only the two PR #438 stress/status variables, at each 50 ms stress "take" (host replica, as in round 2). Nothing else.

Scripts in `scripts/sim/renode/`:
* `run_profile_v3.py`: host driver.
* `hakea_profile_stim_v3.py`: runs inside Renode.
* `analyze_profile_v3.py`: all definitions are in its docstring.
* `elfsyms_v3.py`: resolves file-static symbols such as `hakea_profile.c:step`. main.c has its own `step`, so plain names are ambiguous.

Logging:
* coarse: 1 ms rows, whole run (`raw/profile_v1_coarse.csv`);
* fine: 25 µs rows (`raw/profile_v1_fine.csv`) in these windows:
  * settle, 2.0–2.5 s;
  * each sine step, from +0.5 s to +1.5 s;
  * the restart's step 1.

Outputs: per-step metrics in `summary_v3.csv`; timeline, restart and timing results in `profile_v3_report.json` and `analysis_output.txt`.

## Answers

### 1. Gate and step timing

**The gate opens exactly as `zero_crosses` passes 1000.**
* The motor is running at t = 1.901 s (spin-up from DShot 48 starts at 1.65 s).
* `step_ticks` starts counting at t = 2.0876 s.
* `zero_crosses` interpolates to 999.95 at that instant (995 one ms before, 1004 at the next sample).

**Steps start exactly on schedule, measured to one loop tick from `step_ticks`.**
* Step 0 (settle): 3.000 s from the gate opening.
* Sine steps: 3.000 s each. Rests: 1.000 s each.
* The programme ends (step 15, plain base) at gate + 31.000 s.

**Frequencies, measured from the delivered `adjusted_duty_cycle`:** 250.002, 250.000, 399.999, 599.999, 600.000, 800.000, 1000.000 Hz.

> **These are NOT the ~2 % low values expected on hardware, because Renode's loop runs at exactly 20 000 ticks/s.**
>
> * ESCSim's `AM32_STM32_BasicTimer.cs` sets the timer limit to ARR (`timer.Limit = value & 0xFFFF`). A real STM32/AT32 up-counter's period is ARR+1. So TMR14 with pr = 50 at 1 MHz gives a 50 µs period in Renode, against 51 µs (19 608 Hz) on hardware.
> * Measured tick rate: 20 000.02/s from `step_ticks`, and 19 996–20 000/s from `tenkhzcounter` for both the profile and the stock image.
> * On Fred's ESC everything should therefore come out 1.96 % slow: 245.1 / 392.2 / 588.2 / 784.3 / 980.4 Hz and 3.06 s / 1.02 s steps. The emulator cannot confirm this; it is arithmetic from the timer definition.
> * The same off-by-one applies to every timer modelled by `AM32_STM32_BasicTimer` (TMR16 commutation timer, TMR3, …): one tick short per period.
> * This is a note for ESCSim; I edited nothing there.
> * It also explains why the SIM_PROFILE trial (below) measured 1020 Hz when 1000 Hz was requested with dphase computed for 19 607.8 Hz.

### 2. Restart

**(a) Gate drop.** DShot 0 for 1 s at t = 33.47 s (the motor stops, `running` = 0), then a ramp back to 1048.
* The programme restarted from step 0.
* The gate reopened at t = 35.469 s with `zero_crosses` = 1000.4.
* Step 1 started at 38.469 s, i.e. after 3.000 s.
* The restarted step 1 measured 250.0006 Hz with ±40.1 amplitude.

**(b) Full disarm and re-arm.**
* The DShot wire was turned off at t = 40.15 s.
* After AM32's 0.5 s armed signal timeout the firmware calls `NVIC_SystemReset`; at 40.65 s Renode reset the MCU. `armed` = 0, the IWDG kick counter reset to 0, `tenkhzcounter` reset to 0, and the motor coasted down.
* The wire was restored at zero throttle. The firmware rebooted, played the startup tune and re-armed at 42.25 s.
* After spin-up the gate opened at 42.739 s (`zero_crosses` = 1000.0) and step 1 began at 45.739 s (3.000 s after the gate).
* So the programme restarts from step 0 on re-arm, as designed.

### 3. Desyncs and status events

There were none during the programme: 0 `desync_happened` increments and no status events in any step 0–14.

Events seen outside the programme:
* spin-up, before the gate opens, at ≈2.8k RPM: 1 desync + 1 bemf timeout (status ALERT, then WARNING);
* the gate-drop stop: 1 bemf timeout ALERT at 34.31 s;
* the re-arm spin-up: the same pattern as the first spin-up.

These are normal AM32 start/stop events and also occur on the stock image.

**PR #438 stress (50 ms windows) reads 255 in every window, including settle**, as in round 2 with the 2 µs batch. False zero crosses at half a step pin the peak-based metric, so in this emulator it carries no step-to-step information.

### 4. Watchdog and tenKhzRoutine timing

**Watchdog.** ESCSim's IWDG model counts kicks but never resets the CPU, so a watchdog reset cannot occur in Renode. Kick counts:

| image | kicks/s | max gap between kick increments (at 1 ms sampling resolution) |
|---|---|---|
| profile, while running | ≈ 225 000 | ≤ 2 ms |
| stock PR #438 | ≈ 227 000 | ≤ 2 ms |

The configured timeout is ≈ 3.2 s (`WDT->div` /32, `rld` 4000, LSI ≈ 40 kHz). So there is no main-loop starvation in either image.

**Missed ticks.** None. Ticks per second over the whole programme equal the timer rate (20 000.02, regression of `step_ticks`). Every step runs exactly 60 000 or 20 000 ticks per 3 or 1 s. The stock image gives 19 999.9/s from `tenkhzcounter`.

**tenKhzRoutine execution time could not be measured in the emulator.**
* Renode's `ExecutedInstructions` only advances at translation-block/sync granularity: values seen in hooks were multiples of 6000 + 1044.
* Address hooks on `tenKhzRoutine` / `TMR14_GLOBAL_IRQHandler` fired only sporadically: 116 times in about 1.4 s.
* Renode is not cycle-accurate anyway: 1 instruction per cycle, no flash wait states.

**Static size (what was added).**
* `tenKhzRoutine` grows from 0x5d8 to 0x648 bytes, and adds calls to `hakea_profile_tick` (0xd4 bytes) and `hakea_profile_apply` (0x28 bytes): 96 instructions in total across both.
* The executed path per tick is a few dozen instructions, roughly ≤1 µs at 120 MHz even with flash wait states, against a 51 µs tick. This is an estimate, not a measurement.
* Measuring it on hardware needs DWT->CYCCNT, which the F421 (Cortex-M4) has.

### 5. Per-step results (2 µs batch, fine-log windows of 1.0 s, i.e. 249–999 whole cycles)

Column definitions:
* **delivered mod:** fundamental amplitude of `(adjusted_duty_cycle-1)*2000/tim1_arr − last_duty_cycle`, in duty units out of 2000.
* **fw mod:** the firmware's own `modulation` variable.
* **idc:** "DC-equivalent" current, (|ia|+|ib|+|ic|)/2, the current in the two conducting phases.
* **ibus:** physics bus current.
* **τ:** electromagnetic torque reconstructed as in motor.c, ke·Σ shape(θe+p·120°)·i_p. Check: at settle the mean τ is 54.1 mN·m, against a model load torque of 52.0 mN·m.
* **Lags:** relative to the delivered modulation; positive means the quantity lags.
* **Pcu:** 3·Irms²·0.0772.
* **Settle reference (step 0):** 14 608 RPM, iph_rms 6.97 A, Pcu 11.25 W, idc 8.14 A, ibus 3.99 A.

| step | prog | f meas (Hz) | delivered mod amp | fw mod amp (min/max) | RPM mean | RPM amp / p-p | RPM lag ° | idc amp (A) / lag ° | ibus amp (A) | τ amp (mN·m) / lag ° | iph rms (A) | Pcu (W) / Δ vs settle | desyncs | stress med/max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | settle | – | – | – | 14607.8 | – | – | – | – | – | 6.969 | 11.25 / 0 | 0 | 255.0/255 |
| 1 | 250 Hz ±40 | 250.002 | 40.1 | 39.9 (-40/39) | 14607.7 | 15.48 / 30.96 | 112.0 | 2.633 / 22.4 | 1.412 | 17.088 / 22.2 | 7.142 | 11.81 / +0.56 | 0 | 255.0/255 |
| 3 | 250 Hz ±100 | 250.0 | 100.29 | 99.92 (-100/99) | 14647.9 | 36.19 / 72.38 | 110.1 | 6.057 / 20.3 | 3.316 | 39.998 / 20.4 | 7.842 | 14.24 / +2.99 | 0 | 255.0/255 |
| 5 | 400 Hz ±100 | 399.999 | 100.24 | 99.87 (-100/99) | 14662.6 | 21.0 / 42.01 | 122.0 | 5.669 / 32.0 | 3.097 | 37.122 / 32.2 | 7.754 | 13.93 / +2.68 | 0 | 255.0/255 |
| 7 | 600 Hz ±100 | 599.999 | 101.09 | 99.95 (-100/99) | 14583.8 | 11.84 / 23.68 | 137.1 | 4.829 / 47.8 | 2.606 | 31.462 / 47.2 | 7.48 | 12.96 / +1.71 | 0 | 255.0/255 |
| 9 | 600 Hz ±200 | 600.0 | 200.72 | 199.72 (-200/199) | 14829.5 | 24.3 / 48.59 | 129.8 | 8.697 / 40.5 | 5.212 | 64.373 / 39.9 | 9.014 | 18.82 / +7.57 | 0 | 255.0/255 |
| 11 | 800 Hz ±100 | 800.0 | 100.61 | 99.73 (-100/99) | 14542.8 | 7.7 / 15.41 | 142.4 | 4.123 / 53.0 | 2.25 | 27.361 / 52.5 | 7.331 | 12.45 / +1.2 | 0 | 255.0/255 |
| 13 | 1000 Hz ±100 | 1000.0 | 100.92 | 99.47 (-100/99) | 14552.1 | 5.64 / 11.28 | 149.4 | 3.788 / 59.3 | 2.06 | 24.862 / 59.4 | 7.278 | 12.27 / +1.02 | 0 | 255.0/255 |

Rest steps (2, 4, …, 14): no desyncs, no status events; stress 255.

Notes:
* **Clamp.** The firmware modulation stays inside ±amp (−100…99, −200…199). With base duty 1020 the clamp [180, 2000] is never reached in this programme.
* **Delivered vs firmware modulation.** The delivered fundamental sits 0.4–1.4 units above the firmware's own value, rising with frequency. That is consistent with integer truncation in `duty*tim1_arr/2000` plus the 2000/ARR reconstruction quantum.
* **Variable-PWM sampling skew.** About 10 % of 25 µs samples pair an `adjusted_duty_cycle` computed in the last tick with a `tim1_arr` the main loop has just changed. The cause: at 14.6k RPM, `commutation_interval` sits at the edge of the variable-PWM map, so ARR flips between about 4200 and 4999 (PWM 24–28 kHz) between ticks. Reconstructed duty spikes by up to about ±150 units in those samples, and the same thing happens at settle with no modulation. It is stock AM32 variable-PWM behaviour, not PROFILE_MODE, and it does not affect the fitted fundamentals. Whether the timer registers on hardware briefly see CCR/ARR from different ticks depends on preload settings; I did not check.
* **Resolution at 1 kHz.**
  * Control ticks per cycle: 20.0 (Renode) / 19.6 (hardware).
  * Sine table: the top 8 bits of phase with no interpolation, phase_inc = 2^32·1000/20000, so 12.8 table entries per tick, i.e. effectively a ~20-point staircase.
  * PWM periods per cycle: ≈ 24–28.
  * Commutations per cycle: ≈ 10.2 (6·7·14 550 RPM/60 = 10.2 kHz).

  At 250 Hz these are 80 ticks, ~100 PWM periods and ~41 commutations per cycle.
* **Torque.** Torque amplitude per ±100 duty units: 40.0 mN·m (250 Hz), 37.1 (400), 31.5 (600), 27.4 (800), 24.9 (1000). ±200 at 600 Hz gives 64.4, so it is close to linear in amplitude (2.05× at 600 Hz).
* **Torque lag** behind the delivered duty: 20° at 250 Hz rising to 59° at 1 kHz. That is consistent with the L/R plus back-EMF current dynamics: Lr ≈ 18 µH / 0.077 Ω ≈ 0.23 ms; 1 kHz → atan(2π·1000·0.23e-3) ≈ 56°.
* **Mean speed.** ±200 at 600 Hz raises mean RPM by +222. The ±100 steps shift it by −65 to +55 RPM.
* **Copper loss above settle.** +0.6 W (250 Hz ±40), +3.0 W (250 Hz ±100) falling to +1.0 W (1 kHz ±100), +7.6 W (600 Hz ±200).

## SIM_PROFILE study patch (superseded)

The first round-3 request used `scripts/sim/firmware-profile-study.patch` (sha256 `656bf2ed…aa37`).
* It was applied in `am32-renode-pr438`, replacing the ramp patch, and built as:
  * `variants/FOXEER_F421_prof_ramp16.elf`, sha256 `3b23b9b1360064960e3dfbf948e0db71d4b61623039a8f54980af5cd2e985b54`;
  * `…_prof_ramp255.elf`, sha256 `1a47b616426b4f1bfecc89aef18acf91463498bcd6af04de350514076a21b3a4`.
* Only one trial run was made before the change of plan: ramp16, nominal 1000 Hz, ±200.
* Its data and analysis are in `sim_profile_trial/`. The scripts are `run_crosscheck_v3.py`, `hakea_stim_v3.py` and `analyze_v3.py`.

Results of that trial:
* Measured command frequency 1020.0 Hz, for the reason explained in answer 1.
* The stock ramp limit (16 per tick) cut the ±199 command to ±63.5 delivered, with 61° lag.
* RPM amplitude was 4.2.
* It validated the torque reconstruction: mean τ at settle 54.0 against a 52.0 mN·m load model.

The rest of the SIM_PROFILE matrix was dropped as instructed.

## Blockers and limits

* **Loop rate.** Renode's loop rate is 20 000 Hz instead of the hardware's 19 608 Hz (ESCSim BasicTimer off-by-one), so frequency and duration values are nominal in Renode. The hardware values follow arithmetically.
* **No watchdog reset model and no cycle-accurate timing in Renode.** I could not measure tenKhzRoutine duration or interrupt lateness; the evidence I do have is the missed-tick count, which is 0.
* **Stress metric.** It saturates with the 2 µs batch (round-2 finding).
* **Phase current RMS** comes from 25 µs samples, asynchronous to PWM and commutation; it is an estimate.
