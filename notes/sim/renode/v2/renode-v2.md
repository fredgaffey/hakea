# Renode cross-check, round 2: PR #438 firmware, calibrated model

Date: 2026-10-26.

* **Firmware:** FOXEER_F421 (AM32 head a47e94c + PR #438, commit bdbc75e).
* **Model:** calibrated prop model with **L = 18 µH**.
* **ESC settings:** Fred's `esc1_config.bin`.
* **Throttle input:** DShot300, unidirectional, one frame per 100 µs of virtual
  time (10 kHz), base 1048. Everything is stepped in virtual time.
* **Run layout:**
  * ramp 48→1048 over 0.5 s;
  * hold 1.0 s;
  * sine at f (250 or 300 Hz) for 1.1 s; the analysis skips the first 0.1 s
    and uses exactly 1.0 s (250 or 300 whole cycles);
  * then 0 for 0.3 s.

  One (f, amplitude) pair per run.

Scripts are in `scripts/sim/renode/*_v2*`. Raw CSV and JSON per run are in
`raw/`, all metrics are in `summary_v2.csv`, and the column definitions are
in the `analyze_v2.py` docstring.

## Key findings

1. **No desyncs, and no WARNING/ALERT/ERROR status events, in any sine
   segment.** That covers all 45 runs:
   * stock, ramp 64 and ramp 255;
   * 250 and 300 Hz;
   * ±100 to ±1000;
   * 2 µs and 20 µs physics batch.

   The only `desync_happened` increments came at startup (1, the normal AM32
   open-loop handover) and at the final cut to 0 throttle (0–4).
2. **Stock ramp (16/tick = 320 000 duty/s) is the binding limit.**

   | applied duty fundamental | 250 Hz | 300 Hz |
   |---|---|---|
   | ±100 | 98 | 98 |
   | ±200 | 195 | 190 (starting to limit) |
   | ±400 to ±1000 | 259 (saturated) | 216 (saturated) |

   With ramp 64 or 255, ±100 to ±600 are tracked in full (586–588 of 600).
3. **±1000 with ramp 64/255 is a trap.**
   * The trough drives `last_duty_cycle` below 150. The firmware then uses
     `max_ramp_startup` = 2 per tick
     (`zero_crosses < 150 || last_duty_cycle < 150` in `tenKhzRoutine`).
   * Duty climbs back out at only 40 000/s, so it spends most of each cycle
     near 100.
   * Mean duty drops by 390–465 and RPM collapses from about 14.9k to
     8.4–9.4k.
   * The stock ramp never gets there, because the swing is limited to ±216–259
     around 1020.
   * **Any cyclic trough that takes duty below about 150 (≈ DShot 180, about
     7 % throttle) engages the startup ramp.** This is real AM32 logic, not an
     emulation effect.
4. **The PR #438 stress metric depends on how the emulator couples the
   comparator** (details below). With the 2 µs batch it reads **255 in every
   window, even at steady throttle**, so it carries no amplitude information.
   With ESCSim's stock 20 µs batch it sits at 53 at steady throttle (that
   floor is 21 µs batch quantisation) and does follow amplitude:
   * 53–70 while the applied duty swing is ≤ ±216;
   * **255 once the applied swing is ≥ ±390** (ramp 64/255 at ±400 and
     ±600). From that point about 9 % of zero crosses fall at half or 1.5
     steps.
   * At ±1000 with ramp 64/255 (the collapsed case) the median is 49–244 and
     the max is 255.
5. **Copper loss:** 3·Irms²·0.0772, from physics phase currents sampled every
   100 µs.

   | case | Pcu |
   |---|---|
   | settle | 11.6 W |
   | stock (any amplitude ≥ ±400) | 23–29 W |
   | ramp 64/255, ±400 | 45–47 W |
   | ramp 64/255, ±600 | 99–103 W |
   | ramp 255, ±1000 (collapsed) | 209–292 W |

   Bus current rises from 4.1 A at settle to 10.5 A at ±600 with full
   tracking.

## Firmware and ELF hashes (sha256)

| variant | ELF | sha256 |
|---|---|---|
| stock (run) | `firmware/AM32_FOXEER_F421_head-pr438_bdbc75e.elf` (Fred's flashed image, used as-is) | `352157592199b86f15634d08e2b25855e32179b73a6050b5783ba5b82ec68688` |
| stock (my rebuild, check only) | `am32-renode-pr438/variants/FOXEER_F421_stock.elf` | `4b1f3588ddad834e5f65cb003e2724a2a211687030cecbe2266b7f6a96a781ce` |
| ramp64 | `am32-renode-pr438/variants/FOXEER_F421_ramp64.elf` | `1a1c3253670d2a0b97a648bf66de46286d8e89c9f30e88c3bdd1aebe9c1f4fd1` |
| ramp255 | `am32-renode-pr438/variants/FOXEER_F421_ramp255.elf` | `221153f35fc3bd90398922824d6eb089afe27309720f721f4a0c1dd55298fe0e` |

**Build.**
* Worktree: `~/vehicles/hakea/am32-renode-pr438`, detached at bdbc75e, with
  `scripts/sim/ramp-study.patch` applied (uncommitted). It is in `.gitignore`.
* Toolchain: xpack arm-none-eabi-gcc 10.3.1-2.3, via a symlink
  `tools/linux/xpack-arm-none-eabi-gcc-10.3.1-2.3` →
  `am32-edt2/tools/linux/...`. Nothing was copied or modified in am32-edt2.

**Stock rebuild check.** My stock build from bdbc75e with the patch and no
SIM_CFLAGS gives a loadable image (`objcopy -O binary`) that is
**byte-identical** to Fred's ELF: both have sha256 `9cf387026ad4b5cd8aa70969ebe10a4eb127b67fa3bede8c78471b2e7d9ac6d5`. The
ELF files themselves differ, as expected, because they embed debug info and
build paths.

## Model (sha256 69749a5ed9bead7bd9c7e1929101ebfe31411420208bbf6e0eaccdb7a91e2923; copy in model_used.json)

```json
{"motor": {"kv": 1380.0, "poles": 14, "resistance": 0.07715877327818697, "inductance": 1.8e-05,
  "mutual_inductance": 0.0, "inertia": 6.696786724630156e-06, "damping": 5.792168694968939e-06,
  "static_friction": 0.002, "load_k_omega2": 1.8425621926787373e-08},
 "battery": {"voltage": 24.3, "resistance": 0.036977286310262855, "capacitance": 0.0},
 "esc": {"rds_on": 0.002, "diode_vf": 0.7, "commutation_transfer": 1.0, "temperature_c": 38}}
```

Steady state at DShot 1048 (duty 1020), all variants:

| batch | RPM | bus current | phase current rms | Pcu |
|---|---|---|---|---|
| 2 µs | 14 881–14 887 | 4.1 A | 7.1 A | 11.6 W |
| 20 µs | 14 881 | 4.1 A | 6.85 A | 10.9 W |

## How the stress and status values were read

PR #438 has two parts:
* **In the main control path (runs without EDT):**
  * `dshot_note_zero_cross_interval()` keeps a running peak of |Δ
    zero-cross interval| in `stress_window_peak_delta`;
  * `dshot_note_status_event()` ORs events into `pending_status_events`
    (ALERT = bemf timeout, WARNING = desync, ERROR = stuck rotor).
* **In `make_dshot_package()` (only when telemetry is sent):**
  * `dshot_take_commutation_stress()` computes
    `min(255, peak*255/thiszctime)` and resets the window. It runs only on
    bidirectional DShot with EDT enabled.

So on the unidirectional wire the host repeats `dshot_take_commutation_stress()`
exactly, from emulated RAM, every **50 ms** window. It sets
`stress_window_reset_requested`, reads the peak and `thiszctime`, then reads
and clears `pending_status_events`. Those PR variables are the only firmware
state written, and nothing else reads them. `edt_max_comm_stress` in the CSV
is the maximum window value while armed; the status frame would carry it >>4.

On hardware, ArduPilot's EDT rate sets the window: about every 200 eligible
telemetry frames, or roughly 250 ms at 800 Hz. A longer window can only raise
a peak-based value.

**I could not run the firmware's own EDT path.** See Blockers.

## Why stress saturates at the 2 µs batch (and the earlier 35 µH result)

`thiszctime` is in 0.5 µs ticks; one 60° step at 14.9k RPM is about 192
ticks. With the 2 µs batch:
* about 7 % of zero crosses (18 µH) come at about half a step (~96 ticks),
  each followed by one at about 1.5 steps (~290);
* this happens at steady throttle too;
* it is consistent with a false zero cross right after commutation
  (demagnetisation / comparator chatter), which the finer batch delivers to
  the EXTI in time;
* one such event per window makes the peak delta about equal to the interval,
  so stress reads 255.

With the 20 µs batch those events disappear at steady throttle. Intervals
instead alternate 160/200 ticks: the 21 µs batch quantisation, which gives the
53 floor. They reappear (about 9 %) only when the applied duty swing is ≥
±390.

**L sensitivity, 35 µH model (the earlier version of the file).** Data is in
`l35_sensitivity/`.

| run | half-step zero crosses | stress |
|---|---|---|
| 35 µH, 2 µs batch, stock, 300 Hz ±600 | about 20 % at settle | 255 throughout |
| 35 µH, 20 µs batch, steady | none | 53 |
| old hakea_2307_5in (15 µH), 2 µs batch | rare | mostly 10–12, occasional 255 windows |

Every metric other than stress was close to the 18 µH result. At 35 µH,
2 µs, stock, 300 Hz ±600: duty 431 p-p, RPM 101 p-p, no desyncs.

**Which coupling is closer to hardware cannot be decided in the emulator.**
* The peak-based stress metric is extremely sensitive to single displaced zero
  crosses.
* On hardware, a few false zero crosses per window would pin it at 255.
* The EDT stress values Fred's ESC reports are the real test.

## Results

The columns are:
* **amp@f / p-p@f:** amplitude and peak-to-peak of the 250/300 Hz least-squares
  fundamental.
* **duty:** `last_duty_cycle`, the committed post-ramp duty on a 0–2000 scale.
* **duty shift:** sine-segment mean minus settle mean.
* **stress:** per 50 ms window, settle and sine.
* **zc odd:** fraction of samples whose zero-cross interval is off by ≥ ¼ step.
* **Pcu:** 3·Irms²·0.0772.

#### batch 2 us

| variant | f | amp | RPM mean | RPM amp@f | RPM p-p@f | duty amp@f | duty p-p@f | duty shift | desync | stress settle med | stress med | stress max | status | zc odd | Ibus A | Iph rms A | Pcu W |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ramp255 | 250 | 100 | 14852.8 | 33.2 | 66.4 | 98.3 | 196.6 | -1.3 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.169 | 4.16 | 7.9 | 14.47 |
| ramp255 | 250 | 200 | 14897.8 | 66.6 | 133.1 | 195.8 | 391.6 | -1.2 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.2 | 4.77 | 9.8 | 22.25 |
| ramp255 | 250 | 400 | 15173.7 | 122.8 | 245.6 | 391.8 | 783.5 | -0.8 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.171 | 6.97 | 14.18 | 46.55 |
| ramp255 | 250 | 600 | 15020.2 | 205.9 | 411.9 | 587.8 | 1175.6 | -1.0 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.13 | 10.5 | 21.09 | 103.01 |
| ramp255 | 250 | 1000 | 13856.9 | 359.6 | 719.2 | 1009.3 | 2018.6 | -45.8 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.125 | 19.16 | 35.5 | 291.85 |
| ramp255 | 300 | 100 | 14835.8 | 27.0 | 54.0 | 98.2 | 196.3 | -1.2 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.161 | 4.18 | 7.83 | 14.22 |
| ramp255 | 300 | 200 | 14889.2 | 54.1 | 108.3 | 195.8 | 391.6 | -1.4 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.195 | 4.67 | 9.69 | 21.74 |
| ramp255 | 300 | 400 | 15130.4 | 99.6 | 199.2 | 391.9 | 783.9 | -0.9 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.189 | 6.64 | 13.94 | 45.02 |
| ramp255 | 300 | 600 | 15072.0 | 167.9 | 335.8 | 587.4 | 1174.8 | -1.6 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.12 | 10.25 | 20.7 | 99.23 |
| ramp255 | 300 | 1000 | 8708.7 | 186.6 | 373.2 | 619.2 | 1238.4 | -445.0 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.114 | 12.01 | 30.05 | 209.08 |
| ramp64 | 250 | 100 | 14852.8 | 33.2 | 66.4 | 98.3 | 196.6 | -1.3 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.169 | 4.16 | 7.9 | 14.47 |
| ramp64 | 250 | 200 | 14897.8 | 66.6 | 133.1 | 195.8 | 391.6 | -1.2 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.2 | 4.77 | 9.8 | 22.25 |
| ramp64 | 250 | 400 | 15173.7 | 122.8 | 245.6 | 391.8 | 783.5 | -0.8 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.171 | 6.97 | 14.18 | 46.55 |
| ramp64 | 250 | 600 | 15014.6 | 204.2 | 408.3 | 586.4 | 1172.7 | -1.4 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.136 | 10.56 | 20.96 | 101.78 |
| ramp64 | 250 | 1000 | 9413.4 | 238.7 | 477.4 | 650.7 | 1301.3 | -392.8 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.124 | 9.24 | 25.85 | 154.81 |
| ramp64 | 300 | 100 | 14835.8 | 27.0 | 54.0 | 98.2 | 196.3 | -1.2 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.161 | 4.18 | 7.83 | 14.22 |
| ramp64 | 300 | 200 | 14889.2 | 54.1 | 108.3 | 195.8 | 391.6 | -1.4 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.195 | 4.67 | 9.69 | 21.74 |
| ramp64 | 300 | 400 | 15127.4 | 99.9 | 199.9 | 391.6 | 783.2 | -1.0 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.193 | 6.73 | 13.99 | 45.34 |
| ramp64 | 300 | 600 | 15060.5 | 168.7 | 337.4 | 585.3 | 1170.7 | -1.7 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.116 | 10.54 | 20.76 | 99.84 |
| ramp64 | 300 | 1000 | 9055.3 | 144.7 | 289.3 | 480.6 | 961.2 | -420.3 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.135 | 8.42 | 23.96 | 132.96 |
| stock | 250 | 100 | 14853.9 | 33.3 | 66.5 | 98.3 | 196.6 | -1.3 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.161 | 4.2 | 7.9 | 14.44 |
| stock | 250 | 200 | 14888.7 | 66.2 | 132.5 | 195.2 | 390.5 | -1.3 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.205 | 4.69 | 9.81 | 22.27 |
| stock | 250 | 400 | 14966.9 | 86.6 | 173.2 | 259.1 | 518.1 | -1.1 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.182 | 5.42 | 11.24 | 29.24 |
| stock | 250 | 600 | 14783.0 | 91.2 | 182.4 | 259.1 | 518.3 | -20.1 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.113 | 5.26 | 11.29 | 29.52 |
| stock | 250 | 1000 | 14952.0 | 86.6 | 173.2 | 258.1 | 516.2 | -1.1 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.188 | 5.36 | 11.26 | 29.37 |
| stock | 300 | 100 | 14830.9 | 26.6 | 53.3 | 98.1 | 196.2 | -1.2 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.165 | 4.17 | 7.81 | 14.13 |
| stock | 300 | 200 | 14881.8 | 52.5 | 104.9 | 190.2 | 380.4 | -1.6 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.201 | 4.69 | 9.56 | 21.17 |
| stock | 300 | 400 | 14890.5 | 58.4 | 116.9 | 215.6 | 431.2 | -1.2 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.206 | 4.98 | 10.07 | 23.47 |
| stock | 300 | 600 | 14884.7 | 58.4 | 116.9 | 215.6 | 431.2 | -1.0 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.202 | 4.95 | 10.06 | 23.44 |
| stock | 300 | 1000 | 14905.3 | 58.5 | 117.1 | 215.6 | 431.1 | -0.1 | 0 | 255.0 | 255.0 | 255 | 0x00 | 0.206 | 4.77 | 10.08 | 23.56 |

#### batch 20 us

| variant | f | amp | RPM mean | RPM amp@f | RPM p-p@f | duty amp@f | duty p-p@f | duty shift | desync | stress settle med | stress med | stress max | status | zc odd | Ibus A | Iph rms A | Pcu W |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ramp255 | 300 | 100 | 14905.6 | 30.1 | 60.1 | 98.2 | 196.5 | -1.3 | 0 | 53.0 | 53.0 | 66 | 0x00 | 0.0 | 4.19 | 7.66 | 13.59 |
| ramp255 | 300 | 200 | 14971.2 | 59.3 | 118.7 | 195.9 | 391.7 | -1.5 | 0 | 53.0 | 53.0 | 70 | 0x00 | 0.0 | 4.88 | 9.61 | 21.38 |
| ramp255 | 300 | 400 | 14883.1 | 111.2 | 222.3 | 391.9 | 783.8 | -1.3 | 0 | 53.0 | 255.0 | 255 | 0x00 | 0.092 | 7.06 | 14.7 | 50.03 |
| ramp255 | 300 | 600 | 14793.1 | 176.9 | 353.7 | 588.1 | 1176.1 | -1.5 | 0 | 53.0 | 255.0 | 255 | 0x00 | 0.073 | 10.71 | 21.71 | 109.17 |
| ramp255 | 300 | 1000 | 8428.3 | 203.4 | 406.8 | 635.7 | 1271.4 | -465.8 | 0 | 53.0 | 49.0 | 255 | 0x00 | 0.006 | 12.51 | 31.06 | 223.44 |
| ramp64 | 300 | 100 | 14905.6 | 30.1 | 60.1 | 98.2 | 196.5 | -1.3 | 0 | 53.0 | 53.0 | 66 | 0x00 | 0.0 | 4.19 | 7.66 | 13.59 |
| ramp64 | 300 | 200 | 14971.2 | 59.3 | 118.7 | 195.9 | 391.7 | -1.5 | 0 | 53.0 | 53.0 | 70 | 0x00 | 0.0 | 4.88 | 9.61 | 21.38 |
| ramp64 | 300 | 400 | 14883.5 | 111.0 | 222.0 | 391.5 | 783.1 | -1.3 | 0 | 53.0 | 255.0 | 255 | 0x00 | 0.091 | 7.15 | 14.69 | 49.99 |
| ramp64 | 300 | 600 | 14792.1 | 176.7 | 353.5 | 585.8 | 1171.7 | -1.5 | 0 | 53.0 | 255.0 | 255 | 0x00 | 0.076 | 10.81 | 21.69 | 108.99 |
| ramp64 | 300 | 1000 | 8631.1 | 174.9 | 349.8 | 548.2 | 1096.4 | -447.5 | 0 | 53.0 | 244.0 | 255 | 0x00 | 0.008 | 9.05 | 25.75 | 153.61 |
| stock | 300 | 100 | 14911.1 | 30.1 | 60.2 | 98.1 | 196.3 | -1.3 | 0 | 53.0 | 53.0 | 66 | 0x00 | 0.0 | 4.23 | 7.66 | 13.59 |
| stock | 300 | 200 | 14964.9 | 58.1 | 116.2 | 190.2 | 380.5 | -1.6 | 0 | 53.0 | 53.0 | 70 | 0x00 | 0.0 | 4.87 | 9.5 | 20.9 |
| stock | 300 | 400 | 15011.3 | 64.3 | 128.7 | 215.7 | 431.3 | -0.7 | 0 | 53.0 | 55.0 | 70 | 0x00 | 0.0 | 5.11 | 10.03 | 23.31 |
| stock | 300 | 600 | 15012.0 | 64.3 | 128.5 | 215.7 | 431.4 | -0.5 | 0 | 53.0 | 67.5 | 255 | 0x00 | 0.002 | 5.1 | 10.05 | 23.37 |
| stock | 300 | 1000 | 15023.4 | 64.2 | 128.3 | 215.6 | 431.3 | 0.5 | 0 | 53.0 | 56.0 | 255 | 0x00 | 0.004 | 5.07 | 10.04 | 23.35 |

## Blockers

1. **The firmware's own EDT path did not run: bidirectional DShot stops
   decoding in my scripted Renode setup.**
   * Setup: `AM32ThrottleGenerator.Bidirectional = True`, set directly rather
     than through ESCSim's GUI link.
   * Symptom: the firmware detects inverted DShot (`dshot_telemetry = 1`),
     arms, and then `dshot_goodcounts` freezes after exactly 1.0 s of frames
     (3899 good + 100 bad at 250 µs; 1899 + 100 at 500 µs). Command 13 is
     therefore never decoded, and `dshot_extended_telemetry` stays 0.
   * The result is the same with the 2 µs or 20 µs batch, with or without the
     telemetry bit, and with the older head firmware (round-1 ramp16 ELF). So
     it is not PR #438.
   * ESCSim's own matrix passes BDShot on F421 through its GUI-link path, so
     the cause is probably something that path does and my driver does not.
     Not investigated further.
   * Reproduce with `scripts/sim/renode/edt_bidir_debug_v2.py` (env `B`, `FR`,
     `TB`, `ELF`; monitor on TCP 34574).
   * Instead, stress and status were computed by replicating the take
     function from RAM, as described above.
2. **Phase current RMS** comes from 100 µs samples, asynchronous to the
   electrical period (≈1.7 kHz at 14.9k RPM) and to the 24 kHz PWM. It is a
   statistical estimate, good to a few percent, not an integral.
3. **The command / duty phase has 100 µs of measurement latency** (one
   sample), which shows up as 9–11° of duty lag.
