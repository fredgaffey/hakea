# AM32 ramp-limit / 1P pulsing simulation study (Hakea)

Date: 2026-10-26. Author: Claude (autonomous study for Fred). Status: complete apart from the items listed under "Not done".

**Model fidelity, read first.**
- Everything here comes from simulation. It is calibrated only against RPM telemetry and the FC battery voltage
  from bench logs 33 (prop off) and 34 (prop on). There is no current or torque measurement.
- RPM predictions below ~50 Hz are verified against the bench (errors given below).
- Everything at 1P (240-300 Hz) is extrapolated. So is every torque, current and heating number, and every
  desync statement.
- Absolute torque, current and heating depend on an assumed winding resistance (0.16 Ω total loop, from datasheet
  scaling) and an assumed inductance (18 µH per phase). Neither is measured.

## 1. Short answers

| Question | Answer (simulation) | Confidence |
|---|---|---|
| Does the stock ramp limit (16 duty/tick) clip 1P? | **Yes.** At 250 Hz anything above about ±10 % throttle is clipped to a triangle. The applied duty swing saturates at ±0.103 N m of torque (≈ ±13 % throttle-equivalent). At 300 Hz it saturates at ±0.084 N m and at 400 Hz at ±0.061 N m. Firmware arithmetic, independent of the motor model. | high |
| Does removing it help? | **Yes.** With ramp 64 or 255 the applied duty follows the command up to ±40 % at 250-300 Hz (ramp 64 starts to clip at ±40 % / 300 Hz). The torque swing is then linear in amplitude: about 0.080 / 0.16 / 0.25 / 0.33 N m for ±10/20/30/40 % at 250 Hz, against a hover torque of about 0.056 N m. Ramp 32 clips from ±30 % at 250 Hz. | medium (torque scale ±30-50 %) |
| RPM ripple at 1P | Small: ±73 rpm (±10 %) to ±225 rpm (±30 %) at 250 Hz, about ±0.5-1.5 % of 15k rpm. The torque swing is the useful quantity, not the rpm swing. | medium |
| Where does sensorless commutation desync? | Nominal SITL: **no desync anywhere** up to ±40 % at 50-400 Hz with any ramp. Renode on Fred's PR #438 F421 image: none up to ±30 % (±50 % hits the trap below). Pessimistic SITL (partial demag model): first desyncs at **±40 %** (duty 12 %↔90 %, ±60 A phase current) at some frequencies; ±30 % clean everywhere. The true boundary is probably at or beyond ±30 %. The sim brackets it; it cannot pin it. | low-medium |
| Hidden firmware trap | If the modulated duty drops **below 150 (7.5 %)**, AM32 switches to the start-up ramp (2 per tick) until it climbs back. At ±50 % this wrecks the waveform and the mean rpm (15k → 8-10k). Keep the trough above 7.5 % duty, or change the condition in PROFILE_MODE. | high (firmware logic) |
| Motor heating | Copper loss ≈ 11 W at hover. 1P pulsing adds ½(ΔI/I₀)²: **≈ ×2 at ±10 %, ×5 at ±20 %, ×10 at ±30 %, ×18 at ±40 %** (22, 56, 116, 200 W). Anything above ±15-20 % continuous is very likely too hot for a 2307. | ratios medium; watts low (∝ 1/R) |

**Recommendation for PROFILE_MODE.**
- Generate the 1P sine inside the 20 kHz loop.
- Bypass the stock ramp limit for the modulation term only. Keep the normal ramp on collective, and keep a
  slew limit of ≥ 64/tick (or none) on the sum.
- Clamp the sum to [≈ 180, 2000] so you never hit the `< 150` start-up-ramp trap.
- Budget amplitude by heat rather than by desync: ±10-15 % continuous, more only in short bursts.
- Expose desync and commutation-stress telemetry: PR #438 EDT2 is a good start; see §10.

## 2. Setup and provenance

**Firmware**
- Fred's fork, worktree `~/vehicles/hakea/am32-sim` (branch `sim/ramp-study`), at **a47e94c** (= upstream AM32
  head). The changes are uncommitted; the full diff is `scripts/sim/am32-sim-full.patch` and the firmware-only
  part is `scripts/sim/ramp-study.patch`:
  - `SIM_CFLAGS` hook in the Makefile and `sitlmakefile.mk`;
  - `SIM_RAMP_HIGH_OVERRIDE` in `Src/main.c`, which forces `max_ramp_high_rpm` after the EEPROM clamp;
  - `SIM_DEAD_TIME` override for the SITL target;
  - SITL additions: a deterministic DShot "wave" generator in simulated time
    (`Mcu/SITL/Src/sitl_input.c`, env `AM32_SITL_WAVE`), a 20 kHz study log (`Mcu/SITL/sim/motor.c`, env
    `AM32_SITL_STUDY_LOG`), and the run ends on wave end or reset.
- The ESC on the bench (logs 31-34) ran the **v2.21 release (6b3ef3d)**. Diff v2.21..a47e94c (44 commits): the
  throttle, ramp and commutation code is identical. The changes are:
  - F421 DShot input-capture prescaler /7 → /6;
  - ADC/temperature code reorganisation;
  - the `low_rpm_throttle_limit` guard (kv<300 → kv≤20, no effect at 1300 kV);
  - SITL/DroneCAN-SITL support.

  None of these changes the control path at these settings.
- Future reference firmware: **head + PR #438 (bdbc75e)**, `firmware/AM32_FOXEER_F421_head-pr438_bdbc75e.elf`
  (sha256 35215759…8688). PR #438 only adds EDT2 status/stress telemetry hooks (`Src/dshot.c`, 10 lines in
  `Src/main.c`). A SITL built from bdbc75e plus the same study patch is in `~/vehicles/hakea/am32-sim-pr438`
  (variants `sitl_pr438_ramp{16,255}.elf`), for byte-identical-source checks; see §7.4.
- SITL ELFs (DEAD_TIME 47 ≈ 294 ns at the SITL's 160 MHz, matching FOXEER_F421's 35 at 120 MHz;
  `scripts/sim/build_variants.sh`):

  | ELF | sha256 |
  |---|---|
  | `am32-sim/variants/sitl_ramp16.elf` (stock) | cae94221…ad82 |
  | `sitl_ramp32.elf` | 82416431…e582 |
  | `sitl_ramp64.elf` | e1164838…1cb9 |
  | `sitl_ramp255.elf` | 01ec40f7…083d |
  | `am32-sim-pr438/variants/sitl_pr438_ramp16.elf` | c43547b5…ceee |
  | `sitl_pr438_ramp255.elf` | ccab18fd…60bc |

**ESC settings**
- Fred's EEPROM `~/Downloads/esc1_config.bin` (sha256 a1ce48e4…f355), padded to 192 bytes; beep volume is set to
  0 in the sim only.
- Relevant settings: max_ramp 160, min duty 4 (=40), comp_pwm 1, variable_pwm 1, pwm_frequency 24, advance 26
  (16/64) with auto_advance 1, startup_power 100, brake_on_zero_throttle 0, current limit off, input DShot.

**Tools**
- ESCSim d53f147 (unmodified; the SITL physics live in the AM32 tree); Renode 1.16.1 via ESCSim.
- Python in `~/venv-ardupilot`. **numba 0.68 was pip-installed there** (llvmlite 0.50; numpy unchanged).

**Key firmware facts confirmed in source**
- EEPROM `max_ramp`/10 can only lower the ramps, so with Fred's 160 a larger `RAMP_SPEED_HIGH_RPM` alone does
  nothing.
- `max_duty_cycle_change` is uint8, so 255 is the real "unlimited".
- The low-rpm ramp (6/tick) applies when e_com_time/3 > 500 µs (below ~5.7k rpm).
- The start-up ramp (2/tick) applies when zero_crosses < 150 **or `last_duty_cycle` < 150**.
- Auto-advance = map(duty, 100..2000 → 13..23)/64 of a commutation interval.
- At zero throttle with comp_pwm=1 the duty ramps to 0 while still commutating. This is dynamic braking: the bench
  coast-downs stop in ~50 ms.

## 3. Bench data used

| log | prop | content | use |
|---|---|---|---|
| 33 | off | bench.lua: steps 1100-1800, 3 coast-downs, 1300±200 sweep, 1500±400 sweep (316.8-504.8 s) | fit (tests 1-3, then all) |
| 34 | on | same, second arm 249.0-446.1 s | fit |
| 32 | on | Lua sweep 1500±400 1→50 Hz, runs 1 and 2 | secondary hold-out only |
| 18, 26, 31 | – | DShot600 / serial telemetry / short | not used (18 was mis-labelled prop-off early on; it is likely prop on) |

**Channels**
- RPM from bidirectional DShot (BNCH/SWP at ~430 Hz).
- FC `BAT.Volt` as an *approximate* bus voltage (100 ms averages; scale not checked against a multimeter).
- Not used: BAT current (phantom pin), ESC volt/current (wrong scaling on this board), IMU (FC not on the motor).
- Extraction: `scripts/sim/bench_extract.py` → `notes/sim/bench/*.csv`.

## 4. Models and physics choices

### 4.1 Averaged model (`scripts/sim/avgmodel.py`), the calibration and "simple physics" model
- **ArduPilot**: DShot = 2·(pwm−1000)+48 (`RCOutput.cpp`).
- **AM32**: duty setpoint = map(DShot, 47..2047 → 40..2000), low-rpm duty clamp map(k_erpm, 6..54 → 400..2000).
  The ramp limiter runs per 20 kHz tick with the start-up / low / high rules above.
- **Motor**: commutation-averaged DC-equivalent BLDC.
  - L dI/dt = g(rpm)·d·V_bus − K_e ω − R I, with complementary PWM, so I can go negative (regenerative braking).
  - J dω/dt = K_t I − k_q ω|ω| − b ω − T₀, with K_e = K_t = 60/(2π·1300).
  - g(rpm) is an effective voltage-gain table at 0/8/14/20/30 k rpm. It absorbs dead time, timing advance and
    variable-PWM effects.
- **Battery**: V_bus = V₀ + V₁t − R_b·I_bus.
- **Zero throttle**: braking as in AM32, with the telemetry dropout below ~2k rpm.
- **Measurement chain**: a pure delay (Lua → loop → DShot → ESC → telemetry → Lua).
- **Prop**: quasi-static aerodynamics (Q = k_q ω²). The no-prop data show the same large-swing behaviour as the
  prop data (§5.3), so prop dynamics/inflow are not the limiting physics below 50 Hz. At 1P the prop torque change
  is tiny compared with the inertial torque (2k_qω₀ ≈ 6e-5 N m s vs J·2πf ≈ 0.01 N m s), so quasi-static is
  adequate for the motor side. It is **not** adequate for thrust/moment at 1P: blade flapping and dynamic inflow
  are outside this study.

### 4.2 Identifiability (important)
- With only RPM and battery voltage, the model is exactly invariant under J→sJ, R→R/s, k_q→s·k_q, R_b→R_b/s,
  T₀,b→s·T₀, s·b.
- RPM predictions do not depend on s. Torque, current and losses do.
- s is fixed by a prior R = 0.16 ± 0.04 Ω: the T-Motor V2307 1950 kV resistance (67 mΩ), scaled by Kv² to
  1300 kV (≈ 0.15 Ω), plus FETs and leads.
- Inductance is not identified at all by data below 50 Hz (ωL ≪ R). It is a physics prior: 18 µH per phase
  (36 µH line-line), from 2306-class motors scaled by Kv².
- **A milliohm and LCR measurement of the motor would remove both assumptions** (§11).

### 4.3 SITL (switched model, `Mcu/SITL/sim/motor.c`)
- The real AM32 firmware runs against a trapezoidal BEMF motor with PWM-level bridge switching, dead time, body
  diodes, comparator RC/noise and the battery.
- Parameters are taken from the averaged fit, except:
  - kv 1380, because the SITL's effective speed per duty is lower than the hardware's (tool-issues #4);
  - L_phase 18 µH (prior);
  - static friction 0.002 N m, for start-up robustness;
  - `commutation_transfer` 1.0 (nominal) or 0.8 (pessimistic demag), see tool-issues #3.
- Final: `notes/sim/sitl_calib/sitl_base_final.json`.
- **No physics code was changed in motor.c.** The prop load (k_q ω|ω|) and inertia already existed, and the
  calibration did not need new physics: the one candidate that helped (thermal R(T)) is unphysical at the fitted
  scale (§5.3). The only motor.c change is the study logger.

## 5. Calibration and verification

### 5.1 Fit strategy
1. **v2:** fit tests 1-3 (steps, coast-downs, small sweep) of 33 and 34 jointly; motor parameters shared; prop adds
   J_p and k_q. Hold out test 4 (big sweep) of both, plus log 32.
2. **v5all:** because v2 fails the big-sweep hold-out (§5.3), refit on all four tests and keep log 32 (the same
   sweep, another day) as the hold-out. This is the model used for predictions.

`scripts/sim/calibrate.py` (`HAKEA_FIT_TESTS=1,2,3,4` for v5all); outputs in `notes/sim/calib/`.

### 5.2 Results (rpm errors)

| data | v2 (fit 1-3) | v5all (fit 1-4) |
|---|---|---|
| log 33 test 1 steps | 380 rms | 331 |
| 33 test 2 coast-downs | 180 | 372 |
| 33 test 3 small sweep | 266 | 466 |
| 33 test 4 big sweep | **1280 (hold-out)** | 799 |
| 34 test 1 steps | 225 | 239 |
| 34 test 2 coast-downs | 195 | 183 |
| 34 test 3 small sweep | 244 | 265 |
| 34 test 4 big sweep | **872 (hold-out)** | 519 |
| log 32 run 2 (hold-out) | 838 | **483** |
| log 32 run 1 (hold-out) | 758 | **443** |
| battery voltage (100 ms avg) | 0.035 / 0.10 V | 0.04 / 0.11 V |

Steady rpm (last second of each 3 s step), bench → v5all model:
- prop off: 1100: 3857→4013 (+4 %), 1300: 10251→10647 (+4 %), 1500: 17050→17470 (+2.5 %), 1800: 26571→26339 (−1 %).
- prop on: 1100: 3757→3859 (+3 %), 1300: 9658→9817 (+2 %), 1500: 15649→15779 (+1 %), 1800: 22484→22846 (+1.6 %).

Parameters (v5all, prop on):

| parameter | value |
|---|---|
| g | 1.09/1.03/1.08/1.05/1.00 |
| R | 0.158 Ω (prior-anchored) |
| J_motor | 4.2e-6 kg m² |
| J_prop | 2.5e-6 kg m² |
| k_q | 1.84e-8 N m s² |
| b | 5.8e-6 N m s |
| R_b | 0.037 Ω |
| measurement delay | **4.98 ms** |

The equivalent mechanical time constant at 1500 µs is τ = J/(K_e²/R + 2k_qω) ≈ 6.7e-6/(3.4e-4+6.0e-5) ≈ 17 ms
plus the 5 ms delay. The earlier single-pole fit (23.5 ms + 6.5 ms) lumped part of the delay into τ.

Figures: `figs/bode_p34_v5all.png`, `figs/bode_np33_v5all.png`, `figs/timeseries_p34_v5all.png`,
`figs/bode_p34_v2.png` (shows the hold-out failure).

SITL replay of the same segments with the final SITL model (`scripts/sim/sitl_calib.py eval --base …final.json`,
figure `figs/sitl_vs_bench_bode.png`):
- steps within −4…+6 %;
- small-sweep gain 17 % low (log error −0.19);
- big-sweep gain 5 % high;
- **0 desyncs in all bench replays** (bench: 0).

### 5.3 The unresolved "big-swing" effect
- On the bench, 1500 ± 400 µs responds 20-40 % less per µs at 8-45 Hz than 1300 ± 200 µs, both prop on and
  **prop off**.
- The physics in the averaged model reproduce only 5-9 % of this (from the different base throttle).
- Hypotheses tested (`scripts/sim/experiments/compression_hypotheses.py`):

| hypothesis | result |
|---|---|
| operating point (base 1300 vs 1500) | explains 5-9 %; not enough |
| ramp-limit regime switch at troughs | troughs are > 10k rpm at ≥ 8 Hz; not reached |
| dynamic inflow / prop aerodynamics | effect is present **without the prop**; ruled out as main cause |
| unlogged fast battery sag (bus impedance) | needs ~5× the DC battery resistance at 10-40 Hz; a LiPo's impedance is *lower* there; not plausible |
| torque-constant drop at high current (Isat ≈ 50 A) | compresses gain, but lowers the mean rpm, which the bench doesn't; partial at best |
| winding heating, R(T) | reproduces the pattern well (ratios 0.82/0.74/0.70/0.66 vs bench 0.79/0.72/0.72/0.6), but needs a ~100 K+ winding temperature rise at the model's current scale. It is rejected by the fit on tests 1-3. Plausible only if the real currents are larger, i.e. R lower than the prior. **Open.** |

**Impact.** The model is accurate for small and medium swings. For large, fast swings it over-predicts rpm and
torque response by up to ~1.6× at 40 Hz. If the cause also acts at 1P, the torque numbers in §7 are optimistic for
amplitudes ≥ ±20 %. The blind test (§6) includes ±250 µs sines at 5-50 Hz designed to discriminate. The
recommended measurements in §11 (motor temperature, current) settle it.

## 6. Blind validation (pre-registered)
- Predictions for Fred's "bench2" test (prop on: steps 1150/1450/1750; coast-down from 1650; 1600 ± 250 µs at
  5/15/35/50 Hz; 1400 ± 150 µs at 25 Hz) were written and hashed **before any log above 00000034 existed or was
  read**.
- Files: `notes/sim/blind/predictions.md`, `predictions.csv`, `HASHES.txt` (sha256 291624e6… and e940fc07…),
  model snapshot in `model_snapshot/`.
- Model: v5all, battery 24.8 V − 0.004 V/s ± 0.6 V.
- Highlights:
  - steady 5,360 / 14,470 / 21,770 rpm;
  - braked stop in < 0.05 s after the cut;
  - fundamental rpm amplitude 5,480 / 3,330 / 1,620 / 1,120 rpm at 5/15/35/50 Hz, lagging the command by
    38/91/158/201° (including the 4.98 ms chain delay);
  - 0 desyncs.
- Amplitude ranges at ≥ 15 Hz are skewed low because of §5.3.
- Check with `scripts/sim/compare_blind.py <new log>`.

## 7. 1P extrapolation (50-400 Hz)

Stimulus:
- base DShot 1048 (ArduPilot 1500 µs ≈ 15k rpm prop-on in the SITL), battery 24.3 V;
- sine ±A % of full throttle (A·20 DShot units) for 1 s after settling;
- 10 kHz unidirectional DShot, injected by the SITL in simulated time;
- metrics over the last 0.8 s.

Scripts: `scripts/sim/grid.py`, `grid_tables.py`, `extrapolate.py`. Raw CSVs in `notes/sim/grid/`.

### 7.1 Applied duty, the ramp limiter (firmware arithmetic, SITL nominal model; `grid_ramp_ct1.0.csv`)

Ratio of the applied duty fundamental to the commanded one (1 = passes unclipped):

| ramp | f Hz | ±5 % | ±10 % | ±20 % | ±30 % | ±40 % |
|---|---|---|---|---|---|---|
| 16 (stock) | 100 | 1 | 1 | 1 | 0.97 | 0.80 |
| 16 | 150 | 1 | 1 | 0.97 | 0.72 | 0.54 |
| 16 | 200 | 1 | 1 | 0.80 | 0.54 | 0.41 |
| 16 | 250 | 1 | 1 | 0.65 | 0.43 | 0.32 |
| 16 | 300 | 1 | 0.97 | 0.54 | 0.36 | 0.27 |
| 16 | 400 | 0.99 | 0.80 | 0.40 | 0.27 | 0.20 |
| 32 | 250 | 1 | 1 | 1 | 0.85 | 0.65 |
| 32 | 300 | 1 | 1 | 0.97 | 0.72 | 0.54 |
| 64 | 250 | 1 | 1 | 1 | 1 | 1 |
| 64 | 300 | 1 | 1 | 1 | 1 | 0.97 |
| 64 | 400 | 1 | 1 | 1 | 0.97 | 0.80 |
| 255 | 50-400 | 1 | 1 | 1 | 1 | 1 |

Analytic check: a slew limit S = 16 × 20 kHz = 320k duty/s passes a sine of duty amplitude a while 2πfa ≤ S, i.e.
a ≤ 204 duty (10 %) at 250 Hz and 170 at 300 Hz. Fully clipped, the output is a triangle with fundamental
(8/π²)·S/(4f) = 259 / 216 / 162 duty at 250 / 300 / 400 Hz. Both match the table.

### 7.2 Torque and rpm at 1P

Fundamental amplitudes, ramp 255 (= ramp 64 up to ±30 %); stock ramp 16 in brackets:

| f | ±10 % torque N m | ±20 % | ±30 % | ±40 % | rpm amp (±10 / ±30 %) |
|---|---|---|---|---|---|
| 100 Hz | 0.083 (0.083) | 0.167 (0.167) | 0.258 (0.250) | 0.348 (0.278) | 188 / 584 |
| 200 Hz | 0.081 (0.081) | 0.164 (0.131) | 0.252 (0.132) | 0.341 (0.132) | 92 / 286 |
| 250 Hz | 0.080 (0.080) | 0.161 (0.103) | 0.248 (0.103) | 0.335 (0.103) | 73 / 225 |
| 300 Hz | 0.079 (0.077) | 0.158 (0.084) | 0.243 (0.084) | 0.328 (0.084) | 60 / 184 |
| 400 Hz | 0.076 (0.061) | 0.151 (0.061) | 0.232 (0.061) | 0.313 (0.061) | 43 / 132 |

Mean electromagnetic torque at 1500 µs ≈ 0.056 N m, so a ±10 % sine already reverses the torque every cycle
(regenerative braking through comp-PWM).

**First-order analytic extrapolation** (`extrapolate.py`, linearised averaged model, same parameters, L = 36 µH
line-line), for comparison:
- torque amplitude 0.107 / 0.105 N m per ±10 % at 250 / 300 Hz;
- rpm amplitude 97 / 80 rpm per ±10 %.

The averaged model is ~25-30 % above the SITL. Most of the difference is the SITL's kv 1380 (lower K_t) and its
explicit commutation/inductance effects. Treat **0.08-0.11 N m per ±10 % throttle at 250-300 Hz** as the
model spread. The physical scale uncertainty from R is on top, and goes as 1/|R + jωL|.
- L sensitivity (averaged model, 300 Hz, ±30 %): 20 µH → 0.324 N m, 36 µH → 0.309, 70 µH → 0.264.

See `figs/torque_heating.png`.

### 7.3 Desync boundary

| SITL model | result |
|---|---|
| nominal (`commutation_transfer` 1) | **0 desyncs and no commutation slip** (ground truth: rotor electrical angle vs firmware step) in all 172 runs, ramps 16/32/64/255 × 50-400 Hz × ±5-50 % |
| pessimistic demag (0.8) | 0 desyncs up to ±30 % everywhere. At ±40 %: ramp 64 / 100 Hz 18 desyncs (rpm collapsed); ramp 255 / 200 Hz 2, 250 Hz 5 (mean rpm dropped to 6-10k); others clean. Stock ramp 16: never, because it cannot deliver the swing. |

- Interpretation: under pessimistic demag the boundary sits around ±35-40 % (duty 12 %↔90 %, phase current
  ±55-60 A at 15k rpm).
- The pessimistic model is known to be too pessimistic: it desyncs bench replays that ran clean on hardware. The
  nominal one is too optimistic (no demag at all).
- So the simulation says: **with the ramp opened up, desync is unlikely below ±30 %. ±40 % is marginal. Heating
  limits you before that.**
- The ±50 % column is dominated by the duty < 150 start-up-ramp trap (§1), not by desync.

### 7.4 PR #438 SITL vs a47e94c SITL
See §9.4 (the same grid points rerun with `sitl_pr438_*.elf`).

## 8. Renode cross-check

### 8.1 Round 1 (old uncalibrated model, a47e94c builds)
Notes in `notes/sim/renode/renode-crosscheck.md`. FOXEER_F421 and VIMDRONES_L431_CAN, 250 Hz, base DShot 1047,
DShot300 at 10 kHz virtual:
- 0 desyncs in any sine segment, either MCU, any ramp.
- Stock ramp: ±10 % passes (391 of 400 duty p-p); ±30 % clipped to a triangle (518 of 1176).
- Ramp 64 = 255: full swing.
- Ramp-limited runs show a mean-duty offset whose sign depends on the emulated ISR timing (20 µs vs 2 µs
  physics batch).
- The L431 CAN build hangs at start-up with Fred's EEPROM CAN node id 253 (tool-issues #11).
- DroneCAN RawCommand was not done: ESCSim's CAN bridge is wall-clock (tool-issues #10).

### 8.2 Round 2 (calibrated model, Fred's PR #438 ELF)
Notes in `notes/sim/renode/v2/renode-v2.md`; data in `summary_v2.csv` and `raw/`.

Setup:
- FOXEER_F421 at **bdbc75e**: stock = Fred's flashed ELF 35215759…8688; ramp64/255 rebuilt from the same commit.
  The stock rebuild's loadable image is byte-identical to Fred's.
- Calibrated model, L = 18 µH, Fred's EEPROM.
- 250/300 Hz, ±100…±1000 DShot (±5…50 %).
- 2 µs physics batch, plus 20 µs at 300 Hz.

Results:
- **0 desyncs and no WARNING/ALERT/ERROR status in all 45 sine runs**, with stock, ramp 64 and ramp 255. This
  agrees with the SITL nominal model.
- Ramp limiter:
  - stock applied-duty fundamental saturates at 259 (250 Hz) / 216 (300 Hz) duty, exactly the analytic triangle
    values;
  - ramp 64/255 track ±600 (586-588 of 600).
- The duty < 150 start-up-ramp trap at ±1000 reproduces: rpm 15k → 8.4-9.4k.
- RPM amplitude at 250 Hz, Renode vs SITL:

  | amplitude | Renode | SITL |
  |---|---|---|
  | ±10 % | 67 rpm | 73 |
  | ±20 % | 123 | 146 |
  | ±30 % | 206 | 225 |

- Copper loss: 11.6 W at settle; ±20 %: 45-50 W; ±30 %: 99-109 W. The SITL gave 56 and 116 W. Renode is 10-15 %
  lower, within the model spread.
- **PR #438 commutation-stress metric** (computed by replicating `dshot_take_commutation_stress()` from emulated
  RAM every 50 ms; the firmware's own EDT path could not run, see below):
  - With the 2 µs batch it reads **255 in every window, even at steady throttle**. About 7 % of zero crosses come
    at half a step and then 1.5 steps, consistent with a false zero cross right after commutation (demag or
    comparator chatter).
  - With the 20 µs batch it sits at 53, the quantisation floor, and **jumps to 255 once the applied duty swing is
    ≥ ±390 (±20 %)**.
  - Because the metric is a *peak* over the window, one displaced zero cross saturates it. On hardware it may
    likewise read 255 well before any desync. It is a sensitive early-warning flag, not a graded margin
    (tool-issues #18).
- Blocker: scripted bidirectional DShot in Renode stops decoding after exactly 1 s (tool-issues #17). The EDT
  path was therefore emulated from RAM rather than run.

**SITL vs Renode verdict.** The two independent emulations (G431-style native SITL, and the F421 instruction set
in Renode with the same physics library) agree on everything that matters here: the ramp limits, torque and rpm
amplitudes within ~10-15 %, losses, the start-up-ramp trap, and no desyncs up to ±30 % with the nominal physics.
They share the same motor/demag physics, so they are **not** independent on the desync question.

## 9. ESC settings, heating and checks

### 9.1 ESC settings search
Setup: ramp 255, 250 and 300 Hz, ±10-40 %. Each setting was run with the nominal (ct 1.0) and pessimistic (ct 0.8)
demag model (`scripts/sim/run_settings_grid.sh`, `notes/sim/grid/settings_*.csv`).

Torque amplitudes are for ±10/20/30/40 %. Desync counts are shown only where non-zero (all were at ±40 %).

| setting (others = Fred's EEPROM) | torque N m, nominal, 250 Hz | vs Fred's | pessimistic model: desyncs at ±40 % (250/300 Hz) | hover copper W | ±30 % copper W |
|---|---|---|---|---|---|
| **Fred's: 24 kHz, comp_pwm 1, var_pwm 1, auto-advance** | 0.080/0.161/0.248/0.335 | – | 0 / 0 | 11.3 | 116 |
| PWM 16 kHz | 0.083/0.166/0.252/0.339 | +2-4 % | **8 / 1** | 12.2 | 120 |
| PWM 48 kHz | 0.074/0.148/0.234/0.321 | −4…−8 % | 0 / 0 | 10.0 | 104 |
| comp_pwm 0 | 0.059/0.088/0.112/0.135 | **−26…−60 %** | 0 / 0 | 11.2 | 46 |
| variable_pwm 0 | 0.081/0.162/0.249/0.336 | ±0 % | 0 / 0 | 11.4 | 117 |
| fixed advance 8/64 (level 18), auto off | 0.080/0.162/0.250/0.338 | ±0 % | 0 / 0 | 10.4 | 112 |
| fixed advance 16/64 (level 26), auto off | 0.079/0.159/0.245/0.332 | −1 % | 0 / **3** | 11.1 | 112 |
| fixed advance 24/64 (level 34), auto off | 0.077/0.154/0.237/0.321 | −4 % | **7 / 8** | 12.7 | 113 |
| fixed advance 32/64 (level 42), auto off | 0.073/0.145/0.224/0.303 | −9 % | **7 / 4** | 15.9 | 116 |

Conclusions:
- **comp_pwm must stay on.** Without active braking the 1P torque swing collapses to about half, as one would
  expect: the negative half-cycle needs regenerative current.
- **Auto-advance (or a low fixed advance) is best.** High fixed advance costs torque and is the first to desync
  in the pessimistic model.
- **PWM frequency is a mild trade-off.** 48 kHz loses ~5 % torque and gains robustness; 16 kHz gains ~3 % and
  is the first to desync. 24 kHz (Fred's) is fine.
- variable_pwm makes no difference.
- **Ramp: 64 is enough up to ±30-40 % at 250-300 Hz. 255 removes the limit entirely.** For PROFILE_MODE the
  question is moot if the modulation bypasses the limiter.

Confidence:
- The setting-to-setting torque differences (a few %) are within the model's systematic error, but the direction
  is physical.
- The desync differences depend entirely on the pessimistic demag knob. Treat them as "which settings are less
  robust", not as absolute boundaries.

### 9.2 Motor heating (model-derived; scales ~1/R; R at 25 °C)

| condition (ramp ≥ 64, 250-300 Hz) | phase I rms | copper loss | × hover | analytic 1+½(ΔI/I₀)² |
|---|---|---|---|---|
| hover 1500 µs, no pulsing | 7.0 A | 11 W | 1 | 1 |
| ±10 % | 9.8-9.9 A | 22-23 W | ×2.0 | ×2.0 |
| ±20 % | 15.3-15.6 A | 54-56 W | ×4.8-5.0 | ×5.1 |
| ±30 % | 22.0-22.4 A | 112-116 W | ×10 | ×10.6 |
| ±40 % | 29.0-29.6 A | 194-202 W | ×17-18 | ×18.9 |
| ±50 % (ramp 255, 250 Hz) | 38 A | ~335 W | ×30 | – |
| stock ramp 16, any amplitude ≥ ±20 % at 250 Hz | 11.4 A | 30 W | ×2.7 | (clipped) |

- At ~100 °C winding temperature, copper R is ×1.3, so the losses are roughly ×1.3 higher at a given current.
  Current falls slightly, so the true factor is a bit less.
- Bridge (FET) losses in the SITL are 0.8-5.7 W.
- The averaged model gives ×2.6 / ×15 at ±10 / ±30 %. It has more current per duty (lower |Z|, higher K_t);
  see §7.2.
- Iron loss: not modelled separately. The fitted viscous term b = 5.8e-6 N m s gives ≈ 15 W at 1600 rad/s. That
  includes bearings, windage and iron loss at no load, and changes little with 1P pulsing because the speed hardly
  changes. Additional eddy loss from the 1P current ripple is small at 250-300 Hz compared with copper loss.

**Rough lumped thermal estimate (LOW confidence, labelled as such).**

Assumptions:
- 2307 motor, ~35 g with ~8-10 g of copper;
- C_th ≈ 15-20 J/K (stator plus windings);
- winding-to-ambient R_th ≈ 2-4 K/W with prop downwash cooling an open-frame outrunner (no data; typical small
  FPV motors are 2-5 K/W).

| condition | winding rise above ambient | how fast |
|---|---|---|
| hover (11 W) | +20-45 K | – |
| ±10 % (22 W) | +45-90 K | time constant ~40-80 s |
| ±20 % (55 W) | +110-220 K | not sustainable: well past the ~150 °C enamel limit within ~1 min |
| ±30 % (115 W) | – | 6-8 K/s initially; a few tens of seconds at most |

So the continuous 1P budget is around ±10-15 % throttle-equivalent, and larger amplitudes only as short
transients. A thermocouple measurement would replace all of this.

### 9.3 Bidirectional DShot sensitivity
Not run on the grid: bidir in the SITL causes false desyncs at low rpm (tool-issues #1). At 15k rpm it was clean
in the bench replays (p34 tests 3/4, 0-2 desyncs, start-up only).

### 9.4 PR #438 firmware in the SITL
The same grid points (ramp 16/255, 250/300 Hz, ±0/10/30 %) with `sitl_pr438_ramp*.elf` (bdbc75e + study patch)
match the a47e94c builds within 0.1 % in rpm and torque, with 0 desyncs in both
(`notes/sim/grid/grid_pr438_check.csv`). The PR does not change the control behaviour, as expected from its diff.

## 10. Proposed protocol: blind desync prediction on the real drone (≈1 page)

**Goal.** Before flying PROFILE_MODE, predict *where* (frequency × amplitude × ramp/slew setting × collective)
sensorless commutation fails. Hash the predictions, then test on the bench and in flight.

1. **Freeze the image.** Fred builds PROFILE_MODE firmware. The *same commit* is built for FOXEER_F421 (flashed) and
   as SITL. Record the ELF sha256 of both, and keep the Renode run on the exact flashed ELF.
2. **Predict.**
   - Renode: F421 instruction-level, use the 2 µs physics batch.
   - SITL: nominal and pessimistic demag.
   - Both with the calibrated motor model, after the R/L measurements in §11.
   - Grid: f ∈ {200, 240, 270, 300, 350}, A ∈ {5…50 %}, 2-3 collectives, 2-3 slew settings.
   - Output per cell: P(desync) as a band (nominal = lower bound, pessimistic = upper), torque amplitude,
     copper loss, EDT2 stress level.
   - Write `predictions.csv` plus `HASHES.txt` as for bench2.
3. **Measure on hardware.** Bench first (clamped motor, prop on), then tethered hover.

   Signals that reveal desyncs:
   - **EDT2 status WARNING** (PR #438 sets it where `desync_happened++` runs; ArduPilot logs it as EDT2 when EDT
     is enabled). This is the primary signal.
   - **EDT2 commutation-stress** (zero-cross interval jitter). It is the leading indicator. Caveat from Renode
     (§8.2): it is a per-window peak and can pin at 255 from isolated false zero crosses, so treat it as an
     alarm, not a margin.
   - **Bidir-DShot RPM**: dropouts (0 or invalid frames), sudden eRPM jumps > 30 % in one frame, telemetry CRC
     errors.
   - Current spikes, if a current sensor is added.
   - Audible and visual: stutter, and the prop slowing when the throttle says it shouldn't.

   Firmware hooks Fred should add in PROFILE_MODE:
   - a **monotonic 16-bit desync counter** (the F421 `desync_happened` is uint8 and wraps), exported in an EDT
     frame or with the status bits;
   - a **"profile active + current amplitude/phase" status**, so the logs show what the ESC was actually doing;
   - optionally a **min/max zero-cross interval per 100 ms** (a cheap stress proxy);
   - optionally the **duty actually applied** at 1P: the fundamental amplitude computed in-ESC, logged at low
     rate;
   - on CAN, `esc.Status.error_count` should carry the desync counter.
4. **Compare.** Same analysis code on sim and hardware (as `compare_blind.py`). Report hit/miss per cell and the
   shift of the boundary.

**Honest expectations.**
- Ripple, torque and loss predictions should be right to ±20-30 % once R and L are measured. Today they are right
  to ±30-50 % (§4.2), and possibly optimistic at large amplitude (§5.3).
- The ramp/slew-limit behaviour is exact.
- **The desync boundary is the weak part.** The SITL brackets it between "never" and "±35-40 %" depending on a
  demag modelling knob (tool-issues #3). Renode adds real F421 instruction timing but uses the same physics
  library and a 2-20 µs coupling granularity. Expect the sim to get the *trend* right: worse at large amplitude,
  low collective (a deeper trough), and high frequency where current swings are largest.
- The absolute amplitude at which desyncs start may be off by ±10 percentage points. One bench run with the EDT2
  stress signal will calibrate the demag knob, and after that the predictions become useful.

## 11. Measurements that would most reduce model uncertainty (ranked)

1. **Motor phase-to-phase resistance** (4-wire: bench supply at ~2 A plus a voltmeter on the motor leads, or a
   milliohm meter). It fixes the absolute scale of torque, current and heat (§4.2); everything currently rests on a
   ±25 % prior. Cost: minutes.
2. **Phase inductance** (LCR meter at 1-10 kHz, phase-phase, at a few rotor positions). It dominates 1P current and
   torque (ωL ≈ R at 300 Hz) and is entirely assumed today.
3. **Bus/phase current at high rate** (a Hall current sensor on the battery lead logged at ≥ 1 kHz via the thrust
   stand ESP32, or an ESC with a working shunt). It breaks the scale degeneracy directly, shows the regen current
   at 1P, and tests the Isat hypothesis.
4. **Motor winding temperature during the big sweep** (thermocouple on the stator, or IR on the windings before and
   after). It decides the heating explanation of §5.3, and Fred needs the thermal data anyway.
5. **Same-base multi-amplitude fixed-frequency sines** (e.g. 1500 ± 100/200/300/400 µs at 10, 20, 40 Hz, short
   segments with cooling gaps, in random order). This separates amplitude from operating point and from heating
   (time order).
6. **Thrust/torque stand (6-DOF)**: k_q directly (removes the J_p/k_q scale question), plus the prop's dynamic
   thrust response at 1P (blade flapping / inflow), which this study cannot model.
7. **A scope capture of one phase voltage at 15k rpm with ±30 % 1P on the bench**: demag duration, to calibrate
   the SITL demag knob (tool-issues #3).
8. **Battery voltage at the ESC with a multimeter cross-check**: removes the few-% BAT scale uncertainty.

## 11b. Follow-up: 0-1000 Hz in-ESC sine predictions (PROFILE_MODE v1)
See `notes/sim/predict-1khz/predictions.md`, which is hashed. In short:
- With Fred's PROFILE v1 (sine after the ramp limiter), the torque swing is ≈ 0.08 N m per ±10 % from 15 to
  300 Hz, falling to 0.068 at 600 Hz and 0.054 at 1 kHz (L/R corner ~700 Hz). Torque lags the duty by 18° at
  250 Hz and 60° at 1 kHz.
- SITL and Renode agree within ~15 %.
- No desyncs up to ±30 % at any frequency. The pessimistic demag model desyncs at ±40 % between 50 and 400 Hz.
- The AM32 loop runs at 19,608 Hz, so v1's frequencies are 2 % low on hardware.

## 12. Not done / limitations
- **Iron loss** is not separated from bearing and windage loss (one viscous term).
- **No thermal validation.** The thermal numbers are a rough lumped estimate.
- **Prop aerodynamics at 1P** (thrust/moment response, flapping, dynamic inflow) are not modelled. The study covers
  the motor/ESC side: torque, rpm, current, desync.
- **Big-swing effect (§5.3) unresolved.** Bounded, and the leading candidate is identified (winding heating / R
  scale).
- **Desync boundary** is bracketed, not predicted (§7.3).
- **DroneCAN RawCommand under Renode** was not done (wall-clock CAN bridge). The L431 CAN build was run with
  DShot only, in round 1.
- **Bidirectional DShot** was not used in the SITL grid (SITL artefact). Under Renode, scripted bidir stops
  decoding after 1 s, so the firmware's own EDT2 path was not executed; the stress metric was reproduced from RAM.
- **PROFILE_MODE** was not written. Fred is writing it; the recommendations in §1 are design input only.

## 13. Files
- Scripts, `~/vehicles/hakea/scripts/sim/`:
  - bench_extract, avgmodel, calibrate;
  - sitl_run (library + smoke test), sitl_replay, sitl_calib, build_variants.sh;
  - grid, grid_tables, run_settings_grid.sh, extrapolate, experiments/compression_hypotheses;
  - predict_blind, write_blind_predictions, compare_blind;
  - patches `ramp-study.patch`, `am32-sim-full.patch`;
  - models/*.json;
  - renode/ (Renode sub-agent scripts).
- Motor-ID workflow, `~/vehicles/hakea/scripts/motorid/`: motorid.py, motorid_model.py, motorid.lua, motorid.md,
  README.md, tests/. Results in `notes/motorid/`.
- Results, `~/vehicles/hakea/notes/sim/`:
  - bench/ (CSV extracts), calib/ (fits, plots), sitl_calib/, grid/ (CSV + meta);
  - blind/ (hashed predictions), renode/ (Renode notes/data), figs/;
  - progress.md (checkpoints), tool-issues.md, this report.
- Worktrees (git-ignored, uncommitted): `am32-sim` (branch sim/ramp-study), `am32-sim-pr438` (detached at bdbc75e),
  `am32-renode` (branch sim/renode-check, Renode agent), plus the Renode agent's PR #438 worktree.
