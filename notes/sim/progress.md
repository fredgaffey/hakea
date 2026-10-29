# Sim study progress log (checkpoint notes)

Working notes, updated after each stage so an interruption loses nothing.
The final write-up is am32-ramp-study.md.

## Stage 0: setup (done)
- Worktree ~/vehicles/hakea/am32-sim, branch sim/ramp-study from fork main
  a47e94c (uncommitted changes, see scripts/sim/ramp-study.patch + SITL
  additions listed below).
- v2.21 (6b3ef3d, firmware on the ESC for logs 31-34) vs a47e94c: the
  throttle/ramp/commutation code is the same. Differences: F421 DShot
  input-capture prescaler (/7 -> /6), ADC/temperature code reorganised,
  low_rpm_throttle_limit guard (kv<300 -> kv<=20; no effect at 1300 KV),
  SITL support. None affects the control path at these settings.
- Firmware patch: SIM_CFLAGS hook; SIM_RAMP_HIGH_OVERRIDE (forces
  max_ramp_high_rpm after the eeprom clamp); SIM_DEAD_TIME (SITL target).
- SITL additions (Mcu/SITL): AM32_SITL_WAVE deterministic DShot generator in
  simulated time (Src/sitl_input.c), AM32_SITL_STUDY_LOG 20 kHz CSV log
  (sim/motor.c), run ends on wave end or any reset.
- Variants (scripts/sim/build_variants.sh): am32-sim/variants/sitl_ramp{16,32,64,255}.elf,
  DEAD_TIME 47 (294 ns at 160 MHz = F421's 35 at 120 MHz).

## Key firmware facts
- EEPROM max_ramp 160 -> /10 = 16 clamps max_ramp_high_rpm: raising
  RAMP_SPEED_HIGH_RPM alone does nothing on Fred's ESC. max_duty_cycle_change
  is uint8 -> 255 is the real maximum ("unlimited").
- Low-rpm ramp (6/tick) applies when average_interval = e_com_time/3 > 500 us,
  i.e. below ~5.7k rpm. Low-rpm duty clamp: map(k_erpm, 6, 54, 400, 2000).
- auto_advance: advance = map(duty, 100, 2000, 13, 23)/64 of a commutation interval.
- Zero throttle with comp_pwm=1, brake_on_zero_throttle=0: duty ramps to 0 while
  still commutating = dynamic short-circuit braking (bench coast-downs stop in ~50 ms).
- FOXEER_F421 (no DroneCAN): desync_happened is uint8 (wraps at 256).

## Stage 1: calibration (in progress)
- Averaged model fit v2 (tests 1-3 of logs 33/34, R prior 0.16+-0.04 ohm):
  steps within ~3 %, fit rms 180-380 rpm; holdout (test 4 big sweep) rms
  870-1280 rpm: the bench responds ~20-35 % less per us at 8-45 Hz on the
  big sweep than the model.
- Hypotheses checked with the averaged model (experiments/compression_hypotheses.py):
  base-throttle/operating point explains only 5-9 %; Isat (current-dependent Kt)
  compresses but drops the mean rpm, which the bench does not; 5x battery
  resistance reproduces the ratio (~0.79) but contradicts the measured DC sag;
  low-rpm ramp regime not reached at 10-40 Hz (troughs > 10k rpm).
  Thermal R(T) fit running.
- SITL (kv 1380, L 20-50 uH, commutation_transfer 1): steps within a few %,
  small sweep gain -5..-10 %, big sweep holdout +20..+50 % at 12-40 Hz.
  commutation_transfer < 1 or low L -> many desyncs never seen on the bench.
  Bidirectional DShot in the SITL desyncs below ~3k rpm (SITL artefact);
  SITL runs use unidirectional DShot.

## Renode cross-check v1 (sub-agent, OLD uncalibrated model)
See notes/sim/renode/renode-crosscheck.md. 250 Hz, base DShot 1047: 0 desyncs;
stock ramp passes +-10 %, clips +-30 % to a triangle (518 of 1176 duty p-p);
ramp 64 = 255 full swing. To redo with the calibrated model + PR #438 ELF.

## Stage 1b: final calibration choice
- params_v5all.json: averaged model fitted on ALL bench.lua tests (1-4) of logs
  33/34; secondary holdout log 32 runs 1/2 (1500+-400 sweep): 440-480 rpm rms.
  Still over-predicts the high-frequency response of the big sweep (13.6 Hz
  +19 %, 37 Hz +59 %) while slightly under-predicting the small one.
- Thermal R(T) and Isat do not fit tests 1-3 better (thermal switched itself
  off: Rth -> 0); a plausible lumped thermal model (Cth 15 J/K, Rth 4 K/W)
  reproduces the test4/test3 ratio pattern but implies >200 C winding at the
  model's current scale -> not credible unless the real R is much lower.
- The compression also exists WITHOUT the prop (log 33 test 4 vs test 3), so
  prop aerodynamics/dynamic inflow cannot be the main cause.
- SITL final base: notes/sim/sitl_calib/sitl_base_final.json (v5all J/kq/R,
  kv 1380, L_phase 35 uH, T0 0.002, commutation_transfer 1). SITL vs bench:
  steps within ~3-9 % (worst at 1100 us), small sweep gain -19 % (log),
  big sweep +2..+6 %.

## Stage 2: blind predictions (DONE, hashed 2026-10-26T15:11+11:00)
notes/sim/blind/{predictions.md, predictions.csv, HASHES.txt, model_snapshot/}.
Compare later with: scripts/sim/compare_blind.py LOG.BIN

## Stage 3: motorid tooling (DONE by sub-agent)
scripts/motorid/{motorid.py, motorid_model.py, motorid.lua, motorid.md, README.md, tests/}
results notes/motorid/. Side effect reported by the agent: a stray import of
/tmp/coverage.py rewrote ~/_hbc2js/hbc2js-local-corpus/ceiling-coverage.json
(14:31) - Fred should check that file.

## Stage 4: SITL ramp grid (running)
scripts/sim/grid.py -> notes/sim/grid/grid_ramp_stock_settings.csv

## Stage 4 results (DONE)
- L_phase changed to 18 uH prior (old L35 grid kept in notes/sim/grid/old_L35/).
- grid_ramp_ct1.0.csv: no desync/slip anywhere; ramp limiter table; torque 0.08 N m per
  +-10 % at 250-300 Hz; duty<150 start-up-ramp trap at +-50 %.
- grid_ramp_ct0.8.csv: desyncs only at +-40 % (some f), +-30 % clean.
- settings_*.csv: comp_pwm must stay 1; auto/low advance best; PWM 48 kHz most robust.
- grid_pr438_check.csv: PR #438 SITL == a47e94c SITL within 0.1 %.
- extrapolate_avgmodel*.csv: analytic/averaged extrapolation, L sensitivity.
- Report draft written: notes/sim/am32-ramp-study.md (Renode v2 section pending).

## Stage 5-6 (DONE)
- Renode v2 (sub-agent): notes/sim/renode/v2/renode-v2.md; agrees with SITL.
- Report complete: notes/sim/am32-ramp-study.md; tool issues: notes/sim/tool-issues.md (18 items).
- All processes stopped.

## Stage 7: 0-1000 Hz PROFILE_MODE predictions (DONE, hashed 2026-10-26T21:33+11:00)
notes/sim/predict-1khz/ (predictions.md/.csv, predictions_programme.csv, renode_programme_v1.csv,
5 PNGs, raw/, HASHES.txt; read-only). SITL from am32-sim-profile (bdbc75e + Fred's hakea_profile.c +
study patch), Renode on Fred's exact PROFILE ELF. New tool issues #19 (Renode timer off-by-one),
#20 (AM32 loop is 19,608 Hz).
