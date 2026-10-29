# Tool issues found during the sim study

This is a running log. Nothing has been filed upstream; Fred decides what to report.

Severity: **high** means it changes conclusions if you don't know about it; **medium** means it costs time or
needs a workaround; **low** means cosmetic or an interface issue. "Affects study" says whether this study's
results depend on it and how that was handled.

Paths: `am32-sim` = ~/vehicles/hakea/am32-sim (AM32 fork a47e94c + study patch).
`ESCSim` = ~/am32/ESCSim (d53f147).

---

## 1. SITL: bidirectional DShot causes repeated false desyncs below ~3,000 rpm
- **Project/file:** AM32 SITL, `Mcu/SITL/Src/sitl_input.c` (BDShot reply path), plus the firmware DShot
  telemetry ISR timing under SITL scheduling.
- **What happens:** with bidirectional DShot at 1200 frames/s, a motor at DShot 248 (ArduPilot 1100 µs) never
  settles. It loops through desync, restart, desync (35 desyncs in 4 s). Unidirectional DShot at the same
  frame rate runs cleanly at 3,830 rpm. The bench runs bidirectional DShot at the same settings with no
  desyncs.
- **Repro:**
  ```
  cd ~/vehicles/hakea/scripts/sim
  python3 -c "import sitl_run as sr; st=sr.Stim(833_333,bidir=True); st.const(0,3); st.const(248,2); lg=sr.run(st,'models/sitl_v1_noprop.json'); print(lg['desync'][-1], lg['rpm'][-1])"
  ```
  Compare with `bidir=False`.
- **Evidence:** progress notes, stage 1, "SITL bidir" run (desync 35 vs 5 at start-up only).
- **Severity:** high for anyone using the SITL to study desyncs with BDShot.
- **Affects study:** yes; worked around. All SITL runs use unidirectional DShot. On hardware the 1200 Hz
  telemetry reply adds ISR load, and this is not represented.
- **Suggested issue title:** "SITL: bidirectional DShot at 1.2 kHz causes desync loop at low rpm (not on
  hardware)".

## 2. SITL: start-up straight into mid/high throttle locks at ~2-3k rpm with a loaded rotor
- **Project/file:** AM32 SITL motor/comparator model (`Mcu/SITL/sim/motor.c`) together with the AM32 start-up
  logic (`Src/main.c`: startup ramp, low-rpm duty clamp, min_startup_duty).
- **What happens:** from rest, a DShot step to ≥600 (ArduPilot ≥1300 µs) with the calibrated prop model
  stays at 2-3k rpm. Duty is held at 400-500 and phase current at 20-30 A rms, with no desync counted, for
  as long as it is held. Ramping gently from DShot 48 works, as does starting at 248. On the bench the
  same steps (logs 33/34 tests 2-4) start fine every time.
- **Repro:** `predict_blind.sitl_plan_run()` with the bench2 plan; or `sitl_run.Stim`: 3 s zero, then
  const(600, 1.5) with `notes/sim/sitl_calib/sitl_base_v2.json`.
- **Severity:** medium. Bench replays need a soft start.
- **Affects study:** worked around with soft starts. `sitl_calib.run_segment` ramps over 1 s, `grid.py`
  ramps to base. The SITL was not used for the blind start-up quantities.
- **Suggested issue title:** "SITL: start-up into high throttle locks in a low-rpm state with prop-loaded
  rotor".

## 3. SITL: `commutation_transfer` decides the desync behaviour, and neither end matches hardware
- **Project/file:** `Mcu/SITL/sim/motor.c` (`esc.commutation_transfer`); ESCSim `SITL/DEMAG.md`.
- **What happens:**
  - With `commutation_transfer = 1`, the outgoing-phase current is transferred instantly at commutation. Demag
    then never masks a zero crossing, and no desync occurred anywhere in the 1P grid, even at ±50 % at 400 Hz.
  - With 0.5-0.8 (L 15 µH), bench-replay sweeps that ran clean on hardware produce tens to hundreds of desyncs.
  - With 0, steady steps fail as well.
- **Severity:** high for desync predictions. The SITL can only bracket the desync boundary; it cannot predict
  it.
- **Affects study:** yes; stated in the report. Grids were run at 1.0 (nominal) and 0.8 (pessimistic).
- **Suggested fix:** a physically based commutation model (finite di/dt through L with the actual bridge
  voltages and diode clamping, which the solver already has), validated against a measured demag waveform from a
  scope on a real ESC. Expose it with a single fidelity flag rather than a free fraction.

## 4. SITL: effective speed per duty differs from hardware at low duty
- **What happens:**
  - SITL no-load speed at duty 0.117 is ~85 % of Kv·d·V.
  - The bench gives 99 % at the same duty (log 33 steps).
  - The calibrated SITL needs kv 1380 to match mid/high throttle, and is still ~9 % slow at 1100 µs.
- **Likely cause:** how dead time and the comp-PWM "+1" compare value are represented at a 160 MHz G431-style
  TIM1. The F421 is 120 MHz with different rounding.
- **Severity:** medium (operating-point mismatch at low throttle).
- **Affects study:** the grid uses base 1500 µs (~15k rpm), where the SITL matches within ~3 %.

## 5. SITL emulates G431 peripherals and timing, not the F421
- **Project/file:** `Mcu/SITL` (README: "The emulated MCU follows the G431 target").
- **What happens:** the CPU, timer clocks, comparator and blanking follow the G431, while Fred's ESC is an AT32F421.
  ISR execution time is essentially zero on the host.
- **Severity:** medium.
- **Affects study:** the desync margins are not F421-specific. Renode (F421 instruction-level emulation) was
  used as the cross-check.

## 6. SITL: the stimulus over UDP depends on host timing, and input is polled every 100 µs
- **Project/file:** `Mcu/SITL/Src/sitl_input.c` (`sitl_input_poll` from `sitl_sched.c` every 100 µs, one
  packet per poll), ESCSim `SITL/sitl_dshot.py`.
- **What happens:**
  - The maximum is one frame per 100 µs of sim time.
  - Frames are timed by the Python sender's wall clock, so the stimulus phase in sim time jitters with host load
    and speedup.
  - Queued packets are consumed one per poll, so they replay late.
- **Severity:** medium for frequency-response studies.
- **Affects study:** worked around with an in-sim deterministic waveform generator (`AM32_SITL_WAVE`, in the
  am32-sim patch).
- **Suggested feature:** upstream a "wave file" or scripted-stimulus input, and a high-rate study log written by
  the sim thread (also in the patch), so tests are deterministic.

## 7. SITL: the state stream is limited to wall-clock rates; there is no lossless high-rate log
- **Project/file:** `Mcu/SITL/Src/sitl_state.c` (`apply_period`: ≥5000·speedup ns; UDP, may drop).
- **What happens:** to record 20 kHz for minutes without loss you must slow the sim. The physics log is 1 kHz
  and has no firmware state.
- **Affects study:** worked around with the `AM32_SITL_STUDY_LOG` ring buffer and thread (in the patch).

## 8. SITL: a firmware reset re-execs the process and simulated time restarts at 0
- **What happens:** any script driving the SITL by sim time loses its time base after a watchdog or
  signal-loss reset.
- **Affects study:** in wave mode a reset now ends the run (`sitl_study_end`), so it is visible rather than
  silently restarting.

## 9. Renode/ESCSim: fixed 20 µs exchange batch between firmware and physics
- **Project/file:** ESCSim Renode bridge (`AM32_F051_Bridge.cs`, `batchUs` hard-coded to 20).
- **What happens:** comparator toggles reach the firmware up to 20 µs late (~12° electrical at 14k rpm).
  Changing the batch to 2 µs moved ripple by ~12 % and flipped the sign of the ramp-limited DC offset.
- **Severity:** medium-high for timing-sensitive questions such as desync and demag.
- **Suggested fix:** a CLI option for the batch period (the Renode sub-agent used a wrapper,
  `scripts/sim/renode/gen_batch_wrapper.py`), and a default small enough for commutation studies.

## 10. Renode/ESCSim: the DroneCAN bridge runs on wall-clock time
- **What happens:** with Renode at ~0.3× real time, a 250 Hz RawCommand stream reaches the firmware at ~90 Hz
  of virtual time. There is no CAN source scheduled in virtual time.
- **Severity:** medium. It blocks CAN-input timing studies in Renode.
- **Suggested feature:** a virtual-time CAN frame scheduler inside Renode.

## 11. AM32 (L431 CAN build): CAN node id 0xFD (253) from an F421 EEPROM hangs start-up
- **Project/file:** AM32 `Src/DroneCAN/DroneCAN.c` (node-id handling) with `Src/main.c` start-up.
- **What happens:** Fred's EEPROM has 0xFD in every CAN byte (176-183, never written on the F421). Loaded into
  an L431_CAN build in Renode, the node stays anonymous and keeps trying DNA. In that state the motor never
  leaves start-up: duty is capped at 440, and 10 bemf timeouts trigger stuck-rotor protection. Node 11 works,
  and so do node-less non-CAN builds.
- **Repro and evidence:** `notes/sim/renode/renode-crosscheck.md` and `notes/sim/renode/diagnostics/`.
- **Severity:** medium. It is not verified on hardware, and it matters if Fred moves the EEPROM to a CAN ESC.
- **Suggested issue title:** "DroneCAN build: invalid node id in EEPROM leaves motor stuck in startup".

## 12. AM32: `last_duty_cycle < 150` forces the start-up ramp (2/tick) while running
- **Project/file:** `Src/main.c` ~1488: `if (zero_crosses < 150 || last_duty_cycle < 150) max_duty_cycle_change = max_ramp_startup;`.
- **What happens:**
  - Whenever a throttle modulation takes the duty below 150 (7.5 %), the next ramp-up is limited to 2 per tick
    (40,000 duty/s), even with the motor running at 15k rpm.
  - In the SITL grid at ±50 % (trough at DShot 48), this distorts every cycle. Duty spends most of the period
    crawling up from ~100, and the mean rpm collapses from 15k to 8-10k.
- **Severity:** high for PROFILE_MODE design: keep the modulated duty above 150, or change the condition to
  `zero_crosses < 150` only.
- **Affects study:** yes, explains the ±50 % anomalies.

## 13. AM32: `desync_happened` is uint8 on non-DroneCAN targets
- **What happens:** on FOXEER_F421 the counter wraps at 256 (`Src/main.c` ~371). Long tests can under-count.
- **Severity:** low.

## 14. AM32: EEPROM `max_ramp` can only lower the ramp limits, and the limit is a uint8
- **What happens:** `RAMP_SPEED_HIGH_RPM` > 16 has no effect while `max_ramp` (Fred's: 160) is clamped by the
  `/10` path. `max_duty_cycle_change` is uint8, so the maximum is 255.
- **Severity:** medium (surprising). This is not a bug, but the documentation should say it.
- **Workaround in the study:** `SIM_RAMP_HIGH_OVERRIDE`.

## 15. Tooling hazard: running Python from /tmp imported a stray `coverage.py`
- **What happens:** numba optionally imports `coverage`. A script run with `/tmp` as the working directory picked up
  an unrelated `/tmp/coverage.py`, which rewrote `~/_hbc2js/hbc2js-local-corpus/ceiling-coverage.json`
  (14:31). This was reported by the motorid sub-agent.
- **Severity:** medium (a side effect outside the project).
- **Fix:** run with `python -I`, or not from /tmp. **Fred: please check that file.**

## 16. ArduPilot bench configuration traps (seen in Fred's logs, not tool bugs as such)
- The BAT current is read from an unconnected CURR pin (phantom ~65-69 A that follows the battery voltage). The
  monitor should be disabled or the pin unset.
- ESC telemetry voltage/current are wrong on this board (FOXEER_F421 divider/shunt). Use FC BAT voltage only.
- `SERVO_BLH_BDMASK` partial mask inverts the group signal but not the CRC (found earlier, see project notes).
  This is a candidate ArduPilot issue.

## 17. Renode/ESCSim: scripted bidirectional DShot stops decoding after 1 s
- **Project/file:** ESCSim Renode `AM32ThrottleGenerator` (`Bidirectional = True` set from a script rather than
  the GUI link).
- **What happens:** the firmware detects inverted DShot and arms. Then `dshot_goodcounts` freezes after exactly
  1.0 s of frames (3899 good + 100 bad at 250 µs; 1899 + 100 at 500 µs), so command 13 (EDT enable) is never
  decoded. This is the same with older firmware, so it is not PR #438.
- **Repro:** `scripts/sim/renode/edt_bidir_debug_v2.py` (env B, FR, TB, ELF; monitor TCP 34574).
- **Severity:** medium. It blocks running EDT/EDT2 telemetry under Renode from scripts.
- **Affects study:** the EDT2 stress metric was reproduced from RAM instead (`notes/sim/renode/v2/renode-v2.md`).
- **Suggested issue title:** "Renode: scripted bidirectional DShot stops decoding after 1 s".

## 18. AM32 PR #438: commutation-stress metric is a window peak and saturates at the first displaced zero cross
- **Project/file:** PR #438 `Src/dshot.c` (`dshot_note_zero_cross_interval`, `dshot_take_commutation_stress`:
  min(255, peak·255/thiszctime)).
- **What happens:** in Renode with a 2 µs physics batch, about 7 % of zero crosses are displaced by half a step
  (demag or chatter right after commutation), even at steady throttle. Every 50 ms window then reads 255. With
  a 20 µs batch it reads 53 at steady and 255 above a ±20 % applied swing. A single event per window saturates
  it, so the value carries little graded information.
- **Severity:** design note, low-medium. It is useful as an alarm but not as a margin.
- **Suggestion:** report a percentile or a count of displaced intervals per window, alongside the peak.

## 19. Renode/ESCSim: basic timer period is ARR, not ARR+1 (loop runs at 20.000 kHz instead of 19.608 kHz)
- **Project/file:** ESCSim `AM32_STM32_BasicTimer.cs` (`timer.Limit = value & 0xFFFF`). Found by the Renode
  sub-agent in round 3.
- **What happens:** a real STM32/AT32 up-counter's period is ARR+1. AM32 sets TMR14 pr = 1000000/20000 = 50 at
  1 MHz, so hardware ticks every 51 µs (19,608 Hz) but Renode ticks every 50 µs. The same off-by-one applies to
  every timer that uses this model (TMR16 commutation timer etc.).
- **Evidence:** Fred's PROFILE v1 measured exactly 250.000 … 1000.000 Hz in Renode. Hardware arithmetic gives
  ×0.980, and the AM32 SITL (which models ARR+1) gives 245.1 … 980.4 Hz.
- **Severity:** medium. ~2 % timing errors in everything loop-rate based. Small effect on commutation timing.
- **Affects study:** the PROFILE frequency predictions use the SITL/hardware values; this is noted in
  predict-1khz/predictions.md.
- **Suggested issue title:** "Renode basic timer uses ARR as period (should be ARR+1)".

## 20. AM32: the "20 kHz" control loop actually runs at 19,608 Hz
- **Project/file:** `Mcu/f421/Src/peripherals.c:170` `TMR14->pr = 1000000 / LOOP_FREQUENCY_HZ;` (and similar on other
  MCUs). This should be `- 1`.
- **What happens:** the loop period is 51 µs. Anything assuming LOOP_FREQUENCY_HZ is 2 % off: ramp rates per
  second, timeouts, and Fred's PROFILE v1 frequencies and step times (×0.980 / ×1.020).
- **Severity:** low for stock AM32. Relevant for PROFILE_MODE (compute phase_inc from 1e6/(1e6/LOOP_FREQUENCY_HZ+1),
  or fix pr).
- **Suggested issue title:** "Loop timer ARR off by one: control loop runs at 19.6 kHz, not 20 kHz".
