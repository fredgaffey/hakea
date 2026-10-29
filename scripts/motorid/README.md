# motorid: motor + ESC + prop identification from ArduPilot bench logs

A small, self-contained workflow to measure how a motor actually responds
to throttle (steady rpm, time constant, delay, braking, sag) and to turn
that into a model a simulator can use:

```
motorid.lua  (on the flight controller: excitation + logging)
      |   dataflash .BIN logs, prop off and prop on
      v
motorid.py   (on a PC: fit, validate, report)
      |   motor_model.json  (+ avgmodel_params.json, escsim_*.json)
      v
simulator    (averaged model, ESCSim SITL, ...)  -> blind validation
```

It was built for Hakea (T-Motor 2307 1300KV, 5" prop, 6S, Foxeer Reaper
65A on AM32 FOXEER_F421, ArduPilot on a Matek H743, bidirectional
DShot300), but nothing in it is Hakea-specific except some defaults.

## Files

| file | what |
|---|---|
| `motorid.lua`, `motorid.md` | ArduPilot applet: runs the test plan and logs MIDC/MIDP/MIDS/MIDT |
| `motorid.py` | log reader, fitting, validation, report (importable; thin CLI) |
| `motorid_model.py` | the model: AM32 throttle chain + averaged BLDC motor + prop + battery |
| `tests/test_motorid.py` | end-to-end test on synthetic logs (incl. thrust-stand channels) |
| `tests/test_motorid_lua.lua`, `tests/mock_ap.lua` | runs `motorid.lua` against a mock of the ArduPilot bindings |

Requirements: Python 3 with numpy, scipy, matplotlib and pymavlink;
numba is optional (about 50x faster; without it the model integrates with
a 1 ms step instead of the 50 us AM32 tick, ~1 % rpm difference, and a fit
takes several minutes instead of about one). No dependency on Hakea's
`scripts/sim`.

## 1. Run the bench test

1. Bench: motor clamped down, ESC rpm telemetry working on the output
   (bidirectional DShot or serial telemetry), battery or supply as in
   flight. The FC does not need to be on the motor.
2. Copy `motorid.lua` to `APM/scripts/`, `SCR_ENABLE 1`, reboot.
3. Set `MID_CHAN` (output, 1-based), `MID_PROP 0`, `MID_ENABLE 1`. The
   default plan (`MID_TESTS 15`, 191 s) is steps 1100-1800 us, three
   coast-downs from 1500 us, a 1300+-200 and a 1500+-400 us sweep from 1
   to 50 Hz. See `motorid.md` for every parameter and the safety notes.
4. Arm; wait for "MotorID: run 1 done, disarm now"; disarm.
5. Fit the prop, `MID_PROP 1`, repeat.
6. Copy the two `.BIN` logs from the SD card.

Optional but recommended: measure the motor's phase-to-phase resistance
with a milliohm meter (or a 4-wire measurement with a bench supply). It
sets the absolute scale of the model (see "Identifiability").

## 2. Fit the model

Check what the tool finds in the logs first:

```
motorid.py info LOG_PROP_OFF.BIN LOG_PROP_ON.BIN --eeprom esc_config.bin
```

It lists, per log, the runs (split at arming, at MIDC records and at
gaps), the prop state and how it was decided, the test ids present and
the voltage channel used. Then fit:

```
motorid.py fit LOG_PROP_OFF.BIN LOG_PROP_ON.BIN \
    --eeprom esc_config.bin --r-phase-phase 0.12 \
    --firmware-hash firmware/SHA256SUMS --prop-diameter 0.127 --out results/
```

What it does:

1. **Stage 1, prop-off runs:** motor/ESC parameters: effective voltage
   gain table g(rpm), loop resistance R, friction T0 and b, rotor inertia
   J_m, battery resistance Rb, measurement-chain delay, plus the battery
   open-circuit voltage V0 + V1 t of every run.
2. **Stage 2, prop-on runs, motor fixed:** prop inertia J_p and torque
   coefficient kq (per prop type), battery resistance and voltages.
3. Optional `--joint`: refine everything together from there.
4. **Validation:** by default test 4 (the big sweep) of every run is held
   out of the fit, and whole logs given with `--validate LOG` are used
   only for validation (their battery voltage is refitted, nothing else).
   Errors are reported separately for fit and held-out data.

Useful options: `--prop LOG=off|on|N` (prop state when the log has no
MIDC record and the heuristic is unsure), `--window LOG=t0:t1` (use part
of a log), `--holdout-tests 4` / `--fit-tests`, `--thermal` (fit the
winding thermal model Rth, Cth), `--free Isat` (torque-constant roll-off
at high current), `--fix NAME=value`, `--init NAME=value`, `--motor
earlier/motor_model.json` (reuse an identified motor, fit only a new
prop), `--reference params.json` (compare with an earlier fit),
`--channels` (see below). `-h` lists everything.

### Inputs read from each log

- excitation records: `MIDT` (motorid.lua), else `BNCH` (bench.lua),
  else `SWP` (plain sweep; no test id, counted as test 4);
- `MIDC` context: prop state, output, servo function, poles, test mask,
  plan version;
- `PARM`: `MOT_PWM_MIN/MAX` (the pwm to DShot mapping), `SERVO_BLH_POLES`,
  `SERVO_DSHOT_RATE`, `SCHED_LOOP_RATE`, `SERVOn_FUNCTION`;
- `BAT` (instance 0), `ARM`, and when present `LOAD`, `BPWR`, `ESC`.

The AM32 settings that shape the throttle response (minimum duty, ramp
limit, startup power, comp_pwm, brake on zero throttle, motor KV and
poles) come from the ESC's EEPROM image (`--eeprom`, the 184/192-byte
file the AM32 configurator saves) or `--esc name=value`.

### Prop off or on?

1. `--prop` on the command line, else
2. the `MIDC` record written by `motorid.lua` (`MID_PROP`), else
3. a heuristic on the steady throttle holds: rpm / (Kv x duty x Vbat).
   Without a prop a motor runs within a few % of its no-load speed (Hakea
   1.01-1.02 at duty 0.6-0.8); a prop loads it down (0.89-0.93). Holds at
   duty >= 0.6 are used (thresholds 0.96 / 0.94), else duty >= 0.4 with a
   wider "ambiguous" band; ambiguous runs are skipped with a message
   asking for `--prop`.

### Measurement channels and trust

Default (`--channels auto`): the excitation rpm and the FC battery
voltage `BAT.Volt`, plus the thrust-stand channels below when they are in
the log. Never used unless named explicitly: the FC battery current
(`bat_curr`; on Hakea a floating pin), ESC-telemetry voltage and current
(`esc_volt`, `esc_curr`; wrong on Hakea's ESC) and the IMU (the FC is not
on the motor). The battery voltage scale only needs to be roughly right:
the gain table g absorbs a constant voltage scale error, so rpm
predictions stay right as long as the simulator is fed the same voltage
measurement.

## 3. Outputs

In `--out` (suffix `--tag`):

| file | content |
|---|---|
| `motor_model.json` | everything: averaged-model parameters (motor, props, battery per run), 1-sigma, ESC settings, ESCSim sections, fit/hold-out errors, steady table, frequency response, and provenance: sha256 of every log and of the EEPROM, firmware hash file, tool version, date, command line, priors |
| `avgmodel_params.json` | flat parameters in the layout of Hakea's `scripts/sim/calibrate.py` (`g0..g4, R, T0, b, Rb, delay, J_m, J_p, kq, V0_<run>, V1_<run>`), for `predict_blind.py` |
| `escsim_prop_off.json`, `escsim_prop_1.json` | ESCSim SITL model files (format of `ESCSim/SITL/example.json`) |
| `report.md` | inputs, identifiability statement, parameters, errors, steady rpm vs throttle, frequency response, speed/bandwidth vs throttle |
| `timeseries_<run>.png`, `freqresp_<run>.png`, `steady.png` | plots |

ESCSim mapping: `resistance = (R - 2 rds_on)/2` (R is the line-to-line
loop resistance of the 6-step drive; `--rds-on`, default 2 mOhm),
`inductance = L/2`, `inertia = J_m (+ J_p)`, `load_k_omega2 = kq`,
`static_friction = T0`, `damping = b`, battery `voltage = V0`,
`resistance = Rb`. kv is the nominal value; the averaged model's
effective gain g(rpm) has no SITL parameter (the SITL's own dead time and
commutation produce it), the report gives `kv_effective_hint`.

## 4. Simulate and validate blind

The model files plug into Hakea's simulation scripts (optional, in
`../sim/`):

- `sim/sitl_run.py` runs the AM32 firmware in SITL with an ESCSim model
  file (`escsim_*.json`); `sim/sitl_replay.py --model escsim_prop_off.json
  --dataset np33` replays a bench log through it; `sim/sitl_calib.py`
  tunes the SITL-only parameters starting from averaged-model parameters.
- `sim/predict_blind.py --params avgmodel_params.json` predicts a new test.

Blind validation, the honest test of the model:

1. Design a test the model has not seen (different levels, frequencies,
   or a different battery state), e.g. `motorid.lua` with only the fixed
   sines (`MID_TESTS 16`) at new frequencies, or Hakea's `bench2.lua`.
2. Pre-register: simulate it with the fitted model, write the predicted
   quantities with uncertainty (`predictions.csv`), and record the file's
   sha256 somewhere time-stamped (commit, message) *before* running it.
3. Run the bench test.
4. Compare with the same analysis code on the real log
   (`sim/compare_blind.py LOG.BIN --pred predictions.csv`; it expects
   bench2's test ids 11-14).

## Model and identifiability

The model (`motorid_model.py`) is a commutation-averaged one: it treats
the 6-step drive as a DC motor but keeps the ESC firmware's throttle
path, because that is where most of the dynamics of a small motor live:

- ArduPilot pwm -> DShot `2000 (pwm - MOT_PWM_MIN)/(MOT_PWM_MAX - MOT_PWM_MIN) + 48`;
- AM32: duty = map(DShot 47..2047 -> min_duty..2000), startup clamp,
  low-rpm duty limit, ramp limiter per 20 kHz tick (startup/low/high rpm);
- voltage g(rpm) d Vbus on `L dI/dt = g d Vbus - Ke w - R I`,
  `J dw/dt = Kt I - kq w|w| - b w - T0`, `Ke = Kt = 60/(2 pi Kv)`;
- battery `Vbus = V0 + V1 t - Rb Ibus`, `Ibus = g d I`;
- zero throttle with comp_pwm: AM32 ramps the duty to 0 while still
  commutating, shorting the windings (strong braking); below ~2000 rpm it
  stops tracking, the bridge turns off and telemetry reports 0;
- a pure delay for the measurement chain (Lua -> loop -> DShot -> ESC ->
  telemetry -> Lua), ~5 ms on Hakea;
- optional winding thermal model (R +0.393 %/K of copper temperature,
  Rth, Cth) and torque-constant roll-off Isat.

**Identifiability.** With only rpm and battery voltage the model has an
exact symmetry: `J, kq, b, T0, Cth, Isat -> s x` and `R, Rb, L, Rth -> x / s`
give identical rpm for any s (the motor current scales by s). The data
cannot tell s, but torque, current, inertia and heating depend on it. So
the tool needs one absolute number:

- by default a prior on R: `--r-phase-phase` (measured phase-to-phase
  resistance; prior = that + 2 rds_on, 10 %) or `--prior R=value,sigma`;
  without either it uses Hakea's `R = 0.16 +- 0.04` and says so loudly;
- when a bench current sensor or a torque cell is logged, s is estimated
  from them (inverse-variance combined with the R prior) and applied to
  all parameters.

The report always states which one fixed s. `tests/test_motorid.py`
checks both: with the stand it recovers R, J, kq to a few %; without it,
J/J_true = kq/kq_true = R_true/R exactly as the symmetry says while the
rpm is still predicted to the noise level.

## Thrust stand and current sensor (optional)

If the bench has a load cell (Hakea: a 6-axis Stewart-platform stand,
ESP32 + 6 HX711) and/or a current sensor, log them into the same `.BIN`
and `motorid.py` uses them automatically:

| message | fields (SI) | meaning |
|---|---|---|
| `LOAD` | `I` instance (B), `Fx, Fy, Fz` (N), `Mx, My, Mz` (N m) | forces/moments on the motor mount, motor frame, z along the shaft. Minimum: `Fz`, `Mz` |
| `BPWR` | `I` instance (B), `Volt` (V), `Curr` (A) | bench power monitor at the ESC input |

Sign conventions: `Fz` positive is thrust; `Mz` is the reaction torque on
the stator about the shaft. If the median of a channel is negative the
tool flips it and notes that in the JSON. With timestamps from the FC log,
sensor latency only matters for the dynamic parts; the fit uses steady
holds (mean of the second half / last 1 s of each constant-throttle hold).

What is fitted when present:

- thrust: `Fz = kT w^2 + offset` per prop type; with `--prop-diameter`
  also `Ct = kT 4 pi^2 / (rho D^4)`;
- torque: `Mz = kQ w^2 + offset` directly (`kQ_direct`, `Cq`, `Cp = 2 pi Cq`),
  and the model's stator torque (`Kt I - b w - T0 = kq w^2 + J dw/dt`)
  against `Mz`, which gives the scale s;
- current: model bus current against `BPWR.Curr` (`+ offset` for the ESC's
  own consumption), which also gives s. `BPWR.Volt` replaces `BAT.Volt`
  as the voltage measurement.

So with current (or torque) logged, R, Kt-equivalent losses, inertias and
kq become absolute; without them the tool falls back to rpm-only
identification with the R prior.

A simple way to get the stand data into the log is a Lua script reading
the ESP32 over a serial port (`SERIALn_PROTOCOL 28` = scripting), e.g.
the ESP32 sends one line per sample at 50-100 Hz:

```
$L,<ms>,<Fx>,<Fy>,<Fz>,<Mx>,<My>,<Mz>,<V>,<I>*<checksum>\n
```

and the script (`serial:find_serial(0)`, `port:begin(115200)`,
`port:read()` byte by byte, split on commas) writes
`logger:write('LOAD','I,Fx,Fy,Fz,Mx,My,Mz','Bffffff', ...)` and
`logger:write('BPWR','I,Volt,Curr','Bff', ...)`. HX711s sample at 10 or
80 Hz, so keep the stand's own filtering short and log the raw rate; the
steady-hold averaging does the rest. Any other stand (RCbenchmark/Tyto,
a single load cell) works the same way as long as it produces these two
messages.

## How this complements the harmonic notch / FFT

ArduPilot's harmonic notch (`INS_HNTCH_*`) filters gyro vibration at the
motor rotation frequency and its harmonics; it needs to know the rotation
frequency (from ESC rpm telemetry, the FFT, or a throttle-based estimate
with `INS_HNTCH_REF` and `INS_HNTCH_FREQ`). motorid models something
different: how the motor actually moves when commanded, under load
(steady rpm vs throttle, time constant and delay, rpm ripple when the
throttle is pulsed). They meet in a few places:

- **Throttle-based notch without telemetry:** the steady rpm vs throttle
  table (and the model's `steady_state`) gives the rotation frequency at
  any throttle and battery voltage, so `INS_HNTCH_REF` (hover throttle)
  and `INS_HNTCH_FREQ` (rotation frequency at hover) can be set from the
  bench instead of guessed; the voltage dependence shows how far the
  throttle-based estimate drifts as the battery sags.
- **Minimum frequency and dynamics:** the report's "speed and bandwidth
  vs throttle" table gives the lowest rotation frequency in the flight
  envelope (for `INS_HNTCH_FREQ`/`INS_HNTCH_BW`) and how fast rpm can
  follow throttle (time constant ~10-30 ms here). A notch driven by
  throttle leads the real rpm by that lag plus the ~5 ms chain delay;
  with rpm telemetry the notch tracks the real rotation instead.
- **Pulsed (1P) throttle, as on Hakea:** once-per-revolution torque
  pulsing puts energy at exactly the rotation frequency, i.e. on top of
  the notch's first harmonic. The model predicts the rpm ripple
  amplitude from a pulsing command (ramp limiter, inertia, delay), which
  tells whether rpm-based notch tracking will wobble with it.

## Future work (sketch, not implemented)

Fuse the bench / thrust-stand model with **flight logs** to get dynamic
thrust and power curves with airflow:

- For a single-rotor craft the total rotor thrust is simply
  mass x measured specific force along the shaft (vibration-filtered IMU
  accelerometer), so there is no per-motor attribution problem.
- Shaft torque from the bench motor model given rpm (ESC telemetry) and,
  if available, current; rotor power = torque x w.
- Inflow / advance ratio from EKF velocity minus the wind estimate
  (axial and edgewise components relative to the rotor axis).
- Fit Ct and Cp vs advance ratio (axial and edgewise), anchored at the
  static thrust-stand point (J = 0).
- Excitation: ArduPilot's SystemID mode (chirps on throttle and attitude
  axes), plus climbs, descents and forward flight at several speeds.
- Method: grey-box batch least squares on the logs, or an augmented EKF
  estimating the coefficients online.
- Limitations: wind and airspeed uncertainty, ground effect near the
  ground, mass and CG errors, vibration on the accelerometers, dynamic
  inflow lag (relevant to 1P pulsing, where the inflow cannot follow the
  per-revolution torque changes). The bench IMU data is useless (FC not on
  the motor), but in flight the IMU measures exactly what is needed.

## Hakea results

`../../notes/motorid/` has the fit of logs 33 (prop off) and 34 (prop
on, second arm), with log 32 (prop-on sweeps) as validation, and a
comparison with `notes/sim/calib/params_v2.json`:

```
motorid.py fit logs/sdcard/00000033.BIN logs/sdcard/00000034.BIN \
    --validate logs/sdcard/00000032.BIN --prop 32=on \
    --eeprom ~/Downloads/esc1_config.bin --firmware-hash firmware/SHA256SUMS \
    --prop-diameter 0.127 --reference notes/sim/calib/params_v2.json \
    --out notes/motorid            # add --joint --tag _joint for the joint refinement
```

## Tests

```
python3 tests/test_motorid.py                 # synthetic logs -> fit -> checks (~1 min)
```

The Lua tests need a Lua 5.3; ArduPilot's own Lua sources work (they have
a few ArduPilot hooks, stubbed for a PC build):

```
mkdir .luabuild && cp ../../ardupilot-hakea/libraries/AP_Scripting/lua/src/*.[ch] .luabuild/
# + small stubs (see .luabuild/runlua.c, AP_*/ headers), then
.luabuild/runlua -c motorid.lua                           # syntax check
.luabuild/runlua tests/test_motorid_lua.lua default       # also: short sines noprop_set not_motor disarm notelem
```

The mock checks parameter names and indices, `logger:write` limits
(4-char name, labels < 58 chars, format < 15 chars, integer fields), and
GCS text length (50 chars), and runs the scheduler at ~430 Hz.

## Limitations

- The averaged model ignores PWM switching, commutation ripple and
  back-EMF sensing (desyncs are out of scope; that is what the ESCSim
  SITL is for).
- One effective-gain table for both load cases: with the motor fixed
  from prop-off data the prop-on steady rpm is a few % off at high
  throttle; `--joint` trades a little prop-off accuracy for that.
- The cost surface is rough (the ESC logic has integer maps and mode
  thresholds), so the fit uses a fairly large finite-difference step
  (`--diff-step 0.01`) and restarts; on synthetic data that made the
  result independent of the starting values. Use `--init` to try others.
- The 50 Hz sweep end is limited by the Lua update rate (~430 Hz) and the
  telemetry; the big sweep is held out because the ramp limiter makes it
  strongly non-linear and it is the hardest test, i.e. a good check.
- Parameter 1-sigma values come from the Jacobian and are optimistic
  (residuals are correlated in time).
- The battery is a single resistance Rb. On Hakea's logs the measured
  BAT.Volt follows the sag of the 3 s throttle steps but ripples much
  less than the model during the 1-50 Hz sweeps (see the bottom panel of
  `timeseries_p34.png`). ArduPilot's analog read is a plain 100 ms mean,
  so this is most likely the battery itself: a LiPo's sag has a fast ohmic
  part and a slow polarization part (seconds), and only the ohmic part
  acts at sweep frequencies. A series R0 + (Rp || Cp) battery branch
  would fit both; until then Rb is a compromise between the DC and AC
  values. It affects rpm by ~1 %.
- The model under-predicts how much the big sweep (+-400 us) is
  attenuated above ~5 Hz (e.g. prop on at 20 Hz: bench 8, model 12.6 rpm
  per us), while the small sweep (+-200 us) matches. Something
  large-signal is missing (current/torque saturation, the ramp limiter
  acting harder than modelled, or ESC current limiting). That is the bulk
  of the hold-out error; `--free Isat` is one candidate.
