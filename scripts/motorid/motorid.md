# Motor ID

This script runs a bench test on one motor output to identify a model of
the motor, ESC and propeller. It drives the output through a configurable
test plan (throttle steps, coast-downs, a small and a big frequency sweep,
fixed-frequency sines) while logging the throttle and the ESC-telemetry
rpm at the script rate (about 430 Hz on an H743). The dataflash log is
analysed offline with `motorid.py`, which fits the model and writes a
model file for a simulator.

Run it once with the propeller removed and once with it fitted: the
prop-off run identifies the motor and ESC, the prop-on run the
propeller.

The script needs bidirectional DShot (or another source of ESC rpm
telemetry) on the tested output; without rpm the log is useless.

## Parameters

### MID_ENABLE

Set to 1 to enable the test. When enabled, the test plan runs once each
time the vehicle is armed.

### MID_TESTS

Bitmask of the tests to run, in this order:

| bit | value | test (log id) | default settings |
|---|---|---|---|
| 0 | 1 | throttle steps (1) | 1100 to 1800 us in 100 us steps of 3 s |
| 1 | 2 | coast-downs (2) | 3 times: 4 s at 1500 us, 4 s at zero |
| 2 | 4 | small sweep (3) | 1300 +- 200 us, 1 to 50 Hz in 60 s |
| 3 | 8 | big sweep (4) | 1500 +- 400 us, 1 to 50 Hz in 60 s |
| 4 | 16 | fixed sines (5) | 1600 +- 250 us at 4 frequencies from 1 to 50 Hz, 8 s each |

Each test is preceded by MID_GAP seconds at zero throttle, and 3 s at
zero throttle are logged at the end. The default (15) is the same plan
as Hakea's original `bench.lua` and takes 191 s.

### MID_CHAN

The 1-based output channel of the motor under test. Its
`SERVOn_FUNCTION` must be a motor (33 to 40 or 82 to 85); otherwise the
script refuses to run. ESC telemetry is read for the same output.

### MID_PROP

What is fitted to the motor: 0 no prop, 1 prop, 2 and above a prop type
number of your choice (to test several props). It is written to the log
so the analysis knows which runs are which. The default, -1, means "not
set": the script refuses to run until you set it, so a run is never
logged with the wrong prop state.

### MID_GAP

Seconds at zero throttle before each test.

### MID_STP_MIN, MID_STP_MAX, MID_STP_INC, MID_STP_TIME

Throttle steps: from MID_STP_MIN up to MID_STP_MAX in MID_STP_INC
increments, MID_STP_TIME seconds each.

### MID_CST_PWM, MID_CST_TIME, MID_CST_NUM

Coast-downs: MID_CST_NUM cycles of MID_CST_TIME seconds at MID_CST_PWM,
then MID_CST_TIME seconds at zero throttle.

### MID_SS_BASE, MID_SS_AMP, MID_BS_BASE, MID_BS_AMP

Centre and amplitude (swing either side) of the small and the big sweep.

### MID_F_MIN, MID_F_MAX, MID_SW_TIME

The sweeps rise exponentially from MID_F_MIN to MID_F_MAX Hz in
MID_SW_TIME seconds. The fixed sines are spaced evenly on a log scale
between the same two frequencies. The output is updated at about 430 Hz,
so keep MID_F_MAX at 50 Hz or below (about 9 updates per cycle). The
ESC itself receives DShot frames at the loop rate times SERVO_DSHOT_RATE
(1200 Hz for 400 Hz and rate 3), holding the script's last value.

### MID_SIN_BASE, MID_SIN_AMP, MID_SIN_NUM, MID_SIN_TIME

Fixed sines: centre, amplitude, number of frequencies, and seconds at
each frequency.

### MID_PWM_MAX

The script never commands more than this throttle, whatever the plan
says (default 1900 us).

## Log messages

| message | when | fields |
|---|---|---|
| MIDC | once at the start of each run | ver (plan version), run (count since boot), prop (MID_PROP), chan, func (servo function), poles (SERVO_BLH_POLES), mask (MID_TESTS), pwmin (MOT_PWM_MIN, zero throttle), pwmax (MID_PWM_MAX), gap, dur (plan length, s) |
| MIDP | once at the start | st0, st1, sti, stt (steps), cpwm, ct, cn (coast-downs), ssb, ssa, bsb, bsa (sweeps), f0, f1, swt (frequencies, sweep time) |
| MIDS | once at the start | sinb, sina, sinn, sint (fixed sines) |
| MIDT | every update (~430 Hz) | test (id), pwm (us), f (excitation frequency, Hz, 0 = none), rpm (ESC telemetry, -1 = none) |

MIDT has the same fields as the BNCH message of the older `bench.lua`,
so analysis written for BNCH also works on MIDT.

## How to use

1. Copy `motorid.lua` to `APM/scripts` on the SD card, set
   `SCR_ENABLE` 1 and reboot. The ground station shows "MotorID loaded".
2. Check that ESC rpm telemetry works on the output (e.g. `ESC` log
   message or the ESC telemetry page) and that `MOT_PWM_MIN` is the
   zero-throttle value (DShot: 1000).
3. Set `MID_CHAN`, choose the tests with `MID_TESTS`, set `MID_PROP` to
   0 (prop off) and `MID_ENABLE` to 1.
4. Arm. The script sends "MotorID: run 1: 191 s, out 1, prop 0" and starts.
   Stay clear of the motor. When it sends "run 1 done, disarm now",
   disarm.
5. Fit the prop, set `MID_PROP` to 1 and repeat.
6. Copy the logs and run `motorid.py fit` on them (see `README.md`).

## Safety

- Only use on a bench with the motor clamped down; with the prop fitted
  the test reaches high thrust. Stay out of the prop plane.
- The test only runs while armed. Disarming stops it immediately and
  rearming starts it again from the beginning.
- Clearing `MID_ENABLE` during a run stops the excitation.
- The output override has a 50 ms timeout: if the script stops or
  crashes, the output returns to the vehicle's own command.
- When the plan finishes, the script stops overriding and the output
  goes back to the vehicle. On a multicopter that is armed with the
  throttle low the motors then spin at `MOT_SPIN_ARM`; disarm promptly.
  Other motor outputs are not touched by the script and also spin at
  `MOT_SPIN_ARM` while armed.
- The script refuses to run if the output is not a motor, if `MID_PROP`
  is not set, or if no test is selected, and says why on the ground
  station.
- `MID_PWM_MAX` caps the throttle of every test.
