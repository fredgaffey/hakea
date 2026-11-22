# ArduPilot starting parameters, 5" single-rotor vehicle (vehicle_spec_v1.md)
All values are **starting point, tune on bench**. Fred applies them himself. Names verified in `ardupilot-hakea/libraries/` (submodule empty in worktree; read from the main checkout). Line refs: HN = Filter/HarmonicNotchFilter.cpp, BL = AP_BLHeli/AP_BLHeli.cpp, SC = SRV_Channel/SRV_Channels.cpp, IS = AP_InertialSensor/AP_InertialSensor.cpp.

## 1. Harmonic notch on ESC rpm (hover 1P ~237 Hz = 14.2k rpm trimmed, range 204-261 Hz over T0.7-1.25)
| param | value | why | source |
|---|---|---|---|
| INS_HNTCH_ENABLE | 1 | enable (reboot, then refresh params) | HN:66 |
| INS_HNTCH_MODE | 3 (ESC Telemetry) | rpm from ESC | HN:127 |
| INS_HNTCH_REF | 1 | enables dynamic tracking; sensor rpm is converted to Hz | HN:121 |
| INS_HNTCH_FREQ | 160 | MINIMUM tracked frequency for ESC-telemetry mode (HarmonicNotchFilter.cpp @Param FREQ: "the minimum limit of center frequency variation for all other notch types"); must sit below the T0.7 1P (~190 Hz), so NOT the 218 Hz hover value | HN:74 |
| INS_HNTCH_BW | 60 | about 1/3 of 1P, covers the 204-261 Hz sweep and rpm lag | HN:82 |
| INS_HNTCH_ATT | 30 | dB; default 40 is deeper than needed for a ~1 rad/s line | HN:90 |
| INS_HNTCH_HMNCS | 3 (bits 0+1 = 1P and 2P) | 1P = 176 mN m torque-reaction line; 2P = blade-pass of a 2-blade rotor | HN:113 |
| INS_HNTCH_OPTS | 16 (triple notch) | wider flat stop band per harmonic for a lightly damped rotor | HN:137 |
| INS_HNTCH_ESCMSK | bit of the motor ESC (1 if ESC 1) | selects which ESC feeds the notch; needed if Multi-Source (OPTS bit 1) is used | HN:183 |
| INS_HNTCH_FM_RAT | 1.0 (default) | no frequency-ratio floor | HN:144 |
Note: the "start-up/bench check" is INS_LOG_BAT_MASK + FFT of the log, and confirm ESC rpm in ESC telemetry logs before trusting MODE 3.

### rpm reaching the FC
| param | value | source |
|---|---|---|
| SERVO_DSHOT_ESC | 3 (AM32 + EDT) | SC:214 (values list SC:213) |
| SERVO_BLH_BDMASK | bit of the motor output (bidirectional DShot) | BL:146 |
| SERVO_BLH_POLES | 14 (check motor; 2307 is normally 14) | BL:129 |
| SERVO_DSHOT_RATE | leave 0 at first | SC:207 |
| SERVO_BLH_OTYPE | set to a DShot type; **I did not verify the value list, check the param description** | BL:114 |
Not verified: any AM32-side setting (that is Fred's am32-hakea code) and the H743 hwdef output-to-timer mapping for bidirectional DShot. RPM_TYPE (ESC telemetry) exists in AP_RPM/AP_RPM.cpp:92 but is not needed for MODE 3.

## 2. Gyro filter
| param | value | source |
|---|---|---|
| INS_GYRO_FILTER | 80 Hz (start 60-100) | IS:350 (default is board/vehicle dependent, IS:65-73) |
One line: a lower cutoff removes more of the ~237 Hz line and its noise but adds phase lag inside the attitude loop; 80 Hz is far above the ~10-20 Hz attitude bandwidth (little lag) and gives roughly 17 dB at 218 Hz (assumes second order), with the notch doing the rest.

## 3. Outputs (TODO, Fred's own code)
FRAME_CLASS: TODO. Mixer: TODO.
| output | function | param | value |
|---|---|---|---|
| MAIN/AUX __ | motor (DShot, rotor 2307) | SERVO_n_FUNCTION | TODO |
| __ | vane servo (2 steering vanes, yaw) | SERVO_n_FUNCTION | TODO |
| __ | cyclic | n/a: spec has no cyclic servo, passive flexure | n/a |
The 4 fixed vanes have no output. Keep the servo output on a different timer group from the DShot output.
