# Log index

Flight-controller dataflash logs copied from the SD card into `logs/sdcard/`.
File dates are 1980 (no clock without GPS), so go by log number.

Hardware for all: Matek H743-SLIM V3 (`MatekH743-bdshot`, ArduCopter 4.8.0-dev
`261f5ddf`), Foxeer Reaper 65A 4-in-1 on AM32 2.21 `FOXEER_F421` (motor on ESC 1 /
output 1), T-Motor 2307 1300KV, 6S.

**Prop column:** taken from what was said at the time where known, otherwise
inferred by comparing RPM against commanded throttle with the prop-on (34) and
prop-off (33) step references. "n/a" means the log has no RPM data while the motor
ran (motor not spinning, or telemetry broken), so it can't be told from the data.

| Log | Date | Prop | Setup | Contents | Notes |
|---|---|---|---|---|---|
| 05-13 | 2026-10-26 | n/a | various | Early setup and motor tests | No usable RPM data |
| 14-17 | 2026-10-26 | n/a | DShot600, `BDMASK 1` | `sin.lua` 1300±50 @ 3 Hz | No RPM: partial bidir mask broke ESC 1/2 |
| 18 | 2026-10-26 | on (low confidence) | DShot600, `BDMASK 0` | `SINE` runs: 1300±50 @ 3 Hz, ±100 @ 6 Hz, ±100 @ 50 Hz, ±500 @ 50 Hz | Serial-telemetry RPM only (~9 Hz). RPM slightly closer to the prop-on reference, but at ~1300 the two references differ by only ~6% |
| 19-25 | 2026-10-26 | n/a | bidir mask experiments | 1300/1700±500 @ 50 Hz | No usable RPM |
| 26 | 2026-10-26 | on | `BDMASK 0`, `TRATE 500` | 1700±500 @ 50 Hz | Serial RPM at 100 Hz; RPM telemetry dropped out at 19.98 s (motor kept running). RPM matches the prop-on reference |
| 27-30 | 2026-10-26 | n/a | bidir mask experiments | 1700±500 @ 50 Hz | No usable RPM (28: partial mask; 30: `BDMASK 15` before ESC power-cycle) |
| 31 | 2026-10-26 | on | DShot300, `BDMASK 15`, `DSHOT_RATE 3` | 1700±500 @ 50 Hz, ~10 s | First clean bidir run; no desync. Prop confirmed by Fred |
| 32 | 2026-10-26 | on | as 31 | `SWP` sweep 1500±400, 1→50 Hz (run 2 complete, 595-662 s) | Time constant ~23.5 ms, ~6.5 ms delay, no desync |
| 33 | 2026-10-26 | **off** | as 31 | `bench.lua` full run (`BNCH`): steps, coast-downs, small sweep, big sweep | Complete, 316.8-504.8 s; idle at 1100 until 571 s; ESC 57 °C. Prop-off reference |
| 34 | 2026-10-26 | **on** | as 31 | `bench.lua` full run (`BNCH`), second arm 249.0-446.1 s (first arm 199.9-237.6 s aborted) | Steps: 1500 → 15,650 RPM, 1800 → 22,480 RPM; ESC 25 → 59 °C; batt 24.8 → 23.2 V. Prop-on reference |
