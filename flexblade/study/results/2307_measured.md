# 2307 1300KV measured vs guessed (sources: notes/log-index.md, notes/motorid/report.md, logs/sdcard/*.BIN)
| Quantity | Measured | Source | Replaces guess |
|---|---|---|---|
| ESC | Foxeer Reaper 65A 4-in-1, AM32 2.21 FOXEER_F421, 6S | log-index.md:7 | (max_current 40 A: rating is 65 A/ch, not a motor limit) |
| Peak/sustained current | UNUSABLE: BAT.Curr reads 64-69 A even at idle, ESC.Curr 80-90 A on all 4 ESCs (logs 31-34); sensors uncalibrated (BATT_AMP_PERVLT=40). ASK FRED: clamp-meter / calibrated current | logs 31-34 BAT/ESC msgs; report.md:19 (current never used) | max_current 40 A: ASK FRED |
| Electrical model | R loop 0.1542 ohm (prior-anchored R=0.16+-0.04), Rb 0.0434 ohm | report.md:31,37; :25 | model current scale rests on R prior |
| Battery sag | 25.30 V (prop off) / 24.82 V (prop on) OCV; 24.8 -> 23.2 V over log 34 steps (1500 -> 1800); min 23.18 V (34), 22.59 V (32); 3300 mAh 6S | log-index.md:26; report.md:44; logs 32/34 | none (new) |
| Thrust vs throttle/rpm | NONE: no load cell data. Prop dia 0.127 m (5"), kq=1.455e-8 N m s^2; rpm 15,650 @1500, 22,480 @1800 | report.md:Fit cmd,40; log-index.md:26 | ASK FRED (thrust stand, prop model) |
| Speed bandwidth (duty->rpm, prop on) | f3dB ~11 Hz, tau 14.5 ms; log 32 sweep tau ~23.5 ms, ~6.5 ms delay | report.md:138-155; log-index.md:25 | 2000 Hz torque BW: NOT measured; speed loop is ~11 Hz, torque/current BW still ASK FRED |
| Motor mass | ~35 g (8-10 g copper) | notes/sim/am32-ramp-study.md:437 | none |
| Vehicle/airframe/battery mass, planned Hakea mass | not recorded anywhere in notes/ (grep mass/weight/AUW/grams). ASK FRED | notes/ | 0.79 kg: ASK FRED |
| Motor temperature | no motor sensor; ESC temp only: 25 -> 59 C (log 34), 57 C (33), 53 C (32), 36 C (31); quad ESC max 69 C. No derating evidence | log-index.md:25-26; ESC.Temp logs 31-34 | Rth 1.5 K/W: ASK FRED (needs motor winding temp) |
| Nov 6/14 quad flights | only .tlog files present (no dataflash BIN); not analysed | logs/2026-11-06, 2026-11-14 | ASK FRED |
