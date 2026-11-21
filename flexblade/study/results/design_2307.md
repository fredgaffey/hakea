# Flexure-cyclic rotor for Hakea's 2307 1300KV (STATUS: DE running, design not final)

## Motor preset `2307_1300kv_hakea` (flexblade/motor.py; source notes/motorid/report.md)
| field | value | source |
|---|---|---|
| kv / voltage_gain | 1300 / 1.032 | nominal Kv (Kt = Ke = 0.00735 N m/A); mean fitted g(rpm) -> effective Kv ~1356 |
| R | 0.1542 ohm | fit (loop 2R_ph + 2Rds_on, 25 C); absolute scale set by the R = 0.16 +- 0.04 prior |
| L | 30 uH | fixed in the fit (not identified) |
| J_rotor | 3.335e-6 kg m^2 | fit J_m |
| b / T0 | 7.425e-6 N m s / 1.32e-6 N m | fit (T0 at lower bound, not identifiable) |
| Rb | 0.0434 ohm | fit |
| supply_V | 24 V | 6S, open-circuit 23.4-25.3 V over the runs |
| max_current | 40 A | GUESS (2307 datasheet class) |
| torque_bandwidth_hz / Rth | 2000 Hz / 1.5 K/W | GUESS (library defaults) |
| duty_slew_max | 160 /s | ESC eeprom ramp 16/2000 per 20 kHz tick. The study assumes 2550 /s (AM32 max_ramp 255), so the ESC must be reflashed. |

## Prop size (2307 on the 6S preset mass law, 7in v6 scaled, trim at T1.0 / T1.4)
- 5": 21.4 krpm, duty 0.72 at hover and 0.85 at T1.4. There is little voltage headroom for the 1P pulse.
- 6": 16.2 krpm, 11 A, duty 0.58, thermal utilisation 0.64 (lowest of the three), 1P capability 2.6x hover Q (current-limited). **Primary.**
- 7": 13.0 krpm, 16.5 A, thermal utilisation 0.88 at hover, capability 1.2x (thermal-limited). **Secondary.**
- At T1.4 the steady thermal utilisation is above 1 for every size. The 2307 cannot hold 1.4x hover continuously.
- Mass: the presets mass law gives 0.79 kg at 6". Hakea's real AUW is not in the notes. Re-trim with the measured mass.

## Design
DE: `run_study.py --sizes 6,7 --concepts tt_strap_steelx --motor 2307_1300kv_hakea --criterion mech_band
--mech-margin 0.07 --goodman --objective pulse_limited --robust 0.7,1.0,1.4 --ground-resonance --maxiter 15
--popmult 6 --j-hub "6:0.58e-6,1.57e-6,3.46e-6;7:0.92e-6,2.5e-6,5.49e-6" --init-designs/--base-design study/seeds2307/*`.
It runs on the Mac in /tmp/flexblade-2307 (own venv; log de2307.log; designs/<D>in_tt_strap_steelx.json on finish).
The 6" J_hub set is the 7" set x (6/7)^3. For sa.py, use `SA_JHUB=0.58e-6,1.57e-6,3.46e-6`, because its default JS is the 7" set.
TODO once the DE finishes: try precone 1.0-1.3, run sa.py, then save designs/<D>in_tt_strap_steelx_2307.json and fill in the dimensions, cr_pl and binding.

## Arm / mount requirements (scaled from the 7" v6 work; 1P = rpm/60)
- 6" 1P: 270 Hz at T1.0, 230-320 Hz over T0.7-1.4. 7" with the 2307: 217 Hz at T1.0.
- The arm in-plane mode must clear the 1P band. 7" spec was >= 200 Hz with a governor (system_check.md). Scaled by 7/6, the 6" needs >= ~235 Hz, and higher is better.
- Vertical (arm_vertical.md): without precone the 1P vertical force is about 7x higher. Use precone ~1.3 and keep f_z out of 0.87-1.22 x 1P.
- Balance to < 1 g mm. The hub mode is about 0.68 x 1P and amplifies imbalance (system_check.md, 7").

## DE debug
- Symptom: every DE candidate fails motor_torque_1p=6.91 (+ motor_thermal 0.61). Not a code bug: --motor IS applied (base and seed evaluate with 2307_1300kv_hakea, 24 V, J_hub 2e-6 swapped per J set).
- Cause: the `--robust 0.7,1.0,1.4` T1.4 point. At 1.4x hover thrust the 6in rotor's mean torque already exceeds the motor current limit ("limit: none (mean over limit)"): 1P allowable = 0, utilisation = inf, clipped to 6.91 by the scorer. Independent of candidate, so constant.
- Base/seed util elsewhere is fine: T0.7 0.92, T1.0 0.91 (limit: current); T1.15 0.905, T1.25 0.902 (limit: thermal). motor_thermal at T1.4 is 1.04-1.08 (>1) too.
- J_hub (0.58e-6..3.46e-6), torque_mod_frac (2.2-2.37) and vehicle_mass 0.79 do not change the result: identical 1P util across the three J_hub values.
- Fix = CLI: lower the top thrust factor, e.g. `--robust 0.7,1.0,1.25` (T1.25 thermal util 0.98, T1.4 infeasible by physics), or give the 6in a lower --vehicle-mass / bigger motor if 1.4x margin is truly required.
