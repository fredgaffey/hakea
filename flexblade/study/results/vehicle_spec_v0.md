# Hakea vehicle spec v0: 2307 1300KV, H743-Slim V3, 6S (single rotor vs coax vs tandem)
Script `study/vehicle_v0.py` (raw `results/vehicle_spec_v0.json`) runs lever_2307 `ev` at the per-rotor mass (T0.7/1.0/1.25, J_hub 1.57e-6). Hover power comes from `vehicle.electrical_power` (ctrl_mod 0.3). Hover time = 22.2 V x Ah x 0.8 / P.
Every mass is est. unless a source is given. LiPo masses use vehicle.py CELLS (150 Wh/kg cells +10 % +6 g leads), about 0.17 g/mAh at 6S: 450/650/850/1100 mAh = 79/112/144/185 g. That matches typical high-C 6S packs; Fred should weigh his.

## 1. Mass budget, single rotor, 6" (g)
| item | minimal | normal | camera | source |
|---|---|---|---|---|
| motor 2307 1300KV | 35 | 35 | 35 | ~35 g, notes/sim/am32-ramp-study.md:437 |
| rotor: blades 4.1 + steel strips 0.3 + hub 8.6 + nut 2 | 15 | 15 | 15 | est., vehicle.rotor_mass on designs/6in_tt_strap_steelx_2307 |
| FC H743-Slim V3 / receiver (ELRS nano) / MT6835 encoder + magnet | 7/2/3 | 7/2/3 | 7/2/3 | est. |
| ESC | 7 (1-ch AM32) | 15 (Reaper 65A 4-in-1) | 15 | est.; Fred owns the Reaper (log-index.md:7) |
| wiring + XT30 / body (PA-CF motor mount + stack) / screws, standoffs | 8/15/5 | 8/15/5 | 8/15/5 | est. |
| vane ring (PETG, 6 vanes) + yaw servo and linkage | 15+5 | 15+5 | 15+5 | est., sized in 3(a) |
| legs (4 x 3 mm CF tube + printed feet) / camera + VTX + antenna | 6/- | 8/- | 8/14 | est. |
| 6S LiPo | 79 (450) | 112 (650) | 144 (850) | est., 0.17 g/mAh |
| **AUW** | **202** | **245** | **291** | the 0.79 kg mass-law guess is 3x too high |
| hover rpm / electrical power / hover time | 8.4k / 35 W / 13.7 min | 9.1k / 44 W / 15.9 min | 9.7k / 54 W / 16.9 min | model, T1.0 |
5" at 245 g (scaled design): 13.1k rpm, 57 W, 12.2 min on 650 mAh. With camera + 850 mAh (~290 g) it is about 74 W and 12 min (est., P ~ m^1.5).

## 2. Finding: at the real mass the 6" flexure is out of tune, and the requirement grows
- designs/6in_tt_strap_steelx_2307 was tuned at 0.79 kg (15k rpm, pitch mode 0.83/rev). At 0.15-0.29 kg per rotor, hover is 7.5-9.7k rpm and the pitch mode sits at 0.96-1.21/rev.
  Every 6" point fails resonance and aeroelastic stability. Near 1.0/rev (6", 245 g, T1.0) cr_pl collapses to 0.009 (fatigue-bound). Off-tune points at 1.08-1.13/rev give 0.070-0.078, an indication of what a retune could reach.
  The 5" scaled design runs 13.1k rpm and sits at 0.88-0.97/rev: T1.25 is feasible, and T1.0 only carries the resonance flag.
- Mechanism gain M/dQ at the real mass is 0.03-0.05 at 6" and 0.05-0.09 at 5" (0.09-0.10 at 0.79 kg), so authority now comes from motor headroom over a light vehicle.
  The pulse amplitude at the limit is 35-37 A at 6" (current-bound, 40 A GUESS) and 21-24 A at 5" (fatigue-bound). dQ/Q is 5-13, so torque reverses every rev.
- Requirement: cr_req = e/R (1 mm, trimmed) + (1.0-1.6) mu, with mu = 2 m/s / (Omega R). A lighter rotor has a slower tip, so the gust term grows:
  6" single at 245 g, Omega R 73 m/s (not 120): 0.013 + 0.028-0.044 = **0.041-0.057**.
  5" single, Omega R 87 m/s, e/R 0.016: **0.039-0.053**.
  6" twin, per rotor 150 g, Omega R 60 m/s: **0.046-0.066**.

## 3. Configuration trade (normal build, 650 mAh, 6" unless noted; cr from the CURRENT flexure, not retuned)
| | (a) single + vanes | (b) coax, 2x 2307 | (c) tandem, 2x 2307 |
|---|---|---|---|
| AUW / per rotor | 245 g / 245 g | 300 g / 150 g | 298 g / 149 g |
| hover P / time | 44 W / 15.9 min (5": 57 W / 12.2 min) | 56 W / 12.3 min (x1.15 coax loss, est.) | 49 W / 14.2 min |
| cr delivered, T0.7/1.0/1.25 | 6": 0.075/0.009/0.058, resonance. **5": 0.070/0.080/0.076** | per rotor 0/0.077/0.072 (T0.7 pulse infeasible). A: 0.925x = 0.071. B: 0.5x = 0.039 | roll 0.077 (both rotors), T0.7 infeasible. Pitch from differential thrust ~0.27 |
| cr required | 0.041-0.057 (5": 0.039-0.053) | 0.046-0.066 | roll 0.046-0.066; pitch met easily |
| margin | 5": x1.5-2.1 hover, x1.3-1.8 robust min. 6": retune, est. x1.1-1.7 | A x1.1-1.5 (robust 0). B x0.6-0.8: FAILS | roll x1.2-1.7 (robust 0) |
| yaw | 2 servo-driven vanes, about +-30 % Q (est.) | differential torque, 1-5x hover Q left | differential lateral cyclic, ~8x the trim need, shared with roll |
| parts | 1 motor, 1 ESC ch, 1 encoder, 1 flexure rotor, vane ring, 1 micro servo | 2 motors, 2 ch, 2 encoders, 2 mirrored flexure rotors, cage | 2 motors, 2 ch, 2 encoders, 2 mirrored flexure rotors, stiff boom |
| build for Fred | easy: one PETG body + vane ring, PA-CF hub/mount, glued CF legs | hard | medium-hard |

**(a)** The vanes have to straighten about 21 deg of swirl at r = 0.6R. The tangential force is Q/r_v = 0.031/0.046 = 0.68 N; the slipstream is at ~1.5 v_i = 11 m/s (q 74 Pa).
At CL 0.8 that needs 0.0115 m², i.e. 6 vanes of ~40 mm chord x 45 mm span (5": 0.008 m², 6 x 36 x 37 mm). In 0.8 mm PETG that is 15-20 g with the ring.
Fixed vanes cannot carry yaw. Vane torque scales with T but rotor Q scales with T^1.5, so the mismatch runs -16/+12 % over T0.7-1.25 before any gust.
Nothing else gives steady yaw independent of thrust: changing mean motor torque yaws the body only transiently, and it moves thrust. So: 4 fixed vanes trimmed for hover plus 2 all-moving vanes on one 3-4 g servo (an H743 PWM output).
**(b)** Coax: motors back-to-back on a printed cage around the battery (CG between the rotors), with a >= 27 mm tip-plane gap (A, design_crm2204.md); the legs must reach outside R.
B (upper flexure, rigid lower) keeps all its yaw (0.27 N m) but halves vehicle cr, so it FAILS here. A works at x1.1-1.5, with yaw-to-collective coupling on both rotors, two mirrored rotors, two phase calibrations and the T0.7 pulse infeasible.
**(c)** Tandem: shafts 162 mm apart (2R + 10 mm). Pitch from differential thrust (large); roll from lateral cyclic on both rotors.
Yaw comes from differential lateral cyclic: about 3 deg tilt at full pulse gives ~0.012 N m, against 0.0016 N m of torque mismatch per 2 mm of fore-aft CG (est., cr ~0.025/deg).
The boom in-plane mode must clear the 110-140 Hz 1P band at both ends, and ArduPilot needs a custom mixer.

## 4. Recommendation: (a) single rotor + 6 vanes (2 servo-driven), 5" rotor, 6S 650 mAh, AUW ~240-245 g (est.), hover ~12 min
- Authority: model cr_pl 0.070-0.080 over T0.7-1.25 against the 0.039-0.053 required, i.e. **x1.3-1.8 robust, x1.5-2.1 at hover**. It is the only option evaluated that is near-feasible with an existing design.
  It matches the README 5" and the measured 5" motor data. 1P is 190-240 Hz.
- Before printing, run a DE retune of the 5" at `--vehicle-mass 0.245 --robust 0.7,1.0,1.25` to pull the pitch mode below ~0.87/rev at T0.7-1.0, and confirm cr_pl >= 0.06.
  The 6" (+30 % hover time) is worth it only if its own retune at 0.245 kg reaches >= 0.06.
- Risk 1, mass/tune: every mass here is est. The flexure tune moves with AUW (0.79 to 0.25 kg moved the pitch mode by +0.2/rev). Fred weighs parts, hang-tests the CG to 1 mm, then retune.
- Risk 2, pulse current: 21-24 A amplitude with torque reversal at 190-240 Hz. The 40 A limit is a guess. Battery ripple is ~1.1 V on 650 mAh (0.6 duty x 24 A x 0.078 ohm, est.); full-pulse copper loss is ~46 W (transient only). Measure with a clamp meter.
- Risk 3, vanes/structure: vane size and servo yaw authority are momentum-theory estimates (thrust loss a few %). The mount, vane ring and legs (PETG/PA-CF) must keep the in-plane mode out of 190-240 Hz; balance < 1 g mm.
