# Miniaturising the steelx strap rotor to 2–3" (feasibility)

Leads: 7" v6 (t 0.15 mm, w 5.8 mm, L 12.5 mm, strand 1.5 mm) and 10" v8 (t 0.10, w 9.0, L 25.1, strand 2.6).
s = D/7". Mass law as in presets (m ∝ D^1.5), so this is still a single-rotor vehicle.

## 1. Scaling laws (preset trims from `run_study.base_design`, new mini motors)
| D | m kg | rpm | f_1P Hz | tip m/s | Q_hover mNm | P_shaft W | eRPM (7 pp) | T/2 ms |
|---|---|---|---|---|---|---|---|---|
| 7" | 0.99 | 12.5k | 208 | 116 | 128 | 167 | 88k | 2.4 |
| 3" | 0.28 | 39.8k | 664 | 159 | 16.9 | 70 | 279k | 0.75 |
| 2.5" | 0.21 | 52.4k | 873 | 174 | 10.9 | 60 | 367k | 0.57 |
| 2" | 0.15 | 79.3k | 1321 | 211 (M 0.62) | 6.1 | 50 | 555k | 0.38 |
- rpm ∝ D^-1.37 and tip speed ∝ D^-0.37. These follow from the disc loading m/D² ∝ D^-0.5, which rises as D shrinks. Q ∝ D^2.8,
  but P_shaft only ∝ D^1.0, so a mini rotor needs an 18xx stator sized for POWER, not a "3-inch motor".
- CF on the blade ∝ m_b Ω² R ∝ D^2.3. The strap stress (CF/A_strand, with A ∝ D²) is roughly size-invariant.
- Strip similarity: the mechanism mode / 1P ratio stays the same if k_strip/(I Ω²) is constant. With k ∝ E w t³/L
  (w, L ∝ D) and I Ω² ∝ D^5·D^-2.7, this needs t ∝ D^1.1. Bending strain ε ≈ t θ/L is then also invariant, so
  fatigue does not change. The CF share of the pitch stiffness is pendulum-like (k_cf/I Ω² ≈ 0.93) and size-free.
  From the 7" lead: t ≈ 0.064 / 0.054 / 0.043 mm at 3 / 2.5 / 2", with w ≈ 2.5 / 2.1 / 1.7 mm and L ≈ 5.4 / 4.5 / 3.6 mm.
  From the 10" lead: t ≈ 0.030 / 0.025 / 0.020 mm. Commercial 1095/301 shim down to 0.025 mm (0.0125 mm is foil)
  covers this. **Shim stock is not the limit.**
- Tolerances do NOT scale: ±0.5° slot/skew and ±0.05 mm print features are fixed. At 3" a ±0.05 mm error in
  free length is ±1 % of L (±0.4 % at 7"), and the corner-margin spread of the mechanism mode grows ∝ 1/D.
- Motor current bandwidth: L/R = 27–67 µs (estimate_L), so the 1P lag is atan(ω_1P L/R) ≈ 12–16°. Calibratable,
  not binding.
- Governor: bw_max ≈ 0.13/(lat + L/R + T/2). At lat 0.5 ms this is 99 / 116 / 143 Hz (3 / 2.5 / 2"); at 2 ms it is
  46 / 49 / 54 Hz. That is above the 7" figure (27–40 Hz), so the governor **is not binding**.
- ESC: commutation rate = 6·pp·f_1P = 28 / 37 / 55 kHz. At 48 kHz PWM that leaves 1.7 / 1.3 / 0.9 PWM periods per
  commutation step and 72 / 55 / 36 PWM periods per rev for shaping the 1P pulse. The FC loop (8 kHz DShot) gives
  only 12 / 9 / 6 updates per rev, and 125 µs of latency is 30° / 39° / 59° of rotor phase.
- **Binding first:** the ESC rate. The commutation step falls to about one PWM period, and the FC-commanded pulse
  phase jitter becomes tens of degrees. This binds before the strips, the governor or the motor L/R.

## 2. Quick sims
3" steelx DE (Mac, 10 gen, 594 evals): best infeasible — viol flexure_inside_hub 0.05, creep_strain 0.01, cr 0.002. Hub geometry, not physics, binds first at 3" → needs a mini hub preset (flexure outboard of a smaller hub) before re-running. 2.5" DE did not run.

## 3. Hardware the mini version would need (requirements only)
- Motor: 1804-class at 4500 kV (3") or 6000 kV (2.5") on 4S, or 1404 at 7500 kV (2"). A real 1103/1104 is
  thermally far too small for a 0.2–0.3 kg single rotor (50–70 W shaft). Prefer 9N12P (pp = 6) over 12N14P to lower
  the eRPM by 14 %.
- ESC: a 32-bit part (AM32-class) that holds sensorless commutation at 280k (3") / 370k (2.5") / 560k (2") eRPM
  with ≥ 48 kHz PWM. 2" is beyond the commonly reported operating range of hobby ESCs.
- The 1P torque pulse must be generated ON THE ESC, phase-locked to the rotor angle (commutation count + timer
  interpolation). Commutation granularity is 360/(6·pp) = 8.6° mech, so interpolation is needed for phase error
  < 3° at 600–1300 Hz. The FC should send only amplitude and phase (DShot/extended telemetry or a bidirectional
  channel) at a few hundred Hz. DShot itself cannot carry a per-rev waveform at 1P ≥ 600 Hz.
- Torque (current) control at ≥ 10× f_1P, i.e. a 7–13 kHz current loop, or a calibrated duty feed-forward. The
  duty slew limit (AM32 max_ramp) must allow a full-depth pulse within ~¼ rev (0.4 / 0.3 / 0.2 ms).
- A rev-averaged rpm governor (1-rev window = 7 eRPM periods), unchanged from drive_rev_avg.md.

## 4. Verdict
VERDICT_PLACEHOLDER
