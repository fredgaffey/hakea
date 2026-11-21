# CF-invariant mechanism mode? (v8 lead, T0.7/1.0/1.4; `study/cf_invariant_mode.py`, raw `cf_invariant_mode.json`)
Method: trim + 1P harmonic per throttle (pulse margin 0.95, pre-lag solved at nominal mass), mechanism mode = largest 1P participation (as run_study). At the frozen trim, the same mode (MAC-matched) is recomputed with lin.Kgeo off (elastic only), and with Omega scaled ×1.03^±1 (Kgeo·k², G·k).
f_cf = 1 − ω_el²/ω² is the CF/geometric share of the mode stiffness. drift = d ln(mpr)/d ln rpm between the T0.7 and T1.4 trims (rpm ×1.41).

| variant | mpr T0.7/1.0/1.4 | f_cf T1.0 | drift (trim) | drift (frozen, T1.0) | ctrl M N m T0.7/1.0/1.4 | phase deg | rpm T1.4 |
|---|---|---|---|---|---|---|---|
| v8 | 0.958/0.948/0.942 | 0.92 | −0.049 | −0.15 | 0.37/0.26/0.19 | 99–111 | 9145 |
| no z balance masses | 0.998/0.987/0.981 | 0.93 | −0.047 | −0.15 | 0.86/0.56/0.37 | 83–106 | 9944 |
| chordwise ±10 mm, 1 g, r 0.2 | 1.003/0.992/0.965 | 0.95 | −0.100 (MAC 0.74 at T1.4: veering) | −0.14 | 1.03/0.63/0.38 | 70–106 | 11072 |
| chordwise ±10 mm, 1 g, r 0.35 | 1.031/1.021/1.007 | 0.95 | −0.062 | −0.12 | 1.39/0.89/0.54 | 36–89 | 10805 |
| strips 0.8 t | 0.956/0.950/0.942 | 0.95 | −0.041 | −0.10 | 0.30/0.24/0.18 | 100–113 | 9121 |
| strap length ×2 | 0.905/0.895/0.888 | 0.92 | −0.054 | −0.17 | 0.26/0.19/0.14 | 106–118 | 9134 |
| 0.8 t + chordwise r 0.2 | 0.943/1.018/0.548 | 0.97 | collapses | — | 0.42/0.72/0 | — | 14004 (pitch flattens) |

**The hypothesis is wrong for v8.** The pitch mode is already CF-dominated: 88–97 % of ω² comes from Kgeo (propeller moment, CF tension through the skew, spin softening), and the elastic strips give only ~8 %.
So mode/rev is already nearly rpm-invariant: 0.958 → 0.942 over T0.7–1.4 (Δ −0.016, d ln mpr/d ln rpm ≈ −0.05). The ±5 % build-tolerance pattern moves k_e by ~±15 %, which shifts mpr by only ~±0.6 %.
The trim drift is about 1/3 of the frozen-state drift (−0.15) because the trim state also changes (tension and coning rise with thrust), and the two effects partly cancel.
Raising f_cf further does little: thin strips take f_cf 0.92 → 0.95 and the drift −0.049 → −0.041. The drift floor (~−0.04) is set by the trim/state change, not by k_e.

**What actually widens the band is proximity to 1P, not drift.** Gain levers (no z masses, chordwise mass) move mpr to 0.98–1.03, and near resonance the same 0.016–0.035 mpr drift swings phase 23–53° and the moment by ×2.3–2.6.
The ×2 fall of the control moment over throttle for v8 is only about half explained by the mode: the 1/|1−mpr²| amplification falls by ×0.73. The rest is forcing/damping scaling.
CF levers ranked by k_prop ∝ Ω² Σ m(y_chord² − z_normal²):
- chordwise mass at larger r. This gives the most gain (×3.7 at T0.7) but raises rpm 18–20 % (the propeller moment flattens pitch).
- removing the normal masses. Gain ×2.3, rpm +9 %.
- thin strips. Little effect.
- a longer strap is not a CF lever: it detunes (mpr −0.05) and costs 28 % gain.
Combining thin strips with chordwise mass overshoots k_prop/k_eff: the blade flattens and the trim collapses at T1.4 (rpm 14000).

**Recipe for the next DE.** Don't spend the objective on drift. Use it as a cheap guard from the nominal T0.7/T1.4 nodes: penalty 10·max(0, |d ln mpr/d ln rpm| − 0.07). Also add these terms and variables:
- one-sided mode placement: mpr(T0.7) ≤ 0.975, or mpr(T1.4) ≥ 1.03. Drift is negative, so a mode below 1P is closest to resonance at T0.7, and a mode above 1P is closest at T1.4. Reject designs where 1−mpr changes sign over the range.
- mech-mode MAC(T0.7, T1.4) ≥ 0.9 (no veering).
- pitch_cf_ratio within limit, and rpm(T1.4) ≤ 1.1 × v8 (≤ 10 000).
- variables: balance pair mass 0–1.5 g, r_frac 0.15–0.35, chord_offset 0–12 mm, normal_offset 0–10 mm; strip thickness 0.085–0.10 mm; strap_length 6–9 mm (detuning trim only).
