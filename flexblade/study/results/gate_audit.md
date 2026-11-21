# Gate audit: cr_pl (control_quality_cal) mechanism-mode gates

## Gates and what each one prevents
- `cqc_phase` / `cqc_gain` (robust.control_quality_cal, residual ≤15° / 25 % after per-unit calibration at every (throttle, build, J) node): the calibration stops being valid across rpm/temp/creep/J, so the cyclic command lands in the wrong direction or at the wrong size.
- `cqc_authority` (|G_build| ≥ 0.5 |G_nom|): a unit without enough cyclic authority (it needs more pulse than the ESC/motor can give).
- `cqc_mech_margin` (worst build corner, mech mode \|f/1P − 1\| ≥ 0.03): a raw frequency band around 1P. It is a proxy for response amplification and phase sensitivity near resonance.
- `cqc_mech_side` (a build corner's mech mode is on the other side of 1P from nominal): a proxy for loss of the control sign, i.e. a phase jump of about 90° or more between units.
- `resonance` (run_study.resonance_margin, RES_BAND 0.15, damping-aware via margin_1p_eff): amplification and fatigue of *other* modes near 1P (and of hub modes near 2P) from aero/imbalance 1P/2P excitation. **Under control_quality_cal the mech mode is exempt at 1P** (`skip_1p=im`). Only its 2P check (hub modes) remains. So RES_BAND does not double-count the mech-mode 1P risk.
- `mech_band` (MECH_MARGIN 0.075, nominal): this belongs to a separate `--criterion mech_band` and is **not applied in the cr_pl / CAL path**. It is an older, superseded version of cqc_mech_margin.
- Goodman fatigue / hard checks (nominal points, pulse-scaled) and ground resonance (FLEXBLADE_GROUND_RES=1): strip/spar fatigue, and coupling of the hub-participating mech mode with the airframe arm modes.

## Physics (o_0.9_-1, single modes eval at T1, design J)
- The mech mode is "pitch antisym+hub" at 0.86/rev, ζ = 0.041 (energy + aero; hub participation 0.48). With ζ = 0.041, the 1P amplification is 6.4 at r = 0.93, 10.1 at 0.97 and 12.2 at r = 1. The phase is 30° at 0.93 and 90° at 1.0.
- Damping bounds the gain at exactly 1P to 1/(2ζ) ≈ 12, only about 1.2–1.9× the nominal value. The saved nodes agree: corner p0 sits at 0.985–1.004/rev with authority 0.90–1.27, so the gain did not blow up.
- The damping-aware margin that RES_BAND uses for every other mode, margin_1p_eff(1.0, 0.041), is about 0.044. That exceeds 0.03 even at exact resonance, so the raw 0.03 band is stricter than the project's own damping-aware rule.
- Side flip: per-unit calibration measures each unit's phase, so a corner on the other side only has a different calibrated phase (p0 −9° vs nominal +43°). What can actually go wrong is sensitivity of phase/gain to rpm, temperature and creep in that unit. CAL already evaluates that *at the corner node, with the corner's own calibration* (p0 phase_res 5.7–9.2°). Per-rev drift with rpm is small (centrifugal stiffening tracks Ω), and the continuous-rpm residual captures it anyway.
- Fatigue: the FC normalises gain per unit, so the strip deflection per unit of cyclic output does not depend on the amplification. Higher authority means less pulse, not more stress.
- Conclusion: cqc_mech_margin and cqc_mech_side duplicate CAL (phase/gain residual) plus authority for the same near-1P sensitivity risk. RES_BAND, Goodman and ground resonance are physically independent.

## Minimal non-redundant set (opt-in: `SA_GATES=minimal` in study/i34tools/sa.py; the default is unchanged)
Keep cqc_phase, cqc_gain, cqc_authority, resonance (other modes, plus mech-mode 2P), the Goodman/hard checks and ground resonance. Drop cqc_mech_margin and cqc_mech_side. cr_pl itself is unchanged.

Risks of dropping them:
1. ζ is a model output. If the real pitch-mode ζ is at or below 0.015, the 1P gain (> 33) and phase slope (1/ζ rad per unit r) get 3× worse. Bench-measure ζ, or add a ζ_mech ≥ 0.02 gate.
2. CAL samples only 4 build corners, and the response is steepest between the samples.
3. Fatigue is checked at nominal points only, so a corner with authority < 1 needs more pulse.
4. A hub-coupled mode near 1P is the ground-resonance candidate. Keep that check on.

## Re-gated saved outputs (SUM lines in the session scratchpad; no new solves)
| design | cr_pl | default feas | minimal feas | worst mech margin | still blocked by |
|---|---|---|---|---|---|
| o_0.9_-1 | 0.128 | F | **T** | 0.004 FLIP | - |
| i36 a0.8 | 0.126 | F | **T** | 0.001 FLIP | - |
| v8c13 | 0.121 | F | **T** | 0.008 FLIP | - |
| crm_sa | 0.110 | F | F | 0.038 | cqc_gain (49 %) |
| i36 a0.6 | 0.108 | F | **T** | 0.004 | - |
| o_0.9_1 | 0.104 | F | **T** | 0.024 FLIP | - |
| o_1.3_2 | 0.101 | F | F | 0.037 FLIP | resonance (pitch sym 1.13/rev) |
| sa_a / i36 a0.4 / o_0.9_3 | 0.092 / 0.090 / 0.086 | F | **T** | 0.034F / 0.022 / 0.056F | - |
| sa_c / sa_d | 0.076 / 0.080 | F | F | FLIP | harmonic_jitter |

The best feasible design moves from v8 (0.088) to o_0.9_-1 (0.128, +45 %). That design's corner p0 sits at 1.004/rev, so risk 1 (ζ) is the deciding one: measure ζ before trusting it.
