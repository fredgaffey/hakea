# 5" lead 5in_v8s_2307_m245_301: is the fixed 0.15 1P margin the right criterion?
Source: `run_study.resonance_margin` = max(|f/kP-1|, `stability.margin_1p_eff`(p, zeta)); zeta is the eigen damping (aero+structural; structural-only energy zeta ~0.03 for these modes). `robust.combine` takes the min over T. Model run: J_hub 0.91e-6, T0.7/1.0/1.25, nominal vs flexure.thickness x0.95 (other tolerances not applied; the 0.158 worst build also had l x1.037, h x0.974).
Closest mode above 1P (SDOF H at 1P, r = 1/p). All zeta 0.029-0.031; 1P phase lag deg:
| case | p /rev | zeta | abs H | phase | res_margin |
|---|---|---|---|---|---|
| nom T0.7 / thin | 1.77 / 1.59 | .031 / .030 | 1.47 / 1.65 | 3.0 / 3.6 | 0.39 / 0.42 (limited by pitch antisym 0.82/rev) |
| nom T1.0 / thin | 1.44 / 1.30 | .030 / .029 | 1.93 / 2.43 | 4.6 / 6.2 | 0.44 / 0.30 |
| nom T1.25 / thin | 1.27 / 1.19 | .030 / .029 | 2.61 / 3.37 | 7.0 / 9.6 | 0.27 / 0.19 |
| worst build (p 1.15) | 1.15 | .03 | 4.01 | 12.1 | 0.158 |
The mode at 1.27 reports as "flap antisym" in this evaluation (thin T1.25: "pitch sym" 1.19). pitch-sym of the strip is at 0.55/rev, below 1P. Below-1P modes: pitch antisym 0.79-0.82/rev, zeta .014-.019, abs H 1.7-2.0, phase ~175 deg.
Cyclic response (sa.py quantities, same runs) nominal -> thin: phase 96.7/101.1/104.6 -> 97.4/102.3/106.2 deg (+0.6/+1.2/+1.6); gain 0.1698/0.1606/0.1603 -> 0.1643/0.1595/0.1633 (-3.2 / -0.7 / +1.9 %). These are inside the sa.py gates (phase_res 0.9-1.8 deg, gain_res 0.034-0.074) and the CQC limits (15 deg / 25 %).
Sensitivity near 1.19-1.27/rev (zeta .03): d|H|/|H| = -3 to -4.4 % and dphi = -0.3 to -0.5 deg per +1 % in p. A +-5 % p drift (more than the hinge-radius spread, +-2.7-4.7 %) costs about 20 % gain and 2.5 deg.
Damping: zeta 0.03 changes |H| by under 2 % relative to undamped at margin 0.15 (4.01 vs 4.10), so margin_1p_eff gives no relief; the mode is lightly damped and the criterion is effectively the undamped amplification cap |H| <= 4.10.
## Verdict
1. The 0.15 margin is a proxy for |H| <= 4.10 (undamped amplification at 1.15/rev); the worst build has |H| 4.01 and 12 deg lag. Neither is large against the measured nominal-to-thin change, +1.6 deg and +-3 % gain at T1.25.
2. It is mostly redundant here: the sa.py phase/gain checks already see the 1P gain/phase shift of the whole structure, and they pass with a large margin (1.6 deg vs 15; 3 % vs 25 %). It adds protection only for what those do not model: a drift that moves p below ~1.1 (|H| rises steeply, 6 at p 1.10) between calibrations, and fatigue/harmonic loads, which have their own checks.
3. It is not over-strict by much: with zeta 0.03 damping buys nothing, so relaxing to p>=1.10 means |H| ~6. Proposed: replace the fixed band by |H|(1P) <= 4.5 at nominal and <= 6 at the worst build, plus phase sensitivity |dphi/dlnp| <= 1 deg per % at the calibrated point (now 0.3-0.5). That passes the current worst build, p>=1.14, and keeps it on the gain/phase criteria. Damping credit needs zeta >= 0.08, which is not available passively (damped_resonance.md).
