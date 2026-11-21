# Ideal -> realised gap on a common footing (current model, pulse-limited)

Scoring: `pulse_limited.robust_eval` = min over throttle {0.7,1.0,1.4} x J_hub {0.92,2.5,5.49}e-6, cr_pl.
Scripts: `ideal_gap.py` (scores), `ideal_gap_k.py` (K6 decomposition), `ideal_gap_swap.py`/`ideal_gap_swap2.py`
(swaps), `ideal_gap_fam.py` (families). Raw: `pulse_limited.json[ideal_gap_score]`, `ideal_gap_*.json`.
Ideal cards: `campaign/ideal_seeds/robust_seed{1,2,3}` (robust objective) and `seed1` (old objective).
Best robust card per size on cr_pl: 7in robust_seed2, 10in robust_seed2 (the cards' own cr ranked 10in seed1 first).

## 1. Re-score
| design | own cr (card) | cr_pl now | feasible now | binding at worst point |
|---|---|---|---|---|
| 7in ideal robust_seed2 | 0.181 | 0.223 | no (resonance, flap antisym 1.11/rev, margin 0.114) | motor_torque_1p, T1.4 J5.49 |
| 7in v6 steelx | - | 0.065 | yes | motor_torque_1p (fatigue_stress at low J) |
| 10in ideal robust_seed2 | 0.155 | 0.426 | no (resonance, pitch antisym+hub 0.96/rev, 0.081) | motor_torque_1p / fatigue_strain |
| 10in steelx v4 | - | 0.095 | no (resonance 0.59 at T0.7/1.0) | motor_torque_1p at T1.4; fatigue_stress at low throttle |

Notes: the ideal cards are NOT feasible on the current resonance check, so their cr_pl is an upper bound
(the `geom:members` flag is an artefact of the ideal topology having no members; ignored). The ideal
cards carry no real fitting mass (massless hub/root); `IdealFlexure.mass` + `lag_inertia` is the hook and
was used in the swaps below (steelx root fitting 1.18 g, Izz 1.06e-7 kg m^2 about hinge z).
At the worst point (T1.4, J5.49, both pulse-limited by motor_torque_1p, pulse_scale ~1.2) the 7in ideal
cr is 0.180 vs v6 0.053: 3.4x.

## 2. 7in gap decomposition (swap one property into the ideal robust_seed2; baseline 0.2235)
Hinge properties, ideal vs v6 effective (frame H, at hinge point):
k_hinge 0.0145 vs 0.0197 (1.4x); k_cross 644 vs 21.5 (30x lower in v6); k_flap 770 vs 21.9 (35x lower);
skew 67.3 vs 59.9-60.6 deg (design vs softest axis); translations kx/ky/kz 0.3M/8.8M/3.0M vs 48M/4.9M/7.0M N/m
(v6 stiffer axially, 160x); lag-pitch coupling k_lp/k_lag 0.42 vs 0.64 (follows from skew); R 16.7 vs 14.3 mm.

| swap | cr_pl | delta |
|---|---|---|
| k_hinge -> v6 | 0.225 | +0.002 |
| k_cross -> v6 (30x softer) | 0.256 | +0.033 |
| k_flap -> v6 (35x softer) | 0.340 | +0.117 |
| k_cross+k_flap | 0.381 | +0.158 |
| skew -> v6 | 0.154 | -0.069 |
| translations -> v6 | 0.202 | -0.021 |
| hinge radius -> v6 | 0.217 | -0.006 |
| loss (v6 has none) | 0.224 | 0.000 |
| steelx root fitting mass | 0.199 | -0.024 |
| full v6 K6 (everything above, off-diagonals incl.) | 0.210 | -0.013 |

Soft cross/flap stiffness is not a cost: it helps (it moves the flap/pitch resonance). Of the hinge matrix
alone only skew (-0.07 single swap) and the stiff translations (-0.02) cost anything, and the full v6 K6 still
scores 0.21. So the 3.4x gap is mostly NOT the hinge stiffness. Beyond-hinge swaps (K6(v6) base + fitting 0.194):
| + step | cr_pl |
|---|---|
| ideal elastic axis -> v6 (0.404 -> 0.350) alone | 0.235 |
| K6(v6) + fitting mass | 0.194 |
| + v6 elastic axis, no tip mass / lag-inertia tuning mass | 0.084 |
| + v6 R and skew | 0.063 (v6 actual: 0.065, reproduced) |
Largest cost: the ideal's free tip mass (0.49 g at +chord offset) and lag-inertia/elastic-axis mass placement,
which v6 does not have (-0.11 of the 0.16 loss from the K6+fitting baseline); then the steelx fitting mass (-0.025)
and skew (-0.02..-0.07). Plus the material side: v6 is fatigue_stress-pulse-limited at low J / low throttle
(pulse_scale 0.5-0.78), which the ideal card has no counterpart for. Tip mass alone removed from the ideal is
infeasible (pulse_infeasible), i.e. the ideal depends on it.
Caveat: the swaps interact through resonance placement (non-additive; infeasible baseline).

## 3. Families (log kc/kh, log kf/kh, log klag/kf, skew, R/Rtip; 4 seeds per size, 2-means on standardised features)
7in: family A "stiff-flap / soft-hinge" robust_seed1+2 (kc/kh 1e4-4e4, kf/kh 5e4-8e4, klag/kf 0.1-0.7, skew 67-69),
cr_pl 0.193 / 0.223. Family B "soft-flap" seed1 (old objective) + robust_seed3 (kc/kh 4e3, kf/kh 7e2-3e3, klag/kf 1-6),
cr_pl 0.000 (pulse-infeasible) / 0.100.
10in: family A = seed1, robust_seed1, robust_seed2 (kf/kh 7e4-7e5, klag/kf<1), cr_pl 0.000 / 0.364 / 0.426; family B = robust_seed3, cr_pl 0.166.
Best family under pulse_limited: A (stiff-flap), both sizes; the old-objective optimum (seed1, the best by own cr
at 10in: 0.359) is the worst on cr_pl (pulse-infeasible), so the best family HAS changed from the old-objective seed1.
Steelx realisations: v6 kc/kh 1.1e3, kf/kh 1.1e3, klag/kf 0.65, skew 60 -- ratios two to three decades below
family A and in the soft-flap family B neighbourhood (kf/kh ~1e3, like seed1/robust_seed3). 10in v4 has a slightly
negative effective k_hinge (-0.055) and softest axis at 31 deg vs 60 deg design skew: not on the target axis.
So the steelx realisations sit in family B, not the best family.

## 4. Recommendation
Realisation target: family A (robust_seed2) with the stiff flap/cross relative to the hinge (kc/kh ~4e4, kf/kh ~5e4,
skew ~67), i.e. a much stiffer flap-and-cross strap (or a shorter/wider strap pair, or a flap stop) at the same
~0.015-0.02 N m/rad hinge, plus a ~0.5 g tip/chord ballast and elastic axis near 0.40 c. Caveat: the ideal swap
test showed softer cross/flap raised the ideal's own cr_pl (resonance placement), so a first test should vary the
tip mass/lag inertia on v6 (ballast) before restiffening. Next compute: pulse-limited DE for the ideal restricted to
a physical tip mass and with the flap resonance margin enforced at the current RES_BAND, to get a feasible upper bound.
