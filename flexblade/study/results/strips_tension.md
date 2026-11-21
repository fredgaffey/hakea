# Steelx strips in tension? (10" v8, single nominal evaluations, study/strips_tension.py -> strips_tension.json)
N = section axial force from beam.element_end_forces(e, cr, st) at both ends of every strip element, min..max per strip (+ = tension;
the strap reads +461 N, so the sign is right). Utils come from api.evaluate + run_study.metrics at the unscaled 1P pulse, so read them relative to base.
**The premise is mostly wrong.** At T0.7/1.0/1.4, 3 of 4 strips are already in TENSION: b1.flex0 +2..+12, b1.flex1 +15..+35, b2.flex1 +9..+24 N.
Only b2.flex0 is compressed: -2..-6 / -2..-8 / -2..-10 N. That is far below fixed-fixed Euler (82 N), and strip_ltb is 0.30.
(strip_ltb.md's "-2 to -36 N, all strips" does not reproduce with this sign convention and v8 at T1.0. Worth re-checking which quantity it read.)
Why: CF does NOT reach the strips. axial_release=true is a free radial slider at each strip's blade end, so the strap carries the whole CF (461 N at T1).
Strip N = strip self-CF (~20 N per strip, 1.4 g at r 21.5 mm, 7720 rpm) + a lag/torque couple across the crossed pair (+-8..12 N, grows with T)
+ a blade-to-blade offset from the asymmetric slot presets (b1 mean ~+15, b2 ~+5 N). With no presets, b2.flex0 drops to -12 N.
| variant (T1.0) | b2.flex0 N | b1/b2.flex1 N | strip_ltb | fatigue | mech /rev | ctrl ratio | hub M mean |
|---|---|---|---|---|---|---|---|
| base | -8.2..-2.4 | 21..25 / 13..17 | 0.30 | 1.10 | 0.846 | 0.123 | 0.260 |
| strap d 2.6->2.0 mm (softer) | -8.1..-2.3 | same | 0.31 | 1.15 | 0.838 | 0.117 | 0.246 |
| strap L 6.5->13 mm (anchor/pin moved) | -7.3..-1.5 | same | 0.32 | 0.95 | 0.783 | 0.089 | 0.188 |
| release spring 3e6 / 1e7 N/m | -10.9 / -10.3 | 15..19 / 14..19 | 0.54 / 0.58 | 1.85 / 2.01 | 0.873 / 0.860 | 0.155 / 0.142 | 0.33 / 0.30 |
| bonded (no release) | -9.8..-3.3 | 18..22 / 16..20 | 0.66 | 2.11 | 0.848 | 0.133 | 0.281 |
| cross 105->110 deg | -7.2..-1.7 | 22..26 / 13..17 | 0.27 | 1.02 | 0.858 | 0.135 | 0.285 |
| skew 62->60 deg | -7.4..-1.2 | 22..26 / 14..18 | 0.28 | 1.19 | 0.871 | 0.143 | 0.303 |
| no slot presets | -12.3..-6.5 | 20..24 / 9..13 | 0.74 | 1.17 | 0.820 | 0.086 | 0.182 |
Levers: (a) the strap (softer strand, longer strap, moved pin) moves strip N by <1 N, because the slider decouples it. A strap-length preload
would also land only on the strap/slider. (b) A release spring, i.e. making the strips share CF, does NOT tension b2.flex0 (the couple dominates).
It doubles the fatigue util (1.10 -> 1.85..2.11) and the LTB util (0.30 -> 0.54..0.66), because the strips then see the 1P CF/flap strain: reject.
(c) Crossing geometry shifts b2.flex0 by only +0.5..+1 N. To reach +5..+20 N on that one strip you would need an axial length offset in its slots
(not modelled: strip_preset_deg holds rotations only) or symmetric b1/b2 presets, which reopens the cyclic calibration.
Recommendation: keep the free release and accept light compression on one strip. It is small (<=12% of Euler, LTB 0.30), it is not CF-driven,
and its Goodman mean is the favourable sign. Free side-find: cross_angle 110 deg improves fatigue (1.10 -> 1.02), LTB (0.27) and control ratio (+10%),
but mech moves 0.846 -> 0.858/rev, toward 1P. Check it on the robust/CAL set before adopting.

## v8 cross_angle robust check (110/120 deg, strip presets re-solved, sa.py default gates)
- v8 (cr 105): cr_pl 0.0715 -> variants: 110: cr_pl 0.0890, ph 7.1 deg / gain 9.0 %, mech 0.0245, res 1.18/rev, INFEASIBLE (cqc_mech_margin 0.182 viol).
- 120: cr_pl 0.1240, ph 7.5 deg / gain 7.6 %, mech 0.0135, res 1.19/rev, INFEASIBLE (cqc_mech_margin 0.549 viol).
- Only violated gate in both is cqc_mech_margin (dropped by SA_GATES=minimal, so likely feasible there; minimal run not done, compute cost ~20 min each).
- No v10 saved (rule: needs default-gate feasibility). Nominal fatigue/LTB gains do not survive the worst-corner mech-mode margin; scores in scratchpad o110/o120.json.
