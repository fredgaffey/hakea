# Precone vs aeroelastic stability (v8, v11d; eig damping, presets re-solved per precone, T0.7/1.0/1.4)
Limit zeta_min=0.005 (util = 0.005/zeta_min, so util 1 = limit). Script-evaluated via rotor.evaluate; no robust "mech" mode here.
Least-damped mode is always a torsion ("pitch sym/antisym[+hub]") at 2.6-5.0/rev, never flap or lag.
| precone | v8 util (T.7/1/1.4) | v8 zeta_min | v11d util | v11d zeta_min |
| 0   | .64/.58/.62 | .0078-.0086 | .78/.75/.68 | .0064-.0073 |
| .65 | .75/.67/.63 | .0067-.0079 | .76/.71/.67 | .0066-.0075 |
| 1.3 | .74/.69/.67 | .0068-.0075 | .72/.71/.70 | .0069-.0071 |
| 2.0 | .84/.81/.81 | .0059-.0062 | .86/.76/.49 | .0058-.0102 |
Flap min zeta 0.011-0.035 (v8 ~0.012 once precone>0; v11d 0.026-0.035 to 1.3 deg). Lag mode only resolved (as elastic) at 2.0 deg: zeta 0.22-0.44, at 0.6-1.4 Hz (~1/rev), large.
Trend: v8 torsion damping falls ~12-20% from 0 to 1.3 deg and ~25% by 2.0 (util +0.1 to +0.2); v11d is flat/improves to 1.3 deg. Lag damping did NOT visibly fall (not
resolved at 0-1.3 deg, 0.2-0.4 at 2.0). Ormiston-Bousman's lag-destabilisation applies to soft-inplane hingeless rotors at low lag frequency; here not seen. Weak qualitative agreement only via the mild torsion/flap trend.
1.3 deg is safe on both: util <= 0.74 at every thrust, margin >= 1.4x over the 0.005 limit. Recommend 0.65-1.3 deg (use ~1.0-1.3 for the vertical-force benefit); avoid 2.0 on v8 (util .84, still passing).
Caveat: lag mode filtered/low-frequency, so lag trend is inconclusive; flap/torsion trend is the reliable result.
