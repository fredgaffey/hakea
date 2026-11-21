# Fatigue levers for 10in steelx v5 (script: study/fatigue_levers_10in.py; robust pulse_limited, throttle {0.7,1,1.4} x J {3.76,2.5,5.49}e-6)

## Key finding: fatigue is not what binds cr_pl
Base v5 per-throttle cr_pl: T0.7 0.180 and T1.0 0.121 (both fatigue_stress), T1.4 0.082-0.089 (motor_torque_1p, pulse scale
capped at 1.15). The overall cr_pl 0.082 is the T1.4 point, set by motor torque and resonance gain. Fatigue has about
47 % headroom at T1.0 and 120 % at T0.7. At T1.4 fatigue is only just behind the motor-torque limit: the 301 FH run (lower
fatigue allowable) gives fatigue_stress 0.083 vs motor_torque 0.082. So any fatigue lever gives about 0 change in cr_pl,
and the >=10 % target is unreachable this way. v5f was NOT saved.

## Binding fatigue point (from rotor.py / materials.py)
- The model is amplitude-only: util = (E * strain_amp) / (0.40 * Xt 1.4 GPa) = 560 MPa, with strain_amp 0.28 % for the
  strain check. There is no Goodman or mean-stress correction. CF tension (mean) does not enter; only the 1P (+nP crossing,
  combined) alternating strain does.
- Location: the clamp edge is a multiplier, not a separate location. strip_clamp_factor = Kt/fretting x(1.05 compliant,
  1.10 radiused R>=10t, 2.19 sharp) on the max over all strip fibres. The fibre is the outer surface of a lamina
  (z = +-t/2 of one 0.1 mm lamina, bending amplitude); mid-span vs edge is not separated.
- Shot peening is not modelled. 17-7 PH is not in the library (SHIM_STEELS = 1095, 301 FH).

## Lever results (cr_pl; base 0.0822, feasible)
| lever | cr_pl | note |
|---|---|---|
| clamp radiused R=10t (1 mm) | 0.0822 | same as compliant at T1.4, T0.7/T1.0 cr_pl 0.172/0.115 (compliant is better, 1.05 vs 1.10) |
| clamp radiused R=20t (2 mm) | 0.0775 | worse: Kt interpolation is saturated at 10t and the wrap stress rises |
| compliant (current) | 0.0822 | already at the model minimum Kt 1.05, so no clamp headroom |
| 301 FH | 0.0821 | fatigue cr_pl T0.7 0.154, T1.0 0.106, T1.4 0.083: lower allowable, hence a lower fatigue ceiling but not binding |
| 8x0.1 -> 4x0.13, w 8.2 mm (same EI) | 0.0764 | infeasible (resonance 0.26); more strain per lamina |
| 8x0.08, w 17.6 mm (same EI) | 0 | infeasible: flexure_inside_hub, aeroelastic stability |
| 8x0.08, w 12 mm | 0 | infeasible (hub, stability; EI drops 0.51 x w ratio) |
| w 12 mm, t 0.0912 mm (same k) | 0.058 | infeasible: hub and aeroelastic stability |
| 10x0.08, 12x0.075 | n/a | schema limit strip_laminae <= 8 |
| 17-7 PH / peening (assumed allowables injected into LIBRARY) | invalid | name-keyed material lookups changed the dynamics (aeroelastic fail); not trustworthy. T0.7 fatigue cr_pl rose to 0.212 vs 0.180, but T1.4 is motor-torque limited at 0.072 anyway |

## Ranking and conclusion
No lever gains cr_pl: all are within +-0 % or negative. Only the material/peening and thinner-laminae levers raise the
fatigue ceiling (T0.7/T1.0), and neither lifts the T1.4 point. To raise cr_pl, work on the motor_torque_1p / resonance gain
at T1.4 (not on fatigue). Thinner laminae at the same k need more width than the hub allows (width x (0.1/t)^3).
