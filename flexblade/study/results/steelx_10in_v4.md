# 10in_tt_strap_steelx_v4 (hand design, current model)

Script: `study/steelx_10in_v4.py` (`diag`, `batch`, `tol`, `save`). Raw: `steelx_10in_v4.json`.
Design: `study/designs/10in_tt_strap_steelx_v4.json`. Drawing: `steelx_10in_v4.png`.
Robust = thrust 0.7/1.0/1.4. 48 robust evals of local search, then the tolerance set.

## v3 on the current model (fitting mass, forced nP, clamp Kt)
cr 0.061/0.035/0.020, infeasible. The loaded pitch stiffness through the skew is negative
(pitch_k_struct -0.41/-0.51/-4.1 N·m/rad), so pitch_cf_ratio is infinite at T1 and the
aeroelastic check is non-finite. There is a 1.53/rev pitch-sym mode at zeta -0.0007. The
hub-coupled pitch antisym mode sits at 1.94-1.97/rev with zeta 0.003-0.004, which violates
the hard 2P margin (res 0.017). Strip fatigue is 1.27 (flex2/flex0, T0.7).

## Recipe (v3 base, with the 7in v5L recipe scaled by 10/7)
- I25 cross 105 deg (was 130), skew 60, hinge radius 21.5 mm, strip free length 21.3 mm,
  cross gap 2.7 mm, strap pin 8.6 mm.
- I28 laminae: 8 x 0.10 mm 1095, 10 mm wide. This is twice the strip area of the 4-lamina
  scaled stack. 1P strip fatigue here is load-limited: the 10" carries about 3x the 7" cyclic
  hub moment, so the stack needs area as well as low bending stiffness. Thinner or narrower
  stacks did not help.
- Compliant clamp. Strap: 2.6 mm aramid, 6.5 mm long. A longer strap lowers the strap
  fatigue strain, and a thicker strand only attracts more load.
- EA 0.33 c, which flattens cr across throttle. Kept v3's balance masses (2 x 1 g at 0.2R,
  +-10 mm normal). Without them cr is about 3x higher, but the antisym+hub mode moves to
  0.96/rev and fatigue reaches 2.7.
- 2P fix: the hub-coupled pitch mode is now 0.84/rev (T1), far below 1.7/rev. Nothing
  hub-coupled lies within 2P +-15 %. Only flap sym 1.90/2.13/rev (zeta 0.03) is a soft nP
  crossing. Strip friction eta 1.7e-4 (laminate.friction_eta at the 1P curvature) has no
  visible effect.

## Result
| throttle | cr nominal | cr P20 (9) | M_cyc N·m |
|---|---|---|---|
| T0.7 | 0.242 | 0.192 | 0.358 |
| T1   | 0.107 | 0.094 | 0.226 |
| T1.4 | 0.0557 | 0.0505 | 0.165 |

Binding constraints:
- strip fatigue flex0 0.83 (1P, T0.7) and strap fatigue 0.82;
- 1P margin 0.229 (flap sym 1.22/rev);
- cr_min at T1.4.

Other margins: pitch_cf_ratio 0.08-0.09, zeta_min 0.008, nP fatigue 0.08 or below.
Tolerance (+-5 % on t, w, L, hinge_r, strand d; nominal + 8 random): 9/9 feasible.

The 7in v5 and v5L are 7" designs; the 10" v4 is compared with them at T1.4:
- 7in v5: cr_min 0.049, M 0.060 N·m.
- 7in v5L: cr_min 0.053, M 0.064 N·m.
- 10in v4: cr_min 0.056, M 0.165 N·m. That is 2.6x the cyclic moment, and 1.5x the DE3 10" best (0.038).

Open: the drawing and parts were not checked for the 8-lamina stack height in the jaws, and
there is no ccx/Floquet verification yet.
