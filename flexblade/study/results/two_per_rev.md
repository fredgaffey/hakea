# 2P (second-order) response, 10in_tt_strap_steelx_v8, T1.0, pulse at s* (script `study/two_per_rev.py`, raw `two_per_rev.json`)

**Model gap:** `harmonic_1p` is a 1P-only linear harmonic balance about the static trim. `forced.py` adds nP only at
mode crossings, from empirical forcing fractions (2P torque 3 %, aero 1/n^2, imbalance). So the Paulos/Liang/Tang
mechanism (1P pitch x 1P heave/lag-rate products, flap x lag kinematics, mismatched blades) was **missing**. The blade
asymmetry itself is captured at 1P (each blade has its own response).
**Method:** second-order perturbation. U1 = s* U(1P). F2 = -1/4 D2R[U1,U1]: the corotational structure + CF + follower aero,
central FD on the nonlinear residual, Richardson in h, converged to 4 digits at h = 0.1 / 0.04. Added to that:
+1/4 D2f[q,q] of the quasi-steady `section_loads` (dU_T, dU_P incl. 1P inflow, dtheta), and the 3 % 2P motor torque.
Solve Z(2 Omega) U2 = F2 with the aero Ka at 2P (no 2P inflow). 2P strip forces = lin(U2) + 1/4 D2(end forces)[U1,U1].
The quadratic term is essential: the stiff rigid-link penalties give a lin-only answer 10-30x too large.

| precone | s* | blade | 1P flap / lag / pitch75 deg | 2P flap / lag / pitch75 deg | strip strain 1P -> 2P (2P/1P) |
|---|---|---|---|---|---|
| 0 | 0.907 | 1 | 0.36 / 2.04 / 4.41 | 0.039 / 0.043 / 0.17 | 1.24e-3 -> 0.16e-3 (13 %) |
| 0 | | 2 | 2.57 / 2.20 / 9.43 | 0.120 / 0.041 / 0.40 | 3.62e-3 -> 0.61e-3 (17 %) |
| 1.3 | 1.137 | 1 | 2.04 / 2.28 / 8.19 | 0.077 / 0.173 / 0.28 | 3.08e-3 -> 0.69e-3 (22 %) |
| 1.3 | | 2 | 1.43 / 3.68 / 9.65 | 0.044 / 0.045 / 0.55 | 2.88e-3 -> 0.83e-3 (29 %) |
(nominal section axial strain, x clamp factor, no Kt/mean; steel E ~200 GPa. The 2P motor torque adds only ~5 % to the 2P motion.)

Hub loads, fixed frame (N, N m). The 1P figures come from the existing linear model:
| precone | 1P vertical F (1/rev) | steady lateral F | 2P vertical F (2/rev) | in-plane 2P -> 1P / 3P lateral F | 2P moment -> 1P / 3P | steady control moment |
|---|---|---|---|---|---|---|
| 0 | **11.0** | 0.76 | 1.92 | 0.20 / 0.20 | 0.19 / 0.19 | 0.23 |
| 1.3 | 1.55 | 1.14 | 2.01 | 0.30 / 2.65 | 0.40 / 0.40 | 0.37 |

**Findings.** (1) The 2P strip strain is 13-29 % of 1P, above 15 % on 3 of 4 blade cases, and higher with precone. With SRSS
(`forced.FATIGUE_COMBINE`) the strip amplitude rises by only 1-4 %. With abs it rises by up to 29 %. (2) Hub vibration is the larger risk:
about 2 N at 2P vertical (≈12 % of thrust) with or without precone, 2P moments of the same size as the control moment, and with
precone a 2.65 N 3P lateral force. On the 10" arm (120 Hz spec, about 60 g on top) 2 N at 256 Hz is about 13 um, against 37-39 um at 1P lateral
(system_check.md). (3) Not 2P, but found on the way: the **mismatched 1P flapping already gives an 11 N 1P vertical hub force at
128 Hz, next to the 120 Hz arm**. system_check.md used only the in-plane fixed-frame mean, so this was never checked. Precone 1.3 brings it to 1.55 N.
**Fatigue inclusion (proposed, not implemented: more than 40 lines and about 34 extra assemblies per evaluate).** Move `second_order` + `strip_2p`
into `flexblade/two_per_rev.py`. Return a Crossing-like object (n=2, freq 2 f1P) that carries precomputed per-element complex end
forces `S`. In `rotor.checks` use `cr.S[i]` instead of `element_end_forces_lin(cr.U)` when present (a 2-line hook), so it is SRSS'd into
`fat_e/fat_s` exactly like the nP terms. Gate it behind an `analysis.two_per_rev` flag so the running studies do not move.
