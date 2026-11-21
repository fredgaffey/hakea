# Verify light-blade variants (v10m_lw = chord x1.3, rho x0.59; v10m_l85 = rho x0.85)
Built as blade_inertia_lever.build + resolve_candidate(6); designs/10in_tt_strap_steelx_v10m_{lw,l85}.json. sa_both.py = sa.py printing default and damped gates from one run.
| | A lw | B l85 |
|---|---|---|
| sa.py default gates | FEASIBLE cr_pl 0.1257, ph 6.14 deg, gain 0.069, res 0.253 (flap sym 1.24/rev) | FEASIBLE cr_pl 0.0965, ph 3.46, gain 0.087, res 0.197 |
| SA_GATES=damped | infeasible: damped_gain 1.61, damped_phase 2.07, damped_amp 1.45 (norm. excess) | infeasible: gain 3.17, phase 1.83, amp 2.33 |
| tol.py 20 random (+-5 % t/w/L/R, presets fixed) | 4/20 pass (nominal 0.1346, res 0.155, pass) | 13/20 pass (nominal 0.1027) |
| tol fail modes | aeroelastic_stability (to 0.23), harmonic_jitter (to 1.35), resonance (to 0.26) | not itemised |
(v8 sa rescore: cr_pl 0.088 feasible. v10m default not re-run here.)
Note tol nominal cr_pl (3 thrusts, margin criterion) differs from sa.py cr_pl (5-node continuous cal).
Goodman fatigue_stress worst util (1P at full pulse, unscaled) T0.7/1.0/1.4: v10m 1.53/1.60/1.04 (strain 3.28), A 1.39/1.13/0.80, B 1.60/1.23/0.88, v8 2.30/1.67/1.15. strip_ltb max: A 0.32-0.37, B 0.27-0.29, v10m 0.27, v8 0.31 (all <1).
two_per_rev (precone 0, hub N): 1P vertical v10m 10.3, A 5.4, B 9.7; 2P vertical 2.2/11.6/1.1; 2P in-plane 0.41/3.0/0.36. A raises 2P loads ~7x (in-plane) and 5x (vertical) - flag. 2P-inclusive fatigue_stress 0.49/0.44/0.46.
Hover T1.0 (same thrust 16.64 N): FoM v8 0.748, v10m 0.753, A 0.759 (P 253.8 W, rpm 7485, -3.8 %), B 0.749 (P 257.1 W). Chord x1.3 gives a small gain (+0.8 % FoM, -0.8 % power vs v10m), no penalty.
Realism of rho x0.59: pa6_cf solid 1200 -> 708 kg/m3 = 59 % volume fill (or e.g. hollow ~0.4-0.6 mm perimeter walls + sparse/zero infill, chord x1.3 section 1.69x area so mass ~ v10m). materials.py variant "@rho=0.59" scales density only; E, G and strengths stay at solid values (the "uniform E/G scaling" is a separate E= key, not used). Real low-infill FDM: modulus drops roughly linear-or-worse with fill, strength and layer adhesion lower, buckling/wall thickness issues. Model is optimistic. Sensitivity: A with E x0.6 -> cr_pl 0.143, res 0.142, resonance viol 0.054 (pitch antisym 0.91/rev below 1P): mode margin shrinks ~0.01 and resonance gate fails. A hollow carbon-skin core_skin build keeps skin stiffness better than infill; needs a real layup, not just rho.
Verdict: A is feasible under default gates (saved as designs/10in_tt_strap_steelx_v11.json) but fails damped gates, 4/20 tolerance builds, and its rho 0.59 is not stiffness-consistent.
