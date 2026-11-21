# Agent run log

One row per agent run, appended when its work lands. Generate the row with
`node tools/usage/trace.mjs row --agent <id> --outcome "..." --green yes|no|n/a`
and write nothing by hand except the outcome. The usage-ledger test rejects
rows without the token field (or the green flag, if the project requires it).

| date | model | type | task | outcome | tokens / calls | green first try |
|---|---|---|---|---|---|---|
| 2026-11-20 | sonnet-5-5 | general-purpose | Motor heating check | motor_thermal check on branch, unmerged | 56k / 15 calls | green: yes |
| 2026-11-20 | haiku-5-5 | general-purpose | Lean test: count conflict markers | base-cost test: general-purpose+haiku, 38k ctx | 38k / 2 calls | green: yes |
| 2026-11-20 | haiku-5-5 | lean | Lean test: count conflict markers | base-cost test: lean+haiku, 9.3k ctx | 9k / 2 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | Diagnose hinge stiffness mismatch | ccx test bound stale (rigid fitting ca47135); found beam 3.4x stiffer than ccx | 20k / 12 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | Real allowables + temp derating | temp derating + hdt check; brand creep/fatigue all class defaults; winner util up to 0.94 at 40C | 60k / 29 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | Add root compliance to beam model | anchorage compliance: beam now within 3% of ccx on E01; winner cr 0.22->0.017 (butted rods) | 83k / 46 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | Ideal flexure spec optimisation | ideal flexure topology + ideal_spec DE; ideal wants soft hinge, kcross/khinge~260 | 82k / 42 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | Lag tuning + inertia map (I12/I13) | lag tuning/J_hub sweeps: sharp resonance, I13 refuted, found motor_thermal 3.7 | 26k / 7 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | Robust multi-throttle objective | robust multi-throttle eval; cr swings 6x over throttle | 37k / 17 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | Realise ideal: skewed cruciform pivot | skewed cruciform KILL: CF through hinge member; -> tension-torsion strap | 78k / 47 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | Tension-torsion strap concept | tt_strap KEEP as CF path (all checks pass); cruciform ratios ~10 too low -> pair strap with crossed plates | 103k / 51 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | Beam vs ccx gap on cf_flexure | plate modulus Q11 fix: crossed plates beam/ccx 0.54->0.70; torsion_bar ccx mesh merge diagnosed | 80k / 42 calls | green: no |
| 2026-11-20 | sonnet-5-5 | lean | Beam vs ccx gap on cf_flexure | plate modulus Q11 fix: crossed plates beam/ccx 0.54->0.70; torsion_bar ccx mesh merge diagnosed | 80k / 42 calls | green: no |
| 2026-11-20 | sonnet-5-5 | lean | Validate vs Hakea bench logs | bench validation: steady rpm 4.5% rms; dynamic gain 2-13% (small), -14..-29% (big sweep); 35-45deg unmodelled phase; Rth not identifiable | 35k / 13 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | Strap + released crossed plates | tt_strap_xplate KILL for now: plates still loaded (k_hinge +0.4), pitch split binds; robust ideal ~ pin joint (ratios 3e3-3e4) | 84k / 43 calls | green: no |
| 2026-11-20 | sonnet-5-5 | lean | Flexure drag + envelope penalty | root drag + envelope: strap hand +16% hover power -> motor_thermal 1.07 FAIL; envelope 3.5-4.1x | 53k / 25 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | Feeler-gauge cross-spring + strap | steel shim cross-spring: unloaded k on target, ratios 1.3e3-4.3e3; KILL hand designs (pitch split from static lag; 10in strip buckles); suspect loaded-K accounting | 107k / 48 calls | green: no |
| 2026-11-20 | sonnet-5-5 | lean | Compact faired root geometry | compact root: height set by flexure members, not fittings; fairing caps -8% power (thermal 1.07->0.98); envelope still 3.5-4.7 | 63k / 40 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | Audit loaded flexure stiffness | BUG FIX condense_to_point missing preload transport term; steelx loaded k_hinge 0.34->0.0195, ratios ~5e3; dynamics unaffected | 86k / 42 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | Pre-lag hinge preset | pre_lag_deg feature (hinge/pitch axis, auto) | 74k / 25 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | CalculiX check of steel cross-spring | ccx hex20 steel strips: k_hinge within 20%, cross/flap 1.3x, axis<1deg | 69k / 28 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | Evaluate pre-lag over throttle | prelag eval: hinge-axis preset fails; split throttle-invariant -> pitch preset works | 31k / 13 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | 7in steelx realisation with pitch preset | 7in steelx v2: split solved (<=0.35deg), cr 0.36/0.17/0.085 but resonance (pitch-antisym+hub ~1/rev) binds; KILL v2 | 69k / 36 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | 10in steelx: buckling + strap fix | 10in steelx v2: strip compression from mean rotation via strap tilt (fixed: CG 0.415, strap 12mm); BUG FIX bought strand judged with print strength; still infeasible (trim nonconv, 1P flap) | 89k / 40 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | Finish evaluation cache | eval cache landed: sqlite content-addressed + model version, host-side pool dedupe; warm 0.01s vs 5s cold | 35k / 34 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | Finish Floquet stability module | stability: true damped eig + Floquet airframe, Coleman validated <2%; no air resonance; damping-aware 1P margin -> 7in v2 feasible at hover | 97k / 29 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | Fix 10in trim non-convergence | trim: bracketed Illinois + trim_ok flag; 10in v2 has NO valid trim (elastic pitch collapse branch) | 49k / 36 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | Robust re-tune of 7in steelx | 7in steelx v3 ROBUSTLY FEASIBLE: cr 0.354/0.196/0.107 at 0.7/1.0/1.4 | 64k / 40 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | Verify 7in steelx v3 | verify v3: ccx confirms strips (0.94-1.02); NOT tolerance-robust (9/20 random fail); air resonance unstable band 160-260Hz; 5P on 2nd pitch; clamp issues | 75k / 29 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | 10in: balance propeller moment | pitch_cf_ratio criterion + balance masses; 10in v3 trims at all throttles, feasible at T1.0 cr 0.136, robust 0.074 (fatigue vs 3P) | 100k / 54 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | 7in v4: tolerance-robust + damping | 7in v4: 21/21 tolerance pass, cr 0.186/0.109/0.062 (P20 0.161/0.095/0.055); CLD damping small; air-res root = 2/rev parametric hub mode on anisotropic support, needs zeta_af>=0.05 | 107k / 49 calls | green: yes |
| 2026-11-20 | sonnet-5-5 | lean | Persistent evaluation result cache | eval cache (sqlite, model-version keyed) in api.evaluate + HostPool.map; warm 550x | 27k / 22 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | Floquet stability + airframe coupling | stability: true zeta near-1P mode 0.02 (no damping-aware relaxation); anisotropic-support 2-blade window when f_x<1P<f_y | 62k / 30 calls | green: yes |
| 2026-11-20 | opus-5-5 | lean | I23: CF-stiffened pitch via strap | I23 bifilar strap KILL: CF pitch spring reaches skewed hinge only via sin^2(skew), lands hinge mode on 1P; 2nd pitch sweep unchanged 1.35x | 89k / 41 calls | green: yes |
| 2026-11-21 | sonnet-5-5 | lean | Strip clamp-edge fatigue model + fix | clamp-edge fatigue model (sharp Kt1.75/fretting, radiused R>=10t Kt1.1); v4 needs radiused jaws (sharp T0.7 util 1.41); 10in v3 still infeasible, needs whole re-opt | 60k / 35 calls | green: yes |
| 2026-11-21 | sonnet-5-5 | lean | 7in v4 build package (STL + sheet) | 7in v4 build package DRAFT: STLs (not watertight, slivers), BUILD.md, drawing; fittings overlap ~62mm3, OCC export crashes | 60k / 33 calls | green: partial |
| 2026-11-21 | opus-5-5 | lean | Buildable parametric steelx fittings | fittings.py CSG parts: watertight STLs, clearance 1.5mm min, overlap 0; FOUND printed fittings ~3.4+3.5 g/blade vs model 0.08 g -> mass under-modelled | 110k / 43 calls | green: yes |
| 2026-11-21 | opus-5-5 | lean | Fitting mass into model + slim + re-verify v4 | real fitting mass in model (default steelx): v4 INFEASIBLE (pitch mode 0.98/rev, 2nd pitch 2.79/rev on 3P); slim 4.1 g still infeasible; v4m least-violated | 111k / 60 calls | green: yes |
| 2026-11-21 | opus-5-5 | lean | I25: fittings as own counter-weights | I25 KILL: cross angle 105 cancels fitting propeller moment (pitch mode 0.98->0.83/rev) but 2nd pitch mode lands at 2.79/rev vs 3P; 0/9 tolerance | 76k / 43 calls | green: yes |
| 2026-11-21 | opus-5-5 | lean | Forced-response check for nP≥2 crossings | forced-response nP check (forced.py): 7in v5/v4m/v4 feasible with real mass; v5 tolerance 9/9, min cr 0.049, binding motor thermal/torque; 10in v3 hub-pitch mode 1.97/rev zeta 0.004 fails | 89k / 32 calls | green: yes |
| 2026-11-21 | sonnet-5-5 | lean | Verify 7in v5: ccx + build package | verify 7in v5: ccx/beam 0.95-1.03, fitting mass exact, STLs watertight, air-res zeta -0.003 at zeta_af 0.02 (+0.012 at 0.05); build package 7in_v5 | 47k / 13 calls | green: yes |
| 2026-11-21 | sonnet-5-5 | lean | Damped isotropic motor mount (I24) | TPU shear-washer isotropic motor mount: hub mode 123-129 Hz, zeta_af 0.065, Floquet min zeta 0.0092->0.0193; cost 1.3 deg disc tilt at T0.7 (k_tilt 4.4 Nm/rad); STLs | 46k / 17 calls | green: yes |
| 2026-11-21 | sonnet-5-5 | lean | v5 motor-limited authority study | authority chain v5: weakest link hub swing 1/(J Omega^2) at T1.4; J_hub -30% => +24% min-cr; J_hub model 5.49e-6 vs printed hub 0.75e-6 (guess); v6 lever 8/9 tol (fatigue) | 66k / 23 calls | green: yes |
| 2026-11-21 | sonnet-5-5 | lean | v6: low-inertia hub + fatigue fix | itemised J_hub 0.92e-6 (model guess 5.49e-6): v5 cr doubles to 0.103 but fatigue 1.84; best v6 cand 7/9 tol min-cr 0.057; added steel_301_fh (worse than 1095) | 73k / 28 calls | green: partial |
| 2026-11-21 | sonnet-5-5 | lean | Surrogate-assisted search + tolerance map | surrogate.py (LHS+GP, resumable CLI); old-model CV R2 cr 0.90 / 1P margin 0.95 / fatigue 0.92; picks 9/9 tol at cr 0.062-0.087; stopped (stale model), queued as job | 73k / 30 calls | green: partial |
| 2026-11-21 | opus-5-5 | lean | I28 laminated shim stack strips | I28 laminated shim stack KEEP: 4x0.10 mm, fatigue 0.686->0.515, tol 9/9, cr_min 0.053; friction damping negligible (zeta ~2e-4) | 88k / 38 calls | green: yes |
| 2026-11-21 | opus-5-5 | lean | Pulse-limited scoring + J_hub tolerance | pulse_limited objective + J_hub robust axis; v6 (cross 88, compliant, L12.5) robust over J 0.92-5.49e-6 cr_pl 0.065, tol 9/9; v5 fails 1P margin at low J | 92k / 43 calls | green: yes |
| 2026-11-21 | opus-5-5 | lean | 10in v4 with all new mechanisms | 10in v4: 8x0.10 laminae w10, cross 105, hinge r 21.5; tol 9/9; cr 0.242/0.107/0.056; T1.4 moment 0.165 Nm = 2.6x 7in; not pulse-limited scored, no ccx | 95k / 49 calls | green: yes |
| 2026-11-21 | sonnet-5-5 | lean | v6L: v6 + laminated strips + build pkg | v6L: laminae don't beat v6 (worst case = motor 1P torque at high J); 2x0.13 0.0644 9/9; 7in_v6 build package (6 watertight STLs, BUILD.md, pulse schedule); itemised 7in J_hub 0.65e-6 < 0.92 corner | 49k / 19 calls | green: yes |
| 2026-11-21 | sonnet-5-5 | lean | Verify 10in v4 (pulse-limited, ccx, Floquet) | verify 10in v4: cr_pl 0.095 (best yet) but 1P margin 0.076-0.106 at T0.7 (pitch antisym+hub 0.94/rev); ccx x8 0.95-1.08; air-res fixed by TPU mount; stack fit tight; J_hub 3.76e-6 | 62k / 25 calls | green: partial |
| 2026-11-21 | sonnet-5-5 | lean | Ideal-vs-realised gap under new scoring | ideal gap: 7in ideal 0.223 vs v6 0.065, 10in 0.426 vs v4 0.095; gap mostly ideal's free 0.49 g tip mass/lag-inertia tuning + skew, not hinge matrix; best family A stiff-flap/soft-hinge skew 67-69 | 64k / 29 calls | green: yes |
| 2026-11-21 | sonnet-5-5 | lean | 7in v7: tip ballast + EA + skew | tip ballast on v6: no feasible v7; +23% cr_pl (0.080) only by pulling antisym+hub mode to 1P (margin 0.035); 1P margin caps all inertia gains | 33k / 17 calls | green: no |
| 2026-11-21 | opus-5-5 | lean | 10in v5: fix T0.7 1P margin | 10in v5: strip L 21.3->25.1, w 9: all hard checks pass, cr_pl 0.082 (best feasible), tol 8/9; 2-DOF fit k_cf/I 0.929; aero stiffness lowers mechanism mode 0.99->0.83/rev | 85k / 30 calls | green: yes |
