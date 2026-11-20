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
