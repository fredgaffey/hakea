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
