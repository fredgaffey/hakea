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
