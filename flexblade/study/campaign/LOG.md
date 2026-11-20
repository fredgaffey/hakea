# Improvement loop log

One entry per loop iteration (newest last). Detailed experiment notes stay in CAMPAIGN.md.

## 2026-11-20 23:30 — iteration 1
- State: merged filament DB, motor_thermal, usage-tracing kit (lean agents); 176 tests pass; pushed.
- Model finding: beam k_hinge ≈ 3.4× stiffer than CalculiX (10.2 vs 2.99 N·m/rad, E01): fitting/anchorage treated rigid. All control_ratio numbers are suspect until fixed.
- Helpers: (1) opus — root/anchorage compliance in beam model; (2) sonnet — real filament allowables + temperature derating.
- Compute: run_study seed 1 (5–10", tpu_rod/torsion_bar/tpu_split/cf_flexure_sym) running local:26 + mac:9; at gen ~20, 10" tpu_rod cr 0.023 feasible. Results will be stale after the compliance fix → rerun leaders.
- Cleaned 2 orphaned idle worker pools on the Mac.
- Next: land helpers, rerun leaders on corrected model, verify top 3 with fea+ccx.
