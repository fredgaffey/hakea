# 2-day unattended design campaign (started 2026-11-20)

Goal (Fred): get as close as possible to the global best flexure-rotor design for Hakea
(2307 1300KV, 0.6 kg, 5–8"), by a careful, problem-driven climb — not brute force —
and at the end write a **generic playbook** for an arbitrary mission.

## Method: problem-driven climbing loop (each check-in = one step)

1. **Read state**: grid/optimiser progress (`full_grid/SUMMARY.md`), leaders, FEA/CalculiX
   results, agent reports.
2. **Diagnose the governing problem** of the current leaders: which check binds
   (creep / fatigue / interlayer / motor torque / ESC / resonance / FEA-only failure such
   as the TPU root fitting), and *why* physically.
3. **Brainstorm 2–4 distinct remedies** for that problem (different mechanisms, not just
   parameter tweaks — e.g. move the load path, change material, change topology, change
   operating point). Record them in the log below.
4. **Test each remedy computationally** as a small background job (grid/LHS on the
   relevant variables, then coarse FEA, then CalculiX on the best). Scripts in
   `campaign/experiments/`, results next to them.
5. **Adopt** remedies that improve the leader without breaking another check (verified by
   FEA, not just the beam model); **reject** with reason otherwise.
6. **Iterate** from the improved leader. Keep a diverse archive (best per concept family),
   not one design.

Rules: verify leaders with FEA every step and CalculiX for adoption; beam/FEA/CalculiX
disagreement > 15 % is a bug flag — investigate before trusting. Token frugality: each
check-in is short; heavy work is background compute; at most one lean agent at a time
and only with a clear need. Commit + push results at every check-in.

## Deliverables when Fred returns
- `campaign/REPORT.md`: ranked, diverse shortlist (beam + FEA + CalculiX), print files for
  top 3, the climbing history (problem → remedies → verdicts), open uncertainties and the
  bench tests that would resolve them.
- `flexblade/PLAYBOOK.md`: generic method for an arbitrary mission spec.

## Background jobs
- `full_grid.py` — large resumable grid over concept families × sizes 5–8" (laptop + Mac).

## Log (newest last)
- 2026-11-20 start: running agents — free-form FEA optimiser, CalculiX cross-check,
  ideation (≥25 concepts + screening), CFD validation (Mac). Known open problem: FEA shows
  the 5" TPU+2-rod design's TPU root fitting at utilisation ≈ 4.8 (beam model: 0.99) —
  rods end inside the flexure so the centrifugal load passes through TPU. First remedy
  candidates: continuous rod into rigid CF blade/hub; CF root fitting; longer rod
  embedment; laminate strip.
- 2026-11-20 CFD agent merged: coarse 3D hover (262k cells, checkMesh 5 failures) for 5in_tpu_rod_winner gives thrust 4.51 N vs BEM 5.88 N (0.77×) at equal torque (1.00×), FoM 0.48 vs 0.72. If confirmed by the fine mesh, BEM over-predicts thrust ~25 % → hover rpm higher, pulse headroom lower. PENDING on Mac: fine hover (1.3M cells), kOmegaSSTLM 1/rev pitching. Batch runner: cfd.validate.blade_shapes().
- 2026-11-20 scope added (Fred): vehicle-level sizing — monocopter vs coaxial, payload/weight ratios, battery configuration (chemistry, S count, capacity, mass; voltage sets motor pulse headroom). Lean agent building flexblade/vehicle.py + study/vehicle/sweep.py; merge at a check-in, then include vehicle-level trade-offs (endurance vs payload vs control authority) in REPORT.md and the PLAYBOOK mission spec.
