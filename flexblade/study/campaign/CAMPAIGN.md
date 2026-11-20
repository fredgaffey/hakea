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
- 2026-11-20 ideation merged (study/ideation/): 38 concepts → 15 KEEP / 11 KILL / 12 NEEDS-CAPABILITY. Beam leaders ~0.21–0.30 control ratio (wide-chord / cambered / larger-cutout blades, steel-wire LE inertia tab, graded prop) — but ranking is driven by lag mode sitting at 0.72–0.85/rev (near resonance). EVERY rod-in-TPU leader fails coarse FEA at the root fitting (2.1–4.3); only the torsion bar passes FEA (0.66) at ctrl 0.076. No 0-pause design viable yet.
  → Governing problem for the climb: (1) root-joint load path (FEA util > 2), (2) robustness to near-resonance (need ctrl at ±15 % stiffness). Remedies to test next: continuous rod into rigid blade/hub (needs geometry/FEA support), CF/PA root fitting resized, rod embedment length, torsion-bar family scale-up, blade-shape lever (lower rpm) to get authority without parking near resonance. Backlog top: root-joint modelling, per-station sweep field, robustness metric in screening. CFD candidates: B01, B04, B02.
- 2026-11-20 vehicle sizing merged (flexblade/vehicle.py, study/vehicle/VEHICLE.md): 864 configs, single vs coax 5–8", 3–6S, LiPo/LiHV/Li-ion. Endurance ~21–46 min single, 21–38 coax (≈30 % less if BEM thrust is 23 % optimistic). e.g. 6" 6S1P 50 g payload: 695 g, 159 W, 35 min. Caveats: control_ratio there uses an UNTUNED placeholder flexure — re-run vehicle sweep with the campaign's tuned leaders before comparing layouts on authority; AUW unconstrained (largest pack wins endurance) → add mass cap/payload-fraction objective in the playbook mission spec.
- 2026-11-20 CalculiX bridge merged (flexblade/ccx.py, study/ccx_crosscheck.md; independent of fea/beam/rotor). BUG FLAG (TOP PRIORITY next check-in): for 5in_tpu_rod_winner the root 6×6 disagrees hugely — beam diag K = [3.2e7, 9.5e5, 3.1e7 N/m; 5.5, 266, 5.8 N·m/rad] vs ccx [2.8e5, 1.2e5, 1.4e5; 0.46, 2.85, 1.66]; modes beam 268/325 Hz vs ccx 151/371 Hz. Our fea.root_stiffness ≈ beam (0.82 hinge). Suspects: ccx drops the blade-root shank (STEP export broken for this design: blade lost, shank inverted) so load passes only through thin TPU/fitting; different clamp/coupling regions; or beam rigid connectors. Resolve by running beam / fea / ccx on IDENTICAL region + BCs (start with a plain cantilever plate, then the flexure alone, then the full root), and fix geometry.export_step for this design. Do not adopt any leader until resolved. ccx needs CCX_NPROC_EQUATION_SOLVER=1 (multithreaded SPOOLES nondeterministic).
- 2026-11-20 18:50 check-in 1. Grid 90k/270k (laptop only; Mac offline). Grid leaders: 6" TPU-95A + 2–3 CF rods (1.4–1.6 mm), ctrl 0.32–0.38 beam-feasible — but beam-only, near-resonant and not FEA-verified.
  Problem: ccx vs beam 6×6 mismatch (24× on hinge). E01 (experiments/e01_stiffness_compare.py): beam vs our FEA agree on same basis (k_hinge_free 10.2 vs 7.7 N·m/rad). Diagnosis: geometry gives blade{k}_hub_fitting the FLEXURE material (TPU 95A) — a soft TPU block between hub and flexure; ccx models it, beam/fea clamp past it. REAL DESIGN BUG (same family as the TPU root-fitting failure: connectors must be rigid).
  Remedy adopted: hub fitting = HUB_MATERIAL (PA6-CF) in geometry.py. E02: ccx k_hinge_free 0.32 → 2.99 N·m/rad; diag now [4.3e6,3.8e5,3.8e6 N/m; 3.7,37,3.9 N·m/rad] vs fea [2.8e7,8.2e5,2.8e7; 5.8,227,5.8]. Residual gap ~2.6× hinge, ~6× ry/translations — remaining suspect: our fea/beam rigidly link member ends, ccx includes PA6-CF fittings + rod anchorage compliance. NEXT: run our fea on the same full region incl. fittings (or ccx with member ends rigid) to close it; until then treat beam stiffness as an upper bound and prefer designs robust to −60 % hinge stiffness.
