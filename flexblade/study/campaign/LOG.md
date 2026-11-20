# Improvement loop log

One entry per loop iteration (newest last). Detailed experiment notes stay in CAMPAIGN.md.

## 2026-11-20 23:30 — iteration 1
- State: merged filament DB, motor_thermal, usage-tracing kit (lean agents); 176 tests pass; pushed.
- Model finding: beam k_hinge ≈ 3.4× stiffer than CalculiX (10.2 vs 2.99 N·m/rad, E01): fitting/anchorage treated rigid. All control_ratio numbers are suspect until fixed.
- Helpers: (1) opus — root/anchorage compliance in beam model; (2) sonnet — real filament allowables + temperature derating.
- Compute: run_study seed 1 (5–10", tpu_rod/torsion_bar/tpu_split/cf_flexure_sym) running local:26 + mac:9; at gen ~20, 10" tpu_rod cr 0.023 feasible. Results will be stale after the compliance fix → rerun leaders.
- Cleaned 2 orphaned idle worker pools on the Mac.
- Next: land helpers, rerun leaders on corrected model, verify top 3 with fea+ccx.
- Fred 23:40: ALWAYS keep compute burning regardless of results. burn.sh (setsid, pid 364650) chains DE runs after seed1; check it's alive each iteration, restart if dead; rerun leaders after model changes on top.

## Design-ideation track (Fred 23:50: compute alone won't solve it — intuition + agentic design in the loop)
One helper slot is reserved for NOVEL DESIGN each iteration: pick one idea, reason it out from first principles (stiffness/strain budget, manufacturing route), implement as a concept (Design template + DE variable ranges in run_study.py), screen with beam + coarse FEA, render a drawing sheet, write KEEP/KILL/NEEDS-CAPABILITY with why. Every few iterations I review drawing sheets myself and propose new ideas from what I see.
Idea backlog (unscreened):
- I1 continuous capped CF rod blade-to-blade through hub (rod = tension member + forming mandrel)
- I2 neutral-axis laminate strip (thin CF/G10/spring-steel strip at the bending neutral axis, TPU around it)
- I3 cross-spring / butterfly pivot with remote centre of rotation placed on the skewed lag axis
- I4 heat-formed: print flat with in-plane inserts, twist into skewed geometry post-print
- I5 cut commercial prop blades clamped into a printed flexure hub
- I6 Kevlar/Dyneema thread tension member (zero bending stiffness, huge axial stiffness)
- I7 whole prop as one graded flexure (infill/wall-count grading along span)
- I8 drilled post-insert rods (standard 1–3.175 mm) after forming, no print pause
- I9 LET (lamina-emergent torsion) hinge in printed CF-nylon
- I10 spring-steel or FR4 strip flexure, bought part, bolted/glued

## Idealise-then-realise track (Fred 23:55)
1. IDEAL: "ideal" flexure topology = generalised 6x6 spring + free mass placement; DE on control_ratio with only aero/motor/ESC/resonance/stability constraints → spec card per size (study/results/ideal_spec_<D>in.json). (helper launched)
2. FEW IDEALS: run multi-seed; cluster optima → expect a handful of distinct ideal families.
3. REALISE: score every concept by distance to its ideal spec (stiffness ratios, mass placement); inverse-design real geometry to hit the spec (FACT constraint spaces, material choice, rods/strips); log gap ideal→real per concept.
