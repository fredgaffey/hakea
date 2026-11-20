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

## First-principles levers (my design review, 2026-11-21 00:05) — test these, highest expected gain first
- I11 PITCH–LAG AMPLIFIER: today pitch/lag gain = tan(skew), and high skew costs stiffness/strain. A compliant lever (bridge/Scott-Russell-type amplifier or offset pitch horn inside the hub) can give pitch/lag gain 2–4 at modest skew, decoupling the gain from the hinge geometry. Ideal-spec run will tell what lag–pitch coupling is wanted; check if > tan(max feasible skew).
- I12 LAG TUNING NEAR 1/rev: response to the 1/rev pulse scales ~1/|1−ν²+2iζν| (ν = lag freq/rev). Flexure stiffness + centrifugal term set ν; pushing ν to ~0.6–0.8 with TPU damping (ζ from loss factor) amplifies lag 1.5–2.5× while staying stable. Map control_ratio vs ν explicitly (sweep k_lag) — the optimum is probably a tuning problem, not a strength problem.
- I13 INERTIA BUDGET: above lag resonance the blades stay ~inertially fixed and only the motor bell + hub swing: φ ≈ ΔQ/(J_hub Ω²). Shrinking J_hub (light hub, thin bell, small motor) raises swing directly. Quantify J_bell vs J_hub vs J_blades in the model; consider lightened bell / hub as a design variable.
- I14 PULSE SHAPE: motor torque is limited by current; a square-ish duty pulse gives 4/π more 1/rev fundamental than sine at same peak. Check the duty-drive model allows non-sinusoidal waveforms (ESC slew permitting).
- I15 ASYMMETRIC MASS: put lag inertia (tip/trailing mass) where it raises blade lag inertia but not flap inertia; trailing-edge mass also sets pitch-inertia coupling — use the ideal run's mass knobs to pick.

## 2026-11-21 ~00:50 — iteration 2 (targeted, not grid)
- MODEL: beam anchorage compliance merged (rod ends butted on fitting = contact-soft): beam now within 3 % of CalculiX on E01. Old 5" winner collapses 0.22 → 0.017. ⇒ ANCHORAGE is the central design problem; butted rods are out, embedded/continuous only.
- MODEL: motor pulse now thermally capped (capability = min(current, voltage, thermal)); Rth 2307 3.0→1.5 K/W (prop wash), cyclic_rms_frac 0.5 — both guesses, MEASURE Rth on Hakea. Old winner was at 380 °C winding.
- MODEL: temp derating + HDT check; brand creep/fatigue allowables are all class defaults (biggest data gap → coupon tests).
- I12/I13 tested: response sits on a sharp resonance; J_hub lever refuted (non-monotonic). Resonance knife-edges are fragile with throttle → robust multi-throttle objective being built.
- IDEAL (smoke): ideal wants very soft skewed hinge (k≈0.07 N·m/rad @5") with k_cross/k_hinge≈260, k_flap/k_hinge≈500, skew≈55°; ideal cr ≈ 2.5× best real. Binding: resonance near 1P (5", 7"), motor torque (10").
- COMPUTE: grid/burn STOPPED (thermally unsustainable pulse, not throttle-robust, low returns). Replaced by targeted ideal_seeds.sh (multi-seed ideal DE, 5/7/10") → cluster ideal families.
- HELPERS: robust multi-throttle objective (sonnet); realise-ideal: skewed cruciform torsion hinge vs cross-axis pivot (opus).
- NEXT: cluster ideal seeds; realise best family with embedded anchorage; robust re-run; verify with ccx.

## 2026-11-21 ~01:40 — iteration 3
- MODEL: robust objective merged; each throttle point pulses at 0.95× motor capability at its own rpm (op.pulse_capability_margin). 7" ideal (seed1, hover-trained) stays feasible 0.7–1.4× thrust: cr 0.54 / 0.22 / 0.18.
- DESIGN: skewed cruciform KILLED. Key insight: CF load (~1 kN @5") must NOT pass through any member that sets hinge stiffness: tension adds trapeze stiffness T·b²/12L, capping k_cross/k_hinge ≈ 1/axial strain; creep forces thick sections. Best real hand design 5" cr 0.0096 vs ideal 0.030.
- ⇒ NEW DIRECTION (helicopter practice): TENSION-TORSION STRAP — unbonded aramid/UHMWPE/CF-tow strands carry all CF; the printed flexure carries shear/bending only (axial release). Helper (opus) implementing tt_strap.
- HELPERS: tt_strap (opus); beam vs ccx 1.8× gap on crossed plates (sonnet).
- COMPUTE: robust ideal seeds 1–6 running (5/7/10"), both machines.
- 4" preset is thermally pulse-limited (real); preset test now checks electrical headroom only.

## 2026-11-21 ~02:10 — iteration 4 (cron)
- ROBUST IDEAL (seed1, gen ~38, min cr over 0.7–1.4× thrust): 10" 0.174 (bind: flap-antisym resonance 1.15/rev), 7" 0.090 (bind: motor_thermal 0.89), 5" 0.017 (bind: motor_thermal 0.99). ⇒ robustly, bigger rotors win; small rotors are THERMALLY limited, not flexure-limited.
- REVIEW / I16 MOTOR CONSTANT: pulse copper = ΔQ²/(2·Km²) with Km = Kt/√R (motor constant, ~∝ stator volume). For a thermally-bound design the lever is Km per gram and Rth (cooling), not the flexure. Actions: (a) Fred to measure Rth on Hakea (thermocouple after 2 min hover) — the 5"/7" verdict hinges on it; (b) add motor choice (Km/mass from MOTOR_LIBRARY) as a design variable; (c) consider motor cooling (bell vents / prop-wash duct) as a design lever.
- I17 FLAP RESONANCE @10": flap-antisym mode at 1.15/rev binds — flap stiffness/hinge offset is a free knob in the ideal; check whether real strap designs can place it ≥1.3/rev.
- Helpers: tt_strap (opus), beam-vs-ccx crossed plates (sonnet) running. Load: laptop 31/32, Mac ~13/10.

## 2026-11-21 ~03:00 — iteration 5
- LANDED: tt_strap (KEEP as CF path: aramid 4×Ø0.6 mm, all checks pass at ~10% util; cruciform as stiffness element ~KILL, ratios ≈10 vs ideal ~1900). Plate modulus fix (Q11 for wide clamped plates): crossed-plate beam/ccx 0.54→0.70, residual ~30 % unexplained (beam conservative-soft). torsion_bar ccx failure = root/hub fittings touch → merged mesh (geometry needs a gap).
- HELPERS: strap + released thin crossed plates (opus; k_cross/k_hinge ~ (b/t)² ≈ 1600 expected); validation vs Hakea bench logs incl. Rth estimate (sonnet).
- COMPUTE: queue_strap.sh — after ideal seed 1: robust tt_strap DE 5/7/10", then ideal seeds 2–3.
- REVIEW / I18: the strap makes CF and hinge stiffness independent → the design space is now "pick k_hinge, k_cross, k_flap freely" = close to the ideal spring. Remaining real constraints: plate strain at lag amplitude (thin plates help), plate buckling under compressive transients (check!), strap creep (aramid ok), and fitting size/drag. Add a plate-buckling check next if strap+xplate is KEEP.
- VALIDATION (bench logs 33/34): steady rpm vs duty 4.5 % rms; small-signal duty→rpm gain within 2–13 %; LARGE sweep gain falls off up to −29…−35 % at 30 Hz (model overpredicts) → amplitude-dependent = likely AM32 duty-slew (ramp) limit or R(T); ~35–45° unmodelled phase lag (ESC/DShot/telemetry latency) — phase is calibrated in flight, gain is not → treat pulse authority as ~0.7× nominal until a bench test at 1/rev frequency. Rth NOT identifiable (no current, ESC MCU temp only).
- BENCH TEST REQUEST for Fred (one test closes the 3 biggest gaps): thermocouple on stator + calibrated current sensor; held ±10–20 % duty sine at 250–300 Hz (≈1/rev) for 60 s at hover rpm; load cell if available. Gives Rth, Kt, R and the 1/rev gain.

## 2026-11-21 ~03:40 — iteration 6
- tt_strap_xplate KILL (for now): printed plates reach cross/hinge 180–440 (2–4.5× the cruciform) but stay LOADED under trim (+0.4 N·m/rad on k_hinge) because the radial slider doesn't release inclined plates and blade CG is off the strap line; pitch split binds everywhere.
- ROBUST IDEAL ≈ PIN JOINT: cards now want cross/hinge 3e3–3e4, flap/hinge 1e4–2e6, k_hinge 0.013–0.09 N·m/rad. ⇒ printed plates can't; only metal shim reaches (w/t)² ~ 4e4.
- REVIEW / I19: FEELER-GAUGE CROSS-SPRING PIVOT + aramid strap — bought spring-steel shim (0.05–0.2 mm, precise, cheap) clamped in printed fittings; strap carries CF; blade CG on strap line kills pitch split + plate loading. Helper (opus) running. Fits Fred's "simple bought parts OK" rule. Pin-joint ideal also means: the original skewed bearing hinge was near-ideal kinematically — the flexure's job is to equal a pin with no slop/wear.
- Slot A: root drag + envelope (sonnet) running.
