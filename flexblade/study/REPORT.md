# flexblade design study — report (partial, 2026-11-20)

Stopped early (usage limit). Search depth: run 1 = all 20 size × concept cases, 40 DE
generations; PA6-CF re-run (cross_gap + symmetric topology) ≈ 25 gens; coaxial ≈ 20 gens;
TPU + radial rod and torsion bar = 1 generation only (feasible design exists, not optimum).

Metric: control ratio = hub moment / (W·R); pulse at 0.95 × motor 1/rev capability;
AM32 max_ramp 255.

| Design | All checks pass | Control ratio | Tilt | 1/rev pitch |
|---|---|---|---|---|
| **5" torsion bar** (CF bar on hinge axis + TPU plate + radial CF strap) | yes | **0.034** | 2.6° | 1.3° |
| torsion bar 10" / 4" / 12" | yes | 0.032 / 0.020 / 0.010 | | |
| **7" PA6-CF symmetric crossed plates** (`skewed_cross_blade_sym`) | yes | **0.022** | 1.7° | 1.0° |
| symmetric plates 10" / 4" / 5" | yes | 0.017 / 0.011 / 0.009 | | |
| PA6-CF offset crossed pair; TPU core + CF skin | yes, all sizes | ≤ 0.003 | | |
| TPU + CF rod inserts | no (creep 1.6–2.1×, under the OLD creep check) | | | |
| monolithic TPU | no, any size | | | |
| ideal skewed pin hinge (reference) | – | 0.018 / 0.024 / 0.058 / 0.053 at 4/5/7/12" | | |

10" pin-hinge reference (0.52) unusable: a mode at 0.99/rev. Feasible-but-stiff designs
(lag mode 2–4/rev) only reach ≈ 0.001–0.003.

## Verdicts
1. **Single tuned TPU blade: no, at any size.** Root strain from the blade's own centrifugal
   load ≈ ρV_tip²/2E ≈ 7–12 % at hover tip speeds 105–135 m/s vs 1.5 % creep allowable —
   set by tip speed, not size.
2. **Winner: split load path** — a stiff member carries centrifugal load (radial CF strap/rod),
   a separate element is the hinge. Torsion bar best found; symmetric PA6-CF crossed plates
   best-searched runner-up. Creep strain governs almost everywhere.
3. **Best size 5–7".** 12" current-limited (pulse ≈ 0.9 × hover torque). 4–5" need AM32
   max_ramp raised (stock 2.7–3.5× over); with 255 all sizes fine.
4. **Single vs coaxial unresolved.** Coax runs reached only 0.003–0.005; the best single-rotor
   flexure in the coax layout gives 0.029 at 7"/10" but fails on resonance at the lower rpm.
5. **Print first: `designs/5in_torsion_bar.json`** — skew 46.3°, hinge radius 8.5 mm, CF bar
   Ø1.24 × 6.8 mm on the hinge axis, TPU-64D plate 1.31 × 5.2 × 7.9 mm, radial CF strap
   Ø0.86 mm, PA6-CF blade, blade attachment line at 0.434 c. Max utilisation 0.89 (ESC duty
   peak); nearest mode 0.82/rev. Backup: `designs/7in_cf_flexure_sym.json`.
   Hand tests at 10" had the strap 4–8× over; these optimised points pass — FEA-check the 5"
   design before printing.

## Bench tests that most reduce uncertainty
Creep of PA6-CF, TPU and CF rod at 40–60 °C; spin test of the 5" torsion-bar hub (stretch,
pitch split); motor inductance + delivered 1/rev torque; TPU DMA 100–450 Hz; ping/Campbell
test for the lag-sym + hub mode.

## Changes made during the study
- Pitch split fixed by `blade.elastic_axis_chord_frac` ≈ 0.43 (chordwise mass balance);
  all reported designs < 1.7°.
- New topology `skewed_cross_blade_sym` (plain offset pair turns centrifugal load into ≈ 0.5 %
  in-plane bending strain).
- Precone tried: does not relieve the governing strain; reverted.
- `rotor.checks` creep now uses each fibre's own allowable (old check judged TPU against the
  CF rod's 0.8 %); 10" rod-insert drops 2.1× → 1.02× — rod-insert not re-optimised since.
- HostPool: persistent per-host readers (second call used to hang), thread-safe, serial
  numba/BLAS in workers; Mac code in a private dir.
- `optimize.run` accepts a HostPool mapper; `api.optimize(progress, mapper, hosts)`; CLI
  `optimize --hosts`. New field `Flexure.tension_rod_diameter`.

Files: `run_study.py`, `ideal_hinge.py`, `analyze.py`, `results/` (summary_table.md,
cases.jsonl, evals.jsonl.gz), `designs/*.json`, `figures/*.png`.
