
## SMT stencil 304 HTA re-screen (steel_304_hta, replaces the 301 FH assumption)

Stencil foil is 304 hard-tempered, not 301 FH. `steel_304_hta` (materials.py): E 193 GPa, UTS 1.2 GPa,
fatigue 0.30 x UTS polished (360 MPa at 1e7-1e8) divided by Kt 1.15 for laser-cut edges = 313 MPa
(0.26 x UTS, eps_a 0.162 %), Su 1.2 GPa for Goodman. All handbook-typical and unverified; the fatigue
number is the weak point (stencil edge HAZ/recast is not characterised). Same lead geometry, k-matched
width, presets re-solved, T0.7 / 1.0 / 1.25, J_hub 0.91e-6, Goodman.

| variant | w mm | cr_pl T0.7 / 1.0 / 1.25 | feasible | note |
|---|---|---|---|---|
| smt304_1x100 (L 1.0) | 4.50 | 0.164 / 0.137 / 0.089 | yes | was 0.308/0.222/0.142 as 301 FH |
| smt304_1x100_L125 | 5.62 | 0.137 / 0.103 / 0.089 | no | aeroelastic stability, split |
| smt304_1x120 (L 1.0) | 2.60 | 0.158 / 0.128 / 0.109 | yes | lightest, 0.099 g |
| smt304_1x120_L125 | 3.26 | 0.202 / 0.162 / 0.128 | yes | best margin |
| smt304_1x80 (L 1.0) | 8.79 | 0.201 / 0.154 / 0 | no | flexure inside hub, pulse infeasible at 1.25 |
| smt304_1x80_L125 | 10.99 | 0.209 / 0.091 / 0.095 | no | many checks fail |

Verdict: a single 0.10 or 0.12 mm 304 stencil strip still passes all three throttles, but with
roughly half the cr_pl of the 301 FH figure (min 0.089 vs 0.142 for 0.10 mm). The binding check is
fatigue_stress throughout, so the result is as good as the 0.26 x UTS edge-derated allowable. 0.12 mm
at L x1.25 (3.26 mm wide) is the best candidate (min cr_pl 0.128); 0.08 mm is out. Order a coupon
batch with a polished/electropolished edge option and fatigue-test before trusting any of this.
Next: DE job `stencil_de1` (steel_304_hta, n 1-3, stencil stock index) in campaign/jobs.txt.
