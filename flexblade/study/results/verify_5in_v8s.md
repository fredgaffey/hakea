# CalculiX verification — 5in_v8s_2307_m245 (8 × 0.05 mm 1095, w 4.5 mm)
Script: study/ccx_preset_strip.py (TF env sets thrust factors). Raw: results/verify_5in_v8s_2307_m245.json.
Written by the main session from the helper's report (helper did not commit this note).

| term | ccx/beam |
|---|---|
| k_flap | 1.06 |
| k_cross | 1.06 |
| kx / ky / kz | 1.05 / 1.01 / 1.02 |
| k_hinge | 0.95 (near-zero axis-noise term; ignore) |

Stiffness PASS (< 20 %). Mesh nl20/nw16/nt2 → nl40/nw24/nt4: all ratios unchanged to 3 digits (converged).

Strip stress, worst strip (MPa, beam vs ccx):
| case | loc | beam | ccx | ccx/beam |
|---|---|---|---|---|
| T1.0 | hub | 205 | 71 | 0.35 |
| T1.0 | blade | 161 | 47 | 0.29 |
| T1.25 | hub | 278 | 77 | 0.28 |
| T1.25 | blade | 210 | 52 | 0.25 |
Beam is 3–4× conservative. Likely cause: ccx strips are straight and stress-free; the beam model carries the preset/slot-misfit curvature. Unresolved (todo: preset-curved ccx strip).

Modelling: ONE 0.05 mm lamina meshed, scaled ×8 analytically (unscaled ratio 0.13 ≈ 1/8) → assumes FREELY SLIDING laminae, no contact. A bonded stack is up to n² stiffer in bending → build rule: laminae dry, epoxy on lands only, no thin CA.
