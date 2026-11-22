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

## Preset-curved strip
`DN=5in_v8s_2307_m245_301 TF=1.0,1.25 python study/ccx_preset_strip.py preset` (301 FH, 9 x 0.05 x 4 mm, binding b1.flex0, Kt 1.05): NLGEOM step 1 = beam stopped (slot misfit only) end motion, step 2 = hover; CCXMODES mixed | inplane | loads.
- Hypothesis FALSE. The script already imposed F = R_node.Q_slot (preset is in the end motion). The preset part is small, and a pre-bent strip gives the same hover result as a straight one.
- Beam split, T1.0 hub surface MPa: preset alone 39.5 (oop); hover 188.5 = oop 144 + in-plane 43 + membrane 1 (x Kt -> Goodman sm 198); 1/rev alternating sa 230. T1.25: 257 (oop 208, in-plane 48), sa 290.
| hub / blade (ccx/beam) | beam | ccx mixed (u_y, th_z free, M3 = 0) | ccx + beam in-plane end loads |
|---|---|---|---|
| preset only | 39.5 / 32.8 | 47 / 40 (1.20 / 1.21) | 47 / 40 (1.20 / 1.21) |
| T1.0 | 189 / 144 | 64 / 47 (0.34 / 0.33) | 168 / 165 (0.89 / 1.15) |
| T1.25 | 257 / 189 | 69 / 52 (0.27 / 0.27) | 192 / 191 (0.75 / 1.01) |
- Cause of the 0.25-0.35: the mixed BCs drop the in-plane moment. The beam's blade-end M3 is 3.8-4.0 N mm per lamina (~0.6 of a rough clamped M_cr,LTB ~6). With it applied as CLOAD, the total agrees within -25..+15 %. Imposing the beam's u_y / th_z as displacements is ill-conditioned (in-plane 6400x stiffer): 333-471 MPa.
- The split still differs: ccx mid-width (~oop) is 59-68 vs beam oop 144-208, and the ccx edge excess is 105-125 vs beam in-plane 43-48. The totals match but the split does not (LTB coupling vs 3D clamp edge; unresolved).
- Verdict: the beam is right on total clamp stress to ~25 %. It is conservative at the hub, and at most 15 % unconservative at the blade end. It is NOT 3-4x conservative. The preset/mean state agrees (ccx 1.20x, plate E/(1-nu^2) + clamp Poisson restraint). The alternating sa is not checked by static ccx.
- Goodman (beam flex0): T1.0 g 1.18, util 0.635; T1.25 g 1.27, util 0.855. The mean enters only through g = 1/(1 - sm/Su). With the ccx+loads sm, util is 0.62 / 0.80 (-2 / -7 %). Even the old ccx-mixed sm only gives 0.57 / 0.71. The DE binding constraint is effectively unchanged.
