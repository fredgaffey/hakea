# Strip clamp-edge stress (ccx_clamp_edge.py)
Model: 2D plane-strain tri6 CalculiX, one 0.10 mm 1095 lamina between two jaws (frictionless surface contact, 20 MPa bolt
pressure, jaw exit radius r, 12 um mesh at the edge). Free end: v8 hub-clamp beam surface bending 143 MPa (sm 150/Kt 1.05) + 1 MPa axial.
K_clamp = max |S11| on strip surface, x = -2..+1 mm, / 143 (load-linear while contact status holds). x<0 = under the jaw; ~+1 = no edge peak.

| jaw E (GPa) | r=0 (0.02, mesh-limited) | r=0.5 | r=1.0 | r=2.0 |
|---|---|---|---|---|
| steel doubler 200 | 1.28 @ -0.03 | 1.03 @ -0.4 | 0.97 (none) | 0.96 (none) |
| PA-CF 8 | 1.19 @ -0.02 | 1.04 @ -0.4 | 0.97 | 0.96 |
| PETG 5 / 2 | 1.12 / 1.11 @ -0.01 | 1.02 / 1.04 @ -0.4 | 0.97 | 0.95 |

Sharp: peak ~1 radius inside the jaw edge; soft jaws relieve it (1.1-1.2 vs 1.28 steel). For r >= 1 mm (10 t) stiffness is irrelevant.
Goodman (rotor.py KT_*): sharp Kt 1.75 / fret 0.80, radiused (R>=10 t) 1.10, compliant 1.05 (v8: compliant). FE <= 1.03 for r >= 0.5,
so assumed Kt is conservative; the 1.75 knife edge is not contradicted (FE sharp is radius-limited).
Not modelled: friction/fretting, preload creep, edge burrs (0.5-0.6 x polished fatigue, flexure_manufacturing.md), which dominate elastic Kt.
Recommendation: jaw/steel-doubler exit radius >= 1.0 mm (0.5 acceptable, +3 %), bonded steel doubler preferred (no creep/fretting
from plastic preload); forbid sharp (<0.5 mm) jaws. Goodman Kt/Kf: no change needed (v8 1.05 vs FE ~1.0 for r >= 1 mm).
