# Easy-build variant of v8 (12.7 mm 1095 roll, PETG, no damper/oil)
Screen = nominal T0.7/1.0/1.4 evals, presets re-solved, PETG (prusament_petg) blade+fittings. v8 cr_pl 0.0715 (full sa.py).
| variant (w12.7) | free L | screen cr_pl | creep util | note |
|---|---|---|---|---|
| 3x0.127 | 1.0x | 0.086 | 1.08 | creep/aeroelastic viol |
| 4x0.127 | 0.8x | 0.227 | 0.97 | |
| 1x0.15 | 0.8x | 0.079 | 1.02 | |
| 1x0.20 | 0.7x | 0.223 | 0.97 | |
| 1x0.25 (A, saved) | 0.8x | 0.229 | 0.954 | sa.py cr_pl 0.2294 |
| 2x0.15 (B) | 0.7x | 0.239 | 0.970 | sa.py cr_pl 0.2386 |
| 2x0.15 PETG-CF fittings | 0.7x | 0.257 | 1.16 | screen only |
Free length >=1.0x fails (creep, jitter, aeroelastic). hub_radius 10.5-12 mm no effect; 13.5 mm breaks flexure_inside_hub.
A and B are INFEASIBLE under default gates: cqc_mech_margin ~0.9 with mech_side flip, creep_strain 0.95-0.97, pitch/hub res margin 0.03.
Single 0.25 lamina: strip fatigue_stress util 1.05-1.27 at T1.4 (Goodman, sa 490-600 MPa).
Tolerance: screen cr_pl moves ~0.04 per 0.1x (2 mm) of free length, so +-1 mm hand length is NOT harmless unless a printed stop sets it.
Hinge radius +-0.2 mm printed = +-0.93 % of 21.5 mm (inside the +-1.5 % allowance). Shim +-1 mm scoring not run.
PETG fittings: no stress/fatigue fail seen; binding is PETG creep_strain (0.95-0.97). PETG-CF screened worse; PLA+ not run.
What Fred needs: PETG (Ender) hub/root fittings with printed bottom stops and over-long blind slots; 12.7 mm 1095 feeler roll (0.25 mm); 3 mm dowels; aramid strand; CA/epoxy; snips.
Steps: 1 print; 2 snip strips over-length; 3 drop strip into slot to the stop; 4 CA; 5 thread aramid; 6 pin-balance.
Status: no feasible easy design yet; next try 2x0.15 / 0.7x with PA-CF X1C fittings and the full sa gates.
