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
Steps: 1 print; 2 snip strips over-length; 3 drop strip into slot to the stop; 4 thick epoxy on lands only (laminae dry/sliding in the free span, no CA); 5 thread aramid; 6 pin-balance.
Status: no feasible easy design yet; next try 2x0.15 / 0.7x with PA-CF X1C fittings and the full sa gates.

## Attempt 2 (12.7 mm 1095 roll, n laminae stacked, PETG blade, PA-CF (pa6_cf) fittings, aramid strand + dowels)
Screen = nominal T0.7/1.0/1.4, CQ_TOL=datumed, presets re-solved. Matching v8 n*w*t^3 needs n=6 at t=0.10 (t=0.075 needs 13, t=0.05 needs 45: >8, so those run softer and need a shorter free length).
| variant (w12.7) | free L | screen cr_pl | violations |
|---|---|---|---|
| 5x0.10 | 1.0/1.1/0.95/0.9x | 0.058/0.046/0.066/0.075 | hub-fit, resonance |
| 6x0.10 | 1.0/1.1/0.95/0.9x | 0.070/0.054/0.079/0.090 | hub-fit, resonance |
| 7x0.10 | 1.0/1.1/0.95/0.9x | 0.076/0.057/0.089/0.105 | hub-fit, resonance |
| 8x0.075 | 1.0x | 0.047 | jitter, aeroelastic, resonance |
| 8x0.075 | 0.92x | 0.061 | jitter, hub-fit |
| **8x0.075 (A, saved)** | **0.88x** | 0.069 | none |
| 8x0.075 (B) | 0.90x | 0.066 | none |
| 8x0.075 PETG-CF fittings | 0.90x | 0.057 | jitter |
| 8x0.05 | 0.7x | 0.045 | jitter, aeroelastic, resonance |
The t=0.10 stacks score higher but trip the 1P resonance margin and flexure_inside_hub; only 8x0.075 at 0.88-0.90x is clean.

Full `sa.py file` (default gates, CQ_TOL=datumed, SA_WORKERS=2), higher cr_pl is better:
| design | cr_pl | feasible |
|---|---|---|
| v8 baseline | 0.0772 | yes |
| A 8x0.075, L 0.88x (designs/10in_easy_steelx.json) | 0.0692 | yes (-10 % vs v8) |
| B 8x0.075, L 0.90x | 0.0660 | yes |

Tolerance (A, free length +-1 mm, presets fixed): -1 mm gives cr_pl 0.0771 (+0.008) but resonance viol 0.057, INFEASIBLE; +1 mm gives 0.0624 (-0.0068) with flexure_inside_hub viol 0.001 (marginal). Hand seating is not safe. Printed-stop requirement: a bottom stop that sets free length to about +-0.3 mm, biased 0.3-0.5 mm long (shorter is the dangerous side), with an over-deep blind slot so the stack bottoms on the stop and the glue land absorbs cut-length error.

What Fred needs:
1. Print PA-CF (X1C) hub/root fittings with a bottom stop and a slot for an 8-lamina 0.075 mm stack (~0.6 mm plus clearance).
2. 12.7 mm 1095 feeler roll, 0.075 mm (0.003"); snip 16 pieces over-length.
3. Dry-stack 8 per strip, drop to the stop at both ends.
4. Thick epoxy on the LANDS only; laminae dry (free to slide) in the free span: no thin CA (wicks, bonds the stack, k up to n² ×).
5. Aramid strand and 3 mm dowels.
6. PETG blades, pin-balance.
Caveat: gates pass in the model only; no bench data.
