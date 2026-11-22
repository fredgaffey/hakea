# Whole-rotor FEA of the real CAD: 5in_v8s_2307_m245_301 (SolidCraft STEPs)

Scripts and data: `study/fea_cad/` (beam_ref.py, build.py, cases.py, post.py, report.py, run_all.sh / run2.sh,
results/*.json, results/hover_strip_fitting.png). Inputs are the SolidCraft STEPs (hub, blade1/2) plus the
strip frames from `cad.solidcraft.rotor_layout`. They are the same numbers that drew the STEPs.

## Model
- **Fittings:** hub, blade1 and blade2 are meshed in gmsh (OCC) with C3D10 elements: 0.5 mm in the root ball (r < 27 mm) and 2 mm outboard, 162k tets. The material is isotropic PA6-CF (7.5 GPa), PETG (2.0 GPa) or "STIFF" (210 GPa, which isolates the strips).
- **Strips:** each 9 x 0.05 mm dry stack is one hex20 (C3D20R) lamina, 0.05 mm thick, at the stack mid-plane, with E x 9 and rho x 9. Mesh is 40 x 8 elements over the free 10.048 mm. Per-lamina stress = model stress / 9. Element check: an isolated lamina cantilever gives 0.97 of M L / EI.
- **Glue:** the 0.325 mm gaps between the lamina and the slot faces are filled with epoxy hex20 layers (3 GPa) over the full 8 mm lands, *TIEd to the slot faces with zero offset. First attempt: tying the lamina faces across the gap with offset ties (translation-only MPC) is not rotation-invariant. It locked the pivot at about 1e4 x the true stiffness. Do not use offset ties for this.
- **Strap:** aramid strand (E 112 GPa, d 1.3 mm), one SPRINGA from knot well to knot well (11.2 mm, 13.3 kN/mm). Each end is averaged (*EQUATION) on the bearing half of the well wall, and the spring node sits at the centroid of that half. The strand line passes e = 0.06 mm from the hinge axis.
- **BCs and loads:** hub bore nodes encastre. Case 1: blade 1 is rigid outboard of r_aero + 3 mm, with a pilot node at the hinge point and 1 N mm pure moments about a (hinge axis), s and y. Hover: CENTRIF at 14079 rpm plus 2.40 N thrust (beam trim).

## Case 1: hinge / cross / flap stiffness (N m/rad), ratio to beam "as-CAD strips only" in ()
The CAD differs from the optimised design in three ways: straight coaxial slots (no slot presets), strips glued at both ends (no axial release), and rigid clamps. So the beam reference is `rotor.flexure_only_model` of that as-CAD variant (`beam_ref.json`).

| model | k_hinge (a) | k_cross (s) | k_flap (y) |
|---|---|---|---|
| beam as-CAD, strips only | 0.0157 | 56.0 | 32.9 |
| beam as-CAD, strips + strap | 0.0180 | 75.5 | 33.0 |
| beam DESIGN (bench_plan k_pitch, presets + axial release, unequilibrated misfit state) | 0.0995 | | |
| ccx strips, 210 GPa fittings | 0.0143 (0.91) | 33.6 (0.60) | 19.3 (0.59) |
| ccx strips, PA6-CF fittings | 0.0142 (0.91) | 9.5 (0.17) | 4.7 (0.14) |
| ccx strips, PETG fittings | 0.0141 (0.90) | 3.2 (0.06) | 1.5 (0.05) |
| ccx + strand, 210 GPa | 0.0216 (1.20 vs beam+strap) | 39.2 | 19.3 |
| ccx + strand, PA6-CF | 0.0157 (0.87 vs beam+strap) | 10.2 | 4.7 |

- **Pitch (hinge) stiffness:** ccx and the beam agree within 10 %. The FE rotation axis lies on the design axis to within 0.03 deg. Fitting material barely changes k_hinge (PETG costs 1.5 %).
- **Flap and cross stiffness:** the beam is >20 % off. Even with 210 GPa fittings, ccx gives 0.6 x the beam (epoxy lands, plate effects). With printed fittings, fitting compliance dominates: PA6-CF gives 0.14-0.17 x the beam, PETG 0.05-0.06 x. The beam's rigid-fitting assumption is badly wrong for flap and cross.
- **Bench plan:** the 0.0999 N m/rad in bench_plan_5in.md is 6-7 x too high for the printed part. Expect about 0.014-0.016 N m/rad, which is about 1.8 deg per gram at the 50 mm arm, not 0.28. The strand adds k_strand x e^2, which is 0.05 N m/rad at e = 0.06 mm. The strand position in its 1.6 mm channel (+-0.15 mm play) can therefore change the measured pitch stiffness several-fold: pretension and position matter on the bench.
- **Mesh convergence:** see "Convergence" below.

## Case 2: hover static (CF + thrust)
The NLGEOM run cut its increment to 1/256 and was stopped after 61 min; it could not finish. The LINEAR run is reported instead, and it is **not valid for the strip, glue or fitting stresses**:
- Linear CF has no pendulum (CF) restoring stiffness about the 0.014 N m/rad hinge.
- The blade therefore swings in lag/pitch: the tip lag displacement is 12-17 mm (about 0.25 rad).
- The strip stress is bending from that swing, not CF tension. Membrane is only 0.4-12 MPa per lamina, so the assumption that "CF tension dominates" does not hold here.

The linear numbers are listed only as inflated upper bounds:

| quantity | PA6-CF | PETG | beam as-CAD (design) |
|---|---|---|---|
| per-lamina strip surface stress at the mouths, MPa | 311-595 | 358-624 | 35-156 (57-184) |
| epoxy land shear, max / mean, MPa | 4-30 / 0.6-3.3 | 5-48 / 0.5-3.5 | n/a |
| blade fitting vM, peak (99.9 %ile), MPa | 36-39 (20-23) | 36-45 (21-24) | n/a |
| hub vM, peak (99.9 %ile), MPa | 497 (148) | 469 (155) | n/a |

- **Hub peak:** it sits at the edge of the strand-knot averaging set (an MPC artifact).
- **No valid fitting safety factor:** even as upper bounds, the PA6-CF blade fitting gives SF about 2.4 (peak) / 4.4 (99.9 %ile) on Xt 110, and PETG gives SF about 0.8-1.1 (peak) / 1.8-2.0 on Xt 45. These are not trustworthy until the hover state is solved with NLGEOM.

## Case 3: modes
- **At rest, PA6-CF, linear:**
  - 34.3 / 35.1 Hz: hinge, one per blade (rotation about the skewed axis a, which shows mostly as tip lag);
  - 358.5 / 359.6 Hz: flap;
  - 862 / 912 Hz: in-plane;
  - 1223 Hz: flap 2.
- **At hover:** the CF-prestressed *FREQUENCY run (c3_PACF: NLGEOM static, then a FREQUENCY step) is capped at 12 min and is not expected to converge (same NLGEOM problem as case 2). Status at hand-back: see the last line.
- **No hover comparison possible yet:** the beam hover modes are pitch 0.616 / 0.787 /rev (141 / 181 Hz), flap 1.71 / 1.74 /rev (392 / 399 Hz) and pitch-lag 4.1 / 4.8 /rev. These are set by CF, so a non-rotating FE run cannot check them. What can be said: the FE flap frequency at rest with PA6-CF fittings (359 Hz) is already below the beam's hover flap (392 Hz), because k_flap is 0.14 x the beam.

## Verdict
- **Hinge (pitch):** the beam model is right within 10 % for the real connected CAD, and the printed fittings do not matter for k_pitch.
- **Flap and cross:** the beam overstates stiffness 6-20x with printed fittings (>20 % off). The flap/lag mode placement (and therefore the Campbell and stability margins) must be re-checked with fitting compliance, especially in PETG.
- **The 0.0999 bench k_pitch** comes from the unequilibrated preset state of the design, not from this part.
- **Hover stresses still unresolved:** they need either a converged NLGEOM whole-rotor run (fixed increments plus stabilisation, or a strap pretension step first) or the strip-only sub-model with beam end kinematics (`study/ccx_preset_strip.py`). The linear whole-rotor hover run is not usable for stresses on this soft-hinge rotor.

## Convergence
Case 1 with PA6-CF fittings: refining the mesh from h 0.5 mm with 40 x 8 strip elements (162k tets) to h 0.36 mm with 60 x 12 (398k tets) changes k_hinge 0.01422 -> 0.01421 (-0.02 %), k_cross 9.50 -> 9.52 (+0.2 %) and k_flap 4.688 -> 4.694 (+0.1 %). Case 1 is mesh-converged.

Status at hand-back: the CF-prestressed modal run c3_PACF (NLGEOM step with *STATIC, DIRECT 0.5) is still running on the Mac, capped at 12 min (/tmp/fea/5in_301/m05/c3_PACF.*). It is not in this report.
