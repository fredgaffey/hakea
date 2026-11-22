# 5" crossed-strip flexure: cheapest off-the-shelf strip material

Basis: lead `5in_v8s_2307_m245_301` (9 x 0.05 x 4 mm 301 FH, L 10.05 mm, aramid strap). Each variant keeps
EI/L: n E w t^3 = K0 x Lf (K0 = 9 x 193e9 x 4e-3 x (0.05e-3)^3). Lf = 1.5 means free length x1.5 and
width x1.5. Presets are re-solved with `resolve_candidate(d, 6)`, and Goodman applies to metals only.
- Nominal screen: `study/materials_5in.py`, T0.7/1.0/1.25 at J_hub 0.91e-6 (the 2307_5_6in.md section 6
  screen). Raw results are in `results/materials_5in.json`. The `cr_pl` column is the minimum over the
  three thrusts.
- `sa.py`: `study/i34tools/sa_t125.py` (sa.py with THRUST_FACTORS 0.7/1.0/1.25, continuous nodes over
  0.7-1.25, SA_JHUB 0.34/0.91/2.0e-6, 1 worker). It reproduces the lead's 0.169 exactly.
- Model change (`materials.LAMINABLE_SHEETS` / `SHEET_METALS`): every dry sheet stack is now n*k1.
  Clamp Kt/fretting, clamp wrap and strip LTB now apply to all sheets (fr4_g10 included, so fr4 is now
  more conservative than before). Goodman applies to metals only. Steel results are unchanged.
- Material data are handbook typical and must be verified by coupon. PET fatigue (40 MPa) is a guess.
- Mass is for 4 strips (2 blades x 2), free span only.
- Hub fit (`geom:flexure_inside_hub`) caps the width at about 6 mm at L0 and about 5.1 mm at L x1.5.

## Ranked (feasible = no hard violation at T0.7/1.0/1.25)
| # | material (stock) | n x t x w (x L) mm | cr_pl nominal | sa.py | feasible | top binding check | mass 4 strips g |
|---|---|---|---|---|---|---|---|
| 1 | clock suspension steel, blued 1095 (127x76 sheet, hand-cut) | 3 x 0.075 x 3.43 | 0.214 | **0.158 FEAS** (res margin 0.65) | yes | motor_torque_1p (strip fatigue 0.31) | 0.24 |
| 2 | clock steel 1095 0.05 (hand-cut) | 9 x 0.05 x 3.86 | 0.221 | - | yes | motor_torque_1p | 0.55 |
| 3 | 301 FH slit 4 mm (made to order) = lead | 9 x 0.05 x 4.00 | 0.203 | 0.169 FEAS (res 0.27) | yes | fatigue_stress 0.99 @T1.25 | 0.57 |
| 4 | phosphor bronze C510 extra-spring 0.10 | 2 x 0.10 x 3.95 | 0.193 | **0.144 FEAS** (res 0.67) | yes | fatigue_stress 0.92 | 0.28 |
| 5 | POM (Delrin) sheet 0.25 | 3 x 0.25 x 5.98 | 0.188 | **0.139 FEAS** (res 0.52) | yes | motor_torque_1p (strip fatigue 0.19) | 0.26 |
| 6 | BeCu C17200 TH04 0.10 | 2 x 0.10 x 3.39 | 0.194 | - | yes | motor_torque_1p | 0.23 |
| 7 | BeCu 0.10, L x1.5 | 2 x 0.10 x 5.09 x1.5 | 0.204 | - | yes | motor_torque_1p | 0.51 |
| 8 | Ti-6Al-4V foil 0.10, L x1.5 | 3 x 0.10 x 3.81 x1.5 | 0.203 | - | yes | motor_torque_1p | 0.31 |
| 9 | PET film (Mylar) 0.19 | 5 x 0.19 x 5.63 | 0.192 | - | yes (fatigue GUESS) | motor_torque_1p | 0.30 |
| 10 | phosphor bronze model-railway 0.127 | 1 x 0.127 x 3.85 | 0.150 | - | yes | fatigue_stress 1.17 | 0.17 |
| 11 | SMT stencil 304 HTA 0.10, laser-cut (301 proxy) | 1 x 0.10 x 4.50 | 0.142 | - | yes | fatigue_stress 1.36 | 0.14 |
| 12 | K&S brass C260 H 0.075 | 4 x 0.075 x 4.68 | 0.125 | - | yes | fatigue_stress 1.46 | 0.48 |
| 13 | PCB FR4 0.2, CNC outline | 1 x 0.20 x 5.43 | 0.070 (t0.17: 0.052, k 0.61; t0.23: 0.075, k 1.52) | - | yes | fatigue_stress 2.36 | 0.08 |

## Infeasible or no k-match
| material | n x t x w (x L) mm | cr_pl nominal | hard fails |
|---|---|---|---|
| dental matrix band 0.038 x 5 (301 proxy, HARD temper assumed; dead-soft rolls useless) | 15/16/17 x 0.038 x 5 | 0.156/0.171/0.187 | harmonic_jitter 1.8 |
| dental band 0.04 x 5 / x 6 | 14 x 5 / 12 x 6 | 0.165 / 0.000 | jitter 1.9 / aeroelastic, jitter, pitch_cf |
| dental band 0.05 x 5 | 6/7/8 x 0.05 x 5 | 0.099/0.106/0.132 | aeroelastic, harmonic_jitter 2.4-2.5 |
| dental band 0.05 x 6, 0.038 x 6 | 6 x 0.05 x 6, 14 x 0.038 x 6 | 0.000 | aeroelastic, jitter, pitch_cf_ratio |
| clock 1095 0.075 wide / L x1.5 | 2 x 0.075 x 5.15; 3 x 5.15 x1.5 | 0.161; 0.017 | aeroelastic 1.48; resonance, pitch_cf |
| SMT 0.10 / 0.12 at L x1.5 (9 x 0.05 = lead) | 1 x 6.75, 2 x 3.37, 1 x 0.12 x 3.91 | 0.113/0.064/0.000 | hub+creep+resonance; jitter; jitter |
| any L x1.5 with w >= 5.7 (brass, PB, Ti 2x, POM 3x, PET, PP, FR4) and L0 w >= 9.4 (brass 2x, PET 3x) | - | 0.08-0.21 | geom:flexure_inside_hub |
| POM 1 x 0.5 x 3.36 x1.5, PP 2 x 0.5 x 3.47 x1.5 | - | 0 | trim failed, aeroelastic, creep (PP 3.0) |
| CF twill/reed-petal 0.25 x 3 x1.5 (k x2.2, the closest) | 1 x 0.25 x 3 | 0 | trim failed, interlayer 2.2, static |
| CF reed-petal 0.30 / 0.35; archery UD 0.38 | analytic | - | k-match w 0.54 / 0.34 / 0.13 mm (< 3) |
| CF pultruded 0.5 x 3, 0.6 x 3 / 5 | analytic | - | k x54 / x93 / x155 (strain ~10x lead) |
| E-glass bow lam 0.76 | analytic | - | k-match w 0.05 mm; k x61 at 3 mm |
| PCB FR4 0.4 / 0.6 | analytic | - | k-match w 0.68 / 0.20 mm |

## Verdict
1. Cheapest feasible option: **clock suspension steel (blued 1095) 0.075 mm, 3 hand-cut strips x 3.4 mm per
   flexure** (`designs/5in_clock1095_v0.json`). It scores sa.py cr_pl 0.158 FEASIBLE, close to the lead's
   0.169 and about 3x the 0.04-0.05 need, with a better resonance margin (0.65 vs 0.27). Stone the cut edges.
2. Next: phosphor bronze 0.10 x 2 (`5in_phosbronze_v0`, 0.144) and POM 0.25 x 3 (`5in_pom_v0`, 0.139).
   Both are cheap hobby stock and use n = 2-3 instead of 9.
3. Dental bands fail harmonic_jitter at their factory 5/6 mm widths. They only work slit to 4 mm, which is
   the lead. CF strip, reed-petal, archery and E-glass are too stiff to k-match at w >= 3 mm.
4. Precision-part options: an SMT stencil 1 x 0.10 x 4.5 (n = 1, 0.142 nominal; 304 HTA fatigue unverified,
   electropolish the laser edges). PCB FR4 0.2 is marginal at 0.052-0.075 over its +-0.03 mm tolerance.

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
