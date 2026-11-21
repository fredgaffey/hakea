# Twin-washer motor mounts sized for hub-tilt stability (7in v6b, 10in v5c)

Floquet with hub tilt + gyroscopics (stability.Mount.tilt), T {0.7,1,1.4} x J_hub {2.5,3.76,5.49}e-6 x arm {190,225,260} Hz (f_y = 1.15 f_x), airframe zeta 0.02. Script: study/mounts_tilt.py (raw: results/mounts_tilt_*.json), docs: study/mounts_tilt_docs.py. Rule: k_t >= margin x Omega_max^2 (I_t - J_p), Omega_max = 1P at T1.4; design margin 2.5x evaluated at 2x the estimated motor I_t (= 0.5 J_p + m (15 mm)^2).

## 7in v6b
- washers ri 4 / ro 8 / t 22 mm x2, centres 76 mm; mass on mount 55 g
- k_lat 35.6 kN/m, k_t 177 N m/rad, axial 121 kN/m
- rule: Omega_max 1636 rad/s, J_p 7.64e-06, est. I_t 1.62e-05; need 2.5 x rule(2 I_t) = 166; margin built/need = 1.07; margin vs rule at estimated I_t = 7.7x, at 2x I_t = 2.7x
- hub mode 122.1-125.6 Hz = 0.67-0.69 x lowest 1P (183 Hz), zeta 0.067-0.072
- Floquet min zeta (9 T x J x 3 arms): I_t x1 0.0043, x2 0.0116, x3 0.0123

Min zeta per arm and I_t:

| arm Hz | x1 | x2 | x3 |
|---|---|---|---|
| 190 | 0.0043 | 0.0116 | 0.0123 |
| 225 | 0.0061 | 0.0210 | 0.0213 |
| 260 | 0.0076 | 0.0229 | 0.0234 |

Min k_t vs measured motor I_t:

| measured motor+hub I_t (kg m^2) | min k_t at 2x margin (N m/rad) | at 2.5x |
|---|---|---|
| 8.10e-06 (0.5x est.) | 2 | 3 |
| 1.62e-05 (1x est.) | 46 | 57 |
| 3.24e-05 (2x est.) | 133 | 166 |
| 4.86e-05 (3x est.) | 219 | 274 |
| 6.48e-05 (4x est.) | 306 | 382 |

## 10in v5c
- washers ri 4 / ro 5.8 / t 12 mm x2, centres 84 mm; mass on mount 126 g
- k_lat 24.0 kN/m, k_t 145 N m/rad, axial 82 kN/m
- rule: Omega_max 952 rad/s, J_p 2.55e-05, est. I_t 4.11e-05; need 2.5 x rule(2 I_t) = 128; margin built/need = 1.13; margin vs rule at estimated I_t = 10.3x, at 2x I_t = 2.8x
- hub mode 68.2-68.9 Hz = 0.63-0.64 x lowest 1P (108 Hz), zeta 0.072-0.074
- Floquet min zeta (9 T x J x 3 arms): I_t x1 0.0089, x2 0.0089, x3 0.0089

Min zeta per arm and I_t:

| arm Hz | x1 | x2 | x3 |
|---|---|---|---|
| 190 | 0.0089 | 0.0089 | 0.0089 |
| 225 | 0.0089 | 0.0089 | 0.0089 |
| 260 | 0.0089 | 0.0089 | 0.0089 |

Min k_t vs measured motor I_t:

| measured motor+hub I_t (kg m^2) | min k_t at 2x margin (N m/rad) | at 2.5x |
|---|---|---|
| 2.05e-05 (0.5x est.) | 0 | 0 |
| 4.11e-05 (1x est.) | 28 | 35 |
| 8.22e-05 (2x est.) | 103 | 128 |
| 1.23e-04 (3x est.) | 177 | 222 |
| 1.64e-04 (4x est.) | 252 | 315 |


## Planar pad-ring alternative (study/build/mount_ring.py, study/mounts_tilt_ring.py; raw results/mounts_tilt_ring.json)
N = 4 round TPU pads (infill 0.4, G 6.5 MPa) on radius R between a motor plate and an arm plate; tilt = pad compression/tension k_t = (N/2) k_c R^2, k_c = E_c A/h, E_c = 3.4 G inf (1+2S^2), S = D/4h; lateral = pad shear N inf G A/h.
Key finding: k_t/k_lat = (R^2/2)(E_c/G) only; shear needs A/h ~ 3 mm, so pads must be thin columns (D 5, S 0.15-0.2, bulge gain only 1.1x) and R must be large (>= ~50 mm) to meet the rule k_t.  The shape-factor trick does not help because a large S forces h < 0.3 mm at this lateral stiffness.

| | 7in twin | 7in ring (R 52, D5, h6) | 10in twin | 10in ring (R 56, D5, h8) |
|---|---|---|---|---|
| k_t (rule need) N m/rad | 177 (166) | 170 | 145 (128) | 143 |
| k_lat kN/m | 35.6 | 34.0 | 24.0 | 25.5 |
| hub mode / 1P | 0.67-0.69 | 0.66-0.67 | 0.63-0.64 | 0.65-0.66 |
| axial sag at 15 / 25 N thrust (assumed) | ~0.12 mm | 0.12 mm | ~0.30 mm | 0.27 mm |
| min zeta, I_t x1 / x2 / x3 | 0.0043 / 0.0116 / 0.0123 | 0.0046 / 0.0040 / 0.0052 | 0.0089 all | 0.0089 all |
| height | ~104 mm | ~12 mm + 34 mm clamp | ~102 mm | ~14 mm + 34 mm clamp |
| footprint | 30 mm | 120 mm dia | 30 mm | 128 mm dia |
| printed mass (solid-ish PLA, est.) | ~43 g | ~90 g | ~28 g | ~100 g |

Verdict: the ring is much shorter but NOT better: 2-4x the mass (adds to arm tip mass and I_t), a 120-128 mm plate in the downwash, min zeta at 7in worse (0.004 vs 0.012 at 2x I_t, 190 Hz arm; equal at 10in, where the 0.0089 floor is not mount-limited), and the pad columns need T-slot retention for tension (tilt puts half the pads in tension). Twin-washer files kept; ring STLs not shipped (generator `python study/build/mount_ring.py build` writes watertight plates/pad if wanted).


## I30: steel XY flexure stage + TPU shear pads (study/build/mount_xy.py, study/mounts_tilt_xy.py; raw results/mounts_tilt_xy.json)
Two nested orthogonal parallelograms (4 horizontal 1095 shims per stage, 8 total; t 0.08, w 10 along the motor axis, free L 12, pair spacing 24), motor on the inner block.  Pads (4 TPU columns in shear, h 3, D 3.5 at 7in / 2.8 at 10in) carry 90-93% of k_lat, so they set zeta; the blades only guide.  Analytic (7in / 10in; blades identical):
- k_lat 35.6 / 24.0 kN/m (pads 33.2 / 21.6, blades 2.4); hub mode f_x 128 / 69 Hz, f_y 118 / 67 Hz (10 g middle frame on y); zeta = 0.070 / 0.068 (pad eta 0.15 x pad fraction; blades 0.001).  zeta 0.07 needs the pad to carry >90% of the stiffness: a "thin pad in parallel" cannot damp a stiff steel stage (zeta = 0.075 x pad fraction).
- k_t 1290 N m/rad (stage1 1670 in series with stage2 5100) = 7.8x / 10x the rule (166 / 128); run in Floquet at 3x derate (430) for clamp/FDM compliance.  k_axial 5.9 MN/m (twin 121 kN/m).
- yaw about the motor axis: k_theta 3.8 kN m/rad (blade axial, lever 12 mm, two stages in series) -> 2.2 kHz (7in) / 1.3 kHz (10in) with J_p included, still 700 / 425 Hz with a 10x clamp knockdown, vs 1P 183 / 108 Hz.  Motor reaction torque ~0.15 N m twists it ~40 urad.
- blade stress: 3 g lateral = 0.054 / 0.167 mm -> 18 / 56 MPa; 1P imbalance (1 g mm assumed) 26 / 12 um -> 9 / 4 MPa (far below fatigue).  Thrust (15 / 25 N) is in-plane shear of the blades, 3.9 / 6.5 MPa vs shear-buckling 93 MPa (margin 24 / 14).  Caveat: with blades on both sides of a block, arc shortening locks the stage axially: ~0.6 E A d^2 / L^2 = 1.9 N (7in) but 18.5 N (10in) at 3 g vs Euler 23 N per blade -> at 10in use one blade pair per stage side only, or w 12 / t 0.1 (more guide, less zeta).
- Floquet (T x J x arm, k_lat x1.09 to compensate the 10 g middle frame, derated k_t 430): 7in min zeta I_t x1/x2/x3 = 0.0014 / 0.0065 / 0.0023 (190 Hz arm; twin 0.0043 / 0.0116 / 0.0123); with +5 g and no k compensation the 190 Hz arm goes unstable (-0.003): the 7in hub is sensitive to mount mass.  10in = 0.0089 floor in all cases (same as twin and ring).  Hub mode 0.64-0.66 x 1P.

| | 7in twin | 7in ring | 7in XY | 10in twin | 10in XY |
|---|---|---|---|---|---|
| height | 104 mm | 12+34 | 15 mm | 102 mm | 15 mm |
| footprint | 30 mm | 120 dia | 79 x 74 mm | 30 mm | 79 x 74 mm |
| printed mass (est., 50% fill PETG) | 43 g | 90 g | ~26 g + 3 g steel (+10 g moving middle frame) | 28 g | ~26 g |
| k_t margin vs rule | 1.07x | 1.02x | 7.8x (2.6x derated) | 1.13x | 10x (3.4x derated) |
| min zeta (I_t x2) | 0.0116 | 0.0040 | 0.0065 | 0.0089 | 0.0089 |

Verdict: the XY stage fixes height and k_t margin (7-10x, tilt/yaw/axial stiff by kinematics) but not the 7in tilt-Floquet damping floor, which is set by hub mass and arm frequency rather than by k_t, and it costs a 79 x 74 mm plate in the downwash.  It is better than the twin only if height is the constraint; at 10in it is a tie on zeta.  Build: 3 FDM frames (slots 0.38 mm open at the top for dropping the shims in, CA, no print pause needed if the top is left open), 8 shim strips 19 x 9.6 mm, 4 glued TPU pads.  STLs (watertight): study/build/{7in_v6,10in_v5}/xy/.  Env knobs of the scan: XY_DERATE, XY_KSCALE, XY_DM.


## I31: diagnosis of the 7in "zeta floor" and fix (study/mounts_tilt_diag.py, mounts_tilt_diag_crpl.py; raw results/mounts_tilt_diag*.json)
**Worst point (twin, T1.4, J 3.76e-6, arm 190 Hz, I_t x1):** s = -11.1 + 953i rad/s, rotating 151.7 Hz = 0.58/rev (1P 260 Hz).  Participation (kinetic): basis mode 1 (149.5 Hz, 0.57/rev, cyclic LAG with pitch coupling: tip lag -1/+0.98, pitch*c 0.48/0.47 in blade axes) 0.35, hub x 0.40, hub y 0.22, tilt 0.00, everything else < 0.02.  Fixed-frame hub harmonics: 96 % at |w - 1P| = 108.6 Hz (regressing), 4 % at w + 1P = 412 Hz.  It is classic ground resonance: the regressing lag (1 - 0.58) 1P = 108.6 Hz coalescing with the hub-on-mount translation mode (114-116 Hz at arm 190; x and y both, f_y 1.15 f_x matters little: iso 190 gives the same root).  It is NOT a tilt mode and not the arm mode itself.
**The 0.001-0.005 "floor" was a metric artefact.**  stability.frame_zeta took the fixed-frame frequency as the 1P alias nearest any support frequency; the tilt mode (k_t 177, I_t x1) sits at 411.8 Hz = w + 1P, so it divided Re s by 412 Hz instead of 108.6 Hz (zeta 0.0043 vs true 0.0163, 3.8x).  That is why it moved with I_t (x2 detunes the tilt mode) and not with k_t "above the rule".  Fixed in stability.frame_zeta (alias = dominant 1P harmonic of the hub/tilt motion when > 75 %; old rule otherwise); tests/test_stability.py pass.  Re-run twin 7in (T x J x arm, I_t x1/x2/x3): min zeta 0.0137 / 0.0116 / 0.0123 (190), 0.023 / 0.021 / 0.021 (225), 0.027 / 0.023 / 0.023 (260): passes 0.005 everywhere, unchanged design.  Ring 7in re-run: 0.0065 / 0.0040 / 0.0052 at 190 (real, heavier mount), >= 0.0106 at 225/260.  XY 7in at default env knobs (XY_KSCALE 1, +10 g frame) is truly unstable at 190 (-0.0034): mass on the mount, see (d).  10in unchanged (0.0089).
**Real constraint: the arm band** (dominant-harmonic zeta, twin, f_y = 1.15 f_x, zeta_af 0.02):

| rigid-mount arm f_x Hz | 60 | 80 | 100 | 110 | 120 | 130 | 140 | 150 | 160 | 170 | 180 | 190 | 225 | 260 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| T0.7 worst (J, I_t) | .024 | **-.014** | .027 | .027 | .027 | .027 | .027 | .027 | .027 | .027 | .027 | .027 | .027 | .027 |
| T1.0 | .025 | .012 | **-.013** | .004 | .031 | .031 | .031 | .031 | .031 | .031 | .031 | .031 | .030 | .030 |
| T1.4 | .024 | .025 | .008 | .001 | **-.009** | **-.013** | **-.010** | **-.006** | -.001 | .004 | .008 | .012 | .021 | .023 |

zeta_af 0.02 -> 0.08 at 190: 0.016 -> 0.021 (weak); k_t 100-430: 0.016-0.017 (no effect).  The regressing lag sweeps 0.43/rev x 1P = 80-110 Hz over T, so the band to avoid is **f_arm 75-175 Hz** (x; y 1.15x); zeta >= 0.005 needs f_x >= 175 Hz (180 at J 5.49e-6 / I_t x2).
**Fixes (T1.4, J 2.5/3.76e-6, I_t x1/x2, worst over f_arm 60-300):**
- (a) arm spec f_x >= 180 Hz both axes (tap test with the mount locked).  Cantilever CF tube, E 70 GPa, tip 85 g + 0.24 m_tube: f = sqrt(3EI/L^3/m)/2pi.  L 120 mm: 14x12 175 Hz (marginal), 16x14 217 Hz; L 150 mm: 16x14 155 Hz (in band), 20x18 220 Hz; the 10 mm clamp tube is 73 Hz (L 150) / 102 Hz (L 120), i.e. inside the T0.7-T1.0 band.  Cost: tube mass/drag only.  Caveat: the model lumps the arm as 0.25 x vehicle mass at the hub, so the spec is on the measured frequency, not on the tube formula.
- (b) lag damping: flexure eta 0.03 -> 0.06 / 0.10 / 0.15 (zeta_struct 0.015 -> 0.03 / 0.05 / 0.075) leaves the band unstable: min -0.010 / -0.008 / -0.004 at 130 Hz; closing it needs zeta_struct > ~0.1 in LAG.  Not plausible: a viscoelastic layer between laminae shears in flap bending, barely in in-plane lag; a TPU strap damper would have to carry ~all the lag strain energy.  Rejected.
- (c) blade knob: flexure thickness x1.15 raises nu_lag and moves the band down (unstable 100-130, >= 0.017 from 150 Hz), width x1.3 makes it worse (band 110-160).  Cost: cr_pl 0.0650 -> 0.0632 (x1.08) / 0.0611 (x1.15) and both FAIL the hard resonance check (pitch antisym + hub 0.90-0.92/rev vs 1P; v6 0.88/rev, margin 0.18 -> 0.14 / 0.11).  Rejected: no v6c.
- (d) mount: 45 g on the mount (vs 55): band 110-130, >= 0.015 from 140 Hz; 70 g: unstable for every f_arm >= 150 Hz (-0.004 to -0.007) -> keep <= 55 g on the mount (this is what sinks the XY stage's moving frame and the ring).  Stiffer washers (k_lat x1.5, G 6.5 -> 9.75 eq.): band 110-130, >= 0.018 from 140 Hz, but raises the hub/1P ratio at T0.7 (not checked).
**Recommendation: (a), arm f_x and f_y >= 180 Hz, keep the v6b twin mount (<= 55 g on it).  Verified T {0.7,1,1.4} x J {2.5,3.76,5.49}e-6 x arm {190,225,260} x I_t x1/x2: min zeta 0.0137 / 0.0116 (190), 0.0229 / 0.0210 (225), 0.0273 / 0.0229 (260) >= 0.005.**  No design change.  If the airframe cannot reach 180 Hz, the fallback is a light mount (45 g) + f_arm >= 140 Hz, or a deliberately soft arm < 65 Hz (zeta 0.024, but poor for control).


## I32: 10in_v5 ground-resonance arm band, re-check of the four leads after the frame_zeta fix (study/mounts_tilt_diag10.py; raw results/mounts_tilt_diag10.json, verify_*.json, mounts_tilt_*.json re-run)
**10in_v5 + v5c twin mount (m 126 g, k_t 145, f_y 1.15 f_x, zeta_af 0.02, J 2.5/3.76/5.49e-6, I_t x1/x2; worst over J, I_t; 1P 108 / 128 / 152 Hz at T0.7 / 1.0 / 1.4):**

| T | unstable (zeta < 0) arm f_x | zeta < 0.005 | zeta at f_x >= 110 |
|---|---|---|---|
| 0.7 | 50 (-0.013) | <= 50 | 0.014 |
| 1.0 | 50-60 (-0.017 / -0.011) | <= 60 | 0.012 |
| 1.4 | 60-90 (min -0.024 at 70) | 50, 60-90, 100 is 0.006 | 0.009 from 110 |

The band is 50-95 Hz (10 Hz grid; 0.4-0.65 x 1P at T1.4, same mechanism as 7in: regressing lag meets hub/arm), far lower in absolute terms than the 7in 75-175 Hz because 10in 1P is 152 vs 260 Hz.  **Spec f_arm >= 120 Hz (min ~110, zeta then floored at the lag-mode 0.009 not by the arm); mount <= 126 g (modelled; not swept).**  Tube (E 70 GPa, tip 126 g + 0.24 m_tube, arm length GUESSED 180-220 mm, nominal 200): 200 mm needs 22x20 (136 Hz; 20x18 = 117 is marginal-borderline, 18x16 = 99 is in the band edge); 180 mm: 20x18 (138); 220 mm: 25x23 (143).  The 10 mm clamp tube is deep in the band at any of these lengths.

**Re-run of the four leads with the fixed frame_zeta (old = before fix, from the stored json):**

| lead / check | old min zeta | new min zeta | verdict |
|---|---|---|---|
| 7in v5 rigid-arm Floquet (air_zeta, 48 pts) | -0.0034 | -0.0034 (0/48 points changed) | none |
| 10in v4 rigid-arm (zaf .02 / .05) | -0.0033 / 0.0023 | -0.0033 / 0.0023 | none |
| 10in v5 rigid-arm (zaf .02 / .05) | -0.0043 / 0.0028 | -0.0043 / 0.0028 | none |
| 7in v6 rigid-arm (zaf .02 / .05), no stored old | n/a | -0.0044 / 0.0067 | n/a |
| 7in v6 twin mount, 190/225/260, I_t x1 / x2 / x3 | 0.0137 / 0.0116 / 0.0123 (190) | identical to 4 dp, all 9 cells | none |
| 10in v5 twin mount, all cells | 0.0089 | 0.0089 | none |
| 10in v5 non-tilt "mount" variant (verify_10in_v5 floq) | 0.0088 | 0.0089 | none |
| 10in v4 non-tilt "mount" variant | 0.0107 | **-1.0 at 190 Hz, T0.7** (other cells 0.0113-0.0187, slightly up) | CHANGED, see below |
| 7in v6 with that non-tilt 10in mount (not its mount) | n/a | -0.09 to -0.14 | n/a, not a valid config |

No feasibility verdict changes for the leads' actual configurations (rigid arm, twin mount): the earlier twin-mount floors were already computed with the fixed metric (mounts_tilt json unchanged), and the rigid-arm cases have no tilt mode to alias.  One new anomaly: with the non-tilt single-washer mount (v4 verification variant, hub mode only) the fixed code returns zeta = -1.0 at 190 Hz T0.7, because two REAL positive roots (s = +7 and +35 rad/s, Im s = 0, |s| just above the 0.05 Omega cut-off) now get wf ~ 0; the old alias heuristic gave 0.0173 for the same point.  This is probably a spurious static/reduction root of the non-tilt hub model (not part of any lead's design mount), but it is unresolved: frame_zeta should treat Im s ~ 0 real roots explicitly (use |Re s|/|s|) before that variant is quoted again.  Followup, not done here.
