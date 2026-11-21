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
