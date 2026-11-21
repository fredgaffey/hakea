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

