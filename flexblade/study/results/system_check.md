# Integrated system check (everything on at once)

Scripts: `study/system_check_10in_v5.py`, `study/system_check_7in_v6.py` (raw: `results/system_check_<name>.json`).
Each case includes the flexure, the twin-washer tilt-stiff mount (v5c / v6b, tilt DOFs and gyroscopics on),
the arm f_x with f_y = 1.15 f_x, the ESC drive with `Drive(rpm_filter="rev_avg")` (10": bw 20 Hz; 7": bw 27 Hz;
tau 0.5 and 2 ms), J_hub {2.5, 3.76, 5.49}e-6, I_t x1 / x2 of the estimated motor I_t, T0.7/1.0/1.4, and the airframe
Floquet. A free-spin (no drive) run is included for each case to isolate the drive. Arms: 10" 120 (spec) / 150 / 200 Hz;
7" 180 (spec) / 200 / 225 / 260 Hz. Airframe zeta 0.02. Diverging real roots (non-oscillatory, Re s > 0.001 Omega) are counted as `ndiv`.

Pulse amplitudes (quasi-static; the 1P cyclic moment/force is constant in the fixed frame): at the cr_pl pulse
(pulse scale s* from the study's own checks) tilt = M/k_t, lateral = F/k_series (mount in series with the arm; rotor mass on
the arm neglected). Imbalance: 2 g mm at 1P on the 2-DOF hub/arm with the complex mount stiffness.
(`rotor.HarmonicData` gained `hub_force_H` for this.)

## 10" v5 (v5c mount: k_t 145 N m/rad, 84 mm spacing)
| T | 1P Hz | min zeta free spin (It x1 / x2) | min zeta rev_avg 0.5 ms | min zeta rev_avg 2 ms | divergences | cyclic G / phase vs free | cr_pl | hub tilt | hub lateral | 1P imbalance 2 g mm |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.7 | 108 | +0.0153 / +0.0140 | same | same | 0 | 1.000 / 0.0 deg | 0.180 | 0.11 deg | 37-40 um | 24-26 um |
| 1.0 | 128 | +0.0127 / +0.0117 | same | same | 0 | 1.000 / 0.0 deg | 0.121 | 0.10 deg | 37-39 um | 20-22 um |
| 1.4 | 152 | +0.0089 / +0.0089 | same | same | 0 | 1.000 / 0.0 deg | 0.082-0.089 | 0.10 deg | 36-40 um | 19-22 um |

Min zeta is the same for every arm (120/150/200 Hz), every governor setting and I_t x1/x2 (to 4 digits): it is the
same blade mode as in `mounts_tilt.md` (min 0.0089, T1.4). Governor-on max Re s is -12..-15 /s (the spin loop is damped).
**No interaction found for the 10".** Tilt 0.1 deg (k_t generous), lateral under 40 um.

## 7" v6 (v6b mount: k_t 177 N m/rad, 76 mm spacing)
| T | 1P Hz | min zeta free spin (It x1 / x2) | min zeta rev_avg 0.5 ms | min zeta rev_avg 2 ms | divergences | G / phase | cr_pl | hub tilt | hub lateral | 1P imbalance 2 g mm |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.7 | 183 | +0.0273 / +0.0295 | +0.0180 / +0.0200 | +0.0289 / +0.0311 | 0 | 1.000 / 0.0 deg | 0.164-0.185 | 0.03-0.04 deg | 12-15 um | 53-65 um |
| 1.0 | 217 | +0.0305 / +0.0334 | +0.0222 / +0.0262 | +0.0333 / +0.0373 | 0 | 1.000 / 0.0 deg | 0.102-0.146 | 0.03-0.04 deg | 11-16 um | 40-53 um |
| 1.4 | 260 | +0.0102 / +0.0080 (arm 180) | +0.0058 / +0.0044 | **+0.0041 / +0.0025** | 0 | 1.000 / 0.0 deg | 0.065-0.105 | 0.03-0.04 deg | 9-15 um | 45-53 um |

Per-arm min zeta at T1.4 with the governor (I_t x1 / x2): 180 Hz 0.0041 / 0.0025, 200 Hz 0.0086 / 0.0071, 225 Hz 0.0128 / 0.0115,
260 Hz 0.0169 / 0.0157. T0.7 and T1.0 are arm-independent (0.018+).

## Interactions that only appear when combined
1. **7" governor x arm at spec (T1.4, arm 180 Hz, J 5.49e-6)**: min zeta 0.0041 (I_t x1) / 0.0025 (I_t x2) with rev_avg at tau 2 ms, vs
   0.0102 / 0.0080 free spin at the same arm and 0.0137 / 0.0116 for the mount alone at 190 Hz (`mounts_tilt.md`). `drive_rev_avg.md`
   (tilt off, 225 Hz arm, 30 Hz) only saw the 0.5 ms drop (0.014-0.019 vs 0.025), so the high-throttle / spec-edge-arm / 2 ms
   combination is new: the drive costs about 0.006-0.008 of zeta there. Still positive, no divergence, but thin.
   **Recommendation: 7" arm spec >= 200 Hz when a governor is fitted** (0.007-0.009), or schedule the governor bandwidth down at high throttle.
2. 10": none. 1P cyclic G is exactly the free-spin value (rev_avg zero at 1P); min zeta is unchanged by the governor, the arm, I_t x2 or tau.
3. Mount x imbalance on the 7": the hub mode is 0.67-0.69 x 1P, which amplifies residual imbalance: 40-65 um per 2 g mm (10": 19-26).
   Balance the 7" to < 1 g mm.
4. Pulse-induced tilt/lateral motion is small on both (<= 0.11 deg, <= 40 um): no cyclic-moment / tilt-stiffness interaction.

Limits: pulse amplitudes are quasi-static (the fixed-frame 1P is DC); arm rotor mass neglected; imbalance is the translational part
only; the governor Pade model is pessimistic by about T/2 (see `drive_rev_avg.md`).
