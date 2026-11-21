# Rev-synchronous rpm averaging for the ESC/FC speed governor

Script `study/drive_rev_avg.py` -> `results/drive_rev_avg.json`. Model: `Drive.rpm_filter` (schema.py).
This is guidance for the ESC/FC firmware only (Fred writes the AM32/FC code).

**Filter.** The governor's speed measurement is a sliding average over exactly one mechanical revolution,
T = 2π/Ω: M(s) = (1 − e^{−sT})/(sT). It has zeros at every nP, so the 1P cyclic and its harmonics never
reach the PI. One possible implementation is the sum of the last pole-pair-count (7 here) eRPM periods,
updated on every eRPM event. Half a rev would also cancel 2P, but **not** the 1P cyclic, so it must be a full rev.
- `harmonic_1p`: exact M(jΩ) = 0, so Z(jΩ) is the back-EMF term (or the current-loop term) only.
- Stability (Floquet/LTI): Padé(4,4) of e^{−sT}, frequency-warped (×1.03) so the 1P zero stays exact, with
  4 extra states in companion form (`stability.drive_ss`). The low-frequency group delay is 1.03·T/2, so the
  model is slightly pessimistic. Padé(3) would need a 1.23 warp, which overstates the delay by 23 %. 2P and
  higher zeros are only approximate (|F| ≈ 0.04–0.1 above 1P).
- The lowpass is first order at 2 × bw.

## Result
* **rev_avg restores the free-spin 1P cyclic exactly**: |G|/|G_free| = 1.00 and Δφ = 0° in every case, even
  at τ = 2 ms and bw = 80 Hz. Raw eRPM gives 0.29–11.5× and up to ±180°. The lowpass is no fix: 0.41–3.4×
  and −146…+133°, and at 80 Hz it destabilises even with τ = 0.5 ms.
* **Stability**: rev_avg at 30 Hz is stable in all 6 cases at τ 0.5 **and 2 ms**. At 30 Hz/2 ms, raw eRPM is
  unstable in 5/6 cases and the lowpass in 3/6. At 80 Hz rev_avg is also unstable (except 7" T1.4 at 0.5 ms).
  That is not the 1P pumping: the speed loop itself runs out of phase, because the averager adds a T/2 delay
  (4.6 ms on the 10" at T0.7). The rigid-spin phase margin predicts the Floquet boundary well (PM ≈ 0 ↔ ζ ≈ 0).
* Open point: on the 7" at τ 0.5 ms, rev_avg 30 Hz lowers min ζ from 0.025 (free) to 0.014–0.019, while at
  2 ms it gives 0.026–0.029. This may come from the approximate 2P/3P zeros in the Padé model. Not yet traced.

## Guidance for Fred
| item | recommendation |
|---|---|
| governor structure | PI on rev-averaged rpm (window = exactly 1 mechanical rev, sliding, updated per eRPM event). Never feed raw or lowpassed eRPM to a fast loop. The alternative is no ESC governor, with the FC closing rpm slowly |
| delay budget | τ_eff = latency + L/R + **T/2 at the LOWEST flight rpm**. Phase margin ≈ 76° − 360°·f_c·τ_eff (PI zero at bw/4) |
| max bandwidth (PM ≥ 30°) | ≈ 0.13/τ_eff. 10" v5: **25 Hz** (τ 0.5 ms) / **20 Hz** (τ 2 ms), set by T0.7. 7" v6: 40 / 27 Hz |
| absolute limit (Floquet ζ > 0) | 10" v5: 40 Hz @0.5 ms, 30 Hz @2 ms. 7" v6: 40–50 Hz. Above that the speed loop itself oscillates |
| max latency | 2 ms is acceptable WITH rev_avg (bw ≤ 20 Hz). Without rev_avg even 30 Hz/2 ms is unstable, so raw eRPM needs τ ≤ 0.5 ms *and* still distorts G by 0.3–4× |
| rpm scheduling | T grows at low rpm, so either schedule bw ∝ rpm (bw ≈ 0.2·f_1P at τ 0.5 ms) or size the loop for the lowest rpm |
| calibration | with rev_avg, the per-throttle cyclic calibration does not depend on the governor gains. Bench check: cyclic response with the governor on and off should be identical |

## Tables
Table 1 cells: |G|/|G_free| / Δφ / min ζ (airframe Floquet, twin-washer mount, tilt off). Table 2: rev_avg
bandwidth sweep (Floquet min ζ, rigid-spin PM). Table 3: rigid-loop max bw (Hz) for PM ≥ 0/30/45°.

| speed loop | filter | 10" v5 T0.7 | 10" v5 T1 | 10" v5 T1.4 | 7" v6 T0.7 | 7" v6 T1 | 7" v6 T1.4 |
|---|---|---|---|---|---|---|---|
| free spin | – | ζ +0.013 | ζ +0.011 | ζ +0.009 | ζ +0.025 | ζ +0.027 | ζ +0.024 |
| 30 Hz, τ 0.5 ms | none | 0.74 / -73° / ζ +0.013 | 0.94 / -58° / ζ +0.011 | 1.02 / -43° / ζ +0.009 | 1.59 / -36° / ζ +0.040 | 1.60 / -20° / ζ +0.047 | 1.55 / -10° / ζ +0.056 |
| 30 Hz, τ 0.5 ms | lowpass | 3.22 / -9° / ζ +0.013 | 1.74 / +1° / ζ +0.011 | 1.36 / +1° / ζ +0.009 | 1.18 / +7° / ζ +0.018 | 1.09 / +6° / ζ +0.020 | 1.04 / +4° / ζ +0.019 |
| 30 Hz, τ 0.5 ms | rev_avg | 1.00 / +0° / ζ +0.013 | 1.00 / +0° / ζ +0.011 | 1.00 / +0° / ζ +0.009 | 1.00 / +0° / ζ +0.014 | 1.00 / +0° / ζ +0.018 | 1.00 / +0° / ζ +0.019 |
| 30 Hz, τ 2 ms | none | 2.23 / -158° / ζ -0.007 | 11.51 / +90° / ζ -0.018 | 2.83 / +31° / ζ -0.015 | 1.12 / +37° / ζ +0.003 | 0.91 / +25° / ζ -0.002 | 0.80 / +15° / ζ -0.005 |
| 30 Hz, τ 2 ms | lowpass | 1.22 / +43° / ζ -0.028 | 1.06 / +25° / ζ -0.010 | 1.00 / +15° / ζ +0.009 | 0.88 / +7° / ζ -0.002 | 0.89 / +2° / ζ +0.009 | 0.93 / -2° / ζ +0.014 |
| 30 Hz, τ 2 ms | rev_avg | 1.00 / +0° / ζ +0.013 | 1.00 / +0° / ζ +0.011 | 1.00 / +0° / ζ +0.009 | 1.00 / +0° / ζ +0.026 | 1.00 / +0° / ζ +0.029 | 1.00 / +0° / ζ +0.029 |
| 80 Hz, τ 0.5 ms | none | 0.29 / -107° / ζ +0.013 | 0.43 / -100° / ζ +0.011 | 0.58 / -87° / ζ +0.009 | 1.03 / -113° / ζ +0.038 | 1.91 / -104° / ζ +0.042 | 3.91 / -87° / ζ +0.056 |
| 80 Hz, τ 0.5 ms | lowpass | 0.43 / -146° / ζ -0.104 | 0.82 / -144° / ζ -0.104 | 1.77 / -126° / ζ -0.090 | 3.37 / +98° / ζ -0.046 | 1.96 / +45° / ζ -0.044 | 1.39 / +31° / ζ -0.037 |
| 80 Hz, τ 0.5 ms | rev_avg | 1.00 / +0° / ζ -0.088 | 1.00 / +0° / ζ -0.071 | 1.00 / +0° / ζ -0.045 | 1.00 / +0° / ζ -0.021 | 1.00 / +0° / ζ -0.010 | 1.00 / +0° / ζ +0.001 |
| 80 Hz, τ 2 ms | none | 0.35 / -180° / ζ -0.289 | 0.58 / +163° / ζ -0.279 | 0.95 / +142° / ζ -0.253 | 0.65 / +78° / ζ -0.170 | 0.61 / +49° / ζ -0.152 | 0.56 / +27° / ζ -0.128 |
| 80 Hz, τ 2 ms | lowpass | 0.41 / +133° / ζ -0.270 | 0.61 / +103° / ζ -0.240 | 0.77 / +74° / ζ -0.189 | 0.57 / +34° / ζ -0.156 | 0.60 / +14° / ζ -0.112 | 0.65 / -2° / ζ -0.066 |
| 80 Hz, τ 2 ms | rev_avg | 1.00 / +0° / ζ -0.204 | 1.00 / +0° / ζ -0.206 | 1.00 / +0° / ζ -0.198 | 1.00 / +0° / ζ -0.179 | 1.00 / +0° / ζ -0.162 | 1.00 / +0° / ζ -0.136 |

| rev_avg bw | τ | 10in_v5:T0.7 | 10in_v5:T1 | 10in_v5:T1.4 | 7in_v6:T0.7 | 7in_v6:T1 | 7in_v6:T1.4 |
|---|---|---|---|---|---|---|---|
| 20 Hz | 0.5 ms | ζ +0.013 (PM +43°) | ζ +0.011 (PM +48°) | ζ +0.009 (PM +52°) | ζ +0.022 (PM +60°) | ζ +0.024 (PM +63°) | ζ +0.023 (PM +66°) |
| 20 Hz | 2 ms | ζ +0.013 (PM +32°) | ζ +0.011 (PM +37°) | ζ +0.009 (PM +41°) | ζ +0.030 (PM +49°) | ζ +0.032 (PM +52°) | ζ +0.030 (PM +55°) |
| 30 Hz | 0.5 ms | ζ +0.013 (PM +24°) | ζ +0.011 (PM +30°) | ζ +0.009 (PM +35°) | ζ +0.014 (PM +44°) | ζ +0.018 (PM +48°) | ζ +0.019 (PM +52°) |
| 30 Hz | 2 ms | ζ +0.013 (PM +9°) | ζ +0.011 (PM +15°) | ζ +0.009 (PM +20°) | ζ +0.026 (PM +28°) | ζ +0.029 (PM +32°) | ζ +0.029 (PM +36°) |
| 40 Hz | 0.5 ms | ζ +0.006 (PM +8°) | ζ +0.011 (PM +15°) | ζ +0.009 (PM +21°) | ζ +0.007 (PM +31°) | ζ +0.012 (PM +36°) | ζ +0.015 (PM +41°) |
| 40 Hz | 2 ms | ζ -0.008 (PM -11°) | ζ +0.011 (PM -5°) | ζ +0.009 (PM +1°) | ζ +0.021 (PM +10°) | ζ +0.026 (PM +15°) | ζ +0.028 (PM +20°) |
| 50 Hz | 0.5 ms | ζ -0.005 (PM -6°) | ζ +0.006 (PM +2°) | ζ +0.009 (PM +9°) | ζ -0.000 (PM +19°) | ζ +0.006 (PM +25°) | ζ +0.012 (PM +31°) |
| 50 Hz | 2 ms | ζ -0.085 (PM -28°) | ζ -0.063 (PM -21°) | ζ -0.035 (PM -15°) | ζ +0.007 (PM -6°) | ζ +0.024 (PM -0°) | ζ +0.027 (PM +5°) |
| 60 Hz | 0.5 ms | ζ -0.025 (PM -18°) | ζ -0.003 (PM -10°) | ζ +0.009 (PM -2°) | ζ -0.007 (PM +9°) | ζ +0.000 (PM +15°) | ζ +0.008 (PM +22°) |
| 60 Hz | 2 ms | ζ -0.139 (PM -42°) | ζ -0.125 (PM -36°) | ζ -0.105 (PM -30°) | ζ -0.071 (PM -20°) | ζ -0.042 (PM -14°) | ζ -0.005 (PM -9°) |
| 80 Hz | 0.5 ms | ζ -0.088 (PM -37°) | ζ -0.071 (PM -29°) | ζ -0.045 (PM -21°) | ζ -0.021 (PM -10°) | ζ -0.010 (PM -2°) | ζ +0.001 (PM +5°) |
| 80 Hz | 2 ms | ζ -0.204 (PM -66°) | ζ -0.206 (PM -60°) | ζ -0.198 (PM -54°) | ζ -0.179 (PM -45°) | ζ -0.162 (PM -40°) | ζ -0.136 (PM -34°) |
| 120 Hz | 0.5 ms | ζ -0.129 (PM -64°) | ζ -0.138 (PM -56°) | ζ -0.132 (PM -49°) | ζ -0.113 (PM -37°) | ζ -0.095 (PM -30°) | ζ -0.063 (PM -22°) |
| 120 Hz | 2 ms | ζ -0.253 (PM -99°) | ζ -0.285 (PM -95°) | ζ -0.298 (PM -91°) | ζ -0.303 (PM -83°) | ζ -0.300 (PM -79°) | ζ -0.290 (PM -74°) |

| τ | 10in_v5:T0.7 (1P 108 Hz) | 10in_v5:T1 (1P 128 Hz) | 10in_v5:T1.4 (1P 152 Hz) | 7in_v6:T0.7 (1P 183 Hz) | 7in_v6:T1 (1P 217 Hz) | 7in_v6:T1.4 (1P 260 Hz) |
|---|---|---|---|---|---|---|
| 0.5ms | 45/25/17.5 | 50/27.5/20 | 57.5/32.5/22.5 | 67.5/40/27.5 | 75/45/32.5 | 85/50/35 |
| 2ms | 32.5/20/15 | 35/22.5/15 | 40/22.5/17.5 | 45/27.5/20 | 47.5/30/22.5 | 52.5/32.5/25 |
