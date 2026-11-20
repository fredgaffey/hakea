# Verification of 7in_tt_strap_steelx_v3 (flexblade/study/verify_v3.py)

Raw data: results/verify_7in_v3.json. cr = control ratio at thrust 0.7/1.0/1.4; feasibility = run_study.metrics (robust.combine) over the 3 throttles.

## 1. Tolerance map (one at a time, then 20 random combined +-5 %)

| case | min cr | feasible | violations | res margin (which) | cr 0.7/1.0/1.4 |
|---|---|---|---|---|---|
| nominal | 0.107 | yes | - | 0.167 (pitch antisym+hub 0.89/rev vs 1P) | 0.354 / 0.196 / 0.107 |
| t=0.13mm | 0.095 | NO | resonance | 0.124 (pitch sym 3.37/rev vs 3P) | 0.331 / 0.185 / 0.095 |
| t=0.20mm | 0.123 | NO | fatigue_strain, fatigue_stress, resonance | 0.081 (pitch antisym+hub 0.94/rev vs 1P) | 0.612 / 0.256 / 0.123 |
| w -10% | 0.108 | yes | - | 0.173 (pitch antisym+hub 0.89/rev vs 1P) | 0.350 / 0.196 / 0.108 |
| w +10% | 0.105 | yes | - | 0.162 (pitch antisym+hub 0.89/rev vs 1P) | 0.357 / 0.195 / 0.105 |
| L -10% | 0.115 | yes | - | 0.154 (pitch antisym+hub 0.90/rev vs 1P) | 0.376 / 0.207 / 0.115 |
| L +10% | 0.094 | NO | resonance | 0.106 (pitch sym 3.31/rev vs 3P) | 0.328 / 0.179 / 0.094 |
| hinge_r -10% | 0.085 | yes | - | 0.168 (pitch sym 3.50/rev vs 3P) | 0.256 / 0.149 / 0.085 |
| hinge_r +10% | 0.143 | NO | fatigue_strain, fatigue_stress, resonance | 0.100 (pitch antisym+hub 0.93/rev vs 1P) | 0.551 / 0.283 / 0.143 |
| skew -2deg | 0.093 | yes | - | 0.181 (pitch antisym+hub 0.88/rev vs 1P) | 0.309 / 0.172 / 0.093 |
| skew +2deg | 0.126 | NO | fatigue_stress, resonance | 0.094 (pitch sym 3.27/rev vs 3P) | 0.422 / 0.233 / 0.126 |
| blade mass -10% | 0.089 | yes | - | 0.215 (pitch sym 3.64/rev vs 3P) | 0.295 / 0.164 / 0.089 |
| blade mass +10% | 0.131 | NO | fatigue_strain, fatigue_stress, resonance | 0.110 (pitch sym 3.32/rev vs 3P) | 0.441 / 0.243 / 0.131 |
| elastic axis -0.02c | 0.115 | NO | fatigue_strain | 0.154 (pitch antisym+hub 0.90/rev vs 1P) | 0.375 / 0.210 / 0.115 |
| elastic axis +0.02c | 0.101 | yes | - | 0.174 (pitch antisym+hub 0.89/rev vs 1P) | 0.340 / 0.186 / 0.101 |
| strap k -20% | 0.105 | yes | - | 0.174 (pitch antisym+hub 0.89/rev vs 1P) | 0.344 / 0.192 / 0.105 |
| strap k +20% | 0.108 | yes | - | 0.160 (pitch antisym+hub 0.89/rev vs 1P) | 0.365 / 0.200 / 0.108 |
| random00 | 0.089 | yes | - | 0.185 (pitch sym 3.55/rev vs 3P) | 0.282 / 0.161 / 0.089 |
| random01 | 0.094 | yes | - | 0.201 (pitch sym 3.60/rev vs 3P) | 0.291 / 0.167 / 0.094 |
| random02 | 0.121 | NO | fatigue_strain, resonance | 0.140 (pitch sym 3.41/rev vs 3P) | 0.405 / 0.223 / 0.121 |
| random03 | 0.113 | NO | fatigue_stress | 0.159 (pitch antisym+hub 0.89/rev vs 1P) | 0.378 / 0.210 / 0.113 |
| random04 | 0.103 | yes | - | 0.184 (pitch antisym+hub 0.88/rev vs 1P) | 0.324 / 0.183 / 0.103 |
| random05 | 0.101 | yes | - | 0.171 (pitch antisym+hub 0.89/rev vs 1P) | 0.349 / 0.191 / 0.101 |
| random06 | 0.088 | yes | - | 0.205 (pitch sym 3.61/rev vs 3P) | 0.287 / 0.161 / 0.088 |
| random07 | 0.112 | NO | aeroelastic_stability, resonance | 0.102 (pitch sym 3.30/rev vs 3P) | 0.385 / 0.213 / 0.112 |
| random08 | 0.107 | NO | fatigue_stress, resonance, aeroelastic_stability | 0.146 (pitch antisym+hub 0.90/rev vs 1P) | 0.382 / 0.204 / 0.107 |
| random09 | 0.090 | yes | - | 0.199 (pitch antisym+hub 0.88/rev vs 1P) | 0.298 / 0.166 / 0.090 |
| random10 | 0.105 | yes | - | 0.169 (pitch antisym+hub 0.89/rev vs 1P) | 0.351 / 0.194 / 0.105 |
| random11 | 0.120 | NO | fatigue_strain, fatigue_stress, resonance | 0.133 (pitch antisym+hub 0.91/rev vs 1P) | 0.420 / 0.228 / 0.120 |
| random12 | 0.084 | yes | - | 0.211 (pitch sym 3.63/rev vs 3P) | 0.276 / 0.155 / 0.084 |
| random13 | 0.113 | yes | - | 0.167 (pitch antisym+hub 0.89/rev vs 1P) | 0.367 / 0.203 / 0.113 |
| random14 | 0.105 | NO | resonance | 0.100 (pitch sym 3.29/rev vs 3P) | 0.337 / 0.191 / 0.105 |
| random15 | 0.103 | yes | - | 0.177 (pitch antisym+hub 0.89/rev vs 1P) | 0.334 / 0.187 / 0.103 |
| random16 | 0.087 | yes | - | 0.229 (pitch antisym+hub 0.87/rev vs 1P) | 0.274 / 0.156 / 0.087 |
| random17 | 0.105 | yes | - | 0.183 (pitch antisym+hub 0.88/rev vs 1P) | 0.340 / 0.189 / 0.105 |
| random18 | 0.104 | yes | - | 0.193 (pitch antisym+hub 0.88/rev vs 1P) | 0.325 / 0.185 / 0.104 |
| random19 | 0.093 | yes | - | 0.172 (pitch sym 3.51/rev vs 3P) | 0.300 / 0.170 / 0.093 |

Random +-5 % combined: 6/20 infeasible; min cr range 0.084-0.121.

Knife-edges (single knob -> infeasible): strip thickness (both neighbouring shims), free length +10 %, hinge radius +10 %, skew +2 deg, blade mass +10 %, elastic axis -0.02c (fatigue strain). Failure modes are the resonance band (pitch mode vs 1P/3P, nominal margin 0.167) and fatigue (cr rises, e.g. 0.55-0.61 at T0.7, with the strain). Insensitive: strip width, strap stiffness +-20 %, elastic axis +0.02c. cr itself stays 0.084-0.143 (never collapses); the design is limited by resonance margin and fatigue, not authority.

## 2. Higher harmonics (modes within +-10 % of 4P/5P/6P; rotating-frame, per rev)

- T0.7 (11137 rpm, 1P = 186 Hz): modes/rev 5.00 pitch sym (z=0.016), 5.43 pitch antisym (z=0.015), 5.94 pitch sym (z=0.015); near 4P: none; 5P: pitch sym 5.003 (+0.1%), pitch antisym 5.425 (+8.5%); 6P: pitch antisym 5.425 (-9.6%), pitch sym 5.942 (-1.0%)
- T1 (13225 rpm, 1P = 220 Hz): modes/rev 4.26 pitch sym (z=0.018), 4.46 pitch antisym (z=0.018), 5.11 pitch sym (z=0.015); near 4P: pitch sym 4.261 (+6.5%); 5P: pitch sym 5.109 (+2.2%); 6P: none
- T1.4 (15693 rpm, 1P = 262 Hz): modes/rev 3.62 pitch sym (z=0.023), 3.73 pitch antisym (z=0.020), 4.41 pitch sym (z=0.014); near 4P: pitch sym 3.62 (-9.5%), pitch antisym 3.728 (-6.8%); 5P: none; 6P: none

Forced response per N m of hub torque at k*Omega (harmonic solve, same linear model and aero; pitch at 0.75R in deg per N m / in-plane hub moment per N m):

| k | T0.7 | T1 | T1.4 |
|---|---|---|---|
| 1P | 39.49 / 1.44 | 25.66 / 1.31 | 17.71 / 1.21 |
| 2P | 2.85 / 0.06 | 2.16 / 0.06 | 1.72 / 0.07 |
| 3P | 1.38 / 0.04 | 1.23 / 0.05 | 1.47 / 0.09 |
| 4P | 1.08 / 0.06 | 1.54 / 0.17 | 0.72 / 0.09 |
| 5P | 2.35 / 0.38 | 1.98 / 0.32 | 0.26 / 0.30 |
| 6P | 3.46 / 0.41 | 0.47 / 0.13 | 0.15 / 0.02 |

Interpretation: the model's forcing is the 1P duty pulse only (a sinusoid at 1P has no 5P content). In hover the rotating-frame aero is steady, so 5P forcing exists only through drive-waveform distortion (ESC slew limiting / non-sinusoidal duty) and effects not modelled here. The 2nd pitch-sym mode sits exactly on 5P at T0.7 (5.003/rev, zeta 0.016), 5.11/rev at T1 (+2 %); 4P neighbours are 4.26/4.46 at T1 (+6.5 %) and 3.62/3.73 at T1.4 (-9.5 %/-6.8 %). These modes couple weakly to hub torque: per-N m pitch response at 5P is 6 % (T0.7), 8 % (T1), 1.5 % (T1.4) of the 1P response; hub moment at 5P is 26 % of 1P at T0.7. So a 5P torque component of x % of the 1P pulse gives ~0.06x (pitch) / 0.26x (hub moment) of the 1P effect at T0.7: a risk only if the drive waveform carries >15-20 % 5P (a square pulse would: 1/5 = 20 %). Keep the pulse sinusoidal or filter it. Caveat: the aero inflow coupling in aero_linear is the 1P form, reused at k*Omega (small effect at 5P).

## 3. CalculiX of the v3 strips (0.15x8x24 mm, cross 130 deg; hex20, hub faces fixed, blade faces rigid)

| quantity | ccx | beam | ratio |
|---|---|---|---|
| k_hinge | 0.03842 | 0.04081 | 0.941 |
| k_cross | 88.08 | 86.17 | 1.022 |
| k_flap | 19.2 | 18.78 | 1.023 |
| kx | 2.023e+06 | 1.982e+06 | 1.021 |
| ky | 1.656e+07 | 1.723e+07 | 0.961 |
| kz | 3.259e+06 | 3.357e+06 | 0.971 |

Hinge axis ccx vs beam: 0.0009 deg. All ratios within 6 % (test accepts 0.8-1.2 / 0.7-1.4): the beam model stiffness for these strips is confirmed (unloaded, no strap).

## 4. Floquet stability with airframe (anisotropic: f_y = 1.15 f_x, zeta_af 0.02, default modal mass 0.25 m_veh)

Coarse sweep f = 50..400 Hz (8 points) per throttle, then refined around 0.92 f1 (the 'pitch antisym+hub' mode). Coarse-grid min air zeta: T0.7 0.0091, T1 -0.0011 (200 Hz), T1.4 -0.0005 (250 Hz). Refined (points below 0.0083 only):

| throttle | airframe f (Hz) | air-resonance zeta |
|---|---|---|
| T0.7 | 170.8 | 0.0006 |
| T0.7 | 181.0 | 0.0037 |
| T1 | 214.9 | -0.0026 |
| T1.4 | 192.5 | 0.0083 |
| T1.4 | 211.8 | 0.0083 |
| T1.4 | 226.2 | 0.0083 |
| T1.4 | 240.7 | 0.0083 |
| T1.4 | 255.1 | -0.0025 |
| T1.4 | 269.6 | 0.0083 |
| T1.4 | 288.8 | 0.0083 |

Outside a narrow tuning window all points have zeta ~0.008-0.009 (> min_damping 0.005). When the airframe in-plane frequency sits within roughly +-5 % of the hub-coupled pitch mode (T0.7: ~171-181 Hz, T1: ~203-227 Hz, T1.4: ~240-255 Hz) the coupled mode goes marginal/unstable: min zeta -0.0026 (T1, 214.9 Hz), -0.0025 (T1.4, 255 Hz), +0.0006 (T0.7, 171 Hz). It is a ground/air-resonance-type coupling of pitch (via the skewed hinge) with in-plane hub motion. Because the mode frequency moves with throttle (171-255 Hz), no fixed airframe frequency in 170-255 Hz is safe across the throttle range: tap-test the arm in-plane mode and keep it out of ~160-260 Hz, or add airframe damping.

## 5. Manufacturing review of study/results/steelx_7in_v3.png (from the drawing only; not a CAD check)

- Strips: 0.15 x 8 x 24 mm 1095 shim, two per blade crossed at 130 deg with a 1.5 mm gap, tilted ~35-40 deg in the side view. Clamping flat shim at a compound angle needs wedge clamps with a 1.5 mm step so the strips lie in separate planes; the drawing shows plain boxes with no clamp faces or fastener holes. A 0.15 mm shim needs a sandwich clamp (e.g. 2 x M1.6) or epoxy on roughened ends; the unclamped free length is the modelled 24 mm only if the clamp land is outside it. A spacer/bond at the crossing is not drawn and would change the cross stiffness.
- Strap: strand 1.9 mm (aramid) but the strip gap is 1.5 mm, so a strand cannot run through the crossing gap; the wrap pin is 7.6 mm against an 8 mm strip (0.2 mm per side) and its position is not visible. The dark diagonal bar through the crossing needs a CAD interference check against the strips and the 1.5 mm rod/plate.
- Envelope: in the end view the flexure envelope reaches z = -10 and y = +-11.4 mm, beyond the 16.8 mm hub block (z +-8.4); the strip boxes overlap the hub faces (x 7-8.4) and the blade-root block (x ~27-29), drawn as interpenetrating wedges; the blade root tapers to a sliver at x = 27.1 mm. The hub pockets need about +-12 mm width and 1.5-2 mm clearance to the strap pin.
- Skew 65 deg hinge axis: clamp faces are not perpendicular to the span, so the print needs supports or a split clamp. The tolerance map needs about +-0.5 mm on length/hinge radius, +-1 deg skew and the exact 0.15 mm shim (0.13 or 0.20 both fail), which is demanding for a hand-built part.
