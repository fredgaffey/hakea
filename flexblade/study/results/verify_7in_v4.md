# Verification of 7in_tt_strap_steelx_v4 (flexblade/study/verify_v4.py)

Raw data: results/verify_7in_v4.json. cr = control ratio at thrust 0.7/1.0/1.4; feasibility = run_study.metrics (robust.combine) over the 3 throttles. res = damping-aware 1P/2P/3P margin; fat = max(fatigue_strain, fatigue_stress) utilisation; hp = distance of pitch modes (>2/rev) from 4P/5P/6P.

## Recipe

```
{
 "flexure.thickness": 0.00015,
 "flexure.width": 0.009,
 "flexure.length": 0.01873,
 "flexure.hinge_radius": 0.01516,
 "flexure.skew_deg": 59.7,
 "flexure.cross_angle_deg": 151.26842,
 "blade.elastic_axis_chord_frac": 0.35014,
 "flexure.strap_strand_diameter": 0.0015
}
```
Strip material in the model: `steel_1095_shim@E=1.1053,S=1.1053,rho=1.4393,eta=0.0298`; crossing gap 1.9 mm for a 1.5 mm strand; pin 6.0 mm. Member envelope z +-7.03 mm (hub +-8.4 mm), min member x 5.40 mm.

## 1. Constrained-layer damper (RKU)

| visco layer | constraining | k | eta_c | EI x | g | e3/e1 | mass x |
|---|---|---|---|---|---|---|---|
| 3M ISD-112 0.05mm | 1095 shim 0.05mm | 1 pi/L | 0.122 | 1.20 | 0.058 | 0.44 | 1.37 |
| 3M ISD-112 0.05mm | 1095 shim 0.05mm | 2 pi/L | 0.037 | 1.08 | 0.014 | 0.36 | 1.37 |
| 3M ISD-112 0.05mm | 1095 shim 0.025mm | 1 pi/L | 0.100 | 1.14 | 0.116 | 0.36 | 1.21 |
| 3M ISD-112 0.05mm | 1095 shim 0.025mm | 2 pi/L | 0.032 | 1.04 | 0.029 | 0.22 | 1.21 |
| 3M ISD-112 0.05mm | PET film 0.125mm | 1 pi/L | 0.060 | 1.19 | 1.027 | 2.19 | 1.19 |
| 3M ISD-112 0.05mm | PET film 0.125mm | 2 pi/L | 0.041 | 1.07 | 0.257 | 1.37 | 1.19 |
| 3M ISD-112 0.05mm | PET film 0.25mm | 1 pi/L | 0.149 | 1.49 | 0.514 | 2.87 | 1.34 |
| 3M ISD-112 0.05mm | PET film 0.25mm | 2 pi/L | 0.077 | 1.22 | 0.128 | 2.06 | 1.34 |
| 3M VHB 9469 0.13mm | 1095 shim 0.05mm | 1 pi/L | 0.128 | 1.29 | 0.033 | 0.43 | 1.44 |
| 3M VHB 9469 0.13mm | 1095 shim 0.05mm | 2 pi/L | 0.040 | 1.10 | 0.008 | 0.36 | 1.44 |
| 3M VHB 9469 0.13mm | 1095 shim 0.025mm | 1 pi/L | 0.115 | 1.22 | 0.066 | 0.35 | 1.27 |
| 3M VHB 9469 0.13mm | 1095 shim 0.025mm | 2 pi/L | 0.036 | 1.06 | 0.016 | 0.21 | 1.27 |
| 3M VHB 9469 0.13mm | PET film 0.125mm | 1 pi/L | 0.081 | 1.27 | 0.585 | 2.20 | 1.25 |
| 3M VHB 9469 0.13mm | PET film 0.125mm | 2 pi/L | 0.043 | 1.09 | 0.146 | 1.30 | 1.25 |
| 3M VHB 9469 0.13mm | PET film 0.25mm | 1 pi/L | 0.148 | 1.55 | 0.292 | 2.69 | 1.40 |
| 3M VHB 9469 0.13mm | PET film 0.25mm | 2 pi/L | 0.066 | 1.23 | 0.073 | 1.97 | 1.40 |
| 3M VHB 4910 0.5mm | 1095 shim 0.05mm | 1 pi/L | 0.208 | 1.48 | 0.009 | 0.40 | 1.74 |
| 3M VHB 4910 0.5mm | 1095 shim 0.05mm | 2 pi/L | 0.068 | 1.15 | 0.002 | 0.35 | 1.74 |
| 3M VHB 4910 0.5mm | 1095 shim 0.025mm | 1 pi/L | 0.204 | 1.43 | 0.017 | 0.30 | 1.57 |
| 3M VHB 4910 0.5mm | 1095 shim 0.025mm | 2 pi/L | 0.067 | 1.11 | 0.004 | 0.20 | 1.57 |
| 3M VHB 4910 0.5mm | PET film 0.125mm | 1 pi/L | 0.187 | 1.48 | 0.152 | 1.98 | 1.56 |
| 3M VHB 4910 0.5mm | PET film 0.125mm | 2 pi/L | 0.073 | 1.14 | 0.038 | 1.15 | 1.56 |
| 3M VHB 4910 0.5mm | PET film 0.25mm | 1 pi/L | 0.222 | 1.69 | 0.076 | 2.33 | 1.70 |
| 3M VHB 4910 0.5mm | PET film 0.25mm | 2 pi/L | 0.082 | 1.26 | 0.019 | 1.84 | 1.70 |
| TPU 95A 0.2mm print | 1095 shim 0.05mm | 1 pi/L | 0.066 | 4.16 | 0.265 | 1.12 | 1.54 |
| TPU 95A 0.2mm print | 1095 shim 0.05mm | 2 pi/L | 0.053 | 2.00 | 0.066 | 0.58 | 1.54 |
| TPU 95A 0.2mm print | 1095 shim 0.025mm | 1 pi/L | 0.052 | 3.41 | 0.529 | 1.42 | 1.37 |
| TPU 95A 0.2mm print | 1095 shim 0.025mm | 2 pi/L | 0.047 | 1.84 | 0.132 | 0.61 | 1.37 |
| TPU 95A 0.2mm print | PET film 0.125mm | 1 pi/L | 0.010 | 1.94 | 4.702 | 4.49 | 1.35 |
| TPU 95A 0.2mm print | PET film 0.125mm | 2 pi/L | 0.021 | 1.62 | 1.176 | 3.24 | 1.35 |
| TPU 95A 0.2mm print | PET film 0.25mm | 1 pi/L | 0.023 | 3.30 | 2.351 | 5.32 | 1.50 |
| TPU 95A 0.2mm print | PET film 0.25mm | 2 pi/L | 0.038 | 2.27 | 0.588 | 3.62 | 1.50 |

Eig damping, v3 geometry, bare vs CLD strips (label, per rev, zeta):

- v3 bare T0.7: pitch sym 0.61 z=0.0312, pitch antisym+hub 0.89 z=0.0237, flap sym 1.60 z=0.0351, flap antisym 1.66 z=0.0360, pitch sym 4.96 z=0.0159
- v3 bare T1: pitch sym 0.60 z=0.0362, pitch antisym+hub 0.87 z=0.0266, flap sym 1.45 z=0.0382, flap antisym 1.50 z=0.0413, pitch sym 4.20 z=0.0183
- v3 bare T1.4: pitch sym 0.59 z=0.0413, pitch antisym+hub 0.86 z=0.0321, flap antisym 1.31 z=0.0412, flap antisym 1.39 z=0.0464, pitch sym 3.68 z=0.0210
- v3 + CLD T0.7: pitch sym 0.61 z=0.0363, pitch antisym+hub 0.88 z=0.0270, flap sym 1.63 z=0.0418, flap antisym 1.67 z=0.0431, pitch sym 4.96 z=0.0241
- v3 + CLD T1: pitch sym 0.59 z=0.0417, pitch antisym+hub 0.86 z=0.0299, flap antisym 1.46 z=0.0444, flap sym 1.51 z=0.0474, pitch sym 4.20 z=0.0259
- v3 + CLD T1.4: pitch sym 0.58 z=0.0469, pitch antisym+hub 0.84 z=0.0360, flap antisym 1.32 z=0.0465, flap sym 1.39 z=0.0510, pitch sym 3.52 z=0.0302

## 2. Floquet + anisotropic airframe (f_y = 1.15 f_x), min air-resonance zeta over f_af 50-400 Hz

| design | zeta_af | T0.7 | T1 | T1.4 |
|---|---|---|---|---|
| v3_bare | 0.02 | -0.0045 @ 164 Hz | -0.0040 @ 195 Hz | -0.0036 @ 231 Hz |
| v3_bare | 0.05 | 0.0093 @ 50 Hz | 0.0088 @ 400 Hz | 0.0036 @ 241 Hz |
| v4 | 0.02 | -0.0062 @ 160 Hz | -0.0059 @ 191 Hz | -0.0055 @ 226 Hz |
| v4 | 0.05 | 0.0040 @ 400 Hz | 0.0060 @ 80 Hz | 0.0071 @ 236 Hz |

## 3. Search acceptance (nominal + 8 random +-5 %, seed 11)

{"p20": 0.05878641697714767, "all_feasible": true, "all_targets": false, "n_feasible": 9, "n_targets": 0, "n": 9, "p20_per_throttle": {"T0.7": 0.17675325065234984, "T1": 0.10287759244890628, "T1.4": 0.05878641697714767}, "min_res": 0.23546494591330736, "max_fat": 0.7218358259716052, "min_hp": 0.0008946300624916859}

| case | min cr | feasible | violations | res margin (which) | fat | hp margin (which) | cr 0.7/1.0/1.4 |
|---|---|---|---|---|---|---|---|
| nominal | 0.062 | yes | - | 0.262 (flap sym 1.25/rev vs 1P) | 0.68 | 0.023 (pitch sym 3.91/rev vs 4P) | 0.186 / 0.109 / 0.062 |
| random00 | 0.059 | yes | - | 0.255 (flap antisym 1.25/rev vs 1P) | 0.65 | 0.024 (pitch sym 6.14/rev vs 6P) | 0.175 / 0.103 / 0.059 |
| random01 | 0.059 | yes | - | 0.264 (flap antisym 1.26/rev vs 1P) | 0.64 | 0.002 (pitch sym 3.99/rev vs 4P) | 0.178 / 0.103 / 0.059 |
| random02 | 0.062 | yes | - | 0.280 (flap antisym 1.27/rev vs 1P) | 0.70 | 0.010 (pitch sym 4.04/rev vs 4P) | 0.190 / 0.109 / 0.062 |
| random03 | 0.061 | yes | - | 0.257 (flap sym 1.25/rev vs 1P) | 0.66 | 0.022 (pitch sym 6.13/rev vs 6P) | 0.183 / 0.107 / 0.061 |
| random04 | 0.065 | yes | - | 0.255 (flap sym 1.25/rev vs 1P) | 0.70 | 0.003 (pitch sym 5.98/rev vs 6P) | 0.197 / 0.114 / 0.065 |
| random05 | 0.060 | yes | - | 0.235 (flap sym 1.23/rev vs 1P) | 0.65 | 0.001 (pitch sym 5.00/rev vs 5P) | 0.180 / 0.106 / 0.060 |
| random06 | 0.066 | yes | - | 0.270 (pitch antisym+hub 0.85/rev vs 1P) | 0.72 | 0.012 (pitch sym 6.07/rev vs 6P) | 0.206 / 0.117 / 0.066 |
| random07 | 0.058 | yes | - | 0.243 (flap sym 1.23/rev vs 1P) | 0.60 | 0.002 (pitch sym 5.01/rev vs 5P) | 0.172 / 0.102 / 0.058 |

## 4. Tolerance map (one at a time, then 20 random combined +-5 %, seed 7)

| case | min cr | feasible | violations | res margin (which) | fat | hp margin (which) | cr 0.7/1.0/1.4 |
|---|---|---|---|---|---|---|---|
| nominal | 0.062 | yes | - | 0.262 (flap sym 1.25/rev vs 1P) | 0.68 | 0.023 (pitch sym 3.91/rev vs 4P) | 0.186 / 0.109 / 0.062 |
| t=0.13mm | 0.061 | yes | - | 0.206 (flap antisym 1.20/rev vs 1P) | 0.72 | 0.016 (pitch sym 4.92/rev vs 5P) | 0.178 / 0.106 / 0.061 |
| w -10% | 0.062 | yes | - | 0.246 (flap sym 1.24/rev vs 1P) | 0.67 | 0.004 (pitch sym 6.02/rev vs 6P) | 0.184 / 0.108 / 0.062 |
| w +10% | 0.062 | yes | - | 0.276 (flap sym 1.27/rev vs 1P) | 0.68 | 0.004 (pitch sym 3.98/rev vs 4P) | 0.188 / 0.109 / 0.062 |
| L -10% | 0.063 | yes | - | 0.279 (flap sym 1.27/rev vs 1P) | 0.69 | 0.001 (pitch sym 4.00/rev vs 4P) | 0.194 / 0.111 / 0.063 |
| L +10% | 0.059 | yes | - | 0.246 (flap antisym 1.24/rev vs 1P) | 0.68 | 0.008 (pitch sym 5.04/rev vs 5P) | 0.179 / 0.105 / 0.059 |
| hinge_r -10% | 0.051 | NO | resonance, geom:flexure_inside_hub | 0.049 (pitch sym+hub 3.14/rev vs 3P) | 0.54 | 0.065 (pitch sym+hub 4.26/rev vs 4P) | 0.150 / 0.089 / 0.051 |
| hinge_r +10% | 0.079 | yes | - | 0.198 (pitch antisym+hub 0.88/rev vs 1P) | 0.90 | 0.011 (pitch sym 4.04/rev vs 4P) | 0.247 / 0.140 / 0.079 |
| skew -2deg | 0.056 | yes | - | 0.269 (flap antisym 1.26/rev vs 1P) | 0.62 | 0.010 (pitch sym 3.96/rev vs 4P) | 0.166 / 0.097 / 0.056 |
| skew +2deg | 0.069 | yes | - | 0.254 (flap sym 1.25/rev vs 1P) | 0.74 | 0.001 (pitch sym 6.00/rev vs 6P) | 0.212 / 0.122 / 0.069 |
| blade mass -10% | 0.055 | yes | - | 0.274 (flap sym 1.26/rev vs 1P) | 0.56 | 0.001 (pitch sym 3.99/rev vs 4P) | 0.164 / 0.096 / 0.055 |
| blade mass +10% | 0.070 | yes | - | 0.238 (pitch antisym+hub 0.86/rev vs 1P) | 0.82 | 0.005 (pitch sym 4.98/rev vs 5P) | 0.215 / 0.124 / 0.070 |
| elastic axis -0.02c | 0.063 | yes | - | 0.239 (flap antisym 1.23/rev vs 1P) | 0.74 | 0.006 (pitch sym 6.03/rev vs 6P) | 0.188 / 0.110 / 0.063 |
| elastic axis +0.02c | 0.060 | yes | - | 0.291 (flap sym 1.28/rev vs 1P) | 0.59 | 0.023 (pitch sym 5.11/rev vs 5P) | 0.182 / 0.105 / 0.060 |
| strap k -20% | 0.062 | yes | - | 0.260 (flap sym 1.25/rev vs 1P) | 0.69 | 0.023 (pitch sym 6.14/rev vs 6P) | 0.184 / 0.108 / 0.062 |
| strap k +20% | 0.062 | yes | - | 0.263 (flap sym 1.26/rev vs 1P) | 0.66 | 0.022 (pitch sym 3.91/rev vs 4P) | 0.189 / 0.109 / 0.062 |
| random00 | 0.054 | yes | - | 0.240 (flap sym 1.23/rev vs 1P) | 0.59 | 0.004 (pitch sym 5.02/rev vs 5P) | 0.159 / 0.094 / 0.054 |
| random01 | 0.055 | yes | - | 0.236 (flap sym 1.23/rev vs 1P) | 0.61 | 0.011 (pitch sym 6.07/rev vs 6P) | 0.159 / 0.095 / 0.055 |
| random02 | 0.067 | yes | - | 0.255 (flap sym 1.25/rev vs 1P) | 0.77 | 0.002 (pitch sym 5.99/rev vs 6P) | 0.205 / 0.118 / 0.067 |
| random03 | 0.065 | yes | - | 0.252 (flap sym 1.24/rev vs 1P) | 0.72 | 0.020 (pitch sym 3.92/rev vs 4P) | 0.193 / 0.113 / 0.065 |
| random04 | 0.059 | yes | - | 0.258 (flap sym 1.25/rev vs 1P) | 0.63 | 0.014 (pitch sym 6.08/rev vs 6P) | 0.176 / 0.102 / 0.059 |
| random05 | 0.061 | yes | - | 0.271 (flap sym 1.26/rev vs 1P) | 0.65 | 0.023 (pitch sym 3.91/rev vs 4P) | 0.185 / 0.107 / 0.061 |
| random06 | 0.054 | yes | - | 0.255 (flap sym 1.25/rev vs 1P) | 0.59 | 0.016 (pitch sym 5.08/rev vs 5P) | 0.161 / 0.094 / 0.054 |
| random07 | 0.066 | yes | - | 0.239 (flap sym 1.23/rev vs 1P) | 0.74 | 0.003 (pitch sym 5.01/rev vs 5P) | 0.198 / 0.116 / 0.066 |
| random08 | 0.063 | yes | - | 0.280 (flap sym 1.27/rev vs 1P) | 0.71 | 0.007 (pitch sym 3.97/rev vs 4P) | 0.194 / 0.112 / 0.063 |
| random09 | 0.055 | yes | - | 0.269 (flap sym 1.26/rev vs 1P) | 0.61 | 0.014 (pitch sym 3.94/rev vs 4P) | 0.165 / 0.096 / 0.055 |
| random10 | 0.062 | yes | - | 0.272 (flap sym 1.26/rev vs 1P) | 0.64 | 0.014 (pitch sym 3.94/rev vs 4P) | 0.184 / 0.108 / 0.062 |
| random11 | 0.069 | yes | - | 0.248 (flap antisym 1.24/rev vs 1P) | 0.78 | 0.007 (pitch sym 6.04/rev vs 6P) | 0.207 / 0.121 / 0.069 |
| random12 | 0.053 | yes | - | 0.250 (flap sym 1.24/rev vs 1P) | 0.58 | 0.024 (pitch sym 5.12/rev vs 5P) | 0.157 / 0.093 / 0.053 |
| random13 | 0.063 | yes | - | 0.271 (flap sym 1.26/rev vs 1P) | 0.66 | 0.023 (pitch sym 3.91/rev vs 4P) | 0.194 / 0.112 / 0.063 |
| random14 | 0.061 | yes | - | 0.249 (flap sym 1.24/rev vs 1P) | 0.65 | 0.006 (pitch sym 5.03/rev vs 5P) | 0.183 / 0.107 / 0.061 |
| random15 | 0.059 | yes | - | 0.263 (flap antisym 1.25/rev vs 1P) | 0.66 | 0.001 (pitch sym 4.01/rev vs 4P) | 0.176 / 0.103 / 0.059 |
| random16 | 0.053 | yes | - | 0.268 (flap sym 1.26/rev vs 1P) | 0.54 | 0.025 (pitch sym 3.90/rev vs 4P) | 0.156 / 0.092 / 0.053 |
| random17 | 0.061 | yes | - | 0.273 (flap sym 1.27/rev vs 1P) | 0.62 | 0.021 (pitch sym 5.11/rev vs 5P) | 0.185 / 0.107 / 0.061 |
| random18 | 0.059 | yes | - | 0.248 (flap sym 1.24/rev vs 1P) | 0.63 | 0.012 (pitch sym 6.07/rev vs 6P) | 0.175 / 0.102 / 0.059 |
| random19 | 0.057 | yes | - | 0.250 (flap sym 1.24/rev vs 1P) | 0.59 | 0.003 (pitch sym 5.01/rev vs 5P) | 0.168 / 0.099 / 0.057 |
| t=0.20mm | 0.068 | yes | - | 0.160 (pitch antisym+hub 0.89/rev vs 1P) | 0.86 | 0.038 (pitch sym 4.15/rev vs 4P) | 0.271 / 0.131 / 0.068 |

Random set summary: {"p20": 0.05476252877194295, "all_feasible": true, "all_targets": false, "n_feasible": 21, "n_targets": 0, "n": 21, "p20_per_throttle": {"T0.7": 0.16092965864938508, "T1": 0.09477691511630333, "T1.4": 0.05476252877194295}, "min_res": 0.23645365262843399, "max_fat": 0.7802114846333909, "min_hp": 0.0012757660979376695}
