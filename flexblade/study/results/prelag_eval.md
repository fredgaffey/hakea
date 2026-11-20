# Pre-lag evaluation (off vs auto)

Split = trim_diff_pitch_deg (limit 2 deg). Lag = mean elastic hinge lag deg (auto column: elastic + pre-lag). k/K_cf from secant slope at T1.0, scaled by (Omega/Omega_ref)^2 (K_cf ~ Omega^2).

| design | thrust | split off | split auto | lag off | lag auto | CR off | CR auto | feas off/auto | binding (auto) | k/K_cf |
|---|---|---|---|---|---|---|---|---|---|---|
| 5in_tt_strap_hand | 0.7 | +1.39 | -0.66 | +0.86 | -0.41 (-2.00 pre) | 0.036 | 0.036 | N/N | root_envelope 4.12; viol resonance | 1.545 |
| 5in_tt_strap_hand | 1 | +1.74 | -0.00 | +1.08 | +0.00 (-2.00 pre) | 0.039 | 0.039 | Y/Y | root_envelope 4.12; viol - | 1.126 |
| 5in_tt_strap_hand | 1.4 | +2.10 | +0.71 | +1.32 | +0.45 (-2.00 pre) | 0.000 | 0.000 | N/N | motor_torque_1p 1000.00; viol check:motor_thermal,check:motor_torque_1p | 0.825 |
| 5in_tt_strap_steelx_hand | 0.7 | +2.78 | +0.33 | +1.85 | +0.31 (-15.00 pre) | 0.084 | 0.083 | N/N | root_envelope 3.25; viol check:fatigue_stress,check:static_stress,resonance | 0.065 |
| 5in_tt_strap_steelx_hand | 1 | +2.88 | +1.57 | +1.93 | +1.15 (-15.00 pre) | 0.047 | 0.047 | N/N | static_stress 3.31; viol check:static_stress,resonance | 0.047 |
| 5in_tt_strap_steelx_hand | 1.4 | +2.92 | +2.53 | +1.98 | +1.82 (-15.00 pre) | 0.002 | 0.001 | N/N | static_stress 4.02; viol check:creep_strain,check:static_stress,split | 0.035 |
| 7in_tt_strap_steelx_hand | 0.7 | +10.21 | +6.55 | +5.57 | +3.76 (+15.00 pre) | 0.311 | 0.416 | N/N | fatigue_stress 2.24; viol check:fatigue_strain,check:fatigue_stress,check:static_stress,resonance,split | nan |
| 7in_tt_strap_steelx_hand | 1 | +9.83 | +4.52 | +5.54 | +2.78 (+15.00 pre) | 0.186 | 0.237 | N/N | static_stress 2.94; viol check:creep_strain,check:fatigue_strain,check:fatigue_stress,check:interlayer,check:static_stress,resonance,split | nan |
| 7in_tt_strap_steelx_hand | 1.4 | +9.93 | -7.30 | +5.44 | -3.31 (+15.00 pre) | 0.047 | 0.000 | N/N | motor_torque_1p 1000.00; viol check:creep_strain,check:esc_duty_peak,check:interlayer,check:motor_thermal,check:motor_torque_1p,check:static_stress,check:strap_anchor,check:strap_tension,convergence,resonance,split | nan |
| 10in_tt_strap_steelx_hand | 0.7 | -10.58 | -10.58 | -5.12 | -5.12 (+0.00 pre) | 0.204 | 0.204 | N/N | fatigue_stress 4.18; viol check:creep_strain,check:fatigue_strain,check:fatigue_stress,check:interlayer,check:static_stress,geom:flexure_inside_hub,resonance,split | nan |
| 10in_tt_strap_steelx_hand | 1 | -9.91 | -9.91 | -5.11 | -5.11 (+0.00 pre) | 0.136 | 0.136 | N/N | static_stress 2.35; viol check:creep_strain,check:fatigue_strain,check:fatigue_stress,check:interlayer,check:static_stress,geom:flexure_inside_hub,resonance,split | nan |
| 10in_tt_strap_steelx_hand | 1.4 | -11.51 | -11.51 | -6.04 | -6.04 (+0.00 pre) | 0.000 | 0.000 | N/N | motor_torque_1p 1000.00; viol check:creep_strain,check:esc_duty_peak,check:interlayer,check:motor_torque_1p,check:static_stress,geom:flexure_inside_hub,resonance,split | nan |

Robust summary:
- 5in_tt_strap_hand: robust control_ratio off 0.000 / auto 0.000; robust feasible off False / auto False; k_hinge 0.4865703933288727; pre-lag -2.00 deg
- 5in_tt_strap_steelx_hand: robust control_ratio off 0.002 / auto 0.001; robust feasible off False / auto False; k_hinge 0.0193016203525681; pre-lag -15.00 deg
- 7in_tt_strap_steelx_hand: robust control_ratio off 0.047 / auto 0.000; robust feasible off False / auto False; k_hinge -0.7427996037285984; pre-lag +15.00 deg
- 10in_tt_strap_steelx_hand: robust control_ratio off 0.000 / auto 0.000; robust feasible off False / auto False; k_hinge 0.15191727911134895; pre-lag +0.00 deg
