# Authority chain of 7in_tt_strap_steelx_v5 (heat -> cyclic moment)

Model facts (rotor.py, motor.py): the hub is ONE torsional DOF, J = J_hub + J_rotor = 5.49e-6 + 6.99e-6 = 12.5e-6 kg m^2 (motor bell + hub). The motor torque pulse acts on that DOF. The blades are NOT part of J in the hub equation: they hang off the hub through the (very soft) lag flexure and respond as pendulums to the hub's angular acceleration (relative lag ~1.3-1.6 x hub swing). So what swings is hub+bell; blades see it as an inertial excitation. Lag -> pitch is the skew coupling (pitch_lag_coupling, ~tan(skew)=1.73 plus centrifugal/aero terms). Pitch -> cyclic moment is aero.

Chain (blade-1 amplitudes, API harmonic, robust throttle points, pulse = 0.95 x capability):

| link | T0.7 | T1.0 | T1.4 |
|---|---|---|---|
| rpm / Omega (rad/s) | 11005 / 1152 | 13132 / 1375 | 15684 / 1642 |
| mean torque Q0 (N m) / copper W (thermal util) | 0.087 / 24.8 (0.48) | 0.121 / 30.8 (0.63) | 0.165 / 44.2 (0.90) |
| L0 1P torque amp tau1 (N m) (limit: current, 1P util 0.86/0.84/0.81) | 0.298 | 0.262 | 0.218 |
| L1 hub swing phi_h per tau1 (rad/N m) [= 1/(J_eff Om^2)] | 0.110 | 0.0745 | 0.044 |
| (J_eff) | 6.8e-6 | 7.1e-6 | 8.4e-6 |
| L2 lag/phi_h (blade lag relative to hub) | 1.62 | 1.52 | 1.31 |
| L3 pitch/lag (skew coupling) | 1.83 | 1.99 | 2.92 |
| L4 hub moment per deg pitch (N m/deg) | 0.0175 | 0.0236 | 0.0283 |
| L5 1/(W R) normalisation (1/N m) | x1 | x0.70 | x0.50 |
| result control_ratio | 0.161 | 0.092 | 0.049 |

T0.7 -> T1.4 factors: tau1 x0.73, L1 x0.40, L2 x0.81, L3 x1.6, L4 x1.6, L5 x0.5 -> product 0.30 = cr ratio 0.305 (checks).

Weakest links at T1.4: (1) L1, hub swing per torque falls as 1/Omega^2 (rpm up 1.43x for 1.4x thrust: -60 %); (2) L0, the pulse itself is current-limited and shrinks as I0 eats the 50 A budget (-27 %), while the thermal budget (0.90) is the other binder; (3) L5 thrust normalisation. The skew coupling and aero stages GAIN at high thrust. So heat is converted to authority by Kt^2/R (motor constant, set by stator volume) and then diluted by 1/(J Omega^2): the lever is J, not kv or skew.

## Sensitivities (robust eval, min-cr over 0.7/1.0/1.4; v5 min cr 0.0490, thermal 0.896, 1P torque 0.86)
| change | min cr | d cr | thermal | notes |
|---|---|---|---|---|
| J_hub -30 % (-1.6e-6) | 0.0606 | +24 % | 0.896 | res margin 0.274 -> 0.216 |
| J_hub +30 % | 0.0411 | -16 % | 0.896 | |
| J_rotor -30 % (-2.1e-6) | 0.0648 | +32 % | 0.896 | res 0.197 |
| kv 900 (R x(1100/900)^2, Imax scaled: same Km) | 0.0426 | -13 % | 0.869 | 1P util 0.91; kv alone does not change heat->torque |
| kv 1300 (same scaling) | 0.0439 | -10 % | 0.873 | |
| 3115_640kv (library, bigger/heavier) | 0 | infeasible | 0.20 | bell J too large, 1P torque + duty peak violated |
| collective +2 deg (rpm trims lower) | 0.0480 | -2 % | 0.924 | lower rpm does not pay: more mean torque/heat |
| collective -2 deg | 0.0525 | +7 % | 0.880 | infeasible (creep, jitter) |
| skew -3 deg | 0.0465 | -5 % | 0.903 | |
| skew +3 deg | 0.0515 | +5 % | 0.887 | infeasible (jitter) |
Motor DB has no similar-mass alternative with a bigger stator; heat->torque efficiency scales with Km (stator volume), so the only motor-side win is a lower-inertia bell (J_rotor). Rth is a guess so thermal numbers are relative only.

## Lever: hub (and bell) polar inertia, elasticity about -1.6
Real printed hub assembly (hub_fitting_assembly.stl, 7.05 g PA6-CF): Izz = 0.75e-6 kg m^2, versus the model's generic J_hub 5.49e-6 (scaled 2e-6 x (D/5)^3), so most of J_hub is an unmeasured guess for adapter/nut/fasteners. Cutting J_hub to 0.7x (3.84e-6) gives +22-24 % min-cr. But the extra 1P lag swing raises strip stress: hp margin falls to ~0.01 (target 0.08) and fatigue_stress to 0.85-0.9 (v5 is within 0.7 target).

Tolerance trials (nominal + 8 x +-5 % samples, study/fitting_balance_v5-style, skew -2 deg to restore 1P margin):
- J_hub x0.7 + skew -2: nominal min cr 0.0598 (+22 %), 8/9 feasible; random07 fails fatigue_stress (1.11).
- J_hub x0.75 + skew -2: nominal 0.0573 (+17 %), 8/9 feasible; random07 fatigue_stress 1.06.
- Lengthening the strip (+5/+10 %) to relieve it fails fatigue itself.
Verdict: v6 NOT saved (needs all 9 feasible). Lever is real but is gated by strip fatigue stress, not by heat; a v6 needs a fatigue fix (thicker/shorter strip or higher-fatigue steel) that is outside this study. Scripts: scratchpad probe/sens/lever (not committed).
