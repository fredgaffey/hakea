# Arm vertical forced response (10in v8 lead, study/arm_vertical.py)
Model: 1-DOF modal arm per direction, modal mass 0.25 x 1.697 kg = 0.424 kg (system_check), zeta 0.02, force at hub harmonic h x 1P.
Only T1.0 forces are in two_per_rev.json; T0.7/1.4 scaled F ~ T (rpm^2), 1P = 108-152 Hz (rpm x sqrt2), 2P = 215-305 Hz, 3P 323-457 Hz.
Tip amplitude and transmitted force vs f_arm 80-400 Hz are in results/arm_vertical.json (worst over T0.7-1.4).

| case | force at T1.0 | f_arm with tip > 50 um |
|---|---|---|
| precone 0, 1P vertical | 11.0 N | 80-203 Hz (peak ~980 um at 128-150 Hz, ~260 N transmitted at 128 Hz) |
| precone 1.3, 1P vertical | 1.55 N | 106-165 Hz (peak ~123 um) |
| 2P vertical (both) | 1.9-2.0 N | none (<= 12 um above 120 Hz) |
| 3P lateral, precone 1.3 | 2.65 N (circular; per-axis = magnitude) | none (2 um) |
| 2P lateral from 1P flap/lag, precone 1.3 | 9.75 N (hubF_1p_fixed key 2; circular, per-axis = magnitude) | 205-339 Hz, 41 um at 200 Hz |

Dynamic magnification > 3 spans f_arm 0.87-1.22 x the excitation, so amplitude was used as the forbidden-band criterion.
Requirement (10 in): f_x, f_y >= 120 Hz (ground resonance) and f_z >= 210 Hz at precone 0; f_z >= 170 Hz with precone 1.3.
The existing 120 Hz arm sits in the worst place for 1P vertical (861 um at 120 Hz, no precone). Precone 1.3 relaxes it; going below 100 Hz not advised.
In-plane nP forces are circular in the fixed frame, so the earlier hypot(|Fx|,|Fy|) overstated them by sqrt2 (13.8 -> 9.75 N, 3.75 -> 2.65 N); fixed in arm_vertical.py.
The 9.75 N 2P lateral (precone 1.3) is real: it equals the global inertial sum (2 Omega^2 sum m u_xy + lag Coriolis) of the mismatched 1P blade motion (mirrored flexures, about 1.4 deg differential lag, about 0.4 mm net CG whirl); precone couples the differential flap into lag/radial CG motion (0.45 N at precone 0). The in-plane arm must avoid f_x, f_y = 205-339 Hz.
Caveats: F ~ rpm^2 scaling assumed; series mount compliance and rotor mass on the arm are ignored (it would lower f_z).
