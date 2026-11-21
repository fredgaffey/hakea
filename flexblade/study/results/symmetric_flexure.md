# Blade-2 flexure arrangement vs blade mismatch (v8, T1.0, s*=0.907; `study/symmetric_flexure.py`, raw `symmetric_flexure.json`)

**Sign argument.** A 1P torque pulse loads both blades the same way in their own frames: lag ζ1 = ζ2 (the common hub swing). Cyclic means θ2 = −θ1, so dθ/dζ must have OPPOSITE signs on the two blades. C2 (rotation by 180° about the shaft) makes blade 2 the same as blade 1 in its own frame, so the same lag gives the same pitch: a pure 1P collective, with no control.
Write blade 2's compliance as C2 = T C1 T, with T = diag(±1) on the rotations (pitch x, flap y, lag z). Flipping C_xz needs s_x s_z = −1. Then s_x s_y · s_y s_z = −1, so exactly one of C_xy and C_yz also flips:
mirror z → (−,−,+) flips pitch-lag and flap-lag. rot_x (180° about the pitch axis) → (+,−,−) flips pitch-lag and pitch-flap. No T flips the diagonal terms.
So per blade: θ = C_xx·fx (always common) ± C_xz·fz, and by reciprocity ζ = C_zz·fz ± C_zx·fx. A COMMON 1P feathering moment fx gives a collective pitch and opposite lag on the two blades, whatever the arrangement.
A flexure symmetric about the span–rotor-axis plane (y → −y; C_xy = C_yz = 0) makes mirror_z and rot_x the same, but it does NOT remove the fx terms. The mismatch is a blade-load property (fx), not a flexure-arrangement property.

**Model option** (24 lines): `flexure.blade2_flexure` = mirror_z (default) | rot_x | c2 (fact.blade_flexure_T, rotor K6/fitting maps). Also `flexure.pitch_link_stiffness` (N m/rad on θ1+θ2): a see-saw collective-pitch link between the blade roots. Model only; CAD/fittings still mirror z. Cases below have presets = 0 unless marked P.

| case | b1 flap/lag/pitch75 deg | b2 flap/lag/pitch75 deg | cyc / coll pitch | 1P vert N | 2P in-plane N | steady ctrl M N m |
|---|---|---|---|---|---|---|
| mirror_z P (v8 nominal) | 0.36/2.04/4.41 | 2.58/2.20/9.44 | 6.92/2.54 | 11.0 | 0.45 | 0.227 |
| mirror_z | 0.12/1.71/3.56 | 2.63/1.48/6.80 | 5.16/1.67 | 12.3 | 2.67 | 0.152 |
| (a) c2 | 0.66/1.88/4.81 | identical | **0**/4.81 | 13.7 | 0 | **0** |
| rot_x | 0.14/3.21/6.56 | 2.53/3.99/11.06 | 8.81/2.26 | 14.5 | 0.62 | 0.193 |
| (b) _sym topology, mirror_z / rot_x | 0.08/1.15/2.39 / 0.21/2.21/4.70 | 3.21/1.14/7.43 / 1.41/4.06/6.14 | 4.86/2.62 / 5.42/0.72 | 17.4 / 6.5 | 4.7 / 21.9 | 0.13 / 0.14 |
| (c1) link 100 N m/rad, mirror_z P | 0.13/2.26/4.06 | 2.12/0.90/4.03 | 4.04/**0.04** | 8.4 | 32.9 | 0.132 |
| (c1) link 100, rot_x | 0.10/1.51/3.02 | 1.29/0.39/3.01 | 3.01/0.03 | 8.5 | 23.6 | 0.092 |
| (c2) mirror_z, no z balance masses | 0.89/2.49/6.50 | 1.38/2.91/8.51 | 7.49/1.07 | **2.8** | 1.77 | **0.355** |
| (c2) same + precone 1.3 | 3.73/2.62/11.30 | 0.34/4.05/8.65 | 9.96/1.45 | 20.8 | 11.7 | 0.496 |
Other balance-mass settings (z ±10 mm at 0.5 g: 9.4 N; chordwise ±10 mm at 1–2 g: 17–19 N, 5–9 N in-plane) and EA 0.25/0.45 were all worse than (c2) on at least one channel. The response swings strongly because the pitch mode sits near 1/rev.

**Verdict.** (a) C2 gives the wrong sign: identical blades, zero cyclic, and the worst vertical force. No flexure arrangement makes the blades identical, because rot_x, mirror_z and the _sym cross all keep the common-fx terms.
(c1) The collective-pitch link equalises pitch exactly (0.04° collective). Its reaction moment is a common fx, though, so by reciprocity the lag splits (2.3 vs 0.9°), the 2P in-plane force rises to 24–33 N, and cyclic falls by 40 %. Rejected.
Best lever found: (c2) null the common 1P feathering moment at the hinge (blade chordwise mass/AC and Ω-dependent propeller-moment balance), keeping mirror_z. With the z balance masses removed: vertical 11 → 2.8 N and cyclic authority +56 %. In-plane 2P is 1.8 N at precone 0, but it does not survive precone 1.3. Not yet checked: stability, ground resonance, fatigue.
Next: add the 1P fx at each hinge (rotor.flexure_root_loads on hd.U) as an sa.py objective/constraint to drive it to zero, rather than hand-scanning masses.
