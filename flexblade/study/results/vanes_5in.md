# Hakea 5" anti-torque vane ring (single rotor, 2307 1300KV, 6S, AUW 0.245 kg)
Script `study/vanes_5in.py` (rotor stations cached in `results/vanes_5in.json`; `rerun` re-evaluates the vehicle_v0 5" design at T0.7/1.0/1.25). All numbers are estimates.
- **Wake assumptions:** the model's BEM v_i(r) and dQ/dr per annulus. Swirl v_th = dQ/(mdot r); dQ is rescaled x1.05 to the trim Q. Actuator-disc contraction: v_z = v_i(1 + z/sqrt(z²+R²)), area conserved per streamtube, r·v_th conserved.
- **Vane assumptions:** strip cascade, with momentum dQ = mdot r (v_th,in - v_th,out) solved together with section lift at the vector-mean angle. Section: 5 % arc plate, CL_alpha = 2π x0.9 (Re 2-6e4) x0.9 (cascade), alpha_L0 = -2f/c, CLmax/min 1.0/-0.3, CD = 0.035 + 0.03 CL². Stall is also checked with the Lieblein DF.

## 1. Torque and swirl
| | rpm | T (N) | Q (N m) | mean v_i | disc swirl v_th / angle at 0.4, 0.6, 0.8R |
|---|---|---|---|---|---|
| T0.7 | 11.4k | 1.68 | 0.0194 | 7.4 m/s | 5.2/4.1/3.5 m/s, 40/27/21 deg |
| T1.0 | 13.1k | 2.40 | 0.0263 | 8.8 m/s | 5.7/4.6/4.0 m/s, 36/26/20 deg |
| T1.25 | 14.3k | 3.00 | 0.0320 | 9.8 m/s | 6.0/5.1/4.4 m/s, 34/25/20 deg |
- v_i rises outboard, from 7.1 m/s (0.35R) to 14 m/s (0.95R). v_th = (dQ/dr)/(2π rho r² v_i) falls with r.
- Fixed pitch with rpm control means the velocity triangles scale with Omega: Q ~ T (model T^0.93, not T^1.5), v_th and v_z both ~ T^0.5, so the swirl angle is about constant over T.
- Vane plane (mid-chord 48 mm = 0.76R, T1.0): the wake edge is at 0.79R, mdot 0.16 kg/s. v_z / v_th / beta = 11.3/7.7/34 deg at 0.28R, 15.5/5.9/21 at 0.47R, 17.3/5.1/16 at 0.63R, 22.5/3.8/10 at 0.75R.

## 2. Geometry (recommended: 6 vanes, leading edge 0.5R = 32 mm below the disc)
- 6 identical vanes: 5 % camber, constant chord **33 mm**, span r 17-56 mm (root clears the bell; tip at the outer ring, 0.88R).
  Twist: stagger = half the local swirl angle (17 deg root, 9 deg at 0.6R, 0 deg at 0.8R), so the leading edge meets the swirl and the trailing edge is axial.
- At T1.0 the fixed set (servo vanes neutral) returns Q: alpha -2.4..+2.0 deg, CL 0.30-0.69, DF ≤ 0.33, solidity 0.83 at 0.6R.
  Chord for N4/6/8 = 48/33/25 mm (ring 21.6-22.1 g); with LE at 0.15R, 52/37/29 mm (~23 g). N6: 2 opposite servo vanes = 1/3 of the set; with B=2 and N even, blade-wake side forces cancel and the yaw tone sits at 6P.
- **Finding:** the cascade returns ~mdot r v_th, and both that and Q scale ~T. The fixed vanes give 100.9/100/99.7 % Q at T0.7/1.0/1.25, so trim needs only **±0.3 deg** of servo.
  The spec's -16/+12 % assumed Q ~ T^1.5. All servo authority is therefore left for model error, CG and gusts.

## 3. Yaw control (servo vanes, T1.0)
- Authority: **+25 % Q (+6.6 mN m) at -8..-10 deg** (servo vane reaches CL 1.0, stall-limited) and **-28 % Q (-7.3 mN m) at +10 deg** (-40 % at +15). Throw: -8..+12 deg, one micro servo.
- Izz ≈ 1.28e-4 kg m² (est. table in the script; battery 47 %, spinning parts excluded), so yaw acceleration is ±52-57 rad/s². Vane yaw damping 2.4e-4 N m s gives a rate time constant of **0.54 s** and ~28 rad/s steady.
  The actuator lag (servo ~12 ms per 10 deg + 333 Hz PWM + vane lift ~4c/V) is ≲ 25 ms, so yaw is acceleration-limited: 90 deg of heading in ~0.35 s.
- Thrust: the vane axial force is **up 2.5 % T at trim** (swirl recovery, L sin(beta) > D cos(beta); bounded by swirl KE, ~5 % T est.). It drops to 1.0 % at +15 deg and 1.7 % at -10 deg, so yaw costs ≤ 1.5 % T. Budget 0 to -1 % net.
- A throttle punch T1.0 to 1.25 in 0.1 s gives a rotor spin-up reaction of 5.6 mN m = 21 % Q (J 4.3e-6 est.). That is comparable to the authority, so slew-limit the collective.

## 4. Coupling with the 1P cyclic pulse (T1.0, pulse-limited s = 0.71)
Model inputs: cyclic M 12.3 mN m; motor 1P torque 176 mN m; thrust_1p 0.36 N (15 % T); rpm ripple 0.6 %. The 1P inflow gain is 0.43 (apparent-mass lag 1.5 ms). Sears attenuation on the 33 mm chord: 0.35 at 1P, 0.26 at 2P.
| relative to M | LE 0.15R (lever 27 mm to CG) | LE 0.5R (lever 20 mm) |
|---|---|---|
| 1P vane yaw ripple (convected collective-torque swirl) | 1.9 mN m = 15 % | 2.0 mN m = 17 % |
| 1P side force: 0 by symmetry (B=2, N even); with 3 % blade mismatch | 3 mN, 0.7 % | 4 mN, 0.6 % |
| DC side force from the cyclic swirl pattern (dQ/Q = 1.5 dT/T), cross-axis | 0.11 N, 24 % (14 deg) | 0.12 N, 19 % (11 deg) |
| 2P side force (potential decay exp(-2z/r): 0.62 vs 0.18) | 19 mN, 4 % | 13 mN, 2 % |
- **Not an attitude problem.** The vane 1P yaw ripple is 1.2 % of the motor's own 1P reaction, which already puts 1.0 rad/s of 1P on gyro z (a yaw RPM notch is needed anyway). The DC side force scales with M: a fixed ~11-14 deg phase plus a 5 % W lateral force, absorbed by the phase calibration and position loop. The rotor's own 2P hub moment (32.5 mN m = 2.6 M) dwarfs the vane 2P.
- Spacing ≥ 0.5R does **not** reduce the 1P terms (vortical, they convect; ~2.7 ms delay). It cuts 2P vane loading ~3x, moves the ring nearer the CG and avoids a ring around the spinning bell. Use LE ≥ 0.5R.

## 5. Printable design and structure
- One PETG print, flange down: hub tube (r 15-17 mm, 33 mm tall, 3 mm flange, 2307 16x16 M3) + 4 fixed vanes (1.0 mm = 2 x 0.5 mm lines) + outer ring (r 56 mm, 6 mm tall, 1.6 mm wall), 117 mm OD.
  Vanes are vertical walls, with camber and twist as layer shift (≤ ~28 deg from vertical), so no supports. PETG under a ~60 °C motor base is near its HDT: glue the ring below the PA-CF mount, or print it in PA-CF on the X1C.
- Servo vanes: separate prints, 3.4 mm boss at 25 % chord glued to 2 mm steel rods in reamed 2.1 mm holes in the hub tube and the ring. They sit opposite each other (side forces cancel).
  One micro servo inside the hub tube (ID 30 mm, below the encoder) drives both through a double horn and 1 mm Z-bend links to 6 mm cranks.
- Mass: printed 17.9 g (hub 3.2, vanes 9.8, bosses 0.6, ring 4.3; with 0.8 mm vanes 15.9 g). Rods 2.5 + links 1.5 bring it to **21.9 g (19.9 g)**, plus a ~4.3 g servo: +4-6 g (~2 % AUW) over the budget's 15+5 g.
- Modes (E 2.0 GPa, beam formulas; 1P 190-240 Hz, 2P 380-480 Hz): cantilever vanes without the ring 263 Hz (too close to 1P). Ring torsion on 4 radial vanes **213 Hz, inside 1P**, driven by the 176 mN m 1P reaction.
  **Lean the vanes 15 deg tangentially** (spoke style) to raise it to 1.1 kHz. Vane bending 1.7 kHz; ring wall segment 600 Hz (keep the wall ≥ 1.6 mm).
