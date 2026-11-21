# Strip lateral-torsional buckling (LTB) under in-plane M3: v5 / v7 / v8 (10") and v6 (7")
Model: laminae are UNBONDED (laminate.laminate_section): EIy, EIz, GJ = n x single lamina, so M_cr ~ n w t^3 / L (not n^2).
M3 = max |M3| per strip, from beam.element_end_forces(e, c, st) (spatial nodal action in the end-section frame), at robust T0.7/1/1.4 trim.
Uniform-moment clamped formula M_cr = k pi sqrt(EIy GJ)/L, k = 2 (L_e = L/2): 389 mN m (10", L 25.1 mm), 235 mN m (7", L 12.5 mm).
The CF does NOT put the strips in tension: every strip is in COMPRESSION, N = -2 to -36 N (strap carries the CF; fixed-fixed Euler 82 N
for 10"). Compression lowers M_cr slightly, so nothing is gained from CF.
Beam check: alpha = linear buckling factor of the strip in its own trim stress state, both ends clamped, from the beam's consistent
initial-stress Hessian K_sigma (K_mat + alpha K_sigma singular). Validated on a 6-element strip: uniform M3 gives M_cr 423 (classical 389,
+9 %); M3 tapering linearly to 0 gives 711 (C1 ~ 1.8). In the rotor, M3 drops from the hub clamp (v8 b2.flex0 T1: 414/309/219/144/85/41/14).

| design | T | worst M3/M_cr,uniform (strip) | min alpha (strip) | util 1.5/alpha |
|---|---|---|---|---|
| 10 v5 | 0.7 / 1.0 / 1.4 | 1.15 / 1.33 / 1.53 (b2.flex0) | 2.19 / **1.77** / 2.42 (b2.flex1) | 0.68 / **0.85** / 0.62 |
| 10 v7 | 0.7 / 1.0 / 1.4 | 1.00 / 1.19 / 1.40 (b2.flex0) | 3.11 / 3.24 / 3.91 (b2.flex1) | 0.48 / 0.46 / 0.38 |
| 10 v8 | 0.7 / 1.0 / 1.4 | 0.87 / 1.06 / 1.29 (b2.flex0) | 4.77 / 4.99 / 4.94 | 0.31 / 0.30 / 0.30 |
| 7 v6  | 0.7 / 1.0 / 1.4 | 0.75 / 0.98 / 1.19 (b2.flex1) | 1.95 / 2.05 / 2.01 (b2.flex1) | 0.77 / 0.73 / 0.75 |
b1 strips carry about half of b2's M3 (b1 flex0 141-360 mN m). The strip with the lowest alpha is not the one with the highest M3: compression and
the shape of the M3 distribution matter as much as the peak.

Cross-check, v8 b2.flex0: the beam is geometrically nonlinear and its K_sigma carries M3 (Wagner and CR terms). The LTB-mode stiffness falls by
1/alpha: 17 % (T0.7), 19 % (T1.0), 20 % (T1.4). It does not collapse, because the effective M_cr (alpha x M3 ~ 2 N m) is about 5x the uniform-moment value.
The uniform-moment ratio 0.87-1.06 is a conservative screen, not the margin. On the beam's own measure v5 is the closest lead (alpha 1.77).
Tolerance: M_cr ~ t^3, so a lamina 5 % thin loses 14 % (v5 at T1: alpha ~1.5).

Added hard check `strip_ltb` (rotor.checks, shim strips, worst blade): util = M3 / (M_cr,eff / 1.5) = 1.5/alpha, with M_cr,eff = alpha M3.
The note also prints the uniform-moment M_cr. Test: tests/test_strip_ltb.py. All four leads pass (max 0.85, v5 T1).

Cheapest lever: thickness and length act hardest on M_cr (~t^3 n w / L), but t raises strain about 1:1 and shorter L raises hinge stiffness.
Lamina count is about neutral: M_cr ~ n, but the in-plane load share EIz ~ n too. Width trades M_cr ~ w against EIz ~ w^3 load attraction, so a
NARROWER strip lowers M3/M_cr if M3 is stiffness-attracted. The cheapest fix removes the source: react the lag moment as an axial couple through
the strap or the cross-pair angle, so the strips stop carrying M3. b2 carrying 2x b1's M3 points to preset/pre-lag asymmetry as a first, free target (untested).
