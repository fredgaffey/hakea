# v8 (10") blade-1 vs blade-2 strip load asymmetry
Per-blade checks (rotor.checks with blade-tagged groups), v8 nominal, Goodman binding lamina sm/sa in MPa:
| case | b1.flex0 | b1.flex1 | b2.flex0 | b2.flex1 | max abs M3, flex0 b1/b2 (mN m) | hinge lag b1/b2 (deg) |
|---|---|---|---|---|---|---|
| v8 T0.7 | 42/140 | 42/160 | 217/**455** (u 1.05) | 104/261 | 142/340 | 3.44/3.94 |
| v8 T1.0 | 74/65 | 69/88 | 283/**361** (u 0.88) | 141/164 | 228/414 | 3.71/4.46 |
| v8 T1.0, pre-lag 0 and presets 0 | 88/45 | 128/59 | 454/**350** (u 0.96) | 26/167 | 253/526 | 2.49/4.18 |
v8 presets: flex0 hub/blade θ2 is -2.45/+2.17 deg on b1 and +4.38/-3.87 deg on b2. Pre-lag resolves to ±4.40 deg about the pitch axis.
End rotations at T1: the b1 strips have almost no twist about the strip axis (-0.02 and -0.19 deg). The b2 strips are twisted (-0.67 and +1.74 deg).
The b2 root also carries 1.5 deg of flap rotation, against 0.1 deg for b1 (blade-local frame).
Verdict: the pre-lag sign and the presets do not cause it. With both removed, the asymmetry gets LARGER.
It is not a leading/trailing pulse effect either: it is already there in the static hover trim (sm and hinge lag differ before any 1/rev load).
Source: rotor.build_model mirrors the even blades with fact.mirror_member (z -> -z):
`mem_local = [fact.mirror_member(m) if k % 2 == 0 ...]`. Blade 2's flexure is therefore the flap-mirror of blade 1's.
The mirror gives the intended opposite pitch-lag coupling (fact.hinge_axis), but the thrust/flap and pitch loads are NOT mirrored.
Each flexure sees (Mx, My, Mz) on b1 and (-Mx, -My, Mz) on b2. On b1 the skewed cross adds the pitch/flap moment to the lag moment.
On b2 it subtracts it. So the b2 strips twist and carry about 2x the M3 of b1.
This is real physics for a rotor built with a mirrored blade-2 flexure. It is not a code sign bug, but it follows from the model's geometric choice.
The one equalising test: re-resolve the presets on all 6 components, including strip twist (θ1, indices 0 and 3).
It DIVERGED: the residual went 1.6 -> 4.0 deg after Aitken and presets reached 20 deg. b2 fatigue u at T0.7 rose from 1.05 to 1.93. Rejected; no v9.
Next lever (untested, needs per-blade geometry in the schema): build blade 2's flexure as blade 1's rotated 180 deg about the pitch axis.
That keeps the opposite pitch-lag sign and loads both flexures alike. Per-blade skew or width would also work.
