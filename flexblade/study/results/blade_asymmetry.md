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

## Precone test (blade.precone_deg; study/asym/precone_my.py, asym_diag.py with PRECONE=)
Steady flexure-root My (rotor.flexure_root_loads, about the hinge point) at T1.0: b1/b2 = +49/+232 mNm at beta0 0. Both blades drop by ~150 mNm/deg.
No single beta0 zeros both: b1 is zero at 0.33 deg, b2 at 1.47 deg, and the mean at 0.92 deg (-130/+120). The b2-b1 gap of ~180 mNm is the same for every beta0 (T0.7 too).
The gap is set by the mirrored flexure. It grows to 400 mNm without presets or pre-lag. Precone moves both blades by the same amount, so it does not close the gap.
Goodman flex0 sm/sa (MPa), b1 | b2, with worst fatigue u:
| beta0 | T0.7 | T1.0 | worst u T0.7 |
|---|---|---|---|
| 0 | 42/140 \| 217/455 | 74/65 \| 283/361 | 1.28 (b2.flex2 aramid), b2.flex0 1.05 |
| 0.92 | 138/229 \| 164/351 | 184/160 \| 198/222 | 1.00 (b2.flex2) |
| 1.25 | 159/290 \| 150/292 | 263/212 \| 179/172 | 0.875 (b2.flex2); flex0 0.64/0.64 |
| 1.47 | 175/331 \| 143/250 | 301/258 \| 168/141 | 0.92 (b1.flex2) |
Even though My never balances, beta0 ~1.25-1.3 deg evens the strip STRESSES and removes the b2 fatigue limit (worst u 1.28 -> 0.875).
sa.py on v8 + precone 1.3 (presets unchanged): cr_pl 0.121, but INFEASIBLE. The violations are cqc_mech_margin 0.72 and mech_side_flip; mech_per_rev is ~0.94 at T0.7, against margin 0.036 on v8.
MARGIN also shows a resonance violation (pitch antisym+hub 0.94/rev vs 1P). So no v9. The preset re-resolve (zero_stress_trim at alpha 0.55) was not tried (budget).
Next: precone 1.3 plus a re-tune that moves the pitch mode back off 1/rev (skew/length), or a smaller precone (~0.9) as a compromise.
