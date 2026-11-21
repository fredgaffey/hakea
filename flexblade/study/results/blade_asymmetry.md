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

## v8 + precone x skew scan (sa.py file, CAL continuous + Goodman + ground_res; skew add on top of v8)
| precone | skew add | mech margin | flip | CAL resid (phase deg / gain) | fatigue util (first nominal pt, strain/stress; member not labelled by sa.py) | cr_pl | feasible |
|---|---|---|---|---|---|---|---|
| 1.3 | +2 | 0.037 | yes | 8.1 / 0.107 | 0.50 / 0.36 | 0.101 | no (resonance 0.055, mech_side) |
| 0.9 | +1 | 0.024 | yes | 8.6 / 0.103 | 0.54 / 0.40 | 0.104 | no (mech_margin, mech_side) |
| 0.9 | -1 | 0.004 | yes | 9.2 / 0.120 | 0.66 / 0.48 | 0.128 | no (mech_margin, mech_side) |
| 0.9 | +3 | 0.056 | yes | 5.6 / 0.102 | 0.42 / 0.31 | 0.086 | no (mech_side flip only; margin-criterion feasible) |
Result: mech margin rises with +skew, but mech_side_flip stays true in all four, so none is feasible; no v9 saved (v8 stays the lead, cr_pl 0.0715). Next: precone 0.9 with skew add +3..+4 plus a preset re-resolve (zero_stress_trim) targeting the flip, or a precone of 0.5-0.6.

## v9 attempt: presets re-solved (rotor.resolve_strip_preset, 15 its, resid floor ~0.014 deg) on v8 + precone + skew
| precone | skew add | cr_pl | feasible | violations |
|---|---|---|---|---|
| 0.9 | +3 | 0.100 | no | resonance 0.151 (pitch antisym+hub 0.92/rev vs 1P), harmonic_jitter 1.46 |
| 0.5 | +2 | 0.100 | no | resonance 0.259 (pitch antisym+hub 0.93/rev vs 1P) |
Re-solving presets removed the mech_side_flip violation (it was the preset mismatch) and lifts cr_pl to 0.100, but the pitch antisym+hub mode now sits at 0.92-0.93/rev, inside the 1P band. v9 deleted; v8 stays the lead.
Next: re-solve presets plus a pitch-mode retune (skew/length) to move that mode above ~1.0/rev, or below 0.85.

## v9 modal scan (presets re-solved via resolve_candidate, J10 hub range, T0.7/1.0/1.4) + sa.py scoring
- Mechanism mode sits BELOW 1P: nominal v8 0.93/0.90/0.87 per rev (T0.7/1/1.4); the 0.92 "pitch antisym+hub" violation is the margin criterion, the cqc blocker is mech_side_flip (perturbed p0 variants reach 0.96-0.97 at T0.7 and cross 1P at low nodes). Need mode <=~0.88 at T0.7 (RES_BAND 0.15).
- Levers: skew +1..+5 lowers it (precone .5: +1 0.93, +2 0.92, +4 0.88 @T0.7); length x0.95/x0.9 and thickness x1.05/x1.1 RAISE it (0.94-0.96), wrong way; length x1.05 lowers it (0.90) but at T1.4 the mode jumps to 1.11 (mode switch above 1P) - avoid.
- sa.py (CAL+Goodman+ground_res): precone 0.5 skew+2 cr_pl 0.092 (flip, margin-crit resonance 0.92); precone 0.5 skew+4 cr_pl 0.076 (flip, harmonic_jitter 0.013, margin-crit resonance clean 0.19); precone 0.9 skew+5 cr_pl 0.0795 (flip, harmonic_jitter 0.044, 0.88/rev pitch antisym+hub).
- None feasible (cqc_mech_side flip persists even at 0.88/rev); no v9 saved, v8 stays lead.
