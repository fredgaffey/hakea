# T0.7 blade-end fallbacks for 5in_v8s_2307_m245_301 (beam only, no ccx)
Superseded premise: b2.flex2 is the aramid strap strand, not steel; corrected steel blade-end util at T0.7 ~0.71, and under pulse_limited fatigue sets the pulse scale (not hard). Screen kept for the record.
Setup: J_hub 0.34/0.91/2.0e-6 (min cr_pl over J per TF), vehicle 0.245 kg, ground-res on, presets resolve_candidate(d,6), margin criterion. Script: flexblade/scratch/t07.py.
| variant | cr_pl T0.7 / T1.0 / T1.25 | hard | fatigue_strain max util T0.7 (unscaled) |
|---|---|---|---|
| lead 9x4.0x0.05, 105 | 0.322 / 0.211 / 0.167 | none | 1.144 |
| (a) t 0.04, n 12 (schema cap 12; n.t^3 only 0.68x lead) | 0.317 / 0.212 / 0.153 | resonance 0.115 | 1.079 |
| (b) w 3.0, n 12 | 0.319 / 0.211 / 0.166 | none | 1.081 |
| (c) 95 deg | 0.307 / 0.208 / 0.124 | none | 1.049 |
| (d) 115 deg | 0.337 / 0.219 / 0.169 | none | 1.312 |
Per-end raw strain (x clamp k, hub/blade, worst strip) at T0.7: a 0.89/0.83; b 0.76/0.91; c 0.60/0.90; d 0.91/1.10 (lead hook returned none). In-plane share not exposed by endutil: omitted.
Verdict: none qualifies. (b) cuts T0.7 strain util most with no hard violation (-5 %) but does not beat the lead's min cr_pl; (c) cuts 8 % but T1.25 cr_pl falls to 0.124; (d) is worse on strain. No t07fix design saved; keep the lead.
