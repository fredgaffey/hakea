# Strap strand audit, 5in_v8s_2307_m245_301 (members[2]/flex2, aramid_yarn, d 1.3 mm, A 1.33 mm2)
- Material (materials.py:318): E 112 GPa, Xt 2.4 GPa (Kevlar 49 filament 3.0 x 0.8), creep allow 0.0086, fatigue strain amp allow 0.004, source DuPont guide (filament data).
  The 0.8 is a "translation efficiency" guess, no cord test; fatigue amp 0.4 % has no cited source and no flex-fatigue/compression derate. Aramid cord flex fatigue is poor: treat 0.4 % as optimistic, 0.25 % more defensible.
- Loading: cable=True only scales EI/GJ by bend factor 0.001 (rotor.py:1662). It is a linear beam, NOT tension-only; no slack/compression logic anywhere (grep).
  Mean strand tension is small (T0.7 75 N, T1 100 N, T1.25 122 N = 0.05-0.08 % strain) while the 1/rev fibre strain amplitude is 0.26 / 0.20 / 0.16 %.
  If that amplitude is axial-dominated (not verified; per-phase min tension not exposed) the strand goes slack every rev at all three thrusts: snap-loading, not modelled.
- Termination: model HAS a knot factor (rotor.py:1677, knot 0.45) but the design file says strap_anchor=pin_wrap (D/d 3.3, eta 0.72, cap 2308 N); BUILD.md says knot, so model and build disagree.
  Static util with knot 0.5 (cap 1593 N): T0.7 0.047, T1 0.063, T1.25 0.077 (vs pin_wrap 0.032/0.043/0.053). Strength is not the issue; fatigue/slack is.
  (endutil.py has no J argument; strap numbers come from R._evaluate checks strap_tension/strap_anchor.)
- Verdict: static strength fine even with knot; fatigue 0.665 at T0.7 is UNVERIFIED and likely optimistic (no cord derate, slack not modelled, knot in a well chafes aramid). Do not trust before a bench cycle test.
- Recommend: printed capstan/pin wrap (2+ turns round a >=6 mm PETG post, D/d>=4.6 gives eta~0.77), tail CA-soaked and trapped in the well. No knot. Pretension the strand ~20-30 N so it never goes slack. Bench: 1e5-cycle flex at +-0.25 % about 0.1 % mean, plus pull test.

## Axial vs bending
Lead strap b2.flex2, J 0.91e-6, pulse 0.95 x motor capability (`study/strap_axial_audit.py`, raw `results/strap_axial_audit.jsonl`). Values are T0.7 / T1.0 / T1.25.
- Axial eps0 mean is 5.03e-4 / 6.75e-4 / 8.21e-4 (75 / 100 / 122 N). The 1P amplitude is 3.2e-5 / 2.8e-5 / 2.5e-5. The minimum over the rev is 4.70e-4 / 6.46e-4 / 7.95e-4 (70 / 96 / 118 N).
- **The strand never goes slack.** The 1P axial swing is only 3-7 % of the mean, so the linear cable model holds. The "slack every rev" concern above is wrong, and pretension is not needed for that reason.
- Bending part of the max fibre strain: static 1.69e-3 of 2.19e-3, 2.05e-3 of 2.73e-3, 2.35e-3 of 3.17e-3. The 1P amplitude is more than 99 % bending (3.60e-3 of 3.63e-3 at T0.7; 1P curvature 190 / 140 / 111 1/m).
- Where the 1P part forms: `harmonic_1p` gives the complex response U. In `checks()`, `element_end_forces_lin` turns it into element forces, then `_fibre_amp` takes |eps0 + z k4 - y k5| per fibre. Cables get no mean term (Goodman is shim steel only), so fatigue_strain = amp / 0.004.
- Premise check: the default does NOT bend over the full cord radius. `_cable_section` already scales the fibre offsets by sqrt(0.001), so the lever is 20.6 um. Axial mode uses r_fil = 6 um, 3.4x lower.
- `CABLE_STRAIN=axial` is opt-in. The default path and non-cable members are bit-identical, and the cache key is tagged. In axial mode, fatigue and creep use eps0 + kappa r_fil; static strength is unchanged.
- Strap fatigue_strain goes from 0.905 / 0.668 / 0.538 to 0.294 / 0.217 / 0.173. Creep goes from 0.255 / 0.317 / 0.368 to 0.116 / 0.148 / 0.175. The remaining amplitude is still about 97 % filament bending, so it scales with r_fil, which is a guess.
- cr_pl is unchanged in both modes: 0.439 / 0.284 / 0.180 (min 0.180). Binding is motor_torque_1p / motor_torque_1p / fatigue_stress flex0 (strip). The strap was 2nd at T0.7 (0.905 vs motor 0.925); in axial mode it drops out of the top 3.
- Anchor: the lead JSON is now `strap_anchor: knot` (η 0.45), matching the print and BUILD.md. strap_anchor util goes from 0.032/0.043/0.053 to 0.052/0.070/0.085, and cr_pl is identical at every T. If the build moves to the pin wrap recommended above, switch the anchor back.
