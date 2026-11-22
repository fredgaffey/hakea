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
