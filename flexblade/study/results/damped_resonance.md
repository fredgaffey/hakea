# Damped-resonance criterion for the mech mode (opt-in `SA_GATES=damped`, `robust.damped_gates`)
**Current gates.** Under CAL the mech mode is exempt from the RES_BAND 1P check (`resonance_margin(skip_1p=im)`). It is gated only by the raw band `cqc_mech_margin` ≥0.03 and the side flip. `tol.py` (criterion "margin") applies `margin_1p_eff` ≥0.15 to it, which gives v10m its 0.080. ζ comes from the modal energy, `Im(x*Zx)/(2ωx*Mx)`, where Z is the complex K (steel η 0.001) minus aero Ka. Steel contributes ~4e-5, so ζ ≈ 0.04 is almost all aero, and it rises with throttle (0.037 at T0.7 to 0.050 at T1.4).
**Criterion.** The mode is modelled as a single-DOF mode (SDOF) at p/rev, forced at 1P: H = 1/(1−s²+2iζs), with s = 1/p. Over p·(1±δ) it must meet (a) gain spread |A/A₀−1| ≤ 0.25 and phase spread ≤ 15° (the CQC limits), and (b) worst amplification A ≤ 4.10. 4.10 is the undamped amplification at RES_BAND 0.15, so (b) is the project's own margin_1p_eff rule taken over the whole tolerance band.
**Required margin |p−1| vs ζ** (`robust.damped_required_margin`):
| ζ | δ 0.05 below/above | δ 0.02 below/above | δ 0.01 below | δ 0 (cap only) below |
|---|---|---|---|---|
| 0.02 | 0.27 / 0.24 | 0.12 / 0.17 | 0.11 | 0.10 |
| 0.04 | 0.27 / 0.23 | 0.115 / 0.17 | 0.107 | 0.098 |
| 0.10 | 0.24 / 0.15 | 0.075 / 0.115 | 0.065 | 0.058 |
| 0.15 | 0.10 / 0.12 | 0 | 0 | 0 |
- Below 1P the response is mass-controlled, A ≈ 1/(s²−1). It barely depends on ζ until the margin falls below ~2ζ, so damping buys margin only once ζ ≳ 0.08.
- With δ = 0.05 and no per-unit calibration, the gain-spread limit fails everything, including v8 (0.38–0.76). ζ = 0.10 still gives 0.31. Per-unit calibration (CAL) is therefore mandatory, and the realistic δ is the residual drift after it (≤0.02).
- Frequency tolerance: the CQ ±5 % corners move p by ±2.7–4.7 %. That spread comes from the hinge radius, not strip stiffness: the CF share is 0.92, so ±10 % strip k gives only about ±0.4 % in frequency.
**v8 / v10m** (scratch evals: trim + modes + harmonic, nominal + 4 CQ corners). Format is p, then ζ (model), then δ = 0.02 verdict at model ζ / at ζ = 0.10:
| J_hub | T | v8 | v10m |
|---|---|---|---|
| 1.6e-5 (design) | 0.7 / 1.0 / 1.4 | .864 .039 OK/OK · .846 .045 OK/OK · .829 .054 OK/OK | .889 .033 **X**(A 4.40)/OK · .875 .037 OK/OK · .862 .043 OK/OK |
| 3.76e-6 (tol.py) | 0.7 / 1.0 / 1.4 | .914 .043 **X**(A 5.7)/OK · .888 .051 OK/OK · .865 .061 OK/OK | .945 .037 **X**(A 9.3)/**X**(A 4.5) · .926 .043 **X**/OK · .908 .050 **X**/OK |
- Worst build corner (p0, δ 0.01, its own ζ): v10m at light J has p 0.979 and A 15 at T0.7. v8 has A 9.9 at T0.7 and light J. The light-hub J and T0.7 are the binding cases for both designs.
- Verdict: v10m fails at its model ζ. Even at ζ = 0.10 it still fails T0.7 at light J. v8 also fails (b) at T0.7 and light J, which the default gates do not see because the mech mode is exempt from the 1P check.
**Passive damping estimate**, using ζ_add ≈ η·SEF/2, where SEF is the share of the mode's strain energy in the damped part. The elastic share of the mode stiffness is 1 − f_cf ≈ 0.08 (cf_invariant_mode.md):
- Constrained-layer viscoelastic damping (CLD) tape on the strips: a 0.05 mm acrylic VEM (η ~1 near 20 °C and 150 Hz) under a 0.05–0.1 mm steel foil gives a composite η_c of about 0.1–0.3 (Ross–Kerwin–Ungar). ζ_add = 0.1–0.3 × 0.08 / 2 ≈ 0.004–0.012. The layers also raise strip EI by 30–100 %, which lifts the elastic share to 0.10–0.16 and pushes p up by 1–4 % toward 1P. That is a net loss, and it adds a fatigue concern for the tape.
- Aramid strap in a silicone sleeve: the strap is the CF tension member and barely stretches in this mode. Silicone (E ~2 MPa) next to aramid (~70 GPa) gives SEF ~1e-4, so ζ_add < 0.001.
- A viscoelastic shear pad across the hinge adds the most damping, but the stiffness it adds raises the frequency: Δp/p = ζ_add/η. +0.06 ζ costs ≥6 % in p (η ≤ 1, and η is temperature-sensitive), so v10m moves from 0.93 to about 0.98, which is worse.
**Net:** a realistic passive total is ζ 0.045–0.06 (≥0.10 is not credible passively; aero damping, set by the Lock number, is the only real lever). At ζ ≈ 0.05 the damping-aware requirement is margin ≈ 0.11 (δ 0.02, mode below 1P), against RES_BAND's 0.15. This gate is less strict than RES_BAND but stricter than the 0.03 band. It does not admit v10m. Defaults are unchanged; Fred decides.
