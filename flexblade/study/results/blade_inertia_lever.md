# Blade inertia / Lock-number lever on the mech mode (script blade_inertia_lever.py; presets re-solved 6 it; tol.py nominal, J 3.76e-6, T0.7/1/1.4)
**Theory.** Rigid blade, small rotation q about the skewed hinge axis n = (n_f, n_l, n_p) through the hinge at offset e. Blade axes: x span, y chord (in plane), z normal. Second moments are A = Σm x², B = Σm y², C = Σm z², and the first moment is S = Σm x.
CF stiffness per Ω²: flap A + eS, lag eS, feather B − C (propeller moment / tennis racket), plus the product terms Σm xy and Σm xz (chordwise or normal CG offset). Mode inertia: flap A + C, lag A + B, feather B + C. So p² = ω²/Ω² ≈ (1 − f_el)·n·K_cf·n / n·I·n + f_el, with f_el ≈ 0.08.
Pure pitch gives p² = (B − C)/(B + C), pure flap gives 1 + eS/A, and pure lag gives eS/A. For the pitch-dominated mode, 1 − p² ≈ 2C/(B + C) + (lag share)(1 − eS/A). p moves away from 1 (downward) when C/B rises. Normal-offset masses do that, as does any cut in blade B relative to the fixed balance/fitting C. Uniform density, thickness or chord all change B against the fixed z-masses.
Chordwise or LE mass raises B and pushes p → 1. Hinge offset e raises the flap/lag CF terms.
The gain mechanism (skew coupling lag→pitch, torque pulse) depends on n and the flexure, not on the blade mass. The cheap knob is therefore blade density. Aero damping follows the Lock number γ = ρ a c R⁴/I_b. Raising chord while cutting density (mass held constant) raises γ ∝ c, and with it ζ, the aero pitch forcing and the quasi-static gain.
**Results** (cr_pl, res_margin [binding], mech/rev at T0.7/1/1.4, mech ζ T0.7–1.4, hard viol):
| variant | cr_pl | res | mech/rev | ζ | viol |
|---|---|---|---|---|---|
| v8 base | .073 | .122 | .919/.891/.866 | .042–.060 | res |
| v10m base | .128 | .073 | .950/.929/.910 | .035–.049 | res |
| v8 ρ×0.6 / ×0.8 | .040 / .052 | .154 / .175 [pitch sym 1.15] | .846–.836 / .866–.834 | .036–.057 | jit .33 / none |
| v8 taper 0.7 / 1.3; chord 0.85 | .030 / .161 / .041 | .149 / .046 / .137 | .894 / 1.02 / .902 | .018–.05 | jit / res / res+jit |
| v8 no z-mass / tc .18 / z ±15 mm | .148 / .128 / trim fail | .047 / .042 / .056 | .987 / 1.06 / .94 | | res |
| v8 LE tip 0.3 g r.9 (alone / +ρ.6) | .097 / .047 | .088 / .174 | .939 / .819 | .036–.057 | res / none |
| v8 chord×1.15 ρ×0.76 | **.084** | **.158** | .903/.863/.832 | .052–.084 | none |
| v8 chord×1.3 ρ×0.59 / ρ×0.7 | .100 / .156 | .185 / .075 | .894 / .962 | .062–.104 | jit .99 / res+jit |
| v10m ρ×0.85 (E×0.85) | **.103 (.105)** | **.155 (.153)** | .900/.885/.870 | .040–.053 | none |
| v10m chord×1.15 ρ×0.68 (E×0.8) | .111 (.114) | .206 (.198) | .882/.858/.841 | .055–.077 | none |
| v10m chord×1.3 ρ×0.59 (E×0.7) | **.135 (.140)** | **.155 (.146)** | .908/.879/.857 | .059–.087 | none (res .025) |
| v10m ρ×0.75 / c1.15 ρ.6 / c1.3 ρ.5 | .078 / .085 / .094 | .193 / .217 / .247 [flap sym 1.2] | .869 / .831 / .832 | .043–.11 | none / jit .1 / none |
**Answer: yes, the frontier moves.** At res ≈ 0.155, v10m with a lighter blade (ρ×0.85) gives cr .103, and with a wider, lighter blade (c×1.3, ρ×0.59: same mass, γ×1.3, ζ ×1.7) gives .135. Both are clean in this nominal check, against v8's 0.0715. Cutting E to ×0.7–0.85 barely changes them. The frontier is not the same SDOF amplification curve. cr·(1−p²) stays ≈ 0.011–0.012 under pure density moves, but rises to 0.016–0.020 with chord↑/ρ↓, because Lock-number aero forcing adds to the inertial one. Lowering ρ alone on v8 just slides along the curve. Losers: taper, smaller chord, thicker t/c, and LE tip mass all move p toward 1 or cut ζ (γ↓). Over-light blades hand the binding margin to flap sym 1.2/rev and to harmonic_jitter.
**Caveats:** the model treats ρ×0.6–0.85 PA6-CF as homogeneous (hollow / low-infill core; skin EI not modelled beyond E×). Chord ×1.3 changes hover power and rpm (−3 %); efficiency is not checked. Not yet run: tol.py random builds, the CQ-corner damped gate, and full Goodman.
