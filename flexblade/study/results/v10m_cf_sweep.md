# v10m CF-stiffness lever sweep (t=0.20 mm, L x1; presets re-solved iters=6; script v10m_cf_sweep.py)
Mech = pitch per rev at T0.7/1/1.4; Goodman = max fatigue_stress util; jit = harmonic_jitter viol (check:) ; res_margin is the combined worst.
point | cr_pl | res | mech/rev | Goodman | viol
base | .129 | .078 | .75/.70/.66 | .32 | resonance
hinge r x0.75 | .227 | .020 | .67/.62/.58 | 1.08 | flexure_inside_hub, res
hinge r x0.9 | .159 | .049 | .72/.67/.63 | .46 | inside_hub, res
hinge r x1.1 | .097 | .106 | .78/.73/.69 | .25 | res
hinge r x1.25 | .068 | .142 | .83/.77/.73 | .26 | jitter .14
strap pin d x0.75..1.25 | identical to base to 6 digits (no effect on mode)
bal 1g c+.01 | .146 | .075 | .74/.70/.66 | .32 | res
bal 2g c+.01 | .144 | .065 | .76/.71/.67 | .43 | res
bal 1g c-.01 | .148 | .064 | .75/.70/.67 | .35 | res ;  bal 2g c-.01 | .143 | .039 | .76/.71/.68 | .55 | res
precone 1.3 | .115 | .089 | .75/.70/.66 | .31 | res
Combos: h1.25+pc1.3 cr .063 res .151 jit .33 | h1.25+bal1g c+.01 cr .080 res .131 res-viol .12 | h1.4 cr .052 res .173 jit .28 | h1.4+pc+bal cr .060 res .154 jit .03 | h1.25+pc+bal cr .078 res .128 | h1.1+pc+bal cr .111 res .095
Finding: hinge radius is the only real lever (res ~ +0.03 per +10 %, but cr falls ~ -0.03 per +10 %, plus harmonic_jitter from x1.25). Pin diameter is inert; chordwise balance masses lower margin (esp. negative offset) while raising cr; precone gives only +0.01.
No point meets res>=0.15 & jitter-clean & cr>0.0715 (closest: h1.4+pc+bal res .154 cr .060 jit .03). SA not run, v11 not saved.
