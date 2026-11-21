# CQ_TOL=datumed (thk +-5, w +-2, L +-2, hinge_r +-1.5 %) vs default +-5 %
sa.py file scores; "tol nom res" = tol.py nominal resonance margin (margin criterion).
| design | tol | cr_pl | feas default | viol default | feas minimal (viol) | mech | res | tol nom res |
|---|---|---|---|---|---|---|---|---|
| a v8 | +-5% | 0.0715 | yes | - | yes | 0.036 | 0.177 | 0.131 |
| a v8 | datumed | 0.0715 | yes | - | yes | 0.057 | 0.177 | 0.131 |
| b v10m | +-5% | 0.1240 | no | cqc_mech_margin 0.549 | yes | 0.0135 | 0.200 | 0.080 |
| b v10m | datumed | 0.1240 | yes | - | yes | 0.031 | 0.200 | 0.080 |
| c v10m t=0.20mm resolved | +-5% | 0.1230 | no | cqc_gain 0.029, mech_margin 0.785 | no (gain, zeta 0.415) | 0.0064 | 0.759 | 0.078 |
| c v10m t=0.20mm resolved | datumed | 0.1230 | no | cqc_mech_margin 0.051 | no (zeta 0.405) | 0.028 | 0.759 | 0.078 |
Note: tol.py nominal flags a resonance viol for b/c (0.465/0.479) that sa.py cal gates do not.
Result: datumed tolerance makes v10m (b) feasible under default gates; saved as designs/10in_tt_strap_steelx_v11d.json (conditional on datumed build spec + CQ_TOL=datumed, pending Fred). (c) stays infeasible (zeta ~0.40, mech margin).
