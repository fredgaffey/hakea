"""Steady flexure-root resultant (blade frame, about the hinge point) per blade vs precone.
usage: precone_my.py design.json Tfactor beta0_deg [beta0_deg ...]"""
import sys, time, numpy as np
from flexblade import rotor, robust
from flexblade.schema import Design


root_loads = rotor.flexure_root_loads


def with_precone(d, b0):
    return d.model_copy(update=dict(blade=d.blade.model_copy(update=dict(precone_deg=float(b0)))))


def run(d0, Tf, b0):
    d = [dv for t, dv in robust.variants(with_precone(d0, b0), (Tf,)) if t == f"T{Tf:g}"][0]
    if d.flexure.pre_lag_deg == "auto":
        d = rotor.resolve_pre_lag(d)
    m = rotor.build_model(d)
    ss = rotor.static_trim(m, rotor.make_aero(m))
    return root_loads(m, ss), ss


if __name__ == "__main__":
    import os
    d0 = Design.model_validate_json(open(sys.argv[1]).read())
    if os.environ.get("NOPL"):   # presets and pre-lag removed (diagnostic)
        d0 = d0.model_copy(update=dict(flexure=d0.flexure.model_copy(update=dict(
            strip_preset_deg=[[[0.0] * 6] * 3] * 2, pre_lag_deg=0.0))))
    Tf = float(sys.argv[2])
    for b0 in map(float, sys.argv[3:]):
        t = time.time()
        rl, ss = run(d0, Tf, b0)
        s = " | ".join(f"b{k+1} F {np.round(F,2)} M[mNm] {np.round(M*1e3,1)}" for k, (F, M) in enumerate(rl))
        print(f"T{Tf:g} beta0 {b0:+.2f}: {s}  rpm {ss.rpm:.0f} ({time.time()-t:.0f}s)", flush=True)
