import sys, os, json
import numpy as np
from flexblade import rotor as R, robust
from flexblade.schema import Design
rec = {}; cnt = {}
orig = R._fibre_amp
def w(sec, *a, **k):
    r = orig(sec, *a, **k)
    f = sys._getframe(1).f_locals
    e = f.get("e"); rec.setdefault("_n", [0,0,0])[0] += 1; rec["_n"][1] = e.kind if e is not None else str(list(f)[:5])
    if e is not None and e.kind == "flexure":
        cnt[e.tag] = cnt.get(e.tag, 0) + 1; end = (cnt[e.tag] - 1) % 2
        d = rec.setdefault(e.tag, [0, 0, e.L0]); 
        d[end] = max(d[end], r[0])
    return r
R._fibre_amp = w
D = Design.model_validate_json(open(f"study/designs/{sys.argv[1]}.json").read())
for tag, d in robust.variants(D, thrust_factors=(0.7, 1.0, 1.25)):
    rec.clear(); cnt.clear()
    res = R._evaluate(d, ["checks"]) if False else R._evaluate(d)
    fs = [c for c in res.checks if c.name == "fatigue_strain"]
    top = max(fs, key=lambda c: c.utilisation) if fs else None
    # group elements by strip prefix
    dbg = rec.pop("_n", None); strips = {}
    for t, v in rec.items():
        strips.setdefault(t.rsplit(".", 1)[0] if t.count(".") > 1 else t, []).append((t, v))
    out = []
    for s, lst in strips.items():
        hub = lst[0][1][0]; bl = lst[-1][1][1]
        out.append((max(hub, bl), s, hub, bl))
    out.sort(reverse=True)
    kfac = R.strip_clamp_factor(d.flexure)[0]
    print(tag, "strain_check", top and (top.member, round(top.value, 5), round(top.utilisation, 3)), "kfac", round(kfac, 3),
          "top strip hub/blade raw ue", [(s, round(h*kfac, 3), round(b*kfac, 3)) for _, s, h, b in out[:2]], flush=True)
