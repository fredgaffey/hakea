import sys, os, json
os.environ.setdefault("FLEXBLADE_GROUND_RES", "1")
sys.path.insert(0, "study")
import run_study as rs
from flexblade import robust, api, rotor as R
from flexblade.schema import Design
from concurrent.futures import ProcessPoolExecutor
rec = {}; cnt = {}
orig = R._fibre_amp
def w(sec, *a, **k):
    r = orig(sec, *a, **k)
    e = sys._getframe(1).f_locals.get("e")
    if e is not None and e.kind == "flexure":
        cnt[e.tag] = cnt.get(e.tag, 0) + 1; end = (cnt[e.tag]-1) % 2
        d = rec.setdefault(e.tag, [0, 0]); d[end] = max(d[end], r[0])
    return r
R._fibre_amp = w
JS = (0.34e-6, 0.91e-6, 2.0e-6)
def ev(a):
    dv, tag, wantend = a
    rec.clear(); cnt.clear()
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    m = rs.metrics(dv, r, criterion="margin")
    out = dict(cr=m.get("cr_pl"), vh={k: round(v, 3) for k, v in (m.get("viol_hard") or {}).items()})
    fs = [c for c in r["checks"] if c["name"] == "fatigue_strain"]
    out["fs"] = max(c["utilisation"] for c in fs) if fs else None
    if wantend:
        kf = R.strip_clamp_factor(dv.flexure)[0]
        strips = {}
        for t, v in rec.items():
            strips.setdefault(t.rsplit(".", 1)[0] if t.count(".") > 1 else t, []).append(v)
        out["ends"] = sorted(((max(l[0][0], l[-1][1])*kf, s, l[0][0]*kf, l[-1][1]*kf) for s, l in strips.items()), reverse=True)[:1]
    return out
def build(name, **kw):
    d = Design.model_validate_json(open("study/designs/5in_v8s_2307_m245_301.json").read())
    upd = {}
    if "t" in kw: upd["thickness"] = kw["t"]
    if "w" in kw: upd["width"] = kw["w"]
    if "n" in kw: upd["strip_laminae"] = kw["n"]
    if "ang" in kw: upd["cross_angle_deg"] = kw["ang"]
    d = d.model_copy(update=dict(name=name, flexure=d.flexure.model_copy(update=upd)))
    return rs.resolve_candidate(d, 6)
V = dict(lead={}, a=dict(t=4e-5, n=12), b=dict(w=3e-3, n=12), c=dict(ang=95.0), d=dict(ang=115.0))
if __name__ == "__main__":
    for name, kw in V.items():
        d = build(name, **kw)
        jobs = []
        for J in JS:
            for tag, dv in robust.variants(d, (0.7, 1.0, 1.25), None, [J], None):
                jobs.append((tag, J, dv))
        with ProcessPoolExecutor(2) as ex:
            res = list(ex.map(ev, [(dv, t, (t.startswith("T0.7") and J == 0.91e-6)) for t, J, dv in jobs], chunksize=1))
        line = {}
        for (t, J, _), r in zip(jobs, res):
            tf = t.split(":")[0]
            line.setdefault(tf, []).append(r["cr"])
        vh = {}
        for r in res: vh.update(r["vh"])
        e = [r for (t, J, _), r in zip(jobs, res) if "ends" in r][0]
        fs07 = max(r["fs"] or 0 for (t, J, _), r in zip(jobs, res) if t.startswith("T0.7"))
        print(name, "cr_pl", {k: round(min(v), 3) for k, v in line.items()}, "hard", vh, "fs07", round(fs07, 3),
              "end(raw*k hub/blade)", [(s, round(h, 3), round(b, 3)) for _, s, h, b in e["ends"]], flush=True)
        json.dump(json.loads(d.model_dump_json()), open(f"scratch/{name}.json", "w"))
