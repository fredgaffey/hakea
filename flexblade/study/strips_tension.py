"""Steelx strips in tension? Single T1.0 evaluations of v8 variants (no search).
Per variant: strip axial N (mean over each strip chain, per blade), strip_ltb util, worst fatigue util,
strap tension util, mech mode /rev, steady hub (control) moment, control_ratio.
PYTHONPATH=flexblade python flexblade/study/strips_tension.py [variant ...] -> results/strips_tension.json
"""
from __future__ import annotations
import json, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import pulse_limited as pl
import run_study as rs
from flexblade import api, robust, rotor
from flexblade.beam import assemble_internal, element_end_forces

VARIANTS = {
    "base": {},
    "strap_d2.0": {"strap_strand_diameter": 0.0020},          # softer strap (release still free)
    "strap_L13": {"strap_length": 0.013},                       # longer strap / moved anchor
    "rel_k3e6": {"axial_release_stiffness": 3e6},               # zig-zag release spring, N/m
    "rel_k1e7": {"axial_release_stiffness": 1e7},
    "rel_k3e7": {"axial_release_stiffness": 3e7},
    "bonded": {"axial_release": False},                          # no release: strips share CF
    "base_T0.7": {"_tf": 0.7},
    "base_T1.4": {"_tf": 1.4},
    "cross100": {"cross_angle_deg": 100.0},                     # strip crossing geometry
    "cross110": {"cross_angle_deg": 110.0},
    "skew60": {"skew_deg": 60.0},
    "skew64": {"skew_deg": 64.0},
    "no_preset": {"strip_preset_deg": []}  ,                     # slot presets removed
}


def design(upd):
    upd = dict(upd)
    tf = upd.pop("_tf", 1.0)
    d0 = pl.load_design("10in_tt_strap_steelx_v8")
    d0 = d0.model_copy(update=dict(flexure=d0.flexure.model_copy(update=upd)))
    return robust.variants(d0, thrust_factors=(tf,))[0][1]


def strip_forces(d):
    if d.flexure.pre_lag_deg == "auto":
        d = rotor.resolve_pre_lag(d)
    model = rotor.build_model(d)
    ss = rotor.static_trim(model, rotor.make_aero(model))
    fe, st = model.fe, ss.state
    cr = assemble_internal(fe, st, False).cr
    N = {}
    for i, e in enumerate(fe.elements):
        if e.kind != "flexure":
            continue
        tag = e.tag.rsplit(".", 1)[0] if e.tag.count(".") > 1 else e.tag
        a, b = element_end_forces(e, cr[i], st)
        N.setdefault(tag, []).extend([a[0], b[0]])
    return {k: (round(float(min(v)), 1), round(float(max(v)), 1)) for k, v in N.items()}, float(ss.rpm)


def run(name):
    t = time.time()
    d = design(VARIANTS[name])
    N, rpm = strip_forces(d)
    r = api.evaluate(d, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    m = rs.metrics(d, r, criterion="control_quality")
    u = m["util"]
    s = r.get("summary") or {}
    fat = {k: round(v, 3) for k, v in u.items() if "fatigue" in k and not k.startswith("member:")}
    out = dict(variant=name, N=N, rpm=rpm, strip_ltb=u.get("strip_ltb"),
               fatigue=max(fat.values(), default=None), fatigue_by=fat,
               strap_tension=u.get("strap_tension"), mech_per_rev=m.get("mech_per_rev"),
               control_ratio=m.get("control_ratio"), hub_moment_mean=s.get("hub_moment_mean"),
               max_util=max((v for k, v in u.items() if not k.startswith("member:")), default=None),
               max_util_by=max((k for k in u if not k.startswith("member:")), key=u.get),
               sec=round(time.time() - t))
    print(json.dumps(out, default=float), flush=True)
    return out


if __name__ == "__main__":
    names = sys.argv[1:] or list(VARIANTS)
    res = [run(n) for n in names]
    p = HERE / "results" / "strips_tension.json"
    old = json.loads(p.read_text()) if p.exists() else {}
    old.update({r["variant"]: r for r in res})
    p.write_text(json.dumps(old, indent=1, default=float))
