"""Verification of study/designs/10in_tt_strap_steelx_v4.json. Stages: J | pl | ccx | floq | fit.
Run from flexblade/ with PYTHONPATH=$PWD. Raw -> study/results/verify_<name>.json
usage: verify_10in_v4.py <stage> [design name]"""
import json, sys, math
from pathlib import Path
import numpy as np
HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import pulse_limited as pl
from flexblade import api, robust
from flexblade.schema import Airframe, Mount
NAME = sys.argv[2] if len(sys.argv) > 2 else "10in_tt_strap_steelx_v4"
RES = HERE / "results" / f"verify_{NAME}.json"
D = pl.load_design(NAME)


def load():
    return json.loads(RES.read_text()) if RES.exists() else {}


def save(k, v):
    r = load(); r[k] = v; RES.write_text(json.dumps(r, indent=1, default=float))


def J_item():
    from flexblade import fittings as FT, geometry as G
    from flexblade.materials import get_material
    import trimesh
    m = trimesh.load(HERE / "build/10in_v4/hub_fitting_assembly.stl")
    m.density = get_material(G.HUB_MATERIAL).rho * 1e-9
    Jh = m.moment_inertia[2, 2] * 1e-6
    mh = m.mass * 1e3
    strips, _, _ = FT.strip_members(D)
    prm = FT.strip_params(D)
    rho = get_material(strips[0].layup.material).rho * 1e-9
    mstrip = prm.land * prm.w * prm.t * rho * 1e3       # g per hub-side land
    xs = [float(np.array(s.p0)[0] * 1e3) for s in strips]
    Jl = 2 * sum(mstrip * 1e-3 * ((abs(x) - prm.land / 2) * 1e-3) ** 2 for x in xs)   # x2 blades, land inboard of jaw exit
    Jscr = 4 * 0.5e-3 * (8e-3) ** 2
    Jmisc = 0.2e-3 * (8e-3) ** 2
    tot = Jh + Jl + Jscr + Jmisc
    out = dict(hub_g=mh, J_hub_stl=Jh, strip_land_g_each=mstrip, n_strips=len(strips), p0_x_mm=xs,
               J_lands=Jl, J_screws=Jscr, J_misc=Jmisc, J_total=tot)
    save("J", out)
    print(json.dumps(out, indent=1))


def pl_stage():
    Jt = load()["J"]["J_total"]
    js = (float(f"{Jt:.4g}"), 2.5e-6, 5.49e-6)
    c = pl.robust_eval([D], js=js)[0]
    s = pl.summary(c)
    s["js"] = js
    save("pl", dict(summary=s, keys=sorted(c.keys()), hard=c.get("viol_hard")))
    print("cr_pl", s["cr_pl"], s["feasible_pl"], s["viol_hard"], s["res_which"], s["res_margin"])
    for p in s["points"]:
        print(p)


def _floq(a):
    J, tag, d, zaf = a
    out = []
    for fa in ((60, 100, 150, 200, 260, 320) if zaf != "mount" else (190, 225, 260)):
        if zaf == "mount":
            af = Airframe(f_hz=fa, f_y_hz=1.15 * fa, zeta=0.02,
                          mount=Mount(G_mpa=6.5, eta=0.15, r_in_mm=4, r_out_mm=9, length_mm=10, infill=0.4))
        else:
            af = Airframe(f_hz=fa, f_y_hz=1.15 * fa, zeta=zaf)
        dd = d.model_copy(update=dict(analysis=d.analysis.model_copy(update=dict(stability="floquet")),
                                      op=d.op.model_copy(update=dict(airframe=af))))
        r = api.evaluate(dd, ["trim", "modes"]).model_dump(mode="json")["summary"]
        out.append((fa, r.get("air_resonance_zeta"), r.get("zeta_min")))
    return dict(J=J, tag=tag, zaf=zaf, sweep=out)


def floq_stage():
    from concurrent.futures import ProcessPoolExecutor
    Jt = load()["J"]["J_total"]
    jobs = []
    for J in (float(f"{Jt:.4g}"), 2.5e-6, 5.49e-6):
        d0 = D.model_copy(update=dict(motor=D.motor.model_copy(update=dict(J_hub=float(J)))))
        for tag, d in robust.variants(d0):
            if tag not in ("T0.7", "T1", "T1.4"):
                continue
            for zaf in (0.02, 0.05, "mount"):
                jobs.append((J, tag, d, zaf))
    with ProcessPoolExecutor(4) as ex:
        res = list(ex.map(_floq, jobs))
    save("floq", res)
    for r in res:
        print(r)


def ccx_stage():
    import tempfile
    from flexblade import ccx
    r = ccx.compare_strip_flexure(D, Path(tempfile.mkdtemp(prefix="v10ccx_", dir="/tmp")))
    cv = lambda v: v.tolist() if hasattr(v, "tolist") else float(v)
    save("ccx", dict(ratio=r["ratio"], ccx={k: cv(v) for k, v in r["ccx"].items()},
                     beam={k: cv(v) for k, v in r["beam"].items()},
                     axis_err_deg=r["axis_err_deg"], axis_design_err_deg=r["axis_design_err_deg"]))
    print(r["ratio"], r["axis_err_deg"])


def fit_stage():
    from flexblade import fittings as FT
    prm = FT.strip_params(D)
    fits = FT.build_strip_fittings(D)
    sw = {}
    for lag, pit in ((5, 8), (8, 12), (12, 16)):
        sw[f"{lag}/{pit}"] = FT.fitting_clearance(D, lag, pit, fits=fits)["min_gap_mm"]
    save("fit", dict(stack_t=prm.t, slot_t=prm.slot_t, slot_w=prm.slot_w, sweep=sw))
    print(prm.t, prm.slot_t, sw)


if __name__ == "__main__":
    {"J": J_item, "pl": pl_stage, "floq": floq_stage, "ccx": ccx_stage, "fit": fit_stage}[sys.argv[1]]()
