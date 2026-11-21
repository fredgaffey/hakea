"""Pulse-limited re-scoring (cr_pl) with J_hub as a robust dimension.

cr_pl = cr at the pulse that binds the most-utilised pulse-scaling check (run_study.add_pulse_limited),
per-throttle pulse schedule, min over throttle {0.7,1.0,1.4} x J_hub {0.92,2.5,5.49}e-6; the
non-scaling checks are hard at every point.
Stages: score | search [n] | tol | save.   PYTHONPATH=flexblade python flexblade/study/pulse_limited.py score
Raw -> study/results/pulse_limited.json
"""
from __future__ import annotations
import json, math, random, sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust                                # noqa: E402
from flexblade.optimize import get_path, set_design               # noqa: E402
from flexblade.schema import Design                               # noqa: E402
import run_study as rs                                            # noqa: E402

RES = HERE / "results" / "pulse_limited.json"
DES = HERE / "designs"
WORKERS = 4
JS = robust.J_HUB_RANGE
J_TOL = 2.5e-6


def load_design(name):
    return Design.model_validate_json((DES / f"{name}.json").read_text())


def v6_candidate():
    """BUILD.md 7in_v5 'best retune' at J 0.92e-6 (not released as v6)."""
    d = set_design(load_design("7in_tt_strap_steelx_v5"),
                   {"flexure.width": 5.84e-3, "flexure.length": 12.5e-3, "flexure.hinge_radius": 14.3e-3,
                    "flexure.cross_angle_deg": 88.0})
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(strip_clamp="compliant"))))


def res_load():
    return json.loads(RES.read_text()) if RES.exists() else {}


def res_save(k, v):
    r = res_load(); r[k] = v; RES.write_text(json.dumps(r, indent=1, default=float))


def _eval_point(dv):
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    return rs.metrics(dv, r)


def robust_eval(designs, js=JS, ex=None):
    """-> list of combined metrics (min over throttle x J)."""
    vs = [robust.variants(d, robust.THRUST_FACTORS, None, js) for d in designs]
    flat = [dv for v in vs for _, dv in v]
    own = ex or ProcessPoolExecutor(WORKERS)
    try:
        ms = list(own.map(_eval_point, flat, chunksize=1))
    finally:
        if ex is None:
            own.shutdown()
    out, k = [], 0
    for v in vs:
        out.append(robust.combine(ms[k:k + len(v)], [t for t, _ in v]))
        k += len(v)
    return out


def summary(c):
    return dict(cr_pl=c.get("cr_pl"), feasible_pl=c.get("feasible_pl"), viol_hard=c.get("viol_hard"),
                cr_min=c.get("control_ratio"), res_margin=c.get("res_margin"), res_which=c.get("res_which"),
                points=[{k: p.get(k) for k in ("tag", "control_ratio", "cr_pl", "pulse_scale", "pulse_frac_cap",
                                                "pulse_binding", "res_margin", "feasible_pl")}
                        for p in c.get("points", [])])


def fitness(c):
    return c["cr_pl"] if c.get("feasible_pl") else -sum(c.get("viol_hard", {}).values())


def score():
    ds = {n: load_design(n) for n in ("7in_tt_strap_steelx_v5", "7in_tt_strap_steelx_v4m",
                                       "10in_tt_strap_steelx_v3")}
    ds["7in_v6_candidate(BUILD.md)"] = v6_candidate()
    cs = robust_eval(list(ds.values()))
    out = {n: summary(c) for n, c in zip(ds, cs)}
    res_save("score", out)
    for n, s in out.items():
        print(n, round(s["cr_pl"], 4), s["feasible_pl"], s["viol_hard"], s["res_which"], flush=True)


VARS = {"flexure.width": 0.06, "flexure.length": 0.06, "flexure.hinge_radius": 0.04,
        "flexure.cross_angle_deg": 0.05, "flexure.skew_deg": 0.03}


def search(budget=48):
    """Coordinate pattern search around v5 (relative steps, halved on failure)."""
    x = load_design("7in_tt_strap_steelx_v5")
    step = dict(VARS)
    with ProcessPoolExecutor(WORKERS) as ex:
        best = robust_eval([x], ex=ex)[0]
        n, hist = 1, [dict(x={}, fit=fitness(best), cr_pl=best["cr_pl"], feas=best["feasible_pl"])]
        print("start", fitness(best), best["viol_hard"], flush=True)
        while n + 2 * len(VARS) <= budget + 1 and max(step.values()) > 0.005:
            cands, moves = [], []
            for p, h in step.items():
                for sg in (1, -1):
                    cands.append(set_design(x, {p: float(get_path(x, p)) * (1 + sg * h)}))
                    moves.append((p, sg))
            cs = robust_eval(cands, ex=ex)
            n += len(cands)
            fs = [fitness(c) for c in cs]
            for (p, sg), c, f in zip(moves, cs, fs):
                hist.append(dict(x={p: float(get_path(x, p)) * (1 + sg * step[p])}, fit=f, cr_pl=c["cr_pl"],
                                 feas=c["feasible_pl"]))
            i = max(range(len(fs)), key=fs.__getitem__)
            if fs[i] > fitness(best):
                x, best = cands[i], cs[i]
                print(f"n={n} accept {moves[i]} fit {fs[i]:.4f}", flush=True)
            else:
                step = {k: v / 2 for k, v in step.items()}
                print(f"n={n} no gain; steps halved", flush=True)
    vals = {p: float(get_path(x, p)) for p in list(VARS) + ["flexure.thickness"]}
    res_save("search", dict(n_evals=n, best=summary(best), x=vals, hist=hist))
    (HERE / "results" / "pulse_limited_best.json").write_text(x.model_dump_json(indent=1))
    print("best", vals, summary(best)["cr_pl"], best["feasible_pl"], best["viol_hard"])


def tol(path=None):
    import verify_v5 as vv
    d0 = Design.model_validate_json(Path(path or HERE / "results" / "pulse_limited_best.json").read_text())
    rnd = random.Random(11)
    T, W, L, s = "flexure.thickness", "flexure.width", "flexure.length", "flexure.strap_strand_diameter"
    cases = {"nominal": []}
    for i in range(8):
        u = lambda a: 1 + rnd.uniform(-a, a)
        cases[f"random{i:02d}"] = [("mul", T, u(.05)), ("mul", W, u(.05)), ("mul", L, u(.05)),
                                   ("mul", "flexure.hinge_radius", u(.05)), ("rho", "", round(u(.05), 4)),
                                   ("mul", s, math.sqrt(u(.05))), ("add", "flexure.skew_deg", rnd.uniform(-1, 1)),
                                   ("add", "blade.elastic_axis_chord_frac", rnd.uniform(-.01, .01))]
    cs = robust_eval([vv.perturbed(d0, sp) for sp in cases.values()], js=(J_TOL,))
    out = {n: summary(c) for n, c in zip(cases, cs)}
    npass = sum(o["feasible_pl"] for o in out.values())
    res_save("tol", dict(J_hub=J_TOL, n_pass=npass, n=len(out), cases=out))
    for n, o in out.items():
        print(n, round(o["cr_pl"], 4), o["feasible_pl"], o["viol_hard"], flush=True)
    print(f"tolerance {npass}/{len(out)}")


if __name__ == "__main__":
    st = sys.argv[1]
    if st == "score":
        score()
    elif st == "search":
        search(int(sys.argv[2]) if len(sys.argv) > 2 else 48)
    elif st == "tol":
        tol(sys.argv[2] if len(sys.argv) > 2 else None)
