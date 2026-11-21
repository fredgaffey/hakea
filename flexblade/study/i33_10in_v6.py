"""I33: 10" v6 local search from v5 under control_quality_cal (rpm_schedule="continuous", 5 nodes)
+ every other hard check (other modes' 0.15 1P margin, stability, ground_resonance, fatigue).

    PYTHONPATH=flexblade python flexblade/study/i33_10in_v6.py scan '<json {tag: x}>'
    ... i33_10in_v6.py table            # table of everything scanned (sorted by cr_pl)
    ... i33_10in_v6.py save '<json x>'  # -> designs/10in_tt_strap_steelx_v6.json
    ... i33_10in_v6.py tol              # nominal + 8 random (verify_v5.perturbed, seed 11), CQ-cal continuous
x = {design path: value} applied to v5 (set_design); "base": "v4" starts from v4 instead.
FLEXBLADE_GROUND_RES=1 is set here (ground_resonance hard check, default arm range).
-> study/results/i33_10in_v6.json
"""
from __future__ import annotations
import json, math, os, random, sys
from pathlib import Path

os.environ.setdefault("FLEXBLADE_GROUND_RES", "1")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from flexblade.optimize import set_design                         # noqa: E402
import control_quality as cq                                      # noqa: E402

RES = HERE / "results" / "i33_10in_v6.json"
OUT = HERE / "designs" / "10in_tt_strap_steelx_v6.json"
LIM = dict(phase=15.0, gain=0.25, mech=0.03)


def load():
    return json.loads(RES.read_text()) if RES.exists() else {}


def save_res(k, v):
    r = load(); r[k] = v; RES.write_text(json.dumps(r, indent=1, default=float))


def build(x):
    x = dict(x)
    base = x.pop("base", "v5")
    d = cq.load_design(f"10in_tt_strap_steelx_{base}")
    return set_design(d, x) if x else d


def score(d):
    s = cq.cal_summary(cq.cal_eval(d, cq.J10, "continuous", 5))
    c = s["control_quality_cal"]
    ph, g, mm = c["phase_res_deg"], c["gain_res"], c["mech_margin_min"]
    ratio = min(LIM["phase"] / ph if ph else 9, LIM["gain"] / g if g else 9,
                (mm / LIM["mech"]) if (mm is not None and not c.get("mech_side_flip")) else 0)
    return dict(cr_pl=c["cr_pl"], feasible=c["feasible_pl"], viol=c["viol_hard"], phase=ph, gain=g,
                auth=c["authority_min"], mech=mm, flip=c.get("mech_side_flip"), cq_ratio=ratio,
                other_margin=c["res_margin"], other_which=c["res_which"], cr_pl_margin_crit=s["margin"]["cr_pl"])


def _fmt(r):
    return json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})


def scan(xs):
    for tag, x in xs.items():
        r = dict(x=x, **score(build(x)))
        save_res(tag, r)
        print(tag, _fmt(r), flush=True)


def table():
    r = {k: v for k, v in load().items() if not k.startswith("tol")}
    print("| tag | x | cr_pl | feas | phase° | gain % | mech min | auth | CQ ratio | viol |\n"
          "|---|---|---|---|---|---|---|---|---|---|")
    for k, v in sorted(r.items(), key=lambda kv: -(kv[1]["cr_pl"] or 0)):
        print(f"| {k} | {json.dumps(v['x'])} | {v['cr_pl']:.4f} | {v['feasible']} | {v['phase']:.1f} | "
              f"{100 * v['gain']:.1f} | {v['mech']:.3f}{' FLIP' if v['flip'] else ''} | {v['auth']:.2f} | "
              f"{v['cq_ratio']:.2f} | {v['viol']} |")


def save(x):
    d = build(x)
    d = d.model_copy(update=dict(name="10in_tt_strap_steelx_v6",
                                 notes=d.notes + f"; v6 (I33, control_quality_cal continuous, i33_10in_v6.py): "
                                                 f"{json.dumps(x)}"))
    OUT.write_text(json.dumps(json.loads(d.model_dump_json()), indent=1) + "\n")


def tol():
    import verify_v5 as vv
    d0 = cq.load_design("10in_tt_strap_steelx_v6")
    rnd = random.Random(11)
    T, W, L, s = "flexure.thickness", "flexure.width", "flexure.length", "flexure.strap_strand_diameter"
    cases = {"nominal": []}
    for i in range(8):
        u = lambda a: 1 + rnd.uniform(-a, a)    # noqa: E731
        cases[f"random{i:02d}"] = [("mul", T, u(.05)), ("mul", W, u(.05)), ("mul", L, u(.05)),
                                   ("mul", "flexure.hinge_radius", u(.05)), ("rho", "", round(u(.05), 4)),
                                   ("mul", s, math.sqrt(u(.05))), ("add", "flexure.skew_deg", rnd.uniform(-1, 1)),
                                   ("add", "blade.elastic_axis_chord_frac", rnd.uniform(-.01, .01))]
    out = {}
    for n, sp in cases.items():
        out[n] = score(vv.perturbed(d0, sp))
        print(n, _fmt(out[n]), flush=True)
    npass = sum(bool(o["feasible"]) for o in out.values())
    save_res("tol:v6", dict(n_pass=npass, n=len(out), cases=out))
    print(f"tolerance {npass}/{len(out)}")


if __name__ == "__main__":
    st = sys.argv[1]
    if st == "scan":
        scan(json.loads(sys.argv[2]))
    elif st == "table":
        table()
    elif st == "save":
        save(json.loads(sys.argv[2]))
    elif st == "tol":
        tol()
