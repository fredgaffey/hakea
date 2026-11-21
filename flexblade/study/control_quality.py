"""Control-quality re-scoring: the cyclic MECHANISM mode (largest participation in the 1/rev pulse
response) is exempt from the hard 0.15 1P margin and judged instead by the spread of the complex
cyclic gain G = hub moment / torque pulse over throttle {0.7,1,1.4} x J_hub {0.92,2.5,5.49}e-6 x
(nominal + robust.CQ_PERTURB, 4 fixed +-5 % samples): phase within +-15 deg of the per-throttle
median, gain within +-25 % of the median at each (throttle, J). All other modes keep the hard margin.

Stages: score | tol <name> | probe.   PYTHONPATH=flexblade python flexblade/study/control_quality.py score
Raw -> study/results/control_quality.json
"""
from __future__ import annotations
import functools, json, math, random, sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust                                # noqa: E402
from flexblade.optimize import set_design                         # noqa: E402
from flexblade.schema import BalanceMass, Design                  # noqa: E402
import run_study as rs                                            # noqa: E402

RES = HERE / "results" / "control_quality.json"
DES = HERE / "designs"
WORKERS = 4
JS = robust.J_HUB_RANGE
J_TOL = 2.5e-6
CRIT = "control_quality"


def load_design(name):
    return Design.model_validate_json((DES / f"{name}.json").read_text())


def ballast_v6(m_g=0.4, r=0.85, hinge_r=15.5e-3):
    """v6 + tip ballast at the quarter-chord (iteration-17 scan best point)."""
    d = set_design(load_design("7in_tt_strap_steelx_v6"), {"flexure.hinge_radius": hinge_r})
    eaf = d.blade.elastic_axis_chord_frac
    c = min(d.blade.stations, key=lambda s: abs(s.r_frac - r)).chord
    bm = [BalanceMass(mass=m_g * 1e-3, r_frac=r, chord_offset=(eaf - 0.25) * c)]
    return d.model_copy(update=dict(blade=d.blade.model_copy(update=dict(balance_masses=bm))))


def designs():
    return {"7in_v6": load_design("7in_tt_strap_steelx_v6"), "7in_v6+ballast": ballast_v6(),
            "10in_v4": load_design("10in_tt_strap_steelx_v4"), "7in_v5": load_design("7in_tt_strap_steelx_v5")}


def _eval_point(dv, criterion=CRIT):
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    return rs.metrics(dv, r, criterion=criterion)


def robust_eval(ds, js=JS, perturb=robust.CQ_PERTURB, criterion=CRIT):
    vs = [robust.variants(d, robust.THRUST_FACTORS, list(perturb), js) for d in ds]
    flat = [dv for v in vs for _, dv in v]
    with ProcessPoolExecutor(WORKERS) as ex:
        ms = list(ex.map(functools.partial(_eval_point, criterion=criterion), flat, chunksize=1))
    out, k = [], 0
    for v in vs:
        out.append(robust.combine(ms[k:k + len(v)], [t for t, _ in v]))
        k += len(v)
    return out


def summary(c):
    cq = c.get("cq") or {}
    return dict(cr_pl=c.get("cr_pl"), feasible_pl=c.get("feasible_pl"), viol_hard=c.get("viol_hard"),
                res_margin_other=c.get("res_margin"), res_which_other=c.get("res_which"),
                phase_spread_deg=cq.get("phase_spread_deg"), gain_spread=cq.get("gain_spread"),
                per_throttle=cq.get("per_throttle"),
                mech_modes=sorted({str(p.get("mech_mode")).split(" ")[0] + " " + " ".join(
                    str(p.get("mech_mode")).split(" ")[1:-1]) for p in c.get("points", [])}),
                points=[{k: p.get(k) for k in ("tag", "cr_pl", "cyc_gain", "cyc_phase_deg", "mech_mode",
                                                "res_margin", "feasible_pl")} for p in c.get("points", [])])


def res_save(k, v):
    r = json.loads(RES.read_text()) if RES.exists() else {}
    r[k] = v
    RES.write_text(json.dumps(r, indent=1, default=float))


def _line(n, s):
    return (f"{n}: cr_pl {s['cr_pl']:.4f} feas {s['feasible_pl']} phase±{s['phase_spread_deg']:.1f}° "
            f"gain±{100 * s['gain_spread']:.1f}% other-margin {s['res_margin_other']:.3f} "
            f"({s['res_which_other']}) hard {({k: round(v, 3) for k, v in (s['viol_hard'] or {}).items()})} "
            f"mech {s['mech_modes']}")


def score():
    ds = designs()
    cs = robust_eval(list(ds.values()))
    out = {n: summary(c) for n, c in zip(ds, cs)}
    res_save("score", out)
    for n, s in out.items():
        print(_line(n, s), flush=True)
        for T, v in s["per_throttle"].items():
            print(f"   {T}: phase med {v['phase_med']:.1f} ±{v['phase_spread']:.1f}  gain ±{100 * v['gain_spread']:.1f}%")


def tol(name):
    """nominal + 8 random (verify_v5.perturbed, seed 11) at J 2.5e-6, each with the CQ tolerance set."""
    import verify_v5 as vv
    d0 = designs()[name]
    rnd = random.Random(11)
    T, W, L, s = "flexure.thickness", "flexure.width", "flexure.length", "flexure.strap_strand_diameter"
    cases = {"nominal": []}
    for i in range(8):
        u = lambda a: 1 + rnd.uniform(-a, a)    # noqa: E731
        cases[f"random{i:02d}"] = [("mul", T, u(.05)), ("mul", W, u(.05)), ("mul", L, u(.05)),
                                   ("mul", "flexure.hinge_radius", u(.05)), ("rho", "", round(u(.05), 4)),
                                   ("mul", s, math.sqrt(u(.05))), ("add", "flexure.skew_deg", rnd.uniform(-1, 1)),
                                   ("add", "blade.elastic_axis_chord_frac", rnd.uniform(-.01, .01))]
    cs = robust_eval([vv.perturbed(d0, sp) for sp in cases.values()], js=(J_TOL,))
    out = {n: summary(c) for n, c in zip(cases, cs)}
    npass = sum(bool(o["feasible_pl"]) for o in out.values())
    res_save(f"tol:{name}", dict(J_hub=J_TOL, n_pass=npass, n=len(out), cases=out))
    for n, o in out.items():
        print(_line(n, o), flush=True)
    print(f"tolerance {npass}/{len(out)}")


def probe():
    d = designs()["7in_v6"]
    dv = robust.variants(d, (0.7,), None, (5.49e-6,))[0][1]
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    im = rs.mechanism_mode(r)
    rpm = r["summary"]["rpm"]
    for i, m in enumerate(r["modes"][:12]):
        print(i, m["label"], round(m["freq_hz"] * 60 / rpm, 3), round(m["damping_ratio"], 4), "<-" if i == im else "")
    m = rs.metrics(dv, r, criterion=CRIT)
    print({k: m.get(k) for k in ("mech_mode", "res_margin", "res_which", "res_margin_all", "res_which_all",
                                 "cyc_gain", "cyc_phase_deg", "cr_pl")})


if __name__ == "__main__":
    st = sys.argv[1]
    {"score": score, "probe": probe}.get(st, lambda: tol(sys.argv[2]))()
