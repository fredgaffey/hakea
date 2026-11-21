"""Control-quality re-scoring: the cyclic MECHANISM mode (largest participation in the 1/rev pulse
response) is exempt from the hard 0.15 1P margin and judged instead by the spread of the complex
cyclic gain G = hub moment / torque pulse over throttle {0.7,1,1.4} x J_hub {0.92,2.5,5.49}e-6 x
(nominal + robust.CQ_PERTURB, 4 fixed +-5 % samples): phase within +-15 deg of the per-throttle
median, gain within +-25 % of the median at each (throttle, J). All other modes keep the hard margin.

Stage `cal [names]`: margin / control_quality / control_quality_cal side by side
(-> study/results/control_quality_cal.json/.md); see robust.CQ_RUNTIME.

Stages: score | tol <name> | probe | cal.   PYTHONPATH=flexblade python flexblade/study/control_quality.py score
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
        tg, mm = [t for t, _ in v], ms[k:k + len(v)]
        c = robust.combine(mm, tg)
        # sensitivity: spread over the two MILD samples only (p2/p3: strip k about +-5 %, vs the
        # aligned stiff/soft corners p0/p1: k about +-28 %)
        sel = [i for i, t in enumerate(tg) if ":p0" not in t and ":p1" not in t]
        c["cq_mild"] = robust.control_quality([tg[i] for i in sel], [mm[i].get("cyc_gain") for i in sel],
                                              [mm[i].get("cyc_phase_deg") for i in sel])
        out.append(c)
        k += len(v)
    return out


def summary(c):
    cq = c.get("cq") or {}
    return dict(cr_pl=c.get("cr_pl"), feasible_pl=c.get("feasible_pl"), viol_hard=c.get("viol_hard"),
                res_margin_other=c.get("res_margin"), res_which_other=c.get("res_which"),
                phase_spread_deg=cq.get("phase_spread_deg"), gain_spread=cq.get("gain_spread"),
                per_throttle=cq.get("per_throttle"),
                mild_phase_spread_deg=(c.get("cq_mild") or {}).get("phase_spread_deg"),
                mild_gain_spread=(c.get("cq_mild") or {}).get("gain_spread"),
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
            f"mech {s['mech_modes']} | mild-tol phase±{s['mild_phase_spread_deg']:.1f}° "
            f"gain±{100 * s['mild_gain_spread']:.1f}%")


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


# ---------------------------------------------------------------- control_quality_cal (stage "cal")
RES_CAL = HERE / "results" / "control_quality_cal.json"
CRITS = ("margin", "control_quality", "control_quality_cal")
J10 = (2.5e-6, 3.76e-6, 5.49e-6)     # 10" J axis (itemised 3.59e-6; 0.92e-6 is not buildable)


def cal_designs():
    """name -> (Design, J axis)."""
    return {"10in_v5": (load_design("10in_tt_strap_steelx_v5"), J10),
            "7in_v6": (load_design("7in_tt_strap_steelx_v6"), JS),
            "7in_v5": (load_design("7in_tt_strap_steelx_v5"), JS),
            "7in_v6+ballast": (ballast_v6(), JS),
            "10in_v4": (load_design("10in_tt_strap_steelx_v4"), J10)}


def _eval_multi(dv, crits=CRITS):
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    return {c: rs.metrics(dv, r, criterion=c) for c in crits}


def cal_eval(d, js):
    """One pass over the control_quality_cal variant set (a superset of the margin / CQ sets);
    every criterion is combined from its own tag subset of the same (cached) evaluations."""
    v = robust.variants(d, robust.THRUST_FACTORS, list(robust.CQ_PERTURB), js, list(robust.CQ_RUNTIME))
    with ProcessPoolExecutor(WORKERS) as ex:
        ms = list(ex.map(_eval_multi, [dv for _, dv in v], chunksize=1))
    tags = [t for t, _ in v]
    sel = {"margin": lambda t: ":p" not in t and ":r" not in t, "control_quality": lambda t: ":r" not in t,
           "control_quality_cal": lambda t: True}
    return {c: robust.combine([m[c] for t, m in zip(tags, ms) if sel[c](t)], [t for t in tags if sel[c](t)])
            for c in CRITS}


def cal_summary(cs):
    def base(c):
        return dict(cr_pl=c.get("cr_pl"), feasible_pl=c.get("feasible_pl"),
                    viol_hard={k: round(v, 4) for k, v in (c.get("viol_hard") or {}).items()},
                    res_margin=c.get("res_margin"), res_which=c.get("res_which"))
    out = {k: base(c) for k, c in cs.items()}
    cq = cs["control_quality"].get("cq") or {}
    out["control_quality"].update(phase_spread_deg=cq.get("phase_spread_deg"), gain_spread=cq.get("gain_spread"))
    q = cs["control_quality_cal"].get("cqc") or {}
    out["control_quality_cal"].update(
        {k: q.get(k) for k in ("phase_res_deg", "gain_res", "groups", "authority_min", "cr_pl_ratio_min",
                               "mech_margin_min", "mech_side_flip")},
        per_node={n: {k: (round(x, 4) if isinstance(x, float) else x) for k, x in v.items() if k != "groups"}
                  for n, v in (q.get("per_node") or {}).items()},
        worst_phase_node=max(q.get("per_node") or {"-": {"phase_res": 0}},
                             key=lambda n: (q.get("per_node") or {}).get(n, {}).get("phase_res", 0)),
        worst_gain_node=max(q.get("per_node") or {"-": {"gain_res": 0}},
                            key=lambda n: (q.get("per_node") or {}).get(n, {}).get("gain_res", 0)))
    return out


def cal_md(res):
    L = ["# control_quality_cal: post-calibration residual (study/control_quality.py cal)", "",
         "Build tolerance (nominal + 4 CQ_PERTURB corners) is calibrated out per airframe (FC schedule of "
         "cyclic gain+phase per throttle at measured J). Residual = LINEAR worst-case stack of one-at-a-time "
         "runtime deviations about each calibrated (throttle, build, J) node: rpm +-5 % (thrust ~ rpm^2), "
         "temp +20 C (op.ambient_C 45: printed parts derate; steel strip E -0.6 %) / -20 C (strip E +0.6 % "
         "only, = 0.3 x creep, model has no stiffening below 25 C), creep strip k -2 %, J error +-10 %. "
         "Strip E applied as strip width (thin strip: EI/EA/GJ linear in w). Aramid strap: no T dependence "
         "in the model (skipped). Limits: phase <= 15 deg, gain <= 25 %; calibratability: |G_b| >= 0.5 |G_nom|, "
         "mechanism-mode 1P margin >= 0.03 on the nominal side, every build sample. Other modes keep the hard "
         "0.15 margin (from the nominal-build points). J axis: 7\" {0.92,2.5,5.49}e-6, 10\" {2.5,3.76,5.49}e-6.", "",
         "| design | margin: feas / cr_pl | CQ: feas, phase±, gain± | CAL: feas | resid phase° | resid gain % "
         "| groups (rpm/temp/creep/J) ph°/gain% | authority min | cr_pl ratio min | mech margin min | CAL viol |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for n, s in res.items():
        m, q, c = s["margin"], s["control_quality"], s["control_quality_cal"]
        g = c.get("groups") or {}
        gs = " ".join(f"{k} {v[0]:.1f}/{100 * v[1]:.0f}" for k, v in g.items())
        L.append(f"| {n} | {m['feasible_pl']} / {m['cr_pl']:.4f} | {q['feasible_pl']}, {q['phase_spread_deg']:.1f}, "
                 f"{100 * q['gain_spread']:.0f} % | {c['feasible_pl']} | {c['phase_res_deg']:.1f} "
                 f"({c['worst_phase_node']}) | {100 * c['gain_res']:.1f} ({c['worst_gain_node']}) | {gs} | "
                 f"{c['authority_min']:.2f} | {c['cr_pl_ratio_min']:.2f} | {c['mech_margin_min']:.3f}"
                 f"{' FLIP' if c['mech_side_flip'] else ''} | {c['viol_hard']} |")
    L += ["", "margin-criterion hard violations: " + "; ".join(f"{n}: {s['margin']['viol_hard']}" for n, s in res.items())]
    return "\n".join(L) + "\n"


def cal(names=None):
    res = json.loads(RES_CAL.read_text()) if RES_CAL.exists() else {}
    for n, (d, js) in cal_designs().items():
        if names and n not in names:
            continue
        res[n] = cal_summary(cal_eval(d, js))
        RES_CAL.write_text(json.dumps(res, indent=1, default=float))
        RES_CAL.with_suffix(".md").write_text(cal_md(res))
        c = res[n]["control_quality_cal"]
        print(n, {k: res[n][k]["feasible_pl"] for k in CRITS}, "cr_pl", res[n]["margin"]["cr_pl"],
              "res", c["phase_res_deg"], c["gain_res"], "auth", c["authority_min"], "mech", c["mech_margin_min"],
              flush=True)


if __name__ == "__main__":
    st = sys.argv[1]
    {"score": score, "probe": probe, "cal": lambda: cal(sys.argv[2:])}.get(st, lambda: tol(sys.argv[2]))()
