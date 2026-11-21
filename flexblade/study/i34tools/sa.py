"""I36 partial preset: score design (v5 base + alpha*v5zs presets, or a given json) under
control_quality_cal (continuous, 5 nodes) + Goodman + ground_resonance, and dump per-case
pulse diagnostics (scale, binding check, utilisation at vanishing pulse) + fatigue notes.
usage: sa.py alpha <a> <out.json> | sa.py file <in.json> <out.json>"""
import sys, os, json
os.environ.setdefault("FLEXBLADE_GROUND_RES", "1")
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import control_quality as cq
import run_study as rs
from flexblade import robust, api
from flexblade.schema import Design
from concurrent.futures import ProcessPoolExecutor
cq.WORKERS = int(os.environ.get("SA_WORKERS", 3))
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


def make(mode, arg):
    if mode == "alpha":
        a = float(arg)
        base = Design.model_validate_json(open(f"{ROOT}/designs/10in_tt_strap_steelx_v5.json").read())
        zs = Design.model_validate_json(open(f"{ROOT}/campaign/i34/10in_tt_strap_steelx_v5zs.json").read())
        p = [[[a * x for x in e] for e in m] for m in zs.flexure.strip_preset_deg]
        d = base.model_copy(update=dict(flexure=base.flexure.model_copy(update=dict(strip_preset_deg=p))))
    else:
        d = Design.model_validate_json(open(arg).read())
    u = dict(strip_fatigue_mean="goodman")
    if os.environ.get("SA_SKEW_ADD"):  # extra skew (deg) added on top of the design's skew_deg
        u["skew_deg"] = d.flexure.skew_deg + float(os.environ["SA_SKEW_ADD"])
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=u)))


def ev(dv):
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    out = {c: rs.metrics(dv, r, criterion=c) for c in cq.CRITS}
    fns = rs.pulse_util_fns(dv, r)
    s, b = rs.pulse_scale(fns)
    out["_diag"] = dict(s=s, bind=b, u0={n: round(f(1e-9), 3) for n, f in fns if f(1e-9) > 0.5},
                        fat=[(c["name"], round(c["utilisation"], 3), c.get("note")) for c in r.get("checks", [])
                             if c["name"].startswith("fatigue")])
    return out


def nominal(t):
    return ":p" not in t and ":r" not in t and ":c" not in t


def main():
    mode, arg, outp = sys.argv[1:4]
    d = make(mode, arg)
    open(outp, "w").write(d.model_dump_json(indent=1))
    js = cq.J10 if abs(d.blade.diameter_in - 10) < 0.5 else cq.JS
    if os.environ.get("SA_JHUB"):   # e.g. 6" 2307 hub: SA_JHUB=0.58e-6,1.57e-6,3.46e-6
        js = tuple(float(x) for x in os.environ["SA_JHUB"].split(","))
    rt = [x for x in robust.CQ_RUNTIME if not x[0].startswith("rpm")]
    v = robust.variants(d, robust.THRUST_FACTORS, list(robust.CQ_PERTURB), js, rt)
    th = sorted(set(sum(robust.cont_points(5), [])) - set(robust.THRUST_FACTORS))
    v += [(t.replace(":J", ":c:J") if ":J" in t else t + ":c", dv) for t, dv in
          robust.variants(d, th, list(robust.CQ_PERTURB), js, None)]
    with ProcessPoolExecutor(cq.WORKERS) as ex:
        ms = list(ex.map(ev, [dv for _, dv in v], chunksize=1))
    tags = [t for t, _ in v]
    for t, m in zip(tags, ms):
        g = m["_diag"]
        if nominal(t) or g["s"] <= 0:
            vh = m["control_quality_cal"].get("viol_hard") or m["control_quality_cal"].get("viol")
            print("CASE", t, "s=%.3f" % g["s"], g["bind"], g["u0"], vh, flush=True)
            if nominal(t):
                for f in g["fat"]:
                    print("  FAT", f, flush=True)
    sel = {"margin": nominal, "control_quality": lambda t: ":r" not in t and ":c" not in t,
           "control_quality_cal": lambda t: True}
    kw = dict(rpm_schedule="continuous", n_nodes=5, rpm_noise=robust.CQ_RPM_NOISE)
    cs = {c: robust.combine([m[c] for t, m in zip(tags, ms) if sel[c](t)], [t for t in tags if sel[c](t)],
                            **(kw if c == "control_quality_cal" else {})) for c in cq.CRITS}
    s = cq.cal_summary(cs)
    c = s["control_quality_cal"]
    if os.environ.get("SA_GATES") == "minimal":
        # opt-in gate set (study/results/gate_audit.md): the mechanism mode's raw 1P margin and
        # side flip duplicate the per-build calibration residual + authority (the unit's side is
        # measured at calibration; zeta bounds the 1P amplification). resonance (other modes,
        # damping-aware), Goodman/hard checks, ground resonance and cqc phase/gain/authority stay.
        c["viol_hard"] = robust.minimal_gates(c)   # also adds cqc_mech_zeta (zeta >= 0.02)
        c["feasible_pl"], c["gates"] = not c["viol_hard"], "minimal"
    print("SUM", outp.split("/")[-1], json.dumps(dict(
        cr_pl=c["cr_pl"], feas=c["feasible_pl"], viol=c["viol_hard"], ph=c["phase_res_deg"], gain=c["gain_res"],
        mech=c["mech_margin_min"], flip=c["mech_side_flip"], res_margin=c["res_margin"], res_which=c["res_which"]),
        default=str), flush=True)
    print("NODES", json.dumps(c.get("per_node"), default=str)[:1500], flush=True)
    print("MARGIN", json.dumps(s["margin"], default=str), flush=True)


if __name__ == "__main__":
    main()
