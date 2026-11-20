"""Independent verification of study/designs/7in_tt_strap_steelx_v3.json.
Stages: tol | harm | ccx | floq.   PYTHONPATH=flexblade python flexblade/study/verify_v3.py <stage>
Raw results -> study/results/verify_7in_v3.json (each stage merges its key); report_v3.py-style
markdown is written by `report`."""
from __future__ import annotations
import json, math, sys, random
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust                                # noqa: E402
from flexblade.optimize import get_path, set_design               # noqa: E402
from flexblade.schema import Design, Airframe                     # noqa: E402
import run_study as rs                                            # noqa: E402

RES = HERE / "results" / "verify_7in_v3.json"
NOM = Design.model_validate_json((HERE / "designs" / "7in_tt_strap_steelx_v3.json").read_text())


def load():
    return json.loads(RES.read_text()) if RES.exists() else {}


def save(k, v):
    r = load(); r[k] = v; RES.write_text(json.dumps(r, indent=1, default=float))


# ------------------------------------------------------------------ 1. tolerances
def perturbed(d, spec):
    """spec: list of (kind, path, value), kind in mul|add|set|rho"""
    for kind, path, v in spec:
        if kind == "mul":
            d = set_design(d, {path: float(get_path(d, path)) * v})
        elif kind == "add":
            d = set_design(d, {path: float(get_path(d, path)) + v})
        elif kind == "set":
            d = set_design(d, {path: v})
        elif kind == "rho":
            lay = d.blade.layup.model_copy(update=dict(material=f"pa6_cf@rho={v}"))
            d = d.model_copy(update=dict(blade=d.blade.model_copy(update=dict(layup=lay))))
    return d


def cases():
    c = {}
    T, W, L = "flexure.thickness", "flexure.width", "flexure.length"
    c["t=0.13mm"] = [("set", T, 0.00013)]
    c["t=0.20mm"] = [("set", T, 0.00020)]
    for n, p in (("w", W), ("L", L), ("hinge_r", "flexure.hinge_radius")):
        c[f"{n} -10%"] = [("mul", p, 0.9)]; c[f"{n} +10%"] = [("mul", p, 1.1)]
    c["skew -2deg"] = [("add", "flexure.skew_deg", -2.0)]; c["skew +2deg"] = [("add", "flexure.skew_deg", 2.0)]
    c["blade mass -10%"] = [("rho", "", 0.9)]; c["blade mass +10%"] = [("rho", "", 1.1)]
    c["elastic axis -0.02c"] = [("add", "blade.elastic_axis_chord_frac", -0.02)]
    c["elastic axis +0.02c"] = [("add", "blade.elastic_axis_chord_frac", 0.02)]
    s = "flexure.strap_strand_diameter"      # strap EA ~ d^2
    c["strap k -20%"] = [("mul", s, math.sqrt(0.8))]; c["strap k +20%"] = [("mul", s, math.sqrt(1.2))]
    rnd = random.Random(7)
    for i in range(20):
        u = lambda a: 1 + rnd.uniform(-a, a)
        c[f"random{i:02d}"] = [("mul", T, u(.05)), ("mul", W, u(.05)), ("mul", L, u(.05)),
                               ("mul", "flexure.hinge_radius", u(.05)), ("rho", "", round(u(.05), 4)),
                               ("mul", s, math.sqrt(u(.05))), ("add", "flexure.skew_deg", rnd.uniform(-1, 1)),
                               ("add", "blade.elastic_axis_chord_frac", rnd.uniform(-.01, .01))]
    return c


def _eval_point(args):
    tag, d = args
    r = api.evaluate(d, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    return tag, rs.metrics(d, r)


def tol(workers=3):
    cs = {"nominal": []} | cases()
    jobs, owner = [], []
    for name, spec in cs.items():
        d = perturbed(NOM, spec)
        for tag, dv in robust.variants(d):
            jobs.append((tag, dv)); owner.append(name)
    with ProcessPoolExecutor(workers) as ex:
        outs = list(ex.map(_eval_point, jobs))
    out = {}
    for name in cs:
        ms = [(t, m) for (t, m), o in zip(outs, owner) if o == name]
        c = robust.combine([m for _, m in ms], [t for t, _ in ms])
        out[name] = dict(cr_min=c["control_ratio"], feasible=c["feasible"], viol=c["viol"],
                         res_margin=c.get("res_margin"), res_which=c.get("res_which"),
                         cr={t: m["control_ratio"] for t, m in ms})
    save("tol", out)


# ------------------------------------------------------------------ 2. harmonics
def harm():
    from flexblade import rotor as R
    from flexblade.rotor import K_complex, aero_linear
    out = {}
    for tag, d in robust.variants(NOM):
        with R.part_temperature(d.op.ambient_C):
            if d.flexure.pre_lag_deg == "auto":
                d = R.resolve_pre_lag(d)
            model = R.build_model(d); aero = R.make_aero(model)
            ss = R.static_trim(model, aero); lin = R.linearize(model, ss, d.analysis.coriolis)
            md = R.modes(model, ss, aero, lin=lin)
            w = ss.Omega; f1 = w / 2 / np.pi
            modes = [(float(f / f1), float(z), lab) for f, z, lab in zip(md.freqs_hz, md.damping, md.labels) if f > 0.05 * f1]
            near = {k: [(lab, round(fr, 3), round(fr / k - 1, 3), round(z, 4)) for fr, z, lab in modes if abs(fr / k - 1) <= 0.10]
                    for k in (4, 5, 6)}
            # unit-torque harmonic solve at k*Omega (hub torque; pure sinusoid at kP)
            n = model.fe.ndof; f = lin.free; hubd = 6 * model.hub + 5
            resp = {}
            for k in (1, 2, 3, 4, 5, 6):
                wk = k * w
                Z = K_complex(model, lin, wk / 2 / np.pi) + 1j * wk * lin.G - wk * wk * lin.M
                al = aero_linear(model, ss.state, aero, ss.trim, wk, d.analysis.dynamic_inflow)
                Z = Z - al.Ka
                nf = len(f); A = np.zeros((nf + 1, nf + 1), complex)
                A[:nf, :nf] = Z[np.ix_(f, f)]; A[:nf, nf] = -al.b[f]
                A[nf, :nf] = -al.kappa * al.c[f]; A[nf, nf] = 1 - al.kappa * al.c0
                rhs = np.zeros(nf + 1, complex); rhs[list(f).index(hubd)] = 1.0
                sol = np.linalg.solve(A, rhs)
                U = np.zeros(n, complex); U[f] = sol[:nf]
                bi = model.blades[0]
                u, rv = R.blade_local_motion_c(model, U, bi)
                i75 = np.argmin(np.abs(bi.chain_r - 0.75 * model.R))
                hub6 = np.r_[6 * model.hub:6 * model.hub + 6]
                Rh = Z[hub6] @ U
                Rh[5] -= 1.0
                Rh = Rh - al.b[hub6] * sol[nf]
                resp[k] = dict(pitch75_deg_per_Nm=float(np.degrees(abs(rv[i75, 0]))),
                               root_pitch_deg_per_Nm=float(np.degrees(abs(rv[0, 0]))),
                               hub_inplane_moment_per_Nm=float(np.hypot(abs(Rh[3]), abs(Rh[4]))))
            out[tag] = dict(rpm=float(ss.rpm), f1_hz=float(f1), modes_per_rev=modes, near=near, resp=resp)
    save("harm", out)


# ------------------------------------------------------------------ 3. ccx
def ccx_cmp():
    import tempfile
    from flexblade import ccx
    wd = Path(tempfile.mkdtemp(prefix="v3ccx_", dir="/tmp"))
    r = ccx.compare_strip_flexure(NOM, wd)
    cv = lambda v: v.tolist() if hasattr(v, "tolist") else float(v)
    save("ccx", dict(ratio=r["ratio"], ccx={k: cv(v) for k, v in r["ccx"].items()},
                     beam={k: cv(v) for k, v in r["beam"].items()},
                     axis_err_deg=r["axis_err_deg"], axis_design_err_deg=r["axis_design_err_deg"]))


# ------------------------------------------------------------------ 4. floquet
def _floq(args):
    tag, d, fa = args
    d = d.model_copy(update=dict(
        analysis=d.analysis.model_copy(update=dict(stability="floquet")),
        op=d.op.model_copy(update=dict(airframe=Airframe(f_hz=fa, f_y_hz=1.15 * fa, zeta=0.02)))))
    r = api.evaluate(d, ["trim", "modes"]).model_dump(mode="json")
    s = r["summary"]
    return tag, fa, s.get("air_resonance_zeta"), s.get("zeta_min"), [w for w in r["warnings"] if "stab" in w.lower()]


def floq(workers=3):
    jobs = [(t, d, fa) for t, d in robust.variants(NOM) for fa in (50, 80, 120, 160, 200, 250, 300, 400)]
    with ProcessPoolExecutor(workers) as ex:
        res = list(ex.map(_floq, jobs))
    save("floq", [dict(tag=t, f_hz=fa, air_zeta=z, zeta_min=zm, warn=w) for t, fa, z, zm, w in res])


def floq2(workers=3):
    """refined airframe sweep around the 'pitch antisym+hub' mode (0.92/rev), where the coarse sweep found negative zeta"""
    f1 = {"T0.7": 185.6, "T1": 220.4, "T1.4": 261.6}
    jobs = [(t, d, round(0.92 * f1[t] * x, 1)) for t, d in robust.variants(NOM) for x in (0.8, 0.88, 0.94, 1.0, 1.06, 1.12, 1.2)]
    with ProcessPoolExecutor(workers) as ex:
        res = list(ex.map(_floq, jobs))
    save("floq2", [dict(tag=t, f_hz=fa, air_zeta=z, zeta_min=zm, warn=w) for t, fa, z, zm, w in res])


if __name__ == "__main__":
    {"tol": tol, "harm": harm, "ccx": ccx_cmp, "floq": floq, "floq2": floq2}[sys.argv[1]]()
