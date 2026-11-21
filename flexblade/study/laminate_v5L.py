"""I28: laminated unbonded shim stacks on the 7in steelx v5 lead (study/designs/..._v5.json).

Each 0.15 mm strip -> n laminae of stock t, width retuned for the same total thin-axis bending
stiffness (n w t^3 = w0 t0^3). Pass 1 (eta_fric = 0) gives the 1P clamp curvature; the
clamp-end Coulomb friction loss factor (laminate.friction_eta, mu 0.15) is then fed back as
flexure.strip_lam_eta for pass 2. Robust at thrust 0.7/1.0/1.4; `tol` = nominal + 8 random +-5 %.

    PYTHONPATH=flexblade python flexblade/study/laminate_v5L.py [sweep|tol|save]
-> study/results/laminate_v5L.json
"""
from __future__ import annotations
import json, math, random, sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust, laminate, rotor                # noqa: E402
from flexblade.materials import get_material                     # noqa: E402
from flexblade.optimize import get_path, set_design               # noqa: E402
from flexblade.schema import Design                               # noqa: E402
import run_study as rs                                            # noqa: E402

RES = HERE / "results" / "laminate_v5L.json"
NOM = Design.model_validate_json((HERE / "designs" / "7in_tt_strap_steelx_v5.json").read_text())
T0, W0 = NOM.flexure.thickness, NOM.flexure.width
CANDS = [(1, 1.5e-4), (2, 1.0e-4), (2, 1.3e-4), (3, 1.0e-4), (3, 8e-5), (4, 8e-5), (4, 1.0e-4)]


def load():
    return json.loads(RES.read_text()) if RES.exists() else {}


def save(k, v):
    r = load(); r[k] = v; RES.write_text(json.dumps(r, indent=1, default=float))


def lam_design(n, t, eta=0.0, base=NOM):
    w = base.flexure.width * base.flexure.thickness**3 / (n * t**3)
    return set_design(base, {"flexure.strip_laminae": n, "flexure.thickness": t, "flexure.width": w,
                             "flexure.strip_lam_eta": eta})


def _eval(args):
    tag, d = args
    r = api.evaluate(d, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    m = rs.metrics(d, r)
    s_ = r.get("summary") or {}
    m["min_zeta"] = float(s_.get("zeta_min", float("nan")))
    m["margin_1p"] = float(s_.get("near_1p_margin_eff", float("nan")))
    fm = {}
    for c in r.get("checks", []):
        if c["name"].startswith("fatigue"):
            fm[c["member"]] = max(fm.get(c["member"], 0.0), c["utilisation"])
    m["fat_members"] = fm
    return tag, m


def robust_eval(pool, designs: dict) -> dict:
    jobs, owner = [], []
    for name, d in designs.items():
        for tag, dv in robust.variants(d):
            jobs.append((tag, dv)); owner.append(name)
    outs = list(pool.map(_eval, jobs))
    res = {}
    for name in designs:
        ms = [(t, m) for (t, m), o in zip(outs, owner) if o == name]
        c = robust.combine([m for _, m in ms], [t for t, _ in ms])
        u = c.get("util", {})
        res[name] = dict(cr={t: m["control_ratio"] for t, m in ms}, cr_min=c["control_ratio"],
                         feasible=c["feasible"], viol=c["viol"], res_margin=c.get("res_margin"),
                         res_which=c.get("res_which"), fatigue=max(u.get("fatigue_strain", 0), u.get("fatigue_stress", 0)),
                         jitter=u.get("harmonic_jitter"), min_zeta=min(m["min_zeta"] for _, m in ms),
                         strain_1p=max(m.get("strip_strain_amp_1p", 0.0) for _, m in ms),
                         margin_1p=min(m["margin_1p"] for _, m in ms),
                         fat_members={k: round(max(m["fat_members"].get(k, 0.0) for _, m in ms), 3)
                                      for k in ms[0][1]["fat_members"]})
    return res


def eta_for(d, strain_1p):
    f = d.flexure
    kfac, _ = rotor.strip_clamp_factor(f)
    kap = 2 * strain_1p / (kfac * f.thickness)
    E = get_material(f.layup.material).E1
    return laminate.friction_eta(f.strip_laminae, E, f.width, f.thickness, f.length, kap), kap


def sweep(pool):
    p1 = robust_eval(pool, {f"n{n}_t{t*1e3:.2f}": lam_design(n, t) for n, t in CANDS})
    designs = {}
    for (n, t), (k, v) in zip(CANDS, p1.items()):
        eta, kap = eta_for(lam_design(n, t), v["strain_1p"])
        v.update(eta_fric=eta, kappa_1p=kap, width=lam_design(n, t).flexure.width)
        if n > 1:
            designs[k + "+fric"] = lam_design(n, t, eta)
    p2 = robust_eval(pool, designs)
    out = p1 | p2
    save("sweep", out)
    for k, v in out.items():
        print(f"{k:18s} w={v.get('width', 0)*1e3:5.2f} cr={[round(x, 3) for x in v['cr'].values()]} "
              f"fat={v['fatigue']:.3f} jit={v['jitter']} res={v['res_margin']:.3f} ({v['res_which']}) "
              f"zmin={v['min_zeta']:.4f} m1p={v['margin_1p']:.3f} fm={v['fat_members']} eta_f={v.get('eta_fric', 0):.2e} feas={v['feasible']} {list(v['viol'])}")


def tol(pool, key):
    n, t = (int(key[1]), float(key.split("_t")[1].split("+")[0]) * 1e-3)
    eta = load()["sweep"][key.split("+")[0]]["eta_fric"]
    base = lam_design(n, t, eta)
    rnd = random.Random(28)
    ds = {"nominal": base}
    for i in range(8):
        u = lambda: 1 + rnd.uniform(-0.05, 0.05)
        ds[f"random{i}"] = set_design(base, {p: float(get_path(base, p)) * u() for p in (
            "flexure.thickness", "flexure.width", "flexure.length", "flexure.hinge_radius",
            "flexure.strap_strand_diameter")})
    out = robust_eval(pool, ds)
    save("tol:" + key, out)
    npass = sum(v["feasible"] for v in out.values())
    for k, v in out.items():
        print(f"{k:10s} cr={v['cr_min']:.3f} fat={v['fatigue']:.3f} fm={v['fat_members']} zmin={v['min_zeta']:.4f} res={v['res_margin']:.3f} feas={v['feasible']} {list(v['viol'])}")
    print(f"pass {npass}/{len(out)}")


if __name__ == "__main__":
    with ProcessPoolExecutor(3) as pool:
        if sys.argv[1] == "sweep":
            sweep(pool)
        elif sys.argv[1] == "tol":
            tol(pool, sys.argv[2])
