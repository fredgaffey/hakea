"""10" tt_strap_steelx v4: hand design from first principles on the current model
(fitting mass, I25 small cross angle, forced-response nP criterion, radiused clamp,
I28 laminated strips, balance masses, pitch_cf_ratio).

Start = 10" v3 with the 7" v5L recipe scaled by D (10/7): cross 105 deg, skew 60 deg,
radiused clamp, 4 x 0.10 mm laminae, hinge radius / gap / strap pin scaled. Robust at
thrust 0.7/1.0/1.4 (robust.variants); `tol` = nominal + 8 random +-5 %.

    PYTHONPATH=flexblade python flexblade/study/steelx_10in_v4.py diag
    PYTHONPATH=flexblade python flexblade/study/steelx_10in_v4.py batch '<json {tag: {path: val}}>'
    PYTHONPATH=flexblade python flexblade/study/steelx_10in_v4.py tol '<json x>'
    PYTHONPATH=flexblade python flexblade/study/steelx_10in_v4.py save '<json x>'
-> study/results/steelx_10in_v4.json
"""
from __future__ import annotations
import json, math, random, sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust, laminate, rotor                # noqa: E402
from flexblade.materials import get_material                     # noqa: E402
from flexblade.optimize import get_path, set_design               # noqa: E402
from flexblade.schema import BalanceMass, Design                  # noqa: E402
import run_study as rs                                            # noqa: E402

RES = HERE / "results" / "steelx_10in_v4.json"
V3 = Design.model_validate_json((HERE / "designs" / "10in_tt_strap_steelx_v3.json").read_text())
OUT = HERE / "designs" / "10in_tt_strap_steelx_v4.json"
S = 10.0 / 7.0
# 7" v5L recipe scaled by D. Strip: v5 0.15 x 6.97 mm -> x S width, then 4 laminae of 0.10 mm
# at equal thin-axis bending stiffness (n w t^3 = w0 t0^3).
BASE = {"flexure.cross_angle_deg": 105.0, "flexure.skew_deg": 60.0,
        "flexure.hinge_radius": 0.0148 * S, "flexure.length": 0.0149 * S,
        "flexure.cross_gap": 0.0019 * S, "flexure.strap_pin_diameter": 0.006 * S,
        "flexure.strap_strand_diameter": 0.0015 * S, "blade.elastic_axis_chord_frac": 0.35,
        "flexure.strip_clamp": "compliant", "flexure.strip_clamp_radius": 0.0,
        "flexure.fitting_mass": "auto", "flexure.fitting_slim": True,
        "flexure.strip_laminae": 4, "flexure.thickness": 1.0e-4,
        "flexure.width": 0.00697 * S * 0.15**3 / (4 * 0.10**3),
        "flexure.strip_lam_eta": 4.8e-4}
PERTURB = ("flexure.thickness", "flexure.width", "flexure.length", "flexure.hinge_radius",
           "flexure.strap_strand_diameter")


def load():
    return json.loads(RES.read_text()) if RES.exists() else {}


def save(k, v):
    r = load(); r[k] = v; RES.write_text(json.dumps(r, indent=1, default=float))


def build(x: dict | None = None) -> Design:
    x = {**BASE, **(x or {})}
    bal = x.pop("balance", None)
    raw = {k: x.pop(k) for k in list(x) if isinstance(x[k], (str, bool))}
    d = set_design(V3, x)
    upd = dict(name="10in_tt_strap_steelx_v4")
    upd["flexure"] = d.flexure.model_copy(update={k.split(".", 1)[1]: v for k, v in raw.items()})
    d = d.model_copy(update=upd)
    d = Design.model_validate_json(d.model_dump_json())          # re-validate (str/bool fields)
    upd = {}
    if bal is not None:
        upd["blade"] = d.blade.model_copy(update=dict(balance_masses=[BalanceMass(**b) for b in bal]))
    return d.model_copy(update=upd)


def _modes(r, rpm):
    f0 = rpm / 60.0
    return [(m["label"], round(m["freq_hz"] / f0, 3), round(m["damping_ratio"], 4))
            for m in r.get("modes", []) if m["freq_hz"] > 0]


def _eval(args):
    tag, d = args
    try:
        r = api.evaluate(d, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    except Exception as e:                      # noqa: BLE001
        return tag, dict(error=str(e))
    m = rs.metrics(d, r)
    s_ = r.get("summary") or {}
    for k in ("zeta_min", "near_1p_margin_eff", "pitch_cf_ratio", "pitch_k_eff", "pitch_k_struct",
              "pitch_k_prop", "hub_moment_mean", "np_jitter_per_rev", "np_jitter_zeta", "np_strip_fat_util", "strip_strain_amp_1p", "near_1p_per_rev", "near_1p_zeta"):
        m[k] = float(s_.get(k, float("nan")))
    m["modes"] = _modes(r, s_.get("rpm", 1.0))
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
        ms = [(t, m) for (t, m), o in zip(outs, owner) if o == name and "control_ratio" in m]
        if len(ms) < 3:
            res[name] = dict(error=[m.get("error") for (t, m), o in zip(outs, owner) if o == name], feasible=False,
                             cr_min=0.0)
            continue
        c = robust.combine([m for _, m in ms], [t for t, _ in ms])
        u = c.get("util", {})
        res[name] = dict(cr={t: m["control_ratio"] for t, m in ms}, cr_min=c["control_ratio"],
                         feasible=c["feasible"], viol=c["viol"], res_margin=c.get("res_margin"),
                         res_which=c.get("res_which"), np_info=sorted({i for _, m in ms for i in m.get("res_np_info", [])}),
                         fatigue=max(u.get("fatigue_strain", 0), u.get("fatigue_stress", 0)),
                         jitter=u.get("harmonic_jitter"), min_zeta=min(m["zeta_min"] for _, m in ms),
                         strain_1p=max(m.get("strip_strain_amp_1p", 0.0) for _, m in ms),
                         pcf={t: round(m["pitch_cf_ratio"], 3) for t, m in ms},
                         k_struct={t: round(m["pitch_k_struct"], 4) for t, m in ms},
                         M_cyc={t: m["hub_moment_mean"] for t, m in ms},
                         fat_np={t: round(m["np_strip_fat_util"], 3) for t, m in ms}, e1p={t: round(m["strip_strain_amp_1p"] * 1e3, 3) for t, m in ms},
                         near1p={t: (round(m["near_1p_per_rev"], 3), round(m["near_1p_zeta"], 3)) for t, m in ms},
                         modes_T1=[m for t, m in ms if t.startswith("T1") and "1.4" not in t][0]["modes"]
                         if any(t.startswith("T1") and "1.4" not in t for t, _ in ms) else ms[0][1]["modes"],
                         fat_members={k: round(max(m["fat_members"].get(k, 0.0) for _, m in ms), 3)
                                      for k in ms[0][1]["fat_members"]})
    return res


def eta_for(d, strain_1p):
    f = d.flexure
    kfac, _ = rotor.strip_clamp_factor(f)
    kap = 2 * strain_1p / (kfac * f.thickness)
    E = get_material(f.layup.material).E1
    return laminate.friction_eta(f.strip_laminae, E, f.width, f.thickness, f.length, kap)


def show(res):
    for k, v in res.items():
        if "cr" not in v:
            print(k, "ERROR", v.get("error")); continue
        print(f"{k:14s} cr={[round(x, 4) for x in v['cr'].values()]} feas={v['feasible']} viol={ {a: round(b, 3) for a, b in v['viol'].items()} } "
              f"res={v['res_margin']:.3f} ({v['res_which']}) fat={v['fatigue']:.3f} fm={v['fat_members']} "
              f"zmin={v['min_zeta']:.4f} pcf={list(v['pcf'].values())} ks={list(v['k_struct'].values())} "
              f"M={[round(x, 4) for x in v['M_cyc'].values()]} fnp={list(v['fat_np'].values())} e1p={list(v['e1p'].values())} n1p={list(v['near1p'].values())}")


if __name__ == "__main__":
    cmd = sys.argv[1]
    with ProcessPoolExecutor(4) as pool:
        if cmd == "diag":
            ds = {"v3": V3}
            for n in ("7in_tt_strap_steelx_v5", "7in_tt_strap_steelx_v5L"):
                ds[n] = Design.model_validate_json((HERE / "designs" / f"{n}.json").read_text())
            out = robust_eval(pool, ds)
            save("diag", out); show(out)
            for k, v in out.items():
                print(k, "modes T1:", [m for m in v.get("modes_T1", []) if "pitch" in m[0] or "hub" in m[0]])
        elif cmd == "batch":
            xs = json.loads(sys.argv[2])
            out = robust_eval(pool, {k: build(x) for k, x in xs.items()})
            for k in out:
                out[k]["x"] = xs[k]
            r = load().get("batch", {}); r.update(out); save("batch", r); show(out)
            for k, v in out.items():
                print(k, "modes T1:", [m for m in v.get("modes_T1", []) if "pitch" in m[0] or "hub" in m[0]])
        elif cmd == "tol":
            x = json.loads(sys.argv[2])
            base = build(x)
            rnd = random.Random(29)
            ds = {"nominal": base}
            for i in range(8):
                ds[f"random{i}"] = set_design(base, {p: float(get_path(base, p)) * (1 + rnd.uniform(-0.05, 0.05))
                                                    for p in PERTURB})
            out = robust_eval(pool, ds)
            save("tol", dict(x=x, res=out)); show(out)
            print("pass", sum(v["feasible"] for v in out.values()), "/", len(out))
            for t in out["nominal"]["cr"]:
                crs = sorted(v["cr"][t] for v in out.values() if "cr" in v)
                print(t, "nominal", round(out["nominal"]["cr"][t], 4), "P20", round(crs[max(0, int(0.2 * (len(crs) - 1)))], 4))
        elif cmd == "eta":
            x = json.loads(sys.argv[2]); s1 = float(sys.argv[3])
            print(eta_for(build(x), s1))
        elif cmd == "save":
            d = build(json.loads(sys.argv[2]))
            OUT.write_text(d.model_dump_json(indent=1))
            from flexblade import drawing
            drawing.sheet(d, str(HERE / "results" / "steelx_10in_v4.png"))
