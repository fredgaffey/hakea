"""7in_tt_strap_steelx_v5 (idea I25): fittings that are their own counter-weight.
The blade-side steelx fitting's centrifugal propeller moment gives a pitch stiffness
K = Omega^2 (Syy - Szz) about the pitch axis (fittings.pitch_axis_moments): the v4 crossed jaws spread
chordwise (y) -> Syy >> Szz -> stiffens pitch (antisym pitch -> 1P). Two cancellations:
  (a/b) smaller cross_angle: strips l = cos(g) x' +- sin(g) y with x' = (cos d, 0, sin d) (skew d), so the
        jaws move from chordwise (y) toward the rotor axis (z); Syy - Szz -> 0 near cross ~ 90 deg;
  (c)   flexure.fitting_balance_mass: a +-z pin pair sized ('auto') to cancel Syy - Szz exactly.
Stages: props | search | tol [key] | write
    PYTHONPATH=flexblade python flexblade/study/fitting_balance_v5.py <stage>
Raw -> study/results/fitting_balance_v5.json; design -> study/designs/7in_tt_strap_steelx_v5.json
(overwrites the killed I23 v5 bifilar-strap file)."""
from __future__ import annotations
import json, random, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from flexblade import fittings as FT                             # noqa: E402
from flexblade.optimize import set_design                        # noqa: E402
from flexblade.schema import Design                              # noqa: E402
import fitting_mass_v4 as FM                                     # noqa: E402
import verify_v4 as V4                                           # noqa: E402
import verify_v3 as V3                                           # noqa: E402

RES = HERE / "results" / "fitting_balance_v5.json"
OUT = HERE / "designs" / "7in_tt_strap_steelx_v5.json"
D4M = Design.model_validate_json((HERE / "designs" / "7in_tt_strap_steelx_v4m.json").read_text())
V4.WORKERS = 6


def load():
    return json.loads(RES.read_text()) if RES.exists() else {}


def save(k, v):
    r = load(); r[k] = v
    RES.parent.mkdir(exist_ok=True); RES.write_text(json.dumps(r, indent=1, default=float))


def mk(x: dict) -> Design:
    d = set_design(D4M, {"flexure.cross_angle_deg": x["cross"], "flexure.hinge_radius": x["h"],
                         "flexure.length": x["L"], "flexure.skew_deg": x["skew"], "flexure.width": x["w"]})
    return FM.fl(d, fitting_balance_mass="auto" if x.get("pin") else 0.0, fitting_balance_z=x.get("pin") or 0.008)


def prop(d: Design) -> dict:
    m, c, I = FT.fitting_mass_props(d)["root"]
    syy, szz, syz = FT.pitch_axis_moments(m, c, I)
    Ig = np.diag(I) * 1e9
    return dict(mass_g=m * 1e3, com_mm=(c * 1e3).round(3).tolist(), Icg_gmm2=Ig.round(1).tolist(),
                Syy=syy * 1e9, Szz=szz * 1e9, Syz=syz * 1e9, Syy_minus_Szz=(syy - szz) * 1e9,
                Iyy_minus_Izz_pitch_axis=(szz - syy) * 1e9)


def stage_props():
    out = {}
    for tag, d in (("v4", FM.D4), ("v4m", D4M), ("v4m_pins_z8", FM.fl(D4M, fitting_balance_mass="auto")),
                   *[(f"v4m_cross{c}", set_design(D4M, {"flexure.cross_angle_deg": c})) for c in (120, 100, 90, 80)]):
        out[tag] = prop(d)
        print(tag, {k: (round(v, 1) if isinstance(v, float) else v) for k, v in out[tag].items()})
    out["v4m_pin_mass_g_z8"] = FT.balance_pin_mass(D4M) * 1e3
    save("props", out)


def _x(rnd, c=None, s=1.0):
    f = D4M.flexure
    if c is None:
        return dict(cross=rnd.uniform(78, 125), h=f.hinge_radius * rnd.uniform(0.85, 1.2),
                    L=f.length * rnd.uniform(0.8, 1.35), skew=f.skew_deg + rnd.uniform(-10, 12),
                    w=rnd.uniform(0.007, 0.010), pin=0.008 if rnd.random() < 0.25 else 0.0)
    return dict(cross=c["cross"] + s * rnd.uniform(-8, 8), h=c["h"] * (1 + s * rnd.uniform(-.07, .07)),
                L=c["L"] * (1 + s * rnd.uniform(-.08, .08)), skew=c["skew"] + s * rnd.uniform(-3, 3),
                w=float(np.clip(c["w"] * (1 + s * rnd.uniform(-.08, .08)), 0.006, 0.011)), pin=c["pin"])


def rank(res):
    return sorted(res, key=lambda k: (res[k]["feasible"], V4.ok_targets(res[k]), -len(res[k]["viol"]),
                                      min(res[k]["res_margin"] or 0, 0.25), res[k]["cr_min"]), reverse=True)


def stage_search(n0=36, n1=24, seed=25):
    rnd = random.Random(seed)
    xs = {f"s{i:02d}": _x(rnd) for i in range(int(n0))}
    res = FM.run({k: mk(x) for k, x in xs.items()}, stab=False)
    rk = rank(res)
    for k in rk[:6]:
        print(k, xs[k]); FM.show({k: res[k]})
    save("search", {k: dict(x=xs[k], **res[k]) for k in rk})
    xs2 = {f"l{i:02d}": _x(rnd, xs[rk[i % 2]]) for i in range(int(n1))}
    res |= FM.run({k: mk(x) for k, x in xs2.items()}, stab=False)
    xs |= xs2
    rk = rank(res)
    print("---- final ranking")
    for k in rk[:8]:
        print(k, xs[k]); FM.show({k: res[k]})
    save("search", {k: dict(x=xs[k], **res[k]) for k in rk})


def stage_tol(key=None):
    s = load()["search"]
    key = key or next(iter(s))
    d = mk(s[key]["x"])
    sets = {"nominal": d} | {k: V3.perturbed(d, spec) for k, spec in V4.samples(8, seed=11).items()}
    r = FM.run(sets, stab=False)
    summ = V4.summarize(r)
    print(key, summ)
    save("tol", dict(key=key, x=s[key]["x"], summary=summ,
                     samples={k: {a: v[a] for a in ("cr", "feasible", "viol", "res_margin", "res_which", "fatigue")}
                              for k, v in r.items()}))


def stage_write():
    r = load(); t = r["tol"]; x = t["x"]; d = mk(x)
    nom = r["search"][t["key"]]
    p, sm = prop(d), t["summary"]
    keep = sm["all_feasible"]
    pins = "auto at +-%.1f mm" % (x["pin"] * 1e3) if x["pin"] else "none"
    d = d.model_copy(update=dict(name="7in_tt_strap_steelx_v5", notes=D4M.notes + (
        f"; v5 (I25, study/fitting_balance_v5.py; OVERWRITES the killed I23 bifilar-strap v5 file, see "
        f"results/verify_7in_v5.json): root fitting as its own counter-weight, propeller-moment pitch "
        f"stiffness Omega^2 (Syy - Szz) about the pitch axis -> {p['Syy_minus_Szz']:.0f} g mm^2 "
        f"(v4m 185, v4 390); cross {x['cross']:.1f} deg, hinge_r {x['h']*1e3:.2f} mm, L {x['L']*1e3:.2f} mm, "
        f"skew {x['skew']:.1f} deg, w {x['w']*1e3:.2f} mm, balance pins {pins}; "
        f"root fitting {p['mass_g']:.2f} g; robust cr " + "/".join(f"{nom['cr'][k]:.3f}" for k in ("T0.7", "T1", "T1.4"))
        + " (P20 " + "/".join(f"{sm['p20_per_throttle'][k]:.3f}" for k in ("T0.7", "T1", "T1.4"))
        + f"), feasible {nom['feasible']} (viol {', '.join(nom['viol']) or '-'}), resonance margin "
        f"{nom['res_margin']:.3f} ({nom['res_which']}), fatigue {nom['fatigue']:.2f}; tolerance nominal + 8 "
        f"random +-5 %: {sm['n_feasible']}/{sm['n']} feasible; {'KEEP' if keep else 'KILL'}")))
    OUT.write_text(d.model_dump_json(indent=1))
    from flexblade import drawing
    drawing.sheet(d, str(HERE / "results" / "steelx_7in_v5.png"))
    print(d.notes[-800:])


if __name__ == "__main__":
    {"props": stage_props, "search": stage_search, "tol": stage_tol, "write": stage_write}[sys.argv[1]](*sys.argv[2:])
