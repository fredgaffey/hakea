"""7in_tt_strap_steelx_v4 with the real printed steelx fitting masses (fittings.fitting_mass_props).
Stages: props | robust | retune | write
    PYTHONPATH=flexblade python flexblade/study/fitting_mass_v4.py <stage>
robust: v4 legacy (fitting_mass none) vs real (auto, PA6-CF) vs slim, 3 throttles + T1 stability.
retune: slim fittings, <= 40 designs over hinge_radius, length, skew_deg (3 throttles each).
Raw -> study/results/fitting_mass_v4.json; design -> study/designs/7in_tt_strap_steelx_v4m.json."""
from __future__ import annotations
import json, random, sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust, fittings as FT                # noqa: E402
from flexblade.optimize import set_design                        # noqa: E402
from flexblade.schema import Design                              # noqa: E402
import verify_v4 as V4                                           # noqa: E402

RES = HERE / "results" / "fitting_mass_v4.json"
V4F = HERE / "designs" / "7in_tt_strap_steelx_v4.json"
OUT = HERE / "designs" / "7in_tt_strap_steelx_v4m.json"
V4.WORKERS = 6
D4 = Design.model_validate_json(V4F.read_text())


def fl(d, **kw):
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=kw)))


def load():
    return json.loads(RES.read_text()) if RES.exists() else {}


def save(k, v):
    r = load(); r[k] = v
    RES.parent.mkdir(exist_ok=True); RES.write_text(json.dumps(r, indent=1, default=float))


def _stab(d):
    dv = dict(robust.variants(d, (1.0,)))["T1"]
    s = api.evaluate(dv).model_dump(mode="json")["summary"]
    return {k: s.get(k) for k in ("zeta_min", "zeta_min_per_rev", "air_resonance_zeta", "pitch_zeta", "lag_zeta")}


def run(sets: dict[str, Design], stab=True) -> dict:
    res = V4.run_sets(sets)
    if stab:
        with ProcessPoolExecutor(V4.WORKERS) as ex:
            for k, z in zip(sets, ex.map(_stab, sets.values())):
                res[k]["stab"] = z
    for k, v in res.items():
        m = v.pop("modes")["T1"]
        v["pitch_per_rev_T1"] = [round(f, 3) for lab, f, _ in m if "pitch" in lab]
    return res


def show(res):
    for k, v in res.items():
        print(f"{k:14s} cr {' / '.join(f'{v['cr'][t]:.3f}' for t in ('T0.7', 'T1', 'T1.4'))} feas {v['feasible']} "
              f"viol {list(v['viol'])} 1P {v['res_margin']:.3f} ({v['res_which']}) fat {v['fatigue']:.2f} "
              f"hp {v['hp_margin']:.3f} ({v['hp_which']}) pitch/rev {v['pitch_per_rev_T1'][:3]} "
              f"stab {v.get('stab')}")


def stage_props():
    out = {}
    for tag, d in (("pa6_cf", D4), ("slim_pa6_cf", fl(D4, fitting_slim=True)),
                   ("slim_petg_cf", fl(D4, fitting_slim=True, fitting_material="petg_cf")),
                   ("slim_pla", fl(D4, fitting_slim=True, fitting_material="pla"))):
        mp = FT.fitting_mass_props(d)
        out[tag] = {s: dict(mass_g=m * 1e3, com_mm=(c * 1e3).tolist(), I_gmm2=(I * 1e9).round(2).tolist())
                    for s, (m, c, I) in mp.items()}
        print(tag, {s: round(v["mass_g"], 3) for s, v in out[tag].items()},
              "I diag", {s: np.diag(v["I_gmm2"]).round(1).tolist() for s, v in out[tag].items()})
    save("props", out)


def stage_robust():
    res = run({"v4_legacy": fl(D4, fitting_mass="none"), "v4_real": D4, "v4_slim": fl(D4, fitting_slim=True)})
    save("robust", res); show(res)


def key(v):
    return (v["feasible"], V4.ok_targets(v), v["cr_min"])


def stage_retune(n0=24, n1=16, seed=5):
    base = fl(D4, fitting_slim=True)
    f = base.flexure
    rnd = random.Random(seed)

    def mk(h, L, s):
        return set_design(base, {"flexure.hinge_radius": h, "flexure.length": L, "flexure.skew_deg": s})
    xs = [(f.hinge_radius * rnd.uniform(0.8, 1.25), f.length * rnd.uniform(0.75, 1.3),
           f.skew_deg + rnd.uniform(-10, 10)) for _ in range(n0)]
    res = run({f"r{i:02d}": mk(*x) for i, x in enumerate(xs)}, stab=False)
    allx = {f"r{i:02d}": x for i, x in enumerate(xs)}
    best = max(res, key=lambda k: key(res[k]))
    bx = allx[best]
    xs2 = [(bx[0] * rnd.uniform(0.93, 1.07), bx[1] * rnd.uniform(0.92, 1.08), bx[2] + rnd.uniform(-3, 3))
           for _ in range(n1)]
    res |= run({f"l{i:02d}": mk(*x) for i, x in enumerate(xs2)}, stab=False)
    allx |= {f"l{i:02d}": x for i, x in enumerate(xs2)}
    rank = sorted(res, key=lambda k: key(res[k]), reverse=True)
    for k in rank[:8]:
        print(k, np.round(allx[k], 5).tolist()); show({k: res[k]})
    save("retune", {k: dict(x=allx[k], **res[k]) for k in res})


def stage_write(name=None):
    r = load()
    if name is None:
        rt = r["retune"]
        # no retune point was robust-feasible: take the least-violated (fewest violations, then 1P/kP margin)
        name = max(rt, key=lambda k: (rt[k]["feasible"], -len(rt[k]["viol"]), rt[k]["res_margin"]))
    h, L, s = r["retune"][name]["x"]
    d = set_design(fl(D4, fitting_slim=True), {"flexure.hinge_radius": h, "flexure.length": L, "flexure.skew_deg": s})
    fin = run({"v4m": d})
    show(fin)
    v = fin["v4m"]
    d = d.model_copy(update=dict(name="7in_tt_strap_steelx_v4m", notes=D4.notes + (
        f"; v4m: v4 re-tuned with real slim PA6-CF steelx fitting masses (study/fitting_mass_v4.py): "
        f"hinge_r {h*1e3:.2f} mm, L {L*1e3:.2f} mm, skew {s:.1f} deg; robust cr "
        + "/".join(f"{v['cr'][t]:.3f}" for t in ("T0.7", "T1", "T1.4"))
        + f", feasible {v['feasible']} (viol {', '.join(v['viol']) or '-'}), resonance margin "
        f"{v['res_margin']:.3f} ({v['res_which']}), fatigue {v['fatigue']:.2f}")))
    OUT.write_text(d.model_dump_json(indent=1))
    save("v4m", dict(source=name, **fin["v4m"]))


if __name__ == "__main__":
    {"props": stage_props, "robust": stage_robust, "retune": stage_retune, "write": stage_write}[sys.argv[1]](*sys.argv[2:])
