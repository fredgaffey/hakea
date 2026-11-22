"""Tolerance check: nominal + N random builds (strip t/w/L, hinge_radius each U(0.95,1.05)), printed
slot presets (strip_preset_deg) kept FIXED. T0.7/1.0/1.4 at J 3.76e-6, Goodman + ground_resonance.
env: TOL_PRESET_DEG=x (perturb presets +-x deg, no re-solve), CQ_TOL_MAG (default 0.05), TOL_TF=0.7,1.0,1.25, TOL_J=3.4e-7,...
usage: tol.py <design.json> [N=8] [seed=0]"""
import sys, os, random
os.environ.setdefault("FLEXBLADE_GROUND_RES", "1")
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import run_study as rs
from flexblade import robust, api
from flexblade.schema import Design
from concurrent.futures import ProcessPoolExecutor
TF = [float(x) for x in os.environ["TOL_TF"].split(",")] if os.environ.get("TOL_TF") else robust.THRUST_FACTORS
JS = [float(x) for x in os.environ["TOL_J"].split(",")] if os.environ.get("TOL_J") else [3.76e-6]
PRE = float(os.environ.get("TOL_PRESET_DEG", 0))   # opt-in preset angle tolerance, deg
KEYS = ("thickness", "width", "length", "hinge_radius")


def ev(dv):
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    return rs.metrics(dv, r, criterion="margin")


def main():
    d = Design.model_validate_json(open(sys.argv[1]).read())
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    rng = random.Random(int(sys.argv[3]) if len(sys.argv) > 3 else 0)
    builds = [("nom", d)]
    for i in range(n):
        R = robust.tol_ranges()
        f = {k: rng.uniform(1 - R[k], 1 + R[k]) for k in KEYS}
        fx = d.flexure.model_copy(update={k: getattr(d.flexure, k) * v for k, v in f.items()})
        if PRE:   # opt-in: printed slot-angle error, uniform +-PRE deg per strip end on theta2/theta3, no re-solve
            sp = [[[x + (rng.uniform(-PRE, PRE) if j % 3 else 0.0) for j, x in enumerate(e)] for e in m]
                  for m in d.flexure.strip_preset_deg]
            fx = fx.model_copy(update=dict(strip_preset_deg=sp))
        builds.append((" ".join(f"{k[0]}{v:.3f}" for k, v in f.items()), d.model_copy(update=dict(flexure=fx))))
    # keyed by build index: labels collide when CQ_TOL_MAG=0 (presets-only builds all read t1.000 ...)
    jobs = [(i, t, dv) for i, (_, bd) in enumerate(builds) for t, dv in robust.variants(bd, TF, None, JS, None)]
    with ProcessPoolExecutor(int(os.environ.get("TOL_WORKERS", 3))) as ex:
        ms = list(ex.map(ev, [j[2] for j in jobs], chunksize=1))
    for i, (b, _) in enumerate(builds):
        sel = [(t, m) for (ii, t, _), m in zip(jobs, ms) if ii == i]
        c = robust.combine([m for _, m in sel], [t for t, _ in sel])
        vh = {k: round(v, 3) for k, v in (c.get("viol_hard") or {}).items()}
        print(f"{b:40s} cr_pl {c.get('cr_pl', 0):.4f} res {c.get('res_margin', 0):.3f} ({c.get('res_which')}) "
              f"viol {vh}", flush=True)


if __name__ == "__main__":
    main()
