"""Selective assembly: like tol.py but lamina count n=round(9*.05^3/t^3) chosen from the build's thickness.
usage: tol_sel.py <design> N seed mode   mode: base (n=9) | sel | real (sel + w2.5% L,h 2%)"""
import sys, os, random
os.environ.setdefault("FLEXBLADE_GROUND_RES", "1")
import tol
from tol import robust, Design, ev, KEYS, ProcessPoolExecutor


def main():
    d = Design.model_validate_json(open(sys.argv[1]).read())
    n, seed, mode = int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
    rng = random.Random(seed)
    builds = [("nom", d)]
    for i in range(n):
        R = robust.tol_ranges()
        if mode == "real":
            R = dict(thickness=.10, width=.025, length=.02, hinge_radius=.02)
        f = {k: rng.uniform(1 - R[k], 1 + R[k]) for k in KEYS}
        up = {k: getattr(d.flexure, k) * v for k, v in f.items()}
        t = up["thickness"] * 1e3
        nl = 9
        if mode != "base":
            nl = round(9 * .05**3 / t**3)
            up["strip_laminae"] = nl
        fx = d.flexure.model_copy(update=up)
        builds.append((f"{i:2d} " + " ".join(f"{k[0]}{v:.3f}" for k, v in f.items()) + f" t={t:.4f} n={nl} stack={nl*t+.25:.3f}",
                       d.model_copy(update=dict(flexure=fx))))
    jobs = [(b, t, dv) for b, bd in builds for t, dv in robust.variants(bd, tol.TF, None, tol.JS, None)]
    with ProcessPoolExecutor(2) as ex:
        ms = list(ex.map(ev, [j[2] for j in jobs], chunksize=1))
    for b, _ in builds:
        sel = [(t, m) for (bb, t, _), m in zip(jobs, ms) if bb == b]
        c = robust.combine([m for _, m in sel], [t for t, _ in sel])
        vh = {k: round(v, 3) for k, v in (c.get("viol_hard") or {}).items()}
        print(f"{b:75s} cr_pl {c.get('cr_pl',0):.4f} res {c.get('res_margin',0):.3f} viol {vh}", flush=True)


main()
