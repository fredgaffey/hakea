"""Tilt Floquet scan of the planar pad-ring mount (mount_ring.py) on the same T x J x arm grid as mounts_tilt.py.
Run from flexblade/ with PYTHONPATH=$PWD:$PWD/study:$PWD/study/build.  -> results/mounts_tilt_ring.json"""
import json
from concurrent.futures import ProcessPoolExecutor
import mounts_tilt as MTT
import mount_ring as MR
import tilt_whirl as TW


def mk(name):
    P = MR.SIZES[name]
    m = MTT.SIZES[name][1]
    return dict(G_mpa=6.5, eta=0.15, mass_motor_kg=m, k_tilt=MR.props(**P)["k_t"], **MR.mount_kwargs(**P))


if __name__ == "__main__":
    MTT.mk = mk
    jobs = [(n, T, J) for n in MR.SIZES for T in (0.7, 1.0, 1.4) for J in (2.5e-6, 3.76e-6, 5.49e-6)]
    with ProcessPoolExecutor(4) as ex:
        res = list(ex.map(MTT.job, jobs))
    for n in MR.SIZES:
        r = [x for x in res if x["design"] == n]
        print(n, "kt %.1f" % r[0]["kt"], "rule1 %.1f rule2 %.1f" % (r[0]["rule1"], r[0]["rule2"]))
        for ix in (1, 2, 3):
            print(" It x%d" % ix, {fa: round(min(x[f"z_{fa:g}_It{ix}"] for x in r), 4) for fa in MTT.ARMS})
        hs = [v for x in r for k, v in x.items() if k.startswith("hub_")]
        print(" hub f/zeta range", min(h[0] for h in hs), max(h[0] for h in hs), min(h[1] for h in hs), max(h[1] for h in hs),
              "1P(min)", min(x["f1p"] for x in r))
    (TW.HERE / "results" / "mounts_tilt_ring.json").write_text(json.dumps(res, indent=1, default=float))
