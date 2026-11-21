"""Tilt Floquet scan of the XY flexure-stage mount (build/mount_xy.py) on the mounts_tilt.py grid.
k_tilt derated 3x from the analytic blade value (clamp/FDM-frame compliance).  Mid-frame mass: +5 g on the mount mass.
Run from flexblade/ with PYTHONPATH=$PWD:$PWD/study:$PWD/study/build.  -> results/mounts_tilt_xy.json"""
import json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import mounts_tilt as MTT
import mount_xy as MX
import tilt_whirl as TW

import os
DERATE = float(os.environ.get("XY_DERATE", 3.0))
KS = float(os.environ.get("XY_KSCALE", 1.0))
DM = float(os.environ.get("XY_DM", 0.005))


def mk(name):
    P = MX.SIZES[name]
    p = MX.props(**P)
    t = 5.0   # mm equivalent shear gap
    ro = float(np.sqrt(P["k_lat"] * t * 1e-3 / (MX.INF * 6.5e6 * np.pi)) * 1e3)
    eta = (MX.ETA_PAD * p["k_pad"] + MX.ETA_BL * p["kb"]) / P["k_lat"]
    return dict(G_mpa=6.5, eta=float(eta), mass_motor_kg=P["m"] + DM, k_tilt=p["kt"] / DERATE,
                r_in_mm=1e-6, r_out_mm=ro, length_mm=t, infill=MX.INF * KS)


if __name__ == "__main__":
    MTT.mk = mk
    names = list(MX.SIZES)
    jobs = [(n, T, J) for n in names for T in (0.7, 1.0, 1.4) for J in (2.5e-6, 3.76e-6, 5.49e-6)]
    with ProcessPoolExecutor(4) as ex:
        res = list(ex.map(MTT.job, jobs))
    for n in names:
        r = [x for x in res if x["design"] == n]
        print(n, "kt %.1f" % r[0]["kt"], "rule2 %.1f" % r[0]["rule2"], "eta %.4f" % mk(n)["eta"])
        for ix in (1, 2, 3):
            print(" It x%d" % ix, {fa: round(min(x[f"z_{fa:g}_It{ix}"] for x in r), 4) for fa in MTT.ARMS})
        hs = [v for x in r for k, v in x.items() if k.startswith("hub_")]
        print(" hub f/zeta", min(h[0] for h in hs), max(h[0] for h in hs), min(h[1] for h in hs), max(h[1] for h in hs), "1P(min)", min(x["f1p"] for x in r))
    (TW.HERE / "results" / "mounts_tilt_xy.json").write_text(json.dumps(res, indent=1, default=float))
