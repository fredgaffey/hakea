"""I32: ground-resonance arm band for 10in_v5 + v5c twin mount (arm f 50-350 Hz, T0.7/1/1.4, I_t x1/x2) and
cantilever tube sizing.  Run from flexblade/ with PYTHONPATH=$PWD:$PWD/study:$PWD/study/build.
-> results/mounts_tilt_diag10.json"""
import json, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import mounts_tilt as MTS
import mounts_tilt_diag as D
import tilt_whirl as TW

D.MNT = MTS.mk("10in_v5")
FN = "10in_tt_strap_steelx_v5"
FA = list(range(50, 351, 10))


def job(a):
    T, J = a
    C = D.setup(T, J, fn=FN)
    out = dict(T=T, J=J, f1p=C["ss"].Omega / 2 / np.pi)
    for fa in FA:
        for ix in (1.0, 2.0):
            out[f"{fa}_{ix:g}"] = D.solve(C, D.af_of(C, float(fa), ix=ix))
    return out


def tube(L, mtip=0.126, E=70e9, rho=1600.0):
    rows = []
    for od, idd in ((12, 10), (14, 12), (16, 14), (18, 16), (20, 18), (22, 20), (25, 23)):
        I = np.pi / 64 * ((od * 1e-3) ** 4 - (idd * 1e-3) ** 4)
        mt = rho * np.pi / 4 * ((od * 1e-3) ** 2 - (idd * 1e-3) ** 2) * L
        f = np.sqrt(3 * E * I / L ** 3 / (mtip + 0.24 * mt)) / 2 / np.pi
        rows.append((od, idd, round(float(f), 0), round(mt * 1e3, 1)))
    return rows


if __name__ == "__main__":
    jobs = [(T, J) for T in (0.7, 1.0, 1.4) for J in (2.5e-6, 3.76e-6, 5.49e-6)]
    with ProcessPoolExecutor(4) as ex:
        res = list(ex.map(job, jobs))
    (TW.HERE / "results" / "mounts_tilt_diag10.json").write_text(json.dumps(res, indent=1, default=float))
    for T in (0.7, 1.0, 1.4):
        rr = [r for r in res if r["T"] == T]
        print("T", T, "1P %.0f" % rr[0]["f1p"], " ".join("%d:%.3f" % (fa, min(r[f"{fa}_{ix:g}"] for r in rr for ix in (1.0, 2.0))) for fa in FA))
    for L in (0.18, 0.20, 0.22):
        print("L", L, tube(L))
