"""Hinge stiffness decomposition: ideal card vs realised steelx (current model)."""
import sys, json, math
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import numpy as np
import pulse_limited as pl
import ideal_gap as ig
from flexblade import api, fact


def dec(d, K):
    K = np.asarray(K, float)
    _, a = fact.hinge_axis(d, 1)
    y = np.array([0, 1., 0])
    b = np.cross(a, y)
    C = np.linalg.inv(K)[3:, 3:]
    kh, kc, kf = (1 / (v @ C @ v) for v in (a, b, y))
    Kr = K[3:, 3:]
    w, v = np.linalg.eigh(0.5 * (C + C.T))
    s = v[:, -1]
    return dict(k_hinge=kh, k_cross=kc, k_flap=kf, skew_soft=math.degrees(math.atan2(abs(s[0]), abs(s[2]))),
                skew_design=d.flexure.skew_deg, k_lag=Kr[2, 2], k_pitch=Kr[0, 0], k_lp=Kr[0, 2], k_flapK=Kr[1, 1],
                kx=K[0, 0], ky=K[1, 1], kz=K[2, 2], R=d.flexure.hinge_radius)


def realised_K(d):
    r = api.evaluate(d, ["trim", "checks"]).model_dump(mode="json")
    return r["flexure_stiffness"]


if __name__ == "__main__":
    out = {}
    for n, d in (("7_v6", pl.load_design("7in_tt_strap_steelx_v6")), ("10_v4", pl.load_design("10in_tt_strap_steelx_v4")),
                 ("7_ideal", ig.ideal("robust_seed2", 7)), ("10_ideal", ig.ideal("robust_seed2", 10))):
        K = fact.ideal_K6(d) if d.flexure.topology == "ideal" else realised_K(d)
        out[n] = dict(K6=np.asarray(K).tolist(), **dec(d, K))
        print(n, {k: float(f"{v:.4g}") for k, v in out[n].items() if k != "K6"})
        if d.flexure.topology == "ideal":
            ip = d.flexure.ideal
            print("   loss", ip.loss_eta, "tip", ip.tip_mass, "lagI", ip.lag_inertia, "ea", d.blade.elastic_axis_chord_frac)
        else:
            print("   ", d.flexure.topology, d.flexure.skew_deg, d.blade.elastic_axis_chord_frac)
    json.dump(out, open(Path(__file__).parent / "results" / "ideal_gap_K.json", "w"), indent=1)
