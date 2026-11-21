"""Families of ideal optima (7in, 10in; 4 seeds each) + their cr_pl on the current model."""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import numpy as np
import pulse_limited as pl
import ideal_gap as ig

HERE = Path(__file__).parent
SEEDS = ("seed1", "robust_seed1", "robust_seed2", "robust_seed3")


def feat(seed, D):
    c = json.loads((ig.SEEDS / seed / f"ideal_spec_{D}in.json").read_text())
    s, t, m = c["stiffness"], c["targets"], c["mass"]
    return c, [np.log10(t["k_cross_over_k_hinge"]), np.log10(t["k_flap_over_k_hinge"]),
               np.log10(t["k_lag_over_k_flap"]), s["skew_deg"] / 45.0, m["hinge_radius"] / (D * 0.0254 / 2) * 10]


if __name__ == "__main__":
    out = {}
    for D in (7, 10):
        ds = {s: ig.ideal(s, D) for s in SEEDS}
        cs = pl.robust_eval(list(ds.values()))
        F = np.array([feat(s, D)[1] for s in SEEDS])
        Z = (F - F.mean(0)) / (F.std(0) + 1e-9)
        # 2-means, farthest-pair init
        dist = np.linalg.norm(Z[:, None] - Z[None], axis=2)
        i, j = np.unravel_index(dist.argmax(), dist.shape)
        cen = Z[[i, j]]
        for _ in range(20):
            lab = np.linalg.norm(Z[:, None] - cen[None], axis=2).argmin(1)
            cen = np.array([Z[lab == k].mean(0) if (lab == k).any() else cen[k] for k in range(2)])
        print(f"== {D} in   features: log kc/kh, log kf/kh, log klag/kf, skew/45, R/Rtip*10")
        for s, f, c_, l in zip(SEEDS, F, cs, lab):
            cc = feat(s, D)[0]
            print(s, "fam", l, np.round(f, 2).tolist(), "own_cr %.3f" % cc["control_ratio"], "cr_pl %.3f" % c_["cr_pl"],
                  c_.get("res_which"), flush=True)
            out[f"{D}_{s}"] = dict(fam=int(l), feat=f.tolist(), own_cr=cc["control_ratio"], cr_pl=c_["cr_pl"])
        print("pairwise normalised dist:\n", np.round(dist, 2))
    (HERE / "results" / "ideal_gap_families.json").write_text(json.dumps(out, indent=1, default=float))
