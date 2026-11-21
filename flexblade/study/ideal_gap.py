"""Ideal-vs-realised gap on the current model (pulse-limited, J_hub robust). See results/ideal_gap.md."""
import json, sys
from pathlib import Path
HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import pulse_limited as pl
from flexblade.schema import Design
SEEDS = HERE / "campaign" / "ideal_seeds"


def ideal(seed, D):
    return Design.model_validate_json((SEEDS / seed / f"ideal_{D}in.json").read_text())


if __name__ == "__main__":
    ds = {"7_robust_seed2": ideal("robust_seed2", 7), "7_robust_seed1": ideal("robust_seed1", 7),
          "10_robust_seed1": ideal("robust_seed1", 10), "10_robust_seed2": ideal("robust_seed2", 10),
          "7_v6": pl.load_design("7in_tt_strap_steelx_v6"), "10_v4": pl.load_design("10in_tt_strap_steelx_v4")}
    cs = pl.robust_eval(list(ds.values()))
    out = {n: pl.summary(c) for n, c in zip(ds, cs)}
    pl.res_save("ideal_gap_score", out)
    for n, s in out.items():
        print(n, round(s["cr_pl"], 4), s["feasible_pl"], s["viol_hard"], s["res_which"])
        for p in s["points"]:
            print("   ", p)
