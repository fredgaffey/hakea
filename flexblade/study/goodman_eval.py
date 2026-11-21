"""Goodman vs amplitude-only strip fatigue: cr_pl + binding check + mean/alt split (robust T x J).
PYTHONPATH=flexblade python flexblade/study/goodman_eval.py <design> [<design> ...] [--su1095 1.4e9]"""
import sys
sys.path.insert(0, __import__("pathlib").Path(__file__).parent.as_posix())
from concurrent.futures import ProcessPoolExecutor
import pulse_limited as pl
from flexblade import api, robust, rotor

argv = sys.argv[1:]
if "--su1095" in argv:
    i = argv.index("--su1095")
    rotor.STRIP_SU["steel_1095_shim"] = float(argv[i + 1])
    del argv[i:i + 2]
if __name__ == "__main__":
    with ProcessPoolExecutor(3) as ex:
        for name in argv:
            for mode in ("none", "goodman"):
                d = pl.load_design(name)
                d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(strip_fatigue_mean=mode))))
                r = pl.summary(pl.robust_eval([d], js=pl.JS, ex=ex)[0])
                worst = None
                for _, dv in robust.variants(d, robust.THRUST_FACTORS, None, pl.JS):
                    ch = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")["checks"]
                    for c in ch:
                        if c["name"] == "fatigue_stress" and (worst is None or c["utilisation"] > worst["utilisation"]):
                            worst = c
                print(name, mode, "cr_pl", round(r["cr_pl"] or 0, 4), "feas", r["feasible_pl"],
                      "bind", sorted({p["pulse_binding"] for p in r["points"]}),
                      "fat_s_util", round(worst["utilisation"], 3), worst["note"][-170:], flush=True)
