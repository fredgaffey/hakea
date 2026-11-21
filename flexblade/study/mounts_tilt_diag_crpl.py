"""cr_pl cost of the blade knob in mounts_tilt_diag (flexure thickness x1.15) vs 7in_v6: robust pulse_limited eval.
Run from the repo root with PYTHONPATH=flexblade.   -> study/results/mounts_tilt_diag_crpl.json"""
import json
import pulse_limited as pl
import mounts_tilt_diag as D

if __name__ == "__main__":
    ds = {"v6": pl.load_design("7in_tt_strap_steelx_v6"), "t1.15": D.variant_design(dict(thickness=1.15)),
          "t1.08": D.variant_design(dict(thickness=1.08))}
    cs = pl.robust_eval(list(ds.values()))
    out = {k: {kk: v for kk, v in pl.summary(c).items() if kk != "points"} for k, c in zip(ds, cs)}
    (pl.HERE / "results" / "mounts_tilt_diag_crpl.json").write_text(json.dumps(out, indent=1, default=float))
    for k, v in out.items():
        print(k, v, flush=True)
