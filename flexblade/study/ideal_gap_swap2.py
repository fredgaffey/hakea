"""Beyond-hinge swaps (blade / mass) for the 7in ideal card -> v6 equivalents."""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import numpy as np
import pulse_limited as pl
import ideal_gap as ig
from ideal_gap_swap import with_ideal, with_flex
from flexblade import fittings

HERE = Path(__file__).parent
if __name__ == "__main__":
    K = json.loads((HERE / "results" / "ideal_gap_K.json").read_text())["7_v6"]
    real = pl.load_design("7in_tt_strap_steelx_v6")
    d0 = ig.ideal("robust_seed2", 7)
    bi, br = d0.blade.model_dump(), real.blade.model_dump()
    print("blade fields differing:", {k: (bi[k], br[k]) for k in bi if bi[k] != br[k] and not isinstance(bi[k], (dict, list))})
    print("blade dict/list differ:", [k for k in bi if bi[k] != br[k] and isinstance(bi[k], (dict, list))])
    oi, orr = d0.op.model_dump(), real.op.model_dump()
    print("op differ:", {k: (oi[k], orr[k]) for k in oi if oi[k] != orr[k]})
    mi, mr = d0.motor.model_dump(), real.motor.model_dump()
    print("motor differ:", {k: (mi[k], mr[k]) for k in mi if mi[k] != mr[k]})
    m, c, I = fittings.fitting_mass_props(real)["root"]
    Izz = float(I[2, 2] + m * (c[0] - real.flexure.hinge_radius) ** 2)
    ea = real.blade.elastic_axis_chord_frac
    base = with_ideal(d0, K6=K["K6"], mass=m, lag_inertia=Izz)
    cases = {
        "ea->v6": with_flex(d0, hinge_radius=d0.flexure.hinge_radius).model_copy(update=dict(
            blade=d0.blade.model_copy(update=dict(elastic_axis_chord_frac=ea)))),
        "tip_mass->0": with_ideal(d0, tip_mass=0.0, tip_mass_chord=0.0, lag_inertia=0.0),
        "K6+fitting": base,
        "K6+fitting+ea+tip0": with_ideal(base.model_copy(update=dict(blade=d0.blade.model_copy(
            update=dict(elastic_axis_chord_frac=ea)))), tip_mass=0.0, tip_mass_chord=0.0),
        "K6+fitting+R(v6)+skew(v6)+ea+tip0": with_flex(with_ideal(base.model_copy(update=dict(blade=d0.blade.model_copy(
            update=dict(elastic_axis_chord_frac=ea)))), tip_mass=0.0, tip_mass_chord=0.0),
            hinge_radius=K["R"], skew_deg=K["skew_design"]),
    }
    cs = pl.robust_eval(list(cases.values()))
    for n, c_ in zip(cases, cs):
        print(n, round(c_["cr_pl"], 4), c_.get("res_which"), round(c_.get("res_margin") or 0, 3), c_.get("viol_hard"),
              [round(p["cr_pl"], 3) for p in c_["points"][-3:]], flush=True)
