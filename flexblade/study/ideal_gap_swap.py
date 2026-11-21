"""Single-property swaps: realised (v6) hinge property -> 7in ideal card; cr_pl on the current model."""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import numpy as np
import pulse_limited as pl
import ideal_gap as ig
from flexblade import fittings
from flexblade.schema import Design

HERE = Path(__file__).parent


def with_ideal(d, **kw):
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(
        update=dict(ideal=d.flexure.ideal.model_copy(update=kw)))))


def with_flex(d, **kw):
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=kw)))


if __name__ == "__main__":
    seed = sys.argv[1] if len(sys.argv) > 1 else "robust_seed2"
    D = int(sys.argv[2]) if len(sys.argv) > 2 else 7
    rn = "7in_tt_strap_steelx_v6" if D == 7 else "10in_tt_strap_steelx_v4"
    K = json.loads((HERE / "results" / "ideal_gap_K.json").read_text())
    r = K["7_v6" if D == 7 else "10_v4"]
    real = pl.load_design(rn)
    fp = fittings.fitting_mass_props(real)
    m, c, I = fp["root"]
    mh, ch, Ih = fp["hub"]
    Izz_root = float(I[2, 2] + m * (c[0] - real.flexure.hinge_radius) ** 2 + m * c[1] ** 2)
    print("root fitting", m * 1e3, "g com_x-R mm", (c[0] - real.flexure.hinge_radius) * 1e3, "Izz", Izz_root,
          "hub", mh * 1e3, "g; v6 strip_lam_eta", real.flexure.strip_lam_eta)
    d0 = ig.ideal(seed, D)
    Kfull = np.asarray(r["K6"])
    cases = {
        "ideal": d0,
        "k_hinge": with_ideal(d0, k_hinge=r["k_hinge"]),
        "k_cross": with_ideal(d0, k_cross=r["k_cross"]),
        "k_flap": with_ideal(d0, k_flap=r["k_flap"]),
        "k_cross+k_flap": with_ideal(d0, k_cross=r["k_cross"], k_flap=r["k_flap"]),
        "skew": with_flex(d0, skew_deg=r["skew_design"]),
        "translations": with_ideal(d0, k_axial=r["kx"], k_inplane=r["ky"], k_outplane=r["kz"]),
        "hinge_radius": with_flex(d0, hinge_radius=r["R"]),
        "loss": with_ideal(d0, loss_eta=real.flexure.strip_lam_eta),
        "fitting_mass": with_ideal(d0, mass=m, lag_inertia=d0.flexure.ideal.lag_inertia + Izz_root),
        "full_K6(v6)": with_ideal(d0, K6=Kfull.tolist()),
    }
    cs = pl.robust_eval(list(cases.values()))
    out = {}
    for n, c_ in zip(cases, cs):
        out[n] = dict(cr_pl=c_["cr_pl"], res_margin=c_.get("res_margin"), res_which=c_.get("res_which"),
                      viol_hard=c_.get("viol_hard"), feas=c_.get("feasible_pl"))
        print(n, round(c_["cr_pl"], 4), c_.get("res_which"), round(c_.get("res_margin") or 0, 3),
              c_.get("viol_hard"), flush=True)
    (HERE / "results" / f"ideal_gap_swap_{D}in.json").write_text(json.dumps(out, indent=1, default=float))
