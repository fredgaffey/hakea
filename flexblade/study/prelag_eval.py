"""Pre-lag evaluation: pre_lag off vs auto at thrust factors 0.7/1.0/1.4 -> study/results/prelag_eval.md.
k/K_cf: K_cf from the auto-solve secant slope (slope = 2 sin(s) k/(k+K_cf)) at the reference op,
scaled by (Omega/Omega_ref)^2."""
import json, math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from flexblade import api, robust
from flexblade.schema import Design
import run_study as rs

HERE = Path(__file__).parent
NAMES = ["5in_tt_strap_hand", "5in_tt_strap_steelx_hand", "7in_tt_strap_steelx_hand", "10in_tt_strap_steelx_hand"]
NAN = float("nan")


def main():
    rows, lines = [], []
    for n in NAMES:
        d0 = Design.model_validate(json.load(open(HERE / "designs" / f"{n}.json")))
        ref = {}
        for mode in ("off", "auto"):
            fl = d0.flexure.model_copy(update=dict(pre_lag_deg=0.0 if mode == "off" else "auto"))
            d = d0.model_copy(update=dict(flexure=fl))
            ms = []
            for tag, v in robust.variants(d):
                try:
                    r = api.evaluate(v).model_dump(mode="json")
                    m = rs.metrics(v, r)
                    s = r["summary"]
                    m.update(split=s.get("trim_diff_pitch_deg", NAN), lag=s.get("hinge_lag_el_deg", NAN),
                             pre=s.get("pre_lag_deg", 0.0), slope=s.get("pre_lag_split_slope"), Om=s.get("Omega"),
                             kh=s.get("hinge_k_free"), tag=tag)
                except Exception as ex:
                    m = dict(tag=tag, error=str(ex)[:80], split=NAN, lag=NAN, pre=0.0, control_ratio=NAN,
                             feasible=False, viol={}, util={}, slope=None, Om=None, kh=None)
                ms.append(m)
            ref[mode] = ms
        skew = math.radians(d0.flexure.skew_deg)
        a1 = ref["auto"][1]
        rr = None
        if a1.get("slope") is not None and math.isfinite(a1["slope"]):
            r_ = a1["slope"] / (2 * math.sin(skew))
            if 0 < r_ < 1:
                rr = r_ / (1 - r_)
        for i, f in enumerate(robust.THRUST_FACTORS):
            o, a = ref["off"][i], ref["auto"][i]
            kk = rr * (a1["Om"] / a["Om"]) ** 2 if rr is not None and a.get("Om") and a1.get("Om") else NAN
            bind = max(a.get("util", {}).items(), key=lambda kv: kv[1], default=("", 0))
            bv = ",".join(sorted(a.get("viol", {}))) or "-"
            rows.append(f"| {n} | {f:g} | {o['split']:+.2f} | {a['split']:+.2f} | {o['lag']:+.2f} | {a['lag'] + a['pre']:+.2f} "
                        f"({a['pre']:+.2f} pre) | {o['control_ratio']:.3f} | {a['control_ratio']:.3f} | "
                        f"{'Y' if o['feasible'] else 'N'}/{'Y' if a['feasible'] else 'N'} | {bind[0]} {bind[1]:.2f}; viol {bv} | {kk:.3f} |")
            print(rows[-1], flush=True)
        rb = {m: min((x['control_ratio'] for x in ref[m]), default=NAN) for m in ref}
        fe = {m: all(x['feasible'] for x in ref[m]) for m in ref}
        lines.append(f"- {n}: robust control_ratio off {rb['off']:.3f} / auto {rb['auto']:.3f}; robust feasible "
                     f"off {fe['off']} / auto {fe['auto']}; k_hinge {a1.get('kh')}; pre-lag {a1['pre']:+.2f} deg")
    out = ["# Pre-lag evaluation (off vs auto)", "",
           "Split = trim_diff_pitch_deg (limit 2 deg). Lag = mean elastic hinge lag deg (auto column: elastic + pre-lag). "
           "k/K_cf from secant slope at T1.0, scaled by (Omega/Omega_ref)^2 (K_cf ~ Omega^2).", "",
           "| design | thrust | split off | split auto | lag off | lag auto | CR off | CR auto | feas off/auto | binding (auto) | k/K_cf |",
           "|---|---|---|---|---|---|---|---|---|---|---|"] + rows + ["", "Robust summary:"] + lines
    (HERE / "results" / "prelag_eval.md").write_text("\n".join(out) + "\n")


main()
