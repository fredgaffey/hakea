"""Tension-torsion strap hand designs: skewed cruciform (axially released) + unbonded strand
strap carrying the centrifugal load. Evaluates stiffness vs the ideal card, control_ratio,
checks and binding constraint; writes study/designs/<D>in_tt_strap_hand.json.

    python study/strap_eval.py [--cards DIR] [--robust] [--sizes 5,7]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import ideal_spec as IS                                 # noqa: E402
import run_study as R                                   # noqa: E402
from flexblade import api, fact, robust                    # noqa: E402
from flexblade.optimize import set_design               # noqa: E402
from flexblade.schema import Design                     # noqa: E402

# hand-picked strap parameters per size (scaled from 5")
HAND = {
    5.0: {"flexure.strap_strands": 4, "flexure.strap_strand_diameter": 0.0006,
          "flexure.strap_radius": 0.0007, "flexure.strap_material": "aramid_yarn",
          "flexure.strap_length": 0.0, "flexure.strap_anchor": "pin_wrap",
          "flexure.strap_pin_diameter": 0.003, "flexure.axial_release": True,
          "flexure.cruciform_rod_diameter": 0.0},
    7.0: {"flexure.strap_strands": 4, "flexure.strap_strand_diameter": 0.0008,
          "flexure.strap_radius": 0.0006, "flexure.strap_material": "aramid_yarn",
          "flexure.strap_length": 0.0, "flexure.strap_anchor": "pin_wrap",
          "flexure.strap_pin_diameter": 0.004, "flexure.axial_release": True,
          "flexure.cruciform_rod_diameter": 0.0},
}


def strap_design(D: float, extra: dict | None = None) -> Design:
    base = Design.model_validate_json((HERE / "designs" / f"{int(D)}in_skewed_cruciform_hand.json").read_text())
    return apply_strap(base, {**HAND[D], **(extra or {})}, f"{int(D)}in_tt_strap_hand")


def apply_strap(base: Design, vals: dict, name: str | None = None) -> Design:
    """set_design for numeric paths; str/bool/int flexure fields set directly (validated)."""
    num = {k: v for k, v in vals.items() if isinstance(v, float)}
    other = {k.split(".", 1)[1]: v for k, v in vals.items() if not isinstance(v, float)}
    d = set_design(base, num)
    fx = d.flexure.model_dump()
    fx.update(other, topology="skewed_cruciform", members=[])
    return d.model_copy(update=dict(name=name or d.name, flexure=type(d.flexure).model_validate(fx)))


def evaluate(d: Design, card: dict | None, use_robust: bool = False) -> dict:
    r = api.evaluate(d).model_dump(mode="json")
    m = IS.metrics(d, r)
    out = dict(control_ratio=m["control_ratio"], feasible=m["feasible"], binding=m["binding"],
               viol=m["viol"], lag_per_rev=m["lag_per_rev"],
               strap={c["name"]: round(c["utilisation"], 3) for c in r["checks"] if c["member"] == "strap"},
               strap_creep=max((c["utilisation"] for c in r["checks"] if c["name"] == "creep_strain"
                                and c["member"] in {f"flex{j}" for j, mm in enumerate(fact.generate_members(d))
                                                    if mm.cable}), default=None))
    if card is not None and r.get("flexure_stiffness"):
        dist = IS.distance(r["flexure_stiffness"], card)
        out["distance"] = dist
        t = card["targets"]
        out["ratios"] = {k: 10 ** v * t[k] for k, v in dist["terms"].items()
                         if k in ("k_hinge", "k_cross_over_k_hinge", "k_flap_over_k_hinge")}
        out["ideal_control_ratio"] = card["control_ratio"]
    if use_robust:
        rb = robust.evaluate_robust(d, metrics_fn=IS.metrics)
        out["robust_control_ratio"] = rb.get("control_ratio")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--cards", default=str(HERE / "results"))
    ap.add_argument("--sizes", default="5,7")
    ap.add_argument("--robust", action="store_true")
    a = ap.parse_args(argv)
    for D in (float(x) for x in a.sizes.split(",")):
        d = strap_design(D)
        p = Path(a.cards) / f"ideal_spec_{int(D)}in.json"
        card = json.loads(p.read_text()) if p.exists() else None
        res = evaluate(d, card, a.robust)
        (HERE / "designs" / f"{int(D)}in_tt_strap_hand.json").write_text(d.model_dump_json(indent=1))
        print(json.dumps({"D": D, **res}, default=float, indent=1))


if __name__ == "__main__":
    main()
