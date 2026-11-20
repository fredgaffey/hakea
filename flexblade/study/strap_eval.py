"""Tension-torsion strap hand designs: skewed cruciform (axially released) + unbonded strand
strap carrying the centrifugal load. Evaluates stiffness vs the ideal card, control_ratio,
checks and binding constraint; writes study/designs/<D>in_tt_strap_hand.json.

    python study/strap_eval.py [--cards DIR] [--robust] [--sizes 5,7]
        [--printed cruciform|cross_blade|cross_blade_sym]

--printed cross_blade[_sym]: strap + axially released thin crossed plates (cross-axis pivot,
concept tt_strap_xplate): plates printed with build direction along the hinge axis, one short
aramid strand on the crossing line; writes study/designs/<D>in_tt_strap_xplate_hand.json.
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


# strap + released crossed plates (tt_strap_xplate); geometry scales with D/5 except thickness
XPLATE = {D: {"flexure.thickness": 0.0006, "flexure.width": min(0.025 * D / 5, 0.04),
              "flexure.length": 0.008 * D / 5, "flexure.cross_angle_deg": 70.0,
              "flexure.cross_gap": 0.0012 * D / 5, "flexure.strap_strands": 1,
              "flexure.strap_strand_diameter": 0.0007 * D / 5, "flexure.strap_radius": 0.0,
              "flexure.strap_length": 0.004 * D / 5, "flexure.strap_material": "aramid_yarn",
              "flexure.strap_anchor": "pin_wrap", "flexure.strap_pin_diameter": 0.003 * D / 5,
              "flexure.axial_release": True, "flexure.cruciform_rod_diameter": 0.0}
          for D in (5.0, 7.0, 10.0)}
for _D, _sk in ((5.0, 55.9), (7.0, 69.2), (10.0, 66.0)):     # ideal-card skew
    XPLATE[_D]["flexure.skew_deg"] = _sk
TOPO = {"cruciform": "skewed_cruciform", "cross_blade": "skewed_cross_blade",
        "cross_blade_sym": "skewed_cross_blade_sym"}


def xplate_design(D: float, printed: str = "cross_blade", extra: dict | None = None) -> Design:
    """Strap + axially released crossed plates. Plates are printed with the build direction
    along the hinge axis a (layers ⟂ a, rasters along x' → plate bending stress in-layer)."""
    base, _ = R.base_design(D, "tt_strap_xplate")
    d = apply_strap(base, {**XPLATE[D], **(extra or {})}, f"{int(D)}in_tt_strap_xplate_hand",
                    topology=TOPO[printed])
    _, a, xp, _ = fact._frame(d)
    ori = d.flexure.orientation.model_copy(update=dict(build_dir=tuple(map(float, a)),
                                                       raster_dir=tuple(map(float, xp))))
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(orientation=ori))))


def strap_design(D: float, extra: dict | None = None) -> Design:
    base = Design.model_validate_json((HERE / "designs" / f"{int(D)}in_skewed_cruciform_hand.json").read_text())
    return apply_strap(base, {**HAND[D], **(extra or {})}, f"{int(D)}in_tt_strap_hand")


def apply_strap(base: Design, vals: dict, name: str | None = None,
                topology: str = "skewed_cruciform") -> Design:
    """set_design for numeric paths; str/bool/int flexure fields set directly (validated)."""
    num = {k: v for k, v in vals.items() if isinstance(v, float)}
    other = {k.split(".", 1)[1]: v for k, v in vals.items() if not isinstance(v, float)}
    d = set_design(base, num)
    fx = d.flexure.model_dump()
    fx.update(other, topology=topology, members=[])
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
    ap.add_argument("--printed", default="cruciform", choices=list(TOPO))
    a = ap.parse_args(argv)
    for D in (float(x) for x in a.sizes.split(",")):
        d = strap_design(D) if a.printed == "cruciform" else xplate_design(D, a.printed)
        p = Path(a.cards) / f"ideal_spec_{int(D)}in.json"
        card = json.loads(p.read_text()) if p.exists() else None
        res = evaluate(d, card, a.robust)
        (HERE / "designs" / f"{d.name}.json").write_text(d.model_dump_json(indent=1))
        print(json.dumps({"D": D, **res}, default=float, indent=1))


if __name__ == "__main__":
    main()
