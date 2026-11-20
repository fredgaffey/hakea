"""10" tt_strap_steelx v3: the 7" v3 recipe scaled to 10", sized against the pitch CF
divergence criterion (rotor.pitch_cf → summary.pitch_cf_ratio).

10" v2 had no valid trim: the blade collapsed to ~−20° nose-down twist (thrust stuck ~3 N).
Everything that resists or drives the blade pitch through the skewed hinge scales with Ω²:
the propeller moment (k_prop = Ω² Σm(y²−z²) → flat pitch), the aero pitch stiffness
(EA aft of the AC: destabilising), the CF lag stiffness seen through the skew (stabilising),
and the strap's geometric stiffness (a CF-tensioned strap whose blade anchor sits outboard
of the hinge acts like a pushed column, T·Δ(Δ/L_strap − 1) < 0). v2's long 12 mm strap and
thin 0.08×10 mm strips left the loaded flexure with negative hinge stiffness → divergence.

    python study/steelx_10in_v3.py [--trim] [--robust] [--draw] [key=value ...]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from flexblade import api, robust                      # noqa: E402
from flexblade.optimize import set_design              # noqa: E402
from flexblade.schema import BalanceMass, Design       # noqa: E402

V2 = HERE / "designs" / "10in_tt_strap_steelx_v2.json"
OUT = HERE / "designs" / "10in_tt_strap_steelx_v3.json"
# 7" v3 recipe scaled to 10": standard 0.15 mm 1095 shim, strips scaled ~D (8→10 mm wide,
# 24→36 mm free length; 12 mm wide lifts the pitch-sym mode off 3P), open 130° cross, skew 65°, hinge radius ∝ hub, short strap.
RECIPE = {"flexure.thickness": 0.00015, "flexure.width": 0.012, "flexure.length": 0.036,
          "flexure.cross_angle_deg": 130.0, "flexure.skew_deg": 65.0,
          "flexure.hinge_radius": 0.022, "flexure.strap_strand_diameter": 0.0022,
          "flexure.strap_length": 0.004, "flexure.strap_pin_diameter": 0.0088,
          "blade.elastic_axis_chord_frac": 0.38}
# "Chinese weight": steel pin pair, 1 g each at r = 0.2R, ±10 mm normal to the chord
BALANCE: list[dict] = [dict(mass=0.001, r_frac=0.2, normal_offset=0.01),
                       dict(mass=0.001, r_frac=0.2, normal_offset=-0.01)]
KEYS = ("rpm", "trim_ok", "pitch_cf_ratio", "pitch_k_prop", "pitch_k_eff", "pitch_k_aero",
        "pitch_k_struct", "hinge_k_free", "trim_diff_pitch_deg")


def build(extra: dict | None = None, balance: list[dict] | None = None) -> Design:
    d = Design.model_validate_json(V2.read_text())
    vals = {**RECIPE, **(extra or {})}
    d = set_design(d, {k: v for k, v in vals.items()})
    bal = BALANCE if balance is None else balance
    d = d.model_copy(update=dict(
        name="10in_tt_strap_steelx_v3",
        blade=d.blade.model_copy(update=dict(balance_masses=[BalanceMass(**b) for b in bal])),
        flexure=d.flexure.model_copy(update=dict(pre_lag_deg="auto", pre_lag_axis="pitch")),
        notes=d.notes.split("; v3")[0] + "; v3 (7\" v3 recipe scaled, pitch_cf_ratio sized): "
        + json.dumps({**vals, "balance_masses": bal})))
    return d


def _trim(dv: Design) -> dict:
    r = api.evaluate(dv, ["trim"])
    s = {k: float(r.summary[k]) for k in KEYS if k in r.summary}
    s["thrust"] = float(getattr(r.trim, "thrust", float("nan")))
    return s


def trims(d: Design) -> dict:
    vs = robust.variants(d)
    with ProcessPoolExecutor(3) as ex:
        return dict(zip([t for t, _ in vs], ex.map(_trim, [v for _, v in vs])))


def _full(dv: Design) -> dict:
    import run_study as R
    return R.metrics(dv, api.evaluate(dv).model_dump(mode="json"))


def robust_eval(d: Design) -> dict:
    vs = robust.variants(d)
    with ProcessPoolExecutor(3) as ex:
        ms = list(ex.map(_full, [v for _, v in vs]))
    out = robust.combine(ms, [t for t, _ in vs])
    out["per_point"] = {t: dict(cr=m.get("control_ratio"), feasible=m.get("feasible"),
                                binding=m.get("binding"), viol=m.get("viol"))
                        for (t, _), m in zip(vs, ms)}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--trim", action="store_true")
    ap.add_argument("--robust", action="store_true")
    ap.add_argument("--draw", action="store_true")
    ap.add_argument("--save", action="store_true")
    ap.add_argument("--balance", default=None, help="JSON list of balance masses")
    ap.add_argument("kv", nargs="*")
    a = ap.parse_args(argv)
    extra = {k: (float(v) if v.replace(".", "", 1).replace("e-", "", 1).isdigit() else v)
             for k, v in (s.split("=", 1) for s in a.kv)}
    d = build(extra, json.loads(a.balance) if a.balance else None)
    if a.trim:
        print(json.dumps(trims(d), default=float))
    if a.robust:
        print(json.dumps(robust_eval(d), default=float))
    if a.save:
        OUT.write_text(d.model_dump_json(indent=1))
    if a.draw:
        from flexblade import drawing
        drawing.sheet(d, str(HERE / "results" / "steelx_10in_v3.png"))


if __name__ == "__main__":
    main()
