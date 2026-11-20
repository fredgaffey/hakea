"""Robust (multi-throttle) evaluation: the same design at several thrust levels.

Knob: `op.vehicle_mass` scaled by the thrust factor (trim thrust = m g) with `op.rpm=None`
so rpm re-trims. control_ratio is moment/(W R) with W the trimmed thrust, so it is comparable
across points. The motor 1/rev torque capability is re-evaluated per point (it depends on rpm).
Cost = len(thrust_factors) * max(1, len(perturb)) evaluations per design.

`perturb`: optional list of {dotted.path: multiplier} dicts (e.g. +-10 % stiffness/inertia);
each is applied at every thrust factor, plus the nominal.
"""
from __future__ import annotations

import math
from typing import Callable

from .optimize import get_path, set_design

THRUST_FACTORS = (0.7, 1.0, 1.4)
PULSE_MARGIN = 0.95      # each throttle point pulses at 0.95 x motor capability at its own rpm


def variants(design, thrust_factors=THRUST_FACTORS, perturb=None):
    """-> list of (tag, Design)."""
    out = []
    pert = [("nom", {})] + [(f"p{i}", p) for i, p in enumerate(perturb or [])]
    for f in thrust_factors:
        for ptag, p in pert:
            d = design
            if p:
                d = set_design(d, {k: float(get_path(d, k)) * m for k, m in p.items()})
            if d.flexure.pre_lag_deg == "auto" and d.flexure.pre_lag_ref_mass is None:
                # pre-lag is built-in geometry: solve it once at the nominal thrust, not per point
                d = d.model_copy(update=dict(flexure=d.flexure.model_copy(
                    update=dict(pre_lag_ref_mass=design.op.vehicle_mass))))
            d = d.model_copy(update=dict(op=d.op.model_copy(
                update=dict(vehicle_mass=design.op.vehicle_mass * f, rpm=None,
                            pulse_capability_margin=PULSE_MARGIN))))
            out.append((f"T{f:g}" + ("" if ptag == "nom" else ":" + ptag), d))
    return out


def combine(ms: list[dict], tags: list[str]) -> dict:
    """Merge per-point metrics dicts: min control_ratio, max util/viol per key, min res_margin."""
    ok = [(t, m) for t, m in zip(tags, ms) if m and "control_ratio" in m]
    if not ok:
        return dict(error="no point evaluated")
    tags, ms = zip(*ok)
    i = min(range(len(ms)), key=lambda j: ms[j]["control_ratio"])
    out = dict(ms[i])
    out["control_ratio"] = float(min(m["control_ratio"] for m in ms))
    util: dict[str, float] = {}
    viol: dict[str, float] = {}
    for m in ms:
        for k, v in m.get("util", {}).items():
            util[k] = max(util.get(k, 0.0), v)
        for k, v in m.get("viol", {}).items():
            viol[k] = max(viol.get(k, 0.0), v)
    out["util"], out["viol"] = util, viol
    mg = [(m.get("res_margin", float("nan")), m.get("res_which", "")) for m in ms]
    mg = [x for x in mg if math.isfinite(x[0])]
    if mg:
        out["res_margin"], out["res_which"] = min(mg)
    out["feasible"] = not viol
    if all("binding" in m for m in ms):
        out["binding"] = max((m["binding"] for m in ms), key=lambda b: b[1])
    out["points"] = [dict(tag=t, control_ratio=m["control_ratio"], rpm=m.get("rpm"),
                          binding=m.get("binding"), res_margin=m.get("res_margin"),
                          feasible=m.get("feasible")) for t, m in zip(tags, ms)]
    return out


def map_metrics(pool, designs, metrics_fn: Callable, factors=None, perturb=None):
    """pool.map over designs -> list of (metrics | None, error). factors=None: single point."""
    if not factors:
        recs = pool.map(designs)
        return [(metrics_fn(d, r["results"]), None) if "results" in r
                else (None, r.get("error", "?")[:200]) for d, r in zip(designs, recs)]
    vs = [variants(d, factors, perturb) for d in designs]
    flat = [dv for v in vs for _, dv in v]
    recs = pool.map(flat)
    out, k = [], 0
    for d, v in zip(designs, vs):
        rr = recs[k:k + len(v)]
        k += len(v)
        bad = [r for r in rr if "results" not in r]
        if bad:
            out.append((None, bad[0].get("error", "?")[:200]))
            continue
        out.append((combine([metrics_fn(dv, r["results"]) for (_, dv), r in zip(v, rr)],
                            [t for t, _ in v]), None))
    return out


def evaluate_robust(design, thrust_factors=THRUST_FACTORS, perturb=None, metrics_fn=None) -> dict:
    """Local (serial) robust evaluation; metrics_fn defaults to study.run_study.metrics-style
    callable (design, results_dict) and must be supplied by the caller from the study modules."""
    from . import api
    assert metrics_fn is not None, "pass metrics_fn(design, results_dict)"
    vs = variants(design, thrust_factors, perturb)
    ms = [metrics_fn(d, api.evaluate(d).model_dump(mode="json")) for _, d in vs]
    return combine(ms, [t for t, _ in vs])
