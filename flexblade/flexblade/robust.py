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


J_HUB_RANGE = (0.92e-6, 2.5e-6, 5.49e-6)   # itemised min-inertia build .. old generic guess


# control_quality criterion: 4 FIXED +-5 % tolerance samples (deterministic -> evaluations cache);
# a 2-level fractional pattern over the flexure dims that set the mechanism-mode frequency
CQ_PERTURB = (
    {"flexure.thickness": 1.05, "flexure.width": 1.05, "flexure.length": 0.95, "flexure.hinge_radius": 1.05},
    {"flexure.thickness": 0.95, "flexure.width": 0.95, "flexure.length": 1.05, "flexure.hinge_radius": 0.95},
    {"flexure.thickness": 1.05, "flexure.width": 0.95, "flexure.length": 1.05, "flexure.hinge_radius": 1.05},
    {"flexure.thickness": 0.95, "flexure.width": 1.05, "flexure.length": 0.95, "flexure.hinge_radius": 0.95},
)
CQ_PHASE_MAX_DEG = 15.0   # +- about the per-throttle median (the FC trims a fixed per-throttle offset)
CQ_GAIN_MAX = 0.25        # +- about the median gain at each (throttle, J) (the FC gain schedule)


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _median(x):
    x = sorted(x)
    n = len(x)
    return 0.5 * (x[(n - 1) // 2] + x[n // 2])


def control_quality(tags, gains, phases_deg) -> dict:
    """Spread of the complex cyclic gain G over the robust set.

    tags 'T<f>[:p<i>][:J<J>]'. Phase: per throttle, max |wrap(phase - median)| over J x tolerance
    (median taken on the circle, about the circular mean). Gain: per (throttle, J), max
    |G/median(G) - 1| over tolerance (the FC schedules gain with throttle and measured J)."""
    import cmath
    thr = {}
    for t, g, ph in zip(tags, gains, phases_deg):
        parts = t.split(":")
        J = next((x for x in parts[1:] if x.startswith("J")), "")
        thr.setdefault(parts[0], []).append((J, g, ph))
    out = dict(phase_spread_deg=0.0, gain_spread=0.0, per_throttle={})
    for T, rows in thr.items():
        ph = [r[2] for r in rows]
        if any(not math.isfinite(x) for x in ph) or any(not (math.isfinite(r[1]) and r[1] > 0) for r in rows):
            out["phase_spread_deg"] = out["gain_spread"] = float("inf")
            out["per_throttle"][T] = dict(phase_med=float("nan"), phase_spread=float("inf"),
                                          gain_spread=float("inf"))
            continue
        ref = math.degrees(cmath.phase(sum(cmath.exp(1j * math.radians(x)) for x in ph)))
        d = [_wrap(x - ref) for x in ph]
        med = _median(d)
        ps = max(abs(_wrap(x - med)) for x in d)
        gs = 0.0
        for J in {r[0] for r in rows}:
            g = [r[1] for r in rows if r[0] == J]
            mg = _median(g)
            gs = max(gs, max(abs(x / mg - 1.0) for x in g))
        out["per_throttle"][T] = dict(phase_med=_wrap(ref + med), phase_spread=ps, gain_spread=gs)
        out["phase_spread_deg"] = max(out["phase_spread_deg"], ps)
        out["gain_spread"] = max(out["gain_spread"], gs)
    return out


def control_quality_viol(cq: dict) -> dict:
    v = {}
    if cq["phase_spread_deg"] > CQ_PHASE_MAX_DEG:
        v["cq_phase"] = min(cq["phase_spread_deg"] / CQ_PHASE_MAX_DEG - 1.0, 10.0)
    if cq["gain_spread"] > CQ_GAIN_MAX:
        v["cq_gain"] = min(cq["gain_spread"] / CQ_GAIN_MAX - 1.0, 10.0)
    return v


def variants(design, thrust_factors=THRUST_FACTORS, perturb=None, j_hub=None):
    """-> list of (tag, Design).  j_hub: optional motor.J_hub values (unknown until measured),
    each evaluated at every thrust factor and perturbation (tag suffix ':J<value>')."""
    out = []
    pert = [("nom", {})] + [(f"p{i}", p) for i, p in enumerate(perturb or [])]
    js = [None] if not j_hub else list(j_hub)
    for f in thrust_factors:
        for (ptag, p), J in ((pp, J) for pp in pert for J in js):
            d = design
            if J is not None:
                d = d.model_copy(update=dict(motor=d.motor.model_copy(update=dict(J_hub=float(J)))))
            if p:
                d = set_design(d, {k: float(get_path(d, k)) * m for k, m in p.items()})
            if d.flexure.pre_lag_deg == "auto" and d.flexure.pre_lag_ref_mass is None:
                # pre-lag is built-in geometry: solve it once at the nominal thrust, not per point
                d = d.model_copy(update=dict(flexure=d.flexure.model_copy(
                    update=dict(pre_lag_ref_mass=design.op.vehicle_mass))))
            d = d.model_copy(update=dict(op=d.op.model_copy(
                update=dict(vehicle_mass=design.op.vehicle_mass * f, rpm=None,
                            pulse_capability_margin=PULSE_MARGIN))))
            out.append((f"T{f:g}" + ("" if ptag == "nom" else ":" + ptag)
                        + ("" if J is None else f":J{J:.3g}"), d))
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
    cqv = {}
    if all(m.get("criterion") == "control_quality" for m in ms):
        cq = control_quality(tags, [m.get("cyc_gain", float("nan")) for m in ms],
                             [m.get("cyc_phase_deg", float("nan")) for m in ms])
        out["cq"] = cq
        cqv = control_quality_viol(cq)
        viol.update(cqv)
    out["feasible"] = not viol
    if all("cr_pl" in m for m in ms):
        # pulse-limited: each point has its own pulse scale (FC schedules the gain with throttle);
        # robust score = min over points, non-scaling (hard) checks required at every point
        out["cr_pl"] = float(min(m["cr_pl"] for m in ms))
        vh: dict[str, float] = {}
        for m in ms:
            for k, v in m.get("viol_hard", {}).items():
                vh[k] = max(vh.get(k, 0.0), v)
        vh.update(cqv)
        out["viol_hard"], out["feasible_pl"] = vh, not vh
    if all("binding" in m for m in ms):
        out["binding"] = max((m["binding"] for m in ms), key=lambda b: b[1])
    out["points"] = [dict(tag=t, control_ratio=m["control_ratio"], rpm=m.get("rpm"),
                          binding=m.get("binding"), res_margin=m.get("res_margin"),
                          feasible=m.get("feasible"), cr_pl=m.get("cr_pl"),
                          pulse_scale=m.get("pulse_scale"), pulse_frac_cap=m.get("pulse_frac_cap"),
                          pulse_binding=m.get("pulse_binding"), cyc_gain=m.get("cyc_gain"),
                          cyc_phase_deg=m.get("cyc_phase_deg"), mech_mode=m.get("mech_mode"),
                          feasible_pl=m.get("feasible_pl")) for t, m in zip(tags, ms)]
    return out


def map_metrics(pool, designs, metrics_fn: Callable, factors=None, perturb=None, j_hub=None):
    """pool.map over designs -> list of (metrics | None, error). factors=None: single point."""
    if not factors:
        recs = pool.map(designs)
        return [(metrics_fn(d, r["results"]), None) if "results" in r
                else (None, r.get("error", "?")[:200]) for d, r in zip(designs, recs)]
    vs = [variants(d, factors, perturb, j_hub) for d in designs]
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


def evaluate_robust(design, thrust_factors=THRUST_FACTORS, perturb=None, metrics_fn=None, j_hub=None) -> dict:
    """Local (serial) robust evaluation; metrics_fn defaults to study.run_study.metrics-style
    callable (design, results_dict) and must be supplied by the caller from the study modules."""
    from . import api
    assert metrics_fn is not None, "pass metrics_fn(design, results_dict)"
    vs = variants(design, thrust_factors, perturb, j_hub)
    ms = [metrics_fn(d, api.evaluate(d).model_dump(mode="json")) for _, d in vs]
    return combine(ms, [t for t, _ in vs])
