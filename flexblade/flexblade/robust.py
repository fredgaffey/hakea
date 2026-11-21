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

import numpy as np

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


# control_quality_cal criterion: the BUILD tolerance (CQ_PERTURB corners) is calibrated out once
# per airframe (bench/in-flight sysid of G per throttle at the measured J -> FC schedule); only
# RUNTIME variation about each calibrated node must fit in the limits. One-at-a-time runtime
# samples, applied on top of every (throttle, build sample, J) node (tag ':r<name>'):
#   rpm+/rpm-  rpm +-5 % (schedule bin half-width) via thrust ~ rpm^2
#   temp+      +20 C: part temperature op.ambient_C 45 C (printed parts derate via
#              materials.temp_knockdown; steel/aramid have no T dependence in the model) and
#              steel strip E -0.03 %/C -> x0.994
#   creep      strip-stiffness relaxation -2 %
#   J+/J-      J_hub measurement error +-10 % (the schedule uses the measured J)
# temp- (-20 C) is not evaluated: below 25 C the model has no part-temperature effect, so only the
# strip E +0.6 % remains -> its deviation is taken as 0.3 x the creep (-2 %) deviation (linear).
# Strip E is applied as a strip-WIDTH multiplier (EI, EA, GJ of a thin strip are all linear in w;
# the in-plane bending / mass shifts are second order) so the variant stays a pure Design (cached).
CQ_RUNTIME = (
    ("rpm+", {"thrust": 1.05 ** 2}),
    ("rpm-", {"thrust": 0.95 ** 2}),
    ("temp+", {"ambient_C": 45.0, "strip_E": 1.0 - 0.0003 * 20}),
    ("creep", {"strip_E": 0.98}),
    ("J+", {"J": 1.10}),
    ("J-", {"J": 0.90}),
)
CQ_RUNTIME_GROUPS = (("rpm", ("rpm+", "rpm-")), ("temp", ("temp+",)), ("creep", ("creep",)),
                     ("J", ("J+", "J-")))
CQ_TEMP_MINUS_FROM_CREEP = 0.006 / 0.02    # temp- deviation = 0.3 x creep deviation
CQC_PHASE_MAX_DEG = 15.0   # residual about the calibrated value, worst-case LINEAR stack of groups
CQC_GAIN_MAX = 0.25
CQC_AUTHORITY_MIN = 0.5    # |G| of every build sample >= 0.5 x nominal at the same (throttle, J)
CQC_MECH_MARGIN_MIN = 0.03  # mechanism mode |f/1P - 1| for every build sample, on the nominal side


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


CQ_RPM_RANGE = (0.7, 1.4)   # throttle (thrust factor) range; rpm ~ sqrt(thrust)
CQ_RPM_NOISE = 0.005        # eRPM measurement noise (relative)


def cont_points(n_nodes=5, n_sub=3, trange=CQ_RPM_RANGE):
    """Continuous-eRPM schedule: n_nodes calibration nodes equally spaced in rpm (~ sqrt thrust) over
    the throttle range, plus n_sub-1 interior test points per interval (n_sub=3: thirds).
    -> (node thrust factors, extra test thrust factors), rounded for stable cache keys."""
    r = np.linspace(math.sqrt(trange[0]), math.sqrt(trange[1]), (n_nodes - 1) * n_sub + 1)
    th = [round(float(x) ** 2, 4) for x in r]
    return th[::n_sub], [t for i, t in enumerate(th) if i % n_sub]


def interp_residual(x, ph, g, nodes_idx, noise=CQ_RPM_NOISE):
    """x ln rpm (ascending), ph phase deg, g gain, over nodes + test points; nodes_idx the calibration
    indices. Linear interpolation of (unwrapped phase, gain) between nodes. -> (phase_deg, gain_rel)
    = worst interpolation error at the test points + noise * worst |d/dln rpm| of the node curve."""
    x, g = np.asarray(x, float), np.asarray(g, float)
    ph = np.degrees(np.unwrap(np.radians(ph)))
    ni = list(nodes_idx)
    ip = np.interp(x, x[ni], ph[ni])
    ig = np.interp(x, x[ni], g[ni])
    dp = float(np.max(np.abs(ip - ph)))
    dg = float(np.max(np.abs(ig / g - 1.0)))
    sp = float(np.max(np.abs(np.diff(ph[ni]) / np.diff(x[ni]))))
    sg = float(np.max(np.abs(np.diff(g[ni]) / np.diff(x[ni])) / np.min(g[ni])))
    return dp + sp * noise, dg + sg * noise


def _parse_tag(t):
    parts = t.split(":")
    get = lambda c: next((x for x in parts[1:] if x.startswith(c)), "")   # noqa: E731
    return parts[0], get("p"), get("r")[1:], get("J")


def control_quality_cal(tags, ms, rpm_schedule="bin", n_nodes=5, rpm_noise=CQ_RPM_NOISE) -> dict:
    """Post-calibration residual + calibratability over the robust set (control_quality_cal).

    tags 'T<f>[:p<i>][:r<name>][:J<J>]', ms per-point metrics (cyc_gain, cyc_phase_deg,
    mech_per_rev, cr_pl). Node = (throttle, build sample, J): calibrated value = its ':r'-free
    point. Per runtime group (CQ_RUNTIME_GROUPS) the worst |dphase| / |G/G_cal - 1|; the residual
    is their LINEAR sum (+ temp- = CQ_TEMP_MINUS_FROM_CREEP x creep). Calibratability per build
    sample: |G_b|/|G_nom| at the same (T, J), and the mechanism-mode 1P margin with the same side
    of 1P as the nominal build."""
    nan = float("nan")
    nodes: dict = {}
    for t, m in zip(tags, ms):
        T, p, r, J = _parse_tag(t)
        nodes.setdefault((T, p, J), {})[r] = m
    cont = {}
    if rpm_schedule == "continuous":
        # rpm group = interpolation error + noise of the n-node rpm schedule, per (build, J)
        th_nodes, _ = cont_points(n_nodes)
        curves: dict = {}
        for (T, p, J), rr in nodes.items():
            c = rr.get("")
            if c is not None:
                curves.setdefault((p, J), []).append((float(T[1:]), c))
        for k, pts in curves.items():
            pts.sort(key=lambda a: a[0])
            ok = all(math.isfinite(c.get("cyc_gain", nan)) and c.get("cyc_gain", 0) > 0
                     and math.isfinite(c.get("cyc_phase_deg", nan)) for _, c in pts)
            idx = [i for i, (t, _) in enumerate(pts) if any(abs(t - u) < 1e-9 for u in th_nodes)]
            if not ok or len(idx) < 2:
                cont[k] = (float("inf"), float("inf"))
                continue
            cont[k] = interp_residual([0.5 * math.log(t) for t, _ in pts], [c["cyc_phase_deg"] for _, c in pts],
                                      [c["cyc_gain"] for _, c in pts], idx, rpm_noise)
    nan = float("nan")
    out = dict(phase_res_deg=0.0, gain_res=0.0, authority_min=float("inf"), cr_pl_ratio_min=float("inf"),
               mech_margin_min=float("inf"), mech_side_flip=False, groups={}, per_node={})
    for (T, p, J), rr in sorted(nodes.items()):
        c = rr.get("")
        if c is None:
            continue
        g0, ph0 = c.get("cyc_gain", nan), c.get("cyc_phase_deg", nan)
        ok0 = math.isfinite(g0) and g0 > 0 and math.isfinite(ph0)
        dev = {}
        if cont:
            if len(rr) < 2:
                continue            # interpolation test point: no runtime samples, not a calibration node
            dev["rpm"] = cont[(p, J)]
        for name, members in CQ_RUNTIME_GROUPS:
            if rpm_schedule == "continuous" and name == "rpm":
                continue
            if not any(r in rr for r in members):
                continue
            dp = dg = 0.0
            for r in members:
                if r not in rr:
                    continue
                g, ph = rr[r].get("cyc_gain", nan), rr[r].get("cyc_phase_deg", nan)
                if not (ok0 and math.isfinite(g) and math.isfinite(ph)):
                    dp = dg = float("inf")
                    break
                dp, dg = max(dp, abs(_wrap(ph - ph0))), max(dg, abs(g / g0 - 1.0))
            dev[name] = (dp, dg)
        if "creep" in dev:
            t_ = dev.get("temp", (0.0, 0.0))
            dev["temp"] = (t_[0] + CQ_TEMP_MINUS_FROM_CREEP * dev["creep"][0],
                           t_[1] + CQ_TEMP_MINUS_FROM_CREEP * dev["creep"][1])
        if not ok0:
            dev = {k: (float("inf"), float("inf")) for k in dev} or {"cal": (float("inf"), float("inf"))}
        ps, gs = sum(v[0] for v in dev.values()), sum(v[1] for v in dev.values())
        node = dict(phase_res=ps, gain_res=gs, gain=g0, phase=ph0, groups=dev)
        nom = nodes.get((T, "", J), {}).get("")
        if p and nom is not None:
            gn = nom.get("cyc_gain", nan)
            a = g0 / gn if (ok0 and math.isfinite(gn) and gn > 0) else 0.0
            node["authority"] = a
            out["authority_min"] = min(out["authority_min"], a)
            cn, cb = nom.get("cr_pl"), c.get("cr_pl")
            if cn and cb is not None and cn > 0:
                out["cr_pl_ratio_min"] = min(out["cr_pl_ratio_min"], cb / cn)
        mpr = c.get("mech_per_rev", nan)
        mpr0 = (nom or c).get("mech_per_rev", nan)
        if math.isfinite(mpr):
            out["mech_margin_min"] = min(out["mech_margin_min"], abs(mpr - 1.0))
            if math.isfinite(mpr0) and (mpr - 1.0) * (mpr0 - 1.0) < 0:
                out["mech_side_flip"] = True
        else:
            out["mech_margin_min"] = 0.0
        node["mech_per_rev"] = mpr
        node["mech_zeta"] = c.get("mech_zeta", nan)
        out["per_node"][f"{T}:{p or 'nom'}:{J}"] = node
        out["phase_res_deg"], out["gain_res"] = max(out["phase_res_deg"], ps), max(out["gain_res"], gs)
        for k, v in dev.items():
            o = out["groups"].setdefault(k, [0.0, 0.0])
            o[0], o[1] = max(o[0], v[0]), max(o[1], v[1])
    return out


def control_quality_cal_viol(c: dict) -> dict:
    v = {}
    if c["phase_res_deg"] > CQC_PHASE_MAX_DEG:
        v["cqc_phase"] = min(c["phase_res_deg"] / CQC_PHASE_MAX_DEG - 1.0, 10.0)
    if c["gain_res"] > CQC_GAIN_MAX:
        v["cqc_gain"] = min(c["gain_res"] / CQC_GAIN_MAX - 1.0, 10.0)
    if math.isfinite(c["authority_min"]) and c["authority_min"] < CQC_AUTHORITY_MIN:
        v["cqc_authority"] = 1.0 - c["authority_min"] / CQC_AUTHORITY_MIN
    if c["mech_margin_min"] < CQC_MECH_MARGIN_MIN:
        v["cqc_mech_margin"] = 1.0 - c["mech_margin_min"] / CQC_MECH_MARGIN_MIN
    if c["mech_side_flip"]:
        v["cqc_mech_side"] = 1.0
    return v


def _apply_runtime(d, rt: dict):
    upd = {}
    if "strip_E" in rt:
        upd["flexure.width"] = float(get_path(d, "flexure.width")) * rt["strip_E"]
    if "J" in rt:
        upd["motor.J_hub"] = float(d.motor.J_hub) * rt["J"]
    if upd:
        d = set_design(d, upd)
    if "ambient_C" in rt:
        d = d.model_copy(update=dict(op=d.op.model_copy(update=dict(ambient_C=float(rt["ambient_C"])))))
    return d


def variants(design, thrust_factors=THRUST_FACTORS, perturb=None, j_hub=None, runtime=None):
    """-> list of (tag, Design).  j_hub: optional motor.J_hub values (unknown until measured),
    each evaluated at every thrust factor and perturbation (tag suffix ':J<value>')."""
    out = []
    pert = [("nom", {})] + [(f"p{i}", p) for i, p in enumerate(perturb or [])]
    js = [None] if not j_hub else list(j_hub)
    rts = [("", {})] + list(runtime or [])     # runtime: CQ_RUNTIME-style (name, spec) samples
    for f in thrust_factors:
        for (ptag, p), (rtag, rt), J in ((pp, rr, J) for pp in pert for rr in rts for J in js):
            d = design
            if J is not None:
                d = d.model_copy(update=dict(motor=d.motor.model_copy(update=dict(J_hub=float(J)))))
            if p:
                d = set_design(d, {k: float(get_path(d, k)) * m for k, m in p.items()})
            if d.flexure.pre_lag_deg == "auto" and d.flexure.pre_lag_ref_mass is None:
                # pre-lag is built-in geometry: solve it once at the nominal thrust, not per point
                d = d.model_copy(update=dict(flexure=d.flexure.model_copy(
                    update=dict(pre_lag_ref_mass=design.op.vehicle_mass))))
            d = _apply_runtime(d, rt)
            d = d.model_copy(update=dict(op=d.op.model_copy(
                update=dict(vehicle_mass=design.op.vehicle_mass * f * rt.get("thrust", 1.0), rpm=None,
                            pulse_capability_margin=PULSE_MARGIN))))
            out.append((f"T{f:g}" + ("" if ptag == "nom" else ":" + ptag) + (":r" + rtag if rtag else "")
                        + ("" if J is None else f":J{J:.3g}"), d))
    return out


def combine(ms: list[dict], tags: list[str], **cqc_kw) -> dict:
    """Merge per-point metrics dicts: min control_ratio, max util/viol per key, min res_margin."""
    ok = [(t, m) for t, m in zip(tags, ms) if m and "control_ratio" in m]
    if not ok:
        return dict(error="no point evaluated")
    tags, ms = zip(*ok)
    cq = cqc = None
    if all(m.get("criterion") == "control_quality_cal" for m in ms):
        # build samples (':p') and runtime samples (':r') only feed the calibration residual /
        # calibratability; cr_pl and the hard checks come from the nominal points
        cqc = control_quality_cal(tags, ms, **cqc_kw)
        nom = [(t, m) for t, m in zip(tags, ms) if ":p" not in t and ":r" not in t and ":c" not in t]
        if nom:
            tags, ms = zip(*nom)
    elif all(m.get("criterion") == "control_quality" for m in ms):
        # the CQ tolerance samples (':p<i>' tags) only measure the G spread; cr_pl, checks and
        # the remaining hard margins come from the nominal points (tolerance is verified separately)
        cq = control_quality(tags, [m.get("cyc_gain", float("nan")) for m in ms],
                             [m.get("cyc_phase_deg", float("nan")) for m in ms])
        nom = [(t, m) for t, m in zip(tags, ms) if ":p" not in t]
        if nom:
            tags, ms = zip(*nom)
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
    if cq is not None:
        out["cq"] = cq
        cqv = control_quality_viol(cq)
        viol.update(cqv)
    if cqc is not None:
        out["cqc"] = cqc
        cqv = control_quality_cal_viol(cqc)
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


def map_metrics(pool, designs, metrics_fn: Callable, factors=None, perturb=None, j_hub=None, runtime=None):
    """pool.map over designs -> list of (metrics | None, error). factors=None: single point."""
    if not factors:
        recs = pool.map(designs)
        return [(metrics_fn(d, r["results"]), None) if "results" in r
                else (None, r.get("error", "?")[:200]) for d, r in zip(designs, recs)]
    vs = [variants(d, factors, perturb, j_hub, runtime) for d in designs]
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


MECH_ZETA_MIN = 0.02


def minimal_gates(c: dict, zmin: float = MECH_ZETA_MIN) -> dict:
    """Opt-in minimal gate set (study/results/gate_audit.md): drop cqc_mech_margin / cqc_mech_side and
    add cqc_mech_zeta (worst per-node cyclic-mechanism-mode damping ratio >= zmin; missing = violation)."""
    v = {k: x for k, x in (c.get("viol_hard") or {}).items() if k not in ("cqc_mech_margin", "cqc_mech_side")}
    zs = [n.get("mech_zeta", float("nan")) for n in (c.get("per_node") or {}).values()]
    zs = [z for z in zs if z is not None]
    z = min((x for x in zs if x == x), default=float("nan"))
    if not zs or z != z:
        v["cqc_mech_zeta"] = 1.0
    elif z < zmin:
        v["cqc_mech_zeta"] = 1.0 - z / zmin
    c["mech_zeta_min"] = z
    return v


# ---------------------------------------------------------------- opt-in damped-resonance criterion
# study/results/damped_resonance.md. Replaces the raw frequency band (cqc_mech_margin / RES_BAND on
# the mechanism mode) by what the band is a proxy for: SDOF 1P response of the mech mode at p/rev
# with damping zeta, H = 1/(1 - s^2 + 2i zeta s), s = 1/p. Over a frequency spread p0 (1 +- delta)
# (build tolerance not calibrated out / drift), gate (a) |A(p)/A(p0) - 1| <= CQC_GAIN_MAX and
# |phi(p) - phi(p0)| <= CQC_PHASE_MAX_DEG, (b) worst amplification A <= DAMPED_AMP_CAP. The cap is
# the undamped amplification at the project's RES_BAND (0.15, above 1P) = 4.10, i.e. (b) is the
# margin_1p_eff >= RES_BAND rule applied over the whole tolerance band. Defaults are NOT changed.
DAMPED_DELTA = 0.05          # +-10 % stiffness -> +-5 % frequency (brief); CF-dominated modes: less
DAMPED_RES_BAND = 0.15
DAMPED_AMP_CAP = 1.0 / (1.0 - 1.0 / (1.0 + DAMPED_RES_BAND) ** 2)


def sdof_1p(p: float, zeta: float) -> tuple[float, float]:
    """(amplification |H|, phase lag deg in [0, 180]) of a mode at p/rev forced at 1P."""
    s = 1.0 / max(p, 1e-9)
    return (float(1.0 / math.hypot(1 - s * s, 2 * zeta * s)),
            float(math.degrees(math.atan2(2 * zeta * s, 1 - s * s))))


def damped_resonance(p: float, zeta: float, delta: float = DAMPED_DELTA, gain_max: float = CQC_GAIN_MAX,
                     phase_max: float = CQC_PHASE_MAX_DEG, amp_cap: float = DAMPED_AMP_CAP, n: int = 81) -> dict:
    """Gain/phase spread and worst amplification over p (1 +- delta); viol keys damped_gain /
    damped_phase / damped_amp (normalised excess, run_study style)."""
    a0, ph0 = sdof_1p(p, zeta)
    ps = [p * (1 + x) for x in np.linspace(-delta, delta, n)]
    pk = 1.0 / math.sqrt(max(1 - 2 * zeta * zeta, 1e-9))          # |H| peak (s = sqrt(1 - 2 zeta^2))
    if p * (1 - delta) <= pk <= p * (1 + delta):
        ps.append(pk)
    rs_ = [sdof_1p(x, zeta) for x in ps]
    gs = max(abs(a / a0 - 1) for a, _ in rs_)
    phs = max(abs(f - ph0) for _, f in rs_)
    aw = max(a for a, _ in rs_)
    v = {}
    if gs > gain_max:
        v["damped_gain"] = gs / gain_max - 1
    if phs > phase_max:
        v["damped_phase"] = phs / phase_max - 1
    if aw > amp_cap:
        v["damped_amp"] = aw / amp_cap - 1
    return dict(p=p, zeta=zeta, amp=a0, phase_deg=ph0, amp_worst=aw, gain_spread=gs, phase_spread_deg=phs,
                viol=v)


def damped_required_margin(zeta: float, side: int = -1, **kw) -> float:
    """Smallest |p - 1| (mode below 1P: side -1, above: +1) from which every larger margin passes."""
    grid = np.arange(0.0, 0.6001, 0.0025)
    ok = [not damped_resonance(1 + side * m, zeta, **kw)["viol"] for m in grid]
    for i in range(len(grid) - 1, -1, -1):
        if not ok[i]:
            return float(grid[i + 1]) if i + 1 < len(grid) else float("nan")
    return 0.0


def damped_gates(c: dict, **kw) -> dict:
    """Opt-in damped gate set (cf. minimal_gates): drop cqc_mech_margin / cqc_mech_side and gate every
    per-node mech mode (mech_per_rev, mech_zeta) with damped_resonance (worst over nodes; a node with
    missing p or zeta = violation 'damped_missing'). Adds c['damped'] = worst-node summary."""
    v = {k: x for k, x in (c.get("viol_hard") or {}).items() if k not in ("cqc_mech_margin", "cqc_mech_side")}
    worst = None
    for name, n in (c.get("per_node") or {}).items():
        p, z = n.get("mech_per_rev"), n.get("mech_zeta")
        if p is None or z is None or not (math.isfinite(p) and math.isfinite(z)):
            v["damped_missing"] = 1.0
            continue
        d = damped_resonance(p, z, **kw)
        for k, x in d["viol"].items():
            v[k] = max(v.get(k, 0.0), x)
        if worst is None or d["amp_worst"] > worst["amp_worst"]:
            worst = dict(d, node=name)
    if worst is None:
        v["damped_missing"] = 1.0
    c["damped"] = worst
    return v
