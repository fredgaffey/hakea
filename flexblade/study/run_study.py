"""flexblade design study: FACT flexure parameters per rotor size × material concept.

For every (size, concept) case, differential evolution over the flexure (and the
chordwise blade placement / mass balance `blade.elastic_axis_chord_frac`) maximises
harmonic.control_ratio at the motor-capability 1/rev torque pulse, subject to

  * every check utilisation ≤ 1 (static, interlayer, creep, fatigue, ESC duty peak,
    ESC duty slew with AM32 max_ramp 255 → duty_slew_max 2550/s, motor 1/rev torque),
  * no elastic mode within ±15 % of 1P, 2P, 3P (rotating frame, at hover rpm),
  * static blade-1 vs blade-2 pitch split |Δθ0.75| ≤ 2°,
  * converged trim (no convergence warnings), and a few printability/geometry rules.

Scoring is feasibility-first: feasible J = −100·control_ratio; infeasible
J = 1 + Σ violations (scale-free), so DE first finds the feasible region, then climbs.

Evaluations run on a flexblade.remote.HostPool (laptop + Mac); all cases run
concurrently in threads that share the pool (HostPool is thread-safe), which keeps
every slot busy despite DE's per-generation barrier.

Usage:
  PYTHONPATH=<worktree>/flexblade ~/venv-flexblade/bin/python study/run_study.py \
      --hosts local:28,mac:10 [--sizes 4,5,7,10,12] [--concepts ...] [--coax CASE ...]
Outputs (study/): results/cases.jsonl (one line per case, best design metrics),
results/evals.jsonl.gz (every evaluation, compact), designs/<case>.json (best Design,
loadable by `flexblade evaluate`).
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import os
import sys
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from flexblade import api, fact, motor as mot, presets, robust    # noqa: E402
from flexblade.optimize import Objective, set_design               # noqa: E402
from flexblade.schema import Design, Layup, OptimizeRequest, Target, Variable  # noqa: E402

RAMP_SLEW = 2550.0          # AM32 max_ramp 255 → 255/2000 per 50 µs tick = 2550 /s
STOCK_SLEW = 160.0
RES_BAND = 0.15
SPLIT_MAX = 2.0
FAIL = 1.0e6
# reported but never a feasibility violation: root height is already paid for through
# root drag -> hover power -> motor thermal; the 2x-airfoil envelope is a rule of thumb.
SOFT_CHECKS = {"root_envelope"}
CONCEPTS = presets.CONCEPTS + ("cf_flexure_sym", "tpu_split", "tpu95_split", "torsion_bar",
                               "skewed_cruciform", "tt_strap", "tt_strap_xplate", "tt_strap_steelx")
SIZES = presets.SIZES_IN

RESULTS = HERE / "results"
DESIGNS = HERE / "designs"


# --------------------------------------------------------------------------- case setup


def variables(concept: str, D: float) -> list[Variable]:
    s = D / 5.0
    V = [Variable(path="flexure.skew_deg", lo=15.0, hi=65.0),
         Variable(path="flexure.hinge_radius", lo=0.008 * s, hi=0.018 * s),
         Variable(path="flexure.length", lo=0.003 * s, hi=0.016 * s),
         Variable(path="blade.elastic_axis_chord_frac", lo=0.15, hi=0.65)]
    if concept == "tpu_monolithic":
        V += [Variable(path="flexure.thickness", lo=0.0008, hi=0.007 * s, log=True),
              Variable(path="flexure.width", lo=0.004 * s, hi=0.022 * s),
              Variable(path="flexure.taper", lo=0.4, hi=1.0)]
    elif concept == "tpu_rod":
        V += [Variable(path="flexure.thickness", lo=0.0008, hi=0.006 * s, log=True),
              Variable(path="flexure.width", lo=0.004 * s, hi=0.022 * s),
              Variable(path="flexure.layup.insert_diameter", lo=0.0003, hi=0.0018 * s, log=True),
              Variable(path="blade.layup.insert_diameter", lo=0.0005, hi=0.0025 * s, log=True)]
    elif concept == "tpu_cf_skin":
        V += [Variable(path="flexure.thickness", lo=0.0008, hi=0.006 * s, log=True),
              Variable(path="flexure.width", lo=0.004 * s, hi=0.022 * s),
              Variable(path="flexure.layup.skin_thickness", lo=0.0002, hi=0.0008 * s, log=True),
              Variable(path="blade.layup.skin_thickness", lo=0.0002, hi=0.0010 * s, log=True)]
    elif concept == "torsion_bar":
        V += [Variable(path="flexure.bar_diameter", lo=0.0005, hi=0.003 * s, log=True),
              Variable(path="flexure.bar_length", lo=0.004 * s, hi=0.022 * s),
              Variable(path="flexure.thickness", lo=0.0008, hi=0.005 * s, log=True),
              Variable(path="flexure.width", lo=0.004 * s, hi=0.022 * s),
              Variable(path="flexure.tension_rod_diameter", lo=0.0003, hi=0.0025 * s, log=True)]
    elif concept == "skewed_cruciform":
        V += [Variable(path="flexure.thickness", lo=0.0003, hi=0.0012 * s, log=True),
              Variable(path="flexure.width", lo=0.004 * s, hi=0.024 * s),
              Variable(path="flexure.bar_length", lo=0.006 * s, hi=0.024 * s),
              Variable(path="flexure.cross_angle_deg", lo=60.0, hi=120.0),
              Variable(path="flexure.cruciform_rod_diameter", lo=0.0003, hi=0.0025 * s, log=True)]
    elif concept == "tt_strap":
        # unloaded (axially released) cruciform sets cross/flap; aramid strap carries CF
        V += [Variable(path="flexure.thickness", lo=0.0003, hi=0.0012 * s, log=True),
              Variable(path="flexure.width", lo=0.004 * s, hi=0.024 * s),
              Variable(path="flexure.bar_length", lo=0.006 * s, hi=0.024 * s),
              Variable(path="flexure.cross_angle_deg", lo=60.0, hi=120.0),
              Variable(path="flexure.strap_strand_diameter", lo=0.0003, hi=0.0012 * s, log=True),
              Variable(path="flexure.strap_radius", lo=0.0002, hi=0.002 * s, log=True),
              Variable(path="flexure.strap_pin_diameter", lo=0.002, hi=0.006 * s)]
    elif concept == "tt_strap_xplate":
        # thin wide axially-released crossed plates (cross-axis pivot, k_cross/k_hinge ~ (w/t)²)
        # + a short single-strand strap on the crossing line carrying the centrifugal load
        V += [Variable(path="flexure.thickness", lo=0.0005, hi=0.0008 * s, log=True),
              Variable(path="flexure.width", lo=0.010 * s, hi=0.025 * s),
              Variable(path="flexure.cross_angle_deg", lo=50.0, hi=110.0),
              Variable(path="flexure.strap_strand_diameter", lo=0.0004, hi=0.0012 * s, log=True),
              Variable(path="flexure.strap_length", lo=0.002 * s, hi=0.012 * s, log=True)]
    elif concept == "tt_strap_steelx":
        # crossed spring-steel shim strips (cross-spring pivot on the hinge axis, clamped in
        # printed fittings, radially released) + single-strand strap on the crossing line;
        # thickness: shim stock 0.05-0.20 mm (snap to SHIM_THICKNESSES when building)
        V += [Variable(path="flexure.thickness", lo=0.00005, hi=0.0002, log=True),
              Variable(path="flexure.width", lo=0.004 * s, hi=0.016 * s),
              Variable(path="flexure.cross_angle_deg", lo=50.0, hi=110.0),
              Variable(path="flexure.strap_strand_diameter", lo=0.0004, hi=0.0012 * s, log=True),
              Variable(path="flexure.strap_length", lo=0.002 * s, hi=0.012 * s, log=True)]
    elif concept in ("tpu_split", "tpu95_split"):
        V += [Variable(path="flexure.thickness", lo=0.0008, hi=0.006 * s, log=True),
              Variable(path="flexure.width", lo=0.004 * s, hi=0.022 * s),
              Variable(path="flexure.tension_rod_diameter", lo=0.0003, hi=0.0025 * s, log=True)]
    elif concept in ("cf_flexure", "cf_flexure_sym"):
        V += [Variable(path="flexure.thickness", lo=0.0003, hi=0.0025 * s, log=True),
              Variable(path="flexure.width", lo=0.003 * s, hi=0.022 * s),
              Variable(path="flexure.cross_angle_deg", lo=40.0, hi=120.0),
              Variable(path="flexure.cross_gap", lo=0.0008, hi=0.004 * s)]
    else:
        raise KeyError(concept)
    return V


def capability_frac(d: Design) -> dict:
    """1/rev torque amplitude at motor capability, as a fraction of the hover torque,
    from a trim of design d (per rotor; coaxial: upper and lower)."""
    r = api.evaluate(d, ["trim"])
    if d.layout == "coaxial":
        out = {}
        for k in ("upper", "lower"):
            sub = getattr(r.coaxial, k)
            cap = mot.capability(d.motor, sub.trim.rpm, sub.trim.torque)
            out[k] = dict(rpm=sub.trim.rpm, Q=sub.trim.torque, dQ_max=cap.dQ_max_1p,
                          frac=cap.mod_frac_max, limit=cap.dQ_limit)
        return out
    cap = mot.capability(d.motor, r.trim.rpm, r.trim.torque)
    return dict(rpm=r.trim.rpm, Q=r.trim.torque, dQ_max=cap.dQ_max_1p, frac=cap.mod_frac_max,
                limit=cap.dQ_limit)


def base_design(D: float, concept: str, coax: bool = False, frac_margin: float = 0.95) -> tuple[Design, dict]:
    pc = "cf_flexure" if concept in ("cf_flexure_sym", "tpu_split", "tpu95_split", "torsion_bar",
                                         "skewed_cruciform", "tt_strap", "tt_strap_xplate",
                                         "tt_strap_steelx") else concept
    d = presets.make_coax_preset(D, pc) if coax else presets.make_preset(D, pc)
    if concept == "torsion_bar":
        # Fred's torsion bar: pultruded CF bar on the skewed hinge axis (hinge = bar torsion) +
        # TPU plate (stiffness/damping) + radial CF strap through the hinge point (CF load)
        sc = D / 5.0
        d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(
            topology="torsion_bar", bar_diameter=0.0015 * sc, bar_material="cf_rod_pultruded",
            bar_length=0.010 * sc, bar_with_plate=True, layup=Layup(kind="solid", material="tpu_64d"),
            thickness=0.0015 * sc, width=0.008 * sc, tension_rod_diameter=0.0007 * sc))))
    if concept == "skewed_cruciform":
        # X-section PA6-CF torsion hinge on the skewed axis (fibres along it), centroid CF rod
        # carrying most of the centrifugal load, both embedded in the fittings (fact.py)
        sc = D / 5.0
        d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(
            topology="skewed_cruciform", layup=Layup(kind="solid", material="pa6_cf"),
            thickness=0.0004 * sc, width=0.012 * sc, bar_length=0.012 * sc, cross_angle_deg=90.0,
            cruciform_rod_diameter=0.0012 * sc, bar_material="cf_rod_pultruded"))))
    if concept == "tt_strap":
        # helicopter tension-torsion strap: 4 unbonded aramid strands (radial, through the
        # hinge point) carry the centrifugal load; the PA6-CF cruciform is axially released
        # (radial slider at the blade end) and only sets hinge/cross/flap stiffness
        sc = D / 5.0
        d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(
            topology="skewed_cruciform", layup=Layup(kind="solid", material="pa6_cf"),
            thickness=0.0005 * sc, width=0.020 * sc, bar_length=0.020 * sc, cross_angle_deg=90.0,
            cruciform_rod_diameter=0.0, strap_strands=4, strap_strand_diameter=0.0006 * sc,
            strap_radius=0.0007 * sc, strap_material="aramid_yarn", strap_anchor="pin_wrap",
            strap_pin_diameter=0.003 * sc, axial_release=True, pre_lag_deg="auto", pre_lag_axis="pitch"))))
    if concept == "tt_strap_xplate":
        # strap + released crossed plates: PA6-CF plates printed with the build direction along
        # the hinge axis a (layers ⟂ a, rasters along the plate length → bending stress in-layer),
        # crossing on a with cross_gap ≥ strand Ø so one aramid strand runs radially through H0
        # (short strap_length: the tensioned strand's geometric stiffness ∝ T·strap_length)
        sc = D / 5.0
        d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(
            topology="skewed_cross_blade", layup=Layup(kind="solid", material="pa6_cf"),
            thickness=0.0006, width=0.025 * sc, length=0.008 * sc, cross_angle_deg=70.0,
            cross_gap=0.0012 * sc, cruciform_rod_diameter=0.0, strap_strands=1,
            strap_strand_diameter=0.0007 * sc, strap_radius=0.0, strap_length=0.004 * sc,
            strap_material="aramid_yarn", strap_anchor="pin_wrap", strap_pin_diameter=0.003 * sc,
            axial_release=True))))
    if concept == "tt_strap_steelx":
        # strap + crossed spring-steel shim strips (Haringx cross-spring pivot, crossing line =
        # skewed hinge axis), strips clamped (embedded) in printed hub/blade fittings; the blade
        # fitting rides a radial slider along the strap line (axial_release) so the strap
        # carries the centrifugal load; one aramid strand through the crossing gap
        sc = D / 5.0
        d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(
            topology="skewed_cross_blade", layup=Layup(kind="solid", material="steel_1095_shim"),
            thickness=0.0001, width=0.008 * sc, length=0.008 * sc, cross_angle_deg=90.0,
            cross_gap=0.0012 * sc, cruciform_rod_diameter=0.0, strap_strands=1,
            strap_strand_diameter=0.0007 * sc, strap_radius=0.0, strap_length=0.004 * sc,
            strap_material="aramid_yarn", strap_anchor="pin_wrap", strap_pin_diameter=0.003 * sc,
            axial_release=True, pre_lag_deg="auto", pre_lag_axis="pitch"))))
    if concept in ("tpu_split", "tpu95_split"):
        # split load path: TPU plate hinge (bending) + radial CF rod through the hinge point
        # (centrifugal load); PA6-CF blade
        mat = "tpu_64d" if concept == "tpu_split" else "tpu_95a"
        sc = D / 5.0
        d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(
            topology="skewed_single_blade", layup=Layup(kind="solid", material=mat),
            thickness=0.002 * sc, width=0.010 * sc, tension_rod_diameter=0.001 * sc))))
    if concept == "cf_flexure_sym":      # symmetric (butterfly) crossed plates, see fact.py
        d = set_design(d, {})
        d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(
            topology="skewed_cross_blade_sym"))))
    d = set_design(d, {"blade.elastic_axis_chord_frac": 0.45, "motor.duty_slew_max": RAMP_SLEW})
    # capability from a stiff reference blade (cf_flexure) so every concept gets the same pulse
    ref = presets.make_coax_preset(D, "cf_flexure") if coax else presets.make_preset(D, "cf_flexure")
    ref = set_design(ref, {"blade.elastic_axis_chord_frac": 0.45})
    cap = capability_frac(ref)
    upd: dict = {}
    if coax:
        upd["op.torque_mod_frac"] = frac_margin * cap["upper"]["frac"]
        upd["coaxial.lower_torque_mod_frac"] = frac_margin * cap["lower"]["frac"]
    else:
        upd["op.torque_mod_frac"] = frac_margin * cap["frac"]
    d = set_design(d, upd)
    d = d.model_copy(update=dict(
        name=f"{presets._fmt(D)}in_{'coax_' if coax else ''}{concept}_study",
        notes=d.notes + "; study: 1/rev duty pulse at 0.95× motor capability (DC-equivalent), "
                        "AM32 max_ramp 255 (duty_slew_max 2550/s)"))
    return d, cap


# --------------------------------------------------------------------------- constraints


def _sub_results(r: dict) -> list[tuple[str, dict]]:
    if r.get("coaxial"):
        return [("upper", r["coaxial"]["upper"]), ("lower", r["coaxial"]["lower"])]
    return [("", r)]


def resonance_margin(r1: dict) -> tuple[float, str]:
    rpm = (r1.get("summary") or {}).get("rpm") or (r1.get("trim") or {}).get("rpm")
    if not rpm or not r1.get("modes"):
        return float("nan"), "no modes"
    f1 = rpm / 60.0
    best, which = float("inf"), ""
    for m in r1["modes"]:
        f = m["freq_hz"]
        if f < 0.05 * f1:          # rigid spin
            continue
        for k in (1, 2, 3):
            mg = abs(f / (k * f1) - 1.0)
            if mg < best:
                best, which = mg, f"{m['label']} {f / f1:.2f}/rev vs {k}P"
    return best, which


def geometry_violations(d: Design) -> dict[str, float]:
    f = d.flexure
    v = {}
    try:
        mem = fact.generate_members(d)
        x0 = min(min(m.p0[0], m.p1[0]) for m in mem)
        if x0 < 0.95 * f.hub_radius:
            v["flexure_inside_hub"] = (0.95 * f.hub_radius - x0) / f.hub_radius
    except Exception:  # noqa: BLE001
        v["members"] = 1.0
    if f.layup.kind == "rod_insert" and f.layup.insert_diameter > 0.7 * f.thickness:
        v["flex_rod_fit"] = f.layup.insert_diameter / (0.7 * f.thickness) - 1
    if f.layup.kind == "core_skin" and 2 * f.layup.skin_thickness > 0.6 * f.thickness:
        v["flex_skin_fit"] = 2 * f.layup.skin_thickness / (0.6 * f.thickness) - 1
    R = d.blade.radius
    t75 = 0.12 * R * (presets.CHORD_LAW[0] - presets.CHORD_LAW[1] * 0.75)
    b = d.blade.layup
    if b.kind == "rod_insert" and b.insert_diameter > 0.6 * t75:
        v["blade_rod_fit"] = b.insert_diameter / (0.6 * t75) - 1
    if b.kind == "core_skin" and b.skin_thickness > 0.3 * t75:
        v["blade_skin_fit"] = b.skin_thickness / (0.3 * t75) - 1
    return v


def metrics(d: Design, r: dict) -> dict:
    """Constraint breakdown + headline metrics from a (stripped) Results dict."""
    out: dict = {}
    viol: dict[str, float] = {}
    util: dict[str, float] = {}
    for c in r.get("checks", []):
        key = c["name"] if c["member"] in ("esc", "motor") else f"{c['name']}"
        u = c["utilisation"]
        if not math.isfinite(u):
            u = 1e3
        if u > util.get(key, -1):
            util[key] = u
        if c["member"] not in ("esc", "motor"):
            k2 = f"member:{c['member']}"
            util[k2] = max(util.get(k2, 0.0), u)
    out["util"] = util
    for k, u in util.items():
        if not k.startswith("member:") and k not in SOFT_CHECKS and u > 1:
            viol["check:" + k] = math.log(u)
    splits, margins, which = [], [], []
    for tag, sub in _sub_results(r):
        sp = (sub.get("summary") or {}).get("trim_diff_pitch_deg", float("nan"))
        splits.append(sp)
        mg, wh = resonance_margin(sub)
        margins.append(mg)
        which.append(wh)
    sp = max((abs(x) for x in splits if math.isfinite(x)), default=float("nan"))
    mg = min((x for x in margins if math.isfinite(x)), default=float("nan"))
    out.update(split_deg=sp, res_margin=mg, res_which=which[int(np.nanargmin(margins))] if
               any(math.isfinite(x) for x in margins) else "")
    if not math.isfinite(sp):
        viol["split"] = 1.0
    elif sp > SPLIT_MAX:
        viol["split"] = (sp - SPLIT_MAX) / SPLIT_MAX
    if not math.isfinite(mg):
        viol["resonance"] = 1.0
    elif mg < RES_BAND:
        viol["resonance"] = (RES_BAND - mg) / RES_BAND
    nconv = sum(("not converge" in w) or ("failed" in w) for w in r.get("warnings", []))
    if nconv:
        viol["convergence"] = float(nconv)
    viol.update({"geom:" + k: v for k, v in geometry_violations(d).items()})
    out["viol"] = viol
    s = r.get("summary") or {}
    h = r.get("harmonic") or {}
    out["control_ratio"] = float(s.get("control_ratio", h.get("control_ratio", 0.0)) or 0.0)
    out["tilt_equiv_deg"] = float(h.get("tilt_equiv_deg", float("nan")))
    out["pitch_amp_deg"] = float(s.get("pitch_amp_deg", float("nan")))
    out["lag_amp_deg"] = float(s.get("lag_amp_deg", float("nan")))
    out["lag_per_rev"] = float(s.get("lag_freq_per_rev", float("nan")))
    out["flap_per_rev"] = float(s.get("flap_freq_per_rev", float("nan")))
    out["pitch_per_rev"] = float(s.get("pitch_freq_per_rev", float("nan")))
    out["moment_phase_deg"] = float(s.get("hub_moment_phase_deg", float("nan")))
    out["rpm"] = float(s.get("rpm", s.get("rpm_upper", float("nan"))))
    out["thrust"] = float(s.get("thrust", s.get("thrust_total", float("nan"))))
    out["fom"] = float((r.get("trim") or {}).get("figure_of_merit", s.get("figure_of_merit", float("nan"))))
    out["power"] = float(s.get("power_total", (r.get("trim") or {}).get("power", float("nan"))))
    out["torque_amp"] = float(h.get("torque_amp", float("nan")))
    su = s.get("duty_slew_util")
    if su is None:
        su = util.get("esc_duty_slew")
    out["slew_util_ramp255"] = float(su) if su is not None else float("nan")
    out["slew_util_stock"] = out["slew_util_ramp255"] * RAMP_SLEW / STOCK_SLEW
    out["coning_deg"] = float(s.get("coning_deg", float("nan")))
    out["feasible"] = not viol
    return out


def score(m: dict) -> float:
    if m["feasible"]:
        return -100.0 * m["control_ratio"]
    return 1.0 + float(sum(m["viol"].values()))


# --------------------------------------------------------------------------- driver


class Logger:
    def __init__(self, path: Path):
        self.fh = gzip.open(path, "at")
        self.lock = threading.Lock()

    def write(self, rec: dict):
        with self.lock:
            self.fh.write(json.dumps(rec, default=float) + "\n")

    def close(self):
        self.fh.close()


def _round(v: dict) -> dict:
    return {k: (round(x, 7) if isinstance(x, float) else x) for k, x in v.items()}


ROBUST = None   # thrust factors tuple when --robust


def run_case(case: str, D: float, concept: str, coax: bool, pool, log: Logger, popmult: int,
             maxiter: int, seed: int) -> dict:
    from scipy.optimize import differential_evolution

    t0 = time.time()
    base, cap = base_design(D, concept, coax)
    V = variables(concept, D)
    obj = Objective(OptimizeRequest(design=base, variables=V,
                                    targets=[Target(quantity="harmonic.control_ratio", kind="max")]))
    best = dict(J=float("inf"))
    n = [0]
    lock = threading.Lock()

    def mapper(_f, pop):
        pop = [np.asarray(u, float) for u in pop]
        designs = [set_design(base, obj.to_values(u)) for u in pop]
        recs = robust.map_metrics(pool, designs, metrics, ROBUST, None)
        Js = []
        for u, dsg, (mm, err) in zip(pop, designs, recs):
            vals = obj.to_values(u)
            if mm is None or "error" in mm:
                J, m = FAIL, dict(error=err or mm["error"])
            else:
                m = mm
                J = score(m)
            Js.append(J)
            with lock:
                n[0] += 1
                if J < best["J"]:
                    best.update(J=J, vals=vals, m=m)
            log.write(dict(case=case, J=J, x=_round(vals), **{k: v for k, v in m.items()
                                                              if k not in ("util",)},
                           util={k: round(v, 4) for k, v in m.get("util", {}).items()}))
        return Js

    u0 = obj.to_u({v.path: min(max(float(_get(base, v.path)), v.lo), v.hi) for v in V})
    gen = [0]

    def cb(xk, convergence=None):
        gen[0] += 1
        b = best.get("m", {})
        print(f"[{case}] gen {gen[0]:3d} evals {n[0]:5d} best J {best['J']:+.4f} "
              f"cr {b.get('control_ratio', float('nan')):.4f} feas {b.get('feasible')} "
              f"viol {_short(b.get('viol', {}))}  {time.time() - t0:.0f}s", flush=True)

    r = differential_evolution(lambda u: 0.0, [(0.0, 1.0)] * len(V), maxiter=maxiter, popsize=popmult,
                               seed=seed, x0=u0, polish=False, tol=1e-4, atol=0.0,
                               init="latinhypercube", mutation=(0.5, 1.0), recombination=0.7,
                               workers=mapper, updating="deferred", callback=cb)
    bd = set_design(base, best["vals"])
    out = dict(case=case, diameter_in=D, concept=concept, layout="coaxial" if coax else "single",
               J=best["J"], n_evals=n[0], generations=gen[0], de_message=str(r.message),
               wall_s=time.time() - t0, capability=cap, x=_round(best["vals"]),
               torque_mod_frac=bd.op.torque_mod_frac, **best["m"])
    DESIGNS.mkdir(parents=True, exist_ok=True)
    bd = bd.model_copy(update=dict(name=case))
    (DESIGNS / f"{case}.json").write_text(json.dumps(bd.model_dump(mode="json"), indent=1))
    return out


def _get(d: Design, path: str):
    from flexblade.optimize import get_path
    return get_path(d.model_dump(mode="json"), path)


def _short(v: dict) -> str:
    return ",".join(f"{k.split(':')[-1]}={x:.2f}" for k, x in sorted(v.items(), key=lambda kv: -kv[1])[:3])


def salvage(case_names: list[str]) -> None:
    """Rebuild case summaries + best designs from evals.jsonl.gz for runs that were stopped
    before DE finished (best evaluated point so far; the gz tail lost at kill is ignored)."""
    best: dict[str, dict] = {}
    count: dict[str, int] = {}
    try:
        with gzip.open(RESULTS / "evals.jsonl.gz", "rt") as fh:
            for line in fh:
                try:
                    r = json.loads(line)
                except json.JSONDecodeError:
                    break
                c = r.get("case")
                if c not in case_names:
                    continue
                D_ = float(c.split("in_")[0])
                con_ = c.split("in_", 1)[1].replace("coax_", "")
                if set(r.get("x", {})) != {v.path for v in variables(con_, D_)}:
                    continue          # evaluation from an earlier run with other variables
                count[c] = count.get(c, 0) + 1
                if "viol" in r and (c not in best or r["J"] < best[c]["J"]):
                    best[c] = r
    except (EOFError, OSError):
        pass
    DESIGNS.mkdir(parents=True, exist_ok=True)
    for c, r in best.items():
        coax = "_coax_" in c
        D = float(c.split("in_")[0])
        concept = c.split("in_", 1)[1].replace("coax_", "")
        base, cap = base_design(D, concept, coax)
        bd = set_design(base, r["x"]).model_copy(update=dict(name=c))
        (DESIGNS / f"{c}.json").write_text(json.dumps(bd.model_dump(mode="json"), indent=1))
        out = dict(case=c, diameter_in=D, concept=concept, layout="coaxial" if coax else "single",
                   J=r["J"], n_evals=count[c], de_message="stopped early (pool resized); salvaged best "
                   "evaluated point from evals.jsonl.gz", capability=cap, x=r["x"],
                   torque_mod_frac=bd.op.torque_mod_frac,
                   **{k: v for k, v in r.items() if k not in ("case", "J", "x")})
        with open(RESULTS / "cases.jsonl", "a") as fh:
            fh.write(json.dumps(out, default=float) + "\n")
        print(f"salvaged {c}: {count[c]} evals, J {r['J']:.4f} cr {r['control_ratio']:.4f} "
              f"feasible {r['feasible']}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--hosts", default="local:28,mac:10")
    ap.add_argument("--sizes", default=",".join(str(s) for s in SIZES))
    ap.add_argument("--concepts", default=",".join(presets.CONCEPTS))
    ap.add_argument("--coax", action="append", default=[], help="D:concept for a coaxial case")
    ap.add_argument("--robust", nargs="?", const="0.7,1.0,1.4", default=None,
                    help="evaluate at several thrust factors (min control_ratio, constraints at every point)")
    ap.add_argument("--no-single", action="store_true")
    ap.add_argument("--popmult", type=int, default=8)
    ap.add_argument("--maxiter", type=int, default=40)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--threads", type=int, default=24)
    ap.add_argument("--redo", action="store_true", help="re-run cases already in cases.jsonl")
    ap.add_argument("--salvage", nargs="*", help="rebuild these cases from evals.jsonl.gz and exit")
    a = ap.parse_args(argv)
    global ROBUST
    ROBUST = tuple(float(x) for x in a.robust.split(",")) if a.robust else None
    if a.salvage:
        salvage(a.salvage)
        return
    os.environ.setdefault("FLEXBLADE_REMOTE_TAG", "study")
    RESULTS.mkdir(parents=True, exist_ok=True)
    done = set()
    cases_path = RESULTS / "cases.jsonl"
    if cases_path.exists() and not a.redo:
        done = {json.loads(l)["case"] for l in cases_path.read_text().splitlines() if l.strip()}
    cases = []
    if not a.no_single:
        for D in [float(x) for x in a.sizes.split(",")]:
            for c in a.concepts.split(","):
                cases.append((f"{presets._fmt(D)}in_{c}", D, c, False))
    for spec in a.coax:
        Ds, c = spec.split(":")
        cases.append((f"{Ds}in_coax_{c}", float(Ds), c, True))
    cases = [c for c in cases if c[0] not in done]
    print(f"{len(cases)} cases: {[c[0] for c in cases]}", flush=True)
    from flexblade import remote
    log = Logger(RESULTS / "evals.jsonl.gz")
    lock = threading.Lock()
    with remote.HostPool(a.hosts) as pool:
        def one(c):
            try:
                out = run_case(*c, pool=pool, log=log, popmult=a.popmult, maxiter=a.maxiter, seed=a.seed)
            except Exception as ex:  # noqa: BLE001
                traceback.print_exc()
                out = dict(case=c[0], error=f"{type(ex).__name__}: {ex}")
            with lock:
                with open(cases_path, "a") as fh:
                    fh.write(json.dumps(out, default=float) + "\n")
            print(f"DONE {c[0]}: J {out.get('J')} cr {out.get('control_ratio')} feasible "
                  f"{out.get('feasible')}", flush=True)
            return out
        with ThreadPoolExecutor(max_workers=a.threads) as ex:
            list(ex.map(one, cases))
    log.close()


if __name__ == "__main__":
    main()
