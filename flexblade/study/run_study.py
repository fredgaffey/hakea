"""flexblade design study: FACT flexure parameters per rotor size × material concept.

For every (size, concept) case, differential evolution over the flexure (and the
chordwise blade placement / mass balance `blade.elastic_axis_chord_frac`) maximises
harmonic.control_ratio at the motor-capability 1/rev torque pulse, subject to

  * every check utilisation ≤ 1 (static, interlayer, creep, fatigue, ESC duty peak,
    ESC duty slew with AM32 max_ramp 255 → duty_slew_max 2550/s, motor 1/rev torque),
  * no elastic mode within ±15 % of 1P, nor hub/airframe-dominated modes within ±15 % of 2P
    (rotating frame, at hover rpm, damping-aware).  nP crossings (n >= 2) of blade modes are
    unavoidable over a sqrt(2) rpm range; they are checked by forced response instead
    (flexblade.forced: fatigue_* include the resonant nP amplitude, harmonic_jitter check)
    and reported here as info (res_np_info),
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
import re
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
LAMINAE_VARS = False   # --laminae-vars: strip_laminae count + stock lamina thickness (steelx)
BLADE_VARS = False     # --blade-vars: blade.chord_scale + hollow-skin layup (steelx)
BAL_VARS = False       # --one-sided also enables the bal.* balance-pair variables (steelx)
ONE_SIDED = False
GATES_MINIMAL = False  # --gates minimal: replaces mech_band/one-sided gates by the mech-mode zeta gate (NOTE: run_study's
# mech_band is a nominal-only 1P band + rpm drift, not sa.py's per-corner CAL cqc_mech_margin/side; closest equivalent)
DRIFT_MAX = 0.07
MECH_MARGIN = 0.075   # --criterion mech_band: nominal hard 1P margin of the mechanism mode
GOODMAN = False       # --goodman: strip_fatigue_mean="goodman" on every evaluated design
# ground resonance (flexblade.ground_resonance): env FLEXBLADE_GROUND_RES=1 (set by --ground-resonance, so
# worker processes inherit it); FLEXBLADE_ARM_RANGE="lo,hi" Hz overrides the per-size default arm range
GROUND_RES = os.environ.get("FLEXBLADE_GROUND_RES") == "1"
ARM_RANGE = tuple(float(x) for x in os.environ["FLEXBLADE_ARM_RANGE"].split(",")) if os.environ.get("FLEXBLADE_ARM_RANGE") else None
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
        if BAL_VARS:   # symmetric balance-mass pair per blade (cf_invariant_mode.md recipe)
            V = [v for v in V if v.path != "flexure.strap_length"]
            V += [Variable(path="flexure.strap_length", lo=0.006, hi=0.009),
                  Variable(path="bal.mass", lo=0.0, hi=1.5e-3),
                  Variable(path="bal.r_frac", lo=0.15, hi=0.35),
                  Variable(path="bal.chord", lo=0.0, hi=0.012),
                  Variable(path="bal.normal", lo=0.0, hi=0.010)]
        if LAMINAE_VARS:   # (n, t) fatigue lever: strip stress ~ t, stiffness ~ n*w*t^3
            V = [v for v in V if v.path != "flexure.thickness"]
            V += [Variable(path="flexure.strip_laminae", lo=1.5, hi=12.5),
                  Variable(path="flexure.lamina_idx", lo=-0.5, hi=4.5)]
        if BLADE_VARS:   # light wide hollow CF-skin blade: mass/EI/GJ follow from the layup
            V += [Variable(path="blade.chord_scale", lo=0.9, hi=1.6),
                  Variable(path="blade.layup.skin_thickness", lo=0.0004, hi=0.0012, log=True),
                  Variable(path="blade.layup.core_fraction", lo=0.0, hi=1.0)]
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


MOTOR = None            # --motor: motor library name replacing motor_for_diameter (J_hub, duty slew kept)
MASS_OVERRIDE = None    # --vehicle-mass: hover thrust/g per rotor (kg)


def with_motor(d: Design, name: str | None = None) -> Design:
    """Swap in library motor `name` (default MOTOR), keeping the size's J_hub and duty_slew_max."""
    name = name or MOTOR
    if not name:
        return d
    m = mot.get_motor(name).model_copy(update=dict(J_hub=d.motor.J_hub, duty_slew_max=d.motor.duty_slew_max))
    return d.model_copy(update=dict(motor=m))


def _override(d: Design) -> Design:
    d = with_motor(d)
    if MASS_OVERRIDE:
        d = d.model_copy(update=dict(op=d.op.model_copy(update=dict(vehicle_mass=MASS_OVERRIDE))))
    return d


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
    d = _override(set_design(d, {"blade.elastic_axis_chord_frac": 0.45, "motor.duty_slew_max": RAMP_SLEW}))
    # capability from a stiff reference blade (cf_flexure) so every concept gets the same pulse
    ref = presets.make_coax_preset(D, "cf_flexure") if coax else presets.make_preset(D, "cf_flexure")
    ref = _override(set_design(ref, {"blade.elastic_axis_chord_frac": 0.45}))
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


from flexblade.stability import margin_1p_eff  # noqa: E402


HUB_MODE_FRAC = 0.15   # kinetic-energy share of hub motion that makes a mode "hub/airframe" (2P air resonance)


def np_crossings_info(r1: dict, kmax: int = 6, band: float = 0.10) -> list[str]:
    """Info only: elastic modes within ±band of nP (n = 2..kmax) at this point."""
    rpm = (r1.get("summary") or {}).get("rpm")
    out = []
    for m in r1.get("modes") or []:
        p = m["freq_hz"] / (rpm / 60.0) if rpm else 0.0
        for k in range(2, kmax + 1):
            if p > 1.5 and abs(p / k - 1) <= band:
                out.append(f"{m['label']} {p:.2f}/rev ~ {k}P (zeta {m.get('damping_ratio', float('nan')):.4f})")
    return out


def modal_participation(U: np.ndarray, Phi: np.ndarray) -> np.ndarray:
    """Share of each mode in a (complex) forced response U ~= Phi q (least squares, all modes in the
    basis): c_k = |q_k| ||Phi_k|| / sum.  U: (n,), Phi: (n, m) real mode shapes (same dof weighting)."""
    Phi = np.asarray(Phi, float)
    q = np.linalg.lstsq(Phi.astype(complex), np.asarray(U, complex), rcond=None)[0]
    c = np.abs(q) * np.linalg.norm(Phi, axis=0)
    return c / max(float(c.sum()), 1e-300)


def mechanism_mode(r1: dict) -> int | None:
    """Index (into r1["modes"]) of the cyclic MECHANISM mode = the elastic mode with the largest
    participation in the 1/rev pulse response (harmonic displacement), not chosen by label.
    Translations are weighted by 1/R so they are commensurate with rotations."""
    h = r1.get("harmonic") or {}
    ms = r1.get("modes") or []
    rpm = (r1.get("summary") or {}).get("rpm") or (r1.get("trim") or {}).get("rpm")
    if not ms or not rpm or h.get("displacement_re") is None or any(m.get("shape") is None for m in ms):
        return None
    U = np.asarray(h["displacement_re"], float) + 1j * np.asarray(h["displacement_im"], float)
    nodes = np.asarray(h.get("nodes") or [[1.0, 0, 0]], float)
    R = max(float(np.max(np.linalg.norm(nodes, axis=1))), 1e-6)
    w = np.r_[np.full(3, 1.0 / R), np.ones(3)]
    try:
        Phi = np.stack([(np.asarray(m["shape"], float) * w).ravel() for m in ms], 1)
        if Phi.shape[0] != U.size:
            return None
        c = modal_participation((U * w).ravel(), Phi)
    except Exception:  # noqa: BLE001
        return None
    f1 = rpm / 60.0
    c = np.where([m["freq_hz"] >= 0.05 * f1 for m in ms], c, -1.0)   # never the rigid spin
    return int(np.argmax(c)) if c.max() >= 0 else None


def resonance_margin(r1: dict, skip_1p: int | None = None) -> tuple[float, str]:
    """min damping-aware margin of every elastic mode to 1P (hub modes also 2P).
    skip_1p: mode index exempt from the 1P check (control_quality criterion: the mechanism
    mode is judged by cyclic phase/gain spread instead; its 2P check, if hub, is kept)."""
    rpm = (r1.get("summary") or {}).get("rpm") or (r1.get("trim") or {}).get("rpm")
    if not rpm or not r1.get("modes"):
        return float("nan"), "no modes"
    f1 = rpm / 60.0
    best, which = float("inf"), ""
    for i, m in enumerate(r1["modes"]):
        f = m["freq_hz"]
        if f < 0.05 * f1:          # rigid spin
            continue
        z = m.get("damping_ratio")
        hub = (m.get("participation") or {}).get("hub", 0.0) > HUB_MODE_FRAC
        for k in ((1, 2) if hub else (1,)):
            if k == 1 and i == skip_1p:
                continue
            mg = abs(f / (k * f1) - 1.0)
            # damping-aware (stability.margin_1p_eff): a well-damped mode near kP counts as the
            # undamped margin with the same amplification, so it is not rejected outright
            if z is not None and math.isfinite(z) and z > 0:
                mg = max(mg, margin_1p_eff(f / (k * f1), z))
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


def one_sided_check(rpms, mprs):
    """(sign_change, drift, penalty) from per-throttle rpm and mode/rev. drift = max over the
    consecutive pairs of |d ln mpr / d ln rpm|."""
    sc = any((1.0 - a) * (1.0 - b) <= 0.0 for a in mprs for b in mprs)
    dr = 0.0
    pts = sorted(zip(rpms, mprs))
    for (r0, m0), (r1, m1) in zip(pts, pts[1:]):
        if r1 > r0 and m0 > 0 and m1 > 0:
            dr = max(dr, abs(math.log(m1 / m0) / math.log(r1 / r0)))
    return sc, dr, 10.0 * max(0.0, dr - DRIFT_MAX)


def metrics(d: Design, r: dict, criterion: str | None = None) -> dict:
    """Constraint breakdown + headline metrics from a (stripped) Results dict.
    criterion: "margin" (hard 0.15 1P margin on every mode) or "control_quality" (mechanism mode
    exempt from the 1P margin; judged across the robust set by robust.control_quality)."""
    criterion = criterion or CRITERION
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
    mech_mgs = []
    mech_rr = []
    for tag, sub in _sub_results(r):
        sp = (sub.get("summary") or {}).get("trim_diff_pitch_deg", float("nan"))
        splits.append(sp)
        mg, wh = resonance_margin(sub)
        if criterion == "mech_band":
            im = mechanism_mode(sub)
            if im is not None:
                mm_ = sub["modes"][im]
                rpm_ = (sub.get("summary") or {}).get("rpm") or 1.0
                out["mech_per_rev"] = float(mm_["freq_hz"] * 60 / rpm_)
                out["mech_mode"] = f"{mm_['label']} {out['mech_per_rev']:.3f}/rev"
                out["mech_zeta"] = float(mm_.get("damping_ratio", float("nan")))
                mech_mgs.append(abs(out["mech_per_rev"] - 1.0))
                mech_rr.append((float(rpm_), out["mech_per_rev"]))
                out["res_margin_all"], out["res_which_all"] = mg, wh
                mg, wh = resonance_margin(sub, skip_1p=im)
        if criterion in ("control_quality", "control_quality_cal"):
            im = mechanism_mode(sub)
            if im is not None:
                mm_ = sub["modes"][im]
                rpm_ = (sub.get("summary") or {}).get("rpm") or 1.0
                out["mech_per_rev"] = float(mm_["freq_hz"] * 60 / rpm_)
                out["mech_mode"] = f"{mm_['label']} {mm_['freq_hz'] * 60 / rpm_:.3f}/rev"
                out["mech_zeta"] = float(mm_.get("damping_ratio", float("nan")))
                out["res_margin_all"], out["res_which_all"] = mg, wh
                mg, wh = resonance_margin(sub, skip_1p=im)
        margins.append(mg)
        which.append(wh)
    sp = max((abs(x) for x in splits if math.isfinite(x)), default=float("nan"))
    mg = min((x for x in margins if math.isfinite(x)), default=float("nan"))
    out["res_np_info"] = [x for _, sub in _sub_results(r) for x in np_crossings_info(sub)]
    s_ = r.get("summary") or {}
    out.update({k: float(s_[k]) for k in ("np_jitter_deg", "np_strip_stress_MPa", "np_strip_strain_amp",
                                         "np_strip_fat_util", "strip_strain_amp_1p") if k in s_})
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
    if GATES_MINIMAL:   # --gates minimal (gate_audit.md): no raw 1P band / one-sided drift; mech-mode zeta >= 0.02
        zm = out.get("mech_zeta", float("nan"))
        if not math.isfinite(zm) or zm < robust.MECH_ZETA_MIN:
            viol["cqc_mech_zeta"] = 1.0 if not math.isfinite(zm) else 1.0 - zm / robust.MECH_ZETA_MIN
    elif mech_mgs:
        mm_min = min(mech_mgs)
        out["mech_margin"] = mm_min
        if mm_min < MECH_MARGIN:
            viol["mech_band"] = (MECH_MARGIN - mm_min) / MECH_MARGIN
    if ONE_SIDED and not GATES_MINIMAL and len(mech_rr) > 1:
        sc, dr, pen = one_sided_check([a for a, _ in mech_rr], [b for _, b in mech_rr])
        out["mech_drift"], out["drift_pen"] = dr, pen
        if sc:
            viol["mech_one_sided"] = 1.0
        if pen > 0:
            viol["mech_drift"] = pen
    if GROUND_RES:
        from flexblade import ground_resonance as gr
        af = d.op.airframe
        rng = ARM_RANGE or gr.default_arm_range(d.blade.radius)
        sm = r.get("summary") or {}
        gv = gr.violation(gr.lag_per_rev(r.get("modes"), float(sm.get("rpm", float("nan")))), float(sm.get("rpm", float("nan"))),
                          rng, af, 0.0, d.op.vehicle_mass or 1.0)
        out["ground_res"] = gv
        if gv > 0:
            viol["ground_resonance"] = gv
    nconv = sum(("not converge" in w) or ("failed" in w) for w in r.get("warnings", []))
    if nconv:
        viol["convergence"] = float(nconv)
    viol.update({"geom:" + k: v for k, v in geometry_violations(d).items()})
    out["viol"] = viol
    s = r.get("summary") or {}
    h = r.get("harmonic") or {}
    out["control_ratio"] = float(s.get("control_ratio", h.get("control_ratio", 0.0)) or 0.0)
    if s.get("trim_ok") is False or not math.isfinite(out["control_ratio"]):
        # no valid trim (e.g. elastic pitch collapse): infeasible, never a NaN objective
        out["control_ratio"] = 0.0
        viol["trim_failed"] = 1.0
        out["feasible"] = False
    out["tilt_equiv_deg"] = float(h.get("tilt_equiv_deg", float("nan")))
    out["pitch_amp_deg"] = float(s.get("pitch_amp_deg", float("nan")))
    out["lag_amp_deg"] = float(s.get("lag_amp_deg", float("nan")))
    out["lag_per_rev"] = float(s.get("lag_freq_per_rev", float("nan")))
    out["flap_per_rev"] = float(s.get("flap_freq_per_rev", float("nan")))
    out["pitch_per_rev"] = float(s.get("pitch_freq_per_rev", float("nan")))
    out["moment_phase_deg"] = float(s.get("hub_moment_phase_deg", float("nan")))
    # complex cyclic gain G = mean fixed-frame hub moment / 1P torque pulse (rotor.harmonic_1p):
    # |G| = moment_per_torque, arg G = moment azimuth relative to the command (torque) peak
    out["cyc_gain"] = float(s.get("moment_per_torque", h.get("moment_per_torque", float("nan"))))
    out["cyc_phase_deg"] = out["moment_phase_deg"]
    out["criterion"] = criterion
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
    add_pulse_limited(d, r, out)
    return out


def score(m: dict) -> float:
    if OBJECTIVE == "pulse_limited":
        return score_pl(m)
    if m["feasible"]:
        return -100.0 * m["control_ratio"]
    return 1.0 + float(sum(m["viol"].values()))


# --------------------------------------------------------------------------- pulse-limited scoring
# The 1/rev pulse amplitude is an in-flight software knob (ESC/FC gain), so a design is scored
# by the authority it delivers when the pulse is scaled (factor s, s = 1 is the evaluated pulse)
# until the most-utilised PULSE-SCALING check reaches 1.0.  Classification verified in code:
#   motor_torque_1p  |dQ|/cap, dQ = G(w)*dd with dd ∝ pulse (motor.duty_drive)       u ∝ s
#                    -> this check IS the motor-capability cap on s
#   esc_duty_slew    dd*w / slew_max                                                   u ∝ s
#   esc_duty_peak    (d0 + dd)/0.95, d0 = steady duty (motor.steady_duty)             u = (d0 + dd s)/0.95
#   motor_thermal    rise ∝ R(I0² + ½(c dI)²) + b w², dI ∝ pulse (motor.thermal)      u = u0 + (u1-u0) s²
#   fatigue_strain/  srss(1P amp ∝ s [linear harmonic solve, no mean-stress term],
#   fatigue_stress   nP forced = motor nP torque (∝ s) ⊕ aero 2P/imbalance (fixed))   ~ s, bounded below
#   harmonic_jitter  (NOT pulse-scaled: hard check evaluated once at s=1; was) jitter = motor-torque part (∝ s) ⊕ aero/imbalance part (fixed), allowable
#                    max(0.3°, 0.1 × 1P pitch ∝ s)                                     mixed, bounded below
#   control_ratio    linear 1/rev harmonic response to dQ                               cr ∝ s
# For the mixed (fixed ⊕ ∝s) parts the true value lies between x and x·s; we take the conservative
# side (x·max(s,1)), so cr_pl never over-states authority.
# Everything else is NON-scaling (static/creep/interlayer/strap/clamp wrap from the static trim
# state, stability zetas, pitch_cf_ratio, 1P resonance margin, trim split, geometry): hard.
# One throttle point uses ONE pulse scale for all its checks; across throttle the scale may differ
# (robust.combine takes min cr_pl over points): the FC schedules the pulse gain with throttle.
PULSE_EXP = {"motor_torque_1p": 1, "esc_duty_slew": 1, "esc_duty_peak": 1, "motor_thermal": 2,
             "fatigue_strain": 1, "fatigue_stress": 1}
PULSE_S_MAX = 50.0
_NP_RE = re.compile(r"1P ([0-9.eE+-]+) \+ nP ([0-9.eE+-]+)")


def pulse_util_fns(d: Design, r: dict) -> list[tuple[str, object]]:
    """-> [(check name, u(s))] for every pulse-scaling check instance at this point."""
    from flexblade import forced as frc
    s_ = r.get("summary") or {}
    rpm = s_.get("rpm")
    Q = (r.get("trim") or {}).get("torque")
    fns = []
    for c in r.get("checks", []):
        nm, u1 = c["name"], float(c["utilisation"])
        if nm not in PULSE_EXP:
            continue
        if not math.isfinite(u1):
            u1 = 1e3
        f = None
        if nm == "motor_thermal" and rpm and Q is not None:
            u0 = min(mot.thermal(d.motor, rpm, Q, 0.0, d.op.temperature_C)["utilisation"], u1)
            f = (lambda s, u0=u0, u1=u1: u0 + (u1 - u0) * s * s)
        elif nm == "esc_duty_peak" and rpm and Q is not None:
            d0 = mot.steady_duty(d.motor, rpm, Q)
            dd = max(0.95 * u1 - d0, 0.0)
            f = (lambda s, d0=d0, dd=dd: (d0 + dd * s) / 0.95)
        elif nm.startswith("fatigue_"):
            mm = _NP_RE.search(c.get("note") or "")
            if mm:
                a, b = float(mm.group(1)), float(mm.group(2))
                f = (lambda s, a=a, b=b: frc.combine(a * s, b * max(s, 1.0)))
        if f is None:
            p = PULSE_EXP[nm]
            f = (lambda s, u1=u1, p=p: u1 * s ** p)
        fns.append((nm, f))
    return fns


def pulse_scale(fns, s_max: float = PULSE_S_MAX) -> tuple[float, str]:
    """Largest s in [0, s_max] with every u(s) <= 1 (all u monotone non-decreasing in s).
    -> (s, binding check name); s = 0 if some check exceeds 1 even at vanishing pulse."""
    if not fns:
        return s_max, "none"
    if all(f(s_max) <= 1.0 for _, f in fns):
        return s_max, "none"
    if any(f(1e-9) > 1.0 for _, f in fns):
        return 0.0, max(fns, key=lambda t: t[1](1e-9))[0]
    lo, hi = 1e-9, s_max
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if all(f(mid) <= 1.0 for _, f in fns):
            lo = mid
        else:
            hi = mid
    return lo, max(fns, key=lambda t: t[1](hi))[0]


def add_pulse_limited(d: Design, r: dict, out: dict) -> None:
    """cr_pl = cr × s* (one pulse scale for all checks of this throttle point) + hard viols."""
    s, bind = pulse_scale(pulse_util_fns(d, r))
    out["pulse_scale"] = s
    out["pulse_binding"] = bind
    pcm = d.op.pulse_capability_margin
    out["pulse_frac_cap"] = s * pcm if pcm is not None else float("nan")   # of motor 1P capability
    out["cr_pl"] = out["control_ratio"] * s
    hard = {k: v for k, v in out["viol"].items()
            if not (k.startswith("check:") and k[6:] in PULSE_EXP)}
    if s <= 0.0:
        hard["pulse_infeasible"] = 1.0
    out["viol_hard"] = hard
    out["feasible_pl"] = not hard


def score_pl(m: dict) -> float:
    if m.get("feasible_pl"):
        return -100.0 * m["cr_pl"]
    return 1.0 + float(sum(m.get("viol_hard", {}).values()))


# --------------------------------------------------------------------------- driver


class Logger:
    """Appends records as COMPLETE gzip members (buffered, flushed every 200 records / 60 s).

    A single long-lived gzip stream left a truncated member when a run was killed, and every
    member appended after it became unreadable (2026-11-21: ~70k records lost from the archive;
    the SQLite eval cache is the authoritative store). Now a kill loses at most the buffer.
    """

    def __init__(self, path: Path):
        self.path = path
        self.buf: list[str] = []
        self.t_flush = time.time()
        self.lock = threading.Lock()

    def write(self, rec: dict):
        with self.lock:
            self.buf.append(json.dumps(rec, default=float) + "\n")
            if len(self.buf) >= 200 or time.time() - self.t_flush > 60:
                self._flush()

    def _flush(self):
        if self.buf:
            data = gzip.compress("".join(self.buf).encode())
            with open(self.path, "ab") as fh:
                fh.write(data)
                fh.flush()
                os.fsync(fh.fileno())
            self.buf = []
        self.t_flush = time.time()

    def close(self):
        with self.lock:
            self._flush()


def _round(v: dict) -> dict:
    return {k: (round(x, 7) if isinstance(x, float) else x) for k, x in v.items()}


ROBUST = None   # thrust factors tuple when --robust
CRITERION = "margin"   # or "control_quality" (robust.control_quality for the mechanism mode)
OBJECTIVE = "cr"   # or "pulse_limited" (cr_pl; hard = non-scaling checks only)
PERTURB = None  # robust tolerance samples (robust.CQ_PERTURB under --criterion control_quality[_cal])
RUNTIME = None  # runtime samples about each calibrated node (robust.CQ_RUNTIME, control_quality_cal)
INIT_DESIGNS: list = []   # --init-designs: Design objects seeding DE (matched by diameter_in)
BASE_DESIGNS: list = []   # --base-design: fixed-field overrides (matched by diameter_in)
INIT_FRAC = 0.3
J_HUB_BY_D: dict = {}     # --j-hub "7:a,b,c;10:d,e,f" size-specific sets


def _jhub(D):
    return J_HUB_BY_D.get(float(D), J_HUB)


J_HUB = None    # tuple of motor.J_hub values: robust/tolerance dimension


def _goodman(d: Design) -> Design:
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(strip_fatigue_mean="goodman"))))


RESOLVE_PRESETS = 0       # --resolve-presets N: iterations (0 = off)
PRESET_ALPHA = 0.55       # v8 convention: strip_preset_deg = alpha * zero-stress slot angles


def resolve_candidate(d: Design, iters: int, alpha: float = PRESET_ALPHA, info: dict | None = None) -> Design:
    """Re-solve zero-stress strip presets at the candidate's nominal op (T1.0), then scale by alpha."""
    from flexblade import rotor
    s0 = [[[x / alpha for x in e] for e in m] for m in d.flexure.strip_preset_deg]   # warm start
    dz = rotor.resolve_strip_preset(d.model_copy(update=dict(flexure=d.flexure.model_copy(
        update=dict(strip_preset_deg=s0)))), iters=iters, info=info)
    p = [[[alpha * x for x in e] for e in m] for m in dz.flexure.strip_preset_deg]
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(strip_preset_deg=p))))


def _resolve_one(d: Design) -> Design:
    try:
        return resolve_candidate(d, RESOLVE_PRESETS)
    except Exception:  # noqa: BLE001  keep stale presets rather than lose the candidate
        return d


RESOLVE_WORKERS = int(os.environ.get("RESOLVE_WORKERS", "16"))


def _resolve_all(designs: list[Design]) -> list[Design]:
    """Processes (fork), not threads: the re-solve is GIL-bound Python. Runs between DE
    evaluation phases, so it reuses the idle local cores without oversubscribing."""
    if not RESOLVE_PRESETS:
        return designs
    n = max(1, min(len(designs), RESOLVE_WORKERS))
    if n == 1:
        return [_resolve_one(d) for d in designs]
    import multiprocessing as mp
    with mp.get_context("fork").Pool(n) as pool:
        return pool.map(_resolve_one, designs, chunksize=1)


def run_case(case: str, D: float, concept: str, coax: bool, pool, log: Logger, popmult: int,
             maxiter: int, seed: int) -> dict:
    from scipy.optimize import differential_evolution

    t0 = time.time()
    base, cap = base_design(D, concept, coax)
    leads = _lead_for(INIT_DESIGNS, D)
    bo = _lead_for(BASE_DESIGNS, D)
    if bo:   # fields outside the param space (laminae, balance pins, mount, clamp...) fixed from the lead
        tmf = base.op.torque_mod_frac
        base = bo[0].model_copy(update=dict(name=base.name))
        if MOTOR:   # lead's motor -> override; pulse re-derived from the override's capability
            base = set_design(with_motor(base), {"op.torque_mod_frac": tmf})
    if GOODMAN:
        base = _goodman(base)
    V = variables(concept, D)
    obj = Objective(OptimizeRequest(design=base, variables=V,
                                    targets=[Target(quantity="harmonic.control_ratio", kind="max")]))
    best = dict(J=float("inf"))
    n = [0]
    lock = threading.Lock()
    gerr = [0.0]

    def mapper(_f, pop):
        pop = [np.asarray(u, float) for u in pop]
        designs = _resolve_all([set_design(base, obj.to_values(u)) for u in pop])
        recs = robust.map_metrics(pool, designs, metrics, ROBUST, PERTURB, _jhub(D), RUNTIME)
        Js = []
        gerr[0] = sum(mm is None or "error" in mm for mm, _ in recs) / max(len(recs), 1)
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
              f"viol {_short(b.get('viol', {}))} err {gerr[0]:.2f}  {time.time() - t0:.0f}s", flush=True)

    init = "latinhypercube"
    if leads:
        init = seeded_population(obj, leads, popmult * len(V), INIT_FRAC, seed)
        u0 = None
        print(f"[{case}] seeding {int(round(INIT_FRAC * len(init)))} of {len(init)} from {len(leads)} lead(s)", flush=True)
    r = differential_evolution(lambda u: 0.0, [(0.0, 1.0)] * len(V), maxiter=maxiter, popsize=popmult,
                               seed=seed, x0=u0, polish=False, tol=1e-4, atol=0.0,
                               init=init, mutation=(0.5, 1.0), recombination=0.7,
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


def load_design(path) -> Design:
    return Design.model_validate_json(Path(path).read_text())


def encode_design(obj: Objective, d: Design) -> np.ndarray:
    """Design -> DE unit vector (inverse of obj.to_values -> set_design); out-of-range values clip."""
    return obj.to_u({v["path"]: min(max(float(_get(d, v["path"])), v["lo"]), v["hi"]) for v in obj.variables})


def seeded_population(obj: Objective, leads: list[Design], npop: int, frac: float, seed: int,
                      sigma: float = 0.05) -> np.ndarray:
    """LHS population of npop with round(frac*npop) members replaced by the lead vectors plus
    Gaussian(sigma*range) perturbations (leads themselves first). Valid scipy init= array."""
    from scipy.stats import qmc
    nv = len(obj.variables)
    pop = qmc.LatinHypercube(d=nv, seed=seed).random(npop)
    k = min(npop, max(len(leads), int(round(frac * npop)))) if leads else 0
    rng = np.random.default_rng(seed)
    U = [encode_design(obj, d) for d in leads]
    for i in range(k):
        u = U[i] if i < len(U) else np.clip(U[i % len(U)] + rng.normal(0, sigma, nv), 0.0, 1.0)
        pop[i] = u
    return pop


def _lead_for(designs: list[Design], D: float) -> list[Design]:
    return [d for d in designs if abs(d.blade.diameter_in - D) < 1e-6]


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
    ap.add_argument("--objective", choices=("cr", "pulse_limited"), default="cr",
                    help="pulse_limited: maximise cr at the pulse that binds the most-utilised "
                         "pulse-scaling check (per-throttle pulse schedule)")
    ap.add_argument("--criterion", choices=("margin", "control_quality", "control_quality_cal", "mech_band"),
                    default="margin",
                    help="control_quality: the mechanism mode is judged by cyclic phase/gain spread over "
                         "throttle x J x 4 fixed +-5%% tolerance samples instead of the 1P margin; "
                         "control_quality_cal: build tolerance calibrated out per airframe, only the "
                         "RUNTIME residual (rpm bin, temperature, creep, J error) about each calibrated "
                         "node is limited; build samples bound calibratability (authority, 1P side)")
    ap.add_argument("--one-sided", action="store_true",
                    help="mech_band: infeasible if mode/rev crosses 1P over the throttle range; "
                         "penalty 10*max(0,|dln mpr/dln rpm|-0.07); adds the bal.* balance-pair variables")
    ap.add_argument("--blade-vars", action="store_true",
                    help="steelx: add blade.chord_scale, blade.layup.skin_thickness/core_fraction variables "
                         "(blade must be a core_skin layup; see study/results/de10_prep.md)")
    ap.add_argument("--laminae-vars", action="store_true",
                    help="steelx: vary flexure.strip_laminae (int 2-12) and lamina thickness "
                         "(index snapped to stock 0.05/0.075/0.10/0.127/0.15 mm)")
    ap.add_argument("--gates", choices=("default", "minimal"), default="default",
                    help="minimal: drop mech_band / one-sided gates, add zeta_mech >= 0.02; keeps bal.* variables")
    ap.add_argument("--mech-margin", type=float, default=0.075,
                    help="mech_band: nominal hard 1P margin of the mechanism mode (others keep 0.15)")
    ap.add_argument("--motor", default=None, help="motor.MOTOR_LIBRARY name for every case (e.g. crm2204_1400kv)")
    ap.add_argument("--vehicle-mass", type=float, default=None, help="kg of hover thrust per rotor (overrides preset)")
    ap.add_argument("--goodman", action="store_true", help='strip_fatigue_mean="goodman" on every design')
    ap.add_argument("--j-hub", default=None, help="comma list of motor.J_hub values (robust dimension)")
    ap.add_argument("--ground-resonance", action=argparse.BooleanOptionalAction, default=None,
                    help="(default on with --objective pulse_limited; ~3 ms/eval) hard check: regressing lag vs the arm/mount mode over the arm-frequency envelope")
    ap.add_argument("--arm-range", default=None, help="lo,hi Hz arm envelope (default 7in 200-300, 10in 120-250)")
    ap.add_argument("--init-designs", default=None, help="comma list of design JSONs seeding the DE population")
    ap.add_argument("--init-from-leads", action="store_true", help="seed from the 7in v6 / 10in v5 leads")
    ap.add_argument("--init-frac", type=float, default=0.3, help="fraction of population seeded (default 0.3)")
    ap.add_argument("--base-design", default=None, help="comma list of design JSONs whose non-DE fields stay fixed "
                    "(matched by diameter; 'init' = use --init-designs)")
    ap.add_argument("--resolve-presets", nargs="?", const=6, default=0, type=int, metavar="N",
                    help="re-solve zero-stress strip presets (N iterations, default 6) at T1.0 per "
                         "candidate, scaled by alpha=0.55 (v8 convention); default off")
    ap.add_argument("--no-single", action="store_true")
    ap.add_argument("--popmult", type=int, default=8)
    ap.add_argument("--maxiter", type=int, default=40)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--threads", type=int, default=24)
    ap.add_argument("--redo", action="store_true", help="re-run cases already in cases.jsonl")
    ap.add_argument("--salvage", nargs="*", help="rebuild these cases from evals.jsonl.gz and exit")
    a = ap.parse_args(argv)
    global ROBUST, OBJECTIVE, J_HUB, CRITERION, PERTURB, RUNTIME
    OBJECTIVE = a.objective
    global GROUND_RES, ARM_RANGE
    if a.ground_resonance or (a.ground_resonance is None and a.objective == "pulse_limited"):
        os.environ["FLEXBLADE_GROUND_RES"] = "1"
        GROUND_RES = True
    if a.arm_range:
        os.environ["FLEXBLADE_ARM_RANGE"] = a.arm_range
        ARM_RANGE = tuple(float(x) for x in a.arm_range.split(","))
    CRITERION = a.criterion
    global MOTOR
    MOTOR = a.motor
    global ONE_SIDED, BAL_VARS, BLADE_VARS, LAMINAE_VARS
    BLADE_VARS = a.blade_vars
    LAMINAE_VARS = a.laminae_vars
    global GATES_MINIMAL
    GATES_MINIMAL = a.gates == "minimal"
    ONE_SIDED = a.one_sided
    BAL_VARS = a.one_sided or GATES_MINIMAL
    global MECH_MARGIN, GOODMAN
    MECH_MARGIN, GOODMAN = a.mech_margin, a.goodman
    global RESOLVE_PRESETS
    RESOLVE_PRESETS = a.resolve_presets
    global MASS_OVERRIDE
    MASS_OVERRIDE = a.vehicle_mass
    PERTURB = list(robust.CQ_PERTURB) if CRITERION.startswith("control_quality") else None
    RUNTIME = list(robust.CQ_RUNTIME) if CRITERION == "control_quality_cal" else None
    global INIT_DESIGNS, BASE_DESIGNS, INIT_FRAC, J_HUB_BY_D
    if a.j_hub and ":" in a.j_hub:
        J_HUB_BY_D = {float(seg.split(":")[0]): tuple(float(x) for x in seg.split(":")[1].split(","))
                      for seg in a.j_hub.split(";")}
        J_HUB = None
    else:
        J_HUB = tuple(float(x) for x in a.j_hub.split(",")) if a.j_hub else None
    paths = [x for x in (a.init_designs or "").split(",") if x]
    if a.init_from_leads:
        paths += [str(DESIGNS / "7in_tt_strap_steelx_v6.json"), str(DESIGNS / "10in_tt_strap_steelx_v5.json")]
    INIT_DESIGNS = [load_design(x) for x in paths]
    INIT_FRAC = a.init_frac
    BASE_DESIGNS = INIT_DESIGNS if a.base_design == "init" else [load_design(x) for x in (a.base_design or "").split(",") if x]
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
    failed = []
    with remote.HostPool(a.hosts) as pool:
        def one(c):
            try:
                out = run_case(*c, pool=pool, log=log, popmult=a.popmult, maxiter=a.maxiter, seed=a.seed)
            except Exception as ex:  # noqa: BLE001
                traceback.print_exc()
                out = dict(case=c[0], error=f"{type(ex).__name__}: {ex}")
                failed.append(c[0])
            with lock:
                with open(cases_path, "a") as fh:
                    fh.write(json.dumps(out, default=float) + "\n")
            print(f"DONE {c[0]}: J {out.get('J')} cr {out.get('control_ratio')} feasible "
                  f"{out.get('feasible')}", flush=True)
            return out
        with ThreadPoolExecutor(max_workers=a.threads) as ex:
            list(ex.map(one, cases))
    log.close()
    if failed:
        print(f"FAILED {len(failed)}/{len(cases)} cases: {failed}", file=sys.stderr, flush=True)
        sys.exit(2)


if __name__ == "__main__":
    main()
