"""Counter-rotating coaxial pair (OWNER: aero).

Upper rotor = the Design's primary definition (spins +Ω about +z).  The lower
rotor (spins −Ω, axial spacing s = spacing_frac·R below) is the same single-rotor
problem in its own mirrored frame (reflection y → −y), built by `lower_design`.
The rotors are coupled only through imposed axial inflow (`op.extra_inflow`) and
their thrust shares (`op.thrust_share`); each is then run through the normal
single-rotor evaluator (`rotor.evaluate`, falling back to aero-only trim).

Interference model (momentum / actuator-disk superposition; cf. Leishman,
*Principles of Helicopter Aerodynamics* 2nd ed. §2.14 coaxial momentum theory,
and Leishman & Syal, JAHS 2008 "Figure of merit definition for coaxial rotors"):

* Lower rotor in the upper wake.  Upper slipstream radius at the lower plane
  (Landgrebe prescribed-wake contraction):
      r_c/R = A + (1 − A) exp(−Λ ψ_w),  A = 0.78,  Λ = 0.145 + 27 C_T,U
  with wake age ψ_w = (s/R)/k₂, tip-vortex descent rate k₂ = 1.41 √(C_T,U/2)
  per radian (Landgrebe's far-wake rate, twist term dropped).  Inside r_c the
  wake velocity follows continuity along stream tubes from the upper disk:
      v_w(r) = v̄_U(r R/r_c) · (R/r_c)²,   v̄_U = F·v_own,U (annulus-mean induced)
  (→ 2v̄ for r_c = R/√2, the fully developed slipstream of simple momentum
  theory); zero outside, with a tanh edge of width 0.03R.
* Upper rotor in the lower rotor's upstream induced field (vortex-cylinder axial
  decay of an actuator disk):
      v_up(r) = v̄_L(r) · (1 − s/√(s² + R_L²))
  (≈ 0.76 v̄_L at s = 0.25R; the on-axis value is used across the disk).
* Each rotor's own BEMT treats the imposed velocity like a climb velocity
  (aero._solve_phi), so its induced power rises with the extra mass flow.

Leishman's ideal result for equal-thrust, fully-developed-wake coax
(κ_int ≈ 1.28 on induced power) bounds the interference_power_factor here
(which divides *total* power, incl. profile power, by two isolated rotors at the
same thrusts, so it is lower than κ_int at equal profile power).

Trim (`design.coaxial.trim`):
  torque_balance — find thrust split and both rpm: T_U + T_L = m g, Q_U = Q_L.
  equal_rpm      — common rpm for T_U + T_L = m g (net yaw torque reported).
  fixed_rpm      — op.rpm (upper) and coaxial.lower_rpm; thrust not matched.
Fixed-point iteration on the two inflow profiles (relaxed), with the share
updated by T_i ∝ Q_i^{2/3} toward equal torque.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np
from scipy.optimize import brentq

from .aero import G0, AeroModel, AeroTrim
from .motor import capability
from .schema import (CoaxialResult, Design, HarmonicResult, InflowProfile, Results, TrimResult)

CONTRACTION_A = 0.78
EDGE_WIDTH = 0.03


# ============================================================================ designs


def lower_design(design: Design, rpm: Optional[float] = None, share: Optional[float] = None,
                 inflow: Optional[InflowProfile] = None) -> Design:
    """Single-rotor Design of the lower rotor in its own (mirrored) frame."""
    cx = design.coaxial
    d = design.model_copy(deep=True)
    upd = dict(layout="single", name=f"{design.name}:lower")
    if cx.lower_blade is not None:
        upd["blade"] = cx.lower_blade.model_copy(deep=True)
    if cx.lower_flexure is not None:
        upd["flexure"] = cx.lower_flexure.model_copy(deep=True)
    if cx.lower_motor is not None:
        upd["motor"] = cx.lower_motor.model_copy(deep=True)
    op = d.op.model_copy(update=dict(
        collective_deg=d.op.collective_deg + cx.lower_collective_offset_deg,
        rpm=rpm, thrust_share=share, extra_inflow=inflow,
        torque_mod_frac=(cx.lower_torque_mod_frac if cx.lower_torque_mod_frac is not None
                         else d.op.torque_mod_frac)))
    upd["op"] = op
    return d.model_copy(update=upd)


def upper_design(design: Design, rpm: Optional[float] = None, share: Optional[float] = None,
                 inflow: Optional[InflowProfile] = None) -> Design:
    d = design.model_copy(deep=True)
    op = d.op.model_copy(update=dict(rpm=rpm, thrust_share=share, extra_inflow=inflow))
    return d.model_copy(update=dict(layout="single", name=f"{design.name}:upper", op=op))


# ============================================================================ interference


def wake_contraction(trim_U: AeroTrim, spacing: float) -> float:
    """Upper slipstream radius r_c/R at axial distance `spacing` [m] (Landgrebe)."""
    ct = max(trim_U.CT, 1e-5)
    lam = np.sqrt(ct / 2)
    psi_w = (spacing / trim_U.R) / (1.41 * lam)
    Lam = 0.145 + 27.0 * ct
    return CONTRACTION_A + (1 - CONTRACTION_A) * np.exp(-Lam * psi_w)


def _mean_own(trim: AeroTrim) -> np.ndarray:
    return trim.F_loss * trim.v_own


def upper_wake_on_lower(trim_U: AeroTrim, r_L: np.ndarray, spacing: float) -> np.ndarray:
    """Axial velocity [m/s, + down] of the upper wake at the lower rotor stations."""
    R = trim_U.R
    rc = wake_contraction(trim_U, spacing) * R
    vbar = _mean_own(trim_U)
    r_src = np.asarray(r_L) * R / rc
    v = np.interp(r_src, trim_U.r, vbar, left=vbar[0], right=0.0) * (R / rc) ** 2
    edge = 0.5 * (1 - np.tanh((np.asarray(r_L) - rc) / (EDGE_WIDTH * R)))
    return v * edge


def lower_upstream_on_upper(trim_L: AeroTrim, r_U: np.ndarray, spacing: float) -> np.ndarray:
    """Axial velocity [m/s, + down] induced at the upper plane by the lower rotor."""
    f = 1.0 - spacing / np.sqrt(spacing**2 + trim_L.R**2)
    vbar = _mean_own(trim_L)
    return f * np.interp(np.asarray(r_U), trim_L.r, vbar, left=0.0, right=0.0)


def _profile(r: np.ndarray, R: float, v: np.ndarray) -> InflowProfile:
    return InflowProfile(r_frac=[float(x) for x in r / R], v=[float(x) for x in v])


# ============================================================================ pair trim


@dataclass
class PairTrim:
    upper: AeroTrim
    lower: AeroTrim
    share_upper: float
    v_ext_upper: np.ndarray
    v_ext_lower: np.ndarray
    r_c: float
    iterations: int
    converged: bool
    aero_upper: AeroModel = field(repr=False)
    aero_lower: AeroModel = field(repr=False)


def pair_trim(design: Design, max_iter: int = 60, tol: float = 1e-4, relax: float = 0.6) -> PairTrim:
    """Aero-only coupled trim of the coaxial pair (see module docstring)."""
    cx = design.coaxial
    aU = AeroModel(upper_design(design))
    aL = AeroModel(lower_design(design))
    s = cx.spacing_frac * aU.R
    W = design.op.vehicle_mass * G0
    mode = cx.trim
    if mode == "fixed_rpm" and (design.op.rpm is None or cx.lower_rpm is None):
        raise ValueError("coaxial.trim='fixed_rpm' needs op.rpm and coaxial.lower_rpm")
    share = 0.5
    vU = np.zeros_like(aU.r)
    vL = np.zeros_like(aL.r)
    conv = False
    tU = tL = None
    for it in range(1, max_iter + 1):
        aU.v_ext, aL.v_ext = vU, vL
        if mode == "torque_balance":
            tU = aU.trim(thrust=share * W)
            tL = aL.trim(thrust=(1 - share) * W)
            qm = np.sqrt(tU.Q * tL.Q)
            TU = tU.T * (qm / tU.Q) ** (2 / 3)
            TL = tL.T * (qm / tL.Q) ** (2 / 3)
            new_share = TU / (TU + TL)
        elif mode == "equal_rpm":
            f = lambda n: aU.trim(rpm=n).T + aL.trim(rpm=n).T - W  # noqa: E731
            g = tU.rpm if tU is not None else aU.trim(thrust=W / 2).rpm
            n = brentq(f, 0.6 * g, 1.6 * g, xtol=1e-3)
            tU, tL = aU.trim(rpm=n), aL.trim(rpm=n)
            new_share = tU.T / (tU.T + tL.T)
        else:
            tU, tL = aU.trim(rpm=design.op.rpm), aL.trim(rpm=cx.lower_rpm)
            new_share = tU.T / max(tU.T + tL.T, 1e-9)
        vU_new = lower_upstream_on_upper(tL, aU.r, s)
        vL_new = upper_wake_on_lower(tU, aL.r, s)
        scale = max(np.abs(vL_new).max(), 1e-6)
        dv = max(np.abs(vU_new - vU).max(), np.abs(vL_new - vL).max()) / scale
        ds = abs(new_share - share)
        vU = vU + relax * (vU_new - vU)
        vL = vL + relax * (vL_new - vL)
        share = share + relax * (new_share - share)
        if dv < tol and ds < tol:
            conv = True
            break
    # final consistent evaluation with the converged inflows
    aU.v_ext, aL.v_ext = vU, vL
    if mode == "torque_balance":
        tU, tL = aU.trim(thrust=share * W), aL.trim(thrust=(1 - share) * W)
    elif mode == "equal_rpm":
        f = lambda n: aU.trim(rpm=n).T + aL.trim(rpm=n).T - W  # noqa: E731
        n = brentq(f, 0.7 * tU.rpm, 1.4 * tU.rpm, xtol=1e-3)
        tU, tL = aU.trim(rpm=n), aL.trim(rpm=n)
    else:
        tU, tL = aU.trim(rpm=design.op.rpm), aL.trim(rpm=cx.lower_rpm)
    share = tU.T / (tU.T + tL.T)
    return PairTrim(tU, tL, share, vU, vL, wake_contraction(tU, s), it, conv, aU, aL)


def yaw_authority(design: Design, pt: PairTrim, n: int = 15) -> tuple[float, float, float]:
    """(authority, yaw_min, yaw_max): net airframe yaw torque range (Q_L − Q_U) over
    thrust splits that keep T_U + T_L = m g with both motors within their steady
    torque limit (`motor.capability(...).Q_mean_max` at the new rpm).  Inflows are
    frozen at the trimmed interference (first-order).  authority = min distance
    from the trimmed yaw torque to either end (symmetric authority)."""
    W = pt.upper.T + pt.lower.T
    aU, aL = pt.aero_upper, pt.aero_lower
    mU = aU.design.motor
    mL = aL.design.motor
    y0 = pt.lower.Q - pt.upper.Q
    ys = [y0]
    for s in np.linspace(0.05, 0.95, n):
        try:
            tU, tL = aU.trim(thrust=s * W), aL.trim(thrust=(1 - s) * W)
        except ValueError:
            continue
        if tU.Q <= capability(mU, tU.rpm, 0).Q_mean_max and tL.Q <= capability(mL, tL.rpm, 0).Q_mean_max:
            ys.append(tL.Q - tU.Q)
    lo, hi = min(ys), max(ys)
    return float(min(y0 - lo, hi - y0)), float(lo), float(hi)


# ============================================================================ results


def aero_only_evaluate(design: Design, what: list[str] | None = None) -> Results:
    """Single-rotor Results with trim only, from AeroModel (fallback evaluator)."""
    t = AeroModel(design).trim()
    res = Results(design_hash=design.hash())
    res.trim = TrimResult(rpm=t.rpm, thrust=t.T, torque=t.Q, power=t.P, figure_of_merit=t.FoM,
                          collective_deg=float(np.degrees(t.collective)),
                          centrifugal_force_root=0.0, stations=t.stations_dict())
    res.summary = {k: float(v) for k, v in t.summary().items()}
    res.warnings.append("aero-only evaluation (no structural model): no modes/harmonic/checks")
    return res


def _single_evaluator() -> Callable[[Design, list[str]], Results]:
    try:
        from . import rotor
        return rotor.evaluate
    except (ImportError, AttributeError):
        return aero_only_evaluate


def _mirror_vec6(v):
    v = list(v)
    if len(v) == 6:   # reflection y → −y: [ux, −uy, uz, −rx, ry, −rz]
        return [v[0], -v[1], v[2], -v[3], v[4], -v[5]]
    return v


def _mirror_results(res: Results, dz: float) -> Results:
    """Express lower-rotor Results (own mirrored frame) in the upper hub frame:
    reflect y → −y (so +Ω becomes −Ω), shift nodes by −dz, negate phases."""
    r = res.model_copy(deep=True)
    h = r.harmonic
    if h is not None:
        upd = dict(hub_moment_phase_deg=-h.hub_moment_phase_deg,
                   blades=[b.model_copy(update=dict(pitch_phase_deg=-b.pitch_phase_deg,
                                                    lag_phase_deg=-b.lag_phase_deg,
                                                    flap_phase_deg=-b.flap_phase_deg))
                           for b in h.blades])
        if h.nodes is not None:
            upd["nodes"] = [(p[0], -p[1], p[2] - dz) for p in h.nodes]
        for k in ("displacement_re", "displacement_im"):
            v = getattr(h, k)
            if v is not None:
                upd[k] = [_mirror_vec6(x) for x in v]
        r.harmonic = h.model_copy(update=upd)
    for m in r.modes:
        if m.shape is not None:
            m.shape = [_mirror_vec6(x) for x in m.shape]
    return r


def _moment(h: Optional[HarmonicResult], attr: str) -> float:
    return float(getattr(h, attr)) if h is not None else 0.0


def evaluate(design: Design, what: list[str] | None = None,
             single: Optional[Callable[[Design, list[str]], Results]] = None) -> Results:
    """Coaxial evaluation → Results with `.coaxial` (CoaxialResult), upper rotor's
    trim/harmonic at top level, combined numbers in `summary`, warnings prefixed
    "upper:" / "lower:".  `single` overrides the single-rotor evaluator
    (default rotor.evaluate, else aero-only)."""
    what = list(what or ["trim", "modes", "harmonic", "checks"])
    t0 = time.perf_counter()
    single = single or _single_evaluator()
    cx = design.coaxial
    pt = pair_trim(design)
    t_pair = time.perf_counter() - t0
    RU, RL = pt.upper.R, pt.lower.R
    spacing = cx.spacing_frac * RU
    share = pt.share_upper
    dU = upper_design(design, rpm=pt.upper.rpm, share=share,
                      inflow=_profile(pt.aero_upper.r, RU, pt.v_ext_upper))
    dL = lower_design(design, rpm=pt.lower.rpm, share=1 - share,
                      inflow=_profile(pt.aero_lower.r, RL, pt.v_ext_lower))
    warnings: list[str] = []
    try:
        rU = single(dU, what)
        rL = single(dL, what)
    except Exception as ex:   # structural model failed → keep aero answer
        warnings.append(f"single-rotor evaluator failed ({type(ex).__name__}: {ex}); aero-only")
        rU, rL = aero_only_evaluate(dU, what), aero_only_evaluate(dL, what)
    rL = _mirror_results(rL, spacing)

    TU = rU.trim.thrust if rU.trim else pt.upper.T
    TL = rL.trim.thrust if rL.trim else pt.lower.T
    QU = rU.trim.torque if rU.trim else pt.upper.Q
    QL = rL.trim.torque if rL.trim else pt.lower.Q
    PU = rU.trim.power if rU.trim else pt.upper.P
    PL = rL.trim.power if rL.trim else pt.lower.P
    T, P = TU + TL, PU + PL
    W = design.op.vehicle_mass * G0
    A = np.pi * RU**2
    rho = design.op.rho
    fom = T**1.5 / np.sqrt(2 * rho * A) / P if P > 0 else 0.0
    # isolated reference: each rotor alone (no interference) at the same thrusts
    aU, aL = pt.aero_upper, pt.aero_lower
    vU, vL = aU.v_ext, aL.v_ext
    aU.v_ext = np.zeros_like(aU.r)
    aL.v_ext = np.zeros_like(aL.r)
    iso_half = aU.trim(thrust=T / 2).P + aL.trim(thrust=T / 2).P
    single_full = aU.trim(thrust=W)
    aU.v_ext, aL.v_ext = vU, vL
    kint = P / iso_half
    auth, ymin, ymax = yaw_authority(design, pt)
    MU, ML = _moment(rU.harmonic, "hub_moment_mean"), _moment(rL.harmonic, "hub_moment_mean")
    m2p = _moment(rU.harmonic, "hub_moment_2p") + _moment(rL.harmonic, "hub_moment_2p")
    if rU.harmonic is None:
        warnings.append("no harmonic results (aero-only): control_moment_max = 0")
    if not pt.converged:
        warnings.append(f"coaxial inflow iteration not converged in {pt.iterations} iterations")
    cmax = MU + ML
    cres = CoaxialResult(
        upper=rU, lower=rL, thrust_total=T, power_total=P, yaw_torque_net=QL - QU,
        thrust_share_upper=TU / T if T > 0 else 0.0, rpm_upper=rU.trim.rpm if rU.trim else pt.upper.rpm,
        rpm_lower=rL.trim.rpm if rL.trim else pt.lower.rpm, figure_of_merit=fom,
        control_moment_max=cmax, control_ratio=cmax / (W * RU), yaw_authority=auth,
        moment_2p_combined=m2p, interference_power_factor=kint)
    res = Results(design_hash=design.hash(), trim=rU.trim, harmonic=rU.harmonic, modes=rU.modes,
                  campbell=rU.campbell, checks=rU.checks + rL.checks,
                  flexure_stiffness=rU.flexure_stiffness, coaxial=cres)
    res.warnings = warnings + [f"upper: {w}" for w in rU.warnings] + [f"lower: {w}" for w in rL.warnings]
    res.summary = dict(
        thrust_total=T, power_total=P, figure_of_merit=fom, rpm_upper=cres.rpm_upper,
        rpm_lower=cres.rpm_lower, thrust_share_upper=cres.thrust_share_upper,
        torque_upper=QU, torque_lower=QL, yaw_torque_net=QL - QU, yaw_authority=auth,
        yaw_torque_min=ymin, yaw_torque_max=ymax, interference_power_factor=kint,
        power_isolated_pair=iso_half, power_single_rotor=single_full.P,
        rpm_single_rotor=single_full.rpm, wake_contraction=pt.r_c,
        control_moment_max=cmax, control_ratio=cres.control_ratio, moment_2p_combined=m2p,
        coax_iterations=float(pt.iterations))
    res.timings_s = dict(pair_trim=t_pair, total=time.perf_counter() - t0)
    return res
