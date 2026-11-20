"""Flexure load cases and hinge-stiffness target from the rotor model.

The parametric (FACT) flexure is replaced by its 6×6 at the hinge point (rotor.py
``ideal_hinge=dict(K6=...)``, link compliance compensated), the rotor is trimmed at hover
and the 1/rev response solved; the hinge-point wrench the blade puts on the flexure is
W = K6 Δ, Δ = [u; θ] of the blade side at the hinge point relative to the hub (blade 1,
frame H):

  W_steady  sustained (trim): blade root CF pull + steady aero/inertial moments as they
            reach the flexure after CF relief (coning, lag equilibrium) → creep case
  W_1p      complex 1/rev amplitude at the motor's design torque modulation → fatigue

k_hinge target: the hinge-axis compliance of the FACT 6×6 is changed (other compliances
kept) until the rotor's lag mode sits at `lag_per_rev` × Ω at hover.
"""
from __future__ import annotations

import math

import numpy as np

from ..schema import Design


def hinge_axis_e6(axis) -> np.ndarray:
    return np.r_[np.zeros(3), np.asarray(axis, float)]


def with_k_hinge(K6: np.ndarray, axis, k_new: float) -> np.ndarray:
    """K6 whose compliance about the hinge axis (at the hinge point, other dofs free) is
    1/k_new; all other compliance entries unchanged."""
    C = np.linalg.inv(0.5 * (K6 + K6.T))
    e = hinge_axis_e6(axis)
    k_old = 1.0 / float(e @ C @ e)
    C = C + (1.0 / k_new - 1.0 / k_old) * np.outer(e, e)
    K = np.linalg.inv(C)
    return 0.5 * (K + K.T)


def _rotor(design: Design, K6: np.ndarray, mass: dict | None = None, harmonic: bool = True,
           modes: bool = True):
    from .. import rotor as rt
    ih = dict(K6=np.asarray(K6, float).tolist())
    if mass:
        ih.update(mass)
    model = rt.build_model(design, ideal_hinge=ih)
    aero = None
    try:
        aero = rt.make_aero(model)
    except Exception:  # noqa: BLE001
        pass
    ss = rt.static_trim(model, aero)
    lin = rt.linearize(model, ss, design.analysis.coriolis) if (harmonic or modes) else None
    md = rt.modes(model, ss, aero, lin=lin) if modes else None
    hd = rt.harmonic_1p(model, ss, aero, lin=lin) if harmonic else None
    return model, ss, md, hd


def _rel_motion_static(model, ss) -> np.ndarray:
    from ..beam import _logm
    fe = model.fe
    bi = model.blades[0]
    st = ss.state
    X0 = fe.Xref
    m, h = bi.master, model.hub
    H = np.asarray(bi.hinge_point, float)
    Rm, Rh = st.R[m], st.R[h]
    pH = st.x[m] + Rm @ (H - X0[m])                   # hinge point carried by the blade side
    hub_H = st.x[h] + Rh @ (H - X0[h])                # ... and by the hub
    du = Rh.T @ (pH - hub_H)
    dth = _logm(Rh.T @ Rm)
    return np.r_[du, dth]


def _rel_motion_harmonic(model, U: np.ndarray) -> np.ndarray:
    fe = model.fe
    bi = model.blades[0]
    X0 = fe.Xref
    m, h = bi.master, model.hub
    H = np.asarray(bi.hinge_point, float)
    um, tm = U[6 * m:6 * m + 3], U[6 * m + 3:6 * m + 6]
    uh, th = U[6 * h:6 * h + 3], U[6 * h + 3:6 * h + 6]
    uH = um + np.cross(tm, H - X0[m])
    uhH = uh + np.cross(th, H - X0[h])
    return np.r_[uH - uhH, tm - th]


def rotor_hinge_loads(design: Design, K6: np.ndarray, mass: dict | None = None) -> dict:
    """Hinge-point wrenches on blade 1's flexure (frame H, [F; M] at the hinge point) with
    the flexure represented by K6: steady (trim) and 1/rev complex amplitude."""
    model, ss, md, hd = _rotor(design, K6, mass)
    if np.linalg.norm(model.blades[0].Rk - np.eye(3)) > 1e-9:
        raise ValueError("blade 1 frame is not frame H")
    K6 = 0.5 * (np.asarray(K6, float) + np.asarray(K6, float).T)
    d_s = _rel_motion_static(model, ss)
    d_a = _rel_motion_harmonic(model, hd.U)
    from ..rotor import _pick
    out = dict(W_steady=K6 @ d_s, W_1p=K6 @ d_a, d_steady=d_s, d_1p=d_a, rpm=float(ss.rpm),
               Omega=float(ss.Omega), lag_freq_per_rev=_pick(md, "lag", ss.Omega),
               control_ratio=float(hd.result.control_ratio),
               lag_amp_deg=float(hd.result.blades[0].lag_amp_deg),
               pitch_amp_deg=float(hd.result.blades[0].pitch_amp_deg))
    return out


def lag_per_rev(design: Design, K6: np.ndarray) -> float:
    from ..rotor import _pick
    model, ss, md, _ = _rotor(design, K6, harmonic=False)
    return _pick(md, "lag", ss.Omega)


def k_hinge_for_lag(design: Design, K6: np.ndarray, axis, lag_target: float = 0.7,
                    tol: float = 0.005, max_iter: int = 12) -> dict:
    """Hinge stiffness (N m/rad, about `axis` at the hinge point) that puts the rotor's lag
    mode at lag_target/rev at hover; secant on log k, bracketed."""
    e = hinge_axis_e6(axis)
    k0 = 1.0 / float(e @ np.linalg.inv(K6) @ e)
    hist = []

    def f(lk):
        nu = lag_per_rev(design, with_k_hinge(K6, axis, math.exp(lk)))
        hist.append((math.exp(lk), nu))
        return nu - lag_target

    a, fa = math.log(k0), f(math.log(k0))
    if abs(fa) < tol:
        return dict(k_hinge=k0, lag_per_rev=fa + lag_target, k_fact=k0, history=hist)
    step = -0.7 if fa > 0 else 0.7
    b, fb = a + step, f(a + step)
    n = 0
    while fa * fb > 0 and n < 8:
        a, fa = b, fb
        b = b + step
        fb = f(b)
        n += 1
    if fa * fb > 0:
        raise ValueError(f"lag target {lag_target}/rev not bracketed: {hist}")
    for _ in range(max_iter):
        c = b - fb * (b - a) / (fb - fa)
        if not (min(a, b) < c < max(a, b)):
            c = 0.5 * (a + b)
        fc = f(c)
        if abs(fc) < tol:
            break
        if fa * fc < 0:
            b, fb = c, fc
        else:
            a, fa = c, fc
    k, nu = hist[-1]
    return dict(k_hinge=k, lag_per_rev=nu, k_fact=k0, history=hist)
