"""Cheap soft ground-resonance check (study I31/I32).

Regressing lag f_reg = |w_lag,rot - Omega|/2pi = |lag_per_rev - 1| * f_rev (fixed frame) coalesces with
the hub/arm support mode f_af.  Violation = max(0, 1 - |f_reg - f_af| / (0.25 f_af)), worst over the
design-envelope arm range (= evaluated at the arm frequency closest to f_reg).  With a mount the support
mode is the lower hub-on-mount mode of the rigid-mount arm frequency (stability.mount_modes).
"""
from __future__ import annotations
import math
import numpy as np

BAND = 0.25
# typical arm ranges, Hz, at the reference radii (scale ~ 1/R^2 for a cantilever arm of fixed section/length ratio)
REF = {3.5 * 0.0254: (200.0, 300.0), 5.0 * 0.0254: (120.0, 250.0)}


def default_arm_range(radius_m: float) -> tuple[float, float]:
    R7, R10 = 3.5 * 0.0254, 5.0 * 0.0254
    if abs(radius_m - R7) < 1e-3:
        return REF[R7]
    if abs(radius_m - R10) < 1e-3:
        return REF[R10]
    lo, hi = REF[R7]
    return lo * R7 / radius_m, hi * R7 / radius_m


def support_freqs(f_arm: np.ndarray, af=None, m_rotor: float = 0.0, vehicle_mass: float = 1.0) -> np.ndarray:
    """Hub support mode frequency per arm frequency (rigid arm if no mount)."""
    mt = getattr(af, "mount", None) if af is not None else None
    if mt is None:
        return np.asarray(f_arm, float)
    from flexblade import stability as st
    m = af.mass if af.mass is not None else 0.25 * vehicle_mass
    km = st.mount_stiffness(mt)
    out = []
    for f in f_arm:
        k_arm = (m + m_rotor) * (2 * math.pi * f) ** 2
        out.append(st.mount_modes(mt.mass_motor_kg + m_rotor, m - mt.mass_motor_kg, k_arm, af.zeta, km, mt.eta)[0])
    return np.asarray(out)


def lag_per_rev(modes, rpm: float, min_part: float = 0.5) -> float:
    """Lowest lag-dominant rotating-frame mode (participation 'lag' >= min_part, f > 0.05/rev), in /rev;
    falls back to the max-lag-participation mode; nan if no modes."""
    f_rev = rpm / 60.0
    c = [(m["freq_hz"] / f_rev, (m.get("participation") or {}).get("lag", 0.0)) for m in modes or []
         if m["freq_hz"] > 0.05 * f_rev]
    good = [f for f, p in c if p >= min_part]
    if good:
        return float(min(good))
    return float(max(c, key=lambda t: t[1])[0]) if c else float("nan")


def violation(lag_per_rev: float, rpm: float, arm_range, af=None, m_rotor: float = 0.0,
              vehicle_mass: float = 1.0, n: int = 61) -> float:
    if not (math.isfinite(lag_per_rev) and math.isfinite(rpm)) or rpm <= 0:
        return 0.0
    f_reg = abs(lag_per_rev - 1.0) * rpm / 60.0
    f_arm = np.linspace(arm_range[0], arm_range[1], n)
    f_af = support_freqs(f_arm, af, m_rotor, vehicle_mass)
    v = 1.0 - np.abs(f_reg - f_af) / (BAND * f_af)
    return float(max(0.0, v.max()))
