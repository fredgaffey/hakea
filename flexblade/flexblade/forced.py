"""Higher-harmonic (nP, n >= 2) forced response at a mode / harmonic crossing.

Why: hover rpm spans sqrt(1.4/0.7) = sqrt(2) over the thrust range, so any fixed-Hz mode above
2P must cross some nP somewhere in the throttle range (a band (nP, (n+1)P) with +-5 % margins
needs (n+1)/n >= 1.56, i.e. n = 1 only).  A frequency-margin rule for n >= 2 is therefore
unsatisfiable; instead the response AT the crossing is computed and checked against fatigue
and pitch-jitter limits.

Model (per elastic mode i, per harmonic n with the mode inside the crossing window):
    q_s = |x^H F_s| / (2 zeta k_gen),   k_gen = Re(x^H M x) omega_i^2     (resonant SDOF)
    q   = sqrt(sum_s q_s^2)          (sources of unknown relative phase: SRSS)
    U   = x q                         (x: complex mode shape, max |x| = 1)
zeta is the mode's damped ratio (energy method with complex moduli E(1 + i eta) of materials.py
plus aero, or the eig/Floquet value when analysis.stability != energy), floored at ZETA_FLOOR.
Steady dwell at the crossing is assumed (hover at the crossing thrust), i.e. full Q = 1/(2 zeta).

Forcing sources (all configurable module constants):
  * motor torque harmonics: a sinusoidal 1/rev duty command produces nP torque only through
    ESC/motor nonlinearity (duty->current saturation, back-EMF/ripple, commutation).  Assumed
    TORQUE_HARMONIC_FRAC[n] x the 1P torque amplitude (3 % at 2P, 1 % at 3P, 0.3 % beyond);
    engineering assumption, check against a bench current spectrum.
  * aero nP blade loads: inflow skew from rotor tilt / forward flight / ground effect gives nP
    blade loads that fall off with n.  For a lightly loaded rotor in modest edgewise flow
    (advance ratio mu ~ 0.05-0.1) the 2P blade flap moment is ~5-10 % of the mean lift moment:
    Johnson, "Rotorcraft Aeromechanics" (CUP 2013) ch. 16 (rotor vibration: harmonic blade
    loads ~ mu^(n-1) / inflow-skew terms, transition-flight peak) and Bramwell, Done & Balmford,
    "Bramwell's Helicopter Dynamics" (2001) ch. 9-10 (blade loads in forward flight).  Higher
    harmonics are taken to fall as ~1/n^2 (AERO_HARMONIC_FRAC).  Applied as a fraction of each
    blade's mean distributed aero load, blade k phased exp(i n 2 pi k / B) (2-blade: even n
    collective, odd n differential).
  * blade imbalance: IMBALANCE_MASS_FRAC of one blade's mass at its CG radius gives a
    fixed-frame 1P hub force; on a free vehicle (rigid-body mass) the hub acceleration
    a = dm Omega^2 r_cg / m_vehicle appears in the rotating frame as 0P + 2P (a/2 each); the
    2P part loads the blades inertially (n = 2 only).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

HARMONICS = (2, 3, 4, 5, 6)
TORQUE_HARMONIC_FRAC = {2: 0.03, 3: 0.01}
TORQUE_HARMONIC_FRAC_HIGH = 0.003
AERO_HARMONIC_FRAC = {2: 0.075, 3: 0.03, 4: 0.015, 5: 0.01, 6: 0.01}
IMBALANCE_MASS_FRAC = 0.01
# crossing window |p/n - 1| <= CROSS_BAND at each evaluated point.  Points are evaluated at
# thrust 0.7/1.0/1.4 (rpm steps ~1.19x), so a half-step window covers the whole throttle range.
CROSS_BAND = 0.10
ZETA_FLOOR = 0.002
# pitch jitter limit: pass if <= max(JITTER_MAX_DEG, JITTER_FRAC_1P x 1P cyclic pitch amplitude)
JITTER_MAX_DEG = 0.3
JITTER_FRAC_1P = 0.10
# combination of the nP amplitude with the 1P cyclic amplitude in the fatigue checks:
# "srss" (different frequencies, rms-like) or "sum" (peak-coincidence, conservative)
FATIGUE_COMBINE = "srss"


def resonant_amplitude(x: np.ndarray, M: np.ndarray, F: np.ndarray, omega: float, zeta: float) -> float:
    """Modal amplitude q at exact resonance (multiplies x): |x^H F| / (2 zeta Re(x^H M x) omega^2)."""
    m = float(np.real(np.conj(x) @ (M @ x)))
    k = m * omega * omega
    return float(abs(np.conj(x) @ F) / (2.0 * max(zeta, 1e-12) * max(k, 1e-300)))


def combine(a: float, b: float, how: str | None = None) -> float:
    how = how or FATIGUE_COMBINE
    return a + b if how == "sum" else float(np.hypot(a, b))


def torque_frac(n: int) -> float:
    return TORQUE_HARMONIC_FRAC.get(n, TORQUE_HARMONIC_FRAC_HIGH)


@dataclass
class Crossing:
    mode: int
    label: str
    n: int
    per_rev: float
    zeta: float
    q: float
    q_parts: dict
    U: np.ndarray           # full complex DOF amplitude at the crossing
    freq_hz: float
    jitter_deg: float = 0.0


def _blade_aero_vectors(model, st, trim) -> list[np.ndarray]:
    from .rotor import _aero_maps, _aero_points, _edofs
    out = []
    maps = _aero_maps(model, st)
    for bi in model.blades:
        F = np.zeros(model.fe.ndof)
        for e, xi, wl, r, qc, ia, s in _aero_points(model, bi):
            g = lambda a: (1 - s) * a[ia] + s * a[ia + 1]       # noqa: E731
            rr, sH, L, Nr = next(maps)
            fH = bi.Rk @ np.array([g(trim.Fx), g(trim.Fy), g(trim.Fz)])
            mH = bi.Rk @ np.array([g(trim.Mx), 0.0, 0.0])
            F[_edofs(e)] += wl * (L.T @ fH + Nr.T @ mH)
        out.append(F)
    return out


def forcing_vectors(model, ss, lin, dQ1: complex, n: int, vehicle_mass: float) -> dict[str, np.ndarray]:
    """nP forcing vectors (full DOF, complex) at the evaluated rpm."""
    fe = model.fe
    out = {}
    F = np.zeros(fe.ndof, complex)
    F[6 * model.hub + 5] = torque_frac(n) * abs(dQ1)
    out["torque"] = F
    fa = AERO_HARMONIC_FRAC.get(n, 0.0)
    if fa > 0 and ss.trim is not None and getattr(ss.trim, "Fz", None) is not None:
        try:
            vs = _blade_aero_vectors(model, ss.state, ss.trim)
            B = len(vs)
            out["aero"] = fa * sum(v * np.exp(2j * np.pi * n * k / B) for k, v in enumerate(vs))
        except Exception:   # noqa: BLE001  (structural-only models)
            pass
    if n == 2 and IMBALANCE_MASS_FRAC > 0 and vehicle_mass > 0 and model.blades:
        M = lin.M
        bi = model.blades[0]
        ms = np.array([M[6 * j, 6 * j] for j in bi.chain_nodes])
        rs = np.array([np.linalg.norm(fe.Xref[j][:2]) for j in bi.chain_nodes])
        mb = float(ms.sum())
        rcg = float((ms * rs).sum() / mb) if mb > 0 else 0.0
        a = IMBALANCE_MASS_FRAC * mb * ss.Omega ** 2 * rcg / vehicle_mass
        T = np.zeros(fe.ndof, complex)
        T[0::6] = 1.0
        T[1::6] = 1j
        out["imbalance"] = -(M @ T) * (a / 2.0)
    return out


def crossings(model, ss, lin, md, zetas, freqs_hz, dQ1: complex, vehicle_mass: float) -> list[Crossing]:
    P = ss.Omega / (2 * np.pi)
    if P <= 0:
        return []
    f = lin.free
    mask = np.zeros(model.fe.ndof, bool)
    mask[f] = True
    out = []
    cache: dict[int, dict] = {}
    for i, (fr, z) in enumerate(zip(freqs_hz, zetas)):
        p = fr / P
        if p < 1.5:
            continue
        x = np.where(mask, md.shapes[i], 0.0)
        w = 2 * np.pi * fr
        ze = max(float(z) if np.isfinite(z) else 0.0, ZETA_FLOOR)
        for n in HARMONICS:
            if abs(p / n - 1.0) > CROSS_BAND:
                continue
            if n not in cache:
                cache[n] = forcing_vectors(model, ss, lin, dQ1, n, vehicle_mass)
            sc = (p / n) ** 2          # forcing ~ Omega^2 at the crossing rpm
            parts = {s: sc * resonant_amplitude(x, lin.M, np.where(mask, F, 0.0), w, ze)
                     for s, F in cache[n].items()}
            q = float(np.sqrt(sum(v * v for v in parts.values())))
            out.append(Crossing(i, md.labels[i], n, float(p), ze, q, parts, x * q, float(fr)))
    return out


def pitch_jitter_deg(model, U: np.ndarray) -> float:
    from .rotor import blade_local_motion_c
    best = 0.0
    for bi in model.blades:
        _, rv = blade_local_motion_c(model, U, bi)
        i75 = int(np.argmin(np.abs(bi.chain_r - 0.75 * model.R)))
        best = max(best, float(np.degrees(abs(rv[i75, 0]))))
    return best
