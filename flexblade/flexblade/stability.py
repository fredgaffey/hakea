"""Damped rotating-frame eigenvalues, air/ground resonance (hub-airframe coupling, Floquet).

Rotating-frame linear system on a reduced basis B (undamped modes of the spinning rotor,
optionally + the two rigid in-plane hub translations expressed in rotating axes):

    M q̈ + (G + C_s + C_a) q̇ + (K + K_a) q = 0

    G        Coriolis/gyroscopic (FE, skew)
    K        elastic + geometric + spin softening at the trim (Re K_complex)
    C_s      structural loss as equivalent viscous at the mode frequency: Im K_complex(ω)/ω
    K_a, C_a quasi-steady aero (aero_linear Ka at ω):  K_a = −Re Ka,  C_a = −Im Ka/ω

Frequency-dependent terms are matched per mode by p-k iteration.  ζ = −Re s/|s|.

Airframe: the hub moves in-plane in the FIXED frame, U = R(ψ) u, ψ = Ωt (spin +z, checked
on the FE matrices).  The FE already gives the rotor's own rigid-translation inertia,
Coriolis (2Ω m J) and spin softening (−Ω² m) for u; the airframe adds, in rotating axes,

    m_af (ü + 2ΩJu̇ − Ω²u) + C_r(ψ)(u̇ + ΩJu) + K_r(ψ) u,   X_r(ψ) = R(−ψ) diag(x_x, x_y) R(ψ)

Isotropic support → constant coefficients (exact LTI eigenproblem, any blade number).
Anisotropic support → 2/rev periodic → Floquet: monodromy over P = π/Ω (solve_ivp on the
reduced system), s = ln μ / P with the 2Ω branch picked from the averaged system.

Hub tilt (soft mount only, ``Mount.tilt``): two more rigid-body DOFs, the rotation of the whole
rotor about the in-plane hub axes (rotating x, y), u = θ × r on every node + node rotation θ, so
the FE brings in the blades' tilt inertia, Coriolis, CF restoring (geometric stiffness) and the
aero of the kinematic flap/pitch change (θx is pitch for a blade on x, θy flap).  The airframe
adds the non-FE motor + carrier + hub (transverse I_t, spinning polar J_p = J_rotor + J_hub) on the
mount (k_t, viscous-equivalent c_t = η k_t/ω_t), fixed frame  I_t Θ̈ + (J_pΩ[[0,1],[−1,0]] + c_t) Θ̇
+ k_t Θ = 0, i.e. in rotating axes (Θ = R(ψ)θ, J_pΩ[[0,1],[−1,0]] = −J_pΩJ):

    I_t (θ̈ + 2ΩJθ̇ − Ω²θ) − J_pΩ J (θ̇ + ΩJθ) + c_t (θ̇ + ΩJθ) + k_t θ

Whirl check (no rotor): I_t ω² ∓ J_pΩ ω − k_t = 0 (tests/test_stability.py).  The mount elastic
centre is taken at the hub origin (no translation-tilt cross stiffness).

Validated against Coleman's isotropic ground-resonance characteristic equation for an
articulated rotor (``coleman_roots``; see tests/test_stability.py).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

J2 = np.array([[0.0, -1.0], [1.0, 0.0]])


@dataclass
class RotSys:
    """Reduced rotating-frame second-order system (real matrices)."""
    M: np.ndarray
    G: np.ndarray        # gyroscopic + all viscous damping (G + C_s + C_a)
    K: np.ndarray
    Omega: float
    ih: tuple | None = None     # indices of rotating-frame hub (x, y) translation, if present
    basis: np.ndarray | None = None   # full-DOF basis (ndof × n)
    m_rotor: float = 0.0
    it: tuple | None = None     # indices of rotating-frame hub tilt (θx, θy), if present


@dataclass
class Airframe:
    m: float
    kx: float
    ky: float
    cx: float
    cy: float
    It: float = 0.0     # non-FE transverse inertia on the mount (motor + carrier + hub), kg m^2
    Jp: float = 0.0     # non-FE spinning polar inertia (motor bell + hub), kg m^2
    kt: float = 0.0     # mount tilt stiffness, N m/rad (0 = no tilt DOFs)
    ct: float = 0.0     # mount tilt viscous-equivalent damping, N m s/rad


def state_matrix(M, G, K) -> np.ndarray:
    n = len(M)
    Mi = np.linalg.inv(M)
    return np.block([[np.zeros((n, n)), np.eye(n)], [-Mi @ K, -Mi @ G]])


def zeta_of(s) -> np.ndarray:
    s = np.asarray(s, complex)
    return -s.real / np.maximum(np.abs(s), 1e-300)


# ------------------------------------------------------------------------- FE reduced system


def _tilt_vectors(model, ss) -> np.ndarray:
    """(ndof × 2) rigid rotation of every node about the rotating x, y axes through the hub
    origin: u = θ × r (deformed trim positions) and node rotation θ."""
    n = model.fe.ndof
    try:
        X = np.asarray(ss.state.x, float).reshape(-1, 3)
    except Exception:
        X = np.asarray(model.fe.X0, float)
    T = np.zeros((n, 2))
    x, y, z = X[:, 0], X[:, 1], X[:, 2]
    T[1::6, 0], T[2::6, 0], T[3::6, 0] = -z, y, 1.0      # θx: (0, −z, y)
    T[0::6, 1], T[2::6, 1], T[4::6, 1] = z, -x, 1.0      # θy: (z, 0, −x)
    return T


def _basis(model, ss, lin, n_basis: int, hub: bool, tilt: bool = False):
    from .rotor import modal_basis
    lam, Phi = modal_basis(model, ss, lin, n_basis=max(40, n_basis + 5), n_keep=n_basis)
    Phi = Phi[:, :n_basis]
    n = model.fe.ndof
    nf = Phi.shape[1]
    B = np.zeros((n, nf + (2 if hub else 0) + (2 if tilt else 0)))
    B[lin.free, :nf] = Phi
    if hub:
        B[0::6, nf] = 1.0       # rigid in-plane translation of every node, rotating x
        B[1::6, nf + 1] = 1.0   # rotating y
    if tilt:
        B[:, -2:] = _tilt_vectors(model, ss)
    return lam[:nf], B


def reduced_system(model, ss, lin, aero, B: np.ndarray, freq_hz: float, hub: bool = False,
                   tilt: bool = False) -> RotSys:
    from .rotor import K_complex, aero_linear
    w = 2 * np.pi * max(freq_hz, 1e-3)
    Kc = B.T @ K_complex(model, lin, max(freq_hz, 1e-3)) @ B
    M = B.T @ lin.M @ B
    G = B.T @ lin.G @ B
    K = Kc.real.copy()
    C = Kc.imag / w
    if aero is not None and ss.trim is not None and model.design.analysis.aero != "none":
        Ka = B.T @ aero_linear(model, ss.state, aero, ss.trim, w, False).Ka @ B
        K = K - Ka.real
        C = C - Ka.imag / w
    n = B.shape[1]
    nh = n - (2 if tilt else 0)
    ih = (nh - 2, nh - 1) if hub else None
    it = (n - 2, n - 1) if tilt else None
    m_rot = float(M[nh - 2, nh - 2]) if hub else 0.0
    return RotSys(M, G + C, K, ss.Omega, ih, B, m_rot, it)


def damped_modes(model, ss, lin, aero, targets_hz, n_basis: int = 20, iters: int = 3):
    """p-k damped eigenvalues tracking each undamped target frequency.

    Returns (s, q, B): complex eigenvalues (rad/s, rotating frame), reduced eigvecs, basis."""
    _, B = _basis(model, ss, lin, n_basis, hub=False)
    n = B.shape[1]
    cache: dict = {}

    def eig_at(fz):
        key = round(fz, 3)
        if key not in cache:
            rs = reduced_system(model, ss, lin, aero, B, fz)
            cache[key] = np.linalg.eig(state_matrix(rs.M, rs.G, rs.K))
        return cache[key]

    out_s, out_q = [], []
    for ft in targets_hz:
        taken = np.array(out_s, complex)
        f = max(ft, 1e-3)
        z = None
        for _ in range(iters):
            ev, EV = eig_at(f)
            wt = 2 * np.pi * f
            cand = np.where(ev.imag >= -1e-9)[0]
            if len(taken):      # close pairs (sym/antisym): don't return the same root twice
                free = [c for c in cand if np.min(np.abs(taken - ev[c])) > 1e-7 * max(abs(ev[c]), 1.0)]
                cand = np.array(free or list(cand))
            i = cand[np.argmin(np.abs(np.abs(ev[cand].imag) - wt) + 0.1 * np.abs(ev[cand].real))]
            z, q = ev[i], EV[:n, i]
            fn = abs(z.imag) / (2 * np.pi)
            if abs(fn - f) < 1e-3 * max(f, 1.0):
                break
            f = max(fn, 1e-3)
        out_s.append(z)
        out_q.append(q)
    return np.array(out_s), np.array(out_q), B


# ------------------------------------------------------------------------- airframe coupling


def mount_stiffness(mt) -> float:
    """In-plane stiffness N/m of an annular elastomer washer loaded in transverse shear (axis along
    the motor axis): k = infill G pi (ro^2 - ri^2) / t.  Axisymmetric, so k_x = k_y exactly.
    (A bonded ring loaded laterally as a bushing is ~10-100x stiffer, incompressible limit.)"""
    ri, ro, t = mt.r_in_mm * 1e-3, mt.r_out_mm * 1e-3, mt.length_mm * 1e-3
    return float(mt.infill * mt.G_mpa * 1e6 * np.pi * (ro**2 - ri**2) / t)


def mount_modes(m_hub, m_arm, k_arm, zeta_arm, k_m, eta):
    """2-DOF hub (on mount) + arm (on rigid support) with complex stiffnesses.  Returns
    (f1_hz, zeta1, modal mass at the hub coordinate, strain-energy zeta1, f2_hz, zeta2) of the lower mode."""
    K = np.array([[k_m * (1 + 1j * eta), -k_m * (1 + 1j * eta)],
                  [-k_m * (1 + 1j * eta), k_m * (1 + 1j * eta) + k_arm * (1 + 2j * zeta_arm)]])
    M = np.diag([m_hub, m_arm])
    w2, V = np.linalg.eig(np.linalg.solve(M, K))
    o = np.argsort(w2.real)
    w2, V = w2[o], V[:, o]
    w = np.sqrt(w2)
    f = w.real / (2 * np.pi)
    z = -w.imag / w.real * -1  # eigenvalue of -w^2: zeta = Im(w2)/(2 Re(w2)) below
    z = w2.imag / (2 * w2.real)
    v = V[:, 0].real
    mm = float(v @ M @ v / v[0] ** 2)
    um = 0.5 * k_m * (v[0] - v[1]) ** 2
    ua = 0.5 * k_arm * v[1] ** 2
    z_se = 0.5 * (eta * um + 2 * zeta_arm * ua) / (um + ua)
    return float(f[0]), float(z[0]), mm, float(z_se), float(f[1]), float(z[1])


def mount_tilt_stiffness(mt) -> float:
    """Tilt stiffness N m/rad of the mount: Mount.k_tilt, else one annular washer in compression,
    E π (ro⁴ − ri⁴)/(4 t), E = 3.4 G infill (bonded-ish elastomer compression modulus)."""
    if getattr(mt, "k_tilt", None) is not None:
        return float(mt.k_tilt)
    ri, ro, t = mt.r_in_mm * 1e-3, mt.r_out_mm * 1e-3, mt.length_mm * 1e-3
    return float(3.4 * mt.G_mpa * 1e6 * mt.infill * np.pi * (ro ** 4 - ri ** 4) / (4 * t))


def tilt_params(mt, motor, I_rotor_tilt: float = 0.0):
    """(I_t, J_p, k_t, c_t) of the non-FE motor + hub on the mount.  c_t = η k_t / ω_t at the
    non-spinning tilt frequency ω_t² = k_t/(I_t + I_rotor_tilt)."""
    Jp = float(motor.J_rotor + motor.J_hub)
    It = float(mt.I_t_kgm2) if getattr(mt, "I_t_kgm2", None) is not None else \
        0.5 * Jp + mt.mass_motor_kg * 0.015 ** 2
    kt = mount_tilt_stiffness(mt)
    wt = np.sqrt(kt / (It + I_rotor_tilt))
    return It, Jp, kt, float(mt.eta * kt / wt)


def tilt_on(af) -> bool:
    mt = getattr(af, "mount", None)
    return mt is not None and bool(getattr(mt, "tilt", False))


def make_airframe(af, vehicle_mass: float, m_rotor: float, motor=None, I_rotor_tilt: float = 0.0) -> Airframe:
    """Schema Airframe -> modal mass/stiffness/damping.  f is the support frequency with the
    (non-spinning) rotor mass on it: k = (m_af + m_rotor) ω².  With a mount (tilt on) and the
    motor given, also the hub-tilt parameters (I_t, J_p, k_t, c_t)."""
    tp = {}
    if motor is not None and tilt_on(af):
        tp = dict(zip(("It", "Jp", "kt", "ct"), tilt_params(af.mount, motor, I_rotor_tilt)))
    m = af.mass if af.mass is not None else 0.25 * vehicle_mass
    mt = m + m_rotor
    if getattr(af, "mount", None) is not None:
        mnt = af.mount
        km = mount_stiffness(mnt)
        fy = af.f_y_hz if af.f_y_hz is not None else af.f_hz
        out = []
        for f_arm in (af.f_hz, fy):       # arm rigid-mount frequency per axis (mount is isotropic)
            k_arm = mt * (2 * np.pi * f_arm) ** 2
            f1, z1, mm, _, _, _ = mount_modes(mnt.mass_motor_kg + m_rotor, m - mnt.mass_motor_kg,
                                              k_arm, af.zeta, km, mnt.eta)
            out.append((f1, z1, mm))
        (fx, zx, mx), (fy_, zy, my) = out
        m_af = 0.5 * (mx + my) - m_rotor
        return Airframe(m_af, (m_af + m_rotor) * (2 * np.pi * fx) ** 2, (m_af + m_rotor) * (2 * np.pi * fy_) ** 2,
                        2 * zx * (m_af + m_rotor) * 2 * np.pi * fx, 2 * zy * (m_af + m_rotor) * 2 * np.pi * fy_,
                        **tp)
    wx = 2 * np.pi * af.f_hz
    wy = 2 * np.pi * (af.f_y_hz if af.f_y_hz is not None else af.f_hz)
    return Airframe(m, mt * wx ** 2, mt * wy ** 2, 2 * af.zeta * mt * wx, 2 * af.zeta * mt * wy)


def _rot(Xx, Xy, psi):
    c, s = np.cos(2 * psi), np.sin(2 * psi)
    a, d = 0.5 * (Xx + Xy), 0.5 * (Xx - Xy)
    return a * np.eye(2) + d * np.array([[c, -s], [-s, -c]])


def coupled_matrices(rs: RotSys, af: Airframe, psi: float = 0.0, average: bool = False):
    """(M, G, K) of rotor + airframe in rotating axes at azimuth psi."""
    Om = rs.Omega
    ih = list(rs.ih)
    M, G, K = rs.M.copy(), rs.G.copy(), rs.K.copy()
    if average:
        Cr = 0.5 * (af.cx + af.cy) * np.eye(2)
        Kr = 0.5 * (af.kx + af.ky) * np.eye(2)
    else:
        Cr, Kr = _rot(af.cx, af.cy, psi), _rot(af.kx, af.ky, psi)
    ix = np.ix_(ih, ih)
    M[ix] += af.m * np.eye(2)
    G[ix] += 2 * Om * af.m * J2 + Cr
    K[ix] += -Om ** 2 * af.m * np.eye(2) + Om * Cr @ J2 + Kr
    if rs.it is not None and af.kt > 0:
        it = np.ix_(list(rs.it), list(rs.it))
        I2 = np.eye(2)
        M[it] += af.It * I2
        G[it] += (2 * Om * af.It - Om * af.Jp) * J2 + af.ct * I2
        K[it] += (-Om ** 2 * af.It + Om ** 2 * af.Jp + af.kt) * I2 + Om * af.ct * J2
    return M, G, K


def isotropic(af: Airframe) -> bool:
    return abs(af.kx - af.ky) <= 1e-9 * af.kx and abs(af.cx - af.cy) <= 1e-9 * max(af.cx, 1e-30)


def coupled_eigs(rs: RotSys, af: Airframe, floquet: bool | None = None, rtol: float = 1e-8):
    """Characteristic exponents (rad/s, rotating frame) of rotor + airframe.
    LTI eig for isotropic support (exact), Floquet otherwise (or if floquet=True)."""
    s_avg, V_avg = np.linalg.eig(state_matrix(*coupled_matrices(rs, af, average=True)))
    if floquet is None:
        floquet = not isotropic(af)
    if not floquet:
        return s_avg, V_avg
    from scipy.integrate import solve_ivp
    Om = rs.Omega
    P = np.pi / Om
    N = 2 * len(rs.M)

    def rhs(t, y):
        A = state_matrix(*coupled_matrices(rs, af, Om * t))
        return (A @ y.reshape(N, N)).ravel()

    sol = solve_ivp(rhs, (0.0, P), np.eye(N).ravel(), method="DOP853", rtol=rtol, atol=1e-10)
    mu, V = np.linalg.eig(sol.y[:, -1].reshape(N, N))
    s0 = np.log(mu.astype(complex)) / P
    # branch (±2Ω k) and pairing from the averaged system
    used = np.zeros(N, bool)
    out = np.empty(N, complex)
    Vout = np.empty((N, N), complex)
    mu_avg = np.exp(s_avg * P)
    for j in np.argsort(-np.abs(s_avg.real)):
        d = np.abs(mu - mu_avg[j]) + used * 1e9
        i = int(np.argmin(d))
        used[i] = True
        k = np.round((s_avg[j].imag - s0[i].imag) / (2 * Om))
        out[j] = s0[i] + 2j * Om * k
        Vout[:, j] = V[:, i]
    return out, Vout


def frame_zeta(rs: RotSys, af: Airframe, s, V, hub_frac: float = 0.5, return_div: bool = False):
    """ζ = −Re s/ω with ω in the frame where the mode lives.  Rotor modes: rotating-frame |s|.
    Hub-dominated modes (kinetic fraction of the hub coordinates > hub_frac): the fixed-frame
    frequency, i.e. the member of the family |Im s + Ω + 2Ωj| nearest the support
    frequencies (a 2/rev-periodic system only defines Im s modulo 2Ω).
    ζ = −Re s/|s_frame| with |s_frame| >= |Re s|.  Growth is frame-invariant, so a real / near-real root
    (|Im s| <= |Re s|) or a fixed-frame frequency below |Re s| is a DIVERGENCE (not a flutter/whirl);
    with return_div=True returns (z, div) where div is a bool array flagging those roots."""
    Mc = coupled_matrices(rs, af, average=True)[0]
    n = len(Mc)
    ih = list(rs.ih)
    mt = af.m + rs.m_rotor
    w_af = np.sqrt(np.array([af.kx, af.ky]) / mt)
    if rs.it is not None and af.kt > 0:
        ih += list(rs.it)
        It = af.It + 0.5 * (Mc[rs.it[0], rs.it[0]] - af.It + Mc[rs.it[1], rs.it[1]] - af.It)
        w_af = np.append(w_af, np.sqrt(af.kt / It))
    j = np.arange(-3, 4)
    z = np.empty(len(s))
    div = np.zeros(len(s), bool)
    for k, sk in enumerate(s):
        q = V[:n, k]
        tot = np.real(np.conj(q) @ Mc @ q)
        qh = q[ih]
        fh = np.real(np.conj(qh) @ Mc[np.ix_(ih, ih)] @ qh) / max(tot, 1e-300)
        w = abs(sk)
        if fh > hub_frac:
            # fixed-frame frequency = the DOMINANT 1P harmonic of the hub motion: rotating (ux, uy) e^{st}
            # -> fixed (ux + i uy) at |w + Omega| and (ux - i uy) at |w - Omega| (tilt likewise).  Picking
            # the alias nearest a support frequency instead mislabels e.g. a regressing-lag ground-resonance
            # mode (hub at w - Omega) when the tilt mode happens to sit at w + Omega (ζ ~4x too low).
            a = np.zeros(2)
            for idx in ([list(rs.ih)] + ([list(rs.it)] if rs.it is not None and af.kt > 0 else [])):
                ux, uy = q[idx]
                mw = Mc[idx[0], idx[0]]
                a += mw * np.array([abs(ux + 1j * uy) ** 2, abs(ux - 1j * uy) ** 2])
            fam = np.abs(sk.imag + rs.Omega + 2 * rs.Omega * j)
            if a.sum() > 0 and max(a) > 0.75 * a.sum():
                wf = abs(sk.imag + rs.Omega) if a[0] > a[1] else abs(sk.imag - rs.Omega)
            else:
                wf = fam[np.argmin(np.min(np.abs(fam[:, None] - w_af[None, :]), axis=1))]
            w = np.hypot(sk.real, wf)
            div[k] = wf < abs(sk.real)
        w = max(w, abs(sk.real), 1e-300)
        div[k] |= abs(sk.imag) <= abs(sk.real)
        z[k] = -sk.real / w
    return (z, div) if return_div else z


def min_zeta(s, Omega: float, skip_frac: float = 0.05, z=None):
    """Minimum ζ over the oscillatory/divergent roots (skips |s| < skip_frac Ω: rigid spin).
    Real roots with Re s > 0 get ζ<0 (divergence) from both zeta_of and frame_zeta(return_div=True)."""
    s = np.asarray(s)
    keep = np.abs(s) > skip_frac * Omega
    if not keep.any():
        return float("nan"), None
    z = zeta_of(s[keep]) if z is None else np.asarray(z)[keep]
    i = int(np.argmin(z))
    return float(z[i]), complex(s[keep][i])


def air_resonance(model, ss, lin, aero, af_schema, n_basis: int = 20, f_eval_hz: float | None = None,
                  floquet: bool | None = None):
    """min ζ of the hub-coupled system; returns (zeta_min, s_min, s_all, Airframe)."""
    tilt = tilt_on(af_schema)
    lam, B = _basis(model, ss, lin, n_basis, hub=True, tilt=tilt)
    if f_eval_hz is None:
        ok = np.sqrt(np.abs(lam)) > 0.05 * ss.Omega
        f_eval_hz = float(np.sqrt(abs(lam[ok][0])) / (2 * np.pi)) if ok.any() else ss.Omega / (2 * np.pi)
    rs = reduced_system(model, ss, lin, aero, B, f_eval_hz, hub=True, tilt=tilt)
    I_rt = 0.5 * (rs.M[rs.it[0], rs.it[0]] + rs.M[rs.it[1], rs.it[1]]) if tilt else 0.0
    af = make_airframe(af_schema, model.design.op.vehicle_mass, rs.m_rotor, model.design.motor, I_rt)
    s, V = coupled_eigs(rs, af, floquet)
    zmin, smin = min_zeta(s, ss.Omega, z=frame_zeta(rs, af, s, V))
    return zmin, smin, s, af


def amp_1p(p_per_rev: float, zeta: float) -> float:
    """Dynamic amplification of a mode at p/rev excited at 1P with damping ζ."""
    r = 1.0 / max(p_per_rev, 1e-9)
    return float(1.0 / np.hypot(1 - r * r, 2 * zeta * r))


def margin_1p_eff(p_per_rev: float, zeta: float) -> float:
    """Damping-aware 1P margin: the undamped (above-1P) frequency margin with the same 1P
    amplification.  ζ = 0 gives the plain margin (above 1P) / slightly more (below 1P)."""
    a = amp_1p(p_per_rev, max(zeta, 0.0))
    x = 1.0 / a                       # = 1 − 1/(1+m)²
    if x >= 1.0:
        return float("inf")
    return float(1.0 / np.sqrt(1.0 - x) - 1.0)


# ------------------------------------------------------------------------- validation model


def articulated_toy(B: int, e: float, rg: float, mb: float, nu: float, Omega: float,
                    c_lag: float = 0.0) -> RotSys:
    """Articulated rotor (point-mass blades at rg outboard of a lag hinge at e) with the hub
    translation, built from exact kinematics in rotating axes (M, 2ΩΣmAᵀJA, Hessian of
    −½Ω²Σm|p|² + ½kζ²): an independent check of the coupling/Floquet path."""
    n = B + 2
    a = 2 * np.pi * np.arange(B) / B
    I = mb * rg ** 2
    k_lag = max(nu ** 2 - e / rg, 0.0) * I * Omega ** 2

    def pos(q):
        u = q[B:]
        return [u + e * np.array([np.cos(a[k]), np.sin(a[k])])
                + rg * np.array([np.cos(a[k] + q[k]), np.sin(a[k] + q[k])]) for k in range(B)]

    def V(q):
        return -0.5 * Omega ** 2 * mb * sum(p @ p for p in pos(q)) + 0.5 * k_lag * np.sum(q[:B] ** 2)

    h = 1e-4
    K = np.zeros((n, n))
    E = np.eye(n) * h
    for i in range(n):
        for j in range(n):
            K[i, j] = (V(E[i] + E[j]) - V(E[i] - E[j]) - V(-E[i] + E[j]) + V(-E[i] - E[j])) / (4 * h * h)
    M = np.zeros((n, n))
    G = np.zeros((n, n))
    for k in range(B):
        A = np.zeros((2, n))
        A[:, k] = rg * np.array([-np.sin(a[k]), np.cos(a[k])])
        A[:, B:] = np.eye(2)
        M += mb * A.T @ A
        G += 2 * Omega * mb * A.T @ J2 @ A
    G[np.arange(B), np.arange(B)] += c_lag
    return RotSys(M, G, K, Omega, (B, B + 1), None, B * mb)


def coleman_roots(B: int, e: float, rg: float, mb: float, nu: float, Omega: float, c_lag: float,
                  af: Airframe) -> np.ndarray:
    """Fixed-frame roots of Coleman's isotropic ground-resonance equation (B ≥ 3):
    (M s² + C s + K)·[I(s² − 2iΩs + (ν²−1)Ω²) + c(s − iΩ)] − (B S²/2) s⁴ = 0."""
    I, S = mb * rg ** 2, mb * rg
    Mt = af.m + B * mb
    hub = np.array([Mt, af.cx, af.kx], complex)
    lag = np.array([I, -2j * Omega * I + c_lag, (nu ** 2 - 1) * Omega ** 2 * I - 1j * Omega * c_lag], complex)
    poly = np.polymul(hub, lag)
    poly[0] -= B * S * S / 2
    return np.roots(poly)
