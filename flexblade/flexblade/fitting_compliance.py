"""Series compliance of clamped-strip joints: glue lands + printed fitting arms (FITTING_COMPLIANCE).

The beam model joins each clamped steel strip end to rigid hub / blade bodies. The whole-rotor
CalculiX run of the real CAD (study/results/fea_cad_5in.md) shows that this overstates the flap
and cross stiffness: ccx/beam = 0.59 / 0.60 with 210 GPa fittings, 0.14 / 0.17 with PA6-CF,
0.05 / 0.06 with PETG (pitch stays at 0.91). This module adds the missing compliance in series,
C_total = C_flexure + C_glue + C_fitting, with no calibration constants:

(a) glue land, one 6x6 spring at each strip end (member frame e1 along the strip, e2 width,
    e3 thickness normal), referred to the slot mouth. The strip stack runs a length `land` into
    the slot, bonded on both faces by an epoxy layer of thickness t_g:
      axial u1 / twist th1: shear-lag bar on a shear / peel foundation,
                            k = sqrt(S k_f) tanh(L sqrt(k_f / S))
                            (S = EA, k_f = 2 G_g w / t_g;  S = GJ, k_f = 2 M_g w^3 / (12 t_g))
      (u2, th3) in-plane and (u3, th2) out-of-plane: Hermite beam on a Winkler foundation over the
                            finite land (free inner end), condensed to the mouth
                            (EI_in-plane, k_f = 2 G_g w / t_g;  EI_flat, k_f = 2 M_g w / t_g)
    M_g = confined (oedometric) glue modulus. t_g defaults to the geometric gap
    (slot_t - n t) / 2 per face from fittings.strip_params.
(b) fitting arms, one 6x6 spring per arm (hub arm: hub node -> strip spokes; blade arm: strip
    connectors -> blade chain). Each arm is one equivalent Timoshenko beam along the hinge axis a,
    with section properties from slices of the real CSG fitting (fittings.build_strip_fittings)
    every ARM_DX: A, principal I, J ~ A^4 / (4 pi^2 Ip) (Saint-Venant), shear area 5/6 A. Hub arm:
    from the rotor axis to the hub land centroids; blade arm: from the blade land centroids to the
    aero blade start (r_aero[0], where the beam blade chain takes over). Each strip carries 1/n
    of the joint load, so a station passes the share of the lands on its loaded side (f^2 weight
    in the compliance integral). Fitting material: flexure.fitting_material, else hub pa6_cf /
    blade = blade layup material, isotropic with its E1 and nu12 (the in-raster modulus, as the
    CalculiX check; the interlayer modulus is lower, so this is a lower bound on the compliance).

Switch: env FITTING_COMPLIANCE=1 (default OFF). Overrides for checks: FITTING_E (Pa, fitting
modulus; "inf" = rigid arms), FITTING_GLUE_E (Pa; "inf" = rigid glue), FITTING_GLUE_T (m, glue
thickness per face; the CalculiX model's lamina-at-mid-plane gap is 0.325e-3), FITTING_BLADE_ARM_END
(m, radial clamp station of the blade arm; default the beam's aero blade start).
"""
from __future__ import annotations

import math
import os
from functools import lru_cache

import numpy as np

ENV = "FITTING_COMPLIANCE"
GLUE_E = 3.0e9          # Pa, structural epoxy (as the CalculiX model)
GLUE_NU = 0.35
ARM_DX = 0.25           # mm, slice spacing along the arm
KAPPA = 5.0 / 6.0


def enabled() -> bool:
    return os.environ.get(ENV, "0").strip().lower() in ("1", "true", "on", "yes")


def _env_float(name: str, default: float) -> float:
    v = os.environ.get(name)
    return float(v) if v not in (None, "") else default


def active(design) -> bool:
    """Flag on and the flexure is clamped sheet strips in printed fittings."""
    if not enabled():
        return False
    from .fittings import is_clamped_strip
    return is_clamped_strip(design)


# --------------------------------------------------------------------------- (a) glue land


def glue_props(design) -> tuple[float, float, float, float]:
    """(E_g, nu_g, t_g m per face, land m)."""
    from .fittings import strip_params
    prm = strip_params(design)
    n = int(getattr(design.flexure, "strip_laminae", 1) or 1)
    t_stack = n * design.flexure.thickness * 1e3
    tg = _env_float("FITTING_GLUE_T", max(0.5 * (prm.slot_t - t_stack), 0.005) * 1e-3)
    return _env_float("FITTING_GLUE_E", GLUE_E), GLUE_NU, tg, prm.land * 1e-3


def _shear_lag(S: float, kf: float, L: float) -> float:
    lam = math.sqrt(kf / S)
    return math.sqrt(S * kf) * math.tanh(lam * L)


def _boef(EI: float, kf: float, L: float, n: int = 32) -> np.ndarray:
    """2x2 stiffness [w, dw/ds] at s = 0 of a beam (EI) on a Winkler foundation kf (N/m^2) over
    0 <= s <= L, free at s = L (cubic Hermite elements, consistent foundation matrix)."""
    h = L / n
    ke = EI / h ** 3 * np.array([[12, 6 * h, -12, 6 * h], [6 * h, 4 * h * h, -6 * h, 2 * h * h],
                                 [-12, -6 * h, 12, -6 * h], [6 * h, 2 * h * h, -6 * h, 4 * h * h]])
    kw = kf * h / 420.0 * np.array([[156, 22 * h, 54, -13 * h], [22 * h, 4 * h * h, 13 * h, -3 * h * h],
                                    [54, 13 * h, 156, -22 * h], [-13 * h, -3 * h * h, -22 * h, 4 * h * h]])
    K = np.zeros((2 * n + 2, 2 * n + 2))
    for i in range(n):
        K[2 * i:2 * i + 4, 2 * i:2 * i + 4] += ke + kw
    Kmm, Kmo, Koo = K[:2, :2], K[:2, 2:], K[2:, 2:]
    return Kmm - Kmo @ np.linalg.solve(Koo, Kmo.T)


def glue_end_spring(sp, width: float, sigma: float, glue) -> np.ndarray | None:
    """6x6 [u1 u2 u3 th1 th2 th3] (member frame) of the glue land at a strip end, about the mouth.
    sp: strip section (EA, EIy flat bending about e2, EIz in-plane about e3, GJ); sigma = +1 if the
    land runs along +e1 from the mouth (blade end), -1 along -e1 (hub end). None = rigid glue."""
    Eg, nu, tg, L = glue
    if not np.isfinite(Eg):
        return None
    Gg = Eg / (2 * (1 + nu))
    Mg = Eg * (1 - nu) / ((1 + nu) * (1 - 2 * nu))
    w = width
    K = np.zeros((6, 6))
    K[0, 0] = _shear_lag(sp.EA, 2 * Gg * w / tg, L)
    K[3, 3] = _shear_lag(sp.GJ, 2 * Mg * w ** 3 / 12 / tg, L)
    # (u2, th3): dw/ds = sigma th3 ; (u3, th2): dw/ds = -sigma th2
    for (iu, it, sg, EI, kf) in ((1, 5, sigma, sp.EIz, 2 * Gg * w / tg), (2, 4, -sigma, sp.EIy, 2 * Mg * w / tg)):
        Kb = _boef(EI, kf, L)
        T = np.diag([1.0, sg])
        K[np.ix_([iu, it], [iu, it])] = T.T @ Kb @ T
    return 0.5 * (K + K.T)


# --------------------------------------------------------------------------- (b) fitting arms


def _poly_props(polys):
    """Signed-area polygon set (outer CCW, holes CW) -> A, centroid (2,), second moments about it."""
    A = Sx = Sy = Ixx = Iyy = Ixy = 0.0
    for P in polys:
        P = np.asarray(P, float)
        if len(P) < 3:
            continue
        x, y = P[:, 0], P[:, 1]
        x1, y1 = np.roll(x, -1), np.roll(y, -1)
        c = x * y1 - x1 * y
        A += c.sum() / 2
        Sx += ((x + x1) * c).sum() / 6
        Sy += ((y + y1) * c).sum() / 6
        Iyy += ((x * x + x * x1 + x1 * x1) * c).sum() / 12      # int x^2
        Ixx += ((y * y + y * y1 + y1 * y1) * c).sum() / 12      # int y^2
        Ixy += ((x * y1 + 2 * x * y + 2 * x1 * y1 + x1 * y) * c).sum() / 24
    if abs(A) < 1e-9:
        return 0.0, np.zeros(2), np.zeros((2, 2))
    s = np.sign(A)
    A, Sx, Sy, Ixx, Iyy, Ixy = (s * v for v in (A, Sx, Sy, Ixx, Iyy, Ixy))
    cx, cy = Sx / A, Sy / A
    Ic = np.array([[Iyy - A * cx * cx, Ixy - A * cx * cy], [Ixy - A * cx * cy, Ixx - A * cy * cy]])
    return A, np.array([cx, cy]), Ic          # Ic = int (p-c)(p-c)^T dA


def blade_arm_end(design) -> float:
    """Radial station (m) where the blade arm is clamped: the aero blade start of the beam model
    (the beam blade chain carries the rest), or env FITTING_BLADE_ARM_END (m), e.g. the CalculiX
    case-1 boundary r_aero(CAD) + 3 mm."""
    return _env_float("FITTING_BLADE_ARM_END", _aero_start(design))


@lru_cache(maxsize=32)
def _arm_geometry(key: str, design_json: str, x_end: float):
    """Blade-1 frame, SI. Per side: (P reference point, list of (ds, f, centroid, A, Ic 3x3 about the
    centroid in the plane normal to a, a)); geometry only (material applied later)."""
    from .fittings import build_strip_fittings, _sub, strip_members
    from .schema import Design
    from . import fact
    d = Design.model_validate_json(design_json)
    fits = build_strip_fittings(d)
    a = np.asarray(fits.hinge_axis, float)
    a /= np.linalg.norm(a)
    v1 = np.cross([0.0, 1.0, 0.0], a)
    v1 /= np.linalg.norm(v1)
    v2 = np.cross(a, v1)
    Q = np.vstack([v1, v2, a])                       # rows: (v1, v2, a) -> slice frame (x, y, z)
    strips, _, _ = strip_members(d)
    land = fits.params.land
    cents = {0: [], 1: []}
    for (j, end, dvec, *_r) in fits.slot_axes:
        m = strips[j]
        p = np.asarray(m.p0 if end == 0 else m.p1, float) * 1e3
        cents[end].append(p + np.asarray(dvec, float) * land / 2)
    n = max(len(strips), 1)
    r_root = x_end * 1e3
    out = {}
    for side, end in (("hub", 0), ("blade", 1)):
        part = _sub(getattr(fits, side), [fits.keepout])
        M = np.zeros((3, 4))
        M[:, :3] = Q
        pr = part.transform(M)
        xi_l = np.array([c @ a for c in cents[end]])
        if side == "hub":
            lo, hi = 0.0, float(xi_l.max())
            P = np.zeros(3)
        else:
            lo, hi = float(xi_l.min()), float(np.array([r_root, 0.0, 0.0]) @ a)
            P = np.mean(cents[end], axis=0)
        st = []
        if hi > lo:
            ns = max(int(math.ceil((hi - lo) / ARM_DX)), 4)
            ds = (hi - lo) / ns
            for i in range(ns):
                z = lo + (i + 0.5) * ds
                f = float((xi_l > z).sum() if side == "hub" else (xi_l < z).sum()) / n
                if f <= 0:
                    continue
                A, c2, I2 = _poly_props(pr.slice(z).to_polygons())
                if A <= 1e-6:
                    continue
                c3 = (c2[0] * v1 + c2[1] * v2 + z * a) * 1e-3
                B = np.column_stack([v1, v2])
                st.append((ds * 1e-3, f, c3, A * 1e-6, B @ I2 @ B.T * 1e-12))
        out[side] = (P * 1e-3, st, a)
    return out


def _aero_start(d) -> float:
    """r_aero[0] of rotor.build_model (aero blade start), m."""
    from . import fact
    b = d.blade
    R = b.radius
    r_root = b.root_cutout_frac * R
    ms = fact.generate_members(d)
    r_m = max(max(m.p0[0], m.p1[0]) for m in ms) if ms else d.flexure.hinge_radius
    return r_m + 0.02 * (R - r_m) if r_m > r_root + 1e-9 else r_root


def fitting_moduli(design) -> dict:
    from .geometry import HUB_MATERIAL
    from .materials import get_material
    fm = design.flexure.fitting_material
    out = {}
    for side, mat in (("hub", fm or HUB_MATERIAL), ("blade", fm or design.blade.layup.material)):
        m = get_material(mat)
        E = _env_float("FITTING_E", m.E1)
        out[side] = (E, float(getattr(m, "nu12", 0.35) or 0.35))
    return out


def arm_compliance(design) -> dict:
    """side -> (P blade-1 frame m, 6x6 compliance [u; th] about P, H frame) or None (rigid)."""
    from .fittings import _geom_key
    g = _arm_geometry(_geom_key(design), design.model_dump_json(), blade_arm_end(design))
    mods = fitting_moduli(design)
    out = {}
    for side, (P, st, a) in g.items():
        E, nu = mods[side]
        if not np.isfinite(E) or not st:
            out[side] = None
            continue
        G = E / (2 * (1 + nu))
        C = np.zeros((6, 6))
        for ds, f, c, A, Ic in st:
            lam, V = np.linalg.eigh(Ic)              # two in-plane principal axes (+ a: zero)
            idx = np.argsort(lam)[1:]
            # Ic is the spread int (p-c)(p-c)^T dA: its eigenvalue along p1 is the bending inertia
            # about p2 and vice versa
            I1, I2 = max(lam[idx[1]], 1e-30), max(lam[idx[0]], 1e-30)     # about p1, about p2
            p1, p2 = V[:, idx[0]], V[:, idx[1]]
            J = A ** 4 / (4 * math.pi ** 2 * (I1 + I2))
            Rs = np.column_stack([a, p1, p2])
            Dinv = np.diag([1 / (E * A), 1 / (KAPPA * G * A), 1 / (KAPPA * G * A),
                            1 / (G * J), 1 / (E * I1), 1 / (E * I2)])
            Rb = np.zeros((6, 6))
            Rb[:3, :3] = Rb[3:, 3:] = Rs
            T = np.eye(6)
            r = P - c
            T[3:, :3] = np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
            C += ds * f * f * T.T @ Rb @ Dinv @ Rb.T @ T
        out[side] = (P, 0.5 * (C + C.T))
    return out
