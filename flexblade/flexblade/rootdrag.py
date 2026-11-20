"""Root-hardware profile drag (hub fitting, flexure members, strap, shank) and the
root envelope check.

The non-airfoil parts inboard of the aero blade are bluff bodies in the rotational flow.
Frontal height h(x) (z-extent in frame H, union over parts at radius x) comes from the
geometry meshes; drag per blade is 0.5 rho Cd h U^2 with U = hypot(Omega x, inflow).
"""
from __future__ import annotations

import numpy as np

from .schema import Design

CD_BLUFF = 1.1      # rectangular / blocky sections
CD_FAIRED = 0.4     # fairing: streamlined cover
_HW_ROLES = ("fitting", "member", "member_core", "member_skin", "rod", "wire")
_CACHE: dict[str, tuple] = {}


def hardware_profile(design: Design, dx: float = 0.5e-3):
    """(x, h): bin centres [m] and frontal height [m] of blade-1 root hardware."""
    key = design.hash() if hasattr(design, "hash") else None
    if key and key in _CACHE:
        return _CACHE[key]
    from .geometry import build_geometry, loft_surface
    geo = build_geometry(design, blades=[1], n_span=6)
    pre = "upper_" if design.layout == "coaxial" else ""
    zmin: dict[int, float] = {}
    zmax: dict[int, float] = {}
    xs_all = []
    segs = []
    for p in geo.parts:
        if p.role not in _HW_ROLES or p.blade != 1 or (pre and not p.name.startswith(pre)):
            continue
        V, F = loft_surface(p)
        E = np.vstack([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
        segs.append((V[E[:, 0]], V[E[:, 1]]))
    if not segs:
        out = (np.zeros(0), np.zeros(0))
    else:
        A = np.vstack([s[0] for s in segs])
        B = np.vstack([s[1] for s in segs])
        x0 = max(float(min(A[:, 0].min(), B[:, 0].min())), 0.0)
        x1 = float(max(A[:, 0].max(), B[:, 0].max()))
        xc = np.arange(x0 + 0.5 * dx, x1, dx)
        lo = np.full(len(xc), np.inf)
        hi = np.full(len(xc), -np.inf)
        a, b = A[:, 0], B[:, 0]
        for i, x in enumerate(xc):
            m = (np.minimum(a, b) <= x) & (np.maximum(a, b) >= x)
            if not m.any():
                continue
            den = b[m] - a[m]
            t = np.where(np.abs(den) > 1e-12, (x - a[m]) / np.where(np.abs(den) > 1e-12, den, 1), 0.0)
            z = A[m, 2] + t * (B[m, 2] - A[m, 2])
            lo[i], hi[i] = z.min(), z.max()
        h = np.where(np.isfinite(lo), hi - lo, 0.0)
        out = (xc, h)
    if key:
        _CACHE[key] = out
    return out


def root_drag_torque(design: Design, omega: float, rho: float, r_end: float, inflow: float = 0.0):
    """Torque [N m] of all blades' root hardware over radius < r_end (aero blade start)."""
    x, h = hardware_profile(design)
    if len(x) == 0 or omega <= 0:
        return 0.0
    cd = CD_FAIRED if design.flexure.fairing else CD_BLUFF
    m = x < r_end
    dx = x[1] - x[0] if len(x) > 1 else 0.5e-3
    U2 = (omega * x[m]) ** 2 + inflow ** 2
    dQ = 0.5 * rho * cd * h[m] * U2 * x[m] * dx
    return float(design.blade.n_blades * dQ.sum())


def root_thickness(design: Design) -> float:
    """Airfoil thickness [m] at the root station."""
    from .bladegeom import airfoil_coords, resolve_stations
    st = resolve_stations(design.blade)[0]
    c = airfoil_coords(design.blade.airfoil, st.thickness_frac)
    return float(np.ptp(c[:, 1]) * st.chord)


def envelope(design: Design) -> tuple[float, float]:
    """(max hardware height, allowed height = flexure.root_envelope_ratio x root airfoil thickness)."""
    x, h = hardware_profile(design)
    hmax = float(h.max()) if len(h) else 0.0
    return hmax, design.flexure.root_envelope_ratio * root_thickness(design)
