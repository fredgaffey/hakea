"""Shared blade definition helpers (OWNER: lead). Used by aero, section, geometry.

resolve_stations(blade): explicit stations, or the default hover law (same as
presets.hover_blade_stations), x = r/R:
  chord c = R (0.30 - 0.12 x);  twist θ = 8° + atan(0.12 / x)
airfoil_coords(airfoil, thickness_frac): closed loop (x/c, y/c), TE → upper → LE → lower → TE.
"""
from __future__ import annotations

import numpy as np

from .schema import Airfoil, Blade, BladeStation

N_DEFAULT_STATIONS = 12


DEFAULT_CHORD_LAW = (0.30, 0.12)
DEFAULT_TWIST_ALPHA_DEG = 8.0
DEFAULT_TWIST_LAMBDA = 0.12


def resolve_stations(blade: Blade) -> list[BladeStation]:
    if blade.stations:
        return sorted(blade.stations, key=lambda s: s.r_frac)
    R = blade.radius
    c0, c1 = DEFAULT_CHORD_LAW
    out = []
    for x in np.linspace(blade.root_cutout_frac, 1.0, N_DEFAULT_STATIONS):
        out.append(BladeStation(
            r_frac=float(x), chord=float(R * (c0 - c1 * x)),
            twist_deg=float(DEFAULT_TWIST_ALPHA_DEG + np.degrees(np.arctan(DEFAULT_TWIST_LAMBDA / x)))))
    return out


def station_arrays(blade: Blade) -> dict[str, np.ndarray]:
    st = resolve_stations(blade)
    R = blade.radius
    return dict(r=np.array([s.r_frac * R for s in st]),
                chord=np.array([s.chord for s in st]),
                twist=np.radians([s.twist_deg for s in st]))


def naca4_coords(code: str, n: int = 81, thickness_frac: float | None = None) -> np.ndarray:
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:4]) / 100.0 if thickness_frac is None else thickness_frac
    beta = np.linspace(0.0, np.pi, n)
    x = 0.5 * (1 - np.cos(beta))
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3
                  - 0.1036 * x**4)  # open TE
    if m > 0 and p > 0:
        yc = np.where(x < p, m / p**2 * (2 * p * x - x**2),
                      m / (1 - p)**2 * ((1 - 2 * p) + 2 * p * x - x**2))
        dyc = np.where(x < p, 2 * m / p**2 * (p - x), 2 * m / (1 - p)**2 * (p - x))
    else:
        yc = np.zeros_like(x)
        dyc = np.zeros_like(x)
    th = np.arctan(dyc)
    xu, yu = x - yt * np.sin(th), yc + yt * np.cos(th)
    xl, yl = x + yt * np.sin(th), yc - yt * np.cos(th)
    upper = np.column_stack([xu[::-1], yu[::-1]])   # TE -> LE
    lower = np.column_stack([xl[1:], yl[1:]])       # LE -> TE
    return np.vstack([upper, lower])


def airfoil_coords(airfoil: Airfoil, thickness_frac: float | None = None, n: int = 81) -> np.ndarray:
    if airfoil.kind == "naca4":
        return naca4_coords(airfoil.naca, n, thickness_frac)
    c = np.asarray(airfoil.coords, float)
    if thickness_frac is not None:
        t0 = c[:, 1].max() - c[:, 1].min()
        mid = 0.0
        c = c.copy()
        c[:, 1] = mid + (c[:, 1] - mid) * thickness_frac / max(t0, 1e-9)
    return c


def section_polygon_H(airfoil_xy: np.ndarray, chord: float, twist_rad: float,
                      r: float, ref_chord_frac: float = 0.25) -> np.ndarray:
    """Place airfoil (x/c, y/c) as a 3D section of blade 1 in frame H at radius r.

    Chord runs from LE at +y to TE at -y (travel +y); thickness along +z; pitch rotation
    about +x by twist (nose up). The reference line (x/c = ref_chord_frac) sits on the x-axis.
    Returns (N,3).
    """
    xc = (airfoil_xy[:, 0] - ref_chord_frac) * chord
    zc = airfoil_xy[:, 1] * chord
    y = -xc                     # LE (x/c=0) at +y
    z = zc
    ct, stw = np.cos(twist_rad), np.sin(twist_rad)
    yr = ct * y - stw * z       # rotation about +x: y->z for positive angle
    zr = stw * y + ct * z
    return np.column_stack([np.full_like(yr, r), yr, zr])
