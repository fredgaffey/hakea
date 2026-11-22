"""Rotor geometry: viewport surfaces, STEP/GLB/STL/OBJ export, gmsh solid meshes for FEA.

OWNER: fea agent. Single geometric description ("parts") feeds three back ends:

* ``build_surface``  numpy triangle meshes per part (viewport, GLB/STL/OBJ/JSON)
* ``export(.., "step")`` gmsh OpenCASCADE solids (ThruSections lofts) -> STEP for CAD (Keel)
* ``build_solid_mesh`` gmsh OCC solids, fragmented (conformal multi-material), tet mesh
  with per-element part/material/orientation tags for :mod:`flexblade.fea`.

Every part is a *loft* through a list of closed planar sections (airfoil rings, rectangles,
circles), so one code path covers blades, cores, rods, flexure members, fittings and hub.

Parts of blade k are built in blade-local coordinates (= frame H of blade 1) and rotated by
2π(k-1)/n about +z. Flexure members of even blades are mirrored z -> -z *at the data level*
(p0, p1, width_dir), which mirrors the hinge-axis skew (a=(sinδ,0,cosδ) -> (sinδ,0,-cosδ),
the same line as skew -δ) without mirroring the cambered blade.

Flexure members come from ``fact.generate_members(design)`` when that module is importable,
otherwise from a local fallback that builds simple FACT-like layouts from Flexure params.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, Optional

import numpy as np

from . import bladegeom
from .schema import Design, FlexureMember, Layup, PrintOrientation

HUB_MATERIAL = "pa6_cf"          # schema has no hub material; rigid printed hub assumed
N_CIRCLE = 20                    # circle tessellation for surfaces
N_AIRFOIL_SURF = 61              # points per airfoil side for viewport/mesh
N_AIRFOIL_STEP = 41              # points per airfoil side for STEP splines
WALL_MIN = 0.0005                # m, minimum printed wall around a clamped member
FIT_MARGIN = WALL_MIN            # m, fitting overhang beyond member end faces (= one wall)
BOND_TAU = 5.0e6                 # Pa, allowable shear on a bonded/clamped member-fitting interface
FAIR_SKIN = 0.0004               # m, faired skin thickness
FAIR_GAP = 0.0                   # m, clearance between member envelope and fairing skin
MM = 1000.0                      # OCC booleans are done in mm (robustness), see _occ_bodies

Role = Literal["blade", "skin", "core", "rod", "member", "member_core", "member_skin",
               "fitting", "fairing", "spoke", "hub"]

# fragment priority: an overlap volume is given to the highest-priority parent
PRIORITY = {"rod": 6, "core": 5, "member_core": 5, "member": 4, "member_skin": 4,
            "skin": 4, "blade": 4, "fitting": 3, "fairing": 0, "spoke": 1, "hub": 1}


# =========================================================================== data


@dataclass
class Section:
    """Closed planar section. kind 'poly': ring (N,3) with OCC edge segments;
    kind 'circle': center, normal, radius (ring is a tessellation for surfaces)."""
    ring: np.ndarray
    segments: list[tuple[int, int, str]] = field(default_factory=list)  # (i0,i1,'spline'|'line')
    kind: str = "poly"
    center: Optional[np.ndarray] = None
    normal: Optional[np.ndarray] = None
    radius: float = 0.0
    xref: Optional[np.ndarray] = None


@dataclass
class Part:
    name: str
    role: str
    material: str
    orientation: PrintOrientation
    blade: int                   # 1..n; 0 = hub/shared
    sections: list[Section]
    ruled: bool = True
    member: int = -1             # index into members of that blade (members only)
    rotor: str = ""              # "" single, "upper"/"lower" coaxial

    @property
    def priority(self) -> int:
        return PRIORITY.get(self.role, 2)


@dataclass
class Fitting:
    center: np.ndarray
    axes: np.ndarray             # columns e1,e2,e3 (right handed)
    half: np.ndarray


@dataclass
class RotorGeometry:
    design: Design
    parts: list[Part]
    members: dict[int, list[FlexureMember]]     # per blade, frame H
    hinge: dict[int, tuple[np.ndarray, np.ndarray]]
    blade_start: float                            # r where blade loft starts
    members_source: str
    warnings: list[str] = field(default_factory=list)
    lower: Optional["RotorGeometry"] = None          # coaxial: lower rotor in its OWN frame
    lower_map: Optional[tuple[np.ndarray, np.ndarray]] = None   # (M, shift): x_H = M x_own + shift


# =========================================================================== small helpers


def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def _rotz(psi: float) -> np.ndarray:
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _perp(t: np.ndarray, hint) -> np.ndarray:
    w = np.asarray(hint, float)
    w = w - t * (w @ t)
    if np.linalg.norm(w) < 1e-9:
        for cand in ([0, 0, 1.0], [0, 1.0, 0], [1.0, 0, 0]):
            w = np.asarray(cand) - t * (np.asarray(cand) @ t)
            if np.linalg.norm(w) > 1e-6:
                break
    return _unit(w)


def _mirror_z(v) -> tuple[float, float, float]:
    return (float(v[0]), float(v[1]), -float(v[2]))


def _rot_vec(R, v) -> tuple[float, float, float]:
    return tuple(float(x) for x in R @ np.asarray(v, float))


def _orient_rot(o: PrintOrientation, R: np.ndarray, mirror: bool = False) -> PrintOrientation:
    b, r = o.build_dir, o.raster_dir
    if mirror:
        b, r = _mirror_z(b), _mirror_z(r)
    return PrintOrientation(build_dir=_rot_vec(R, b), raster_dir=_rot_vec(R, r))


# =========================================================================== members / hinge


def hinge_axis_fallback(design: Design) -> tuple[np.ndarray, np.ndarray]:
    d = math.radians(design.flexure.skew_deg)
    # same convention as fact.hinge_axis: a = (sinδ, 0, −cosδ) so that lag ζ (about −z)
    # about the skewed axis gives pitch θ = +ζ·tanδ (DESIGN.md)
    return (np.array([design.flexure.hinge_radius, 0.0, 0.0]),
            np.array([math.sin(d), 0.0, -math.cos(d)]))


def _hinge_frame(point, axis):
    a = _unit(axis)
    s = _perp(a, [1.0, 0, 0])
    t = np.cross(a, s)
    return np.asarray(point, float), s, t, a


def fallback_members(design: Design) -> list[FlexureMember]:
    """Simple FACT-like layouts from Flexure parameters (blade 1, frame H).

    p0 = hub side, p1 = blade side. Hinge frame: a = hinge axis, s = span ⟂ a, t = a × s.
    """
    fx = design.flexure
    P, s, t, a = _hinge_frame(*hinge_axis_fallback(design))
    L = fx.length
    half_c = math.radians(fx.cross_angle_deg) / 2
    common = dict(thickness=fx.thickness, taper=fx.taper, layup=fx.layup,
                  orientation=fx.orientation)

    def plate(d, offset=np.zeros(3), width=fx.width, kind="blade", shape="rect", wdir=a):
        d = _unit(d)
        p0 = P + offset - d * L / 2
        p1 = P + offset + d * L / 2
        return FlexureMember(kind=kind, p0=tuple(p0), p1=tuple(p1), width=width,
                             width_dir=tuple(_unit(wdir)), shape=shape, **common)

    topo = fx.topology
    out: list[FlexureMember] = []
    if topo == "custom":
        out = list(fx.members)
    elif topo == "skewed_cross_blade":
        d1 = math.cos(half_c) * s + math.sin(half_c) * t
        d2 = math.cos(half_c) * s - math.sin(half_c) * t
        out = [plate(d1), plate(d2)]
    elif topo in ("skewed_single_blade", "integral_taper"):
        out = [plate(s)]
    elif topo == "wire_triplet":
        d1 = math.cos(half_c) * s + math.sin(half_c) * t
        d2 = math.cos(half_c) * s - math.sin(half_c) * t
        h = fx.width / 2
        out = [plate(d1, -h * a, width=fx.thickness, kind="wire", shape="circle"),
               plate(s, 0 * a, width=fx.thickness, kind="wire", shape="circle"),
               plate(d2, h * a, width=fx.thickness, kind="wire", shape="circle")]
    elif topo == "rigid":
        m = plate(s, width=fx.width, kind="rigid")
        out = [m.model_copy(update=dict(thickness=max(fx.thickness, 0.4 * fx.width)))]
    if fx.tension_member is not None and topo != "custom":
        out.append(fx.tension_member)
    if fx.twist_deg and topo != "custom":
        # pre-twist: rotate width_dir about member axis by twist (uniform)
        tw = math.radians(fx.twist_deg)
        tw_out = []
        for m in out:
            tt = _unit(np.subtract(m.p1, m.p0))
            w = _perp(tt, m.width_dir)
            n = np.cross(tt, w)
            tw_out.append(m.model_copy(update=dict(
                width_dir=tuple(math.cos(tw) * w + math.sin(tw) * n))))
        out = tw_out
    return out


def get_members(design: Design) -> tuple[list[FlexureMember], tuple[np.ndarray, np.ndarray], str]:
    """Blade-1 members + hinge axis, from fact.py if available, else fallback."""
    try:
        from . import fact  # type: ignore
        members = list(fact.generate_members(design))
        try:
            p, d = fact.hinge_axis(design)
            hinge = (np.asarray(p, float), _unit(d))
        except Exception:  # noqa: BLE001
            hinge = hinge_axis_fallback(design)
        return members, hinge, "fact"
    except (ImportError, AttributeError, NotImplementedError):
        pass
    return fallback_members(design), hinge_axis_fallback(design), "fallback"


def members_for_blade(members: list[FlexureMember], k: int, n: int) -> list[FlexureMember]:
    """Transform blade-1 members to blade k (z-mirror for even k, then rotate)."""
    R = _rotz(2 * math.pi * (k - 1) / n)
    mirror = (k % 2 == 0)
    out = []
    for m in members:
        p0, p1, wd = m.p0, m.p1, m.width_dir
        if mirror:
            p0, p1, wd = _mirror_z(p0), _mirror_z(p1), _mirror_z(wd)
        out.append(m.model_copy(update=dict(
            p0=_rot_vec(R, p0), p1=_rot_vec(R, p1), width_dir=_rot_vec(R, wd),
            orientation=_orient_rot(m.orientation, R, mirror))))
    return out


def hinge_for_blade(hinge, k: int, n: int):
    R = _rotz(2 * math.pi * (k - 1) / n)
    p, d = hinge
    if k % 2 == 0:
        p, d = np.array(_mirror_z(p)), -np.array(_mirror_z(d))   # keep axis pointing +z-ish
    return R @ np.asarray(p, float), R @ np.asarray(d, float)


def member_frame(m: FlexureMember):
    p0, p1 = np.asarray(m.p0, float), np.asarray(m.p1, float)
    L = float(np.linalg.norm(p1 - p0))
    t = (p1 - p0) / L
    w = _perp(t, m.width_dir)
    nrm = np.cross(t, w)
    return p0, p1, L, t, w, nrm


def member_end_corners(m: FlexureMember, end: int) -> np.ndarray:
    p0, p1, L, t, w, n = member_frame(m)
    c = p0 if end == 0 else p1
    th = m.thickness * (1.0 if end == 0 else m.taper)
    wid = m.width if m.shape == "rect" else m.thickness * (1.0 if end == 0 else m.taper)
    return np.array([c + sw * wid / 2 * w + sn * th / 2 * n for sw in (-1, 1) for sn in (-1, 1)])


# =========================================================================== sections


def _rect_section(c, w, n, wid, th) -> Section:
    ring = np.array([c - wid / 2 * w - th / 2 * n, c + wid / 2 * w - th / 2 * n,
                     c + wid / 2 * w + th / 2 * n, c - wid / 2 * w + th / 2 * n])
    return Section(ring=ring, segments=[(i, (i + 1) % 4, "line") for i in range(4)])


def _circle_section(c, normal, r, xref) -> Section:
    normal = _unit(normal)
    e1 = _perp(normal, xref)
    e2 = np.cross(normal, e1)
    ang = np.linspace(0, 2 * np.pi, N_CIRCLE, endpoint=False)
    ring = c + r * (np.outer(np.cos(ang), e1) + np.outer(np.sin(ang), e2))
    return Section(ring=ring, kind="circle", center=np.asarray(c, float), normal=normal,
                   radius=float(r), xref=e1)


def _box_part(name, role, material, orientation, blade, fit: Fitting) -> Part:
    e1, e2, e3 = fit.axes.T
    h = fit.half
    secs = [_rect_section(fit.center + s * h[0] * e1, e2, e3, 2 * h[1], 2 * h[2]) for s in (-1, 1)]
    return Part(name, role, material, orientation, blade, secs, ruled=True)


def _airfoil_split(xy: np.ndarray):
    """Return (x_le_to_te_upper, y_upper, x_lower, y_lower) as x-ascending arrays."""
    ile = int(np.argmin(xy[:, 0]))
    up = xy[ile::-1]
    lo = xy[ile:]
    return up[:, 0], up[:, 1], lo[:, 0], lo[:, 1]


def _airfoil_ring(xy: np.ndarray) -> tuple[np.ndarray, list]:
    """Clean closed airfoil loop -> ring (no duplicate closure) + OCC segments."""
    xy = np.asarray(xy, float)
    if np.linalg.norm(xy[0] - xy[-1]) < 1e-9:
        xy = xy[:-1]
        segs = [(0, len(xy) - 1, "spline"), (len(xy) - 1, 0, "line")]
    else:
        segs = [(0, len(xy) - 1, "spline"), (len(xy) - 1, 0, "line")]
    return xy, segs


def _core_xy(xy: np.ndarray, ts_c: float, n: int) -> Optional[np.ndarray]:
    """Inset core loop (normalised by chord) for skin thickness ts_c = t_skin/chord.

    Vertical inset of upper/lower surfaces, truncated where local thickness < 2.2 ts
    (blunt ends). Same point count/order as an airfoil ring: TE upper -> LE -> TE lower.
    """
    xu, yu, xl, yl = _airfoil_split(xy)
    xs = np.linspace(0, 1, 400)
    thick = np.interp(xs, xu, yu) - np.interp(xs, xl, yl)
    ok = (thick > 2.2 * ts_c) & (xs > ts_c)
    if ok.sum() < 3:
        return None
    idx = np.where(ok)[0]
    # take the largest contiguous run
    runs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
    run = max(runs, key=len)
    xa, xb = xs[run[0]], xs[run[-1]]
    beta = np.linspace(0, np.pi, n)
    x = xa + (xb - xa) * 0.5 * (1 - np.cos(beta))
    ycu = np.interp(x, xu, yu) - ts_c
    ycl = np.interp(x, xl, yl) + ts_c
    upper = np.column_stack([x[::-1], ycu[::-1]])
    lower = np.column_stack([x, ycl])
    return np.vstack([upper, lower])


def _camber_point(xy: np.ndarray, xc: float) -> tuple[float, float]:
    xu, yu, xl, yl = _airfoil_split(xy)
    return xc, 0.5 * (np.interp(xc, xu, yu) + np.interp(xc, xl, yl))


def _thickness_at(xy: np.ndarray, xc: float) -> float:
    xu, yu, xl, yl = _airfoil_split(xy)
    return float(np.interp(xc, xu, yu) - np.interp(xc, xl, yl))


# =========================================================================== blade stations


def _blade_stations(design: Design, r_start: float, n_side: int, n_span: Optional[int] = None):
    """List of (r, chord, twist, xy) from r_start to tip (interpolated root station).

    n_span: resample to this many sections with a monotone cubic (PCHIP) through the
    stations (smooth viewport surface); None keeps the design stations."""
    blade = design.blade
    st = bladegeom.resolve_stations(blade)
    R = blade.radius
    r = np.array([s.r_frac * R for s in st])
    chord = np.array([s.chord for s in st])
    twist = np.radians([s.twist_deg + design.op.collective_deg for s in st])
    tfrac = [s.thickness_frac for s in st]
    default_t = None
    if blade.airfoil.kind == "naca4":
        default_t = int(blade.airfoil.naca[2:4]) / 100.0
    tf = np.array([t if t is not None else (default_t if default_t is not None else np.nan)
                   for t in tfrac], float)
    out = []
    keep = r > r_start + 1e-6
    rr = np.concatenate([[r_start], r[keep]]) if r_start > r[0] - 1e-9 else r
    interp = {k: (lambda x, v=v: float(np.interp(x, r, v))) for k, v in
              (("c", chord), ("tw", twist), ("t", tf))}
    if n_span and n_span > len(rr) and len(r) >= 3:
        from scipy.interpolate import PchipInterpolator
        rr = np.linspace(rr[0], rr[-1], n_span)
        for k, v in (("c", chord), ("tw", twist), ("t", tf)):
            if not np.any(np.isnan(v)):
                f = PchipInterpolator(r, v, extrapolate=True)
                interp[k] = (lambda x, f=f: float(f(x)))
    for ri in rr:
        c = interp["c"](ri)
        tw = interp["tw"](ri)
        t = interp["t"](ri) if not np.all(np.isnan(tf)) else None
        if t is not None and np.isnan(t):
            t = None
        xy = bladegeom.airfoil_coords(blade.airfoil, t, n=n_side)
        out.append((float(ri), c, tw, xy))
    return out


def _airfoil_section(xy, chord, twist, r, ref) -> Section:
    ring_xy, segs = _airfoil_ring(xy)
    ring = bladegeom.section_polygon_H(ring_xy, chord, twist, r, ref)
    return Section(ring=ring, segments=segs)


def _core_section(core_xy, chord, twist, r, ref) -> Section:
    n = len(core_xy) // 2
    ring = bladegeom.section_polygon_H(core_xy, chord, twist, r, ref)
    segs = [(0, n - 1, "spline"), (n - 1, n, "line"), (n, 2 * n - 1, "spline"),
            (2 * n - 1, 0, "line")]
    return Section(ring=ring, segments=segs)


# =========================================================================== assembly


def _layup_parts(prefix: str, blade_k: int, layup: Layup, orient: PrintOrientation,
                 outer_secs: list[Section], core_secs: Optional[list[Section]],
                 rods: list[list[Section]], ruled: bool, base_role: str,
                 member: int = -1) -> list[Part]:
    parts = []
    role_skin = "skin" if base_role == "blade" else "member_skin"
    role_core = "core" if base_role == "blade" else "member_core"
    if layup.kind == "core_skin" and core_secs is not None and layup.skin_material:
        parts.append(Part(f"{prefix}_skin", role_skin, layup.skin_material, orient, blade_k,
                          outer_secs, ruled, member))
        parts.append(Part(f"{prefix}_core", role_core, layup.material, orient, blade_k,
                          core_secs, ruled, member))
    else:
        parts.append(Part(prefix, base_role, layup.material, orient, blade_k, outer_secs,
                          ruled, member))
    if layup.kind == "rod_insert" and layup.insert_material:
        for i, rs in enumerate(rods):
            axis = _unit(rs[-1].center - rs[0].center)
            ro = PrintOrientation(build_dir=tuple(_perp(axis, [0, 0, 1.0])),
                                  raster_dir=tuple(axis))
            parts.append(Part(f"{prefix}_rod{i}", "rod", layup.insert_material, ro, blade_k,
                              rs, True, member))
    return parts


def _member_parts(m: FlexureMember, name: str, k: int, idx: int) -> list[Part]:
    p0, p1, L, t, w, n = member_frame(m)
    twist_sections = 2
    secs, core, rods = [], [], []
    lay = m.layup
    for j in range(twist_sections):
        f = j / (twist_sections - 1)
        c = p0 + f * (p1 - p0)
        th = m.thickness * (1 + (m.taper - 1) * f)
        if m.shape == "circle":
            secs.append(_circle_section(c, t, th / 2, w))
            if lay.kind == "core_skin" and th > 2.5 * lay.skin_thickness:
                core.append(_circle_section(c, t, th / 2 - lay.skin_thickness, w))
        else:
            secs.append(_rect_section(c, w, n, m.width, th))
            ts = lay.skin_thickness
            if lay.kind == "core_skin" and th > 2.5 * ts and m.width > 2.5 * ts:
                core.append(_rect_section(c, w, n, m.width - 2 * ts, th - 2 * ts))
    if lay.kind == "rod_insert" and lay.insert_diameter > 0 and m.shape == "rect":
        for pos in (lay.insert_positions or [0.5]):
            off = (pos - 0.5) * m.width * w
            rods.append([_circle_section(p0 + off + f * (p1 - p0), t, lay.insert_diameter / 2, w)
                         for f in (0.0, 1.0)])
    role = "fitting" if m.kind == "rigid" else "member"
    return _layup_parts(name, k, lay, m.orientation, secs,
                        core if len(core) == len(secs) else None, rods, True,
                        "member" if role == "member" else "fitting", idx)


def _shank_sections(fit: Fitting, bg, design: Design, r_start: float, n_side: int) -> list[Section]:
    """Two rectangular sections (blade-1 frame): the fitting's outer end face and a rectangle
    placed just inside the blade root section (PCA of the root airfoil in its y-z plane)."""
    e1, e2, e3 = fit.axes.T
    h = fit.half
    c0 = fit.center + h[0] * e1
    ring0 = _rect_section(c0, e2, e3, 2 * h[1], 2 * h[2]).ring
    st = _blade_stations(design, r_start, n_side)[0]
    r, chord, tw, xy = st
    root = bg.section_polygon_H(_airfoil_ring(xy)[0], chord, tw, r, design.blade.elastic_axis_chord_frac)
    yz = root[:, 1:]
    cen = yz.mean(0)
    u, sv, vt = np.linalg.svd(yz - cen, full_matrices=False)
    ext = np.ptp((yz - cen) @ vt.T, axis=0)
    x_end = r + 0.0006
    ax_maj = np.array([0.0, *vt[0]])
    ax_min = np.array([0.0, *vt[1]])
    c1 = np.array([x_end, *cen])
    ring1 = _rect_section(c1, ax_maj, ax_min, 0.6 * ext[0], 0.7 * ext[1]).ring
    axis = c1 - c0

    def _wind(r):    # winding sense of a quad about the loft axis
        return np.sign(np.cross(r[1] - r[0], r[2] - r[1]) @ axis) or 1.0
    if _wind(ring1) != _wind(ring0):
        ring1 = ring1[::-1]
    best = min((float(((np.roll(ring1, sh, axis=0) - ring0) ** 2).sum()), sh) for sh in range(4))
    ring1 = np.roll(ring1, best[1], axis=0)
    # smooth (C1, zero-slope ends) blend of the quad corners from the fitting face to the root
    # section, so the hub-fitting -> airfoil transition has no step in height or width
    segs = [(i, (i + 1) % 4, "line") for i in range(4)]
    out = []
    for f in np.linspace(0.0, 1.0, 5):
        u = f * f * (3 - 2 * f)
        out.append(Section(ring=(1 - u) * ring0 + u * ring1, segments=segs))
    return out


def _clamp_length(design: Design, members, r_start: float) -> float:
    from .materials import get_material
    b = design.blade
    try:
        rho = get_material(b.layup.material).rho
    except Exception:
        rho = 1300.0
    st = bladegeom.resolve_stations(b)
    ch = float(np.mean([x.chord for x in st])) if hasattr(st[0], "chord") else 0.012
    from .rootdrag import root_thickness
    m_bl = 0.6 * rho * ch * root_thickness(design) * 0.5 * (b.radius - r_start)
    om = 2 * math.pi * (design.op.rpm or 12000.0) / 60.0
    F = m_bl * om ** 2 * 0.5 * (b.radius + r_start)
    per = sum(2 * (m.width + m.thickness) if m.shape == "rect" else math.pi * m.thickness
              for m in members if m.kind != "rigid") or 0.02
    return float(np.clip(F / (per * BOND_TAU), 2 * WALL_MIN, 0.1 * 0.02 + 0.004))


def _fitting_box(corners: np.ndarray, P, s, t, a, extend_out: float, extend_in: float,
                 margin: float) -> Fitting:
    loc = (corners - P) @ np.column_stack([s, t, a])
    lo = loc.min(0) - margin
    hi = loc.max(0) + margin
    lo[0] -= extend_in
    hi[0] += extend_out
    c_loc = 0.5 * (lo + hi)
    axes = np.column_stack([s, t, a])
    return Fitting(center=P + axes @ c_loc, axes=axes, half=0.5 * (hi - lo))


def lower_design(design: Design) -> Design:
    """Coaxial lower rotor as a single-rotor Design in its own (mirrored) frame."""
    cx = design.coaxial
    op = design.op.model_copy(update=dict(
        collective_deg=design.op.collective_deg + cx.lower_collective_offset_deg))
    return design.model_copy(update=dict(
        layout="single", blade=cx.lower_blade or design.blade,
        flexure=cx.lower_flexure or design.flexure, motor=cx.lower_motor or design.motor, op=op))


def coaxial_map(design: Design) -> tuple[np.ndarray, np.ndarray]:
    """Lower-rotor own frame -> frame H: mirror about the xz plane (y -> -y; counter-rotation,
    leading edge of lower blade 1 faces -y) and shift to z = -spacing_frac * R_upper."""
    M = np.diag([1.0, -1.0, 1.0])
    shift = np.array([0.0, 0.0, -design.coaxial.spacing_frac * design.blade.radius])
    return M, shift


def _map_part(p: Part, M: np.ndarray, shift: np.ndarray, prefix: str, rotor: str) -> Part:
    secs = []
    for sec in p.sections:
        ns = Section(ring=sec.ring @ M.T + shift, segments=sec.segments, kind=sec.kind,
                     radius=sec.radius)
        if sec.kind == "circle":
            ns.center, ns.normal, ns.xref = M @ sec.center + shift, M @ sec.normal, M @ sec.xref
        secs.append(ns)
    o = PrintOrientation(build_dir=tuple(M @ np.asarray(p.orientation.build_dir, float)),
                         raster_dir=tuple(M @ np.asarray(p.orientation.raster_dir, float)))
    return Part(prefix + p.name, p.role, p.material, o, p.blade, secs, p.ruled, p.member, rotor)


def build_geometry(design: Design, n_side: int = N_AIRFOIL_SURF,
                   blades: Optional[list[int]] = None, n_span: Optional[int] = None) -> RotorGeometry:
    """Part list in frame H. Coaxial layout: upper_* parts (primary rotor, +Ω) and lower_*
    parts (lower_design mirrored y->-y, at z = -spacing·R). members/hinge refer to the upper
    rotor; geo.lower holds the lower rotor in its own frame and geo.lower_map the mapping."""
    if design.layout != "coaxial":
        return _build_single(design, n_side, blades, n_span)
    gu = _build_single(design, n_side, blades, n_span)
    gl = _build_single(lower_design(design), n_side, blades, n_span)
    M, shift = coaxial_map(design)
    parts = [Part("upper_" + p.name, p.role, p.material, p.orientation, p.blade, p.sections,
                  p.ruled, p.member, "upper") for p in gu.parts]
    parts += [_map_part(p, M, shift, "lower_", "lower") for p in gl.parts]
    return RotorGeometry(design, parts, gu.members, gu.hinge, gu.blade_start, gu.members_source,
                         gu.warnings + ["lower: " + w for w in gl.warnings], gl, (M, shift))


def _build_single(design: Design, n_side: int = N_AIRFOIL_SURF,
                  blades: Optional[list[int]] = None, n_span: Optional[int] = None) -> RotorGeometry:
    """Single rotor part list (all blades unless `blades` given) in frame H."""
    blade = design.blade
    n = blade.n_blades
    blades = blades or list(range(1, n + 1))
    members1, hinge1, src = get_members(design)
    warnings: list[str] = []

    # ---- blade-1 fittings in hinge frame
    P, s, t, a = _hinge_frame(*hinge1)
    flex_members = [m for m in members1]
    cos_sx = max(abs(s[0]), 0.2)
    if flex_members:
        c0 = np.vstack([member_end_corners(m, 0) for m in flex_members])
        c1 = np.vstack([member_end_corners(m, 1) for m in flex_members])
    else:
        c0 = np.array([P - 0.002 * s])
        c1 = np.array([P + 0.002 * s])
    r_root = bladegeom.resolve_stations(blade)[0].r_frac * blade.radius
    x_mem_max = float(c1[:, 0].max())
    r_start = max(r_root, x_mem_max + 0.0003)
    if r_start > r_root + 1e-6:
        warnings.append(f"blade loft starts at r={r_start*1e3:.1f} mm (flexure outer end) "
                        f"instead of root station r={r_root*1e3:.1f} mm")
    if r_start >= blade.radius * 0.95:
        raise ValueError("flexure extends beyond the blade")
    L_ref = max(np.linalg.norm(c1.mean(0) - c0.mean(0)), 1e-3)
    # compact clamp blocks; the blade-side one is bridged to the blade root by a shank part
    # clamp/bond length from requirements: centrifugal load of the blade carried through the
    # member-fitting interface / (bonded perimeter x allowable shear), at least 2 walls
    ext_out = ext_in = _clamp_length(design, flex_members, r_start)
    fit_blade = _fitting_box(c1, P, s, t, a, ext_out, 0.0, FIT_MARGIN)
    # bridge: ruled loft from the outer face of the blade-side fitting to a rectangle inside
    # the blade root section (no gap between flexure and blade)
    shank_secs = _shank_sections(fit_blade, bladegeom, design, r_start, n_side)
    fit_hub = _fitting_box(c0, P, s, t, a, 0.0, ext_in, FIT_MARGIN)
    hub_r = design.flexure.hub_radius
    hub_corners = fit_hub.center + (np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1)
                                              for k in (-1, 1)]) * fit_hub.half) @ fit_hub.axes.T
    hub_h = max(2 * float(np.abs(hub_corners[:, 2]).max()), 0.004)
    hub_h = min(hub_h, max(0.004, 2 * hub_r))
    x_spoke = max(hub_r * 0.9, float(c0[:, 0].min()) - 1e-4)
    spoke = Fitting(center=np.array([x_spoke / 2, 0, 0]), axes=np.eye(3),
                    half=np.array([x_spoke / 2, max(fit_hub.half[1], 0.6 * hub_r), hub_h / 4]))

    # ---- blade sections (blade-1 frame)
    stations = _blade_stations(design, r_start, n_side, n_span)
    ref = blade.elastic_axis_chord_frac
    outer = [_airfoil_section(xy, c, tw, r, ref) for (r, c, tw, xy) in stations]
    lay = blade.layup
    core = None
    if lay.kind == "core_skin" and lay.skin_material and lay.skin_thickness > 0:
        core = []
        for (r, c, tw, xy) in stations:
            cxy = _core_xy(xy, lay.skin_thickness / c, n_side)
            if cxy is None:
                break
            core.append(_core_section(cxy, c, tw, r, ref))
        if len(core) < 2:
            warnings.append("skin too thick: no core region")
            core = None
        elif len(core) < len(outer):
            warnings.append(f"core stops at r={stations[len(core)-1][0]*1e3:.1f} mm (thin)")
    rods = []
    if lay.kind == "rod_insert" and lay.insert_material and lay.insert_diameter > 0:
        for pos in lay.insert_positions:
            cs = []
            for (r, c, tw, xy) in stations:
                if _thickness_at(xy, pos) * c < 1.3 * lay.insert_diameter:
                    break
                cp = np.array([_camber_point(xy, pos)])
                cs.append(bladegeom.section_polygon_H(cp, c, tw, r, ref)[0])
            if len(cs) < 2:
                warnings.append(f"rod at x/c={pos} does not fit (diameter vs thickness)")
                continue
            cs = np.array(cs)
            tang = np.gradient(cs, axis=0)
            rods.append([_circle_section(cs[i], tang[i], lay.insert_diameter / 2, [0, 0, 1.0])
                         for i in range(len(cs))])
            if len(cs) < len(stations):
                warnings.append(f"rod x/c={pos} ends at r={stations[len(cs)-1][0]*1e3:.1f} mm")

    # ---- assemble per blade
    parts: list[Part] = []
    members_all: dict[int, list[FlexureMember]] = {}
    hinge_all = {}
    for k in blades:
        R = _rotz(2 * math.pi * (k - 1) / n)
        mirror = (k % 2 == 0)

        def tr_sec(sec: Section, mirror_z=False) -> Section:
            ring = sec.ring.copy()
            if mirror_z:
                ring[:, 2] *= -1
            ring = ring @ R.T
            out = Section(ring=ring, segments=sec.segments, kind=sec.kind, radius=sec.radius)
            if sec.kind == "circle":
                c, nn, xr = sec.center.copy(), sec.normal.copy(), sec.xref.copy()
                if mirror_z:
                    c[2], nn[2], xr[2] = -c[2], -nn[2], -xr[2]
                out.center, out.normal, out.xref = R @ c, R @ nn, R @ xr
            return out

        bo = _orient_rot(blade.orientation, R)
        parts += _layup_parts(f"blade{k}", k, lay, bo, [tr_sec(x) for x in outer],
                              [tr_sec(x) for x in core] if core else None,
                              [[tr_sec(x) for x in rs] for rs in rods], False, "blade")
        mk = members_for_blade(members1, k, n)
        members_all[k] = mk
        hinge_all[k] = hinge_for_blade(hinge1, k, n)
        for i, m in enumerate(mk):
            parts += _member_parts(m, f"blade{k}_flex{i}", k, i)

        def tr_fit(f: Fitting) -> Fitting:
            c, ax = f.center.copy(), f.axes.copy()
            if mirror:
                c[2] = -c[2]
                ax[2, :] *= -1
                ax[:, 2] = np.cross(ax[:, 0], ax[:, 1])     # restore right-handedness
            return Fitting(R @ c, R @ ax, f.half)

        fo = _orient_rot(design.flexure.orientation, R, mirror)
        parts.append(_box_part(f"blade{k}_root_fitting", "fitting", lay.material, bo, k,
                               tr_fit(fit_blade)))
        parts.append(Part(f"blade{k}_shank", "fitting", lay.material, bo, k,
                          [tr_sec(x) for x in shank_secs], ruled=True))
        parts.append(_box_part(f"blade{k}_hub_fitting", "fitting",
                               HUB_MATERIAL, PrintOrientation(), k, tr_fit(fit_hub)))   # rigid: a TPU hub fitting made the real part ~24x softer than modelled (CalculiX, E01/E02)
        parts.append(_box_part(f"blade{k}_spoke", "spoke", HUB_MATERIAL, PrintOrientation(), 0,
                               tr_fit(spoke)))
        if design.flexure.fairing:
            parts += _fairing_parts(design, [k], fit_hub, fit_blade, c0, c1, r_start, tr_fit, tr_sec)
    hub_secs = [_circle_section(np.array([0, 0, z]), [0, 0, 1.0], hub_r, [1.0, 0, 0])
                for z in (-hub_h / 2, hub_h / 2)]
    parts.append(Part("hub", "hub", HUB_MATERIAL, PrintOrientation(), 0, hub_secs, True))
    return RotorGeometry(design, parts, members_all, hinge_all, r_start, src, warnings)


def _fairing_parts(design, blades, fit_hub, fit_blade, c0, c1, r_start, tr_fit, tr_sec) -> list[Part]:
    """Thin faired skin: a top and a bottom lens-shaped cap (FAIR_SKIN thick, FAIR_GAP clear of
    the hardware envelope, smooth thickness taper to the ends) from the hub fitting to the blade
    root. The skin carries no load; it only streamlines the root hardware (Cd 0.4)."""
    pts = []
    for f in (fit_hub, fit_blade):
        pts.append(f.center + (np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
                               * f.half) @ f.axes.T)
    cc = [c0[i] + (c1[i] - c0[i]) * f for i in range(min(len(c0), len(c1))) for f in np.linspace(0, 1, 15)]
    pts = np.vstack(pts + [c0, c1, np.array(cc)])
    x0, x1 = float(pts[:, 0].min()), float(r_start + 0.0006)
    y0, y1 = float(pts[:, 1].min()), float(pts[:, 1].max())
    xs = x0 + (x1 - x0) * np.linspace(0.0, 1.0, 13)
    win = 0.6 * (x1 - x0) / 12
    zhi, zlo = [], []
    for xi in xs:
        zz = pts[np.abs(pts[:, 0] - xi) <= win, 2]
        zhi.append(float(zz.max(initial=0.0)))
        zlo.append(float(zz.min(initial=0.0)))
    out = []
    for k in blades:
        for sgn, nm in ((1, "top"), (-1, "bot")):
            secs = []
            for xi, zh, zl in zip(xs, zhi, zlo):
                f = (xi - x0) / (x1 - x0)
                th = FAIR_SKIN * (0.25 + 0.75 * math.sin(math.pi * min(max(f, 0.0), 1.0)) ** 0.5)
                z0, z1 = (zh, zh + th) if sgn > 0 else (zl - th, zl)
                ring = np.array([[xi, y0, z0], [xi, y1, z0], [xi, y1, z1], [xi, y0, z1]])
                secs.append(tr_sec(Section(ring=ring, segments=[(i, (i + 1) % 4, "line") for i in range(4)])))
            out.append(Part(f"blade{k}_fairing_{nm}", "fairing", "tpu_95a", PrintOrientation(), k, secs, ruled=True))
    return out


# =========================================================================== surface triangulation


def _ear_clip(pts2: np.ndarray) -> list[tuple[int, int, int]]:
    """Ear clipping of a simple polygon (CCW or CW). Returns triangles, CCW in 2D."""
    n = len(pts2)
    idx = list(range(n))
    area = 0.5 * np.sum(pts2[:, 0] * np.roll(pts2[:, 1], -1) - np.roll(pts2[:, 0], -1) * pts2[:, 1])
    if area < 0:
        idx.reverse()
    tris = []

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    guard = 0
    while len(idx) > 3 and guard < 10 * n * n:
        guard += 1
        m = len(idx)
        clipped = False
        # pick the ear with best (largest) minimum angle-ish: simple first-found
        for i in range(m):
            ia, ib, ic = idx[i - 1], idx[i], idx[(i + 1) % m]
            A, B, C = pts2[ia], pts2[ib], pts2[ic]
            if cross(A, B, C) <= 1e-30:
                continue
            ok = True
            for j in idx:
                if j in (ia, ib, ic):
                    continue
                Pp = pts2[j]
                if cross(A, B, Pp) >= 0 and cross(B, C, Pp) >= 0 and cross(C, A, Pp) >= 0:
                    ok = False
                    break
            if ok:
                tris.append((ia, ib, ic))
                idx.pop(i)
                clipped = True
                break
        if not clipped:      # degenerate: fall back to fan
            for i in range(1, len(idx) - 1):
                tris.append((idx[0], idx[i], idx[i + 1]))
            return tris
    if len(idx) == 3:
        tris.append(tuple(idx))
    return tris


def _cap(ring: np.ndarray) -> np.ndarray:
    c = ring.mean(0)
    nrm = np.zeros(3)
    for i in range(len(ring)):     # Newell normal
        p, q = ring[i] - c, ring[(i + 1) % len(ring)] - c
        nrm += np.cross(p, q)
    nrm = _unit(nrm)
    e1 = _perp(nrm, ring[0] - c)
    e2 = np.cross(nrm, e1)
    p2 = np.column_stack([(ring - c) @ e1, (ring - c) @ e2])
    return np.array(_ear_clip(p2), int).reshape(-1, 3)


def loft_surface(part: Part) -> tuple[np.ndarray, np.ndarray]:
    """Watertight triangle mesh of a loft part (outward normals)."""
    secs = part.sections
    N = len(secs[0].ring)
    V = np.vstack([s.ring for s in secs])
    K = len(secs)
    faces = []
    for i in range(K - 1):
        for j in range(N):
            a, b = i * N + j, i * N + (j + 1) % N
            c, d = (i + 1) * N + (j + 1) % N, (i + 1) * N + j
            faces += [(a, b, c), (a, c, d)]
    F = np.array(faces, int)
    cen = V.mean(0)
    # sides keep ring order (consistent by construction); orient each cap against the ring
    # traversal (cap0 reversed, cap1 forward) so the surface is a consistently wound closed
    # mesh even for twisted/offset sections, then flip globally if the volume is negative
    caps = []
    for ci, sec in enumerate((secs[0], secs[-1])):
        T = _cap(sec.ring) + (0 if ci == 0 else (K - 1) * N)
        if len(T) == 0:
            continue
        c = sec.ring.mean(0)
        N_ring = np.cross(sec.ring - c, np.roll(sec.ring, -1, axis=0) - c).sum(0)
        p = V[T[0]]
        nn = np.cross(p[1] - p[0], p[2] - p[0])
        follows = nn @ N_ring > 0
        if follows == (ci == 0):
            T = T[:, ::-1]
        caps.append(T)
    F = np.vstack([F] + caps)
    vol = np.einsum("ij,ij->i", V[F[:, 0]] - cen, np.cross(V[F[:, 1]] - cen, V[F[:, 2]] - cen)).sum()
    if vol < 0:
        F = F[:, ::-1]
    return V, F


N_SPAN_SURF = 28                 # blade sections in the viewport surface (spline-resampled)


_STL_CACHE: dict = {}


def _load_topopt_mesh(path: str):
    from .topopt.export import read_stl
    st = os.stat(path)
    key = (path, st.st_mtime_ns, st.st_size)
    if key not in _STL_CACHE:
        if len(_STL_CACHE) > 16:
            _STL_CACHE.clear()
        _STL_CACHE[key] = read_stl(path)
    return _STL_CACHE[key]


def _with_topopt_meshes(design: Design, parts: list[dict], blades: list[int], warnings: list) -> list[dict]:
    """Replace the drawn FACT member parts by the topology-optimised flexure surfaces
    (flexure.topopt.parts: blade-1 STL per material) on every blade (z-mirror for even k)."""
    tp = design.flexure.topopt
    if tp is None or not tp.parts:
        return parts
    n = design.blade.n_blades
    meshes = []
    for pd in tp.parts:
        path = pd.get("stl", "")
        if path and not os.path.isabs(path) and tp.source:
            # source: absolute, relative to the cwd, or relative to the flexblade project dir
            proj = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            cands = [os.path.join(tp.source, path), os.path.join(proj, tp.source, path)]
            path = next((c for c in cands if os.path.exists(c)), cands[0])
        try:
            meshes.append((pd, *_load_topopt_mesh(path)))
        except OSError as e:
            warnings.append(f"topopt mesh {path}: {e}")
    if not meshes:
        return parts
    keep = [p for p in parts if not (p["role"] in ("member", "member_core", "member_skin", "rod", "wire")
                                     and str(p["name"]).startswith(f"blade{p['blade']}_flex"))]
    for k in blades:
        R = _rotz(2 * math.pi * (k - 1) / n)
        mirror = (k % 2 == 0)
        for pd, V, F in meshes:
            Vk = V.copy()
            Fk = F
            if mirror:
                Vk[:, 2] *= -1
                Fk = F[:, ::-1]
            mat = pd.get("material", "tpu_95a")
            role = pd.get("role", "member")
            keep.append(dict(name=f"blade{k}_{pd.get('name', 'topopt')}", role=role, material=mat,
                             blade=k, member=-1, rotor="", vertices=Vk @ R.T, faces=np.asarray(Fk),
                             color=color_hex(mat, role), color_base=color_hex(mat),
                             orientation=_orient_rot(design.flexure.orientation, R, mirror).model_dump(mode="json")))
    return keep


def build_surface(design: Design, as_lists: bool = False, n_side: int = N_AIRFOIL_SURF,
                  n_span: Optional[int] = N_SPAN_SURF) -> dict:
    """Triangle meshes for the viewport.

    Returns dict(parts=[{name, role, material, blade, member, vertices (V,3), faces (F,3)}],
    hinges=[{blade, point, dir}], members=[{blade, index, p0, p1}], blade_start, radius,
    members_source, warnings). Arrays are numpy unless as_lists.
    """
    geo = build_geometry(design, n_side=n_side, n_span=n_span)
    parts = []
    for p in geo.parts:
        V, F = loft_surface(p)
        parts.append(dict(name=p.name, role=p.role, material=p.material, blade=p.blade,
                          member=p.member, rotor=p.rotor, vertices=V, faces=F,
                          color=color_hex(p.material, p.role), color_base=color_hex(p.material),
                          orientation=p.orientation.model_dump(mode="json")))
    if design.flexure.topology == "topopt" and design.layout != "coaxial":
        parts = _with_topopt_meshes(design, parts, sorted(geo.hinge), geo.warnings)
    out = dict(parts=parts,
               hinges=[dict(blade=k, point=geo.hinge[k][0], dir=geo.hinge[k][1])
                       for k in sorted(geo.hinge)],
               members=[dict(blade=k, index=i, p0=m.p0, p1=m.p1, kind=m.kind)
                        for k, ms in geo.members.items() for i, m in enumerate(ms)],
               blade_start=geo.blade_start, radius=design.blade.radius,
               layout=design.layout,
               members_source=geo.members_source, warnings=geo.warnings)
    if geo.lower is not None:
        M, shift = geo.lower_map
        gl = geo.lower
        for hh in out["hinges"]:
            hh["rotor"] = "upper"
        for mm in out["members"]:
            mm["rotor"] = "upper"
        out["hinges"] += [dict(blade=k, rotor="lower", point=M @ gl.hinge[k][0] + shift,
                               dir=M @ gl.hinge[k][1]) for k in sorted(gl.hinge)]
        out["members"] += [dict(blade=k, index=i, rotor="lower", kind=m.kind,
                                p0=tuple(M @ np.asarray(m.p0) + shift),
                                p1=tuple(M @ np.asarray(m.p1) + shift))
                           for k, ms in gl.members.items() for i, m in enumerate(ms)]
        out["lower_blade_start"] = gl.blade_start
        out["lower_radius"] = lower_design(design).blade.radius
        out["lower_map"] = dict(M=M, shift=shift)
    return _to_lists(out) if as_lists else out


def _to_lists(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, dict):
        return {k: _to_lists(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_to_lists(v) for v in o]
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return o


# =========================================================================== material colours

MATERIAL_COLORS = {            # one clearly distinct hue per material class (RGB)
    "tpu_95a": (255, 140, 26), "tpu_64d": (255, 190, 40),          # TPU: orange / amber
    "pa6_cf": (58, 62, 70), "petg_cf": (84, 98, 120),              # CF nylon / PETG-CF: charcoal / slate
    "pla": (46, 190, 170),                                         # PLA: teal
    "cf_rod_pultruded": (225, 25, 35),                             # CF rod / strip: bright red
    "steel_302_spring": (205, 212, 220),                           # steel: silver
}
_ROLE_SHADE = {"hub": -0.30, "spoke": -0.15, "fitting": 0.20, "fairing": 0.35, "member": 0.0, "member_core": -0.1,
               "member_skin": 0.1, "blade": 0.0, "skin": 0.1, "core": -0.1, "rod": 0.0}


# Opt-in photo-real palette (FLEXBLADE_REAL_COLORS=1): what the parts look like on the bench.
REAL_COLORS = {
    "pa6_cf": (48, 48, 52), "petg_cf": (58, 60, 64), "pla": (215, 215, 210),
    "tpu_95a": (235, 232, 220), "tpu_64d": (230, 226, 210),
    "cf_rod_pultruded": (28, 28, 30), "cf_tow": (28, 28, 30),
    "steel_301_fh": (196, 202, 210), "steel_302_spring": (196, 202, 210), "steel_1095_shim": (52, 62, 98),
    "fr4_g10": (196, 200, 140), "aramid_yarn": (232, 188, 52), "uhmwpe_braid": (244, 244, 238),
}


def color_hex(material: str, role: str = "") -> str:
    """Material hue with a small role-based lightness shift (hub darker, fittings lighter)."""
    import os
    pal = REAL_COLORS if os.environ.get("FLEXBLADE_REAL_COLORS") else MATERIAL_COLORS
    c = np.array(pal.get(material, MATERIAL_COLORS.get(material, (160, 110, 220))), float)
    if pal is REAL_COLORS:
        role = "" if role in ("hub", "spoke", "fitting") else role     # real parts: one colour per material
    sh = _ROLE_SHADE.get(role, 0.0)
    c = c * (1 + sh) if sh < 0 else c + (255 - c) * sh
    return "#%02x%02x%02x" % tuple(int(round(min(255, max(0, v)))) for v in c)


def _color(material: str, role: str):
    c = tuple(int(color_hex(material, role)[i:i + 2], 16) for i in (1, 3, 5))
    return (*c, 120 if role in ("skin", "member_skin") else 255)    # translucent skin shows core


# =========================================================================== export


def export(design: Design, fmt: str, path: str | os.PathLike) -> str:
    """Export geometry. fmt in {glb, stl, obj, step, json, stl_parts, 3mf}. Returns the path
    written. stl/obj: outer shells only (viewing). stl_parts / 3mf: multi-material print set
    in a directory (see export_print_parts; use that function for blades='one')."""
    fmt = fmt.lower().lstrip(".")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if fmt == "step" or fmt == "stp":
        return export_step(design, path)
    if fmt in ("stl_parts", "3mf", "3mf_parts"):
        # path = output directory (a file-like path's suffix is stripped)
        d = path.with_suffix("") if path.suffix else path
        export_print_parts(design, d, blades="all", fmt="stl" if fmt == "stl_parts" else "3mf")
        return str(d)
    surf = build_surface(design)
    if fmt == "json":
        path.write_text(json.dumps(_to_lists(surf)))
        return str(path)
    import trimesh
    if fmt == "glb":
        scene = trimesh.Scene()
        for p in surf["parts"]:
            m = trimesh.Trimesh(p["vertices"], p["faces"], process=False)
            m.visual = trimesh.visual.ColorVisuals(
                m, face_colors=np.tile(_color(p["material"], p["role"]), (len(p["faces"]), 1)))
            m.metadata["material"] = p["material"]
            scene.add_geometry(m, node_name=p["name"], geom_name=p["name"])
        scene.export(str(path), file_type="glb")
        return str(path)
    if fmt in ("stl", "obj"):
        # STL/OBJ: outer surfaces only (skins/cores/rods would double-count in a slicer);
        # use STEP for multi-material solids
        meshes = []
        for p in surf["parts"]:
            if p["role"] in ("core", "member_core", "rod"):
                continue
            meshes.append(trimesh.Trimesh(p["vertices"], p["faces"], process=False))
        m = trimesh.util.concatenate(meshes)
        m.export(str(path), file_type=fmt)
        return str(path)
    raise ValueError(f"unknown format {fmt!r}")


# =========================================================================== gmsh OCC solids


class _Gmsh:
    """Context manager for an isolated gmsh session."""

    def __init__(self, verbose=False):
        self.verbose = verbose

    def __enter__(self):
        import gmsh
        self.gmsh = gmsh
        if gmsh.isInitialized():
            gmsh.finalize()
        gmsh.initialize(interruptible=False)
        gmsh.option.setNumber("General.Terminal", 1 if self.verbose else 0)
        gmsh.option.setNumber("General.Verbosity", 2)
        gmsh.model.add("flexblade")
        return gmsh

    def __exit__(self, *exc):
        self.gmsh.finalize()


def _occ_wire(occ, sec: Section) -> int:
    if sec.kind == "circle":
        c = sec.center
        curve = occ.addCircle(c[0], c[1], c[2], sec.radius, zAxis=list(sec.normal),
                              xAxis=list(sec.xref))
        return occ.addWire([curve])
    ring = sec.ring
    ptags = [occ.addPoint(*p) for p in ring]
    curves = []
    for (i0, i1, kind) in sec.segments:
        if kind == "line":
            curves.append(occ.addLine(ptags[i0], ptags[i1]))
        else:
            ids = list(range(i0, i1 + 1)) if i1 > i0 else list(range(i0, len(ring))) + list(range(0, i1 + 1))
            curves.append(occ.addSpline([ptags[i] for i in ids]))
    return occ.addWire(curves)


def occ_part(occ, part: Part) -> list[tuple[int, int]]:
    wires = [_occ_wire(occ, s) for s in part.sections]
    return occ.addThruSections(wires, makeSolid=True, makeRuled=part.ruled)


def _step_geometry(design: Design) -> RotorGeometry:
    return build_geometry(design, n_side=N_AIRFOIL_STEP)


def _body_key(p: Part, per_part: bool = False, per_blade: bool = True) -> str:
    if per_part:
        return p.name
    cls = "rod_" if p.role == "rod" else ("core_" if p.role in ("core", "member_core") else "")
    owner = (f"{p.rotor}_" if p.rotor else "") + (f"blade{p.blade}" if p.blade else "hub")
    return f"{owner}_{cls}{p.material}" if per_blade else f"{cls}{p.material}"


def _occ_bodies(gmsh, parts: list[Part], keyfn, fuse: bool = True) -> dict[int, str]:
    """Build OCC solids of `parts` and group them by keyfn(part).

    All raw part solids are fragmented together once (disjoint, conformal pieces); each
    piece belongs to its highest-priority parent (rod > core > member/blade/skin > fitting >
    hub), so overlaps are resolved exactly once. Geometry is scaled to MILLIMETRES. (core removed from skin, rod cavities from
    the matrix). With fuse=True the pieces of each group are then fused into as few solids
    as possible (shared internal faces removed). Returns {volume tag: group key}.
    (A fuse-then-cut pipeline was abandoned: OCC cut of thin rods through spline lofts
    returned spurious extra solids.)"""
    occ = gmsh.model.occ
    vols = []
    for p in parts:
        vols.append((p, occ_part(occ, p)))
    occ.synchronize()
    flat = [d for _, dts in vols for d in dts]
    # booleans in millimetres: in metres OCC fragment of thin rods through spline lofts
    # produced duplicate pieces (sum of pieces 1 % above the union); in mm it is exact
    occ.dilate(flat, 0, 0, 0, MM, MM, MM)
    occ.synchronize()
    if len(flat) > 1:
        _, omap = occ.fragment(flat, [])
    else:
        omap = [[d] for d in flat]
    occ.synchronize()
    owner: dict[int, Part] = {}
    k = 0
    for p, dts in vols:
        for _ in dts:
            for (dim, tag) in omap[k]:
                if dim == 3 and (tag not in owner or p.priority > owner[tag].priority):
                    owner[tag] = p
            k += 1
    tag_name = {t: keyfn(p) for t, p in owner.items()}
    if not fuse:
        return tag_name
    out: dict[int, str] = {}
    for key in dict.fromkeys(tag_name.values()):
        dts = [(3, t) for t, kk in tag_name.items() if kk == key]
        if len(dts) > 1:
            # fuse copies; keep the originals unless the fused result conserves volume
            # (OCC occasionally drops or degrades a solid when fusing spline lofts)
            v_in = sum(occ.getMass(*d) for d in dts)
            try:
                cp = occ.copy(dts)
                res, _ = occ.fuse(cp[:1], cp[1:])
                occ.synchronize()
                res = [d for d in res if d[0] == 3]
                v_out = sum(occ.getMass(*d) for d in res)
                if res and abs(v_out - v_in) <= 1e-3 * v_in:
                    occ.remove(dts, recursive=True)
                    dts = res
                else:
                    occ.remove(res, recursive=True)
            except Exception:  # noqa: BLE001  (keep pieces; still disjoint)
                pass
            occ.synchronize()
        for d in dts:
            out[d[1]] = key
    occ.synchronize()
    return out


def export_step(design: Design, path: Path, fuse_by_material: bool = True) -> str:
    """STEP (millimetres) of all solids for CAD (Keel / SolidCraft insert).

    fuse_by_material: parts of equal (blade, material, rod/core class) are fused into one
    printable body; bodies of different materials are fragmented against each other so
    interfaces are conformal (rods sit in matching holes, core inside skin).
    Each solid's STEP PRODUCT name is set to e.g. ``blade1_tpu_64d``, ``blade1_rod_cf_rod_pultruded``,
    ``hub_pa6_cf`` (per-part names like ``blade1_flex0`` when fuse_by_material=False).
    """
    geo = _step_geometry(design)
    with _Gmsh() as gmsh:
        occ = gmsh.model.occ
        tag_name = _occ_bodies(gmsh, geo.parts,
                               lambda p: _body_key(p, per_part=not fuse_by_material))
        # round-trip through BREP importing solids only: drops the loft section wires,
        # which gmsh keeps as internal OCC shapes and would export as a wireframe product
        props = {t: (gmsh.model.occ.getCenterOfMass(3, t), gmsh.model.occ.getMass(3, t))
                 for (_, t) in gmsh.model.getEntities(3)}
        with tempfile.TemporaryDirectory() as td:
            bp = os.path.join(td, "solids.brep")
            gmsh.write(bp)
            gmsh.model.remove()
            gmsh.model.add("flexblade_step")
            occ.importShapes(bp, highestDimOnly=True)
            occ.synchronize()
        new_name: dict[int, str] = {}
        for (_, t) in gmsh.model.getEntities(3):
            c, m = np.array(occ.getCenterOfMass(3, t)), occ.getMass(3, t)
            best = min(props, key=lambda k: np.linalg.norm(np.array(props[k][0]) - c)
                       + abs(props[k][1] - m) / max(m, 1e-30) * 1e-3)
            new_name[t] = tag_name.get(best, f"solid{t}")
        tag_name = new_name
        vols = [t for (_, t) in gmsh.model.getEntities(3)]      # already mm (_occ_bodies)
        with _quiet_stdout():
            gmsh.write(str(path))
    # name products (written in volume-tag order)
    used: dict[str, int] = {}
    ordered = []
    for t in vols:
        nm = tag_name.get(t, f"solid{t}")
        used[nm] = used.get(nm, 0) + 1
        ordered.append(nm if used[nm] == 1 else f"{nm}_{used[nm]}")
    _rename_step_products(Path(path), ordered)
    return str(path)


def _point_inside(mesh, p) -> bool:
    """Ray-parity point-in-closed-mesh test (vectorised Möller–Trumbore, no rtree)."""
    d = np.array([0.5773, 0.5774, 0.5775])
    d /= np.linalg.norm(d)
    T = np.asarray(mesh.triangles)
    e1, e2 = T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]
    h = np.cross(d, e2)
    a = np.einsum("ij,ij->i", e1, h)
    ok = np.abs(a) > 1e-14
    f = np.zeros_like(a)
    f[ok] = 1.0 / a[ok]
    sv = np.asarray(p, float) - T[:, 0]
    u = f * np.einsum("ij,ij->i", sv, h)
    q = np.cross(sv, e1)
    v = f * (q @ d)
    t = f * np.einsum("ij,ij->i", e2, q)
    hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-12)
    return bool(hit.sum() % 2 == 1)


def _orient_shells(m):
    """Orient a multi-shell closed mesh: outer shells outward, enclosed cavity shells (e.g.
    rod holes) inward, by nesting depth. (trimesh.fix_normals alone turns every shell
    outward and would add cavity volumes.)"""
    import trimesh
    comps = m.split(only_watertight=False)
    if len(comps) <= 1:
        trimesh.repair.fix_winding(m)
        if m.volume < 0:
            m.invert()
        return m
    for c in comps:
        trimesh.repair.fix_winding(c)
        if c.volume < 0:
            c.invert()
    out = []
    for i, c in enumerate(comps):
        p = c.vertices[:1]
        depth = sum(1 for j, o in enumerate(comps)
                    if j != i and o.volume > c.volume and _point_inside(o, p[0]))
        if depth % 2 == 1:
            c.invert()
        out.append(c)
    return trimesh.util.concatenate(out)


def print_transform(design: Design) -> np.ndarray:
    """3×3 rotation H -> print frame: blade print raster_dir -> +X, build_dir -> +Z."""
    from .materials import rotation_matrix
    return rotation_matrix(design.blade.orientation).T


def export_print_parts(design: Design, out_dir: str | os.PathLike, blades: str = "all",
                       fmt: str = "stl", mesh_size: Optional[float] = None) -> dict:
    """Multi-material print export from the conformal OCC bodies (same as STEP).

    One closed, non-overlapping triangle mesh per print material (all parts of that material
    fused, higher-priority materials subtracted: core removed from skin, rod cavities removed
    from the matrix) plus ``rod_channels`` (= the insert volumes, i.e. cavities to leave for
    print-pause rods). Shared coordinate system, mm, positioned for printing: blade print
    orientation build_dir -> +Z, raster_dir -> +X, min z = 0, centred in XY.
    blades: 'all' (full rotor incl. hub) or 'one' (blade 1 + its flexure/fittings, no hub).
    fmt: 'stl' -> <out_dir>/<name>_<material>.stl per material; '3mf' -> <out_dir>/<name>.3mf
    with one named object per material. Returns a manifest (also written as manifest.json);
    occ_volume_mm3 there is informational (OCC getMass of fused thin-shell bodies was seen to
    be ~10 % off; the mesh volumes were verified against loft volumes).
    """
    import trimesh
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    from .fittings import is_clamped_strip
    if is_clamped_strip(design):
        # bought steel strips clamped in printed fittings: CSG parts (fittings.py), not OCC lofts
        return _export_strip_parts(design, out_dir, blades, fmt)
    geo = _step_geometry(design)
    parts = [p for p in geo.parts if blades == "all" or (p.blade == 1 and p.role != "spoke"
                                                          and p.rotor in ("", "upper"))]
    if blades != "all":
        parts = [p for p in parts if not p.name.endswith("hub_fitting")] + \
            [p for p in parts if p.name.endswith("hub_fitting")]
    keyfn = lambda p: "rod_channels" if p.role == "rod" else p.material  # noqa: E731
    h = mesh_size or 0.5 * auto_mesh_size(design)
    Rp = print_transform(design)
    meshes: dict[str, trimesh.Trimesh] = {}
    occ_vol: dict[str, float] = {}
    with _Gmsh() as gmsh:
        tag_name = _occ_bodies(gmsh, parts, keyfn)
        gmsh.option.setNumber("Mesh.MeshSizeMax", h * MM)
        gmsh.option.setNumber("Mesh.MeshSizeMin", h * MM * 0.05)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 24)
        gmsh.option.setNumber("General.NumThreads", min(os.cpu_count() or 1, 16))
        gmsh.model.mesh.generate(2)
        ntags, coords, _ = gmsh.model.mesh.getNodes()
        idx = np.zeros(int(ntags.max()) + 1, int)
        idx[ntags.astype(int)] = np.arange(len(ntags))
        X = coords.reshape(-1, 3)
        for key in dict.fromkeys(tag_name.values()):
            vols = [(3, t) for t, k in tag_name.items() if k == key]
            occ_vol[key] = sum(gmsh.model.occ.getMass(3, t) for _, t in vols)
            bnd = gmsh.model.getBoundary(vols, combined=True, oriented=True, recursive=False)
            tris = []
            for (d, st) in bnd:
                _, _, nodes = gmsh.model.mesh.getElements(2, abs(st))
                for nd in nodes:
                    T = idx[nd.astype(int)].reshape(-1, 3)
                    tris.append(T[:, ::-1] if st < 0 else T)
            F = np.vstack(tris)
            used = np.unique(F)
            remap = -np.ones(len(X), int)
            remap[used] = np.arange(len(used))
            m = _orient_shells(trimesh.Trimesh(X[used] @ Rp.T, remap[F], process=True))
            meshes[key] = m
    allv = np.vstack([m.vertices for m in meshes.values()])
    shift = np.array([-(allv[:, 0].min() + allv[:, 0].max()) / 2,
                      -(allv[:, 1].min() + allv[:, 1].max()) / 2, -allv[:, 2].min()])
    stem = design.name or "rotor"
    files, info = {}, {}
    for key, m in meshes.items():
        m.apply_translation(shift)
        info[key] = dict(watertight=bool(m.is_watertight), volume_mm3=float(m.volume),
                         occ_volume_mm3=occ_vol[key], n_triangles=int(len(m.faces)))
    if fmt == "stl":
        for key, m in meshes.items():
            f = out_dir / f"{stem}_{key}.stl"
            m.export(str(f))
            files[key] = str(f)
    elif fmt == "3mf":
        f = out_dir / f"{stem}.3mf"
        write_3mf(f, meshes)
        files = {"3mf": str(f)}
    else:
        raise ValueError(fmt)
    manifest = dict(design=stem, design_hash=design.hash(), units="mm", blades=blades,
                    print_frame="build_dir->+Z, raster_dir->+X, on bed (z>=0)",
                    print_rotation=Rp.tolist(), shift_mm=shift.tolist(), files=files, parts=info)
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=1))
    return manifest


def _export_strip_parts(design: Design, out_dir: Path, blades: str, fmt: str) -> dict:
    """export_print_parts for clamped-strip flexures: watertight manifold3d parts from
    fittings.strip_print_parts (hub + spokes + hub fittings; blade k + shank + root fitting).
    Frame H, mm, z up (= build direction), no print rotation; the strips/strand are bought."""
    from . import fittings as FT
    from .materials import get_material
    parts = FT.strip_print_parts(design)
    fits = parts.pop("_fits")
    keys = [k for k in parts if blades == "all" or k == "blade1"]
    stem = design.name or "rotor"
    meshes, info, files = {}, {}, {}
    for k in keys:
        m, kept, dropped = FT.to_trimesh(parts[k])
        m.apply_translation([0.0, 0.0, -m.bounds[0][2]])
        meshes[k] = m
        mat = HUB_MATERIAL if k == "hub" else design.blade.layup.material
        info[k] = dict(watertight=bool(m.is_watertight), volume_mm3=float(m.volume),
                       mass_g=float(m.volume) * get_material(mat).rho * 1e-6, material=mat,
                       components=kept, fragments_dropped=dropped, n_triangles=int(len(m.faces)))
    if fmt == "stl":
        for k, m in meshes.items():
            f = out_dir / f"{stem}_{k}.stl"
            m.export(str(f))
            files[k] = str(f)
    elif fmt == "3mf":
        f = out_dir / f"{stem}.3mf"
        write_3mf(f, meshes)
        files = {"3mf": str(f)}
    else:
        raise ValueError(fmt)
    p = fits.params
    manifest = dict(design=stem, design_hash=design.hash(), units="mm", blades=blades,
                    print_frame="rotor frame H, z up = build direction, each part on bed (z>=0)",
                    files=files, parts=info, strip=dict(free_length=p.L, land=p.land, slot_t=p.slot_t,
                                                        slot_w=p.slot_w, jaw_r=p.jaw_r),
                    notes=fits.notes)
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=1))
    return manifest


def write_3mf(path: str | os.PathLike, meshes: dict) -> str:
    """Minimal 3MF (core spec, mm): one named <object> per mesh, all in one <build>, identity
    transforms (positions are in the vertex coordinates). No lxml dependency."""
    import zipfile
    from xml.sax.saxutils import quoteattr
    parts = ['<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">\n<resources>\n']
    for oid, (name, m) in enumerate(meshes.items(), start=1):
        parts.append(f'<object id="{oid}" name={quoteattr(name)} type="model"><mesh><vertices>\n')
        parts.append("".join(f'<vertex x="{x:.5f}" y="{y:.5f}" z="{z:.5f}"/>\n'
                             for x, y, z in np.asarray(m.vertices)))
        parts.append("</vertices><triangles>\n")
        parts.append("".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>\n'
                             for a, b, c in np.asarray(m.faces)))
        parts.append("</triangles></mesh></object>\n")
    parts.append("</resources>\n<build>\n")
    parts.append("".join(f'<item objectid="{oid}"/>\n' for oid in range(1, len(meshes) + 1)))
    parts.append("</build>\n</model>\n")
    ct = ('<?xml version="1.0" encoding="UTF-8"?>\n<Types xmlns="http://schemas.openxmlformats.org/'
          'package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.'
          'openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType='
          '"application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.'
            'openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" '
            'Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
            '</Relationships>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", ct)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", "".join(parts))
    return str(path)


def _rename_step_products(path: Path, names: list[str]) -> None:
    import re
    s = path.read_text()
    pat = re.compile(r"PRODUCT\(\s*'Open CASCADE STEP translator \d+\.\d+ \d+\.(\d+)'\s*,"
                     r"\s*'Open CASCADE STEP translator \d+\.\d+ \d+\.\d+'")

    def rep(m):
        i = int(m.group(1)) - 1
        nm = names[i] if 0 <= i < len(names) else f"solid{i+1}"
        return f"PRODUCT('{nm}','{nm}'"
    s = pat.sub(rep, s)
    s = re.sub(r"PRODUCT\(\s*'Open CASCADE STEP translator \d+\.\d+ \d+'\s*,\s*"
               r"'Open CASCADE STEP translator \d+\.\d+ \d+'", "PRODUCT('flexblade_rotor','flexblade_rotor'", s)
    path.write_text(s)


class _quiet_stdout:
    """Silence C-level stdout (OpenCASCADE STEP writer prints statistics)."""

    def __enter__(self):
        import sys
        sys.stdout.flush()
        self.fd = os.dup(1)
        self.null = os.open(os.devnull, os.O_WRONLY)
        os.dup2(self.null, 1)
        return self

    def __exit__(self, *exc):
        os.dup2(self.fd, 1)
        os.close(self.fd)
        os.close(self.null)


# =========================================================================== solid mesh for FEA


@dataclass
class SolidMesh:
    points: np.ndarray           # (Np,3) m
    tets: np.ndarray             # (Ne,4) int
    elem_part: np.ndarray        # (Ne,) index into parts
    parts: list[Part]
    members: list[FlexureMember]     # blade-1 members (frame H)
    hinge: tuple[np.ndarray, np.ndarray]
    region: str
    h: float
    warnings: list[str] = field(default_factory=list)

    @property
    def materials(self) -> list[str]:
        return [p.material for p in self.parts]

    @property
    def n_elements(self) -> int:
        return len(self.tets)

    def summary(self) -> dict:
        cnt = np.bincount(self.elem_part, minlength=len(self.parts))
        return dict(region=self.region, h=self.h, n_points=len(self.points),
                    n_tets=len(self.tets),
                    parts={p.name: int(c) for p, c in zip(self.parts, cnt)})


def _region_parts(geo: RotorGeometry, region: str) -> list[Part]:
    if region == "root":
        return [p for p in geo.parts if p.blade == 1 and p.member >= 0]
    if region == "blade1":
        return [p for p in geo.parts if p.blade == 1 and p.role != "spoke"
                and not p.name.endswith("hub_fitting")]
    if region == "full":
        return list(geo.parts)
    raise ValueError(f"unknown region {region!r}")


def auto_mesh_size(design: Design) -> float:
    st = bladegeom.resolve_stations(design.blade)
    chord = float(np.mean([s.chord for s in st]))
    return max(0.06 * chord, 0.0004)


def build_solid_mesh(design: Design, region: str = "root", h: Optional[float] = None,
                     h_member: Optional[float] = None, order: int = 1,
                     verbose: bool = False, n_side: int = N_AIRFOIL_STEP,
                     rotor: str = "upper", skip_shank: bool = False) -> SolidMesh:
    """Conformal multi-material tetra mesh of a region via gmsh OCC + fragment.

    region: 'root' (flexure members of blade 1 only, ends free for BCs), 'blade1'
    (members + blade-root fitting + blade 1), 'full' (all blades, fittings, hub).
    h: blade/fitting mesh size (default auto 6 % mean chord); h_member: member size
    (default min(h, 0.7*min member thickness)). Element tags: SolidMesh.elem_part.
    Linear tets returned (fea.py uses P2 fields on them).
    """
    if design.layout == "coaxial":
        # FEA always in a single rotor's own frame (lower rotor = mirrored copy of the
        # same single-rotor analysis)
        design = lower_design(design) if rotor == "lower" else \
            design.model_copy(update=dict(layout="single"))
    geo = build_geometry(design, n_side=n_side, blades=[1] if region != "full" else None)
    parts = _region_parts(geo, region)
    if skip_shank:     # skewed-hinge designs: the bridging shank loft can defeat the mesher
        parts = [p for p in parts if not p.name.endswith("_shank")]
        geo.warnings.append("blade-root shank omitted from the solid mesh (meshing fallback)")
    if not parts:
        raise ValueError(f"region {region!r} is empty for this design")
    h = h or auto_mesh_size(design)
    tmin = min([m.thickness * min(1.0, m.taper) for m in geo.members[1]] or [h])
    h_member = h_member or min(h, 0.7 * tmin)
    with _Gmsh(verbose) as gmsh:
        occ = gmsh.model.occ
        part_vols = []
        for p in parts:
            part_vols.append(occ_part(occ, p))
        occ.synchronize()
        flat = [d for dts in part_vols for d in dts]
        occ.dilate(flat, 0, 0, 0, MM, MM, MM)          # booleans/meshing in mm
        occ.synchronize()
        if len(flat) > 1:
            out, omap = occ.fragment(flat, [])
        else:
            omap = [[d] for d in flat]
        occ.synchronize()
        # owner of each resulting volume = highest priority parent
        owner: dict[int, int] = {}
        k = 0
        for pi, dts in enumerate(part_vols):
            for _ in dts:
                for (dim, tag) in omap[k]:
                    if dim != 3:
                        continue
                    if tag not in owner or parts[pi].priority > parts[owner[tag]].priority:
                        owner[tag] = pi
                k += 1
        # mesh sizes
        gmsh.option.setNumber("Mesh.MeshSizeMax", h * MM)
        gmsh.option.setNumber("Mesh.MeshSizeMin", min(h_member, h) * 0.3 * MM)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 1)
        gmsh.option.setNumber("Mesh.Algorithm3D", 10)   # HXT, parallel
        gmsh.option.setNumber("General.NumThreads", min(os.cpu_count() or 1, 16))
        gmsh.option.setNumber("Mesh.Optimize", 1)
        sizes = {}
        for tag, pi in owner.items():
            role = parts[pi].role
            sz = h_member if role in ("member", "member_core", "member_skin") else h
            if role == "rod":
                sz = min(sz, 0.6 * parts[pi].sections[0].radius * 2)
            if role in ("skin", "member_skin"):
                # skins are thin; resolve thickness
                lay = design.blade.layup if role == "skin" else None
                ts = lay.skin_thickness if lay else min(h_member, h)
                sz = min(sz, max(ts, 0.2 * h))
            bnd = gmsh.model.getBoundary([(3, tag)], combined=False, oriented=False, recursive=True)
            for (d, t) in bnd:
                if d == 0:
                    sizes[t] = min(sizes.get(t, 1e9), sz)
        for t, sz in sizes.items():
            gmsh.model.mesh.setSize([(0, t)], sz * MM)
        for tag, pi in owner.items():
            gmsh.model.addPhysicalGroup(3, [tag], name=f"{parts[pi].name}")
        gmsh.model.mesh.generate(3)
        etypes, etags, enodes = [], [], []
        ntags, coords, _ = gmsh.model.mesh.getNodes()
        node_index = np.zeros(int(ntags.max()) + 1, int)
        node_index[ntags.astype(int)] = np.arange(len(ntags))
        pts = coords.reshape(-1, 3) / MM                # back to metres
        tets, epart = [], []
        for tag, pi in owner.items():
            types, _, nodes = gmsh.model.mesh.getElements(3, tag)
            for ty, nd in zip(types, nodes):
                if ty != 4:
                    continue
                T = node_index[nd.astype(int)].reshape(-1, 4)
                tets.append(T)
                epart.append(np.full(len(T), pi))
    tets = np.vstack(tets)
    epart = np.concatenate(epart)
    used = np.unique(tets)
    remap = -np.ones(len(pts), int)
    remap[used] = np.arange(len(used))
    pts = pts[used]
    tets = remap[tets]
    # positive orientation
    d = np.einsum("ij,ij->i", pts[tets[:, 1]] - pts[tets[:, 0]],
                  np.cross(pts[tets[:, 2]] - pts[tets[:, 0]], pts[tets[:, 3]] - pts[tets[:, 0]]))
    neg = d < 0
    tets[neg] = tets[neg][:, [0, 2, 1, 3]]
    return SolidMesh(pts, tets, epart, parts, geo.members[1], geo.hinge[1], region, h,
                     geo.warnings)


# =========================================================================== animation


def _sector_nodes(nodes: np.ndarray, psi: float, n: int) -> np.ndarray:
    ang = np.arctan2(nodes[:, 1], nodes[:, 0])
    d = np.angle(np.exp(1j * (ang - psi)))
    return np.abs(d) <= np.pi / n + 1e-9


def _interp_disp(X: np.ndarray, nodes: np.ndarray, U: np.ndarray, sel: np.ndarray,
                 mode: str, span_dir: np.ndarray, k_nn: int = 3) -> np.ndarray:
    """Rigid-section interpolation of beam-node displacement/rotation onto points X.

    U: (Nn,6) complex [u, θ]. mode 'span': linear along span coordinate between the two
    bracketing nodes of sel; 'idw': inverse-distance over k nearest selected nodes.
    Displacement of point x attached to node j: u_j + θ_j × (x − x_j).
    """
    idx = np.where(sel)[0]
    if len(idx) == 0:
        return np.zeros((len(X), 3), complex)
    Ns, Us = nodes[idx], U[idx]
    if mode == "span" and len(idx) >= 2:
        s_n = Ns @ span_dir
        order = np.argsort(s_n)
        s_n, Ns, Us = s_n[order], Ns[order], Us[order]
        s_x = X @ span_dir
        j = np.clip(np.searchsorted(s_n, s_x) - 1, 0, len(s_n) - 2)
        ds = np.maximum(s_n[j + 1] - s_n[j], 1e-12)
        f = np.clip((s_x - s_n[j]) / ds, 0.0, 1.0)[:, None]
        u = (1 - f) * Us[j, :3] + f * Us[j + 1, :3]
        th = (1 - f) * Us[j, 3:] + f * Us[j + 1, 3:]
        xr = (1 - f) * Ns[j] + f * Ns[j + 1]
        return u + np.cross(th, X - xr)
    d = np.linalg.norm(X[:, None, :] - Ns[None, :, :], axis=2)
    k = min(k_nn, len(idx))
    nn = np.argsort(d, axis=1)[:, :k]
    dn = np.take_along_axis(d, nn, 1)
    w = 1.0 / np.maximum(dn, 1e-6) ** 2
    w /= w.sum(1, keepdims=True)
    out = np.zeros((len(X), 3), complex)
    for c in range(k):
        jj = nn[:, c]
        out += w[:, c:c + 1] * (Us[jj, :3] + np.cross(Us[jj, 3:], X - Ns[jj]))
    return out


def _harm_arrays(h):
    h = h if isinstance(h, dict) else h.model_dump()
    if not h.get("nodes") or h.get("displacement_re") is None:
        raise ValueError("HarmonicResult has no nodal displacements")
    nodes = np.asarray(h["nodes"], float)
    U = np.asarray(h["displacement_re"], float) + 1j * np.asarray(h["displacement_im"], float)
    if U.shape[1] < 6:
        U = np.hstack([U, np.zeros((len(U), 6 - U.shape[1]))])
    return nodes, U


def _part_displacement(p: dict, X: np.ndarray, nodes, U, n: int, r_start: float,
                       hub_r: float) -> np.ndarray:
    """Displacement of points X (single-rotor frame) of surface part p."""
    rn = np.linalg.norm(nodes[:, :2], axis=1)
    k = p["blade"]
    if k == 0:
        sel = rn <= hub_r * 1.05
        if not sel.any():
            sel = rn <= rn.min() + 1e-9
        return _interp_disp(X, nodes, U, sel, "idw", np.array([1.0, 0, 0]), k_nn=1)
    psi = 2 * np.pi * (k - 1) / n
    span = np.array([np.cos(psi), np.sin(psi), 0.0])
    sect = _sector_nodes(nodes, psi, n)
    if p["role"] in ("blade", "skin", "core", "rod"):
        sel = sect & (rn >= r_start - 1e-4)
        if sel.sum() < 2:
            sel = sect
        return _interp_disp(X, nodes, U, sel, "span", span)
    sel = sect & (rn <= r_start + 0.002)
    if not sel.any():
        sel = sect
    return _interp_disp(X, nodes, U, sel, "idw", span)


def _split_harmonic(harmonic):
    """-> (upper_harmonic, lower_harmonic|None) from HarmonicResult/Results/dict."""
    h = harmonic.model_dump() if hasattr(harmonic, "model_dump") else dict(harmonic)
    if h.get("coaxial"):
        cx = h["coaxial"]
        return cx["upper"]["harmonic"], cx["lower"]["harmonic"]
    if "harmonic" in h and "nodes" not in h:
        return h["harmonic"], None
    return h, None


def vertex_displacements(design: Design, harmonic, surface: Optional[dict] = None) -> dict:
    """Complex per-vertex displacement amplitude for each surface part (frame H).

    harmonic: HarmonicResult, Results (uses .harmonic, or .coaxial.upper/.lower harmonic),
    or equivalent dict; nodes + displacement_re/_im (Nn,6) [u, θ] of the beam model.
    Rigid-section interpolation: blade vertices interpolate the two span-bracketing beam
    nodes (u + θ×r), flexure/fitting vertices inverse-distance over nearby nodes, hub
    follows the hub node. Coaxial lower rotor: its harmonic is taken in the lower rotor's own
    frame (single-rotor code output) and mapped by coaxial_map (mirror y, shift z).
    """
    if surface is None:
        surface = build_surface(design)
    hu, hl = _split_harmonic(harmonic)
    nodes_u, U_u = _harm_arrays(hu)
    lower = None
    if surface.get("layout") == "coaxial":
        ld = lower_design(design)
        M, shift = coaxial_map(design)
        lower = (ld, M, shift, _harm_arrays(hl) if hl is not None else None)
    out = []
    for p in surface["parts"]:
        X = np.asarray(p["vertices"], float)
        if p.get("rotor") == "lower":
            ld, M, shift, hh = lower
            if hh is None:
                D = np.zeros((len(X), 3), complex)
            else:
                Xo = (X - shift) @ M          # M is its own inverse (and symmetric)
                Do = _part_displacement(p, Xo, hh[0], hh[1], ld.blade.n_blades,
                                        surface["lower_blade_start"], ld.flexure.hub_radius)
                D = Do @ M.T
        else:
            D = _part_displacement(p, X, nodes_u, U_u, design.blade.n_blades,
                                   surface["blade_start"], design.flexure.hub_radius)
        out.append(dict(name=p["name"], rotor=p.get("rotor", ""), disp=D))
    return dict(parts=out)


def deformed_frames(design: Design, harmonic, n_frames: int = 24,
                    scale: float | str = 1.0, surface: Optional[dict] = None,
                    as_lists: bool = False, spin: bool = False) -> dict:
    """Per-vertex displacement frames for animating the 1/rev harmonic response.

    harmonic: HarmonicResult or Results (coaxial Results animate both rotors).
    frame j, phase φ_j = 2πj/n_frames (= Ωt): d = scale·Re{D e^{iφ_j}} (rotating frame).
    spin=True adds the rigid rotor rotation: upper rotor +φ_j about +z, coaxial lower −φ_j
    (counter-rotating); frames then hold (R(±φ)(X + d) − X).
    scale='auto' scales the max elastic amplitude to 5 % of rotor radius.
    Returns dict(phase, scale, max_amplitude, parts=[{name, rotor, spin_sign, amp_re,
    amp_im, frames (n_frames,V,3)}]). A GUI can use amp_re/amp_im directly:
    d(φ) = Re{(re + i·im) e^{iφ}}, and rotate each part by spin_sign·φ about +z.
    """
    surface = surface or build_surface(design)
    vd = vertex_displacements(design, harmonic, surface)
    amax = max([float(np.abs(p["disp"]).max()) for p in vd["parts"]] + [1e-30])
    if scale == "auto":
        scale = 0.05 * design.blade.radius / amax
    scale = float(scale)
    ph = 2 * np.pi * np.arange(n_frames) / n_frames
    parts = []
    for p, sp_ in zip(vd["parts"], surface["parts"]):
        D = p["disp"] * scale
        frames = np.real(D[None, :, :] * np.exp(1j * ph)[:, None, None])
        sgn = -1.0 if p["rotor"] == "lower" else 1.0
        if spin:
            X = np.asarray(sp_["vertices"], float)
            for j, a in enumerate(ph):
                R = _rotz(sgn * a)
                frames[j] = (X + frames[j]) @ R.T - X
        parts.append(dict(name=p["name"], rotor=p["rotor"], spin_sign=sgn, amp_re=D.real,
                          amp_im=D.imag, frames=frames))
    out = dict(phase=ph, scale=scale, max_amplitude=amax, parts=parts)
    return _to_lists(out) if as_lists else out
