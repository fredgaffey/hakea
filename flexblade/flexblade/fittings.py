"""Buildable clamp fittings for clamped spring-steel strip flexures ("steelx" designs).

geometry.py models the hub and blade-root fittings of a strip flexure as lofted envelope boxes
(fine for mass/FEA envelopes, not printable: they span the whole strip, so there is no free span
or clamp land). This module builds the real parts by manifold3d CSG from the flexure
parameters, in frame H of blade 1, millimetres:

  * per owned strip end a jaw block: clamp land along the strip (length max(4 sqrt(w_mm), 5) mm)
    with a slot (t + 0.05) x (w + 0.2), a radiused jaw exit (R = strip_clamp_radius, default
    10 t) on both faces, walls >= 1.2 mm. Where the crossed strips are closer than two walls
    (crossing gap) the slot is open on that edge (U jaw) instead of leaving a sub-wall sliver.
    The jaw exit faces sit exactly on the member end points p0 / p1, so the free span is the
    modelled strip length. Hub slots run through the back (strip inserted from behind through
    the hub jaw); blade slots are blind (strip end stops at the land end).
  * a web: hull of the jaws and the geometry.py fitting envelope, trimmed to the far side of the
    strip exit plane (so it never enters the free span), plus a strand post along the strand
    trimmed below/above both strip planes.
  * strand channel (strand d + 0.3) running CH_LEN from each strand end into the fitting, ending
    in a vertical knot well open to +z (anchor).
A free-span keep-out (strip +-0.5 mm) is subtracted from everything.

`fitting_clearance` sweeps the blade fitting through lag (about the hinge axis) x pitch (about
the radial x axis through the hinge point) and returns the minimum gap to the hub fitting.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from .schema import Design

WALL = 1.2              # mm, minimum printed wall
SLOT_ALLOW = 0.05       # mm, slot thickness over the shim (glue / press)
SIDE = 0.1              # mm, slot width clearance per side
OPEN_LIP = 0.5          # mm, strip edge overhang past an open-edged (U) jaw
STRIP_CLEAR = 0.5       # mm, posts / web clearance to the strip planes and free span
CH_CLEAR = 0.3          # mm, strand channel diameter over the strand
WEB_T = 3.0             # mm, web slab thickness (connects the jaws)
CH_LEN = 8.0            # mm, strand channel length from the strand end to the knot well
SEG = 48


def _mf():
    import manifold3d
    return manifold3d


@dataclass
class StripParams:
    t: float            # strip thickness, mm
    w: float            # strip width, mm
    L: float            # modelled free length, mm
    land: float         # clamp land length, mm
    slot_t: float
    slot_w: float
    jaw_r: float
    wall: float
    strand_d: float
    channel_d: float
    well_d: float


def strip_members(design: Design):
    from .geometry import get_members
    ms, hinge, _ = get_members(design)
    strips = [m for m in ms if m.kind == "blade" and m.shape == "rect"]
    wires = [m for m in ms if m.kind == "wire"]
    return strips, wires, hinge


def is_clamped_strip(design: Design) -> bool:
    """Flexure made of bought sheet strips (spring-steel shim) clamped in printed fittings."""
    if design.layout != "single":
        return False
    try:
        strips, _, _ = strip_members(design)
    except Exception:
        return False
    return bool(strips) and all(m.layup.material.lower().startswith("steel") for m in strips)


def strip_params(design: Design) -> StripParams:
    f = design.flexure
    strips, _, _ = strip_members(design)
    m = strips[0]
    t, w = m.thickness * 1e3, m.width * 1e3
    L = float(np.linalg.norm(np.subtract(m.p1, m.p0))) * 1e3
    R = f.strip_clamp_radius * 1e3 if f.strip_clamp_radius > 0 else 10.0 * t
    d = f.strap_strand_diameter * 1e3
    return StripParams(t=t, w=w, L=L, land=max(4.0 * math.sqrt(w), 5.0), slot_t=t + SLOT_ALLOW,
                       slot_w=w + 2 * SIDE, jaw_r=R, wall=WALL, strand_d=d,
                       channel_d=d + CH_CLEAR, well_d=max(2.5 * d, 3.5))


# --------------------------------------------------------------------------- primitives
def _box(lo, hi, R=None, c=None):
    mf = _mf()
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    b = mf.Manifold.cube(tuple(hi - lo)).translate(tuple(lo))
    if R is not None:
        b = b.transform(np.c_[np.asarray(R, float), np.asarray(c, float)])
    return b


def _basis(d):
    d = np.asarray(d, float) / np.linalg.norm(d)
    h = np.array([0, 0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0, 0])
    e1 = np.cross(h, d); e1 /= np.linalg.norm(e1)
    return np.c_[e1, np.cross(d, e1), d]          # right-handed, z -> d


def _cyl(p0, d, r, length):
    mf = _mf()
    c = mf.Manifold.cylinder(length, r, r, SEG)
    return c.transform(np.c_[_basis(d), np.asarray(p0, float)])


def _union(ms):
    mf = _mf()
    ms = [m for m in ms if m is not None and not m.is_empty()]
    return mf.Manifold.batch_boolean(ms, mf.OpType.Add) if len(ms) > 1 else ms[0]


def _sub(a, cuts):
    mf = _mf()
    return mf.Manifold.batch_boolean([a] + list(cuts), mf.OpType.Subtract) if cuts else a


def _frame(m):
    p0, p1 = np.asarray(m.p0, float) * 1e3, np.asarray(m.p1, float) * 1e3
    L = float(np.linalg.norm(p1 - p0))
    t = (p1 - p0) / L
    w = np.asarray(m.width_dir, float); w = w - t * (w @ t); w /= np.linalg.norm(w)
    return p0, p1, L, t, w, np.cross(t, w)


def _hull_of_part(design: Design, name: str):
    from .geometry import build_geometry, loft_surface
    geo = _geo(design)
    p = next(p for p in geo.parts if p.name == name)
    V, _ = loft_surface(p)
    return _mf().Manifold.hull_points(np.asarray(V, float) * 1e3)


@lru_cache(maxsize=8)
def _geo_cached(key, design_json):
    from .geometry import build_geometry
    return build_geometry(Design.model_validate_json(design_json), blades=[1])


def _geo(design: Design):
    return _geo_cached(design.hash(), design.model_dump_json())


# --------------------------------------------------------------------------- parts
@dataclass
class StripFittings:
    params: StripParams
    hub: object             # manifold3d.Manifold, blade-1 hub fitting (jaws + web + post), mm
    blade: object           # blade-1 root fitting (jaws + web + post), mm
    hub_cuts: list
    blade_cuts: list
    keepout: object         # free-span keep-out of the strips
    hinge_point: np.ndarray  # mm
    hinge_axis: np.ndarray
    notes: list
    hub_plane: tuple = ()     # (normal, offset): hub web is on normal.x >= offset
    blade_plane: tuple = ()


def _side(design, strips, wires, P, s, end, prm: StripParams):
    """Jaws, web, post and cutters of the fitting owning strip end `end` (0 hub, 1 blade)."""
    mf = _mf()
    H = prm.slot_t / 2
    R = prm.jaw_r
    hn = H + R + prm.wall                                 # jaw half-thickness
    fr = [_frame(m) for m in strips]
    centres = [0.5 * (f[0] + f[1]) for f in fr]
    # side direction (along s) of this fitting and its strip-exit level
    lv = np.array([s @ ((f[0] if end == 0 else f[1]) - P) for f in fr])
    sg = 1.0 if lv.mean() > 0 else -1.0
    e = sg * s
    h_exit = float(np.abs(lv).max())
    jaws, cuts, notes = [], [], []
    for i, (p0, p1, L, t, w, n) in enumerate(fr):
        o = p0 if end == 0 else p1
        d = -t if end == 0 else t                       # exit -> into the land
        nn = np.cross(d, w)
        Rm = np.c_[d, w, nn]
        lo_w, hi_w = -(prm.w / 2 + SIDE + prm.wall), prm.w / 2 + SIDE + prm.wall
        cl_w, ch_w = -prm.slot_w / 2, prm.slot_w / 2
        others = [c for j, c in enumerate(centres) if j != i]
        if others:
            dc = min(others, key=lambda c: abs(w @ (c - centres[i])))
            off = float(w @ (dc - centres[i]))
            edge_gap = abs(off) - prm.w
            if edge_gap < 2 * (SIDE + prm.wall) + STRIP_CLEAR:
                if off > 0:
                    hi_w, ch_w = prm.w / 2 - OPEN_LIP, prm.slot_w / 2 + 5.0
                else:
                    lo_w, cl_w = -(prm.w / 2 - OPEN_LIP), -(prm.slot_w / 2 + 5.0)
                if i == 0:
                    notes.append(f"crossing edge gap {edge_gap:.2f} mm < 2 walls: U jaws, slot open "
                                 f"on the crossing edge (strip overhangs the jaw by {OPEN_LIP} mm)")
        jaws.append(_box([0, lo_w, -hn], [prm.land, hi_w, hn], Rm, o))
        depth = prm.land + (80.0 if end == 0 else 0.3)
        cuts.append(_box([-0.5, cl_w, -H], [depth, ch_w, H], Rm, o))
        fl = _box([-0.5, cl_w, -(H + R)], [R, ch_w, H + R], Rm, o)
        cyls = [_cyl(o + d * R + nn * k * (H + R) + w * (cl_w - 1.0), w, R, ch_w - cl_w + 2.0)
                for k in (1, -1)]
        cuts.append(_sub(fl, cyls))
    env = _hull_of_part(design, "blade1_hub_fitting" if end == 0 else "blade1_root_fitting")
    zc = h_exit + min(2.0, 0.6 * hn)
    # web: slab WEB_T thick of the hull of jaws + envelope, beyond the strip exit plane
    web = mf.Manifold.batch_hull([*jaws, env]).trim_by_plane(tuple(e), float(e @ P + zc)) \
        .trim_by_plane(tuple(-e), float(-(e @ P) - zc - WEB_T))
    # strand post + channel + knot well
    planes = []
    for (p0, p1, L, t, w, n), c in zip(fr, centres):
        k = 1.0 if n @ e > 0 else -1.0
        planes.append((k * n, float(k * n @ c + prm.t / 2 + STRIP_CLEAR)))
    posts = []
    for m in wires:
        a0, a1 = np.asarray(m.p0, float) * 1e3, np.asarray(m.p1, float) * 1e3
        o = a0 if end == 0 else a1
        away = (a0 - a1) if end == 0 else (a1 - a0)
        away /= np.linalg.norm(away)
        rp = prm.channel_d / 2 + prm.wall
        plen = (float(o @ away) if end == 0 else CH_LEN + prm.well_d + prm.wall + 4.0)
        post = _cyl(o, away, rp, max(plen, CH_LEN + prm.well_d))
        for nrm, off in planes:
            post = post.trim_by_plane(tuple(nrm), off)
        posts.append(post)
        cuts.append(_cyl(o - away * 0.5, away, prm.channel_d / 2, CH_LEN + 0.5 + prm.well_d / 2))
        wc = o + away * CH_LEN
        cuts.append(_cyl(wc - np.array([0, 0, prm.channel_d / 2]), [0, 0, 1.0], prm.well_d / 2, 60.0))
    part = _union(jaws + [web] + posts)
    return part, cuts, notes, (e, float(e @ P + zc))


def _keepout(strips, prm: StripParams):
    ks = []
    for m in strips:
        p0, p1, L, t, w, n = _frame(m)
        ks.append(_box([0.05, -(prm.w / 2 + SIDE + STRIP_CLEAR), -(prm.t / 2 + STRIP_CLEAR)],
                       [L - 0.05, prm.w / 2 + SIDE + STRIP_CLEAR, prm.t / 2 + STRIP_CLEAR],
                       np.c_[t, w, n], p0))
    return _union(ks)


def build_strip_fittings(design: Design) -> StripFittings:
    """Blade-1 hub and root fittings (frame H, mm) as manifold3d solids, with their cutters."""
    from .geometry import _hinge_frame
    strips, wires, hinge = strip_members(design)
    if not strips:
        raise ValueError("design has no strip flexure members")
    prm = strip_params(design)
    P, s, t_, a = _hinge_frame(*hinge)
    P = P * 1e3
    hub, hc, notes, hpl = _side(design, strips, wires, P, s, 0, prm)
    bl, bc, _, bpl = _side(design, strips, wires, P, s, 1, prm)
    ko = _keepout(strips, prm)
    hub = _sub(hub, hc + [ko])
    bl = _sub(bl, bc + [ko])
    return StripFittings(prm, hub, bl, hc, bc, ko, P, np.asarray(a, float), notes, hpl, bpl)


# --------------------------------------------------------------------------- motion sweep
def _rot(axis, ang):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def motion_range(design: Design, summary: dict | None = None) -> tuple[float, float]:
    """(lag, pitch) half-ranges in degrees: static + 1P amplitude + 1 deg margin if a rotor
    summary is given (keys lag_static_deg, lag_1p_deg, pitch_static_deg, pitch_1p_deg), else
    +-5 lag, +-8 pitch."""
    if summary:
        lag = abs(summary.get("lag_static_deg", 0.0)) + abs(summary.get("lag_1p_deg", 0.0)) + 1.0
        pit = abs(summary.get("pitch_static_deg", 0.0)) + abs(summary.get("pitch_1p_deg", 0.0)) + 1.0
        return max(lag, 1.0), max(pit, 1.0)
    return 5.0, 8.0


def fitting_clearance(design: Design, lag_deg: float = 5.0, pitch_deg: float = 8.0,
                      n: int = 5, fits: StripFittings | None = None) -> dict:
    """Minimum gap (mm) between the blade-1 root fitting and the hub fitting over the grid
    lag in [-lag, lag] (about the hinge axis) x pitch in [-pitch, pitch] (about x through the
    hinge point), both rigid rotations of the blade fitting."""
    fits = fits or build_strip_fittings(design)
    P = fits.hinge_point
    best = (np.inf, 0.0, 0.0)
    for lg in np.linspace(-lag_deg, lag_deg, n):
        for pt in np.linspace(-pitch_deg, pitch_deg, n):
            Rm = _rot([1.0, 0, 0], math.radians(pt)) @ _rot(fits.hinge_axis, math.radians(lg))
            moved = fits.blade.transform(np.c_[Rm, P - Rm @ P])
            if not (moved ^ fits.hub).is_empty():
                g = -float((moved ^ fits.hub).volume())
            else:
                g = float(moved.min_gap(fits.hub, 10.0))
            if g < best[0]:
                best = (g, float(lg), float(pt))
    return dict(min_gap_mm=best[0], at_lag_deg=best[1], at_pitch_deg=best[2],
                lag_range_deg=lag_deg, pitch_range_deg=pitch_deg,
                note="negative = interference volume mm^3")


@lru_cache(maxsize=16)
def _clearance_cached(key, design_json, lag, pitch):
    return fitting_clearance(Design.model_validate_json(design_json), lag, pitch)


def clearance_check(design: Design, summary: dict | None = None):
    """Soft Check 'fitting_clearance' (cached per design hash); None if not a strip design."""
    from .schema import Check
    if not is_clamped_strip(design):
        return None
    lag, pit = motion_range(design, summary)
    r = _clearance_cached(design.hash(), design.model_dump_json(), lag, pit)
    g = r["min_gap_mm"]
    need = 0.3
    return Check(name="fitting_clearance", member="fittings", value=g, allowable=need,
                 utilisation=float(need / g) if g > 0 else 99.0,
                 note=f"soft: min hub/root fitting gap mm over lag +-{lag:.1f}, pitch +-{pit:.1f} deg "
                      f"(rigid sweep, worst at lag {r['at_lag_deg']:.1f} pitch {r['at_pitch_deg']:.1f})")


# --------------------------------------------------------------------------- export
def to_trimesh(m, clean_min_mm3: float = 1.0):
    """manifold -> trimesh; drops floating fragments smaller than clean_min_mm3. Returns
    (mesh, n_components_kept, n_dropped)."""
    import trimesh
    mf = _mf()
    comps = m.decompose()
    keep = [c for c in comps if c.volume() >= clean_min_mm3]
    mm = keep[0] if len(keep) == 1 else mf.Manifold.batch_boolean(keep, mf.OpType.Add)
    mo = mm.to_mesh()
    tm = trimesh.Trimesh(np.asarray(mo.vert_properties)[:, :3].astype(float),
                         np.asarray(mo.tri_verts), process=True)
    return tm, len(keep), len(comps) - len(keep)


def loft_manifold(design: Design, name: str, blade: int = 1, hull: bool = False):
    """A geometry.py lofted part as a manifold (mm); hull=True for box-like parts."""
    from .geometry import build_geometry, loft_surface
    import trimesh
    geo = _geo(design) if blade == 1 else build_geometry(design, blades=[blade])
    p = next(p for p in geo.parts if p.name == name)
    V, F = loft_surface(p)
    V = np.asarray(V, float) * 1e3
    if hull:
        return _mf().Manifold.hull_points(V)
    tm = trimesh.Trimesh(V, F, process=True)
    tm.fix_normals()
    mf = _mf()
    return mf.Manifold(mf.Mesh(np.asarray(tm.vertices, np.float32), np.asarray(tm.faces, np.uint32)))


def _mirror_rot(m, k: int, n: int):
    """blade-1 fitting -> blade k (mirror z for even blades, then rotate about z)."""
    if k % 2 == 0:
        m = m.mirror((0.0, 0.0, 1.0))
    return m.rotate((0.0, 0.0, 360.0 * (k - 1) / n))


def strip_print_parts(design: Design) -> dict:
    """Printable solids (manifolds, mm, frame H): 'hub' (hub + spokes + all hub fittings) and
    'blade<k>' (blade k airfoil + shank + root fitting), with all cutters applied."""
    fits = build_strip_fittings(design)
    n = design.blade.n_blades
    sp = loft_manifold(design, "blade1_spoke", hull=True)
    sp = _mf().Manifold.batch_hull([sp, sp.translate((-1.0, 0.0, 0.0))])  # overlap, not touch, at the centre
    hub_core = _union([loft_manifold(design, "hub", hull=True), sp])
    hub_core = _sub(hub_core, fits.hub_cuts + [fits.keepout])
    hub1 = _union([fits.hub, hub_core])
    hub = _union([_mirror_rot(hub1, k, n) for k in range(1, n + 1)])
    out = {"hub": hub}
    mf = _mf()
    env = loft_manifold(design, "blade1_root_fitting", hull=True)
    shank0 = loft_manifold(design, "blade1_shank", hull=True)
    e, off = fits.blade_plane
    body = loft_manifold(design, "blade1")      # airfoils are not mirrored between blades
    for k in range(1, n + 1):
        fb = _sub(fits.blade, fits.blade_cuts)
        cuts = list(fits.blade_cuts) + [fits.keepout]
        env_k = env
        if k % 2 == 0:      # blade k flexure/fitting is mirrored in z (geometry.members_for_blade)
            fb = fb.mirror((0.0, 0.0, 1.0))
            cuts = [c.mirror((0.0, 0.0, 1.0)) for c in cuts]
            env_k = env.mirror((0.0, 0.0, 1.0))
            ek = np.array([e[0], e[1], -e[2]])
        else:
            ek = np.asarray(e)
        # shank: hull of the fitting envelope and the shank loft, kept beyond the web plane
        shank = mf.Manifold.batch_hull([env_k, shank0]).trim_by_plane(tuple(ek), off)
        bk = _sub(_union([fb, shank, body]), cuts)
        out[f"blade{k}"] = bk.rotate((0.0, 0.0, 360.0 * (k - 1) / n))
    out["_fits"] = fits
    return out
