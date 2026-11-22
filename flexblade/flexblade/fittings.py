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

import dataclasses
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
WEB_T_SLIM = 2.0        # mm, slim web slab thickness (> WALL, two perimeters + skin)
STOP_CLEAR = 0.25       # mm, slot thickness clearance over the stack (strip_stop)
OVERDEEP = 3.0          # mm, blade slot depth beyond the land in the breakaway-jig build (no stops)
SPRUE = 1.2             # mm, breakaway bridge square section
SPRUE_NECK = 0.8        # mm, notched (necked) section at both joints
SPRUE_NECK_LEN = 0.8    # mm, neck length at each joint
SPRUE_CLEAR = 0.6       # mm, bridge clearance to strips, strand and channels
STOP_BIAS = 0.3         # mm, stop-to-stop distance over the nominal cut length (free length biased long)
JAW_FACE_SLIM = 1.5     # mm, slim jaw face wall over the slot beyond the exit radius (>= WALL)


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
    stop: bool = False      # printed stop: blind slots, stop face at stop_depth from each exit
    stop_depth: float = 0.0
    slim: bool = False      # stepped jaws, thin web, per-jaw web hulls (fittings ~40 % lighter)
    blade_extra: float = 0.3  # mm, blade slot depth beyond the land (breakaway jig: OVERDEEP)


def strip_members(design: Design):
    from .geometry import get_members
    ms, hinge, _ = get_members(design)
    strips = [m for m in ms if m.kind == "blade" and m.shape == "rect"]
    wires = [m for m in ms if m.kind == "wire"]
    return strips, wires, hinge


def stop_free_length(design: Design) -> float:
    """Free length (mm) of a strip cut to L + 2 land when it bottoms on both printed stops, from
    the actual slot cutters: stop face = far end of the slot cutter along the slot axis."""
    fits = build_strip_fittings(design)
    prm = fits.params
    strips, _, _ = strip_members(design)
    out = []
    for i, m in enumerate(strips):
        p0, p1, _, _, _, _ = _frame(m)
        pts = {}
        for (j, end, d, dw) in fits.slot_axes:
            if j != i:
                continue
            cuts = fits.hub_cuts if end == 0 else fits.blade_cuts
            k = [a for a in fits.slot_axes if a[1] == end].index((j, end, d, dw))
            cut = cuts[2 * k]                       # slot box (the next entry is the exit flare)
            v = np.asarray(cut.to_mesh().vert_properties)[:, :3]
            o = p0 if end == 0 else p1
            pts[end] = o + d * float(((v - o) @ d).max())
        out.append(float(np.linalg.norm(pts[1] - pts[0])) - (prm.L + 2 * prm.land))
    return prm.L + float(np.mean(out))


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
    t_lam = m.thickness * 1e3
    t, w = t_lam * max(1, int(getattr(f, "strip_laminae", 1))), m.width * 1e3   # stack thickness
    L = float(np.linalg.norm(np.subtract(m.p1, m.p0))) * 1e3
    R = f.strip_clamp_radius * 1e3 if f.strip_clamp_radius > 0 else 10.0 * t_lam
    d = f.strap_strand_diameter * 1e3
    stop = bool(getattr(f, "strip_stop", False))
    sc = float(getattr(f, "strip_slot_clear", -1.0))
    land = max(4.0 * math.sqrt(w), 5.0)
    return StripParams(t=t, w=w, L=L, land=land, slot_t=t + (sc if sc >= 0 else (STOP_CLEAR if stop else SLOT_ALLOW)),
                       stop=stop, stop_depth=land + STOP_BIAS / 2,
                       slot_w=w + 2 * SIDE, jaw_r=R, wall=WALL, strand_d=d,
                       channel_d=d + CH_CLEAR, well_d=max(2.5 * d, 3.5), slim=bool(f.fitting_slim))


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
    slot_axes: list = ()     # (strip, end, unit axis exit -> land, dowel cutter) per slot
    hub_plane: tuple = ()     # (normal, offset): hub web is on normal.x >= offset
    blade_plane: tuple = ()


DOWEL_INTRUDE = 0.15    # mm, dowel intrusion into the slot (opens a ~1.1 mm contact slit; 0.05-0.10 gave non-manifold CSG on some parts)
DOWEL_D = 3.0           # mm, datum dowel hole diameter (hinge_tolerance.md)


def slot_rotations(design: Design, blade: int = 1):
    """Per strip, [hub-end Q, blade-end Q] 3x3 maps (blade-1 frame) rotating the nominal slot frame
    to the pre-angled one: Q = E0 expm(-p) E0^T in the blade-`blade` member frame (as
    beam.slot_frames, p = flexure.strip_preset_deg[blade-1][member] radians), mapped back by the
    z-mirror for even blades. None where no preset."""
    from .geometry import get_members
    from .beam import _rotv
    ms, _, _ = get_members(design)
    sp = design.flexure.strip_preset_deg
    T = np.eye(3) if blade % 2 else np.diag([1.0, 1.0, -1.0])
    out = []
    for j, m in enumerate(ms):
        if not (m.kind == "blade" and m.shape == "rect"):
            continue
        row = sp[blade - 1][j] if len(sp) >= blade and len(sp[blade - 1]) > j else []
        if len(row) < 6 or not any(row):
            out.append([None, None])
            continue
        p0, p1 = T @ np.asarray(m.p0, float), T @ np.asarray(m.p1, float)
        e1 = (p1 - p0) / np.linalg.norm(p1 - p0)
        e2 = T @ np.asarray(m.width_dir, float)
        e2 = e2 - e1 * (e1 @ e2); e2 /= np.linalg.norm(e2)
        E0 = np.column_stack([e1, e2, np.cross(e1, e2)])
        pv = np.radians(np.asarray(row, float))
        out.append([T.T @ (E0 @ _rotv(-pv[k:k + 3]) @ E0.T) @ T for k in (0, 3)])
    return out


def _side(design, strips, wires, P, s, end, prm: StripParams, Qs=None, axes=None):
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
    jaws, cuts, notes, dowels = [], [], [], []
    for i, (p0, p1, L, t, w, n) in enumerate(fr):
        o = p0 if end == 0 else p1
        d = -t if end == 0 else t                       # exit -> into the land
        nn = np.cross(d, w)
        Rm = np.c_[d, w, nn]
        Q = Qs[i][end] if Qs else None
        if axes is not None:
            axes.append((i, end, None))
        if Q is not None:                               # pre-angled slot, rotated about its exit point
            Rm = Q @ Rm
            d, w, nn = Rm[:, 0], Rm[:, 1], Rm[:, 2]
        if axes is not None:
            axes[-1] = (i, end, d.copy())               # slot axis, exit -> into the land
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
        if prm.slim and prm.land > R + prm.wall + 1.0:
            # stepped jaw: full radiused-exit thickness over the first R + wall, then faces of
            # JAW_FACE_SLIM over the rest of the (unchanged) clamp land
            x1 = R + prm.wall
            hs = H + JAW_FACE_SLIM
            jaws.append(_union([_box([0, lo_w, -hn], [x1, hi_w, hn], Rm, o),
                                _box([x1 - 0.01, lo_w, -hs], [prm.land, hi_w, hs], Rm, o)]))
        else:
            jaws.append(_box([0, lo_w, -hn], [prm.land, hi_w, hn], Rm, o))
        depth = prm.stop_depth if prm.stop else prm.land + (80.0 if end == 0 else prm.blade_extra)
        cuts.append(_box([-0.5, cl_w, -H], [depth, ch_w, H], Rm, o))
        fl = _box([-0.5, cl_w, -(H + R)], [R, ch_w, H + R], Rm, o)
        cyls = [_cyl(o + d * R + nn * k * (H + R) + w * (cl_w - 1.0), w, R, ch_w - cl_w + 2.0)
                for k in (1, -1)]
        cuts.append(_sub(fl, cyls))
        # datum dowel hole: tangent to the slot face and the jaw end face (DOWEL_INTRUDE into the
        # slot so the dowel touches the strip), on the side facing the web
        kd = 1.0 if nn @ e > 0 else -1.0
        rd = DOWEL_D / 2 - DOWEL_INTRUDE
        dowels.append(_cyl(o + d * rd + nn * kd * (H + rd) + w * (cl_w - 1.0), w, DOWEL_D / 2, ch_w - cl_w + 2.0))
    env = _hull_of_part(design, "blade1_hub_fitting" if end == 0 else "blade1_root_fitting")
    zc = h_exit + min(2.0, 0.6 * hn)
    web_t = WEB_T_SLIM if prm.slim else WEB_T
    # web: slab web_t thick of the hull of jaws + envelope, beyond the strip exit plane
    # (slim: union of per-jaw hulls with the envelope, no filler between the jaws)
    hulls = [mf.Manifold.batch_hull([j, env]) for j in jaws] if prm.slim else \
        [mf.Manifold.batch_hull([*jaws, env])]
    web = _union([h.trim_by_plane(tuple(e), float(e @ P + zc))
                  .trim_by_plane(tuple(-e), float(-(e @ P) - zc - web_t)) for h in hulls])
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
    cuts.extend(dowels)
    if axes is not None:
        axes[-len(dowels):] = [a + (dw,) for a, dw in zip(axes[-len(dowels):], dowels)]
    return part, cuts, notes, (e, float(e @ P + zc))


# --------------------------------------------------------------------------- mass properties
def mass_properties(m) -> tuple[float, np.ndarray, np.ndarray]:
    """Closed manifold (mm) -> (volume mm^3, centroid mm, second-moment tensor about the
    centroid int (x-c)(x-c)^T dV in mm^5). Signed tetrahedra from the origin."""
    mo = m.to_mesh()
    V = np.asarray(mo.vert_properties, float)[:, :3]
    F = np.asarray(mo.tri_verts, np.int64)
    if not len(F):
        return 0.0, np.zeros(3), np.zeros((3, 3))
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    det = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = det.sum() / 6.0
    com = (det[:, None] * (a + b + c)).sum(0) / 24.0 / vol
    s = a + b + c
    C = (np.einsum("t,ti,tj->ij", det, a, a) + np.einsum("t,ti,tj->ij", det, b, b)
         + np.einsum("t,ti,tj->ij", det, c, c) + np.einsum("t,ti,tj->ij", det, s, s)) / 120.0
    return float(vol), com, C - vol * np.outer(com, com)


def _geom_key(design: Design) -> str:
    import hashlib, json
    d = design.model_dump(mode="json")
    for k in ("name", "notes", "op", "motor", "analysis"):
        d.pop(k, None)
    for k in ("fitting_balance_mass", "fitting_balance_z"):     # point pins, added after the CSG
        d["flexure"].pop(k, None)
    return hashlib.sha256(json.dumps(d, sort_keys=True).encode()).hexdigest()[:16]


@lru_cache(maxsize=64)
def _fit_geom_cached(key, design_json):
    fits = build_strip_fittings(Design.model_validate_json(design_json))
    return {side: mass_properties(_sub(getattr(fits, side), [fits.keepout]))
            for side in ("hub", "blade")}


def fitting_mass_props(design: Design) -> dict:
    """Blade-1 fittings as rigid bodies, SI, frame H: {'hub'|'root': (mass kg, com m (3,),
    inertia about com kg m^2 (3,3))}. Geometry cached per geometry hash; density from
    materials.py (flexure.fitting_material, else hub pa6_cf / root = blade layup material)."""
    from .geometry import HUB_MATERIAL
    from .materials import get_material
    g = _fit_geom_cached(_geom_key(design), design.model_dump_json())
    fm = design.flexure.fitting_material
    out = {}
    for side, key, mat in (("hub", "hub", fm or HUB_MATERIAL), ("blade", "root", fm or design.blade.layup.material)):
        vol, com, C = g[side]
        rho = get_material(mat).rho * 1e-9          # kg/mm^3
        Cs = rho * C * 1e-6                          # kg m^2
        out[key] = (rho * vol, com * 1e-3, np.trace(Cs) * np.eye(3) - Cs)
    f = design.flexure
    mb = balance_pin_mass(design, out["root"]) if f.fitting_balance_mass == "auto" else float(f.fitting_balance_mass)
    if mb > 0:
        m, c, I = out["root"]
        z = f.fitting_balance_z
        out["root"] = add_point_masses(m, c, I, [(mb, np.array([c[0], 0.0, z])), (mb, np.array([c[0], 0.0, -z]))])
    return out


def add_point_masses(m, c, I, pts):
    """rigid body (m, com c, inertia I about c) + point masses [(mi, xi)] -> (m, com, I about com)"""
    ms = [m] + [p[0] for p in pts]
    xs = [np.asarray(c, float)] + [np.asarray(p[1], float) for p in pts]
    M = float(sum(ms))
    C = sum(mi * xi for mi, xi in zip(ms, xs)) / M
    J = np.array(I, float).copy()
    for mi, xi in zip(ms, xs):
        r = xi - C
        J += mi * ((r @ r) * np.eye(3) - np.outer(r, r))
    return M, C, J


def pitch_axis_moments(m, c, I) -> tuple[float, float, float]:
    """(Syy, Szz, Syz) = sum m (y^2, z^2, y z) of a rigid body (m, com c, inertia I about c; frame H)
    about the pitch axis (the radial x line through the hinge point, y = z = 0), kg m^2.
    Centrifugal propeller moment of the body for a pitch rotation th about x (Omega about z):
    M = -Omega^2 [(Syy - Szz) sin th cos th + Syz cos 2th]  ->  pitch stiffness
    K_prop = Omega^2 (Syy - Szz) (+ = restoring / pitch-flattening, chordwise spread; - = normal
    spread, destabilising: "Chinese weight") and a static moment -Omega^2 Syz."""
    S = 0.5 * np.trace(I) * np.eye(3) - np.asarray(I, float)      # second moments about com
    return (float(S[1, 1] + m * c[1] ** 2), float(S[2, 2] + m * c[2] ** 2),
            float(S[1, 2] + m * c[1] * c[2]))


def balance_pin_mass(design: Design, root=None) -> float:
    """kg per pin of the +-fitting_balance_z pin pair (at y = 0) that zeroes the root fitting's
    Syy - Szz about the pitch axis: the pair adds Szz = 2 m z^2 and no Syy / Syz."""
    if root is None:
        root = fitting_mass_props(design.model_copy(update=dict(
            flexure=design.flexure.model_copy(update=dict(fitting_balance_mass=0.0)))))["root"]
    syy, szz, _ = pitch_axis_moments(*root)
    return max(syy - szz, 0.0) / (2.0 * design.flexure.fitting_balance_z ** 2)


def _keepout(strips, prm: StripParams):
    ks = []
    for m in strips:
        p0, p1, L, t, w, n = _frame(m)
        ks.append(_box([0.05, -(prm.w / 2 + SIDE + STRIP_CLEAR), -(prm.t / 2 + STRIP_CLEAR)],
                       [L - 0.05, prm.w / 2 + SIDE + STRIP_CLEAR, prm.t / 2 + STRIP_CLEAR],
                       np.c_[t, w, n], p0))
    return _union(ks)


def build_strip_fittings(design: Design, blade: int = 1, over_deep: float | None = None) -> StripFittings:
    """Blade-1 hub and root fittings (frame H, mm) as manifold3d solids, with their cutters."""
    from .geometry import _hinge_frame
    strips, wires, hinge = strip_members(design)
    if not strips:
        raise ValueError("design has no strip flexure members")
    prm = strip_params(design)
    if over_deep is not None:
        prm = dataclasses.replace(prm, blade_extra=over_deep)
    P, s, t_, a = _hinge_frame(*hinge)
    P = P * 1e3
    Qs = slot_rotations(design, blade)
    ax = []
    hub, hc, notes, hpl = _side(design, strips, wires, P, s, 0, prm, Qs, ax)
    bl, bc, _, bpl = _side(design, strips, wires, P, s, 1, prm, Qs, ax)
    ko = _keepout(strips, prm)
    hub = _sub(hub, hc + [ko])
    bl = _sub(bl, bc + [ko])
    return StripFittings(prm, hub, bl, hc, bc, ko, P, np.asarray(a, float), notes, ax, hpl, bpl)


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
    return fitting_clearance(Design.model_validate_json(design_json), lag, pitch, n=3)


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
    V, F = np.asarray(mo.vert_properties)[:, :3].astype(float), np.asarray(mo.tri_verts)
    def ok(v):      # watertight after the float32 STL round trip and vertex merge a slicer sees
        return trimesh.Trimesh(v.astype(np.float32).astype(float), F, process=True).is_watertight
    if not ok(V):                                    # pinch edge (two sheets touching along a line,
        V = V.copy()                                 # valid for manifold3d, 4-valent after a merge)
        _, inv, cnt = np.unique(V, axis=0, return_inverse=True, return_counts=True)
        inv = np.asarray(inv).ravel()
        for g in np.where(cnt > 1)[0]:
            for k in np.where(inv == g)[0][1:]:      # split the extra copies 4 um into their own sheet
                nb = np.unique(F[(F == k).any(axis=1)])
                nb = nb[nb != k]
                dv = V[nb].mean(axis=0) - V[k]
                if np.linalg.norm(dv) > 1e-9:
                    V[k] += 0.004 * dv / np.linalg.norm(dv)
    tm = trimesh.Trimesh(V, F, process=True)
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


def breakaway_bridges(hub, blade, e, P, ko, n_bridges: int = 4):
    """2-4 thin breakaway sprues (SPRUE square, necked to SPRUE_NECK for SPRUE_NECK_LEN at both
    joints) between the hub fitting and the root fitting in the assembled pose (blade-1 frame).
    Each runs along the web-plane normal (62 deg from horizontal for 10in_easy: prints unsupported),
    is anchored 1 mm inside both webs, and clears the strip keep-out, the strand path and every
    cutter in `ko` (strip + strand keep-out). Returns (manifold, [(a, b, gap_mm)])."""
    e = np.asarray(e, float)
    P = np.asarray(P, float)
    v = np.array([0.0, 1.0, 0.0])
    u = np.cross(v, e); u /= np.linalg.norm(u)
    R = np.c_[e, u, v]
    both = _union([hub, blade])

    def seg(t0, t1, a, b, sz):
        return _box([t0, -sz / 2, -sz / 2], [t1, sz / 2, sz / 2], R, P + a * u + b * v)

    cand = []
    for a in np.arange(-30.0, 60.0, 2.0):
        for b in np.arange(-20.0, 20.1, 2.0):
            line = seg(-80.0, 80.0, a, b, 0.4)
            hp, bp = line ^ hub, line ^ blade
            if hp.is_empty() or bp.is_empty():
                continue
            vh = np.asarray(hp.to_mesh().vert_properties)[:, :3]
            vb = np.asarray(bp.to_mesh().vert_properties)[:, :3]
            th = float(((vh - (P + a * u + b * v)) @ e).max())
            tb = float(((vb - (P + a * u + b * v)) @ e).min())
            gap = tb - th
            if not (2 * SPRUE_NECK_LEN + 1.0 < gap < 20.0):
                continue
            anc_h, anc_b = seg(th - 1.0, th, a, b, SPRUE), seg(tb, tb + 1.0, a, b, SPRUE)
            full = SPRUE * SPRUE * 1.0 * 0.995
            if (anc_h ^ hub).volume() < full or (anc_b ^ blade).volume() < full:
                continue
            mid = seg(th + 1e-3, tb - 1e-3, a, b, SPRUE + 2 * SPRUE_CLEAR / 2)
            if not (mid ^ _union([both, ko])).is_empty():
                continue
            cand.append((a, b, th, tb))
    if len(cand) < 2:
        raise ValueError("no clear breakaway bridge sites between the fittings")
    gmin = min(c[3] - c[2] for c in cand)
    cand = [c for c in cand if c[3] - c[2] < gmin + 3.0] if sum(c[3] - c[2] < gmin + 3.0 for c in cand) >= 3 else cand
    pick = [max(cand, key=lambda c: (c[0] - np.mean([x[0] for x in cand])) ** 2 + (c[1] ** 2) * 4)]
    while len(pick) < min(n_bridges, len(cand)):
        pick.append(max(cand, key=lambda c: min((c[0] - p[0]) ** 2 + (c[1] - p[1]) ** 2 for p in pick)))
    parts, info = [], []
    nl = SPRUE_NECK_LEN
    for a, b, th, tb in pick:
        parts += [seg(th - 1.0, th, a, b, SPRUE), seg(th - 1e-3, th + nl, a, b, SPRUE_NECK),
                  seg(th + nl - 1e-3, tb - nl + 1e-3, a, b, SPRUE),
                  seg(tb - nl, tb + 1e-3, a, b, SPRUE_NECK), seg(tb, tb + 1.0, a, b, SPRUE)]
        info.append((float(a), float(b), float(tb - th)))
    return _union(parts), info


def strip_print_parts(design: Design, breakaway_jig: bool = False) -> dict:
    """Printable solids (manifolds, mm, frame H): 'hub' (hub + spokes + all hub fittings) and
    'blade<k>' (blade k airfoil + shank + root fitting), with all cutters applied.

    breakaway_jig=True: easy build. Blade slots are over-deep (land + OVERDEEP, no stops) and the
    whole rotor prints as ONE body 'rotor' (hub + every blade) with 2-4 breakaway sprues holding
    each blade's root fitting at its assembled pose, so the print sets the jaw-exit spacing;
    'bridges' holds the sprues alone. The sprues are snipped after the strips are glued in. The
    blade (not just the root fitting) is joined: the blade is already one body with its fitting in
    the assembled pose, so there is no second joint to align or glue."""
    fits = build_strip_fittings(design, over_deep=OVERDEEP if breakaway_jig else None)
    n = design.blade.n_blades
    fits_k = {1: fits, 0: build_strip_fittings(design, 2, OVERDEEP if breakaway_jig else None) if n > 1 else fits}   # odd / even blades
    sp = loft_manifold(design, "blade1_spoke", hull=True)
    sp = _mf().Manifold.batch_hull([sp, sp.translate((-1.0, 0.0, 0.0))])  # overlap, not touch, at the centre
    hub_core = _union([loft_manifold(design, "hub", hull=True), sp])
    hubs = []
    for par, f_ in fits_k.items():
        core = _sub(hub_core, f_.hub_cuts + [f_.keepout])
        hubs.append((par, _union([f_.hub, core])))
    hub = _union([_mirror_rot(dict(hubs)[k % 2], k, n) for k in range(1, n + 1)])
    out = {"hub": hub}
    mf = _mf()
    env = loft_manifold(design, "blade1_root_fitting", hull=True)
    shank0 = loft_manifold(design, "blade1_shank", hull=True)
    e, off = fits.blade_plane
    body = loft_manifold(design, "blade1")      # airfoils are not mirrored between blades
    for k in range(1, n + 1):
        fk = fits_k[k % 2]
        fb = _sub(fk.blade, fk.blade_cuts)
        cuts = list(fk.blade_cuts) + [fk.keepout]
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
        if breakaway_jig:
            hk = dict(hubs)[k % 2]
            fz = (lambda m_: m_.mirror((0.0, 0.0, 1.0))) if k % 2 == 0 else (lambda m_: m_)
            zs = np.array([1.0, 1.0, -1.0 if k % 2 == 0 else 1.0])
            keep = [fz(fk.keepout)]
            for m in strip_members(design)[1]:           # strand path keep-out
                a0, a1 = np.asarray(m.p0, float) * 1e3 * zs, np.asarray(m.p1, float) * 1e3 * zs
                keep.append(_cyl(a0, a1 - a0, fk.params.channel_d / 2 + SPRUE_CLEAR + WALL,
                                 float(np.linalg.norm(a1 - a0))))
            br, binfo = breakaway_bridges(fz(hk), bk, ek, fk.hinge_point * zs, _union(keep))
            out.setdefault("_bridges", []).append(br.rotate((0.0, 0.0, 360.0 * (k - 1) / n)))
            out.setdefault("_bridge_info", []).append(binfo)
    out["_fits"] = fits
    if breakaway_jig:
        br = _union(out.pop("_bridges"))
        out["bridges"] = br
        out["rotor"] = _union([out.pop("hub")] + [out.pop(f"blade{k}") for k in range(1, n + 1)] + [br])
    return out
