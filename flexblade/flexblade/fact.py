"""Freedom-and-Constraint Topology (FACT) for the skewed lag flexure (OWNER: structures).

Screw conventions (all in frame H, about the origin):
  twist  T = [ω; v],  pure rotation about the line (p, d):  T = [d; p × d]
  wrench W = [f; m],  pure force along the line (p, d):     W = [d; p × d]
  pure translation along d: T = [0; d];  pure couple about d: W = [0; d]
  reciprocal (virtual power) product  T ∘ W = ω·m + v·f.
A constraint (wrench) space is the set of wrenches reciprocal to every twist of the
freedom space; dim(freedom) + dim(constraint) = 6.

Hinge axis (blade 1)
--------------------
Lag ζ is rotation about −z, pitch θ is rotation about +x. A rotation α about the unit
axis a gives pitch α·a_x and lag −α·a_z, so θ/ζ = −a_x/a_z. For θ = s ζ tanδ
(s = +1 blade 1, −1 blade 2) the axis is

        a = (s·sinδ, 0, −cosδ),     through H0 = (hinge_radius, 0, 0),

i.e. the lag axis −z tilted by δ toward the span (+x) for blade 1. Positive rotation
about `a` = positive lag (blade trails) with pitch-up for blade 1. `hinge_axis`
returns (H0, a). Blade 2 is the mirror image z → −z of blade 1's flexure (then
rotated by π about z): a mirror in the rotor plane flips pitch but keeps lag, so
s = −1 exactly as required. For blade k even this mirror is applied by rotor.py.

Members are generated for blade 1 with the hub end at p0 and the blade end at p1.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .schema import Design, Flexure, FlexureMember, Layup, PrintOrientation
from .materials import SHEET_MATERIALS

# --------------------------------------------------------------------------- screw algebra


def unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def rotation_twist(p, d) -> np.ndarray:
    d = unit(d)
    return np.r_[d, np.cross(p, d)]


def translation_twist(d) -> np.ndarray:
    return np.r_[np.zeros(3), unit(d)]


def line_wrench(p, d) -> np.ndarray:
    d = unit(d)
    return np.r_[d, np.cross(p, d)]


def couple_wrench(d) -> np.ndarray:
    return np.r_[np.zeros(3), unit(d)]


_DELTA = np.block([[np.zeros((3, 3)), np.eye(3)], [np.eye(3), np.zeros((3, 3))]])


def reciprocal(T, W) -> float:
    return float(np.asarray(T) @ _DELTA @ np.asarray(W))


def complement(screws: np.ndarray, tol: float = 1e-9) -> np.ndarray:
    """Reciprocal complement: columns spanning {W : T∘W = 0 ∀ T in span(screws)}."""
    S = np.atleast_2d(np.asarray(screws, float))
    if S.shape[0] != 6:
        S = S.T
    A = (S.T @ _DELTA)                          # rows: Tᵀ Δ
    u, s, vt = np.linalg.svd(A)
    rank = int((s > tol * max(1.0, s.max() if s.size else 1.0)).sum())
    return vt[rank:].T                          # 6 × (6 − rank)


def screw_rank(screws, tol: float = 1e-8) -> int:
    S = np.atleast_2d(np.asarray(screws, float))
    if S.shape[0] != 6:
        S = S.T
    # normalise translational parts by a length scale to make rank well posed
    s = np.linalg.svd(S, compute_uv=False)
    return int((s > tol * s.max()).sum()) if s.size else 0


def span_contains(basis: np.ndarray, screws: np.ndarray, tol: float = 1e-8) -> bool:
    """True if every column of `screws` lies in span(basis)."""
    B = np.atleast_2d(basis)
    S = np.atleast_2d(screws)
    if B.shape[0] != 6:
        B = B.T
    if S.shape[0] != 6:
        S = S.T
    x, *_ = np.linalg.lstsq(B, S, rcond=None)
    return bool(np.abs(B @ x - S).max() < tol * max(1.0, np.abs(S).max()))


# --------------------------------------------------------------------------- hinge axis / spaces


def hinge_axis(design: Design, blade: int = 1) -> tuple[np.ndarray, np.ndarray]:
    """(point, unit direction) of the virtual lag hinge for blade 1 (or 2: mirrored)."""
    f = design.flexure
    d = np.radians(f.skew_deg)
    s = 1.0 if (blade % 2 == 1 or getattr(f, "blade2_flexure", "mirror_z") == "c2") else -1.0
    return np.array([f.hinge_radius, 0.0, 0.0]), np.array([s * np.sin(d), 0.0, -np.cos(d)])


def ideal_K6(design: Design) -> np.ndarray:
    """6×6 stiffness of the "ideal" topology at the hinge point, blade-1 frame H
    ([ux,uy,uz,rx,ry,rz] → [F;M]): flexure.ideal.K6 if given, else block-diagonal with
    translations diag(k_axial, k_inplane, k_outplane) and rotations Q diag(k_hinge, k_cross,
    k_flap) Qᵀ, Q = [a, a × y, y] (a = skewed hinge axis)."""
    from .schema import IdealFlexure
    ip = design.flexure.ideal or IdealFlexure()
    if ip.K6 is not None:
        return np.asarray(ip.K6, float)
    _, a = hinge_axis(design, 1)
    y = np.array([0.0, 1.0, 0.0])
    Q = np.column_stack([a, np.cross(a, y), y])
    K = np.zeros((6, 6))
    K[:3, :3] = np.diag([ip.k_axial, ip.k_inplane, ip.k_outplane])
    K[3:, 3:] = Q @ np.diag([ip.k_hinge, ip.k_cross, ip.k_flap]) @ Q.T
    return K


def freedom_space(design: Design, radial: bool = False) -> np.ndarray:
    """6×k twists: rotation about the hinge axis (+ radial translation if requested)."""
    p, a = hinge_axis(design)
    T = [rotation_twist(p, a)]
    if radial:
        T.append(translation_twist([1.0, 0.0, 0.0]))
    return np.array(T).T


def constraint_space(design: Design, radial: bool = False) -> np.ndarray:
    return complement(freedom_space(design, radial))


def _frame(design: Design):
    """Hinge frame for blade 1: H0, a (axis), xp (⟂ a, closest to +x), n = a × xp."""
    H0, a = hinge_axis(design)
    xp = unit(np.array([1.0, 0, 0]) - a * a[0])
    n = np.cross(a, xp)
    return H0, a, xp, n


def _rot_about(v, axis, ang):
    v = np.asarray(v, float)
    k = unit(axis)
    return v * np.cos(ang) + np.cross(k, v) * np.sin(ang) + k * (k @ v) * (1 - np.cos(ang))


# --------------------------------------------------------------------------- layouts


def generate_members(design: Design) -> list[FlexureMember]:
    """FACT member layout for blade 1 (hub end p0, blade end p1), per Flexure.topology.

    skewed_cross_blade : two plates, planes both containing the hinge axis a, length
        directions l± = cos(γ/2) x' ± sin(γ/2) ŷ (⟂ a), crossing at the hinge point,
        width along a; offset ±(w + cross_gap)/2 along a so they do not touch or fuse when
        printed (Haringx cross-spring). Constraint lines: all lines in either plane → 5-system.
    skewed_cross_blade_sym: as above but the "−" set is split into two half-width plates
        either side of the full-width "+" plate, cross_gap clear of it along a (symmetric
        cross-spring; centrifugal load produces no parasitic couple).
    skewed_single_blade: one plate, length along x' (⟂ a), width along a, centred on
        the hinge point. Constrains 3 (in-plane); bending about a is the hinge, plate
        twist and out-of-plane shear are the parasitic soft DOFs (centrifugal tension
        stiffens the latter).
    integral_taper     : as single blade, but it *is* the blade root (the blade
        material/print orientation, thickness taper `taper`), from the hub radius to
        hinge_radius + length/2 along x'.
    wire_triplet       : 5 wires, all intersecting a: crossed pairs (l±) through
        A± = H0 ± (w/2) a and one inclined wire through H0 along (x' + a)/√2 (gives
        the force component along a). Wire diameter = thickness.
    custom             : flexure.members as given.
    rigid              : one rigid member from hub radius to hinge radius + length/2.
    The optional tension_member is appended for all topologies, and a radial tension rod
    (hub edge → blade end, through H0) when flexure.tension_rod_diameter > 0.
    """
    f = design.flexure
    H0, a, xp, n = _frame(design)
    L, w, t = f.length, f.width, f.thickness
    lay, ori = f.layup, f.orientation
    tw = np.radians(f.twist_deg)
    out: list[FlexureMember] = []

    def plate(c, ldir, wdir, length, taper=f.taper, layup=lay, orientation=ori, start=None):
        ldir = unit(ldir)
        wd = _rot_about(wdir, ldir, tw)
        p0 = c - 0.5 * length * ldir if start is None else start
        p1 = p0 + length * ldir
        # sheet stock (spring-steel shim, FR4) is clamped in the printed fittings, not butted
        anc = "embedded" if (layup.kind == "solid" and layup.material.split("@")[0]
                             in SHEET_MATERIALS) else "butt"
        return FlexureMember(kind="blade", p0=tuple(map(float, p0)), p1=tuple(map(float, p1)),
                             width=w, thickness=t, width_dir=tuple(map(float, wd)), shape="rect",
                             taper=taper, layup=layup, orientation=orientation, anchorage=anc)

    topo = f.topology
    if topo == "skewed_cross_blade":
        g = np.radians(f.cross_angle_deg) / 2
        for sg in (1.0, -1.0):
            ldir = np.cos(g) * xp + sg * np.sin(g) * np.array([0.0, 1.0, 0.0])
            ldir = unit(ldir - a * (a @ ldir))
            c = H0 + sg * 0.5 * (w + f.cross_gap) * a
            out.append(plate(c, ldir, a, L))
    elif topo == "skewed_cross_blade_sym":
        # set "+" : one plate of width w centred on the hinge point; set "−": two plates of
        # width w/2 centred at ±(w/2 + cross_gap + w/4) along a. Each set's
        # resultant lies on the hinge point, so axial (centrifugal) load gives no couple.
        g = np.radians(f.cross_angle_deg) / 2
        gap = f.cross_gap
        for sg in (1.0, -1.0):
            ldir = np.cos(g) * xp + sg * np.sin(g) * np.array([0.0, 1.0, 0.0])
            ldir = unit(ldir - a * (a @ ldir))
            if sg > 0:
                out.append(plate(H0, ldir, a, L))
            else:
                for side in (1.0, -1.0):
                    m = plate(H0 + side * (0.75 * w + gap) * a, ldir, a, L)
                    out.append(m.model_copy(update=dict(width=0.5 * w)))
    elif topo == "skewed_single_blade":
        out.append(plate(H0, xp, a, L))
    elif topo == "integral_taper":
        # plate along x' from where it leaves the hub (x = hub_radius) through the hinge point
        s0 = (f.hub_radius - H0[0]) / xp[0]          # parameter along x' with x = hub radius
        start = H0 + s0 * xp
        length = max(H0[0] + 0.5 * L * xp[0] - f.hub_radius, 0.25 * L) / xp[0]
        out.append(plate(None, xp, a, length, start=start, layup=f.layup,
                         orientation=design.blade.orientation if f.layup == design.blade.layup else ori))
    elif topo == "wire_triplet":
        g = np.radians(f.cross_angle_deg) / 2
        for side in (1.0, -1.0):
            A = H0 + side * 0.5 * w * a
            for sg in (1.0, -1.0):
                ldir = unit(np.cos(g) * xp + sg * np.sin(g) * np.array([0.0, 1.0, 0.0]))
                ldir = unit(ldir - a * (a @ ldir))
                p0, p1 = A - 0.5 * L * ldir, A + 0.5 * L * ldir
                out.append(FlexureMember(kind="wire", p0=tuple(p0), p1=tuple(p1), width=t, thickness=t,
                                         width_dir=tuple(a), shape="circle", taper=f.taper,
                                         layup=lay, orientation=ori))
        d5 = unit(xp + a)
        p0, p1 = H0 - 0.5 * L * d5, H0 + 0.5 * L * d5
        out.append(FlexureMember(kind="wire", p0=tuple(p0), p1=tuple(p1), width=t, thickness=t,
                                 width_dir=tuple(a), shape="circle", taper=f.taper, layup=lay,
                                 orientation=ori))
    elif topo == "torsion_bar":
        Lb = f.bar_length or L
        # bar on the hinge axis; raster/fibre direction along the bar
        ybuild = np.cross(a, xp)
        ybuild = ybuild / np.linalg.norm(ybuild)
        bar_ori = PrintOrientation(build_dir=tuple(map(float, ybuild)), raster_dir=tuple(map(float, a)))
        p0, p1 = H0 - 0.5 * Lb * a, H0 + 0.5 * Lb * a
        out.append(FlexureMember(kind="wire", p0=tuple(map(float, p0)), p1=tuple(map(float, p1)),
                                 width=f.bar_diameter, thickness=f.bar_diameter,
                                 width_dir=tuple(map(float, xp)), shape="circle", taper=1.0,
                                 layup=Layup(kind="solid", material=f.bar_material),
                                 orientation=bar_ori))
        if f.bar_with_plate:
            out.append(plate(H0, xp, a, L))
    elif topo == "skewed_cruciform":
        # X-section on the hinge axis: plate lengths along a, widths at ±γ/2 about xp in the
        # plane ⟂ a; fibres/raster along a; ends embedded in the fittings (no butt joint)
        Lb = f.bar_length or L
        g = np.radians(f.cross_angle_deg) / 2
        p0, p1 = H0 - 0.5 * Lb * a, H0 + 0.5 * Lb * a
        for sg in (1.0, -1.0):
            wd = unit(np.cos(g) * xp + sg * np.sin(g) * n)
            nb = np.cross(a, wd)            # plate normal = build dir (plate printed flat)
            out.append(FlexureMember(kind="blade", p0=tuple(map(float, p0)), p1=tuple(map(float, p1)),
                                     width=w, thickness=t, width_dir=tuple(map(float, wd)),
                                     shape="rect", taper=1.0, layup=lay, anchorage="embedded",
                                     orientation=PrintOrientation(build_dir=tuple(map(float, nb)),
                                                                  raster_dir=tuple(map(float, a)))))
        if f.cruciform_rod_diameter > 0:
            d_c = f.cruciform_rod_diameter
            out.append(FlexureMember(kind="wire", p0=tuple(map(float, p0)), p1=tuple(map(float, p1)),
                                     width=d_c, thickness=d_c, width_dir=tuple(map(float, xp)),
                                     shape="circle", taper=1.0, anchorage="embedded",
                                     layup=Layup(kind="solid", material=f.bar_material),
                                     orientation=PrintOrientation(build_dir=tuple(map(float, n)),
                                                                  raster_dir=tuple(map(float, a)))))
    elif topo in ("custom", "topopt", "ideal"):
        out.extend(f.members)
    elif topo == "rigid":
        r1 = H0[0] + 0.5 * L
        out.append(FlexureMember(kind="rigid", p0=(f.hub_radius, 0.0, 0.0), p1=(r1, 0.0, 0.0),
                                 width=max(w, 0.004), thickness=max(t, 0.004), layup=lay,
                                 orientation=ori))
    else:  # pragma: no cover
        raise ValueError(topo)
    if f.tension_member is not None:
        out.append(f.tension_member)
    if f.tension_rod_diameter > 0 and out:
        # radial rod through H0 (on the hinge axis → reciprocal to the hinge twist), from the
        # hub edge to the outermost blade-end x of the members (where the blade attaches)
        x1 = max(max(m.p0[0], m.p1[0]) for m in out)
        d_r = f.tension_rod_diameter
        out.append(FlexureMember(kind="wire", p0=(float(f.hub_radius), 0.0, 0.0), p1=(float(x1), 0.0, 0.0),
                                 width=d_r, thickness=d_r, width_dir=(0.0, 0.0, 1.0), shape="circle",
                                 taper=1.0, layup=Layup(kind="solid", material=f.tension_rod_material),
                                 orientation=PrintOrientation(build_dir=(0.0, 0.0, 1.0),
                                                              raster_dir=(1.0, 0.0, 0.0))))
    from .printsetup import orient_generated_members
    out = orient_generated_members(design, out)
    if f.strap_strands > 0 and out:     # strands: not printed, fibre along the strand
        out.extend(strap_members(design, out))
    return out


def strap_members(design: Design, others: list[FlexureMember] | None = None) -> list[FlexureMember]:
    """Tension-torsion strap: flexure.strap_strands unbonded strands (cable members) parallel to
    the radial line through the hinge point H0, on a circle of radius strap_radius about it,
    spanning strap_length centred on H0 (0 = hub radius → outermost blade-end x of `others`).
    Each strand carries axial load only (EI, GJ × strap_bend_factor); its hinge stiffness is the
    geometric (tension) stiffness from the co-rotational beam: ≈ T r²/L per strand about the
    radial axis, and −T L1 L2/(L1+L2) (destabilising) for rotation about an axis ⟂ to it."""
    f = design.flexure
    H0, _ = hinge_axis(design)
    if f.strap_length > 0:
        x0, x1 = H0[0] - 0.5 * f.strap_length, H0[0] + 0.5 * f.strap_length
    else:
        x0 = f.hub_radius
        x1 = max((max(m.p0[0], m.p1[0]) for m in (others or [])), default=H0[0] + 0.5 * f.length)
    n, r, d = f.strap_strands, f.strap_radius if f.strap_strands > 1 else 0.0, f.strap_strand_diameter
    out = []
    for i in range(n):
        ph = 2 * np.pi * (i + 0.5) / n
        o = np.array([0.0, r * np.cos(ph), r * np.sin(ph)])
        out.append(FlexureMember(kind="wire", p0=tuple(map(float, H0 + o + [x0 - H0[0], 0, 0])),
                                 p1=tuple(map(float, H0 + o + [x1 - H0[0], 0, 0])),
                                 width=d, thickness=d, width_dir=(0.0, 0.0, 1.0), shape="circle",
                                 taper=1.0, anchorage="embedded", cable=True,
                                 cable_bend_factor=f.strap_bend_factor,
                                 layup=Layup(kind="solid", material=f.strap_material),
                                 orientation=PrintOrientation(build_dir=(0.0, 0.0, 1.0),
                                                              raster_dir=(1.0, 0.0, 0.0))))
    return out


def blade_flexure_T(design: Design, k: int) -> np.ndarray:
    """Point map blade-1 flexure → blade-k flexure in blade k's own frame (flexure.blade2_flexure):
    mirror_z diag(1,1,−1) (default), rot_x diag(1,−1,−1) (180° about the pitch axis), c2 identity."""
    mode = getattr(design.flexure, "blade2_flexure", "mirror_z")
    if k % 2 == 1 or mode == "c2":
        return np.eye(3)
    return np.diag([1.0, -1.0, -1.0]) if mode == "rot_x" else np.diag([1.0, 1.0, -1.0])


def mirror_member(m: FlexureMember, T: np.ndarray | None = None) -> FlexureMember:
    """Map by T (default mirror z → −z): blade-2 flexure in its own blade frame."""
    T = np.diag([1.0, 1.0, -1.0]) if T is None else np.asarray(T, float)
    def mz(v):
        return tuple(map(float, T @ np.asarray(v, float)))
    o = m.orientation
    return m.model_copy(update=dict(
        p0=mz(m.p0), p1=mz(m.p1), width_dir=mz(m.width_dir),
        orientation=PrintOrientation(build_dir=mz(o.build_dir), raster_dir=mz(o.raster_dir))))


def member_constraint_wrenches(m: FlexureMember) -> np.ndarray:
    """Ideal constraint wrenches of a member: wire → its line; blade plate → all
    wrenches in its plane (force along length, force along width, couple ⟂ plane)."""
    p0, p1 = np.asarray(m.p0), np.asarray(m.p1)
    c = 0.5 * (p0 + p1)
    l = unit(p1 - p0)
    if m.kind == "wire":
        return line_wrench(c, l)[:, None]
    if m.kind == "rigid":
        return np.eye(6)
    wd = np.asarray(m.width_dir, float)
    wd = unit(wd - l * (l @ wd))
    nrm = np.cross(l, wd)
    return np.array([line_wrench(c, l), line_wrench(c, wd), couple_wrench(nrm)]).T


def layout_constraint_space(members: list[FlexureMember]) -> np.ndarray:
    W = np.hstack([member_constraint_wrenches(m) for m in members])
    u, s, vt = np.linalg.svd(W, full_matrices=False)
    r = int((s > 1e-9 * s.max()).sum())
    return u[:, :r]


# --------------------------------------------------------------------------- stiffness check


@dataclass
class LayoutCheck:
    K6: np.ndarray                  # 6×6 stiffness at the hinge point H0, [f; m] vs [u; θ]
    soft_axis: np.ndarray           # unit direction of softest rotation
    soft_point: np.ndarray          # point on the instantaneous axis
    axis_error_deg: float           # angle between soft axis and target hinge axis
    axis_offset: float              # distance of soft axis from H0 (m) ⟂ to the axis
    k_soft: float                   # N m/rad about the soft axis (translations free)
    stiffness_ratio: float          # min(next rotational, translational·Lc²) / k_soft
    ideal_constraint_dim: int       # rank of the ideal member constraint lines
    ideal_constraints_reciprocal: bool  # every ideal constraint reciprocal to the hinge twist
    message: str = ""


def check_layout(design: Design, members: list[FlexureMember] | None = None,
                 Omega: float = 0.0) -> LayoutCheck:
    """Build the members alone with beam.py (hub ends clamped, blade ends joined by a
    stiff connector to a node at H0), compute the 6×6 stiffness at H0 and compare its
    softest rotation with the target hinge axis."""
    from .rotor import flexure_only_model     # local import (rotor uses fact)
    members = members if members is not None else generate_members(design)
    H0, a = hinge_axis(design)
    K6 = flexure_only_model(design, members).stiffness_at(H0)
    Ktt, Ktr, Krt, Krr = K6[:3, :3], K6[:3, 3:], K6[3:, :3], K6[3:, 3:]
    Kr = Krr - Krt @ np.linalg.solve(Ktt, Ktr)
    lam, V = np.linalg.eigh(0.5 * (Kr + Kr.T))
    w = V[:, 0]
    if w @ a < 0:
        w = -w
    v = -np.linalg.solve(Ktt, Ktr @ w)           # translation at H0 accompanying unit rotation
    p_rel = np.cross(w, v) / (w @ w)             # rotation axis point relative to H0
    p_perp = p_rel - a * (a @ p_rel)
    ang = float(np.degrees(np.arccos(np.clip(abs(w @ a), -1, 1))))
    Lc = max(design.flexure.length, 1e-4)
    k_tt = float(np.linalg.eigvalsh(Ktt).min()) * Lc**2
    ratio = float(min(lam[1], k_tt) / max(lam[0], 1e-30))
    Wid = layout_constraint_space(members) if all(m.kind != "rigid" for m in members) else np.eye(6)
    tw = rotation_twist(H0, a)
    recip = bool(np.all(np.abs(tw @ _DELTA @ Wid) < 1e-8)) if Wid.shape[1] < 6 else False
    return LayoutCheck(K6=K6, soft_axis=w, soft_point=H0 + p_rel, axis_error_deg=ang,
                       axis_offset=float(np.linalg.norm(p_perp)), k_soft=float(lam[0]),
                       stiffness_ratio=ratio, ideal_constraint_dim=int(Wid.shape[1]),
                       ideal_constraints_reciprocal=recip)


# --------------------------------------------------------------------------- stopped (assembled) state
STOPPED_BUCKLING_MIN = 2.0       # buckling multiplier of the slot-misfit stress state
STOPPED_YIELD_FRAC = 0.6         # max clamp-edge stress / Xt


def _hinge_terms(K, a):
    a = np.asarray(a, float) / np.linalg.norm(a)
    y = np.array([0.0, 1.0, 0.0])
    s = np.cross(y, a)
    s /= np.linalg.norm(s)
    Cr = np.linalg.inv(K)[3:, 3:]
    # fixed-axis terms (as ccx.compare_strip_flexure) are contaminated by the ~1e3x softer hinge
    # compliance once the soft axis tilts (sin^2 tilt * C_hinge); the principal ones are not
    lam, V = np.linalg.eigh(0.5 * (Cr + Cr.T))
    tilt = float(np.degrees(np.arccos(min(1.0, abs(V[:, -1] @ a)))))
    return dict(k_hinge=float(1 / (a @ Cr @ a)), k_cross=float(1 / (s @ Cr @ s)), k_flap=float(1 / (y @ Cr @ y)),
                k_soft=float(1 / lam[-1]), k_2nd=float(1 / lam[-2]), axis_tilt_deg=tilt)


def stopped_state(design: Design, n_steps: int = 10, tol: float = 1e-6, max_it: int = 40,
                  probe: float = 1e-4, dbg: bool = False) -> dict:
    """Rotor stopped on the bench (no CF, no aero, no gravity): the pre-angled strip slots
    (flexure.strip_preset_deg) force the strips + blade fittings into a self-equilibrated state.
    Full Newton on the flexure-only assembly (all blades; blade chains removed, fittings free),
    preset ramped 0 -> 100 % in n_steps. Returns per-strip axial force N vs Euler (clamped-clamped
    4 pi^2 EI_min/L^2), max fibre stress (clamp ends x strip_kt), the buckling multiplier mu of the
    misfit stress state (Km + mu Kgeo singular), and the stopped tangent k_cross/k_flap/k_hinge per
    blade at the hinge point (vs the same assembly without presets)."""
    from . import rotor as R
    from .beam import assemble, assemble_Kmat, element_end_forces
    import scipy.linalg as sla
    m = R.build_model(design)
    fe = m.fe
    drop = {i for bi in m.blades for i in bi.blade_elems}
    fe.elements = [e for i, e in enumerate(fe.elements) if i not in drop]
    used = {e.n0 for e in fe.elements} | {e.n1 for e in fe.elements}
    for n in range(fe.nn):
        if n == m.hub or n not in used:
            fe.fix(n)
    fe.prepare()
    fe._pack = None
    pre = {i: e.d7_0.copy() for i, e in enumerate(fe.elements) if e.d7_0 is not None and np.any(e.d7_0)}
    free = fe.free_dofs()

    def tangent(st):
        sa = assemble(fe, st, 0.0, True)
        return sa, assemble_Kmat(fe, sa), sa.Kgeo

    def kterms(st):
        out = {}
        for k, bi in enumerate(m.blades, 1):
            fo = R.FlexureOnly(fe, bi.master, [m.hub])
            out[k] = _hinge_terms(fo.stiffness_at(bi.hinge_point, state=st), bi.hinge_dir)
        return out

    for i in pre:
        fe.elements[i].d7_0 = None
    fe._pack = None
    st = fe.initial_state()
    ref = kterms(None)
    # solve_static normalises its residual by max(|F|, |f_int|) on free DOFs, which is 1 at a
    # self-equilibrated (F = 0) state; a probe force (PROBE N along z on each blade fitting, ~0.1 % of
    # the misfit reactions) gives it an absolute scale: converged = |r| < tol * PROBE.
    from .beam import solve_static
    fprobe = np.zeros(fe.ndof)
    for bi in m.blades:
        fprobe[6 * bi.master + 2] = probe
    ok, path, iters, msgs = True, [], [], []
    for i, p in pre.items():
        fe.elements[i].d7_0 = p
    fe._pack = None
    fscale = float(np.abs(assemble(fe, st, 0.0, False).fint[free]).max()) if pre else 1.0  # unrelaxed misfit
    for s in (np.linspace(1.0 / n_steps, 1.0, n_steps) if pre else []):
        for i, p in pre.items():
            fe.elements[i].d7_0 = s * p
        fe._pack = None
        st, info = solve_static(fe, 0.0, fprobe, state0=st, tol=tol, max_iter=max_it, verbose=dbg)
        ok = ok and bool(info.converged or info.residual * probe < 1e-4 * fscale)
        iters.append(getattr(info, "iterations", None))
        msgs.append(info.message)
        path.append(kterms(st)[1]["k_cross"])
    sa, Km, Kg = tangent(st)
    lam = sla.eigh(-Kg[np.ix_(free, free)], Km[np.ix_(free, free)], eigvals_only=True)
    mu = float(1.0 / lam.max()) if lam.max() > 1e-12 else float("inf")
    kt = R.strip_kt(design.flexure)
    groups: dict = {}
    for i, e in enumerate(fe.elements):
        if e.kind == "flexure":
            groups.setdefault(e.tag, []).append(i)
    cab = {f"b{k}.flex{j}" for k, bi in enumerate(m.blades, 1) for j, mm in enumerate(bi.members) if mm.cable}
    strips, smax = [], 0.0
    for tag, ids in groups.items():
        if tag in cab:
            continue
        L = sum(fe.elements[i].L0 for i in ids)
        sec = fe.elements[ids[0]].sections[0]
        Pcr = 4 * np.pi ** 2 * min(sec.EIy, sec.EIz) / L ** 2
        Ns, sg = [], 0.0
        for n_, i in enumerate(ids):
            e = fe.elements[i]
            fa, fb = element_end_forces(e, sa.cr[i], st)
            Ns.append(0.5 * (fa[0] + fb[0]))
            for end, sf in ((0, fa), (1, fb)):
                fac = kt if (n_ == 0 and end == 0) or (n_ == len(ids) - 1 and end == 1) else 1.0
                sg = max(sg, fac * max(abs(x) for x in R._fibre_mean(e.sections[0], sf)))
        strips.append(dict(tag=tag, N=float(np.mean(Ns)), Pcr=float(Pcr), sigma=float(sg)))
        smax = max(smax, sg)
    kk = kterms(st) if pre else ref
    from .beam import _logm
    for k, bi in enumerate(m.blades, 1):
        w = _logm(st.R[bi.master])
        kk[k]["fitting_rot_deg"] = float(np.degrees(np.linalg.norm(w)))
        kk[k]["fitting_rot_hinge_deg"] = float(np.degrees(w @ bi.hinge_dir / np.linalg.norm(bi.hinge_dir)))
    for k in kk:
        kk[k].update({f"{n}0": v for n, v in ref[k].items()})
    return dict(converged=ok, strips=strips, sigma_max=float(smax), buckling_multiplier=mu, k=kk,
                path_kcross=path, iters=iters, messages=msgs, n_steps=n_steps, misfit_scale=fscale,
                model=m, state=st)
