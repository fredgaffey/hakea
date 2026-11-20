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

from .schema import Design, Flexure, FlexureMember, PrintOrientation

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
    s = 1.0 if blade % 2 == 1 else -1.0
    return np.array([f.hinge_radius, 0.0, 0.0]), np.array([s * np.sin(d), 0.0, -np.cos(d)])


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
        width along a; offset ±(w/2) along a so they do not intersect (Haringx
        cross-spring). Constraint lines: all lines in either plane → 5-system.
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
    The optional tension_member is appended for all topologies.
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
        return FlexureMember(kind="blade", p0=tuple(map(float, p0)), p1=tuple(map(float, p1)),
                             width=w, thickness=t, width_dir=tuple(map(float, wd)), shape="rect",
                             taper=taper, layup=layup, orientation=orientation)

    topo = f.topology
    if topo == "skewed_cross_blade":
        g = np.radians(f.cross_angle_deg) / 2
        for sg in (1.0, -1.0):
            ldir = np.cos(g) * xp + sg * np.sin(g) * np.array([0.0, 1.0, 0.0])
            ldir = unit(ldir - a * (a @ ldir))
            c = H0 + sg * 0.5 * w * a
            out.append(plate(c, ldir, a, L))
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
    elif topo in ("custom", "topopt"):
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
    return out


def mirror_member(m: FlexureMember) -> FlexureMember:
    """Mirror z → −z (blade-2 flexure in its own blade frame)."""
    def mz(v):
        return (float(v[0]), float(v[1]), float(-v[2]))
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
