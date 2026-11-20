"""Rotor assembly: trim prestress, modes/Campbell, 1/rev harmonic response, checks
(OWNER: structures).

Model
-----
Hub node 0 at the origin: all DOFs fixed except rz (hub rotation φ about +z, with
J_hub + J_rotor lumped). For each blade k (azimuth ψ0_k = 2π(k−1)/B):
  * flexure members from fact.generate_members (blade 1 frame; k even mirrored z→−z,
    then rotated by ψ0_k), n_flexure_elements beam elements each;
  * stiff massless "spokes" hub node → member hub ends, and "connectors" member blade
    ends → blade master node (on the blade reference line at the outermost member end);
  * blade beam chain from the master node to the tip on the reference line
    (x/c = elastic_axis_chord_frac); a shank (root-station section) up to the aero
    root cut-out, then n_blade_elements uniform elements whose nodes are the aero
    stations. Sections from section.py (airfoil polygon, twisted by twist+collective).
Stiff elements are 1e3 × the stiffest real section (EA, EI, GJ) — not exact rigid links,
but co-rotational, so their initial-stress terms are exact.

Blade-local frame of blade k: v_local = Rz(−ψ0_k) v_H. Pitch = local rx, lag = −local rz,
flap = −local ry (DESIGN.md).
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import scipy.linalg as sl

from . import fact
from .beam import (Element, FEModel, State, _edofs, assemble, assemble_internal,
                   assemble_Kmat, element_end_forces, element_end_forces_lin, element_point_maps,
                   skew, solve_static, unit, _logm)
from .bladegeom import airfoil_coords, resolve_stations
from .materials import get_material
from .schema import (BladeHarmonic, CampbellResult, Check, Design, FlexureMember, HarmonicResult,
                     Layup, Mode, PrintOrientation, Results, TrimResult)
from .section import (CircleShape, RectShape, SectionProps, airfoil_shape, section_properties)

G0 = 9.80665
import os as _os
_DEBUG = bool(_os.environ.get("FLEXBLADE_DEBUG"))
STIFF_FACTOR = 1.0e2


# =============================================================================== helpers


def Rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot_orient(o: PrintOrientation, R: np.ndarray) -> PrintOrientation:
    return PrintOrientation(build_dir=tuple(map(float, R @ np.asarray(o.build_dir))),
                            raster_dir=tuple(map(float, R @ np.asarray(o.raster_dir))))


def _ea_fracs(sec: SectionProps, layup: Layup) -> dict:
    """EA fractions per material (for complex modulus); approximate for composites."""
    if layup.kind == "solid":
        return {layup.material: 1.0}
    other = layup.skin_material if layup.kind == "core_skin" else layup.insert_material
    if not other:
        return {layup.material: 1.0}
    # estimate from section: rods/skin share by modulus × area proxy
    Eb = get_material(layup.material).E1
    Eo = get_material(other).E1
    if layup.kind == "rod_insert":
        Ao = len(layup.insert_positions) * np.pi * layup.insert_diameter**2 / 4
    else:
        # skin area ≈ perimeter × skin_thickness; perimeter from fibres span (rough)
        Ao = min(sec.area * 0.9, 2.2 * (np.sqrt(sec.area) * 4) * layup.skin_thickness)
    Ab = max(sec.area - Ao, 0.0)
    fo = Eo * Ao / max(Eo * Ao + Eb * Ab, 1e-30)
    return {layup.material: 1.0 - fo, other: fo}


def stiff_section(EA: float, EI: float, GJ: float) -> SectionProps:
    return SectionProps(EA=EA, EIy=EI, EIz=EI, EIyz=0.0, GJ=GJ, GAy=EA, GAz=EA, mass_per_len=0.0,
                        Ip_mass=0.0, Iyy_mass=0.0, Izz_mass=0.0, Iyz_mass=0.0, centroid=(0.0, 0.0),
                        shear_centre=(0.0, 0.0), mass_centre=(0.0, 0.0), area=0.0, method="stiff")


# =============================================================================== model


@dataclass
class BladeInfo:
    k: int
    psi0: float
    Rk: np.ndarray
    master: int
    chain_nodes: list[int]
    chain_r: np.ndarray
    aero_nodes: list[int]
    aero_r: np.ndarray
    blade_elems: list[int]
    member_elems: dict[str, list[int]]
    members: list[FlexureMember]
    hinge_point: np.ndarray
    hinge_dir: np.ndarray


@dataclass
class RotorModel:
    design: Design
    fe: FEModel
    hub: int
    blades: list[BladeInfo]
    r_aero: np.ndarray
    ea_frac: float
    st_r: np.ndarray
    st_chord: np.ndarray
    st_twist: np.ndarray
    warnings: list[str] = field(default_factory=list)

    @property
    def R(self) -> float:
        return self.design.blade.radius

    def chord(self, r):
        return np.interp(r, self.st_r, self.st_chord)

    def twist(self, r):
        return np.interp(r, self.st_r, self.st_twist)


def _blade_section(design: Design, r: float, st_r, stations, coll: float) -> tuple[SectionProps, float, float, Layup]:
    b = design.blade
    rr = np.clip(r, st_r[0], st_r[-1])
    chord = float(np.interp(rr, st_r, [s.chord for s in stations]))
    twist = float(np.interp(rr, st_r, np.radians([s.twist_deg for s in stations]))) + coll
    tcs = [s.thickness_frac for s in stations]
    if all(t is None for t in tcs):
        tc = None
    else:
        base = float(int(b.airfoil.naca[2:4]) / 100) if b.airfoil.kind == "naca4" else None
        vals = [t if t is not None else (base or 0.12) for t in tcs]
        tc = float(np.interp(rr, st_r, vals))
    i = int(np.argmin(np.abs(st_r - rr)))
    layup = stations[i].layup or b.layup
    xy = airfoil_coords(b.airfoil, tc, n=61)
    shape = airfoil_shape(xy, round(chord, 7), b.elastic_axis_chord_frac)
    # untwisted chord-frame section for blade 1; axes in H rotated by twist
    e2 = np.array([0.0, np.cos(twist), np.sin(twist)])
    e3 = np.array([0.0, -np.sin(twist), np.cos(twist)])
    sec = section_properties(shape, layup, b.orientation, (1.0, 0.0, 0.0), axes=(e2, e3))
    return sec, chord, twist, layup


def build_model(design: Design, ideal_hinge: dict | None = None) -> RotorModel:
    """Build the rotor FE model.

    ideal_hinge: optional dict(k_hinge, k_rot, k_trans) replacing the flexure by a
    zero-length spring at the hinge point (soft rotation k_hinge about the skewed hinge
    axis, k_rot about the other two axes, k_trans in translation) — the ideal skewed pin
    hinge reference (validation / Hakea as built).
    ideal_hinge may instead carry a full "K6" (6×6 at the hinge point, blade-1 frame H,
    [u; θ] → [f; m]) and optional "mass", "com", "inertia" (lumped on the blade side) — any
    flexure given its stiffness. Flexure.topology == "topopt" builds this from
    design.flexure.topopt."""
    if design.print_setup is not None:
        from .printsetup import apply_print_setup
        design = apply_print_setup(design)
    if ideal_hinge is None and design.flexure.topology == "topopt":
        tp = design.flexure.topopt
        if tp is None:
            raise ValueError("flexure.topology == 'topopt' needs flexure.topopt (run `flexblade topopt`)")
        ideal_hinge = dict(K6=tp.K6, mass=tp.mass, com=tp.com, inertia=tp.inertia)
    an = design.analysis
    b = design.blade
    R = b.radius
    B = b.n_blades
    fe = FEModel()
    hub = fe.add_node([0.0, 0.0, 0.0], "hub")
    fe.fix(hub, range(5))
    J = design.motor.J_hub + design.motor.J_rotor
    Mh = np.zeros((6, 6))
    Mh[5, 5] = J
    fe.add_lumped_inertia(hub, Mh)
    warnings: list[str] = []

    stations = resolve_stations(b)
    st_r = np.array([s.r_frac * R for s in stations])
    coll = np.radians(design.op.collective_deg)
    st_chord = np.array([s.chord for s in stations])
    st_twist = np.radians([s.twist_deg for s in stations]) + coll

    members1 = fact.generate_members(design) if ideal_hinge is None else []
    r_root = b.root_cutout_frac * R
    r_m = max(max(m.p0[0], m.p1[0]) for m in members1) if members1 else design.flexure.hinge_radius
    if r_m > r_root + 1e-9:
        warnings.append(f"flexure outer end r={r_m*1e3:.1f} mm ≥ aero root cut-out "
                        f"{r_root*1e3:.1f} mm; aero blade starts at the flexure end")
        r_root = r_m + 0.02 * (R - r_m)
    nbe = max(2, an.n_blade_elements)
    r_aero = np.linspace(r_root, R, nbe + 1)
    n_sh = int(np.clip(round((r_root - r_m) / (R - r_root) * nbe), 1, 4)) if r_root - r_m > 1e-6 else 0
    r_chain = np.r_[np.linspace(r_m, r_root, n_sh + 1)[:-1] if n_sh else [], r_aero]

    # blade sections (blade-1 frame), per element midpoint
    blade_secs = []
    for i in range(len(r_chain) - 1):
        rmid = 0.5 * (r_chain[i] + r_chain[i + 1])
        blade_secs.append(_blade_section(design, max(rmid, r_aero[0]) if rmid < r_aero[0] else rmid,
                                         st_r, stations, coll))

    # flexure member sections (computed per blade below, cached) — gather stiffness scale
    def member_sections(m: FlexureMember, Rk: np.ndarray, xi0: float, xi1: float, e1, e2, e3):
        if m.kind == "rigid":
            return None, None, None
        ori = _rot_orient(m.orientation, Rk)
        def sec_at(xi):
            t = m.thickness * (1 + (m.taper - 1) * (xi0 + xi * (xi1 - xi0)))
            if m.shape == "circle":
                shp = CircleShape(round(t, 9))
            else:
                shp = RectShape(round(m.width, 9), round(t, 9))
            return section_properties(shp, m.layup, ori, e1, axes=(e2, e3))
        if m.taper == 1.0 or m.layup.kind != "solid":
            s = sec_at(0.5)
            return [s], None, _ea_fracs(s, m.layup)
        from .beam import GAUSS3, GAUSS4
        return [sec_at(x) for x in GAUSS3[0]], [sec_at(x) for x in GAUSS4[0]], _ea_fracs(sec_at(0.5), m.layup)

    max_EA = max(s[0].EA for s in blade_secs)
    max_EI = max(max(s[0].EIy, s[0].EIz) for s in blade_secs)
    max_GJ = max(s[0].GJ for s in blade_secs)

    blades: list[BladeInfo] = []
    full_springs: list = []
    pending_stiff: list[tuple[int, int, np.ndarray, str, int]] = []
    nfe = max(1, an.n_flexure_elements)
    for k in range(1, B + 1):
        psi0 = 2 * np.pi * (k - 1) / B
        Rk = Rz(psi0)
        mem_local = [fact.mirror_member(m) if k % 2 == 0 else m for m in members1]
        mem_H = []
        for m in mem_local:
            mem_H.append(m.model_copy(update=dict(
                p0=tuple(map(float, Rk @ np.asarray(m.p0))), p1=tuple(map(float, Rk @ np.asarray(m.p1))),
                width_dir=tuple(map(float, Rk @ np.asarray(m.width_dir))))))
        # blade chain nodes
        chain = [fe.add_node(Rk @ np.array([r, 0.0, 0.0]), f"b{k}.r{r*1e3:.1f}") for r in r_chain]
        master = chain[0]
        blade_elems = []
        for i in range(len(r_chain) - 1):
            sec, chord, twist, layup = blade_secs[i]
            e2 = Rk @ np.array([0.0, np.cos(twist), np.sin(twist)])
            kind = "shank" if i < n_sh else "blade"
            el = Element(chain[i], chain[i + 1], e2, [sec], material_fracs=_ea_fracs(sec, layup),
                         tag=f"b{k}.{kind}", kind=kind, blade=k)
            blade_elems.append(fe.add_element(el))
        member_elems: dict[str, list[int]] = {}
        for j, m in enumerate(mem_H):
            p0, p1 = np.asarray(m.p0), np.asarray(m.p1)
            tag = f"b{k}.flex{j}" if m.kind != "rigid" else f"b{k}.rigid{j}"
            e1 = unit(p1 - p0)
            wd = np.asarray(m.width_dir, float)
            e2 = wd - e1 * (e1 @ wd)
            if np.linalg.norm(e2) < 1e-9:
                e2 = np.cross([0.0, 0.0, 1.0], e1) if abs(e1[2]) < 0.9 else np.cross([0.0, 1.0, 0.0], e1)
            e2 = unit(e2)
            e3 = np.cross(e1, e2)
            n_el = nfe if m.kind != "rigid" else 1
            nodes = [fe.add_node(p0 + (p1 - p0) * i / n_el, f"{tag}.n{i}") for i in range(n_el + 1)]
            ids = []
            for i in range(n_el):
                secs, msecs, fr = member_sections(m, Rk, i / n_el, (i + 1) / n_el, e1, e2, e3)
                if secs is None:
                    pending_stiff.append((nodes[i], nodes[i + 1], e2, tag, k))
                    continue
                max_EA = max(max_EA, max(s.EA for s in secs))
                max_EI = max(max_EI, max(max(s.EIy, s.EIz) for s in secs))
                max_GJ = max(max_GJ, max(s.GJ for s in secs))
                ids.append(fe.add_element(Element(nodes[i], nodes[i + 1], e2, secs, msecs, fr, tag=tag,
                                                  kind="flexure", blade=k)))
            member_elems[tag] = ids
            # spokes and connectors (stiff, massless)
            if np.linalg.norm(p0) > 1e-9:
                pending_stiff.append((hub, nodes[0], np.array([0.0, 0.0, 1.0]), f"b{k}.spoke{j}", k))
            else:
                # member starts at the hub centre: merge by a very short stiff link
                pending_stiff.append((hub, nodes[0], np.array([0.0, 0.0, 1.0]), f"b{k}.spoke{j}", k))
            if np.linalg.norm(p1 - fe.X0[master]) > 1e-9:
                pending_stiff.append((nodes[-1], master, np.array([0.0, 0.0, 1.0]), f"b{k}.conn{j}", k))
            else:
                pending_stiff.append((nodes[-1], master, None, f"b{k}.conn{j}", k))
        H0, a = fact.hinge_axis(design, k)
        if ideal_hinge is not None:
            A = fe.add_node(Rk @ H0, f"b{k}.hingeA")
            pending_stiff.append((hub, A, np.array([0.0, 0.0, 1.0]), f"b{k}.spoke0", k))
            kh = ideal_hinge
            if "K6" in kh:
                # full 6×6 in blade-1 frame H; blade k even: mirror z → −z (translations
                # diag(1,1,−1), rotations (pseudo-vector) diag(−1,−1,1)); then rotate by Rk
                D = np.diag([1.0, 1, -1, -1, -1, 1]) if k % 2 == 0 else np.eye(6)
                Ks = D @ np.asarray(kh["K6"], float) @ D
                Ks = 0.5 * (Ks + Ks.T)
                frame = np.column_stack([Rk[:, 0], Rk[:, 1], Rk[:, 2]])
            else:
                Ks = np.diag([kh.get("k_trans", 1e7)] * 3 + [kh["k_hinge"], kh.get("k_rot", 1e4), kh.get("k_rot", 1e4)])
                frame = np.column_stack([Rk @ a, Rk @ np.array([0.0, 1.0, 0.0]), np.zeros(3)])
            B_node = master
            if np.linalg.norm(fe.X0[master] - Rk @ H0) > 1e-9:
                B_node = fe.add_node(Rk @ H0, f"b{k}.hingeB")
                pending_stiff.append((B_node, master, np.array([0.0, 0.0, 1.0]), f"b{k}.conn0", k))
            ie = fe.add_element(Element(A, B_node, np.zeros(3), [], tag=f"b{k}.hinge", kind="spring",
                                        blade=k, spring_K=Ks, spring_frame=frame))
            if "K6" in kh:
                full_springs.append((k, ie, Ks.copy()))
            if kh.get("mass", 0.0) > 0.0:
                Mt = np.diag([1.0, 1, -1]) if k % 2 == 0 else np.eye(3)
                c = Rk @ (Mt @ np.asarray(kh.get("com", H0), float)) - fe.X0[master]
                Ic = Rk @ Mt @ np.asarray(kh.get("inertia", np.zeros((3, 3))), float) @ Mt @ Rk.T
                m_ = float(kh["mass"])
                S = skew(c)
                Mn = np.zeros((6, 6))
                Mn[:3, :3] = m_ * np.eye(3)
                Mn[:3, 3:] = -m_ * S
                Mn[3:, :3] = m_ * S
                Mn[3:, 3:] = Ic + m_ * ((c @ c) * np.eye(3) - np.outer(c, c))
                fe.add_lumped_inertia(master, Mn)
        blades.append(BladeInfo(k=k, psi0=psi0, Rk=Rk, master=master, chain_nodes=chain,
                                chain_r=r_chain, aero_nodes=chain[n_sh:], aero_r=r_aero,
                                blade_elems=blade_elems, member_elems=member_elems, members=mem_H,
                                hinge_point=Rk @ H0, hinge_dir=Rk @ a))
    stiff = stiff_section(STIFF_FACTOR * max_EA, STIFF_FACTOR * max_EI, STIFF_FACTOR * max_GJ)
    merges: list[tuple[int, int]] = []
    for n0, n1, e2, tag, k in pending_stiff:
        if e2 is None or np.linalg.norm(fe.X0[n1] - fe.X0[n0]) < 1e-9:
            merges.append((n0, n1))
            continue
        fe.add_element(Element(n0, n1, e2, [stiff], tag=tag, kind="rigid", blade=k, massless=True))
    if merges:
        _merge_nodes(fe, merges, blades)
    fe.prepare()
    model = RotorModel(design, fe, hub, blades, r_aero, b.elastic_axis_chord_frac,
                       st_r, st_chord, st_twist, warnings)
    if full_springs:
        _compensate_links(model, full_springs)
    return model


def _compensate_links(model: RotorModel, springs) -> None:
    """A full-6×6 spring (ideal_hinge["K6"]) sits in series with the stiff spoke from the
    hub node to the hinge point (STIFF_FACTOR × the stiffest section, not rigid): ~5–10 %
    extra compliance on a stiff flexure. Remove it so flexure_stiffness() at the hinge point
    returns the stored K6: C_spring = C_target − C_link, C_link = C_measured − C_spring (one
    linear pass, exact since the link compliance does not depend on the spring)."""
    fe = model.fe
    for k, ie, Ks0 in springs:
        el = fe.elements[ie]
        Q = np.zeros((6, 6))
        Q[:3, :3] = Q[3:, 3:] = el.E0
        Kt = Q @ Ks0 @ Q.T
        Km = flexure_stiffness(model, blade=k)
        C_link = np.linalg.inv(Km) - np.linalg.inv(Kt)
        Cs = np.linalg.inv(Kt) - C_link
        Cs = 0.5 * (Cs + Cs.T)
        if np.linalg.eigvalsh(Cs).min() <= 0:
            model.warnings.append(f"blade {k}: 6×6 spring stiffer than its stiff links; not compensated")
            continue
        Kn = Q.T @ np.linalg.inv(Cs) @ Q
        Kn = 0.5 * (Kn + Kn.T)
        el.spring_K = Kn
        el.K7[1:, 1:] = Kn
    fe._pack = None                     # batched-kernel cache holds the element K7s


def _merge_nodes(fe: FEModel, merges, blades) -> None:
    """Replace node n1 by n0 everywhere (coincident rigid joints)."""
    remap = {}
    for n0, n1 in merges:
        while n0 in remap:
            n0 = remap[n0]
        remap[n1] = n0
    def f(n):
        while n in remap:
            n = remap[n]
        return n
    for e in fe.elements:
        e.n0, e.n1 = f(e.n0), f(e.n1)
    fe.lumped = [(f(n), Mn) for n, Mn in fe.lumped]
    for bi in blades:
        bi.master = f(bi.master)
        bi.chain_nodes = [f(n) for n in bi.chain_nodes]
        bi.aero_nodes = [f(n) for n in bi.aero_nodes]
    # dangling nodes keep zero stiffness: fix them
    used = {e.n0 for e in fe.elements} | {e.n1 for e in fe.elements} | {0}
    for n in range(fe.nn):
        if n not in used:
            fe.fix(n)


# =============================================================================== flexure-only model (FACT check / 6×6)


class FlexureOnly:
    def __init__(self, fe: FEModel, master: int, clamp: list[int]):
        self.fe, self.master, self.clamp = fe, master, clamp

    def stiffness_at(self, point, state: State | None = None, extra_K: np.ndarray | None = None) -> np.ndarray:
        fe = self.fe
        st = state or fe.initial_state()
        sa = assemble_internal(fe, st, True)
        K = assemble_Kmat(fe, sa) + sa.Kgeo
        if extra_K is not None:
            K = K + extra_K
        return condense_to_point(fe, K, self.master, self.clamp, st.x[self.master], point)


def condense_to_point(fe: FEModel, K: np.ndarray, master: int, clamp: list[int], x_master, point) -> np.ndarray:
    fixed = set()
    for n in clamp:
        fixed.update(range(6 * n, 6 * n + 6))
    fixed |= {d for d in fe.fixed if d // 6 not in clamp}
    keep = list(range(6 * master, 6 * master + 6))
    others = [i for i in range(fe.ndof) if i not in fixed and i not in keep]
    Kmm = K[np.ix_(keep, keep)]
    Kmo = K[np.ix_(keep, others)]
    Koo = K[np.ix_(others, others)]
    Kc = Kmm - Kmo @ np.linalg.solve(Koo, Kmo.T)
    d = np.asarray(x_master) - np.asarray(point)
    T = np.eye(6)
    T[:3, 3:] = -skew(d)      # u_m = u0 + θ × d = u0 − d × θ
    Kp = T.T @ Kc @ T
    return 0.5 * (Kp + Kp.T)


def flexure_only_model(design: Design, members: list[FlexureMember]) -> FlexureOnly:
    """Members (blade-1 frame) clamped at the hub ends; blade ends tied by stiff links to
    a master node at the outermost member end."""
    d2 = design.model_copy(deep=True)
    d2.flexure.topology = "custom"
    d2.flexure.members = list(members)
    d2.flexure.tension_member = None
    d2.blade.n_blades = 1
    m = build_model(d2)
    bi = m.blades[0]
    # remove blade chain elements (keep only flexure + stiff links)
    keep = [e for i, e in enumerate(m.fe.elements) if i not in bi.blade_elems]
    m.fe.elements = keep
    m.fe.fix(m.hub)
    for n in bi.chain_nodes[1:]:
        m.fe.fix(n)
    m.fe.prepare()
    return FlexureOnly(m.fe, bi.master, [m.hub])


# =============================================================================== aero interface


def _aero_module():
    try:
        from . import aero
        return aero
    except Exception:   # pragma: no cover
        return None


def make_aero(model: RotorModel):
    am = _aero_module()
    if am is None:
        return None
    return am.AeroModel(model.design, r=model.r_aero)


def _aero_points(model: RotorModel, bi: BladeInfo):
    """Gauss points over the aero blade elements: (elem, xi, w·l, r, qc_local(y,z), interp idx/weight)."""
    fe = model.fe
    out = []
    gx = np.array([0.5 - 0.5 / np.sqrt(3), 0.5 + 0.5 / np.sqrt(3)])
    n_sh = len(bi.chain_r) - len(bi.aero_r)
    for i, ei in enumerate(bi.blade_elems):
        if i < n_sh:
            continue
        ia = i - n_sh
        e = fe.elements[ei]
        ra, rb = bi.aero_r[ia], bi.aero_r[ia + 1]
        for xi in gx:
            r = ra + xi * (rb - ra)
            c = float(model.chord(r))
            y_qc = (model.ea_frac - 0.25) * c
            out.append((e, xi, 0.5 * (rb - ra), r, (y_qc, 0.0), ia, xi))
    return out


def _aero_maps(model: RotorModel, st: State):
    """element_point_maps at every aero point of every blade, in _aero_points order
    (one batched numba call on the fast path)."""
    from . import beam as _beam
    pts = [p for bi in model.blades for p in _aero_points(model, bi)]
    if not _beam._FAST:
        return iter([element_point_maps(p[0], st, p[1], p[4]) for p in pts])
    from .beam_fast import element_point_maps_batch
    idx = {id(e): k for k, e in enumerate(model.fe.elements)}
    return zip(*element_point_maps_batch(model.fe, st, [idx[id(p[0])] for p in pts],
                                         [p[1] for p in pts], [p[4] for p in pts]))


def _blade_proj(bi: BladeInfo, L: np.ndarray, Nr: np.ndarray) -> np.ndarray:
    """3×12: [uz, uy, θ] blade-local from element DOFs."""
    Lb = bi.Rk.T @ L
    Nb = bi.Rk.T @ Nr
    return np.vstack([Lb[2], Lb[1], Nb[0]])


def aero_load_vector(model: RotorModel, st: State, trim) -> np.ndarray:
    """Dead aero loads from AeroTrim (per-unit-span Fx, Fy, Fz, Mx at quarter chord)."""
    F = np.zeros(model.fe.ndof)
    maps = _aero_maps(model, st)
    for bi in model.blades:
        for e, xi, wl, r, qc, ia, s in _aero_points(model, bi):
            fz = (1 - s) * trim.Fz[ia] + s * trim.Fz[ia + 1]
            fy = (1 - s) * trim.Fy[ia] + s * trim.Fy[ia + 1]
            fx = (1 - s) * trim.Fx[ia] + s * trim.Fx[ia + 1]
            mx = (1 - s) * trim.Mx[ia] + s * trim.Mx[ia + 1]
            rr, sH, L, Nr = next(maps)
            fH = bi.Rk @ np.array([fx, fy, fz])
            mH = bi.Rk @ np.array([mx, 0.0, 0.0])
            F[_edofs(e)] += wl * (L.T @ fH + Nr.T @ mH)
    return F


def _dlog_left(phi: np.ndarray) -> np.ndarray:
    """d log(exp(ω)R)/dω at ω = 0 for log R = φ (inverse left Jacobian of SO(3))."""
    th = float(np.linalg.norm(phi))
    P = skew(phi)
    if th < 1e-6:
        return np.eye(3) - 0.5 * P + P @ P / 12.0
    return np.eye(3) - 0.5 * P + (1.0 / th**2 - (1 + np.cos(th)) / (2 * th * np.sin(th))) * P @ P


def aero_shape_sens(model: RotorModel, aero, rpm: float, shape: dict, h: float = 1e-4):
    """Station-wise d[Fx, Fy, Fz, Mx]/dθ̄ and /dβ̄ of aero.trim(rpm, blade_shape) (BEM
    inflow re-solved — stations are independent in the BEM, so one central difference
    of all stations at once gives the diagonal).  None if the aero grid ≠ r_aero."""
    if len(aero.r) != len(model.r_aero) or not np.allclose(aero.r, model.r_aero, rtol=0, atol=1e-12):
        return None
    out = {}
    for key in ("theta", "beta"):
        sp, sm = dict(shape), dict(shape)
        sp[key] = np.asarray(shape[key], float) + h
        sm[key] = np.asarray(shape[key], float) - h
        tp, tm = aero.trim(rpm=rpm, blade_shape=sp), aero.trim(rpm=rpm, blade_shape=sm)
        out[key] = np.array([(np.asarray(getattr(tp, f)) - np.asarray(getattr(tm, f))) / (2 * h)
                             for f in ("Fx", "Fy", "Fz", "Mx")])        # (4, n_st)
    return out


def aero_shape_tangent(model: RotorModel, st: State, sens: dict) -> np.ndarray:
    """Consistent d(aero_load_vector(model, s, aero.trim(rpm, mean blade shape(s))))/dq
    through the blade-mean elastic pitch θ̄ and flap slope β̄ (load geometry frozen).
    Replaces the per-blade quasi-steady Ka in the trim Newton: the trim loads see only
    the blade-MEAN shape and re-solve the BEM inflow, so Ka was inconsistent (linear
    convergence, and wrong stiffness on the soft antisymmetric pitch mode)."""
    n = model.fe.ndof
    nst = len(model.r_aero)
    B = len(model.blades)
    X0 = model.fe.Xref
    Ct = np.zeros((nst, n))        # dθ̄_i/dq
    Cb = np.zeros((nst, n))        # dβ̄_i/dq
    for bi in model.blades:
        G = np.gradient(np.eye(len(bi.aero_r)), bi.aero_r, axis=0)
        for i, nd in enumerate(bi.aero_nodes):
            Ct[i, 6 * nd + 3:6 * nd + 6] += (bi.Rk.T @ _dlog_left(_logm(st.R[nd])))[0] / B
            Cb[:, 6 * nd:6 * nd + 3] += np.outer(G[:, i], bi.Rk[:, 2]) / B
    At = np.zeros((n, nst))
    Ab = np.zeros((n, nst))
    maps = _aero_maps(model, st)
    for bi in model.blades:
        for e, xi, wl, r, qc, ia, s in _aero_points(model, bi):
            rr, sH, L, Nr = next(maps)
            d = _edofs(e)
            for A, dF in ((At, sens["theta"]), (Ab, sens["beta"])):
                for j, wj in ((ia, 1 - s), (ia + 1, s)):
                    fH = bi.Rk @ dF[:3, j]
                    mH = bi.Rk[:, 0] * dF[3, j]
                    A[d, j] += wj * wl * (L.T @ fH + Nr.T @ mH)
    return At @ Ct + Ab @ Cb


@dataclass
class AeroLinear:
    Ka: np.ndarray           # n×n complex: generalised aero force = Ka u (+ b X)
    b: np.ndarray            # n complex: force per unit inflow amplitude X
    c: np.ndarray            # n complex: Σ e^{-iψ0}∫ r dFz from u
    c0: complex              # same from X
    kappa: complex           # X = kappa * (c u + c0 X)
    Fz_rows: list            # per blade: (n_pts) list of (row over u, coeff on X, w·l) for thrust


def aero_linear(model: RotorModel, st: State, aero, trim, omega: float, dynamic_inflow: bool) -> AeroLinear:
    n = model.fe.ndof
    A = np.stack(aero.linearized(trim, omega))     # (ns,3,4)
    Ka = np.zeros((n, n), complex)
    bvec = np.zeros(n, complex)
    cvec = np.zeros(n, complex)
    c0 = 0.0 + 0j
    Fz_rows = []
    B = len(model.blades)
    maps = _aero_maps(model, st)
    for bi in model.blades:
        ph = np.exp(1j * bi.psi0)
        row = np.zeros(n, complex)
        rowX = 0.0 + 0j
        for e, xi, wl, r, qc, ia, s in _aero_points(model, bi):
            Ai = (1 - s) * A[ia] + s * A[ia + 1]
            rr, sH, L, Nr = next(maps)
            P = _blade_proj(bi, L, Nr)
            d = _edofs(e)
            Ka[np.ix_(d, d)] += wl * P.T @ Ai[:, :3] @ P
            dl = r * ph                                  # dλ_k(r) = X r e^{iψ0_k}
            bvec[d] += wl * P.T @ Ai[:, 3] * dl
            # thrust (dFz) linear forms
            fz_u = wl * Ai[0, :3] @ P
            row[d] += fz_u
            rowX += wl * Ai[0, 3] * dl
            cvec[d] += np.conj(ph) * r * fz_u
            c0 += np.conj(ph) * r * wl * Ai[0, 3] * dl
        Fz_rows.append((row, rowX))
    kappa = 0.0
    if dynamic_inflow and B >= 2:
        kappa = _pp_gain(aero, trim, B)
    return AeroLinear(Ka, bvec, cvec, c0, kappa, Fz_rows)


def _pp_gain(aero, trim, B: int) -> complex:
    """κ with X = Ω(λc − iλs) = κ Σ_k e^{−iψ0_k} ∫ r dFz_k dr (steady Pitt–Peters),
    derived numerically from aero.mean_hub_moment / cyclic_inflow / blade_1p so the
    sign conventions are the aero module's."""
    dF = np.zeros((B, len(trim.r)), complex)
    # unit ∫ r F dr on blade 1: put F such that Σ w r F = 1
    F1 = np.zeros(len(trim.r))
    F1[-1] = 1.0 / (trim.w_int[-1] * trim.r[-1])
    dF[0] = F1
    Mx, My = aero.mean_hub_moment(trim, dF)
    ci = aero.cyclic_inflow(trim, Mx, My)
    X1 = complex(ci.blade_1p(1.0, 1, B))
    dF[0] = 1j * F1
    Mx, My = aero.mean_hub_moment(trim, dF)
    Xi = complex(aero.cyclic_inflow(trim, Mx, My).blade_1p(1.0, 1, B))
    if abs(Xi - 1j * X1) > 1e-6 * max(abs(X1), 1e-30):   # pragma: no cover
        raise RuntimeError("Pitt–Peters mapping is not complex-linear (non-isotropic inflow)")
    return X1


# =============================================================================== static trim


@dataclass
class StaticState:
    state: State
    Omega: float
    rpm: float
    trim: object = None
    F_aero: Optional[np.ndarray] = None
    converged: bool = True
    iterations: int = 0
    warnings: list[str] = field(default_factory=list)
    blade_pitch_el: Optional[np.ndarray] = None     # (B, n_aero) elastic pitch, rad
    blade_beta: Optional[np.ndarray] = None


def blade_local_motion(model: RotorModel, st: State, bi: BladeInfo, nodes=None):
    """(u_local (n,3), rotvec_local (n,3)) for blade-chain nodes."""
    X0 = model.fe.Xref
    nodes = bi.chain_nodes if nodes is None else nodes
    u = np.array([bi.Rk.T @ (st.x[n] - X0[n]) for n in nodes])
    rv = np.array([bi.Rk.T @ _logm(st.R[n]) for n in nodes])
    return u, rv


def _blade_shape(model: RotorModel, st: State):
    th, be = [], []
    for bi in model.blades:
        u, rv = blade_local_motion(model, st, bi, bi.aero_nodes)
        th.append(rv[:, 0])
        be.append(np.gradient(u[:, 2], bi.aero_r))
    return np.array(th), np.array(be)


def static_trim(model: RotorModel, aero=None, rpm: float | None = None, max_outer: int = 6) -> StaticState:
    """Nonlinear aeroelastic hover trim.

    Inner: at fixed rpm, Newton on the structure with aero loads that follow the
    deformed blade (elastic pitch θ_el(r) and flap slope from the current state fed to
    aero.trim(rpm, blade_shape)) and the quasi-steady aero pitch stiffness in the tangent.
    Outer: secant on rpm for thrust = vehicle weight (unless op.rpm is given).
    The hub angle is held (the motor's mean torque balances drag)."""
    d = model.design
    fe = model.fe
    warn: list[str] = []
    use_aero_loads = aero is not None and d.analysis.aero != "none"
    hubfix = (6 * model.hub + 5,)
    if aero is None:
        rpm = rpm if rpm is not None else d.op.rpm
        if rpm is None:
            raise ValueError("no aero model: op.rpm must be given")
        Om = rpm * 2 * np.pi / 60
        st, info = solve_static(fe, Om, None, n_steps=2, extra_fixed=hubfix)
        w = [] if info.converged else [f"nonlinear static did not converge: {info.message}"]
        th, be = _blade_shape(model, st)
        return StaticState(st, Om, rpm, None, None, info.converged, info.iterations, w, th, be)
    trim0 = aero.trim(rpm=rpm) if rpm is not None else aero.trim()
    fixed_rpm = rpm is not None or d.op.rpm is not None
    W = trim0.T if fixed_rpm else d.op.vehicle_mass * G0
    if not fixed_rpm:
        W = float(trim0.T)     # aero.trim() trims to its own thrust target (incl. thrust share)

    def shape_of(s):
        th, be = _blade_shape(model, s)
        return dict(theta=th.mean(0), beta=be.mean(0), r=model.r_aero), th, be

    cache = {}

    def aero_at(s, rpm_x):
        shp, _, _ = shape_of(s)
        return aero.trim(rpm=rpm_x, blade_shape=shp)

    def solve_at(rpm_x, st0):
        Om = rpm_x * 2 * np.pi / 60
        if not use_aero_loads:
            s1, inf = solve_static(fe, Om, None, state0=st0, n_steps=2, extra_fixed=hubfix)
            return s1, inf, aero_at(s1, rpm_x)

        def fext(s):
            tr = aero_at(s, rpm_x)
            cache["tr"] = tr
            return aero_load_vector(model, s, tr)

        def ktan(s):
            if cache.get("sens_rpm") != rpm_x:      # sensitivities: once per rpm
                cache["sens"] = aero_shape_sens(model, aero, rpm_x, shape_of(s)[0])
                cache["sens_rpm"] = rpm_x
            if cache["sens"] is None:
                tr = cache.get("tr") or aero_at(s, rpm_x)
                return np.real(aero_linear(model, s, aero, tr, 0.0, False).Ka)
            return aero_shape_tangent(model, s, cache["sens"])

        its0 = 0
        if st0 is None:   # cold start: centrifugal only, then dead rigid-blade aero loads
            st0, inf0 = solve_static(fe, Om, None, n_steps=2, extra_fixed=hubfix)
            its0 = inf0.iterations
            st0, inf0 = solve_static(fe, Om, aero_load_vector(model, st0, aero_at(fe.initial_state(), rpm_x)),
                                     state0=st0, extra_fixed=hubfix)
            its0 += inf0.iterations
        s1, inf = solve_static(fe, Om, fext, state0=st0, extra_fixed=hubfix, ext_tangent=ktan)
        inf.iterations += its0
        return s1, inf, aero_at(s1, rpm_x)

    x = float(trim0.rpm)
    st, info, tr = solve_at(x, None)
    its, conv = info.iterations, info.converged
    xs, fs = [x], [tr.T - W]
    ok_outer = fixed_rpm
    if not fixed_rpm and conv:
        for k in range(max_outer):
            if abs(fs[-1]) < 1e-3 * W:
                ok_outer = True
                break
            if len(xs) == 1:
                xn = x * np.sqrt(W / max(tr.T, 1e-9))
            else:
                xn = xs[-1] - fs[-1] * (xs[-1] - xs[-2]) / (fs[-1] - fs[-2])
            x = float(np.clip(xn, 0.7 * xs[-1], 1.4 * xs[-1]))
            st, info, tr = solve_at(x, st)
            its += info.iterations
            conv = conv and info.converged
            if not info.converged:
                xs.append(x)
                fs.append(tr.T - W)
                break
            xs.append(x)
            fs.append(tr.T - W)
            if _DEBUG:
                print(f"trim outer: rpm {x:.1f} T-W {fs[-1]:+.4f} N  newton its {info.iterations} {info.message}")
        else:
            ok_outer = abs(fs[-1]) < 1e-3 * W
    if not ok_outer:
        warn.append(f"trim rpm iteration not converged (T−W = {fs[-1]:+.3f} N)")
    if not conv:
        warn.append(f"nonlinear static did not converge: {info.message}")
    _, th, be = shape_of(st)
    Om = x * 2 * np.pi / 60
    F = aero_load_vector(model, st, tr) if use_aero_loads else None
    return StaticState(st, Om, x, tr, F, conv, its, warn, th, be)


# =============================================================================== linearisation


@dataclass
class Linear:
    sa: object
    ia: object
    free: np.ndarray
    Kgeo: np.ndarray
    Kcf: np.ndarray
    M: np.ndarray
    G: np.ndarray


def linearize(model: RotorModel, ss: StaticState, coriolis: bool = True) -> Linear:
    fe = model.fe
    sa = assemble(fe, ss.state, ss.Omega, True, True)
    G = sa.G if coriolis else np.zeros_like(sa.G)
    M = sa.M.copy()
    # regularise massless DOFs (stiff links); modes use shift-invert so this is benign
    dM = np.diag(M).copy()
    eps = 1e-9 * max(dM.max(), 1e-30)
    M[np.diag_indices_from(M)] += np.where(dM < eps, eps, 0.0)
    return Linear(sa, None, fe.free_dofs(), sa.Kgeo, np.zeros_like(sa.Kgeo), M, G)


def K_complex(model: RotorModel, lin: Linear, freq_hz: float, real_only: bool = False) -> np.ndarray:
    scales = np.array([e.complex_scale(freq_hz) for e in model.fe.elements])
    if real_only:
        scales = scales.real
    return assemble_Kmat(model.fe, lin.sa, scales) + lin.Kgeo + lin.Kcf


# =============================================================================== modes


def _labels(model: RotorModel, shape_full: np.ndarray, Mfull: np.ndarray) -> tuple[str, dict]:
    """Kinetic-energy partition of a (complex) mode over blade-local motion classes."""
    fe = model.fe
    v = shape_full
    parts = {"hub": 0.0}
    per_blade = []
    hubd = 6 * model.hub + 5
    parts["hub"] = float(np.real(np.conj(v[hubd]) * Mfull[hubd, hubd] * v[hubd]))
    for bi in model.blades:
        nodes = bi.chain_nodes
        e_lag = e_flap = e_pitch = e_ax = 0.0
        sgn = {}
        for n in nodes:
            u = bi.Rk.T @ v[6 * n:6 * n + 3]
            rv = bi.Rk.T @ v[6 * n + 3:6 * n + 6]
            idx = list(range(6 * n, 6 * n + 6))
            m = Mfull[idx[0], idx[0]]
            e_ax += m * abs(u[0]) ** 2
            e_lag += m * abs(u[1]) ** 2
            e_flap += m * abs(u[2]) ** 2
            Ip = Mfull[idx[3], idx[3]]
            e_pitch += Ip * abs(rv[0]) ** 2
        tip = nodes[-1]
        ut = bi.Rk.T @ v[6 * tip:6 * tip + 3]
        rt = bi.Rk.T @ v[6 * tip + 3:6 * tip + 6]
        sgn = dict(lag=ut[1], flap=ut[2], pitch=rt[0], axial=ut[0])
        per_blade.append((dict(lag=e_lag, flap=e_flap, pitch=e_pitch, axial=e_ax), sgn))
    tot = {k: sum(pb[0][k] for pb in per_blade) for k in ("lag", "flap", "pitch", "axial")}
    tot["hub"] = parts["hub"]
    # pitch energy is small (small inertia) – weight it up to compare as "motion"
    s = sum(tot.values()) or 1.0
    part = {k: float(v_ / s) for k, v_ in tot.items()}
    w = dict(part)
    w["pitch"] *= 30.0
    dom = max(("lag", "flap", "pitch", "axial"), key=lambda k: w[k])
    lab = dom
    if len(per_blade) >= 2:
        a, b2 = per_blade[0][1][dom], per_blade[1][1][dom]
        if dom == "pitch":
            # pitch sign flips by mirror: compare lag-coupled sense via lag where available
            pass
        ratio = np.real(a * np.conj(b2))
        lab += " sym" if ratio > 0 else " antisym"
    if part["hub"] > 0.15:
        lab += "+hub"
    return lab, part


@dataclass
class ModeData:
    freqs_hz: np.ndarray
    damping: np.ndarray
    shapes: np.ndarray            # (nm, ndof) complex
    labels: list[str]
    parts: list[dict]


def modes(model: RotorModel, ss: StaticState, aero=None, n_modes: int | None = None,
          coriolis: bool = True, lin: Linear | None = None, n_basis: int = 40) -> ModeData:
    fe = model.fe
    lin = lin or linearize(model, ss, coriolis)
    f = lin.free
    n_modes = n_modes or model.design.analysis.n_modes
    f_ref = max(ss.Omega / (2 * np.pi), 1.0)
    K = K_complex(model, lin, f_ref)
    Kr = np.real(K)[np.ix_(f, f)]
    Kr = 0.5 * (Kr + Kr.T)
    M = lin.M[np.ix_(f, f)]
    nb = min(n_basis, len(f))
    # shift-invert Lanczos: accurate low modes despite the huge stiffness/mass range
    # (stiff links, near-massless rotational DOFs) that ruins Cholesky-based eigh
    w_ref = max(ss.Omega, 2 * np.pi * 10.0)
    from scipy.sparse.linalg import eigsh
    try:
        lam, Phi = eigsh(Kr, k=min(nb, len(f) - 2), M=M, sigma=-(0.05 * w_ref) ** 2, which="LM")
    except Exception:   # pragma: no cover
        lam, Phi = sl.eigh(Kr, M, subset_by_index=[0, nb - 1])
    o = np.argsort(lam)
    lam, Phi = lam[o], Phi[:, o]
    # drop basis vectors far above the band of interest (regularised massless DOFs)
    ok = np.sqrt(np.abs(lam)) < 200.0 * w_ref
    ok[: min(len(ok), n_modes + 2)] = True
    lam, Phi = lam[ok], Phi[:, ok]
    nb = len(lam)
    G = lin.G[np.ix_(f, f)]
    Gr = Phi.T @ G @ Phi
    # scaled state space  y = [q; q̇/w_ref]:  ẏ = w_ref [[0, I], [−Λ/w², −Gr/w]] y
    Aq = w_ref * np.block([[np.zeros((nb, nb)), np.eye(nb)],
                           [-np.diag(lam) / w_ref**2, -Gr / w_ref]])
    ev, EV = np.linalg.eig(Aq)
    sel = [i for i in range(len(ev)) if ev[i].imag > 1e-6 * max(1.0, abs(ev[i])) or
           (abs(ev[i].imag) <= 1e-6 * max(1.0, abs(ev[i])) and ev[i].real >= -1e-9 and abs(ev[i]) >= 0)]
    # keep one of each conjugate pair (Im>0) plus real/zero roots once
    seen, keep = [], []
    for i in np.argsort(np.abs(ev)):
        z = ev[i]
        if z.imag < -1e-9 * max(1.0, abs(z)):
            continue
        tolz = 2e-2 * w_ref
        if abs(z) < tolz and any(abs(s_) < tolz for s_ in seen):
            continue          # rigid-spin double root: keep one
        if any(abs(z - s_) < 1e-9 * max(1.0, abs(z)) for s_ in seen):
            continue
        seen.append(z)
        keep.append(i)
    del sel
    out_f, out_z, shapes, labels, parts = [], [], [], [], []
    for i in keep[:n_modes]:
        z = ev[i]
        w = abs(z.imag)
        q = EV[:nb, i]
        x = np.zeros(fe.ndof, complex)
        x[f] = Phi @ q
        k = np.argmax(np.abs(x))
        x = x * np.exp(-1j * np.angle(x[k])) / max(abs(x[k]), 1e-30)
        # damping: structural loss (+ aero) by energy at this frequency
        fz = max(w / (2 * np.pi), 1e-3)
        Kc = K_complex(model, lin, fz)
        Z = Kc.copy()
        if aero is not None and ss.trim is not None and model.design.analysis.aero != "none" and w > 0:
            al = aero_linear(model, ss.state, aero, ss.trim, w, False)
            Z = Z - al.Ka
        num = np.conj(x) @ Z @ x
        den = np.real(np.conj(x) @ lin.M @ x) * max(w, 1e-12) ** 2
        zeta = float(np.imag(num) / (2 * den)) if w > 0 else 0.0
        labels_extra = " (divergent)" if (z.real > 1e-3 * w_ref and abs(z.imag) < 1e-3 * w_ref) else ""
        lab, part = _labels(model, x, lin.M)
        if w < 1e-3 * ss.Omega:
            lab = "rigid spin" if part.get("hub", 0) > 0.05 else lab + " (zero)"
            labels_extra = ""
        out_f.append(w / (2 * np.pi))
        out_z.append(zeta)
        shapes.append(x)
        labels.append(lab + labels_extra)
        parts.append(part)
    return ModeData(np.array(out_f), np.array(out_z), np.array(shapes), labels, parts)


def campbell(model: RotorModel, rpm_grid, n_modes: int | None = None) -> CampbellResult:
    freqs, labels = [], None
    st0 = None
    for rpm in rpm_grid:
        Om = rpm * 2 * np.pi / 60
        if Om > 0:
            st, info = solve_static(model.fe, Om, None, state0=st0, n_steps=1 if st0 else 2,
                                    extra_fixed=(6 * model.hub + 5,))
            st0 = st
        else:
            st = model.fe.initial_state()
        ss = StaticState(st, Om, rpm)
        md = modes(model, ss, None, n_modes)
        freqs.append([float(x) for x in md.freqs_hz])
        labels = md.labels
    nmin = min(len(f) for f in freqs)
    return CampbellResult(rpm=[float(r) for r in rpm_grid], freqs_hz=[f[:nmin] for f in freqs],
                          labels=(labels or [])[:nmin])


# =============================================================================== harmonic 1/rev


@dataclass
class HarmonicData:
    result: HarmonicResult
    U: np.ndarray                 # full complex DOF amplitudes
    X: complex                    # inflow amplitude Ω(λc − iλs)
    dQ: complex
    drive: dict
    hub_moment_H: np.ndarray      # complex (3,) rotating-frame moment on vehicle
    aero_moment_mean: tuple[float, float]
    blade_angles: list[dict]


def _fixed_frame(Mh: np.ndarray, n: int = 64):
    """Fixed-frame (mean vector, 2/rev amplitude) of a rotating-frame 1/rev moment phasor."""
    psi = np.linspace(0, 2 * np.pi, n, endpoint=False)
    mx = np.real(Mh[0] * np.exp(1j * psi))
    my = np.real(Mh[1] * np.exp(1j * psi))
    Fx = np.cos(psi) * mx - np.sin(psi) * my
    Fy = np.sin(psi) * mx + np.cos(psi) * my
    mean = np.array([Fx.mean(), Fy.mean()])
    c2x = 2 * np.mean(Fx * np.exp(-2j * psi))
    c2y = 2 * np.mean(Fy * np.exp(-2j * psi))
    amp2 = float(np.sqrt((abs(c2x) ** 2 + abs(c2y) ** 2) / 2))
    return mean, amp2


def harmonic_1p(model: RotorModel, ss: StaticState, aero=None, lin: Linear | None = None,
                unit_torque: bool = False) -> HarmonicData:
    d = model.design
    fe = model.fe
    lin = lin or linearize(model, ss, d.analysis.coriolis)
    f = lin.free
    w = ss.Omega
    fz = w / (2 * np.pi)
    n = fe.ndof
    Z = K_complex(model, lin, fz) + 1j * w * lin.G - w * w * lin.M
    Q0 = float(ss.trim.Q) if ss.trim is not None else 0.0
    drive = {}
    hubd = 6 * model.hub + 5
    if unit_torque:
        dQ = 1.0 + 0j
    elif d.op.drive == "duty":
        from . import motor as mot
        drive = mot.duty_drive(d.motor, ss.rpm, Q0, d.op.torque_mod_frac, d.op.duty_mod_amp)
        dQ = complex(drive["Q_drive"])
        Z[hubd, hubd] += drive["k_hub"]
    else:
        dQ = complex(d.op.torque_mod_frac * Q0)
    use_aero = aero is not None and ss.trim is not None and d.analysis.aero != "none"
    F = np.zeros(n, complex)
    F[hubd] = dQ
    if use_aero:
        al = aero_linear(model, ss.state, aero, ss.trim, w, d.analysis.dynamic_inflow)
        Z = Z - al.Ka
        # unknowns [u_free, X]
        nf = len(f)
        A = np.zeros((nf + 1, nf + 1), complex)
        A[:nf, :nf] = Z[np.ix_(f, f)]
        A[:nf, nf] = -al.b[f]
        A[nf, :nf] = -al.kappa * al.c[f]
        A[nf, nf] = 1.0 - al.kappa * al.c0
        rhs = np.r_[F[f], 0.0]
        sol = np.linalg.solve(A, rhs)
        U = np.zeros(n, complex)
        U[f] = sol[:nf]
        X = complex(sol[nf])
    else:
        al = None
        U = np.zeros(n, complex)
        U[f] = np.linalg.solve(Z[np.ix_(f, f)], F[f])
        X = 0j
    # reactions at hub (moment on vehicle = −reaction)
    hub6 = np.r_[6 * model.hub:6 * model.hub + 6]
    Rhub = Z[hub6] @ U - F[hub6]
    if al is not None:
        Rhub -= al.b[hub6] * X
    M_H = -Rhub[3:6]
    mean, amp2 = _fixed_frame(M_H)
    Mmag = float(np.hypot(*mean))
    # aero-only moment (diagnostic) and thrust
    T1 = 0j
    aero_mean = (0.0, 0.0)
    if al is not None:
        dF = []
        for (row, rowX) in al.Fz_rows:
            T1 += row @ U + rowX * X
        Mx_a = My_a = 0.0
        # mean fixed-frame aero moment from Σ e^{-iψ0}∫ r dFz:  Mx − iMy = (i/2) Σ
        S = al.c @ U + al.c0 * X
        m = 0.5j * S
        aero_mean = (float(m.real), float(-m.imag))
        del dF, Mx_a, My_a
    # blade angles
    blades_out, angles = [], []
    r75 = 0.75 * model.R
    for bi in model.blades:
        u, rv = blade_local_motion_c(model, U, bi)
        i75 = np.argmin(np.abs(bi.chain_r - r75))
        th = rv[i75, 0]
        lag = -(rv[0, 2] - U[hubd])          # relative to the hub
        flap = -rv[0, 1]
        ref = np.exp(-1j * np.angle(dQ)) if not unit_torque else 1.0
        ref = 1.0   # phases relative to the command (duty / torque) peak at ψ = 0
        def ap(z):
            z = z * ref
            return float(np.degrees(abs(z))), float(np.degrees(np.angle(z)))
        pa, pp = ap(th)
        la, lp = ap(lag)
        fa, fp = ap(flap)
        blades_out.append(BladeHarmonic(blade=bi.k, pitch_amp_deg=pa, pitch_phase_deg=pp,
                                        lag_amp_deg=la, lag_phase_deg=lp, flap_amp_deg=fa,
                                        flap_phase_deg=fp))
        angles.append(dict(pitch=th, lag=lag, flap=flap, pitch_root=rv[0, 0],
                           lag_tip=-(u[-1, 1] - U[hubd] * model.R) / (model.R - bi.chain_r[0]),
                           flap_tip=u[-1, 2] / (model.R - bi.chain_r[0])))
    W = d.op.vehicle_mass * G0
    T = float(ss.trim.T) if ss.trim is not None else W
    dQa = abs(dQ)
    hr = HarmonicResult(
        torque_amp=float(dQa), hub_speed_amp_rpm=float(abs(1j * w * U[hubd]) * 60 / (2 * np.pi)),
        blades=blades_out, hub_moment_mean=Mmag,
        hub_moment_phase_deg=float(np.degrees(np.arctan2(mean[1], mean[0]))),
        hub_moment_2p=amp2, thrust_1p=float(abs(T1)),
        moment_per_torque=float(Mmag / dQa) if dQa > 0 else 0.0,
        control_ratio=float(Mmag / (W * model.R)) if W > 0 else 0.0,
        tilt_equiv_deg=float(np.degrees(np.arctan2(Mmag, max(T, 1e-9) * model.R * 0.75))),
        nodes=[tuple(map(float, x)) for x in fe.Xref],
        displacement_re=np.real(U).reshape(-1, 6).tolist(),
        displacement_im=np.imag(U).reshape(-1, 6).tolist())
    return HarmonicData(hr, U, X, dQ, drive, M_H, aero_mean, angles)


def blade_local_motion_c(model: RotorModel, U: np.ndarray, bi: BladeInfo):
    u = np.array([bi.Rk.T @ U[6 * n:6 * n + 3] for n in bi.chain_nodes])
    rv = np.array([bi.Rk.T @ U[6 * n + 3:6 * n + 6] for n in bi.chain_nodes])
    return u, rv


# =============================================================================== flexure stiffness


def flexure_stiffness(model: RotorModel, ss: StaticState | None = None, blade: int = 1) -> np.ndarray:
    """6×6 stiffness of blade `blade`'s flexure (members + stiff links, incl. the
    centrifugal initial-stress stiffness of the trim state) at its hinge point, frame H."""
    fe = model.fe
    bi = model.blades[blade - 1]
    st = ss.state if ss is not None else fe.initial_state()
    sub = FEModel()
    sub.X0 = fe.X0
    sub.node_tags = fe.node_tags
    sub.elements = [e for i, e in enumerate(fe.elements)
                    if e.blade == blade and e.kind in ("flexure", "rigid", "spring")]
    sub._X = fe.Xref
    sub._prepared = True
    sa = assemble_internal(sub, st, True)
    K = assemble_Kmat(sub, sa) + sa.Kgeo
    used = {e.n0 for e in sub.elements} | {e.n1 for e in sub.elements}
    clamp = [model.hub] + [n for n in range(fe.nn) if n not in used]
    sub.fixed = set()
    return condense_to_point(sub, K, bi.master, clamp, st.x[bi.master], bi.hinge_point)


# =============================================================================== checks


_STRENGTH_MEMO: dict = {}


def _strength_along(fb, t_hat) -> float:
    """Memoised Material.strength_along — a pure function of (material, print
    orientation, t̂); it dominated `checks` (~14k calls per evaluate)."""
    o = fb.orientation
    mat = get_material(fb.material)
    key = (id(mat), None if o is None else (tuple(o.build_dir), tuple(o.raster_dir)),
           np.asarray(t_hat, float).tobytes())
    v = _STRENGTH_MEMO.get(key)
    if v is None:
        if len(_STRENGTH_MEMO) > 100_000:
            _STRENGTH_MEMO.clear()
        v = _STRENGTH_MEMO[key] = mat.strength_along(o, t_hat)
    return v


def _fibre_checks(e: Element, sec: SectionProps, s_force: np.ndarray, t_hat: np.ndarray):
    """Max stress utilisation, interlayer utilisation, max tensile strain, details, and the
    creep utilisation max(strain / creep_strain_allow) evaluated per fibre with that fibre's
    own material (a stiff rod near the neutral axis and a compliant matrix at the surface
    are each judged against their own allowable)."""
    S = sec.stiffness6()
    eps = np.linalg.solve(S, s_force)
    out_static = out_il = 0.0
    max_strain = -np.inf
    creep_u = 0.0
    tau = abs(sec.tau_per_twist * eps[3])
    det = None
    for fb in sec.fibres:
        ex = eps[0] + fb.z * eps[4] - fb.y * eps[5]
        sig = fb.E * ex
        mat = get_material(fb.material)
        Xa = _strength_along(fb, t_hat)
        u1 = np.sqrt((sig / Xa) ** 2 + (tau / mat.S) ** 2)
        b = unit(fb.orientation.build_dir)
        c = float(abs(t_hat @ b))
        sn = sig * c * c
        til = abs(sig) * c * np.sqrt(max(0.0, 1 - c * c)) + tau
        u2 = np.sqrt((max(sn, 0.0) / mat.Zt) ** 2 + (til / mat.S) ** 2)
        if u1 > out_static:
            out_static, det = u1, (sig, Xa, fb.material)
        out_il = max(out_il, u2)
        max_strain = max(max_strain, ex)
        creep_u = max(creep_u, ex / mat.creep_strain_allow)
    return out_static, out_il, max_strain, det, creep_u


def _fibre_amp(sec: SectionProps, s_amp: np.ndarray, t_hat, freq_hz: float):
    S = sec.stiffness6()
    eps = np.linalg.solve(S.astype(complex), s_amp)
    best_e = best_s = 0.0
    for fb in sec.fibres:
        ex = abs(eps[0] + fb.z * eps[4] - fb.y * eps[5])
        mat = get_material(fb.material)
        sa = ex * fb.E * mat.storage_scale(freq_hz)
        Xa = _strength_along(fb, t_hat)
        best_e = max(best_e, ex / mat.fatigue_strain_amp_allow)
        best_s = max(best_s, sa / (mat.fatigue_stress_ratio * Xa))
    return best_e, best_s, float(np.max([abs(eps[0] + fb.z * eps[4] - fb.y * eps[5]) for fb in sec.fibres]))


def checks(model: RotorModel, ss: StaticState, hd: HarmonicData | None = None) -> list[Check]:
    fe = model.fe
    st = ss.state
    sa = assemble_internal(fe, st, False)
    groups: dict[str, dict] = {}
    fz = ss.Omega / (2 * np.pi)
    for i, (e, c) in enumerate(zip(fe.elements, sa.cr)):
        if e.kind in ("rigid", "spring") or e.massless:
            continue
        name = e.tag.split(".", 1)[1] if "." in e.tag else e.tag
        if e.kind == "blade":
            name = "blade"
        g = groups.setdefault(name, dict(static=0.0, il=0.0, creep=0.0, fat_e=0.0, fat_s=0.0,
                                         strain=0.0, famp=0.0, det=None))
        t_hat = unit(st.x[e.n1] - st.x[e.n0])
        fa, fb = element_end_forces(e, c)
        secs = [e.sections[0], e.sections[-1]]
        for sec, sf in zip(secs, (fa, fb)):
            u_s, u_il, strain, det, u_cr = _fibre_checks(e, sec, sf, t_hat)
            if u_s > g["static"]:
                g["static"], g["det"] = u_s, det
            g["il"] = max(g["il"], u_il)
            g["creep"] = max(g["creep"], u_cr)
            g["strain"] = max(g["strain"], strain)
        if hd is not None:
            du = hd.U[_edofs(e)]
            fa_c, fb_c = element_end_forces_lin(e, c, du)
            for sec, sf in zip(secs, (fa_c, fb_c)):
                ue, us, amp = _fibre_amp(sec, sf, t_hat, fz)
                g["fat_e"] = max(g["fat_e"], ue)
                g["fat_s"] = max(g["fat_s"], us)
                g["famp"] = max(g["famp"], amp)
    out: list[Check] = []
    for name, g in groups.items():
        det = g["det"]
        note = f"σ={det[0]/1e6:.2f} MPa vs {det[1]/1e6:.1f} MPa ({det[2]})" if det else ""
        out.append(Check(name="static_stress", member=name, value=g["static"], allowable=1.0,
                         utilisation=g["static"], note="Hill-type along-axis strength + torsion; " + note))
        out.append(Check(name="interlayer", member=name, value=g["il"], allowable=1.0,
                         utilisation=g["il"], note="normal/shear on the layer plane vs Zt, S"))
        out.append(Check(name="creep_strain", member=name, value=g["strain"],
                         allowable=g["strain"] / g["creep"] if g["creep"] > 0 else 0.0,
                         utilisation=g["creep"], note="sustained max tensile strain at trim"))
        if hd is not None:
            out.append(Check(name="fatigue_strain", member=name, value=g["famp"],
                             allowable=g["famp"] / g["fat_e"] if g["fat_e"] > 0 else 0.0,
                             utilisation=g["fat_e"], note="1/rev strain amplitude (design torque)"))
            out.append(Check(name="fatigue_stress", member=name, value=g["fat_s"], allowable=1.0,
                             utilisation=g["fat_s"], note="1/rev stress amp / (ratio × strength)"))
    if hd is not None and hd.drive:
        dr = hd.drive
        out.append(Check(name="esc_duty_slew", member="esc", value=float(dr["slew"]),
                         allowable=float(model.design.motor.duty_slew_max),
                         utilisation=float(dr["slew_util"]),
                         note="AM32 duty ramp limit; >1 clips the 1/rev pulse to a triangle"))
        out.append(Check(name="esc_duty_peak", member="esc", value=float(dr["duty_peak"]),
                         allowable=0.95, utilisation=float(dr["duty_peak"] / 0.95)))
    if hd is not None and ss.trim is not None:
        try:
            from . import motor as mot
            cap = mot.capability(model.design.motor, ss.rpm, float(ss.trim.Q))
            dq = abs(hd.dQ)
            out.append(Check(name="motor_torque_1p", member="motor", value=float(dq),
                             allowable=float(cap.dQ_max_1p),
                             utilisation=float(dq / cap.dQ_max_1p) if cap.dQ_max_1p > 0 else float("inf"),
                             note=f"limit: {cap.dQ_limit}"))
            th = mot.thermal(model.design.motor, ss.rpm, float(ss.trim.Q), dq / cap.Kt,
                             model.design.op.temperature_C)
            ss.thermal = th
            out.append(Check(name="motor_thermal", member="motor", value=th["T_winding_C"],
                             allowable=float(model.design.motor.max_winding_C),
                             utilisation=th["utilisation"],
                             note=f"copper {th['copper_W']:.1f} W, total {th['total_W']:.1f} W, "
                                  f"Rth {model.design.motor.thermal_resistance_KW:.2f} K/W (guess)"))
        except Exception as ex:   # pragma: no cover
            model.warnings.append(f"motor capability check failed: {ex}")
    return out


# =============================================================================== evaluate


def _trim_result(model: RotorModel, ss: StaticState) -> TrimResult:
    fe = model.fe
    st = ss.state
    bi = model.blades[0]
    u, rv = blade_local_motion(model, st, bi)
    X0 = fe.Xref
    tip = bi.chain_nodes[-1]
    tipd = bi.Rk.T @ (st.x[tip] - X0[tip])
    coning = float(np.degrees(np.arctan2(tipd[2], model.R - bi.chain_r[0])))
    i75 = np.argmin(np.abs(bi.chain_r - 0.75 * model.R))
    pitch = float(np.degrees(rv[i75, 0]))
    stretch = float(tipd[0])
    # root centrifugal force: Ω² Σ m r_x over blade-1 mass points (blade + flexure)
    cf = 0.0
    sa = assemble_internal(fe, st, False)
    for e, c in zip(fe.elements, sa.cr):
        if e.blade != bi.k or e.massless or len(e.mpts) == 0:
            continue
        cf += ss.Omega**2 * float(np.sum(e.mpts[:, 3] * (c.pos @ bi.Rk[:, 0])))
    tr = ss.trim
    if tr is not None:
        return TrimResult(rpm=ss.rpm, thrust=float(tr.T), torque=float(tr.Q), power=float(tr.P),
                          figure_of_merit=float(tr.FoM), collective_deg=float(np.degrees(tr.collective)),
                          centrifugal_force_root=float(cf), tip_deflection=tuple(map(float, tipd)),
                          coning_deg=coning, root_pitch_change_deg=pitch, radial_stretch=stretch,
                          stations=tr.stations_dict())
    return TrimResult(rpm=ss.rpm, thrust=0.0, torque=0.0, power=0.0, figure_of_merit=0.0,
                      collective_deg=model.design.op.collective_deg, centrifugal_force_root=float(cf),
                      tip_deflection=tuple(map(float, tipd)), coning_deg=coning,
                      root_pitch_change_deg=pitch, radial_stretch=stretch)


def _pick(md: ModeData, key: str, Omega: float) -> float:
    for f, lab in zip(md.freqs_hz, md.labels):
        if lab.startswith(key) and f > 1e-3 * Omega / (2 * np.pi):
            return float(f / (Omega / (2 * np.pi)))
    return float("nan")


def evaluate(design: Design, what: list[str] | None = None) -> Results:
    what = list(what or ["trim", "modes", "harmonic", "checks"])
    t = {}
    t0 = time.perf_counter()
    res = Results(design_hash=design.hash())
    model = build_model(design)
    t["build"] = time.perf_counter() - t0
    t1 = time.perf_counter()
    aero = None
    try:
        aero = make_aero(model)
    except Exception as ex:
        res.warnings.append(f"aero model unavailable ({ex}); structural only")
    t["aero_init"] = time.perf_counter() - t1
    t1 = time.perf_counter()
    ss = static_trim(model, aero)
    t["trim"] = time.perf_counter() - t1
    res.warnings += model.warnings + ss.warnings
    res.trim = _trim_result(model, ss)
    summary = dict(rpm=ss.rpm, Omega=ss.Omega)
    if ss.trim is not None and ss.blade_pitch_el is not None and len(model.blades) >= 2:
        i75 = np.argmin(np.abs(model.r_aero - 0.75 * model.R))
        dp = ss.blade_pitch_el[0, i75] - ss.blade_pitch_el[1, i75]
        summary["trim_diff_pitch_deg"] = float(np.degrees(dp))
        if abs(np.degrees(dp)) > 0.5:
            res.warnings.append(f"static blade-1 vs blade-2 elastic pitch differs by {np.degrees(dp):.2f}° at "
                                "0.75R (drag lag × skew coupling): pre-set blade pitch to balance")
    t1 = time.perf_counter()
    lin = linearize(model, ss, design.analysis.coriolis)
    t["linearize"] = time.perf_counter() - t1
    try:
        res.flexure_stiffness = flexure_stiffness(model, ss).tolist()
        # FACT intent check: is the softest rotation of the root really about the intended
        # skewed hinge axis? (rods across the hinge can turn it into a twist about another axis)
        try:
            _a = np.asarray(fact.hinge_axis(design)[1], float)
            _C = np.linalg.inv(np.asarray(res.flexure_stiffness, float))[3:, 3:]
            _w, _v = np.linalg.eigh(0.5 * (_C + _C.T))
            _soft = _v[:, int(np.argmax(_w))]
            _err = float(np.degrees(np.arccos(min(1.0, abs(float(_soft @ _a))))))
            res.summary["hinge_axis_err_deg"] = _err
            res.summary["hinge_k_free"] = float(1.0 / (_a @ _C @ _a))
            res.summary["hinge_k_ratio"] = float((1.0 / (_a @ _C @ _a)) * float(np.max(_w)))
            if _err > 20.0 and design.flexure.topology not in ("rigid",):
                res.warnings.append(f"flexure softest rotation is {_err:.0f}° from the intended hinge axis "
                                    f"(k about intended axis {res.summary['hinge_k_ratio']:.1f}× the softest): "
                                    "mechanism differs from the FACT design intent")
        except Exception:   # noqa: BLE001
            pass
    except Exception as ex:   # pragma: no cover
        res.warnings.append(f"flexure stiffness failed: {ex}")
    md = None
    if "modes" in what or "campbell" in what:
        t1 = time.perf_counter()
        md = modes(model, ss, aero, lin=lin)
        for i, (f, z, lab, p, shp) in enumerate(zip(md.freqs_hz, md.damping, md.labels, md.parts, md.shapes)):
            res.modes.append(Mode(index=i, freq_hz=float(f), damping_ratio=float(z), label=lab,
                                  participation=p, shape=np.real(shp).reshape(-1, 6).tolist()))
        Om = ss.Omega
        summary.update(lag_freq_per_rev=_pick(md, "lag", Om), flap_freq_per_rev=_pick(md, "flap", Om),
                       pitch_freq_per_rev=_pick(md, "pitch", Om))
        t["modes"] = time.perf_counter() - t1
    if "campbell" in what:
        t1 = time.perf_counter()
        grid = design.op.rpm_grid or list(np.linspace(0.0, 1.2 * ss.rpm, 7))
        res.campbell = campbell(model, grid)
        t["campbell"] = time.perf_counter() - t1
    hd = None
    if "harmonic" in what or "checks" in what:
        t1 = time.perf_counter()
        hd = harmonic_1p(model, ss, aero, lin=lin)
        res.harmonic = hd.result
        a1 = hd.blade_angles[0]
        summary.update(pitch_lag_coupling=float(np.real(a1["pitch_root"] / a1["lag"])) if abs(a1["lag"]) > 0 else 0.0,
                       pitch_lag_coupling_075=float(np.real(a1["pitch"] / a1["lag"])) if abs(a1["lag"]) > 0 else 0.0,
                       tan_skew=float(np.tan(np.radians(design.flexure.skew_deg))),
                       control_ratio=hd.result.control_ratio, moment_per_torque=hd.result.moment_per_torque,
                       hub_moment_mean=hd.result.hub_moment_mean, hub_moment_phase_deg=hd.result.hub_moment_phase_deg,
                       aero_moment_mean=float(np.hypot(*hd.aero_moment_mean)),
                       pitch_amp_deg=hd.result.blades[0].pitch_amp_deg,
                       lag_amp_deg=hd.result.blades[0].lag_amp_deg,
                       inflow_X=float(abs(hd.X)))
        if hd.drive:
            dr = hd.drive
            summary.update(dd=float(dr["dd"]), d0=float(dr["d0"]), drive_phase_deg=float(dr["phase_deg"]),
                           drive_gain_ratio=float(dr["gain_ratio"]), duty_slew_util=float(dr["slew_util"]))
            if dr["duty_peak"] > 0.95:
                res.warnings.append(f"duty peak {dr['duty_peak']:.2f} > 0.95: ESC saturates")
        t["harmonic"] = time.perf_counter() - t1
    if "checks" in what:
        t1 = time.perf_counter()
        res.checks = checks(model, ss, hd)
        if design.print_setup is not None:
            from .printsetup import print_checks
            _pc, _ps, _pw = print_checks(design)
            res.checks += _pc
            res.warnings += _pw
            res.summary.update(_ps)
        for _c in res.checks:
            if _c.name == "motor_thermal":
                _n = _c.note
                summary["T_winding_C"] = float(_c.value)
                summary["copper_W"] = float(_n.split("copper ")[1].split(" W")[0])
        t["checks"] = time.perf_counter() - t1
        if res.checks:
            worst = max(res.checks, key=lambda c: c.utilisation)
            summary["max_utilisation"] = float(worst.utilisation)
            struct = [c for c in res.checks if c.member not in ("esc", "motor") and not c.member.startswith("print:")]
            if struct:
                summary["max_structural_utilisation"] = float(max(c.utilisation for c in struct))
            for c in res.checks:
                if c.name == "motor_torque_1p" and c.utilisation > 1:
                    res.warnings.append(f"1/rev torque {c.value:.4f} N m exceeds motor capability "
                                        f"{c.allowable:.4f} N m")
    if ss.trim is not None:
        summary.update(thrust=float(ss.trim.T), torque=float(ss.trim.Q))
    summary.update(coning_deg=res.trim.coning_deg, radial_stretch=res.trim.radial_stretch,
                   root_cf=res.trim.centrifugal_force_root, elastic_pitch_075_deg=res.trim.root_pitch_change_deg)
    res.summary = {**res.summary, **{k: float(v) for k, v in summary.items()}}
    t["total"] = time.perf_counter() - t0
    res.timings_s = t
    return res
