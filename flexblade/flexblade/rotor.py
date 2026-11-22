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

import dataclasses
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
from .materials import LAMINABLE_SHEETS, SHEET_METALS, SHIM_STEELS  # noqa: F401
from .materials import _get_raw, get_material, part_temperature
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


def pre_lag_rad(design: Design) -> float:
    """flexure.pre_lag_deg as rad ('auto' unresolved → 0; resolve with resolve_pre_lag)."""
    pl = design.flexure.pre_lag_deg
    return 0.0 if pl == "auto" else float(np.radians(pl))


def _rot_axis(a: np.ndarray, ang: float) -> np.ndarray:
    """Rotation matrix by ang about unit axis a (Rodrigues)."""
    S = skew(unit(a))
    return np.eye(3) + np.sin(ang) * S + (1 - np.cos(ang)) * (S @ S)


def preset_pitch(model: RotorModel) -> np.ndarray:
    """(B,) built-in pitch of each blade from the pre-lag rotation (blade-local x component)."""
    psi = pre_lag_rad(model.design)
    return np.array([psi * float((bi.Rk.T @ bi.hinge_dir)[0]) for bi in model.blades])


def split_and_lag(model: RotorModel, ss) -> tuple[float, float]:
    """(blade-1 − blade-2 total pitch at 0.75R incl. preset, rad; mean elastic hinge rotation
    about each blade's own hinge axis at the blade root (lag-positive), rad)."""
    i75 = np.argmin(np.abs(model.r_aero - 0.75 * model.R))
    th = ss.blade_pitch_el[:, i75] + preset_pitch(model)
    lag = np.mean([float(_logm(ss.state.R[bi.master]) @ bi.hinge_dir) for bi in model.blades])
    return float(th[0] - th[1]), float(lag)


def resolve_pre_lag(design: Design, passes: int = 3, info: dict | None = None) -> Design:
    """flexure.pre_lag_deg == 'auto' → numeric: trim at the reference op (pre_lag_ref_mass),
    first step pre-lag = −(pre-lag + mean elastic hinge lag), then secant on the total pitch
    split. The secant slope d(split)/d(pre-lag) = 2 sin(s)·k/(k + K_cf) (info['slope'])."""
    f = design.flexure
    if f.pre_lag_deg != "auto":
        return design
    ref = design
    if f.pre_lag_ref_mass is not None:
        ref = design.model_copy(update=dict(op=design.op.model_copy(
            update=dict(vehicle_mass=f.pre_lag_ref_mass))))

    def run(psi_deg):
        d = ref.model_copy(update=dict(flexure=ref.flexure.model_copy(update=dict(pre_lag_deg=psi_deg))))
        m = build_model(d)
        ss = static_trim(m, make_aero(m))
        return split_and_lag(m, ss)

    xs, gs = [0.0], []
    sp, lag = run(0.0)
    gs.append(sp)
    x = float(np.degrees(-lag))
    for _ in range(max(1, passes - 1)):
        x = float(np.clip(x, -15.0, 15.0))
        sp, lag = run(x)
        xs.append(x)
        gs.append(sp)
        if abs(np.degrees(sp)) < 0.02 or gs[-1] == gs[-2]:
            break
        x = xs[-1] - gs[-1] * (xs[-1] - xs[-2]) / (gs[-1] - gs[-2])
    j = int(np.argmin(np.abs(gs)))
    if info is not None:
        info.update(pre_lag_deg=xs, split_deg=list(np.degrees(gs)),
                    slope=float((gs[1] - gs[0]) / np.radians(xs[1] - xs[0])) if len(xs) > 1 and xs[1] != xs[0] else float("nan"))
    psi = xs[j] if len(xs) == len(gs) else xs[-1]
    return design.model_copy(update=dict(flexure=f.model_copy(update=dict(pre_lag_deg=float(psi)))))


def flexure_root_loads(model: RotorModel, ss) -> list[tuple[np.ndarray, np.ndarray]]:
    """Per blade: steady resultant (F, M) the hub passes into the flexure member set, in the
    blade frame Rk (x radial, y lag/chord, z thrust); M about the blade's hinge point.
    M[1] is the steady flap moment My that blade.precone_deg unloads."""
    from .beam import element_cr
    fe, st = model.fe, ss.state
    out = []
    for bi in model.blades:
        F, M = np.zeros(3), np.zeros(3)
        for ids in bi.member_elems.values():
            if not ids:
                continue
            e = fe.elements[ids[0]]
            c = element_cr(e, st, ss.Omega, want_h=False)
            g = c.B.T @ c.f7
            F += g[0:3]
            M += g[3:6] + np.cross(st.x[e.n0] - bi.hinge_point, g[0:3])
        out.append((bi.Rk.T @ F, bi.Rk.T @ M))
    return out


def strip_end_rotations(model: RotorModel, ss) -> list[list[list[float]]]:
    """Elastic CR end rotations of every flexure member at the static state, deg:
    [blade][member] -> [hub-end θ1..3, blade-end θ1..3] (same layout as strip_preset_deg;
    zero = the member end carries no static bending/torsion)."""
    from .beam import assemble
    sa = assemble(model.fe, ss.state, ss.Omega, want_tangent=False)
    out = []
    for bi in model.blades:
        rows = []
        for j in range(len(bi.members)):
            ids = bi.member_elems.get(f"b{bi.k}.flex{j}", [])
            if not ids or getattr(bi.members[j], "cable", False):
                rows.append([0.0] * 6)
                continue
            ta, tb = sa.cr[ids[0]].d7[1:4], sa.cr[ids[-1]].d7[4:7]
            rows.append([float(v) for v in np.degrees(np.r_[ta, tb])])
        out.append(rows)
    return out


def resolve_strip_preset(design: Design, iters: int = 10, components=(1, 2, 4, 5),
                         tol_deg: float = 0.01, info: dict | None = None) -> Design:
    """Zero-stress trim (I34): fixed point p <- p + ω·(elastic end rotation at hover) on the
    flexure members' bending components (θ2, θ3 at each end; `components` index the 6-vector),
    at design.op (pre_lag 'auto' re-resolved each pass). The plain iteration contracts by
    ρ ≈ k_strip/(k_strip + k_rest) (the strips' share of the stiffness); after two plain
    passes ω = 1/(1 − ρ̂) (Aitken). Returns the design with numeric strip_preset_deg;
    info['resid_deg'] = max |residual| per pass."""
    d = design
    hist = []
    w = 1.0
    for it in range(iters):
        if it == 2 and hist[0] > 0:
            w = 1.0 / (1.0 - min(0.85, hist[1] / hist[0]))
        dd = resolve_pre_lag(d)
        m = build_model(dd)
        ss = static_trim(m, make_aero(m))
        r = strip_end_rotations(m, ss)
        cur = d.flexure.strip_preset_deg
        new = []
        mx = 0.0
        for kb, rows in enumerate(r):
            nr = []
            for j, row in enumerate(rows):
                p = list(cur[kb][j]) if len(cur) > kb and len(cur[kb]) > j else [0.0] * 6
                for c in components:
                    p[c] += w * row[c]
                    mx = max(mx, abs(row[c]))
                nr.append([float(v) for v in p])
            new.append(nr)
        hist.append(mx)
        d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(strip_preset_deg=new))))
        if mx < tol_deg:
            break
    if info is not None:
        info.update(resid_deg=hist, omega=w)
    return d


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
    if ideal_hinge is None and design.flexure.topology == "ideal":
        from .schema import IdealFlexure
        ip = design.flexure.ideal or IdealFlexure()
        ideal_hinge = dict(K6=fact.ideal_K6(design).tolist(), mass=ip.mass,
                           com=fact.hinge_axis(design, 1)[0].tolist(),
                           inertia=np.diag([0.0, 0.0, ip.lag_inertia]).tolist(),
                           loss_eta=ip.loss_eta, tip_mass=ip.tip_mass, tip_mass_chord=ip.tip_mass_chord)
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
    pre_lag = pre_lag_rad(design)
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
    from .laminate import laminate_section
    n_lam = int(getattr(design.flexure, "strip_laminae", 1) or 1)
    eta_lam = float(getattr(design.flexure, "strip_lam_eta", 0.0) or 0.0)
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
            sp = section_properties(shp, m.layup, ori, e1, axes=(e2, e3))
            if not m.cable and m.kind == "blade" and m.shape != "circle" and m.layup.kind == "solid" and m.width > 2 * t:
                # wide plate clamped at both fittings bends cylindrically: width strain and
                # in-plane shear are restrained except in a free-edge layer ~ t wide each side
                mat = get_material(m.layup.material)
                Eq, E0 = mat.plate_modulus(ori, e1, e2, e3), mat.axial_modulus(ori, e1)
                phi = 1.0 - 2.0 * t / m.width
                fq = 1.0 + phi * (Eq / E0 - 1.0)
                sp = dataclasses.replace(sp, EIy=sp.EIy * fq, EA=sp.EA * fq)
            if n_lam > 1 and not m.cable and m.kind == "blade" and m.layup.material.startswith(LAMINABLE_SHEETS):
                sp = laminate_section(sp, n_lam)     # unbonded sheet stack (laminate.py)
            return _cable_section(sp, m.cable_bend_factor) if m.cable else sp
        if m.taper == 1.0 or m.layup.kind != "solid":
            s = sec_at(0.5)
            return [s], None, _ea_fracs(s, m.layup)
        from .beam import GAUSS3, GAUSS4
        return [sec_at(x) for x in GAUSS3[0]], [sec_at(x) for x in GAUSS4[0]], _ea_fracs(sec_at(0.5), m.layup)

    # clamped steel strips: the printed CSG hub/root fittings as rigid bodies (fittings.py)
    fit_mp, fit_pending = None, []
    if ideal_hinge is None and design.flexure.fitting_mass == "auto":
        from .fittings import fitting_mass_props, is_clamped_strip
        if is_clamped_strip(design):
            fit_mp = fitting_mass_props(design)

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
        Tk = fact.blade_flexure_T(design, k)
        mem_local = [fact.mirror_member(m, Tk) if k % 2 == 0 else m for m in members1]
        mem_H = []
        for m in mem_local:
            mem_H.append(m.model_copy(update=dict(
                p0=tuple(map(float, Rk @ np.asarray(m.p0))), p1=tuple(map(float, Rk @ np.asarray(m.p1))),
                width_dir=tuple(map(float, Rk @ np.asarray(m.width_dir))))))
        # blade chain nodes (pre-lag: blade side rotated about this blade's own hinge axis)
        H0k, ak = fact.hinge_axis(design, k)
        av = ak if design.flexure.pre_lag_axis == "hinge" else np.array([ak[0], 0.0, 0.0])
        Rp = _rot_axis(Rk @ av, pre_lag * float(np.linalg.norm(av)))
        # precone: about this blade's own chord axis −Rk·ŷ so Rk·x̂ → cosβ Rk·x̂ + sinβ ẑ (tip up)
        # on every blade — Rk is a z-rotation, so the cone is the same for the mirrored blades
        beta0 = np.radians(float(getattr(b, "precone_deg", 0.0) or 0.0))
        if beta0 != 0.0:
            Rp = Rp @ _rot_axis(Rk @ np.array([0.0, -1.0, 0.0]), beta0)
        pk = Rk @ H0k
        chain = [fe.add_node(pk + Rp @ (Rk @ np.array([r, 0.0, 0.0]) - pk), f"b{k}.r{r*1e3:.1f}")
                 for r in r_chain]
        master = chain[0]
        blade_elems = []
        for i in range(len(r_chain) - 1):
            sec, chord, twist, layup = blade_secs[i]
            e2 = Rp @ Rk @ np.array([0.0, np.cos(twist), np.sin(twist)])
            kind = "shank" if i < n_sh else "blade"
            el = Element(chain[i], chain[i + 1], e2, [sec], material_fracs=_ea_fracs(sec, layup),
                         tag=f"b{k}.{kind}", kind=kind, blade=k)
            xm = []
            for bm in b.balance_masses:
                rb = float(np.clip(bm.r_frac * b.radius, r_chain[0], r_chain[-1]))
                if bm.mass > 0 and (r_chain[i] <= rb < r_chain[i + 1] or (i == len(r_chain) - 2 and rb >= r_chain[i])):
                    xi = (rb - r_chain[i]) / (r_chain[i + 1] - r_chain[i])
                    xm.append([xi, bm.chord_offset, bm.normal_offset, bm.mass])
            if xm:
                el.extra_mpts = np.array(xm)
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
                if eta_lam > 0 and not m.cable and m.kind == "blade" and m.layup.material.startswith(LAMINABLE_SHEETS):
                    fe.elements[ids[-1]].loss_eta = eta_lam
            member_elems[tag] = ids
            sp = design.flexure.strip_preset_deg
            if ids and len(sp) >= k and len(sp[k - 1]) > j and any(sp[k - 1][j]):
                pv = np.radians(np.asarray(sp[k - 1][j], float))
                for ie, sl, pr in ((ids[0], slice(1, 4), pv[:3]), (ids[-1], slice(4, 7), pv[3:])):
                    el_ = fe.elements[ie]
                    if el_.d7_0 is None:
                        el_.d7_0 = np.zeros(7)
                    el_.d7_0[sl] = pr
            # member-end anchorage on the printed fittings (zero-length punch springs)
            n_in, n_out = nodes[0], nodes[-1]
            anc = an.member_anchorage
            if m.kind != "rigid" and anc != "none" and m.anchorage != "embedded":
                from .anchorage import end_spring
                from .geometry import HUB_MATERIAL
                mo = _rot_orient(m.orientation, Rk)
                F = np.column_stack([e1, e2, e3])
                ends = [("hub", HUB_MATERIAL, None)]
                if anc == "both":
                    ends.append(("blade", b.layup.material, _rot_orient(b.orientation, Rk)))
                for side, fmat, fo in ends:
                    Ka = end_spring(m, mo, e1, e2, e3, fmat, fo)
                    nd = nodes[0] if side == "hub" else nodes[-1]
                    s_node = fe.add_node(fe.X0[nd].copy(), f"{tag}.anc_{side}")
                    pair = (s_node, nd) if side == "hub" else (nd, s_node)
                    fe.add_element(Element(*pair, np.zeros(3), [], tag=f"{tag}.anc_{side}",
                                           kind="spring", blade=k, spring_K=Ka, spring_frame=F))
                    if side == "hub":
                        n_in = s_node
                    else:
                        n_out = s_node
            if design.flexure.axial_release and not m.cable and m.kind != "rigid":
                # radial slider / zig-zag at the blade end: soft along the rotor radial
                # direction, stiff otherwise → the printed member carries ~no centrifugal load
                Lr = max(design.flexure.length, 1e-3)
                kt, kr = STIFF_FACTOR * max_EA / Lr, STIFF_FACTOR * max_EI / Lr
                Ka = np.diag([design.flexure.axial_release_stiffness, kt, kt, kr, kr, kr])
                s_node = fe.add_node(fe.X0[n_out].copy(), f"{tag}.release")
                fe.add_element(Element(n_out, s_node, np.zeros(3), [], tag=f"{tag}.release",
                                       kind="spring", blade=k, spring_K=Ka,
                                       spring_frame=np.column_stack([Rk[:, 0], Rk[:, 1], Rk[:, 2]])))
                n_out = s_node
            # spokes and connectors (stiff, massless)
            pending_stiff.append((hub, n_in, np.array([0.0, 0.0, 1.0]), f"b{k}.spoke{j}", k))
            if np.linalg.norm(p1 - fe.X0[master]) > 1e-9:
                pending_stiff.append((n_out, master, np.array([0.0, 0.0, 1.0]), f"b{k}.conn{j}", k))
            else:
                pending_stiff.append((n_out, master, None, f"b{k}.conn{j}", k))
        H0, a = fact.hinge_axis(design, k)
        if ideal_hinge is not None:
            A = fe.add_node(Rk @ H0, f"b{k}.hingeA")
            pending_stiff.append((hub, A, np.array([0.0, 0.0, 1.0]), f"b{k}.spoke0", k))
            kh = ideal_hinge
            if "K6" in kh:
                # full 6×6 in blade-1 frame H; blade k even: mirror z → −z (translations
                # diag(1,1,−1), rotations (pseudo-vector) diag(−1,−1,1)); then rotate by Rk
                D = np.zeros((6, 6))
                D[:3, :3], D[3:, 3:] = Tk, np.linalg.det(Tk) * Tk
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
            fe.elements[ie].loss_eta = float(kh.get("loss_eta", 0.0))
            if "K6" in kh:
                full_springs.append((k, ie, Ks.copy()))
            if kh.get("tip_mass", 0.0) > 0.0:
                c = Rk @ np.array([0.0, float(kh.get("tip_mass_chord", 0.0)), 0.0])
                S = skew(c)
                m_ = float(kh["tip_mass"])
                Mn = np.zeros((6, 6))
                Mn[:3, :3] = m_ * np.eye(3)
                Mn[:3, 3:] = -m_ * S
                Mn[3:, :3] = m_ * S
                Mn[3:, 3:] = m_ * ((c @ c) * np.eye(3) - np.outer(c, c))
                fe.add_lumped_inertia(chain[-1], Mn)
            if kh.get("mass", 0.0) > 0.0 or np.any(np.asarray(kh.get("inertia", 0.0), float)):
                Mt = Tk
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
        if fit_mp is not None:
            Mt = Tk
            m_, c1, I1 = fit_mp["hub"]          # hub side: rigid with the hub node (origin)
            Rh = Rk @ Mt
            # spin inertia only: the hub node translates only in the stability hub basis, whose
            # Coriolis terms come from the mass points (fe.lumped has no rotating terms)
            Mhf = np.zeros((6, 6))
            Mhf[5, 5] = _rigid6(m_, Rh @ c1, Rh @ I1 @ Rh.T)[5, 5]
            fe.add_lumped_inertia(hub, Mhf)
            m_, c1, I1 = fit_mp["root"]         # blade side: pre-lagged with the blade chain
            Rb = Rp @ Rk @ Mt
            fit_pending.append((k, master, m_, pk + Rp @ (Rk @ Mt @ c1 - pk), Rb @ I1 @ Rb.T))
        blades.append(BladeInfo(k=k, psi0=psi0, Rk=Rk, master=master, chain_nodes=chain,
                                chain_r=r_chain, aero_nodes=chain[n_sh:], aero_r=r_aero,
                                blade_elems=blade_elems, member_elems=member_elems, members=mem_H,
                                hinge_point=Rk @ H0, hinge_dir=Rk @ a))
    k_link = float(getattr(design.flexure, "pitch_link_stiffness", 0.0) or 0.0)
    if k_link > 0.0 and B == 2:
        # collective-pitch interlink (see-saw through the hub): rotational spring about the
        # blade-1 pitch axis between the two blade roots. Blade 2's pitch axis is −x, so the
        # relative rotation is θ1 + θ2 (collective); cyclic θ1 = −θ2 is a free rigid roll.
        ex = blades[0].Rk[:, 0]
        fe.add_element(Element(blades[0].master, blades[1].master, np.zeros(3), [], tag="pitch_link",
                               kind="spring", blade=0, spring_K=np.diag([0.0, 0, 0, k_link, 0, 0]),
                               spring_frame=np.column_stack([ex, blades[0].Rk[:, 1], np.zeros(3)])))
    stiff = stiff_section(STIFF_FACTOR * max_EA, STIFF_FACTOR * max_EI, STIFF_FACTOR * max_GJ)
    for k, mnode, m_, cw, Ic in fit_pending:
        _fitting_body(fe, mnode, m_, cw, Ic, stiff, k)
    merges: list[tuple[int, int]] = []
    for n0, n1, e2, tag, k in pending_stiff:
        if np.linalg.norm(fe.X0[n1] - fe.X0[n0]) < 1e-9:
            merges.append((n0, n1))
            continue
        if e2 is None:      # coincident before the pre-lag rotation moved the blade side
            e2 = np.array([0.0, 0.0, 1.0])
        fe.add_element(Element(n0, n1, e2, [stiff], tag=tag, kind="rigid", blade=k, massless=True))
    if merges:
        _merge_nodes(fe, merges, blades)
    fe.prepare()
    model = RotorModel(design, fe, hub, blades, r_aero, b.elastic_axis_chord_frac,
                       st_r, st_chord, st_twist, warnings)
    if full_springs:
        _compensate_links(model, full_springs)
    return model


def _rigid6(m: float, c: np.ndarray, Ic: np.ndarray) -> np.ndarray:
    """6x6 lumped mass of a rigid body (mass m, centre c from the node, inertia Ic about c)."""
    S = skew(c)
    Mn = np.zeros((6, 6))
    Mn[:3, :3] = m * np.eye(3)
    Mn[:3, 3:] = -m * S
    Mn[3:, :3] = m * S
    Mn[3:, 3:] = Ic + m * ((c @ c) * np.eye(3) - np.outer(c, c))
    return Mn


def _fitting_body(fe: FEModel, node: int, m: float, cw: np.ndarray, Ic: np.ndarray, stiff, k: int) -> None:
    """Rigid body (m, centre cw, inertia Ic about cw) hung on `node` as 6 point masses on a stiff
    cantilever element, so it gets centrifugal, Coriolis and hub-acceleration (Omega-dot) terms
    like the blade mass points (fe.lumped carries no rotating terms)."""
    if m <= 0:
        return
    xa = fe.X0[node]
    lam, V = np.linalg.eigh(0.5 * (Ic + Ic.T))
    J = np.maximum(0.5 * lam.sum() - lam, 0.0)      # principal second moments about cw
    d = np.sqrt(3.0 * J / m)
    pts = [cw + sg * d[i] * V[:, i] for i in range(3) for sg in (1.0, -1.0)]
    dv = cw - xa
    e1 = dv / np.linalg.norm(dv) if np.linalg.norm(dv) > 1e-5 else unit(np.r_[xa[:2], 0.0] + [1e-9, 0, 0])
    ref = np.array([0.0, 0.0, 1.0]) if abs(e1[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e2 = unit(ref - e1 * (e1 @ ref))
    e3 = np.cross(e1, e2)
    L = 2.0 * (np.linalg.norm(dv) + d.max()) + 1e-3
    nb = fe.add_node(xa + e1 * L, f"b{k}.fitting")
    xm = np.array([[(p - xa) @ e1 / L, (p - xa) @ e2, (p - xa) @ e3, m / 6.0] for p in pts])
    el = Element(node, nb, e2, [stiff], tag=f"b{k}.fitting", kind="rigid", blade=k)
    el.extra_mpts = xm
    fe.add_element(el)


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
        fm = sa.fint[6 * self.master:6 * self.master + 6] if state is not None else None
        return condense_to_point(fe, K, self.master, self.clamp, st.x[self.master], point, fm,
                                 fe.X0[self.master] - np.asarray(point, float))


def condense_to_point(fe: FEModel, K: np.ndarray, master: int, clamp: list[int], x_master, point,
                      f_master=None, lever=None) -> np.ndarray:
    """Condense K onto the master node and transport it to `point` (rigid offset d = x_master −
    point). Under a preload (f_master = internal force [F; M] the substructure exerts at the
    master, i.e. ∂U/∂q_m) the transport is nonlinear: x_m = x_p + R(θ)d, so the Hessian of
    U(q_m(q_p)) gains F·∂²(R d)/∂θ² = ½(F dᵀ + d Fᵀ) − (F·d) I on the rotation block. Without
    it a tension T reported at a point a distance d inboard of where it acts shows a spurious
    pendulum stiffness ≈ T·d (the stiffness of the same structure would depend on the
    reporting point). With it the 6×6 is the consistent tangent "as if cut at `point`" — what
    a zero-length 6×6 spring at `point` (topology "ideal") must carry to reproduce it.
    `lever` (default d) is the rigid-connection lever used in that term; flexure_stiffness
    passes the reference-geometry lever so the flexure's own deformation (e.g. the stretch of
    an ideal zero-length spring under CF) is left to its element tangent, not re-transported."""
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
    if f_master is not None:
        F = np.asarray(f_master, float)[:3]
        dl = d if lever is None else np.asarray(lever, float)
        Kp[3:, 3:] += 0.5 * (np.outer(F, dl) + np.outer(dl, F)) - (F @ dl) * np.eye(3)
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
    trim_ok: bool = True         # False: T≠W trim failed; result is the best point, never use it as a trim


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

    x0 = float(trim0.rpm)      # rigid-blade BEMT/momentum rpm (T = C_T ρ A (ΩR)²) = bracket centre
    tol = 1e-3 * W
    pts: dict = {}              # rpm -> (state, info, aero trim): warm starts + bracket data
    its = 0
    fail_msg = None

    def ev(xr):
        nonlocal its
        near = min(pts, key=lambda k: abs(k - xr)) if pts else None
        s1, inf, t1 = solve_at(xr, pts[near][0] if near is not None else None)
        its += inf.iterations
        pts[xr] = (s1, inf, t1)
        if _DEBUG:
            print(f"trim outer: rpm {xr:.1f} T-W {t1.T - W:+.4f} N  newton its {inf.iterations} {inf.message}")
        return float(t1.T - W)

    def bracket(start_list, up_first):
        """Walk geometrically (warm) from the first point until T−W changes sign."""
        f_prev, x_prev = None, None
        for xr in start_list:
            f = ev(xr)
            if not pts[xr][1].converged:
                return None
            if abs(f) < tol:
                return (xr, xr)
            if f_prev is not None and f * f_prev < 0:
                return (min(x_prev, xr), max(x_prev, xr))
            f_prev, x_prev = f, xr
        return None

    st = info = tr = None
    x = x0
    conv = ok_outer = False
    if fixed_rpm:
        st, info, tr = solve_at(x0, None)
        its, conv, ok_outer = info.iterations, info.converged, True
    else:
        f0 = ev(x0)
        if pts[x0][1].converged:
            g = 1.06 if f0 < 0 else 1 / 1.06
            up = [x0 * g ** i for i in range(0, 26)]
            br = bracket(up, f0 < 0)
            if br is None:      # fallback: continuation from low rpm upward (other branch)
                br = bracket([x0 * 0.4 * 1.15 ** i for i in range(0, 18)], True)
            if br is not None:
                if br[0] == br[1]:
                    x = br[0]
                else:
                    xl, xh = br
                    fl, fh = pts[xl][2].T - W, pts[xh][2].T - W
                    # bracketed regula-falsi/bisection hybrid (Illinois), warm-started evaluations
                    side = 0
                    for _ in range(max_outer + 14):
                        xm = xh - fh * (xh - xl) / (fh - fl)
                        if not (min(xl, xh) < xm < max(xl, xh)):
                            xm = 0.5 * (xl + xh)
                        fm = ev(xm)
                        if not pts[xm][1].converged:
                            break
                        x = xm
                        if abs(fm) < tol:
                            break
                        if fm * fl < 0:
                            xh, fh = xm, fm
                            if side == -1:
                                fl *= 0.5
                            side = -1
                        else:
                            xl, fl = xm, fm
                            if side == 1:
                                fh *= 0.5
                            side = 1
                        if abs(xh - xl) < 1e-6 * xm:
                            break
        # choose result: the evaluated point closest to the target
        x = min(pts, key=lambda k: abs(pts[k][2].T - W))
        st, info, tr = pts[x]
        conv = bool(info.converged)
        ok_outer = conv and abs(tr.T - W) < tol
        fail_msg = None if ok_outer else (
            f"trim failed: no converged rpm with T = W found (best rpm {x:.0f}: T = {tr.T:.3f} N vs W = {W:.3f} N; "
            "thrust(rpm) does not cross the weight, e.g. torsional collapse branch)")
    if not ok_outer:
        warn.append(fail_msg or f"trim rpm iteration not converged (T−W = {tr.T - W:+.3f} N)")
    if not conv:
        warn.append(f"nonlinear static did not converge: {info.message}")
    _, th, be = shape_of(st)
    Om = x * 2 * np.pi / 60
    F = aero_load_vector(model, st, tr) if use_aero_loads else None
    return StaticState(st, Om, x, tr, F, conv, its, warn, th, be, trim_ok=bool(ok_outer and conv))


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


def modal_basis(model: RotorModel, ss: StaticState, lin: Linear, n_basis: int = 40,
                n_keep: int = 10) -> tuple[np.ndarray, np.ndarray]:
    """Undamped real modal basis (λ = ω², Φ over lin.free) of the rotating-frame system."""
    f = lin.free
    n_modes = n_keep
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
    return lam, Phi


def modes(model: RotorModel, ss: StaticState, aero=None, n_modes: int | None = None,
          coriolis: bool = True, lin: Linear | None = None, n_basis: int = 40) -> ModeData:
    fe = model.fe
    lin = lin or linearize(model, ss, coriolis)
    f = lin.free
    n_modes = n_modes or model.design.analysis.n_modes
    lam, Phi = modal_basis(model, ss, lin, n_basis, n_modes)
    nb = len(lam)
    w_ref = max(ss.Omega, 2 * np.pi * 10.0)
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
    hub_force_H: Optional[np.ndarray] = None   # complex (3,) rotating-frame force on vehicle


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
        frac = d.op.torque_mod_frac
        if d.op.pulse_capability_margin is not None and Q0 > 0:
            # controller uses what the motor can sustain at THIS rpm/torque (robust throttle sweeps)
            frac = d.op.pulse_capability_margin * mot.capability(
                d.motor, ss.rpm, Q0, ambient_C=d.op.temperature_C).mod_frac_max
        drive = mot.duty_drive(d.motor, ss.rpm, Q0, frac, d.op.duty_mod_amp)
        dQ = complex(drive["Q_drive"])
        if d.motor.esc is not None and d.motor.esc.current_control:
            dQ = complex(drive["Q_dc"])                     # torque loop: command delivered as torque
            Z[hubd, hubd] += 1j * w * d.motor.b             # no back-EMF damping
        else:
            Z[hubd, hubd] += drive["k_hub"]
    else:
        dQ = complex(d.op.torque_mod_frac * Q0)
    if d.motor.esc is not None:
        # ESC speed loop at 1P: Q = −(kp + ki/jΩ) F(jΩ) e^{−jΩτ}/(1 + jΩ tl) · jΩ θ  (back-EMF is in k_hub);
        # rpm_filter=rev_avg: F(jΩ) = 0 exactly -> only the direct (back-EMF / current-loop) terms remain
        from . import stability as stb
        dp = stb.drive_params(d.motor, stb.spin_inertia(model, ss, lin), drive.get("d0", 0.6),
                               Omega=ss.Omega)
        k_hub_in = d.op.drive == "duty" and not unit_torque and not d.motor.esc.current_control
        emf = not d.motor.esc.current_control and not k_hub_in
        Z[hubd, hubd] += 1j * w * stb.drive_impedance(dp, 1j * w, emf=emf)
        drive = dict(drive, esc=dp)
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
    F_H = -Rhub[0:3]
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
        control_ratio=(float(Mmag / (W * model.R)) if W > 0 else 0.0) if getattr(ss, 'trim_ok', True) else float('nan'),
        tilt_equiv_deg=float(np.degrees(np.arctan2(Mmag, max(T, 1e-9) * model.R * 0.75))),
        nodes=[tuple(map(float, x)) for x in fe.Xref],
        displacement_re=np.real(U).reshape(-1, 6).tolist(),
        displacement_im=np.imag(U).reshape(-1, 6).tolist())
    return HarmonicData(hr, U, X, dQ, drive, M_H, aero_mean, angles, F_H)


def blade_local_motion_c(model: RotorModel, U: np.ndarray, bi: BladeInfo):
    u = np.array([bi.Rk.T @ U[6 * n:6 * n + 3] for n in bi.chain_nodes])
    rv = np.array([bi.Rk.T @ U[6 * n + 3:6 * n + 6] for n in bi.chain_nodes])
    return u, rv


# =============================================================================== flexure stiffness


def flexure_stiffness(model: RotorModel, ss: StaticState | None = None, blade: int = 1,
                      cables_only: bool = False) -> np.ndarray:
    """6×6 stiffness of blade `blade`'s flexure (members + stiff links, incl. the
    initial-stress (geometric) stiffness of the trim state) at its hinge point, frame H.

    Loaded (ss given): consistent tangent of the flexure's internal forces, transported from
    the blade master node to the hinge point with the preload term (see condense_to_point),
    so a pin at the hinge point + rigid link to the master reports the pin stiffness, not
    pin + T·d. It is directly comparable with an "ideal" zero-length 6×6 at the hinge point
    (whose loaded K equals its K6). The rotor CF lag stiffness (Ω² e S) is NOT included: it
    belongs to the blade and is in the full dynamic model either way. Unloaded (ss None):
    material stiffness only.

    cables_only: keep only the strap strands (cable members) of the flexure, e.g. to read the
    tension-torsion strap's own geometric pitch stiffness (bifilar: ≈ Σ T_i r²/L about the
    radial axis through the hinge point, ∝ Ω²)."""
    fe = model.fe
    bi = model.blades[blade - 1]
    st = ss.state if ss is not None else fe.initial_state()
    sub = FEModel()
    sub.X0 = fe.X0
    sub.node_tags = fe.node_tags
    cab = {f"flex{j}" for j, m in enumerate(bi.members) if m.cable}
    sub.elements = [e for i, e in enumerate(fe.elements)
                    if e.blade == blade and e.kind in ("flexure", "rigid", "spring")
                    and not (cables_only and e.kind == "flexure" and e.tag.split(".", 1)[-1] not in cab)]
    sub._X = fe.Xref
    sub._prepared = True
    sa = assemble_internal(sub, st, True)
    K = assemble_Kmat(sub, sa) + sa.Kgeo
    used = {e.n0 for e in sub.elements} | {e.n1 for e in sub.elements}
    clamp = [model.hub] + [n for n in range(fe.nn) if n not in used]
    sub.fixed = set()
    fm = sa.fint[6 * bi.master:6 * bi.master + 6] if ss is not None else None
    return condense_to_point(sub, K, bi.master, clamp, st.x[bi.master], bi.hinge_point, fm,
                             fe.X0[bi.master] - bi.hinge_point)


PITCH_CF_RATIO_MAX = 1.0
A_LIFT_EFF = 5.7          # 3-D lift-curve slope used in the aero pitch-stiffness estimate, /rad


def pitch_cf(model: RotorModel, Omega: float, K6=None, blade: int = 1) -> dict:
    """CF propeller ("tennis-racket") moment vs pitch stiffness through the skewed hinge.

    Centrifugal potential V = -1/2 Omega^2 sum m |P r|^2 (P: projection on the rotor plane).
    For a small blade rotation w about the hinge point p (d = r - p), its Hessian is
        K_cf = -Omega^2 sum m [ S^T P S + sym(P r (x) d) - (P r . d) I ],  S w = w x d,
    over the blade's mass points (incl. balance masses) at the design geometry.
    K_cf[x,x] = Omega^2 sum m (y^2 - z^2) = Omega^2 (I_chord - I_normal) cos 2theta is the
    propeller-moment stiffness: a spring pulling the sections to FLAT pitch. Against it act the
    flexure (K6, loaded) seen through the skew (a rotation about the hinge axis couples pitch
    to lag, which CF lag stiffness resists), the rest of K_cf, and the aero pitch stiffness
    -1/2 rho (Omega r)^2 a c^2 (x_ea - 0.25) (negative = destabilising, EA aft of the AC).
    With k_eff = 1/[(K_rest)^-1]_xx the static pitch is theta ~ theta0/(1+ratio),
    ratio = k_prop/k_eff, and thrust T ~ Omega^2 theta saturates as Omega grows at
    T_max ~ T(1+ratio)/ratio: ratio >> 1 means the trim cannot reach T=W (blade collapses
    flat). Reported as pitch_cf_ratio (check limit PITCH_CF_RATIO_MAX)."""
    fe = model.fe
    bi = model.blades[blade - 1]
    p = np.asarray(bi.hinge_point, float)
    P = np.diag([1.0, 1.0, 0.0])
    W2 = Omega ** 2
    Kcf = np.zeros((3, 3))
    for e in fe.elements:
        if e.blade != blade or e.kind not in ("blade", "shank") or e.mpts is None or not len(e.mpts):
            continue
        xa = fe.X0[e.n0]
        for xi, y, z, m in e.mpts:
            r = xa + e.E0 @ np.array([xi * e.L0, y, z])
            d = r - p
            S = -skew(d)
            Pr = P @ r
            A = S.T @ P @ S + 0.5 * (np.outer(Pr, d) + np.outer(d, Pr)) - (Pr @ d) * np.eye(3)
            Kcf -= W2 * m * A
    Rk = bi.Rk
    Kcf = Rk.T @ Kcf @ Rk          # blade-local frame (pitch = local x)
    k_prop = float(Kcf[0, 0])
    rho = model.design.op.rho
    r = np.asarray(model.r_aero, float)
    c = model.chord(r)
    w = np.gradient(r) if len(r) > 1 else np.ones(1)
    k_aero = float(-np.sum(0.5 * rho * W2 * r ** 2 * A_LIFT_EFF * c ** 2 * (model.ea_frac - 0.25) * w))
    if K6 is None:
        K6 = flexure_stiffness(model, None, blade)
    K6 = np.asarray(K6, float)
    Ks = np.linalg.inv(np.linalg.inv(0.5 * (K6 + K6.T))[3:, 3:])
    if blade != 1:
        Ks = Rk.T @ Ks @ Rk
    Kr = Ks + Kcf - k_prop * np.diag([1.0, 0, 0]) + np.diag([k_aero, 0, 0])
    try:
        k_eff = float(1.0 / np.linalg.inv(Kr)[0, 0])
    except np.linalg.LinAlgError:
        k_eff = 0.0
    ratio = k_prop / k_eff if k_eff > 0 else float("inf")
    return dict(pitch_cf_ratio=float(ratio), pitch_k_prop=k_prop, pitch_k_eff=k_eff,
                pitch_k_aero=k_aero, pitch_k_struct=float(1.0 / np.linalg.inv(Ks)[0, 0]))


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


def _cable_section(sp: SectionProps, f: float) -> SectionProps:
    """Unbonded strand: keep EA and mass, scale EI/GJ by f (≈ 1/filaments) and fibre offsets
    by √f so a curvature gives the (small) filament-level bending strain."""
    import dataclasses as _dc
    q = np.sqrt(f)
    return _dc.replace(sp, EIy=sp.EIy * f, EIz=sp.EIz * f, EIyz=sp.EIyz * f, GJ=sp.GJ * f,
                       tau_per_twist=sp.tau_per_twist * q,
                       fibres=[_dc.replace(fb, y=fb.y * q, z=fb.z * q) for fb in sp.fibres])


# strand anchorage (per strand leg). Pin wrap: rope bend efficiency η = 1 − c/√(D/d)
# (Cordage-Institute-style D/d rule; c = 0.5 for aramid/UHMWPE, 1.0 for brittle CF tow).
# Knot efficiency (typical figure-8/bowline in HMPF/aramid: 0.35-0.5; CF tow ~0.25).
# Epoxy pot: pull-out = τ π d L with τ ≈ 15 MPa aramid, 2 MPa UHMWPE (poor adhesion),
# 20 MPa CF tow; capped at the strand strength. All ROUGH: verify with a pull coupon.
STRAP_ANCHOR = {"aramid_yarn": dict(c_pin=0.5, knot=0.45, tau=15e6),
                "uhmwpe_braid": dict(c_pin=0.5, knot=0.35, tau=2e6),
                "cf_tow": dict(c_pin=1.0, knot=0.25, tau=20e6)}


def strap_anchor_capacity(f) -> tuple[float, str]:
    """Pull-out/anchor capacity (N) of one strap strand of design.flexure f, and a note."""
    mat = get_material(f.strap_material)
    d = f.strap_strand_diameter
    A = np.pi * d * d / 4
    P = mat.Xt * A
    a = STRAP_ANCHOR.get(f.strap_material, dict(c_pin=0.7, knot=0.35, tau=5e6))
    if f.strap_anchor == "pin_wrap":
        eta = max(0.05, 1.0 - a["c_pin"] / np.sqrt(max(f.strap_pin_diameter / d, 1e-9)))
        return eta * P, f"pin wrap D/d={f.strap_pin_diameter / d:.1f}, η={eta:.2f}"
    if f.strap_anchor == "knot":
        return a["knot"] * P, f"knot η={a['knot']:.2f}"
    cap = min(P, a["tau"] * np.pi * d * f.strap_pot_length)
    return cap, f"epoxy pot τ={a['tau'] / 1e6:.0f} MPa, L={f.strap_pot_length * 1e3:.1f} mm"


# Opt-in (env CABLE_STRAIN=axial; default = solid-section fibre strain): a bought twisted or
# braided cord's filaments bend individually, so its fatigue/creep strain is the axial strain
# eps0 + kappa * r_filament, not the bending strain over the cord radius. r_filament is a material
# constant (Material.filament_radius if present, else Kevlar 49 ~12 um filament -> 6 um).
CABLE_FILAMENT_RADIUS = 6e-6


def _cable_axial() -> bool:
    return _os.environ.get("CABLE_STRAIN", "").strip().lower() == "axial"


def _r_fil(mat) -> float:
    return getattr(mat, "filament_radius", None) or CABLE_FILAMENT_RADIUS


def _fibre_checks(e: Element, sec: SectionProps, s_force: np.ndarray, t_hat: np.ndarray,
                  cable: bool = False):
    """Max stress utilisation, interlayer utilisation, max tensile strain, details, and the
    creep utilisation max(strain / creep_strain_allow) evaluated per fibre with that fibre's
    own material (a stiff rod near the neutral axis and a compliant matrix at the surface
    are each judged against their own allowable).

    cable=True (bought unbonded strand, not printed): the filaments follow the strand axis
    whatever its deflected direction, so the strength is the along-fibre Xt (no off-axis
    Hill knock-down from the print-orientation frame) and there is no interlayer check."""
    S = sec.stiffness6()
    eps = np.linalg.solve(S, s_force)
    out_static = out_il = 0.0
    max_strain = -np.inf
    creep_u = 0.0
    tau = abs(sec.tau_per_twist * eps[3])
    det = None
    ax = cable and _cable_axial()
    for fb in sec.fibres:
        ex = eps[0] + fb.z * eps[4] - fb.y * eps[5]
        sig = fb.E * ex
        mat = get_material(fb.material)
        Xa = mat.Xt if cable else _strength_along(fb, t_hat)
        u1 = np.sqrt((sig / Xa) ** 2 + (tau / mat.S) ** 2)
        b = unit(fb.orientation.build_dir)
        c = float(abs(t_hat @ b))
        sn = sig * c * c
        til = abs(sig) * c * np.sqrt(max(0.0, 1 - c * c)) + tau
        u2 = 0.0 if cable else np.sqrt((max(sn, 0.0) / mat.Zt) ** 2 + (til / mat.S) ** 2)
        if u1 > out_static:
            out_static, det = u1, (sig, Xa, fb.material)
        out_il = max(out_il, u2)
        if ax:      # creep strain only (static strength above keeps the section strain)
            ex = eps[0] + _r_fil(mat) * float(np.hypot(eps[4], eps[5]))
        max_strain = max(max_strain, ex)
        creep_u = max(creep_u, ex / mat.creep_strain_allow)
    return out_static, out_il, max_strain, det, creep_u


# Ultimate tensile strength used for the modified-Goodman / SWT mean-stress check on shim steels.
# 1095 spring temper (ASTM A684 / SAE J403 strip, hardened+tempered): UTS 1.6-1.9 GPa -> 1.75 GPa
# used (the library Xt = 1.4 GPa is the conservative static allowable); 301 FH (AMS 5517): 1.28 GPa.
STRIP_SU = {"steel_1095_shim": 1.75e9, "steel_301_fh": 1.28e9}


def goodman_factor(sm: float, su: float) -> float:
    """Modified Goodman at SF=1: sa/Se + sm/Su <= 1  <=>  (sa/Se) * g <= 1, g = 1/(1 - sm/Su).
    Compressive mean is not credited (g >= 1); sm >= Su -> 1e3 (infeasible)."""
    r = max(sm, 0.0) / su
    return 1e3 if r >= 0.999 else 1.0 / (1.0 - r)


def swt_util(sa: float, sm: float, se: float) -> float:
    """Smith-Watson-Topper: sqrt(smax * sa) / Se, smax = sm + sa (0 if smax <= 0)."""
    return float(np.sqrt(max(sm + sa, 0.0) * sa)) / se


def _fibre_mean(sec: SectionProps, s_force: np.ndarray) -> list[float]:
    """Static axial stress per fibre (Pa) at this section."""
    eps = np.linalg.solve(sec.stiffness6(), s_force)
    return [fb.E * (eps[0] + fb.z * eps[4] - fb.y * eps[5]) for fb in sec.fibres]


def _fibre_amp(sec: SectionProps, s_amp: np.ndarray, t_hat, freq_hz: float, cable: bool = False,
               mean: list[float] | None = None, kt: float = 1.0, diag: dict | None = None):
    S = sec.stiffness6()
    eps = np.linalg.solve(S.astype(complex), s_amp)
    best_e = best_s = 0.0
    ax = cable and _cable_axial()
    exs = []
    for i, fb in enumerate(sec.fibres):
        ex = abs(eps[0] + fb.z * eps[4] - fb.y * eps[5])
        mat = get_material(fb.material)
        if ax:      # 1P amplitude bound |eps0| + r_fil |kappa|
            ex = abs(eps[0]) + _r_fil(mat) * float(np.hypot(abs(eps[4]), abs(eps[5])))
            exs.append(ex)
        sa = ex * fb.E * mat.storage_scale(freq_hz)
        Xa = mat.Xt if cable else _strength_along(fb, t_hat)
        ge = 1.0
        if mean is not None and fb.material.startswith(SHEET_METALS):
            su = STRIP_SU.get(fb.material, mat.Xt)
            ge = goodman_factor(mean[i] * kt, su)
            if diag is not None:
                u = sa / (mat.fatigue_stress_ratio * Xa) * ge
                if u > diag.get("u", -1.0):
                    se = mat.fatigue_stress_ratio * Xa
                    diag.update(u=u, sm=mean[i] * kt, sa=sa * kt, su=su, se=se, ge=ge,
                                swt=swt_util(sa * kt, mean[i] * kt, se))
        best_e = max(best_e, ex / mat.fatigue_strain_amp_allow * ge)
        best_s = max(best_s, sa / (mat.fatigue_stress_ratio * Xa) * ge)
    if ax:
        return best_e, best_s, float(max(exs))
    return best_e, best_s, float(np.max([abs(eps[0] + fb.z * eps[4] - fb.y * eps[5]) for fb in sec.fibres]))


# --- clamp-edge model for steel shim strips -------------------------------------------------
# Real shim flexures fail at the clamp edge (stress concentration + fretting), not mid-span.
# Values (engineering judgement from): Henein, "Flexure pivot design" / Henein et al. cross-spring
# pivots (clamped-blade fatigue, edge fillets); Smith, "Flexures: Elements of Elastic Mechanisms"
# (2000), ch. on stress concentration at blade roots (Kt~1.5-2 at square clamp, ~1.1 with radius
# >~10 t); Cosandier et al., "Fatigue/design of leaf-spring pivots"; Peterson's Stress
# Concentration Factors (fillet r/t); Shigley fretting Marin-type knock-down ~0.7-0.85.
KT_SHARP, FRET_SHARP = 1.75, 0.80
KT_RADIUSED, FRET_RADIUSED = 1.10, 1.0
KT_COMPLIANT, FRET_COMPLIANT = 1.05, 1.0
R_FULL_FRAC = 10.0  # jaw radius (in strip thicknesses) at which the radiused Kt is reached


LTB_SF = 1.5      # strip lateral-torsional buckling: required load factor on the static stress state


def strip_ltb_factor(fe: FEModel, st: State, elems: list) -> float:
    """Linear buckling load factor of one strip (element chain `elems`) at state st, both strip ends
    clamped: smallest alpha with (K_mat + alpha K_sigma) singular on the interior nodes. K_sigma is
    the beam's own consistent initial-stress Hessian (CR + Wagner term), so it carries the in-plane
    moment M3 (lateral-torsional buckling), the axial force and the actual M3 distribution
    (moment-gradient factor). Validated: uniform M3 on a 6-element 0.1x9 mm x8 strip gives
    M_cr 423 vs 2 pi/L sqrt(EIy GJ) = 389 mN m. inf if the state does not destabilise the strip."""
    from .beam import element_cr
    deg: dict[int, int] = {}
    for e in elems:
        deg[e.n0] = deg.get(e.n0, 0) + 1
        deg[e.n1] = deg.get(e.n1, 0) + 1
    inner = sorted(n for n, k in deg.items() if k > 1)
    if not inner:
        return float("inf")
    idx = {n: k for k, n in enumerate(inner)}
    m = 6 * len(inner)
    Km, Ks = np.zeros((m, m)), np.zeros((m, m))
    for e in elems:
        c = element_cr(e, st)
        Kme = c.B.T @ e.K7 @ c.B
        loc = np.array([6 * idx[n] + j if n in idx else -1 for n in (e.n0, e.n1) for j in range(6)])
        ok = loc >= 0
        ix = np.ix_(loc[ok], loc[ok])
        Km[ix] += Kme[np.ix_(ok, ok)]
        Ks[ix] += c.Kr[np.ix_(ok, ok)]
    Km, Ks = 0.5 * (Km + Km.T), 0.5 * (Ks + Ks.T)
    ev = sl.eigh(Ks, Km, eigvals_only=True)
    return float(-1.0 / ev.min()) if ev.min() < 0 else float("inf")


def strip_kt(f) -> float:
    """Clamp-edge stress concentration Kt alone (no fretting knock-down) -- applied to the mean."""
    t = f.thickness
    R = f.strip_clamp_radius if f.strip_clamp_radius > 0 else R_FULL_FRAC * t
    if f.strip_clamp == "sharp":
        return KT_SHARP
    if f.strip_clamp == "compliant":
        return KT_COMPLIANT
    x = min(1.0, R / (R_FULL_FRAC * t))
    return KT_RADIUSED + (KT_SHARP - KT_RADIUSED) * (1 - x)


def strip_clamp_factor(f) -> tuple[float, float]:
    """(effective fatigue multiplier Kt/fretting, jaw radius m) for the steel strip clamp."""
    t = f.thickness
    R = f.strip_clamp_radius if f.strip_clamp_radius > 0 else R_FULL_FRAC * t
    if f.strip_clamp == "sharp":
        return KT_SHARP / FRET_SHARP, 0.0
    if f.strip_clamp == "compliant":
        return KT_COMPLIANT / FRET_COMPLIANT, R
    x = min(1.0, R / (R_FULL_FRAC * t))
    kt = KT_RADIUSED + (KT_SHARP - KT_RADIUSED) * (1 - x)
    fr = FRET_RADIUSED + (FRET_SHARP - FRET_RADIUSED) * (1 - x)
    return kt / fr, R


def checks(model: RotorModel, ss: StaticState, hd: HarmonicData | None = None,
           forced: list | None = None) -> list[Check]:
    """forced: forced.Crossing list (nP resonant responses), added to the fatigue checks."""
    from . import forced as frc
    fe = model.fe
    st = ss.state
    sa = assemble_internal(fe, st, False)
    groups: dict[str, dict] = {}
    used: dict[str, set] = {}
    fz = ss.Omega / (2 * np.pi)
    ff = model.design.flexure
    kfac, Rj = strip_clamp_factor(ff)
    gm = ff.strip_fatigue_mean == "goodman"
    kt_mean = strip_kt(ff) if gm else 1.0
    gdiag: dict[str, dict] = {}
    cables = {k + 1: {f"flex{j}" for j, m in enumerate(bi.members) if m.cable}
              for k, bi in enumerate(model.blades)}
    chains: dict[tuple, dict] = {}
    for i, (e, c) in enumerate(zip(fe.elements, sa.cr)):
        if e.kind in ("rigid", "spring") or e.massless:
            continue
        name = e.tag.split(".", 1)[1] if "." in e.tag else e.tag
        if e.kind == "blade":
            name = "blade"
        g = groups.setdefault(name, dict(static=0.0, il=0.0, creep=0.0, fat_e=0.0, fat_s=0.0,
                                         strain=0.0, famp=0.0, det=None, fat_e_n=0.0, fat_s_n=0.0,
                                         famp_n=0.0, cross_n=""))
        t_hat = unit(st.x[e.n1] - st.x[e.n0])
        is_cable = e.kind == "flexure" and name in cables.get(e.blade, ())
        fa, fb = element_end_forces(e, c, st)
        if e.kind == "flexure" and not is_cable:
            ch = chains.setdefault((e.blade, name), dict(els=[], M3=0.0, L=0.0))
            ch["els"].append(e)
            ch["M3"] = max(ch["M3"], abs(float(fa[5])), abs(float(fb[5])))
            ch["L"] += e.L0
        secs = [e.sections[0], e.sections[-1]]
        used.setdefault(name, set()).update(fb_.material for sec in secs for fb_ in sec.fibres)
        for sec, sf in zip(secs, (fa, fb)):
            u_s, u_il, strain, det, u_cr = _fibre_checks(e, sec, sf, t_hat, is_cable)
            if u_s > g["static"]:
                g["static"], g["det"] = u_s, det
            g["il"] = max(g["il"], u_il)
            g["creep"] = max(g["creep"], u_cr)
            g["strain"] = max(g["strain"], strain)
        if hd is not None:
            du = hd.U[_edofs(e)]
            fa_c, fb_c = element_end_forces_lin(e, c, du, st)
            means = [_fibre_mean(sec, sf0) if gm else None for sec, sf0 in zip(secs, (fa, fb))]
            gd = gdiag.setdefault(name, {})
            for sec, sf, mn in zip(secs, (fa_c, fb_c), means):
                ue, us, amp = _fibre_amp(sec, sf, t_hat, fz, is_cable, mn, kt_mean, gd)
                g["fat_e"] = max(g["fat_e"], ue)
                g["fat_s"] = max(g["fat_s"], us)
                g["famp"] = max(g["famp"], amp)
            for cr in forced or ():
                fa_c, fb_c = element_end_forces_lin(e, c, cr.U[_edofs(e)], st)
                for sec, sf, mn in zip(secs, (fa_c, fb_c), means):
                    ue, us, amp = _fibre_amp(sec, sf, t_hat, cr.freq_hz, is_cable, mn, kt_mean, gd)
                    if ue > g["fat_e_n"]:
                        g["cross_n"] = f"{cr.label} {cr.per_rev:.2f}/rev x {cr.n}P"
                    g["fat_e_n"] = max(g["fat_e_n"], ue)
                    g["fat_s_n"] = max(g["fat_s_n"], us)
                    g["famp_n"] = max(g["famp_n"], amp)
    shim_groups = [n for n, ms in used.items() if any(m.startswith(LAMINABLE_SHEETS) for m in ms)]
    for n in shim_groups:
        for k_ in ("fat_e", "fat_s", "famp", "fat_e_n", "fat_s_n", "famp_n"):
            groups[n][k_] *= kfac
    model.np_info = {}
    for n in shim_groups:
        g = groups[n]
        E = get_material([m for m in used[n] if m.startswith(LAMINABLE_SHEETS)][0]).E1
        model.np_info[n] = dict(strain=g["famp_n"], stress=g["famp_n"] * E, util=g["fat_e_n"],
                                strain_1p=g["famp"], which=g["cross_n"])
    for g in groups.values():
        g["fat_e1"], g["fat_s1"] = g["fat_e"], g["fat_s"]
        g["fat_e"] = frc.combine(g["fat_e"], g["fat_e_n"])
        g["fat_s"] = frc.combine(g["fat_s"], g["fat_s_n"])
    out: list[Check] = []
    T_part = float(model.design.op.ambient_C)
    for name, mats in used.items():
        lim = [(_get_raw(m).hdt_C - HDT_MARGIN_C, m) for m in mats if _get_raw(m).hdt_C > 0]
        if lim:
            a, m = min(lim)
            out.append(Check(name="hdt_temperature", member=name, value=T_part, allowable=a,
                             utilisation=T_part / a if a > 0 else float("inf"),
                             note=f"part T = ambient (no self-heating) vs HDT {a + HDT_MARGIN_C:.0f} C "
                                  f"- {HDT_MARGIN_C:.0f} C margin ({m}); moduli/allowables derated "
                                  "linearly to 0.3 at HDT"))
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
                             utilisation=g["fat_e"], note="1/rev strain amplitude (design torque)" + (
                                 f" {frc.FATIGUE_COMBINE} nP crossing: util 1P {g['fat_e1']:.3f} + nP "
                                 f"{g['fat_e_n']:.3f} ({g['cross_n']})" if g["fat_e_n"] > 0 else "")))
            gdn = gdiag.get(name, {})
            gtxt = ""
            if gm and "sm" in gdn:
                gtxt = (f" [Goodman: binding lamina sm={gdn['sm']/1e6:.0f} MPa sa={gdn['sa']/1e6:.0f} MPa "
                        f"Se={gdn['se']/1e6:.0f} Su={gdn['su']/1e6:.0f} g={gdn['ge']:.2f} "
                        f"SWT={gdn['swt']:.2f}]")
            out.append(Check(name="fatigue_stress", member=name, value=g["fat_s"], allowable=1.0,
                             utilisation=g["fat_s"], note="1/rev stress amp / (ratio × strength)" + gtxt + (
                                 f" {frc.FATIGUE_COMBINE} nP: 1P {g['fat_s1']:.3f} + nP {g['fat_s_n']:.3f}"
                                 if g["fat_s_n"] > 0 else "")))
    for n in shim_groups:
        g = groups[n]
        mat = get_material([m for m in used[n] if m.startswith(LAMINABLE_SHEETS)][0])
        if ff.strip_clamp == "sharp" or Rj <= 0:
            continue
        kap = 2 * g["strain"] / ff.thickness           # peak operating curvature (conservative: incl. axial)
        sw = mat.E1 * ff.thickness / (2 * Rj)
        # jaw only governs if the strip is forced onto it (operating curvature > 1/R)
        u = sw / mat.Xt if kap > 1.0 / Rj else 0.0
        out.append(Check(name="strip_clamp_wrap", member=n, value=sw, allowable=mat.Xt, utilisation=u,
                         note=f"jaw R={Rj*1e3:.2f} mm, wrap stress E t/2R={sw/1e6:.0f} MPa vs Xt; "
                              f"active only if kappa_op={kap:.0f}>1/R={1/Rj:.0f} 1/m; "
                              f"fatigue x{kfac:.2f} ({ff.strip_clamp})"))
    for n in shim_groups:     # lateral-torsional buckling of the thin strip under its in-plane M3
        worst = None
        for (b, nm), ch in chains.items():
            if nm != n:
                continue
            a = strip_ltb_factor(fe, st, ch["els"])
            if worst is None or a < worst[0]:
                worst = (a, b, ch)
        if worst is None:
            continue
        a, b, ch = worst
        s0 = ch["els"][0].sections[0]
        mcr_u = 2 * np.pi / ch["L"] * np.sqrt(s0.EIy * s0.GJ)
        mcr = a * ch["M3"]
        out.append(Check(name="strip_ltb", member=n, value=ch["M3"], allowable=mcr / LTB_SF,
                         utilisation=LTB_SF / a,
                         note=f"b{b}: max|M3| {ch['M3']*1e3:.0f} mN m, beam buckling factor {a:.2f} "
                              f"(M_cr,eff {mcr*1e3:.0f}; uniform-M clamped 2pi/L sqrt(EIy GJ) "
                              f"{mcr_u*1e3:.0f}); util = M3/(M_cr,eff/{LTB_SF:g})"))
    f = model.design.flexure
    if f.strap_strands > 0 and model.blades:
        cable_tags = {f"flex{j}" for j, m in enumerate(model.blades[0].members) if m.cable}
        Nmax = 0.0
        for e, c in zip(fe.elements, sa.cr):
            if e.kind == "flexure" and e.tag.split(".", 1)[-1] in cable_tags:
                fa, fb = element_end_forces(e, c)
                Nmax = max(Nmax, float(abs(fa[0])), float(abs(fb[0])))
        mat = get_material(f.strap_material)
        P = mat.Xt * np.pi * f.strap_strand_diameter ** 2 / 4
        out.append(Check(name="strap_tension", member="strap", value=Nmax, allowable=P,
                         utilisation=Nmax / P, note=f"max strand tension vs strength ({f.strap_material})"))
        cap, note = strap_anchor_capacity(f)
        out.append(Check(name="strap_anchor", member="strap", value=Nmax, allowable=cap,
                         utilisation=Nmax / cap, note=note))
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
            cap = mot.capability(model.design.motor, ss.rpm, float(ss.trim.Q), ambient_C=model.design.op.temperature_C)
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
    try:   # soft check: root hardware height vs local airfoil thickness
        from .rootdrag import envelope
        hmax, allow = envelope(model.design)
        if allow > 0 and hmax > 0:
            out.append(Check(name="root_envelope", member="root", value=hmax, allowable=allow,
                             utilisation=hmax / allow,
                             note=f"hardware height {hmax*1e3:.1f} mm vs {model.design.flexure.root_envelope_ratio:g} x "
                                  f"root airfoil thickness (soft)"))
    except Exception as ex:   # pragma: no cover
        model.warnings.append(f"root envelope check failed: {ex}")
    try:   # soft check: clamped-strip fittings clearance over the lag/pitch range (cached, ~0.5 s)
        from .fittings import clearance_check
        c = clearance_check(model.design)
        if c is not None:
            out.append(c)
    except Exception as ex:   # pragma: no cover
        model.warnings.append(f"fitting clearance check failed: {ex}")
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


def _zeta_check(name: str, z: float, zmin: float, note: str) -> Check:
    util = 1e3 if not (np.isfinite(z) and z > 0) else min(1e3, zmin / z)
    return Check(name=name, member="stability", value=float(z) if np.isfinite(z) else -1.0,
                 allowable=zmin, utilisation=float(util), note=note)


def _stability(model: RotorModel, ss: StaticState, lin: Linear, aero, md: ModeData, res,
               summary: dict) -> list[Check]:
    """Mode damping per analysis.stability (energy | eig | floquet), the 1P-proximity metrics
    and the aeroelastic_stability / air_resonance checks (see stability.py)."""
    from . import stability as stb
    an = model.design.analysis
    Om = ss.Omega
    P = Om / (2 * np.pi)
    fr = np.array(md.freqs_hz, float)
    zt = np.array(md.damping, float)
    if an.stability != "energy":
        s, _, _ = stb.damped_modes(model, ss, lin, aero, fr, n_basis=an.stability_basis)
        el = np.abs(s) > 0.05 * Om
        fr = np.where(el, np.abs(s.imag) / (2 * np.pi), fr)
        zt = np.where(el, stb.zeta_of(s), zt)
        for m_, z_, f_ in zip(res.modes, zt, fr):
            m_.participation["zeta_energy"] = float(m_.damping_ratio)
            m_.damping_ratio, m_.freq_hz = float(z_), float(f_)
    el = fr > 0.05 * P
    out: list[Check] = []
    if el.any():
        i = int(np.argmin(np.where(el, zt, np.inf)))
        summary.update(zeta_min=float(zt[i]), zeta_min_per_rev=float(fr[i] / P))
        out.append(_zeta_check("aeroelastic_stability", float(zt[i]), an.min_damping,
                               f"min damping ({an.stability}) {zt[i]:.4f} of '{md.labels[i]}' at "
                               f"{fr[i] / P:.3f}/rev"))
        j = int(np.argmin(np.where(el, np.abs(fr / P - 1.0), np.inf)))
        pj = float(fr[j] / P)
        summary.update(near_1p_per_rev=pj, near_1p_zeta=float(zt[j]),
                       near_1p_amp=stb.amp_1p(pj, float(zt[j])),
                       near_1p_margin_eff=max(abs(pj - 1), stb.margin_1p_eff(pj, float(zt[j]))))
        for key in ("lag", "pitch"):
            for k, lab in enumerate(md.labels):
                if lab.startswith(key) and el[k]:
                    summary[f"{key}_zeta"] = float(zt[k])
                    break
    af = model.design.op.airframe
    if af is not None and an.stability != "energy":
        fl = None if an.stability == "floquet" else False
        za, sa_, _, afm = stb.air_resonance(model, ss, lin, aero, af, an.stability_basis, floquet=fl)
        iso = stb.isotropic(afm)
        how = "LTI (isotropic support, exact)" if iso else ("Floquet" if fl is None else
                                                            "LTI on the averaged support (approx.)")
        summary.update(air_resonance_zeta=float(za))
        fz = abs(sa_.imag) / (2 * np.pi) if sa_ is not None else float("nan")
        out.append(_zeta_check("air_resonance", float(za), an.min_damping,
                               f"rotor + in-plane hub support f={af.f_hz:g}/{af.f_y_hz or af.f_hz:g} Hz "
                               f"zeta_af={af.zeta:g}: min zeta {za:.4f} at {fz:.1f} Hz (rotating "
                               f"frame), {how}"))
    return out


def _forced_np(model: RotorModel, ss: StaticState, lin: Linear, md: ModeData, res, hd: HarmonicData,
               summary: dict):
    """nP (n >= 2) resonant crossing responses (forced.py) + the harmonic_jitter check."""
    from . import forced as frc
    fr = np.array([m.freq_hz for m in res.modes], float) if res.modes else md.freqs_hz
    zt = np.array([m.damping_ratio for m in res.modes], float) if res.modes else md.damping
    xs = frc.crossings(model, ss, lin, md, zt, fr, hd.dQ, model.design.op.vehicle_mass)
    for c in xs:
        c.jitter_deg = frc.pitch_jitter_deg(model, c.U)
    p1 = float(hd.result.blades[0].pitch_amp_deg) if hd.result.blades else 0.0
    allow = max(frc.JITTER_MAX_DEG, frc.JITTER_FRAC_1P * p1)
    summary["np_crossings"] = float(len(xs))
    out = []
    if xs:
        w = max(xs, key=lambda c: c.jitter_deg)
        summary.update(np_jitter_deg=w.jitter_deg, np_jitter_per_rev=w.per_rev, np_jitter_n=float(w.n),
                       np_jitter_zeta=w.zeta)
        out.append(Check(name="harmonic_jitter", member="blade", value=w.jitter_deg, allowable=allow,
                         utilisation=w.jitter_deg / allow,
                         note=f"worst nP crossing '{w.label}' {w.per_rev:.2f}/rev x {w.n}P, zeta {w.zeta:.4f}, "
                              f"Q={1 / (2 * w.zeta):.0f}; modal parts " +
                              ", ".join(f"{k} {v:.3g}" for k, v in w.q_parts.items()) +
                              f"; limit max({frc.JITTER_MAX_DEG} deg, {frc.JITTER_FRAC_1P:g} x 1P {p1:.2f} deg)"))
    else:
        summary["np_jitter_deg"] = 0.0
    return xs, out


HDT_MARGIN_C = 10.0   # part temperature must stay below HDT - margin


def evaluate(design: Design, what: list[str] | None = None) -> Results:
    """Evaluate at part temperature = op.ambient_C: moduli and allowables of every material
    are derated by materials.temp_knockdown for the whole analysis."""
    with part_temperature(design.op.ambient_C):
        return _evaluate(design, what)


def _evaluate(design: Design, what: list[str] | None = None) -> Results:
    what = list(what or ["trim", "modes", "harmonic", "checks"])
    t = {}
    t0 = time.perf_counter()
    res = Results(design_hash=design.hash())
    pl_info: dict = {}
    if design.flexure.pre_lag_deg == "auto":
        design = resolve_pre_lag(design, info=pl_info)
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
    summary = dict(rpm=ss.rpm, Omega=ss.Omega, trim_ok=bool(ss.trim_ok))
    if ss.trim is not None and ss.blade_pitch_el is not None and len(model.blades) >= 2:
        dp, lag = split_and_lag(model, ss)
        summary["trim_diff_pitch_deg"] = float(np.degrees(dp))
        summary["pre_lag_deg"] = float(np.degrees(pre_lag_rad(design)))
        summary["hinge_lag_el_deg"] = float(np.degrees(lag))
        if pl_info.get("slope") is not None and np.isfinite(pl_info["slope"]):
            summary["pre_lag_split_slope"] = pl_info["slope"]
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
        try:
            pc = pitch_cf(model, ss.Omega, res.flexure_stiffness)
            res.summary.update(pc)
            if pc["pitch_cf_ratio"] > PITCH_CF_RATIO_MAX:
                res.warnings.append(f"pitch_cf_ratio {pc['pitch_cf_ratio']:.2f} > {PITCH_CF_RATIO_MAX}: CF "
                                    "propeller moment dominates the pitch stiffness through the hinge "
                                    "(blade driven to flat pitch)")
        except Exception as ex:   # noqa: BLE001
            res.warnings.append(f"pitch_cf failed: {ex}")
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
    stab_checks: list[Check] = []
    if md is not None:
        t1 = time.perf_counter()
        try:
            stab_checks = _stability(model, ss, lin, aero, md, res, summary)
        except Exception as ex:   # pragma: no cover
            res.warnings.append(f"stability analysis failed: {ex}")
        t["stability"] = time.perf_counter() - t1
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
    forced_x = None
    if md is not None and hd is not None and "checks" in what:
        t1 = time.perf_counter()
        try:
            forced_x, fchecks = _forced_np(model, ss, lin, md, res, hd, summary)
            stab_checks = stab_checks + fchecks
        except Exception as ex:   # pragma: no cover
            res.warnings.append(f"nP forced response failed: {ex}")
        t["forced_np"] = time.perf_counter() - t1
    if "checks" in what:
        t1 = time.perf_counter()
        res.checks = checks(model, ss, hd, forced_x) + stab_checks
        _ni = getattr(model, "np_info", {})
        if _ni:
            _w = max(_ni.values(), key=lambda v: v["stress"])
            summary.update(np_strip_strain_amp=float(_w["strain"]), np_strip_stress_MPa=float(_w["stress"] / 1e6),
                           np_strip_fat_util=float(_w["util"]), strip_strain_amp_1p=float(_w["strain_1p"]))
        _pr = res.summary.get("pitch_cf_ratio")
        if _pr is not None:
            _pr = float(_pr) if np.isfinite(_pr) else 1e3
            res.checks.append(Check(name="pitch_cf_ratio", member="blade", value=_pr,
                                    allowable=PITCH_CF_RATIO_MAX, utilisation=_pr / PITCH_CF_RATIO_MAX,
                                    note="CF propeller-moment pitch stiffness / effective pitch stiffness "
                                         "through the skewed hinge (>1: blade driven flat, trim fails)"))
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
            struct = [c for c in res.checks if c.member not in ("esc", "motor", "stability")
                      and not c.member.startswith("print:")]
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
