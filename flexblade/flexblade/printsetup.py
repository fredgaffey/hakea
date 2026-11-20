"""Print orientation as a design variable + printability / heat-forming checks.

`Design.print_setup` holds ONE build direction for the whole multi-material print (frame H),
per-region in-layer raster angles, pause inserts and optional post-print heat-forming ops.
`apply_print_setup` derives every part's PrintOrientation (blade, flexure, members, hub) so the
anisotropic strength/stiffness models and the printability checks see the same print.
print_setup=None -> per-part orientations stay as given (legacy behaviour).

State model: AS-PRINTED = build_dir + raster angles (checks for inserts, overhang, bed are
evaluated here); FINAL = region rigid-rotation R_form applied to the as-printed orientation
(this is what strength/stiffness see). Forming residual stress is not modelled.

Conventions: raster angle 0 = projection of the blade span (+x) into the layer plane (fallback
+y), right-handed about build_dir. Overhang is measured from vertical: asin(max(0, -n·b));
0 = wall, 90 = ceiling. Inserts (pultruded rod, steel) have their axis as raster direction and
are not printed (no overhang / wall for them).
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from .materials import get_material
from .schema import Check, Design, FlexureMember, FormOp, PrintOrientation

INSERT_MATERIALS = ("cf_rod_pultruded", "steel_302_spring")
BED = (0.270, 0.270, 0.270)          # m; Snapmaker U1 (printers.py in git history)
MAX_OVERHANG_DEG = 50.0
INSERT_TOL_DEG = 5.0
MIN_WALL = 0.5e-3
SEAT_TOL = 1.0e-3                    # a face this close to the bed plane sits on it (<= ~3 layers: the flexure can be offset onto the bed)
BRIDGE_MAX = 10e-3                   # m, flat ceiling spanning <= this between supports is bridged (counted as 0.9 x allowable)
FORM_NOTE = "forming residual stress not modelled"


def _u(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def is_insert_material(name: Optional[str]) -> bool:
    return name in INSERT_MATERIALS


def rot(axis, ang_deg: float) -> np.ndarray:
    k = _u(axis)
    t = math.radians(ang_deg)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


def hinge_a(design: Design) -> np.ndarray:
    d = math.radians(design.flexure.skew_deg)
    return np.array([math.sin(d), 0.0, -math.cos(d)])


# --------------------------------------------------------------------------- build / forms


def effective_forms(design: Design) -> list[FormOp]:
    ps = design.print_setup
    if ps.form or ps.preset != "formed_torsion_bar":
        return list(ps.form)
    # printed flat: bar along x in-plane; twist 90° about the bar (x), then bend about y so the
    # bar (unbent segment) ends on the skewed hinge axis a = (sinδ, 0, -cosδ)
    return [FormOp(region="flexure", kind="twist", axis=(1.0, 0.0, 0.0), angle_deg=90.0),
            FormOp(region="flexure", kind="bend", axis=(0.0, 1.0, 0.0),
                   angle_deg=90.0 - design.flexure.skew_deg,
                   radius=max(0.006, 3.5 * design.flexure.thickness), inserts_in_unbent=True)]


def effective_pauses(design: Design) -> list[str]:
    ps = design.print_setup
    extra = ["bar", "rod_insert", "tension_rod", "tension_member"] if ps.preset == "formed_torsion_bar" else []
    return list(ps.pauses) + extra


def form_rotation(design: Design, region: str) -> np.ndarray:
    R = np.eye(3)
    for op in effective_forms(design):
        if op.region == region:
            R = rot(op.axis, op.angle_deg) @ R
    return R


def tilt_build_dir(tilt_deg: float, azimuth_deg: float = 0.0) -> np.ndarray:
    """Continuous build direction from two angles: tilt from the rotor axis z, azimuth about z
    from +x (tilt 0 = flat, 90 = lying on its side)."""
    t, a = math.radians(tilt_deg), math.radians(azimuth_deg)
    return np.array([math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), math.cos(t)])


STOCK_DIAMETERS_MM = (1.0, 1.5, 2.0, 2.38, 3.0, 3.175)     # rod / drill stock (3/32", 1/8" incl.)
DRILL_LEN_PER_D = 12.0


def snap_diameter(d_m: float) -> float:
    """Nearest standard stock/drill diameter (m)."""
    return min(STOCK_DIAMETERS_MM, key=lambda x: abs(x - d_m * 1e3)) * 1e-3


def install_mode(design: Design, name: str) -> str:
    ps = design.print_setup
    if name in ps.install:
        return ps.install[name]
    if name == "bar":
        return "pause" if ps.preset == "formed_torsion_bar" else "drilled"
    return "pause"


def _build_from_phi(design: Design, phi: float) -> np.ndarray:
    a = hinge_a(design)
    y = np.array([0.0, 1.0, 0.0])
    n = _u(np.cross(a, y))
    return math.cos(phi) * y + math.sin(phi) * n


def resolve_build_dir(design: Design) -> np.ndarray:
    ps = design.print_setup
    if ps.build_dir is not None:
        return _u(ps.build_dir)
    if ps.preset in ("flat", "formed_torsion_bar", "drilled_torsion_bar"):
        return np.array([0.0, 0.0, 1.0])
    if ps.preset == "tilted":
        return tilt_build_dir(ps.tilt_deg, ps.azimuth_deg)
    if ps.preset == "on_edge":
        return np.array([0.0, 1.0, 0.0])
    # hinge_in_plane: b ⟂ a, rotate about a to minimise flexure overhang (as printed)
    from . import fact
    members = fact.generate_members(design.model_copy(update=dict(print_setup=None)))
    R = form_rotation(design, "flexure")
    best, bphi = None, 0.0
    for phi in np.radians(np.arange(0.0, 360.0, 5.0)):
        b = _build_from_phi(design, phi)
        oh = max([_member_overhang(_asprinted(m, R), b, None) for m in members] or [0.0])
        if best is None or oh < best - 1e-9:
            best, bphi = oh, phi
    return _build_from_phi(design, bphi)


def _ref_dir(b: np.ndarray) -> np.ndarray:
    r = np.array([1.0, 0, 0]) - b * b[0]
    if np.linalg.norm(r) < 1e-6:
        r = np.array([0.0, 1, 0]) - b * b[1]
    return _u(r)


def _raster(b: np.ndarray, ang_deg: float) -> np.ndarray:
    e1 = _ref_dir(b)
    t = math.radians(ang_deg)
    return math.cos(t) * e1 + math.sin(t) * np.cross(b, e1)


def _ori(b, r) -> PrintOrientation:
    return PrintOrientation(build_dir=tuple(map(float, b)), raster_dir=tuple(map(float, r)))


def region_orientations(design: Design) -> dict[str, PrintOrientation]:
    """FINAL blade / flexure / hub orientations (frame H, blade 1): R_form · as-printed."""
    ps, b = design.print_setup, resolve_build_dir(design)
    Rb, Rf = form_rotation(design, "blade"), form_rotation(design, "flexure")
    return dict(blade=_ori(Rb @ b, Rb @ _raster(b, ps.blade_raster_deg)),
                flexure=_ori(Rf @ b, Rf @ _raster(b, ps.flexure_raster_deg)),
                hub=_ori(b, _raster(b, 0.0)))


def member_orientation(design: Design, m: FlexureMember, b=None) -> PrintOrientation:
    """FINAL orientation of a blade-1 member (axis/pos given in the final state)."""
    ps = design.print_setup
    b = resolve_build_dir(design) if b is None else b
    R = form_rotation(design, "flexure")
    if is_insert_material(m.layup.material):
        t = R.T @ _u(np.subtract(m.p1, m.p0))          # as-printed axis
        r = t - b * (t @ b)
        r = _raster(b, 0.0) if np.linalg.norm(r) < 1e-6 else _u(r)
    else:
        r = _raster(b, ps.flexure_raster_deg)
    return _ori(R @ b, R @ r)


def apply_print_setup(design: Design) -> Design:
    """Design copy whose blade/flexure orientations (and explicit members) follow print_setup.
    Idempotent; identity if print_setup is None. FACT members: see orient_generated_members."""
    if design.print_setup is None:
        return design
    ro = region_orientations(design)
    b = resolve_build_dir(design)
    f = design.flexure
    fx = f.model_copy(update=dict(
        orientation=ro["flexure"],
        members=[m.model_copy(update=dict(orientation=member_orientation(design, m, b))) for m in f.members],
        tension_member=(None if f.tension_member is None else f.tension_member.model_copy(
            update=dict(orientation=member_orientation(design, f.tension_member, b)))),
    ))
    return design.model_copy(update=dict(blade=design.blade.model_copy(update=dict(orientation=ro["blade"])),
                                         flexure=fx))


def orient_generated_members(design: Design, members: list[FlexureMember]) -> list[FlexureMember]:
    if design.print_setup is None:
        return members
    b = resolve_build_dir(design)
    drilled = {it["member"] for it in insert_items(design, members)
               if install_mode(design, it["name"]) == "drilled"}
    out = []
    for i, m in enumerate(members):
        o = member_orientation(design, m, b)
        if i in drilled:     # drilled/pultruded after printing: fibres along the final axis, any build
            ax = _u(np.subtract(m.p1, m.p0))
            bb = np.asarray(o.build_dir, float)
            bb = bb - ax * (bb @ ax)
            bb = _u(bb) if np.linalg.norm(bb) > 1e-6 else _u(np.cross(ax, [0.0, 0.0, 1.0]))
            o = _ori(bb, ax)
        out.append(m.model_copy(update=dict(orientation=o)))
    return out


# --------------------------------------------------------------------------- inserts / faces


def insert_items(design: Design, members: list[FlexureMember]) -> list[dict]:
    f = design.flexure
    out, n = [], len(members)
    for i, m in enumerate(members):
        axis = _u(np.subtract(m.p1, m.p0))
        if is_insert_material(m.layup.material):
            if f.topology == "torsion_bar" and i == 0:
                nm, enc = "bar", f.bar_with_plate
            elif f.tension_rod_diameter > 0 and i == n - 1:
                nm, enc = "tension_rod", False
            elif f.tension_member is not None and i == n - 1 - (1 if f.tension_rod_diameter > 0 else 0):
                nm, enc = "tension_member", False
            else:
                nm, enc = f"member{i}", False
            out.append(dict(name=nm, axis=axis, enclosed=enc, member=i))
        elif m.layup.kind == "rod_insert" and m.layup.insert_diameter > 0 and \
                is_insert_material(m.layup.insert_material):
            out.append(dict(name="rod_insert", axis=axis, enclosed=True, member=i))
    return out


def _is_printed(m: FlexureMember) -> bool:
    return m.kind != "rigid" and not is_insert_material(m.layup.material)


def _asprinted(m: FlexureMember, R: np.ndarray, Mk: Optional[np.ndarray] = None) -> FlexureMember:
    """Member with its direction vectors rotated back to the as-printed state (about its centre).
    R = region form rotation in the blade-1 frame; Mk maps blade-1 frame -> blade-k frame."""
    Rk = R if Mk is None else Mk @ R @ Mk.T
    p0, p1 = np.asarray(m.p0, float), np.asarray(m.p1, float)
    c, h = 0.5 * (p0 + p1), 0.5 * (p1 - p0)
    return m.model_copy(update=dict(p0=tuple(c - Rk.T @ h), p1=tuple(c + Rk.T @ h),
                                    width_dir=tuple(Rk.T @ np.asarray(m.width_dir, float))))


def _faces(m: FlexureMember):
    p0, p1 = np.asarray(m.p0, float), np.asarray(m.p1, float)
    t = _u(p1 - p0)
    w = np.asarray(m.width_dir, float)
    w = w - t * (t @ w)
    w = _u(w) if np.linalg.norm(w) > 1e-9 else _u(np.cross(t, [0.0, 0.0, 1.0]))
    return t, w, np.cross(t, w), 0.5 * (p0 + p1)


def _member_overhang(m: FlexureMember, b, hmin: Optional[float]) -> float:
    """Max overhang (deg from vertical) of a printed member's long faces; faces lying on the bed
    plane (hmin) are fine. Inserts / rigid -> 0."""
    if not _is_printed(m):
        return 0.0
    t, w, n, c = _faces(m)
    if m.shape == "circle":
        perp = math.sqrt(max(0.0, 1.0 - float(t @ b) ** 2))
        if hmin is not None and float(c @ b) - 0.5 * m.thickness <= hmin + SEAT_TOL and perp > 0.99:
            return 0.0
        return math.degrees(math.asin(min(1.0, perp)))
    worst = 0.0
    for nrm, half in ((n, m.thickness / 2), (w, m.width / 2)):
        for s in (1.0, -1.0):
            d = float(s * nrm @ b)
            ang = math.degrees(math.asin(min(1.0, max(0.0, -d))))
            if ang <= 0:
                continue
            if hmin is not None and abs(d) > 0.999 and float(c @ b) - half <= hmin + SEAT_TOL:
                continue
            if abs(d) > 0.999 and float(np.linalg.norm(np.subtract(m.p1, m.p0))) <= BRIDGE_MAX:
                ang = 0.9 * MAX_OVERHANG_DEG          # flat ceiling bridged between two supports
            worst = max(worst, ang)
    return worst


def bed_extent(pts: np.ndarray, b: np.ndarray) -> tuple[float, float, float]:
    """(height along b, bx, by) of the min-footprint rotated bounding box, m."""
    b = _u(b)
    u = _ref_dir(b)
    v = np.cross(b, u)
    P = np.column_stack([pts @ u, pts @ v])
    best = (np.inf, 0.0, 0.0)
    for phi in np.radians(np.arange(0.0, 90.0, 2.0)):
        c, s = math.cos(phi), math.sin(phi)
        bx = float(np.ptp(P[:, 0] * c + P[:, 1] * s))
        by = float(np.ptp(-P[:, 0] * s + P[:, 1] * c))
        if max(bx, by) < best[0]:
            best = (max(bx, by), bx, by)
    return float(np.ptp(pts @ b)), best[1], best[2]


def _Mk(k: int, n: int) -> np.ndarray:
    psi = 2 * math.pi * (k - 1) / n
    Rz = np.array([[math.cos(psi), -math.sin(psi), 0], [math.sin(psi), math.cos(psi), 0], [0, 0, 1.0]])
    return Rz @ np.diag([1.0, 1.0, -1.0 if k % 2 == 0 else 1.0])


def _asprinted_points(geo, design: Design, n: int) -> np.ndarray:
    """All part section points in the as-printed state (formed regions rotated back about
    their blade's hinge point)."""
    out = []
    Rf, Rb = form_rotation(design, "flexure"), form_rotation(design, "blade")
    for p in geo.parts:
        P = np.vstack([s.ring for s in p.sections])
        R = Rf if (p.role == "member" or "_flex" in p.name) else Rb if p.role == "blade" else None
        if R is not None and p.blade > 0 and not np.allclose(R, np.eye(3)):
            Rk = _Mk(p.blade, n) @ R @ _Mk(p.blade, n).T
            h0 = np.asarray(geo.hinge[p.blade][0], float)
            P = h0 + (P - h0) @ Rk      # Rk^T applied to row vectors
        out.append(P)
    return np.vstack(out)


# --------------------------------------------------------------------------- checks


def _region_thickness(design: Design, region: str) -> tuple[float, str]:
    if region == "flexure":
        return design.flexure.thickness, design.flexure.layup.material
    st = design.blade.stations
    c = st[0].chord if st else 0.3 * design.blade.radius
    return 0.12 * c, design.blade.layup.material


def print_checks(design: Design, geo=None, bed=BED):
    """Printability + forming checks of a design with print_setup.
    Returns (checks, summary floats, warnings). Insert/overhang/bed are evaluated AS PRINTED."""
    from . import geometry
    d = apply_print_setup(design)
    ps = d.print_setup
    b = resolve_build_dir(d)
    geo = geo or geometry.build_geometry(d)
    n = d.blade.n_blades
    Rf = form_rotation(d, "flexure")
    pts = _asprinted_points(geo, d, n)
    hmin = float((pts @ b).min())
    pauses = effective_pauses(d)
    forms = effective_forms(d)
    checks: list[Check] = []
    ins_err, oh_max = 0.0, 0.0
    for k, mem in geo.members.items():
        Mk = _Mk(k, n)
        ap = [_asprinted(m, Rf, Mk) for m in mem]
        for it in insert_items(d, ap):
            nm = f"b{k}.{it['name']}"
            mode = install_mode(d, it["name"])
            if mode == "drilled":
                dchecks, derr = _drill_checks(d, geo, mem, it, nm, k)
                checks += dchecks
                continue
            ang = math.degrees(math.asin(min(1.0, abs(float(it["axis"] @ b)))))
            checks.append(Check(name="insert_in_layer", member=nm, value=ang, allowable=INSERT_TOL_DEG,
                                utilisation=ang / INSERT_TOL_DEG,
                                note="as-printed angle of the straight insert out of the layer plane (deg)"))
            ins_err = max(ins_err, ang)
            # forming: how the pause-installed insert is carried through the ops of its region
            ax = it["axis"]
            for op in forms:
                if op.region != "flexure":
                    continue
                oa = Mk @ _u(op.axis)
                par = math.degrees(math.asin(min(1.0, float(np.linalg.norm(np.cross(ax, oa))))))
                ok = par <= INSERT_TOL_DEG or (op.kind == "bend" and op.inserts_in_unbent)
                checks.append(Check(
                    name="form_insert", member=f"{nm}.{op.kind}", value=par, allowable=INSERT_TOL_DEG,
                    utilisation=(par / INSERT_TOL_DEG if par <= INSERT_TOL_DEG else (0.5 if ok else 2.0)),
                    note=f"{op.kind}: twist axis must be the rod axis (rod = mandrel, stays straight)"
                         + ("; bend: rod parallel to the bend line or in an unbent segment (asserted)"
                            if op.kind == "bend" else "") + "; pultruded CF/steel cannot be bent"))
                ax = Rk_apply(op, ax, Mk)
                _, mat_r = _region_thickness(d, op.region)
                m_r, m_i = get_material(mat_r), get_material(mem[it["member"]].layup.material
                                                             if it["name"] != "rod_insert" else
                                                             mem[it["member"]].layup.insert_material)
                if m_r.forming_temp_C > 0 and m_i.tg_C > 0 and k == 1:
                    checks.append(Check(
                        name="form_temp", member=f"{nm}.{op.kind}", value=m_r.forming_temp_C,
                        allowable=m_i.tg_C, utilisation=m_r.forming_temp_C / m_i.tg_C,
                        note=f"forming temp of {mat_r} ({m_r.forming_temp_C:.0f} C, assumed) vs insert Tg "
                             f"{m_i.tg_C:.0f} C (assumed): above it the rod must be inserted after forming (drilled)"))
        for i, m in enumerate(ap):
            if not _is_printed(m):
                continue
            oh = _member_overhang(m, b, hmin)
            oh_max = max(oh_max, oh)
            checks.append(Check(name="overhang", member=f"b{k}.flex{i}", value=oh, allowable=MAX_OVERHANG_DEG,
                                utilisation=oh / MAX_OVERHANG_DEG,
                                note="as-printed max face overhang from vertical (deg); > allowable = "
                                     "supports inside a flexure member"))
            wall = min(m.thickness * min(1.0, m.taper), m.width) if m.shape == "rect" else \
                m.thickness * min(1.0, m.taper)
            lay = m.layup
            if lay.kind == "core_skin" and lay.skin_thickness > 0:
                wall = min(wall, lay.skin_thickness)
            checks.append(Check(name="min_wall", member=f"b{k}.flex{i}", value=wall, allowable=MIN_WALL,
                                utilisation=min(99.0, MIN_WALL / wall) if wall > 0 else 99.0,
                                note="thinnest printed wall (must be >= allowable)"))
    lay = d.blade.layup
    if lay.kind == "core_skin" and lay.skin_thickness > 0:
        checks.append(Check(name="min_wall", member="blade", value=lay.skin_thickness, allowable=MIN_WALL,
                            utilisation=MIN_WALL / lay.skin_thickness))
    height, bx, by = bed_extent(pts, b)
    u = max(height / bed[2], bx / bed[0], by / bed[1])
    checks.append(Check(name="bed_fit", member="rotor", value=max(height, bx, by), allowable=min(bed),
                        utilisation=u, note=f"as-printed height {height*1e3:.0f} mm, footprint "
                        f"{bx*1e3:.0f}x{by*1e3:.0f} mm vs bed {bed[0]*1e3:.0f}x{bed[1]*1e3:.0f}x{bed[2]*1e3:.0f} mm"))
    # forming: material + bend radius
    warnings = []
    for op in forms:
        t, mat = _region_thickness(d, op.region)
        tpu = mat.startswith("tpu")
        if is_insert_material(mat):
            fu, why = 2.0, "pultruded/steel cannot be heat-formed"
        elif tpu:
            fu, why = 0.3, "TPU forms readily"
        else:
            fu, why = 0.7, "formable with heat (PA-CF/PETG/PLA): keep below Tg+, watch fibre/layer cracking"
        checks.append(Check(name="form_material", member=f"{op.region}.{op.kind}", value=fu, allowable=1.0,
                            utilisation=fu, note=f"{mat}: {why}; {FORM_NOTE}"))
        if op.kind == "bend":
            k = 3.0 if tpu else 6.0
            checks.append(Check(name="form_bend_radius", member=f"{op.region}", value=op.radius,
                                allowable=k * t, utilisation=k * t / op.radius,
                                note=f"bend radius must be >= {k:.0f}x thickness ({t*1e3:.2f} mm); {FORM_NOTE}"))
    if forms:
        warnings.append("heat-forming modelled as rigid region rotation of orientation; " + FORM_NOTE)
    for c in checks:
        c.member = "print:" + c.member
    summary = dict(print_insert_err_deg=ins_err, print_overhang_max_deg=oh_max, print_height_mm=height * 1e3,
                   print_bed_util=u, print_max_util=max(c.utilisation for c in checks),
                   print_build_x=float(b[0]), print_build_y=float(b[1]), print_build_z=float(b[2]))
    return checks, summary, warnings


def Rk_apply(op: FormOp, ax: np.ndarray, Mk: np.ndarray) -> np.ndarray:
    """Insert axis after one more form op (axis of the op in blade-k frame)."""
    return rot(Mk @ _u(op.axis), op.angle_deg) @ ax


# --------------------------------------------------------------------------- drilled inserts


def _hulls(geo):
    c = getattr(geo, "_hulls", None)
    if c is None:
        from scipy.spatial import Delaunay
        c = []
        for p in geo.parts:
            if p.role == "member":
                continue
            P = np.vstack([s.ring for s in p.sections])
            try:
                c.append((p.name, Delaunay(P)))
            except Exception:   # noqa: BLE001  degenerate (flat) part
                pass
        try:
            geo._hulls = c
        except Exception:   # noqa: BLE001
            pass
    return c


def _drill_checks(design: Design, geo, mem, it, nm: str, k: int):
    """Post-drilled insert, FINAL geometry: hole is straight (insert axis), entry on an outer
    surface (ray out of an end leaves the part without re-entering), drillable length <= 12 d,
    wall around the hole, stock diameter."""
    m = mem[it["member"]]
    f = design.flexure
    d = (m.layup.insert_diameter if it["name"] == "rod_insert" else m.thickness)
    p0, p1 = np.asarray(m.p0, float), np.asarray(m.p1, float)
    ax = _u(p1 - p0)
    Lb = float(np.linalg.norm(p1 - p0))
    hulls = _hulls(geo)
    ts = np.arange(0.0, 0.25, 0.5e-3)
    best = None
    for end, sgn in ((p1, 1.0), (p0, -1.0)):
        P = end[None, :] + sgn * ts[:, None] * ax[None, :]
        inside = np.zeros(len(ts), bool)
        for _, dl in hulls:
            inside |= dl.find_simplex(P) >= 0
        idx = np.nonzero(inside)[0]
        if len(idx) == 0:
            tex, ok = 0.0, True                       # free end, open to air
        else:
            # first exit after the host: first outside sample after the leading inside run
            run = 0
            while run < len(ts) and inside[run]:
                run += 1
            ok = not inside[run:].any()
            tex = ts[min(run, len(ts) - 1)]
        length = Lb + tex
        if ok and (best is None or length < best[0]):
            best = (length, tex)
    out: list[Check] = []
    if best is None:
        out.append(Check(name="drill_access", member=nm, value=0.0, allowable=1.0, utilisation=2.0,
                         note="neither end of the hole line exits the part without re-entering material"))
        length = Lb
    else:
        length = best[0]
        out.append(Check(name="drill_access", member=nm, value=best[1], allowable=1.0, utilisation=0.0,
                         note=f"entry accessible, {best[1]*1e3:.1f} mm of host to drill through"))
    out.append(Check(name="drill_length", member=nm, value=length, allowable=DRILL_LEN_PER_D * d,
                     utilisation=length / (DRILL_LEN_PER_D * d),
                     note="drilled length (insert + host) <= 12 x diameter (drill wander); "
                          "hole straight in the final formed geometry by construction"))
    if it["enclosed"] or it["name"] == "rod_insert":
        host_t = f.thickness if it["name"] != "rod_insert" else m.thickness
        wall = (host_t - d) / 2
        out.append(Check(name="drill_wall", member=nm, value=max(wall, 0.0), allowable=MIN_WALL,
                         utilisation=min(99.0, MIN_WALL / wall) if wall > 1e-9 else 99.0,
                         note="TPU/plate wall around the hole (host thickness - d)/2 >= 0.5 mm"))
    dd = abs(d - snap_diameter(d)) * 1e3
    out.append(Check(name="stock_diameter", member=nm, value=d, allowable=snap_diameter(d),
                     utilisation=0.0 if dd < 0.02 else 2.0,
                     note="nearest stock: " + ", ".join(f"{x:g}" for x in STOCK_DIAMETERS_MM) + " mm"))
    return out, 0.0
