"""Iso-surface extraction, STL/STEP export and matplotlib renders of a topopt result.

Iso-surfaces: skimage.measure.marching_cubes if installed, else the built-in vectorised
marching tetrahedra (each voxel cell split into 6 Kuhn tetrahedra along its main diagonal,
consistent across cells → watertight; vertices merged on grid edges; triangles oriented
outward). Per material: soft = ρ(1−φ), stiff = ρφ (+ hub/plate interface pads as stiff).
STEP: AP203 faceted B-rep (FACETED_BREP of planar triangular FACE_SURFACEs), one product
per material, millimetres.
"""
from __future__ import annotations

import datetime
import itertools
from pathlib import Path

import numpy as np

from .grid import OFFSETS

# 6 Kuhn tetrahedra of the unit cell: 0 → e_i → e_i+e_j → 7 for each axis permutation
_E = {0: 4, 1: 2, 2: 1}                       # axis → local-node bit (n = 4a+2b+c)
KUHN = np.array([[0, _E[i], _E[i] + _E[j], 7] for i, j, k in itertools.permutations(range(3))])


def _case_table():
    """For each of the 16 inside/outside patterns: list of triangles, each a triple of
    tet edges (va, vb)."""
    tab = []
    for case in range(16):
        ins = [v for v in range(4) if case >> v & 1]
        out = [v for v in range(4) if not case >> v & 1]
        tris = []
        if len(ins) in (1, 3):
            lone = ins[0] if len(ins) == 1 else out[0]
            oth = [v for v in range(4) if v != lone]
            tris.append([(lone, oth[0]), (lone, oth[1]), (lone, oth[2])])
        elif len(ins) == 2:
            a, b = ins
            c, d = out
            tris.append([(a, c), (a, d), (b, d)])
            tris.append([(a, c), (b, d), (b, c)])
        tab.append(tris)
    return tab


_TABLE = _case_table()


def nodal_field(elem: np.ndarray) -> np.ndarray:
    """Element field (nx,ny,nz) → node field (nx+1,...) (mean of the 8 adjacent elements,
    zero outside the grid → closed surfaces)."""
    p = np.pad(elem, 1)
    nx, ny, nz = elem.shape
    out = np.zeros((nx + 1, ny + 1, nz + 1))
    for a, b, c in OFFSETS:
        out += p[a:a + nx + 1, b:b + ny + 1, c:c + nz + 1]
    return out / 8.0


def marching_tets(F: np.ndarray, iso: float, origin, h) -> tuple[np.ndarray, np.ndarray]:
    """Iso-surface {F = iso} of a node field F (X,Y,Z) on the grid → (vertices (n,3) m,
    faces (m,3)), outward = towards F < iso."""
    X, Y, Z = F.shape
    nx, ny, nz = X - 1, Y - 1, Z - 1
    # node values (almost) at the iso level put several surface vertices on top of each other
    # (pinched, non-manifold after an STL reader merges coincident points): push them out
    rng_ = max(float(F.max() - F.min()), 1e-30)
    dlt = 0.01 * rng_
    F = np.where(np.abs(F - iso) < dlt, iso - dlt, F)
    cv = np.stack([F[a:a + nx, b:b + ny, c:c + nz] for a, b, c in OFFSETS], -1)   # (nx,ny,nz,8)
    act = (cv.max(-1) > iso) & (cv.min(-1) <= iso)
    ci = np.argwhere(act)                                                       # (nc,3)
    if len(ci) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), int)
    offs = np.array(OFFSETS)
    gnode = ci[:, None, :] + offs[None]                                         # (nc,8,3)
    gid = (gnode[..., 0] * Y + gnode[..., 1]) * Z + gnode[..., 2]               # (nc,8)
    vals = cv[act]                                                              # (nc,8)
    tv = gid[:, KUHN].reshape(-1, 4)                                            # (nc*6,4)
    tf = vals[:, KUHN].reshape(-1, 4)
    case = ((tf > iso) * (1 << np.arange(4))).sum(1)
    Ff = F.ravel()
    tris_e = []           # (k,3,2) global node pairs
    for cs in range(1, 15):
        sel = np.where(case == cs)[0]
        if len(sel) == 0:
            continue
        for tri in _TABLE[cs]:
            e = np.stack([np.stack([tv[sel, a], tv[sel, b]], -1) for a, b in tri], 1)
            tris_e.append(e)
    E = np.concatenate(tris_e)                                                  # (m,3,2)
    E.sort(axis=2)
    keys = E[..., 0].astype(np.int64) * (X * Y * Z) + E[..., 1]
    uk, inv = np.unique(keys.ravel(), return_inverse=True)
    n0, n1 = uk // (X * Y * Z), uk % (X * Y * Z)
    f0, f1 = Ff[n0], Ff[n1]
    t = np.clip((iso - f0) / np.where(f1 != f0, f1 - f0, 1.0), 0.0, 1.0)

    def coords(g):
        k = g % Z
        j = (g // Z) % Y
        i = g // (Y * Z)
        return np.asarray(origin)[None] + np.stack([i, j, k], -1) * np.asarray(h)[None]

    V = coords(n0) * (1 - t)[:, None] + coords(n1) * t[:, None]
    faces = inv.reshape(-1, 3)
    # orientation: normal must point from high F to low F (outward of the solid)
    P = V[faces]
    nrm = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    # gradient direction at triangle: use the edge endpoints (high end − low end)
    Eg = E[:, 0]
    hi = np.where(Ff[Eg[:, 0]] > Ff[Eg[:, 1]], Eg[:, 0], Eg[:, 1])
    lo = np.where(Ff[Eg[:, 0]] > Ff[Eg[:, 1]], Eg[:, 1], Eg[:, 0])
    out_dir = coords(lo) - coords(hi)
    flip = (nrm * out_dir).sum(1) < 0
    faces[flip] = faces[flip][:, [0, 2, 1]]
    return V, faces


def isosurface(F: np.ndarray, iso: float, origin, h):
    try:
        from skimage.measure import marching_cubes  # type: ignore
        V, Fc, _, _ = marching_cubes(F, iso, spacing=tuple(h))
        return V + np.asarray(origin)[None], Fc[:, ::-1]
    except ImportError:
        return marching_tets(F, iso, origin, h)


def material_fields(rho, phi, hub=None, plate=None, include_pads: bool = True):
    soft = rho * (1 - phi)
    stiff = rho * phi
    if include_pads and hub is not None:
        stiff = np.maximum(stiff, (hub | plate).astype(float))
    return dict(soft=soft, stiff=stiff)


def write_stl(path, V, F, name="part"):
    """Binary STL (vertices in m → written in mm)."""
    Vm = V * 1e3
    P = Vm[F]
    n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-30)
    rec = np.zeros(len(F), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"] = n
    rec["v"] = P
    with open(path, "wb") as fh:
        hdr = f"flexblade topopt {name} (mm)".encode()[:80]
        fh.write(hdr + b" " * (80 - len(hdr)))
        fh.write(np.uint32(len(F)).tobytes())
        fh.write(rec.tobytes())
    return str(path)


def write_step(path, parts: dict[str, tuple[np.ndarray, np.ndarray]], name="flexure"):
    """AP203 faceted B-rep STEP; parts {name: (V m, F)} → one FACETED_BREP each (mm)."""
    lines = []
    idx = [0]

    def ent(s):
        idx[0] += 1
        lines.append(f"#{idx[0]}={s};")
        return idx[0]

    def f(x):
        return f"{x:.6E}".replace("E+00", "E+00")

    actx = ent("APPLICATION_CONTEXT('configuration controlled 3D designs of mechanical parts and assemblies')")
    ent(f"APPLICATION_PROTOCOL_DEFINITION('international standard','config_control_design',1994,#{actx})")
    mctx = ent(f"MECHANICAL_CONTEXT('',#{actx},'mechanical')")
    dctx = ent(f"DESIGN_CONTEXT('',#{actx},'design')")
    lu = ent("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
    au = ent("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
    su = ent("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
    unc = ent(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#{lu},'distance_accuracy_value','')")
    gctx = ent(f"(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc}))"
               f"GLOBAL_UNIT_ASSIGNED_CONTEXT((#{lu},#{au},#{su}))REPRESENTATION_CONTEXT('',''))")
    o = ent("CARTESIAN_POINT('',(0.,0.,0.))")
    dz = ent("DIRECTION('',(0.,0.,1.))")
    dx = ent("DIRECTION('',(1.,0.,0.))")
    ax0 = ent(f"AXIS2_PLACEMENT_3D('',#{o},#{dz},#{dx})")
    for pname, (V, F) in parts.items():
        if len(F) == 0:
            continue
        Vm = V * 1e3
        pts = [ent(f"CARTESIAN_POINT('',({f(p[0])},{f(p[1])},{f(p[2])}))") for p in Vm]
        faces = []
        for tri in F:
            a, b, c = Vm[tri]
            nrm = np.cross(b - a, c - a)
            ln = np.linalg.norm(nrm)
            if ln < 1e-20:
                continue
            nrm /= ln
            ref = (b - a) / max(np.linalg.norm(b - a), 1e-30)
            loc = ent(f"CARTESIAN_POINT('',({f(a[0])},{f(a[1])},{f(a[2])}))")
            dn = ent(f"DIRECTION('',({f(nrm[0])},{f(nrm[1])},{f(nrm[2])}))")
            dr = ent(f"DIRECTION('',({f(ref[0])},{f(ref[1])},{f(ref[2])}))")
            ax = ent(f"AXIS2_PLACEMENT_3D('',#{loc},#{dn},#{dr})")
            pl = ent(f"PLANE('',#{ax})")
            lp = ent(f"POLY_LOOP('',(#{pts[tri[0]]},#{pts[tri[1]]},#{pts[tri[2]]}))")
            fb = ent(f"FACE_OUTER_BOUND('',#{lp},.T.)")
            faces.append(ent(f"FACE_SURFACE('',(#{fb}),#{pl},.T.)"))
        sh = ent(f"CLOSED_SHELL('',({','.join('#%d' % i for i in faces)}))")
        br = ent(f"FACETED_BREP('{pname}',#{sh})")
        rep = ent(f"FACETED_BREP_SHAPE_REPRESENTATION('{pname}',(#{br},#{ax0}),#{gctx})")
        prod = ent(f"PRODUCT('{name}_{pname}','{name}_{pname}','',(#{mctx}))")
        pdf = ent(f"PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',#{prod},.NOT_KNOWN.)")
        pd = ent(f"PRODUCT_DEFINITION('design','',#{pdf},#{dctx})")
        pds = ent(f"PRODUCT_DEFINITION_SHAPE('','',#{pd})")
        ent(f"SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})")
    now = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    head = ["ISO-10303-21;", "HEADER;", "FILE_DESCRIPTION(('flexblade topology-optimised flexure'),'2;1');",
            f"FILE_NAME('{Path(path).name}','{now}',(''),(''),'flexblade','flexblade','');",
            "FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));", "ENDSEC;", "DATA;"]
    with open(path, "w") as fh:
        fh.write("\n".join(head + lines + ["ENDSEC;", "END-ISO-10303-21;", ""]))
    return str(path)


def export_result(res, dom, out_dir, fields: dict | None = None, iso: float = 0.5,
                  step: bool = True) -> dict:
    """Write <name>_<material>.stl per material (+ one multi-product STEP).
    fields: element fields to contour (default: the final projected fields, smoother than
    the crisp 0/1 ones; same 0.5 boundary)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    if fields is None:
        gray = getattr(res, "rho_gray", None) is not None
        fields = dict(rho=res.rho_gray if gray else res.rho, phi=res.phi_gray if gray else res.phi)
    rho, phi = fields["rho"], fields["phi"]
    mf = material_fields(rho, phi, dom.hub, dom.plate)
    files, parts = {}, {}
    for mname, fld in mf.items():
        V, F = isosurface(nodal_field(fld), iso, dom.origin, dom.h)
        parts[mname] = (V, F)
        if len(F):
            files[f"stl_{mname}"] = write_stl(out / f"{res.design_name}_{mname}.stl", V, F, mname)
    if step:
        files["step"] = write_step(out / f"{res.design_name}_topopt.step", parts, res.design_name)
    files["n_triangles"] = {k: int(len(v[1])) for k, v in parts.items()}
    return files


# =========================================================================== renders


def render(res, dom, path, title: str | None = None, util: np.ndarray | None = None,
           history: list | None = None):
    """PNG: three max-intensity projections of ρ coloured by material (soft = orange,
    stiff = dark blue), a 3D view of the iso-surfaces, and the convergence history."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    rho, phi = res.rho, res.phi
    soft_c = np.array([0.93, 0.55, 0.20])
    stiff_c = np.array([0.12, 0.22, 0.45])
    pad_c = np.array([0.6, 0.6, 0.6])
    fig = plt.figure(figsize=(14, 8.5))
    fig.suptitle(title or f"{res.design_name}: topology-optimised flexure", fontsize=13)
    ext = [dom.origin[d] * 1e3 for d in range(3)], [(dom.origin[d] + dom.shape[d] * dom.h[d]) * 1e3 for d in range(3)]
    labels = "xyz"
    for k, (ax_drop, (u, v)) in enumerate([(2, (0, 1)), (1, (0, 2)), (0, (1, 2))]):
        ax = fig.add_subplot(2, 3, k + 1)
        r = rho.max(ax_drop)
        stiff = (rho * phi).max(ax_drop)
        pads = (dom.hub | dom.plate).max(ax_drop)
        img = np.ones(r.shape + (3,))
        frac = np.where(r > 0, stiff / np.maximum(r, 1e-9), 0)[..., None]
        col = soft_c * (1 - frac) + stiff_c * frac
        img = img * (1 - r[..., None]) + col * r[..., None]
        img[pads] = pad_c
        ax.imshow(np.transpose(img, (1, 0, 2)), origin="lower", interpolation="nearest",
                  extent=[ext[0][u], ext[1][u], ext[0][v], ext[1][v]], aspect="equal")
        H = dom.hinge_point * 1e3
        a = dom.hinge_axis
        L = 0.6 * max(ext[1][u] - ext[0][u], ext[1][v] - ext[0][v])
        ax.plot([H[u] - L * a[u], H[u] + L * a[u]], [H[v] - L * a[v], H[v] + L * a[v]], "r--", lw=1)
        ax.plot(H[u], H[v], "r+", ms=10)
        ax.set_xlim(ext[0][u], ext[1][u])
        ax.set_ylim(ext[0][v], ext[1][v])
        ax.set_xlabel(f"{labels[u]} (mm)")
        ax.set_ylabel(f"{labels[v]} (mm)")
        ax.set_title(f"max projection along {labels[ax_drop]}")
    ax = fig.add_subplot(2, 3, 4, projection="3d")
    mf = material_fields(rho, phi, dom.hub, dom.plate)
    for mname, col in (("soft", soft_c), ("stiff", stiff_c)):
        V, F = isosurface(nodal_field(mf[mname]), 0.5, dom.origin, dom.h)
        if len(F):
            P = V[F] * 1e3
            nrm = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
            nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-30)
            shade = 0.45 + 0.55 * np.clip(nrm @ np.array([0.4, -0.5, 0.75]) / 1.03, 0, 1)
            pc = Poly3DCollection(P, facecolors=np.clip(col[None] * shade[:, None] + 0.05, 0, 1),
                                  edgecolors="none")
            ax.add_collection3d(pc)
    for d, setl in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
        setl(ext[0][d], ext[1][d])
    ax.set_box_aspect([ext[1][d] - ext[0][d] for d in range(3)])
    H = dom.hinge_point * 1e3
    a = dom.hinge_axis
    L = 6.0
    ax.plot([H[0] - L * a[0], H[0] + L * a[0]], [H[1], H[1]], [H[2] - L * a[2], H[2] + L * a[2]], "r--")
    ax.view_init(elev=22, azim=-60)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title("iso-surfaces (orange soft, blue stiff)")
    ax = fig.add_subplot(2, 3, 5)
    hist = history if history is not None else res.history
    if hist:
        it = [h["it"] for h in hist]
        ax.semilogy(it, [h["f0"] for h in hist], label="f0 (stiff-dir compliance / seed)")
        ax.semilogy(it, [h["k_axis"] / res.targets["k_hinge_target"] for h in hist], label="k_hinge / target")
        ax.set_xlabel("iteration")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
        ax.set_title("convergence")
    ax = fig.add_subplot(2, 3, 6)
    ax.axis("off")
    m, fm, sm = res.metrics, res.fact_metrics, res.seed_metrics
    rows = [("", "FACT beam", "voxel seed", "topopt"),
            ("k_hinge (N m/rad)", f"{fm.get('k_axis', np.nan):.3g}", f"{sm['k_axis']:.3g}", f"{m['k_axis']:.3g}"),
            ("stiffness ratio", f"{fm.get('stiffness_ratio', np.nan):.3g}", f"{sm['stiffness_ratio']:.3g}",
             f"{m['stiffness_ratio']:.3g}"),
            ("axis error (deg)", f"{fm.get('axis_error_deg', np.nan):.2f}", f"{sm['axis_error_deg']:.2f}",
             f"{m['axis_error_deg']:.2f}"),
            ("stretch (µm)", f"{res.targets['stretch_limit'] * 1e6:.1f}*", f"{res.responses['seed_stretch'] * 1e6:.1f}",
             f"{res.responses['stretch'] * 1e6:.1f}"),
            ("stress util", "", f"{res.responses['seed_stress_max']:.2f}", f"{res.responses['stress_max']:.2f}"),
            ("mass (g)", "", "", f"{res.mass_props['mass'] * 1e3:.3f}")]
    tb = ax.table(cellText=rows, loc="center", cellLoc="center")
    tb.auto_set_font_size(False)
    tb.set_fontsize(9)
    tb.scale(1, 1.5)
    ax.set_title("* stretch limit (FACT beam under blade CF)", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return str(path)
