"""Independent CalculiX (ccx 2.21) cross-check for flexblade designs.

Deliberately shares no code with fea.py / rotor.py / beam.py / section.py. It uses only
  * schema.Design / materials.Material (+ rotation_matrix for the print axes)
  * geometry.build_geometry (solid *sections* only -- the lofts are rebuilt, fragmented and
    meshed here with gmsh; geometry.export_step's own fuse/cut loses the aerodynamic blade for
    some designs, see study/ccx_crosscheck.md)
  * gmsh (second-order tets, C3D10) and the ccx binary.

Units inside ccx: mm, N, MPa, tonne, s  (rho kg/m^3 -> *1e-12 t/mm^3; E Pa -> *1e-6 MPa).
Frames: hub frame H as in the design, z = rotor axis, origin on the axis.

Public API
----------
  Model                       mesh + element sets (material, orientation) + node sets
  mesh_design(design)         -> Model   (all solids, conformal bonded interfaces)
  write_inp(design, workdir, rpm=None, case="centrifugal"|"modal"|"root_stiffness")
  write_inp_model(model, workdir, case=..., ...)   generic (used by the validation tests)
  run(workdir)                -> parsed results dict (ccx subprocess, OMP_NUM_THREADS=4)
  check(design, rpm=None)     -> dict (per-body stresses/strains/utilisations, frequencies,
                                 6x6 hinge stiffness, timings)
"""
from __future__ import annotations

import dataclasses as dc
import math
import os
import re
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from . import materials as M
from .schema import Design, PrintOrientation

CCX = os.environ.get("CCX_BIN", "/usr/bin/ccx")
# gmsh tet10 node order -> ccx C3D10 order
_G2C = [0, 1, 2, 3, 4, 5, 6, 7, 9, 8]
MIN_VOL_MM3 = 0.05            # fragments smaller than this are slivers; dropped


# =========================================================================== model container


@dataclass
class SetInfo:
    material: M.Material
    orientation: Optional[PrintOrientation]
    elems: np.ndarray                        # 0-based element indices
    role: str = ""


@dataclass
class Model:
    nodes: np.ndarray                        # (N,3) mm
    elems: np.ndarray                        # (E,10) 0-based node ids, ccx order
    sets: dict[str, SetInfo]
    nsets: dict[str, np.ndarray] = field(default_factory=dict)   # 0-based node ids
    meta: dict = field(default_factory=dict)

    def set_nodes(self, names) -> np.ndarray:
        idx = np.concatenate([self.sets[n].elems for n in names])
        return np.unique(self.elems[idx].ravel())


# =========================================================================== meshing


def _mm_section(s):
    s2 = dc.replace(s, ring=s.ring * 1e3, radius=s.radius * 1e3)
    if s.center is not None:
        s2.center = s.center * 1e3
    return s2


def _orient_key(o: PrintOrientation) -> tuple:
    return tuple(np.round(np.r_[o.build_dir, o.raster_dir], 6))


def _body_name(p) -> str:
    cls = "rod_" if p.role == "rod" else ("core_" if p.role in ("core", "member_core") else "")
    owner = f"blade{p.blade}" if p.blade else "hub"
    return f"{owner}_{cls}{p.material}"


def _size_for(p, h_blade, h_flex, h_hub) -> float:
    if p.role in ("hub", "spoke"):
        return h_hub
    if p.role in ("blade", "skin", "core"):
        return h_blade
    return h_flex


def mesh_design(design: Design, blades: Optional[list[int]] = None, h_blade: float = 1.4,
                h_flex: float = 0.7, h_hub: float = 2.6, verbose: bool = False) -> Model:
    """Mesh all solids of `design` (second-order tets, conformal at material interfaces).

    blades: keep only these blade indices (hub always kept); None = all.
    Returns Model with element sets named like the STEP bodies (blade1_tpu_95a, ...);
    bodies of one name but different print orientation get a _o2, _o3 suffix.
    """
    import gmsh
    from . import geometry as G

    geo = G.build_geometry(design, n_side=G.N_AIRFOIL_STEP)
    parts = []
    for p in geo.parts:
        if "shank" in p.name:          # self-intersecting wedge in geometry.py (negative volume)
            continue
        if blades is not None and p.blade and p.blade not in blades:
            continue
        if blades is not None and p.name.startswith("blade") and \
                int(re.match(r"blade(\d+)", p.name).group(1)) not in blades:
            continue
        parts.append(p)

    if gmsh.isInitialized():
        gmsh.finalize()
    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 1 if verbose else 0)
        gmsh.model.add("ccx")
        occ = gmsh.model.occ
        ins = []
        for p in parts:
            wires = [G._occ_wire(occ, _mm_section(s)) for s in p.sections]
            dt = occ.addThruSections(wires, makeSolid=True, makeRuled=p.ruled)
            ins.append(dt[0])
        occ.synchronize()
        _, omap = occ.fragment(ins, [])
        occ.synchronize()
        # owner of each fragment = highest-priority parent (ties: first)
        owner: dict[int, int] = {}
        for ip, m in enumerate(omap):
            for (dim, tag) in m:
                if dim != 3:
                    continue
                if tag not in owner or parts[ip].priority > parts[owner[tag]].priority:
                    owner[tag] = ip
        vols = []
        for tag, ip in owner.items():
            vol = occ.getMass(3, tag)
            if vol >= MIN_VOL_MM3:
                vols.append(tag)
        # per-point sizes
        gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 1)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 1)
        gmsh.option.setNumber("Mesh.MeshSizeMin", 0.02)
        gmsh.option.setNumber("Mesh.MeshSizeMax", h_hub)
        psize: dict[int, float] = {}
        for tag in vols:
            h = _size_for(parts[owner[tag]], h_blade, h_flex, h_hub)
            for (_, pt) in gmsh.model.getBoundary([(3, tag)], combined=False,
                                                  oriented=False, recursive=True):
                psize[pt] = min(psize.get(pt, 1e9), h)
        for pt, h in psize.items():
            gmsh.model.mesh.setSize([(0, pt)], h)
        gmsh.option.setNumber("Mesh.Algorithm3D", 1)
        gmsh.option.setNumber("Mesh.ElementOrder", 2)
        gmsh.option.setNumber("Mesh.HighOrderOptimize", 0)
        gmsh.option.setNumber("Mesh.SecondOrderLinear", 1)   # straight-edged tets: J > 0
        gmsh.model.mesh.generate(3)
        gmsh.model.mesh.setOrder(2)

        ntags, ncoord, _ = gmsh.model.mesh.getNodes()
        ncoord = np.asarray(ncoord).reshape(-1, 3)
        remap = np.full(int(ntags.max()) + 1, -1, dtype=np.int64)
        used = set()
        elem_rows, elem_owner = [], []
        for tag in vols:
            types, _, enodes = gmsh.model.mesh.getElements(3, tag)
            for t, en in zip(types, enodes):
                if t != 11:
                    raise RuntimeError(f"unexpected gmsh element type {t}")
                en = np.asarray(en, dtype=np.int64).reshape(-1, 10)
                elem_rows.append(en)
                elem_owner += [owner[tag]] * len(en)
                used.update(np.unique(en).tolist())
        used = np.array(sorted(used))
        remap[used] = np.arange(len(used))
        nid_to_row = {int(t): i for i, t in enumerate(ntags)}
        nodes = np.array([ncoord[nid_to_row[int(t)]] for t in used])
        elems = remap[np.vstack(elem_rows)][:, _G2C]
        elem_owner = np.array(elem_owner)
    finally:
        gmsh.finalize()

    # element sets: (body name, orientation) -> set
    sets: dict[str, SetInfo] = {}
    seen: dict[str, list] = {}
    keyset: dict[tuple, str] = {}
    for ip in sorted(set(elem_owner.tolist())):
        p = parts[ip]
        base = _body_name(p)
        k = (base, _orient_key(p.orientation))
        if k not in keyset:
            lst = seen.setdefault(base, [])
            lst.append(k)
            name = base if len(lst) == 1 else f"{base}_o{len(lst)}"
            keyset[k] = name
            sets[name] = SetInfo(M.get_material(p.material), p.orientation, np.empty(0, int),
                                 role=p.role)
        sets[keyset[k]].elems = np.concatenate(
            [sets[keyset[k]].elems, np.nonzero(elem_owner == ip)[0]])
    model = Model(nodes, elems, sets)
    model.meta["parts"] = {p.name: (p.role, p.material) for p in parts}
    model.meta["owner_role"] = {int(i): parts[i].role for i in set(elem_owner.tolist())}
    model.meta["owner_name"] = {int(i): parts[i].name for i in set(elem_owner.tolist())}
    model.meta["elem_owner"] = elem_owner
    model.meta["hinge"] = tuple(np.asarray(geo.hinge[min(geo.hinge)][0]) * 1e3)
    model.meta["hinge_axis"] = tuple(np.asarray(geo.hinge[min(geo.hinge)][1]))
    model.meta["hub_radius_mm"] = design.flexure.hub_radius * 1e3
    return model


# =========================================================================== deck writer


def _fmt_nums(a, per=8):
    return "\n".join(", ".join(f"{x:d}" for x in a[i:i + per]) for i in range(0, len(a), per))


def _elastic_card(m: M.Material) -> str:
    s = 1e-6
    return ("*ELASTIC, TYPE=ENGINEERING CONSTANTS\n"
            f"{m.E1*s:.8g}, {m.E2*s:.8g}, {m.E3*s:.8g}, {m.nu12:.6g}, {m.nu13:.6g}, "
            f"{m.nu23:.6g}, {m.G12*s:.8g}, {m.G13*s:.8g},\n{m.G23*s:.8g}\n")


def _orient_card(name: str, o: PrintOrientation) -> str:
    R = M.rotation_matrix(o)
    e1, e2 = R[:, 0], R[:, 1]
    return (f"*ORIENTATION, NAME={name}, SYSTEM=RECTANGULAR\n"
            + ", ".join(f"{x:.9g}" for x in np.r_[e1, e2]) + "\n")


def write_inp_model(model: Model, workdir, case: str = "static", rpm: Optional[float] = None,
                    nlgeom: bool = True, fixed=(), loads=(), n_modes: int = 8,
                    prestress: bool = True, rigid: Optional[dict] = None,
                    name: str = "model", inc0: float = 0.5, max_inc: int = 200) -> Path:
    """Write <workdir>/<name>.inp.

    case: "static" (fixed + loads), "centrifugal" (+rpm), "modal" (rpm None/0 -> unstressed),
          "root_stiffness" (rigid dict needed).
    fixed:  iterable of (nset_name, first_dof, last_dof)
    loads:  iterable of (nset_name, dof, value_per_node) for *CLOAD
    rigid:  dict(nset=<name>, ref=<xyz mm>) for 6x6 compliance (adds ref/rot nodes).
    """
    wd = Path(workdir)
    wd.mkdir(parents=True, exist_ok=True)
    N = len(model.nodes)
    out = ["** flexblade ccx cross-check (mm, N, MPa, tonne)", "*NODE, NSET=Nall"]
    out += [f"{i+1}, {x:.9g}, {y:.9g}, {z:.9g}" for i, (x, y, z) in enumerate(model.nodes)]
    ref_id = rot_id = None
    if rigid is not None:
        ref_id, rot_id = N + 1, N + 2
        rx, ry, rz = rigid["ref"]
        out += [f"{ref_id}, {rx:.9g}, {ry:.9g}, {rz:.9g}", f"{rot_id}, {rx:.9g}, {ry:.9g}, {rz:.9g}",
                "*NSET, NSET=RN", f"{ref_id}, {rot_id}"]
    out.append("")
    # elements by set
    for sname, si in model.sets.items():
        out.append(f"*ELEMENT, TYPE=C3D10, ELSET={sname}")
        for e in si.elems:
            out.append(f"{e+1}, " + ", ".join(str(n + 1) for n in model.elems[e]))
    out.append("*ELSET, ELSET=Eall")
    for sname in model.sets:
        out.append(sname)
    # materials / orientations / sections
    mats = {}
    for sname, si in model.sets.items():
        m = si.material
        mats.setdefault(m.name, m)
    for mn, m in mats.items():
        out.append(f"*MATERIAL, NAME=M_{mn}")
        out.append(_elastic_card(m).rstrip())
        out.append("*DENSITY")
        out.append(f"{m.rho*1e-12:.9g}")
    for i, (sname, si) in enumerate(model.sets.items()):
        oname = ""
        if si.orientation is not None:
            out.append(_orient_card(f"O{i}", si.orientation).rstrip())
            oname = f", ORIENTATION=O{i}"
        out.append(f"*SOLID SECTION, ELSET={sname}, MATERIAL=M_{si.material.name}{oname}")
    # node sets
    nsets = dict(model.nsets)
    if rigid is not None and fixed:
        # a node cannot be both an SPC node and a dependent node of the rigid-body MPC (ccx
        # "MPC and SPC"). It happens when the blade-side and hub-side fittings touch and the
        # conforming mesh merges their nodes: the load then bypasses the flexure, so the
        # result would be meaningless -> fail early with a geometry message.
        fx = np.unique(np.concatenate([np.asarray(nsets[ns]) for ns, _, _ in fixed]))
        both = np.intersect1d(np.asarray(nsets[rigid["nset"]]), fx)
        if len(both):
            raise ValueError(f"ccx model: {len(both)} nodes are both fixed and in the rigid blade-side "
                             "set (fittings touch / share nodes, bypassing the flexure); "
                             "separate the root and hub fittings in the geometry")
    for nname, ids in nsets.items():
        out.append(f"*NSET, NSET={nname}")
        out.append(_fmt_nums((np.asarray(ids) + 1).tolist(), 12))
    if rigid is not None:
        out.append(f"*RIGID BODY, NSET={rigid['nset']}, REF NODE={ref_id}, ROT NODE={rot_id}")

    def bcs():
        if fixed:
            out.append("*BOUNDARY")
            for ns, d0, d1 in fixed:
                out.append(f"{ns}, {d0}, {d1}")

    om2 = (rpm * 2 * math.pi / 60.0) ** 2 if rpm else 0.0
    nl = ", NLGEOM" if nlgeom else ""
    if case == "static":
        out.append("*STEP")
        out.append("*STATIC")
        bcs()
        if loads:
            out.append("*CLOAD")
            for ns, d, v in loads:
                out.append(f"{ns}, {d}, {v:.9g}")
        out += ["*EL PRINT, ELSET=Eall", "S", "*EL PRINT, ELSET=Eall", "E",
                "*NODE PRINT, NSET=Nall", "U", "*END STEP"]
    elif case in ("centrifugal", "modal"):
        stressed = case == "centrifugal" or (prestress and om2 > 0)
        if stressed:
            out.append(f"*STEP{nl}, INC={max_inc}")
            out.append("*STATIC")
            out.append(f"{inc0}, 1.0")
            # (boundary must be re-stated only in the first step)
            bcs()
            out.append("*DLOAD")
            out.append(f"Eall, CENTRIF, {om2:.9g}, 0., 0., 0., 0., 0., 1.")
            if case == "centrifugal":
                # FREQUENCY=1000: only the last increment of the step is written
                out += ["*EL PRINT, ELSET=Eall, FREQUENCY=1000", "S",
                        "*EL PRINT, ELSET=Eall, FREQUENCY=1000", "E",
                        "*NODE PRINT, NSET=Nall, FREQUENCY=1000", "U"]
            out.append("*END STEP")
        if case == "modal":
            out.append("*STEP, PERTURBATION" if stressed else "*STEP")
            out.append("*FREQUENCY")
            out.append(f"{n_modes}")
            if not stressed:
                bcs()
            out.append("*END STEP")
    elif case == "root_stiffness":
        bcs()
        for j in range(6):
            out.append("*STEP")
            out.append("*STATIC")
            out.append("*CLOAD, OP=NEW")
            out.append(f"{ref_id if j < 3 else rot_id}, {j % 3 + 1}, 1.0")
            out += ["*NODE PRINT, NSET=RN", "U", "*END STEP"]
    else:
        raise ValueError(case)
    p = wd / f"{name}.inp"
    p.write_text("\n".join(out) + "\n")
    return p


def box_model(size_mm, h: float, material: M.Material,
              orientation: Optional[PrintOrientation] = None) -> Model:
    """Single-body brick [0,Lx]x[0,Ly]x[0,Lz] (mm), tet10 mesh. Node sets: XMIN, XMAX."""
    import gmsh
    if gmsh.isInitialized():
        gmsh.finalize()
    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("box")
        gmsh.model.occ.addBox(0, 0, 0, *size_mm)
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", h)
        gmsh.option.setNumber("Mesh.ElementOrder", 2)
        gmsh.option.setNumber("Mesh.Algorithm3D", 1)
        gmsh.model.mesh.generate(3)
        gmsh.model.mesh.setOrder(2)
        ntags, xyz, _ = gmsh.model.mesh.getNodes()
        xyz = np.asarray(xyz).reshape(-1, 3)
        order = np.argsort(ntags)
        ntags, xyz = np.asarray(ntags)[order], xyz[order]
        _, _, en = gmsh.model.mesh.getElements(3)
        en = np.asarray(en[0], dtype=np.int64).reshape(-1, 10)
        row = {int(t): i for i, t in enumerate(ntags)}
        elems = np.vectorize(row.get)(en)[:, _G2C]
    finally:
        gmsh.finalize()
    m = Model(xyz, elems, {"box": SetInfo(material, orientation, np.arange(len(elems)))})
    tol = 1e-6
    m.nsets["XMIN"] = np.nonzero(xyz[:, 0] < tol)[0]
    m.nsets["XMAX"] = np.nonzero(xyz[:, 0] > size_mm[0] - tol)[0]
    return m


def default_bcs(model: Model, design: Optional[Design] = None) -> list:
    """Hub-inner-region nodes (|r| < 0.5 hub radius) fully fixed -> node set FIXHUB."""
    hub_sets = [n for n, s in model.sets.items() if s.role in ("hub", "spoke")]
    ids = model.set_nodes(hub_sets)
    xy = np.hypot(model.nodes[ids, 0], model.nodes[ids, 1])
    sel = ids[xy < 0.5 * model.meta.get("hub_radius_mm", 6.0)]
    model.nsets["FIXHUB"] = sel
    return [("FIXHUB", 1, 3)]


def _build_for_case(design: Design, case: str, **mesh_kw):
    if case == "root_stiffness":
        model = mesh_design(design, blades=[1], **mesh_kw)
    else:
        model = mesh_design(design, **mesh_kw)
    return model


def blade_side_nodes(model: Model) -> np.ndarray:
    """Blade-1 outboard side of the flexure: blade/skin/core + root fitting + blade rods."""
    own = model.meta["elem_owner"]
    names = model.meta["owner_name"]
    roles = model.meta["owner_role"]
    keep = [i for i in names if names[i].startswith("blade1") and (
        roles[i] in ("blade", "skin", "core")
        or names[i].endswith("root_fitting")
        or re.fullmatch(r"blade1_rod\d+", names[i]))]
    mask = np.isin(own, keep)
    return np.unique(model.elems[np.nonzero(mask)[0]].ravel())


def hub_side_nodes(model: Model) -> np.ndarray:
    own = model.meta["elem_owner"]
    roles = model.meta["owner_role"]
    keep = [i for i in roles if roles[i] in ("hub", "spoke")]
    return np.unique(model.elems[np.nonzero(np.isin(own, keep))[0]].ravel())


def write_inp(design: Design, workdir, rpm: Optional[float] = None, case: str = "centrifugal",
              model: Optional[Model] = None, nlgeom: bool = True, n_modes: int = 8,
              prestress: bool = True, **mesh_kw) -> Path:
    """Mesh `design` and write the ccx deck for `case` into workdir/model.inp."""
    rpm = _resolve_rpm(design, rpm) if case == "centrifugal" or (case == "modal" and prestress
                                                                  and rpm is not None) else rpm
    if model is None or (case == "root_stiffness") != bool(model.meta.get("single_blade")):
        model = _build_for_case(design, case, **mesh_kw)
        model.meta["single_blade"] = case == "root_stiffness"
    if case == "root_stiffness":
        model.nsets["BLADESIDE"] = blade_side_nodes(model)
        model.nsets["FIXHUB"] = hub_side_nodes(model)
        return write_inp_model(model, workdir, "root_stiffness",
                               fixed=[("FIXHUB", 1, 3)], rigid=dict(
                                   nset="BLADESIDE", ref=model.meta["hinge"]))
    fixed = default_bcs(model, design)
    return write_inp_model(model, workdir, case, rpm=rpm, nlgeom=nlgeom, fixed=fixed,
                           n_modes=n_modes, prestress=prestress)


def _resolve_rpm(design: Design, rpm):
    if rpm is not None:
        return rpm
    if design.op.rpm:
        return design.op.rpm
    from . import aero  # trim to hover thrust (only needed when rpm is not given)
    return float(aero.AeroModel(design).trim().rpm)


# =========================================================================== running / parsing


_NUM = re.compile(r"^\s*[-+0-9.]")


def _parse_dat(path: Path) -> dict:
    """Parse ccx .dat: stresses, strains, displacements (last block of each) + eigenfrequencies."""
    blocks: dict[str, list] = {"stress": [], "strain": [], "disp": []}
    freqs = []
    lines = path.read_text().splitlines()
    i, n = 0, len(lines)
    while i < n:
        ln = lines[i].lower()
        key = None
        if ln.strip().startswith("stresses (elem"):
            key = "stress"
        elif "strains (elem" in ln:
            key = "strain"
        elif ln.strip().startswith("displacements (vx"):
            key = "disp"
        if key:
            i += 1
            rows = []
            while i < n and (not lines[i].strip() or _NUM.match(lines[i])):
                if lines[i].strip():
                    rows.append(lines[i].split()[:8] if key != "disp" else lines[i].split()[:4])
                i += 1
            if rows:
                blocks[key].append(np.array(rows, dtype=float))
            continue
        if "e i g e n v a l u e" in ln:
            i += 1
            while i < n:
                t = lines[i].split()
                if len(t) == 5 and t[0].isdigit():
                    freqs.append(float(t[3]))
                elif freqs and not lines[i].strip():
                    pass
                elif freqs and not _NUM.match(lines[i]):
                    break
                i += 1
            continue
        i += 1
    return dict(stress=blocks["stress"], strain=blocks["strain"], disp=blocks["disp"],
                freqs=freqs)


def run(workdir, name: str = "model", timeout: float = 3600, threads: int = 4) -> dict:
    wd = Path(workdir)
    # NOTE: ccx 2.21's multithreaded SPOOLES solve is NON-DETERMINISTIC on this machine (same
    # input, ~1 run in 10 returns a wrong displacement field). Element/stress loops stay
    # threaded (OMP_NUM_THREADS), the linear-equation solver is forced to 1 thread.
    env = dict(os.environ, OMP_NUM_THREADS=str(threads), CCX_NPROC_EQUATION_SOLVER="1")
    t0 = time.time()
    r = subprocess.run([CCX, "-i", name], cwd=wd, env=env, capture_output=True, text=True,
                       timeout=timeout)
    dt = time.time() - t0
    (wd / f"{name}.log").write_text(r.stdout + "\n" + r.stderr)
    dat = wd / f"{name}.dat"
    if r.returncode != 0 or "*ERROR" in r.stdout + r.stderr:
        tail = "\n".join((r.stdout + r.stderr).splitlines()[-15:])
        raise RuntimeError(f"ccx failed (rc={r.returncode}):\n{tail}")
    res = _parse_dat(dat) if dat.exists() else dict(stress=[], strain=[], disp=[], freqs=[])
    res["wall_s"] = dt
    return res


# =========================================================================== post-processing


def _voigt_to_tensor(a: np.ndarray, shear_factor: float = 1.0) -> np.ndarray:
    """rows: xx yy zz xy xz yz  -> (n,3,3). shear_factor 0.5 converts engineering gamma."""
    T = np.zeros((len(a), 3, 3))
    T[:, 0, 0], T[:, 1, 1], T[:, 2, 2] = a[:, 0], a[:, 1], a[:, 2]
    T[:, 0, 1] = T[:, 1, 0] = a[:, 3] * shear_factor
    T[:, 0, 2] = T[:, 2, 0] = a[:, 4] * shear_factor
    T[:, 1, 2] = T[:, 2, 1] = a[:, 5] * shear_factor
    return T


def _mises(T: np.ndarray) -> np.ndarray:
    s = T
    return np.sqrt(0.5 * ((s[:, 0, 0] - s[:, 1, 1]) ** 2 + (s[:, 1, 1] - s[:, 2, 2]) ** 2
                          + (s[:, 2, 2] - s[:, 0, 0]) ** 2)
                   + 3 * (s[:, 0, 1] ** 2 + s[:, 0, 2] ** 2 + s[:, 1, 2] ** 2))


def body_stats(model: Model, res: dict, strain_shear_factor: float = 1.0) -> dict:
    """Per element-set maxima over integration points (no extrapolation / smoothing).
    Invariants (mises, principal) are frame independent; the build-axis stress is the local szz."""
    S = res["stress"][-1]
    E = res["strain"][-1] if res["strain"] else None
    out = {}
    eid = S[:, 0].astype(int) - 1
    Ts = _voigt_to_tensor(S[:, 2:8])
    ev = np.linalg.eigvalsh(Ts)
    vm = _mises(Ts)
    if E is not None:
        Te = _voigt_to_tensor(E[:, 2:8], strain_shear_factor)
        ee = np.linalg.eigvalsh(Te)
    for name, si in model.sets.items():
        sel = np.isin(eid, si.elems)
        if not sel.any():
            continue
        d = dict(material=si.material.name, n_elems=int(len(si.elems)),
                 mises_MPa=float(vm[sel].max()), mises_p99_MPa=float(np.percentile(vm[sel], 99)),
                 s1_MPa=float(ev[sel, 2].max()),
                 s3_MPa=float(ev[sel, 0].min()))
        if E is not None:
            d["e1_max"] = float(ee[sel, 2].max())
            d["e_abs_max"] = float(np.abs(ee[sel]).max())
        if si.orientation is not None:
            # ccx prints S/E of elements that have an *ORIENTATION in the LOCAL material system
            # (verified in tests/test_ccx.py), so the stress along the build axis is just szz.
            snn = Ts[sel][:, 2, 2]
            d["s_build_max_MPa"] = float(snn.max())
            d["s_build_min_MPa"] = float(snn.min())
        out[name] = d
    return out


def max_displacement(res: dict) -> float:
    if not res["disp"]:
        return float("nan")
    u = res["disp"][-1][:, 1:4]
    return float(np.linalg.norm(u, axis=1).max())


def stiffness_from_run(res: dict) -> np.ndarray:
    """6x6 stiffness in SI (N/m, N, N m/rad ordering: [ux uy uz rx ry rz]) from 6 unit cases.

    Cases run with 1 N and 1 N*mm; converted to metres / N m.
    """
    C = np.zeros((6, 6))
    for j, blk in enumerate(res["disp"][:6]):
        ref = blk[blk[:, 0] == blk[:, 0].min()][0, 1:4]
        rot = blk[blk[:, 0] == blk[:, 0].max()][0, 1:4]
        u = np.r_[ref * 1e-3, rot]           # m, rad
        f = 1.0 if j < 3 else 1e-3           # N, N m
        C[:, j] = u / f
    C = 0.5 * (C + C.T)
    return np.linalg.inv(C)


# =========================================================================== public check


def _strengths(m: M.Material) -> dict:
    return dict(Xt=m.Xt / 1e6, Zt=m.Zt / 1e6, creep=m.creep_strain_allow)


def check(design: Design, rpm: Optional[float] = None, workdir=None, mesh_scale: float = 1.0,
          nlgeom: bool = True, do_modal: bool = True, do_stiffness: bool = True,
          do_centrifugal: bool = True, inc0: float = 0.5) -> dict:
    """Independent FEA check: centrifugal stress/strain, prestressed modes, 6x6 hinge stiffness."""
    import tempfile
    wd = Path(workdir) if workdir else Path(tempfile.mkdtemp(prefix="ccxcheck_"))
    wd.mkdir(parents=True, exist_ok=True)
    tm: dict[str, float] = {}
    rpm = _resolve_rpm(design, rpm)
    mk = dict(h_blade=1.4 * mesh_scale, h_flex=0.7 * mesh_scale, h_hub=2.6 * mesh_scale)
    out: dict = dict(design=design.name, rpm=rpm, workdir=str(wd))

    t = time.time()
    model = mesh_design(design, **mk)
    model.meta["single_blade"] = False
    tm["mesh_full_s"] = time.time() - t
    out["n_nodes"], out["n_elems"] = len(model.nodes), len(model.elems)
    fixed = default_bcs(model, design)

    if do_centrifugal:
        cdir = wd / "centrifugal"
        write_inp_model(model, cdir, "centrifugal", rpm=rpm, nlgeom=nlgeom, fixed=fixed,
                        inc0=inc0)
        r = run(cdir)
        tm["centrifugal_s"] = r["wall_s"]
        stats = body_stats(model, r)
        for name, d in stats.items():
            s = _strengths(model.sets[name].material)
            d["util_mises_Xt"] = d["mises_MPa"] / s["Xt"]
            d["util_interlayer"] = max(d.get("s_build_max_MPa", 0.0), 0.0) / s["Zt"]
            d["util_strain_creep"] = d.get("e1_max", 0.0) / s["creep"]
        out["bodies"] = stats
        out["u_max_mm"] = max_displacement(r)

    if do_modal:
        mdir = wd / "modal"
        write_inp_model(model, mdir, "modal", rpm=rpm, nlgeom=nlgeom, fixed=fixed)
        rm = run(mdir)
        tm["modal_s"] = rm["wall_s"]
        out["freqs_hz"] = rm["freqs"]
    if do_stiffness:
        t = time.time()
        m1 = mesh_design(design, blades=[1], **mk)
        tm["mesh_1blade_s"] = time.time() - t
        m1.nsets["BLADESIDE"] = blade_side_nodes(m1)
        m1.nsets["FIXHUB"] = hub_side_nodes(m1)
        sdir = wd / "stiffness"
        write_inp_model(m1, sdir, "root_stiffness", fixed=[("FIXHUB", 1, 3)],
                        rigid=dict(nset="BLADESIDE", ref=m1.meta["hinge"]))
        rs = run(sdir)
        tm["stiffness_s"] = rs["wall_s"]
        K = stiffness_from_run(rs)
        out["K6"] = K.tolist()
        out["K_diag"] = np.diag(K).tolist()
        out["hinge_mm"] = list(m1.meta["hinge"])
        a = np.asarray(m1.meta["hinge_axis"], float)
        a /= np.linalg.norm(a)
        Cr = np.linalg.inv(K)[3:, 3:]
        out["hinge_axis"] = a.tolist()
        # rotation stiffness about the hinge axis with all other DOF free (1 / compliance)
        out["k_hinge_Nm_rad"] = float(1.0 / (a @ Cr @ a))
        out["K_rot_eig"] = np.linalg.eigvalsh(K[3:, 3:]).tolist()
    out["timings_s"] = tm
    return out
