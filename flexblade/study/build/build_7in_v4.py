"""Build the printable parts for 7in_tt_strap_steelx_v4 (run from flexblade/ with PYTHONPATH=$PWD).

Takes the lofted solids from geometry.build_surface and adds, by boolean cuts (manifold3d):
  * per steel strip and per owning fitting: a tight clamp slot (shim t + 0.05) x (w + 0.2) over the
    clamp land, a RADIUSED jaw exit (R = 1.5 mm, both faces) and a clearance window over the rest of
    the strip span (so the free length is really free);
  * the aramid strand channel (dia 1.8) from the crossing into each fitting, ending in a 4 mm
    vertical knot well open to +z.
Outputs STL (mm, hub frame, z up = build direction) into study/build/7in_v4/.
"""
import json, sys, numpy as np, trimesh
from pathlib import Path
from flexblade.schema import Design
from flexblade.geometry import build_surface

OUT = Path("study/build/7in_v4"); OUT.mkdir(parents=True, exist_ok=True)
D = Design.model_validate(json.load(open("study/designs/7in_tt_strap_steelx_v4.json")))
F = D.flexure
T, W, ALLOW, R, LAND = F.thickness * 1e3, F.width * 1e3, 0.05, 1.5, 5.0
SIDE = 0.1            # width clearance per side
H = (T + ALLOW) / 2   # slot half-thickness
STRAND, CH_D, WELL_D = F.strap_strand_diameter * 1e3, 1.8, 4.0

s = build_surface(D)
M = {p["name"]: trimesh.Trimesh(np.round(np.array(p["vertices"]) * 1e3, 3), p["faces"], process=True) for p in s["parts"]}


def U(ms):
    return trimesh.boolean.union(ms, engine="manifold") if len(ms) > 1 else ms[0]


def frame(p0, p1, mesh):
    t = (p1 - p0); L = np.linalg.norm(t); t /= L
    ev, evec = np.linalg.eigh(np.cov((mesh.vertices - mesh.vertices.mean(0)).T))
    n = evec[:, 0]; n -= t * (n @ t); n /= np.linalg.norm(n)
    w = np.cross(n, t)
    return t, w, n, L


def box(c, t, w, n, ls, lw, ln):
    """box centred at c, local axes (t,w,n) with extents (ls,lw,ln)"""
    m = trimesh.creation.box((ls, lw, ln)); Tm = np.eye(4); Tm[:3, :3] = np.c_[t, w, n]; Tm[:3, 3] = c
    m.apply_transform(Tm); return m


def cyl(c, t, w, n, r, ax_w):
    """cylinder axis along w, centre c"""
    m = trimesh.creation.cylinder(r, ax_w, sections=64)   # axis z -> map z->w, x->t, y->n
    Tm = np.eye(4); Tm[:3, :3] = np.c_[t, n, w]; Tm[:3, 3] = c
    if np.linalg.det(Tm[:3, :3]) < 0: Tm[:3, :3][:, 1] *= -1
    m.apply_transform(Tm); return m


def strip_cutters(p0, p1, mesh, owner_end):
    """cutter list for the fitting that owns strip end `owner_end` (0 or 1)"""
    t, w, n, L = frame(p0, p1, mesh)
    o = p0 if owner_end == 0 else p1
    d = t if owner_end == 0 else -t              # direction from owner end toward the other end
    wid = W + 2 * SIDE
    c = []
    # tight slot: from 0.5 mm behind the strip end to the land exit
    s0, s1 = -0.5, LAND
    c.append(box(o + d * (s0 + s1) / 2, d, w, n, s1 - s0, wid, 2 * H))
    # flared radiused mouth: [LAND-R, LAND] x +-(H+R), minus two R cylinders (tangent to wall and exit face)
    fl = box(o + d * (LAND - R / 2), d, w, n, R, wid, 2 * (H + R))
    cs = [cyl(o + d * (LAND - R) + n * sg * (H + R), d, w, n, R, wid + 2) for sg in (1, -1)]
    fl = trimesh.boolean.difference([fl] + cs, engine="manifold")
    c.append(fl)
    # clearance window over remaining span (+ the other end's land, owned by the other fitting)
    c.append(box(o + d * (LAND + (L + 1.0 - LAND) / 2), d, w, n, L + 1.0 - LAND, wid, 2 * (H + R)))
    return c


def cut_fitting(name_list, blade_k, owner_end):
    """union of the named parts minus all strip cutters of blade_k whose end `owner_end` is in these parts"""
    base = U([M[n] for n in name_list])
    mem = [m for m in s["members"] if m["blade"] == blade_k]
    cuts = []
    for m in mem:
        if m["kind"] == "wire": continue
        p0, p1 = np.array(m["p0"]) * 1e3, np.array(m["p1"]) * 1e3
        sm = M[f"blade{blade_k}_flex{m['index']}"]
        cuts += strip_cutters(p0, p1, sm, owner_end)
    # strand channel + knot well
    for m in mem:
        if m["kind"] != "wire": continue
        p0, p1 = np.array(m["p0"]) * 1e3, np.array(m["p1"]) * 1e3
        a = p0 if owner_end == 0 else p1
        away = (p0 - p1) if owner_end == 0 else (p1 - p0); away /= np.linalg.norm(away)
        depth = 8.0
        z = np.array([0, 0, 1.0]); y = np.cross(z, away)
        cc = trimesh.creation.cylinder(CH_D / 2, depth + 1.0, sections=48)
        Rm = trimesh.geometry.align_vectors([0, 0, 1], away); Rm[:3, 3] = a + away * (depth - 1.0) / 2
        cc.apply_transform(Rm); cuts.append(cc)
        well = trimesh.creation.cylinder(WELL_D / 2, 30.0, sections=48)
        well.apply_translation(a + away * depth + np.array([0, 0, 15.0 - 0.0]))
        well.apply_translation([0, 0, -2.0]); cuts.append(well)
    return trimesh.boolean.difference([base] + cuts, engine="manifold")


def finish(m, name):
    print(name, "raw watertight", m.is_watertight)
    m = m.copy(); m.apply_translation([0, 0, -m.bounds[0][2]])
    import manifold3d as mf
    mm = mf.Manifold(mf.Mesh(np.asarray(m.vertices, np.float32), np.asarray(m.faces, np.uint32))).simplify(0.002)
    parts = [c for c in mm.decompose() if c.volume() > 50.0]
    print(' components kept', len(parts), 'dropped', len(mm.decompose()) - len(parts), 'floating slivers')
    mm = parts[0] if len(parts) == 1 else mf.Manifold.batch_boolean(parts, mf.OpType.Add)
    mo = mm.to_mesh(); m = trimesh.Trimesh(np.asarray(mo.vert_properties)[:, :3].astype(np.float64), np.asarray(mo.tri_verts), process=False)
    m.export(OUT / f"{name}.stl"); m = trimesh.load(OUT / f"{name}.stl")

    print(name, "watertight", m.is_watertight, "vol_mm3", round(m.volume, 1),
          "bbox_mm", np.round(m.extents, 1).tolist(), "tris", len(m.faces))
    return m


# blade 1 hub fitting end = owner of strip end 0; root fitting owns end 1.
hub1 = cut_fitting(["blade1_hub_fitting"], 1, 0)
root1 = cut_fitting(["blade1_root_fitting", "blade1_shank", "blade1"], 1, 1)
hub2 = cut_fitting(["blade2_hub_fitting"], 2, 0)
hubcore = U([M["hub"], M["blade1_spoke"], M["blade2_spoke"], hub1, hub2])
hubm = finish(hubcore, "hub_fitting_assembly")
blade = finish(root1, "blade_with_root_fitting")

# interference check (blade vs hub assembly) in assembly frame
iv = trimesh.boolean.intersection([hubcore, root1], engine="manifold")
print("hub/blade overlap volume mm3 (assembly frame):", round(iv.volume, 2) if len(iv.faces) else 0.0)
# slot geometry check: strip box must fit inside slot cut
print("slot", round(T + ALLOW, 3), "x", round(W + 2 * SIDE, 2), "mm, land", LAND, "mm, jaw R", R, "mm")
