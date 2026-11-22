"""Blade-1 surface of the real CAD (SolidCraft STEP, mm) as an ASCII STL in metres for the CFD hover sector.

Keeps the largest solid (the rotor; strips dropped), cuts it to x > XCUT (blade 1 side, hub post and blade 2
removed; the cut leaves a flat cap normal to x, which carries no thrust or torque), keeps only the outer shell
(internal strip slits are closed voids) and writes `solid blade` in metres (rotor3d.rotor_sector_case(stl=...)).
usage: python cad_blade_stl.py ROTOR.step XCUT_mm OUT.stl [h_mm=0.4]
"""
import sys

import gmsh
import numpy as np

STEP, XCUT, OUT = sys.argv[1], float(sys.argv[2]), sys.argv[3]
H = float(sys.argv[4]) if len(sys.argv) > 4 else 0.4
gmsh.initialize()
gmsh.option.setNumber("General.Terminal", 0)
gmsh.model.occ.importShapes(STEP)
gmsh.model.occ.synchronize()
vols = gmsh.model.getEntities(3)
big = max(vols, key=lambda e: gmsh.model.occ.getMass(*e))
gmsh.model.occ.remove([e for e in vols if e != big], recursive=True)
box = gmsh.model.occ.addBox(XCUT, -60, -60, 200, 120, 120)
gmsh.model.occ.intersect([big], [(3, box)])
gmsh.model.occ.synchronize()
surfs = [t for _, t in gmsh.model.getBoundary(gmsh.model.getEntities(3), oriented=False)]
# outer shell = the connected set of faces (shared curves) holding the face reaching furthest out
par = {s: s for s in surfs}
def find(s):
    while par[s] != s:
        par[s] = par[par[s]]; s = par[s]
    return s
by_curve = {}
for s in surfs:
    for _, c in gmsh.model.getBoundary([(2, s)], oriented=False):
        by_curve.setdefault(abs(c), []).append(s)
for ss in by_curve.values():
    for s in ss[1:]:
        par[find(s)] = find(ss[0])
far = max(surfs, key=lambda s: gmsh.model.getBoundingBox(2, s)[3])
keep = [s for s in surfs if find(s) == find(far)]
for k_, v_ in (("Mesh.MeshSizeMax", H), ("Mesh.MeshSizeMin", H / 8), ("Mesh.MeshSizeFromCurvature", 24),
               ("Mesh.Algorithm", 6)):
    gmsh.option.setNumber(k_, v_)
gmsh.model.mesh.generate(2)
ntag, xyz, _ = gmsh.model.mesh.getNodes()
X = xyz.reshape(-1, 3) * 1e-3
idx = {int(t): i for i, t in enumerate(ntag)}
tris = []
for s in keep:
    et, _, en = gmsh.model.mesh.getElements(2, s)
    for t, n in zip(et, en):
        if t == 2:
            tris.append(np.array([idx[int(v)] for v in n]).reshape(-1, 3))
T = np.concatenate(tris)
gmsh.finalize()
# orient outward: signed volume > 0 for a closed outward shell
v = np.einsum("ij,ij->i", X[T[:, 0]], np.cross(X[T[:, 1]], X[T[:, 2]])).sum() / 6
if v < 0:
    T = T[:, [0, 2, 1]]
L = ["solid blade"]
for a, b, c in X[T]:
    n = np.cross(b - a, c - a); n /= max(np.linalg.norm(n), 1e-30)
    L.append(f" facet normal {n[0]:.6e} {n[1]:.6e} {n[2]:.6e}\n  outer loop")
    L += [f"   vertex {p[0]:.9e} {p[1]:.9e} {p[2]:.9e}" for p in (a, b, c)]
    L.append("  endloop\n endfacet")
L.append("endsolid blade")
open(OUT, "w").write("\n".join(L) + "\n")
print(f"{OUT}: {len(T)} tris, {len(keep)}/{len(surfs)} faces kept (outer shell), vol {abs(v) * 1e9:.1f} mm3, "
      f"x {X[:, 0].min() * 1e3:.2f}..{X[:, 0].max() * 1e3:.2f} mm")
