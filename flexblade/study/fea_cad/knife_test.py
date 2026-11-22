"""CalculiX prediction for the knife-flexure BENCH TEST part (knife_rotor.py slits=0 paddle=1), 2026-11-23.

Half model x >= 0 (blade-1 hinge + paddle): the part is cut at x = 0 and that face is encastre (hub bolted on
the M5 bore; the hub between the bore and the hinge is solid, so the cut stands in for the bolt clamp).
Paddle outboard of x = X_RIGID is a rigid body (T-section ~100x stiffer than the webs) with its pilot on the
hinge point h, so the pilot ROT node gives the paddle rotation directly.
Steps (linear, isotropic PETG, no contact: valid below the +-11 deg stair contact):
  1-3  unit pure moments 1 N.mm about a (hinge axis), s = y x a, y   -> k_hinge, k_cross, k_flap (as cases.py c1)
  4,5  a 10 g weight hung in the paddle hole at (55, +4.5) / (55, -4.5): force -z through the hole axis
Outputs (summary.json + printed table): stiffnesses, hole drops + paddle twist per 10 g, peak web strain per
degree of hinge turn (max principal, Gauss points in the hinge zone; 99.9th percentile reported beside the max
because sharp printed corners are singular).
usage: python knife_test.py PART.step WORKDIR [h_fine=0.5] [E_MPa=2000] [nu=0.38]
"""
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import gmsh
import numpy as np

STEP, WD = sys.argv[1], Path(sys.argv[2])
H = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
E = float(sys.argv[4]) if len(sys.argv) > 4 else 2000.0
NU = float(sys.argv[5]) if len(sys.argv) > 5 else 0.38
WD.mkdir(parents=True, exist_ok=True)
HP = np.array([10.75, 0.0, 0.0])                          # hinge point (blade 1)
A = np.array([0.882947593, 0.0, -0.469471563])            # hinge axis
S_AX = np.cross([0.0, 1.0, 0.0], A); S_AX /= np.linalg.norm(S_AX)
X_RIGID = 30.0
HOLES = [np.array([55.0, 4.5, 0.0]), np.array([55.0, -4.5, 0.0])]
G10 = 0.010 * 9.81                                        # 10 g in N
# bench loads (name, point mm, grams): 10 g in each paddle hole; 50 g at +-60 mm on a ruler taped square across
# the paddle at x = 55 (top face z = 1.5 + ruler 1 mm -> load line z ~ 2.5)
LOADS = [("hole_y+4.5_10g", (55.0, 4.5, 0.0), 10.0), ("hole_y-4.5_10g", (55.0, -4.5, 0.0), 10.0),
         ("ruler_y+60_50g", (55.0, 60.0, 2.5), 50.0), ("ruler_y-60_50g", (55.0, -60.0, 2.5), 50.0)]
PROBES = dict(hole_p=(55.0, 4.5, 0.0), hole_m=(55.0, -4.5, 0.0), tip=(60.0, 0.0, 0.0), ruler_p=(55.0, 60.0, 2.5),
              ruler_m=(55.0, -60.0, 2.5))
G2C = [0, 1, 2, 3, 4, 5, 6, 7, 9, 8]                      # gmsh tet10 -> ccx C3D10

# ------------------------------------------------------------------ mesh
gmsh.initialize()
gmsh.option.setNumber("General.Terminal", 0)
gmsh.model.occ.importShapes(STEP)
gmsh.model.occ.synchronize()
vols = gmsh.model.getEntities(3)                          # keep the rotor; loose strips (slotted builds) dropped:
big = max(vols, key=lambda e: gmsh.model.occ.getMass(*e))  # their slits are modelled as empty (conservative)
gmsh.model.occ.remove([e for e in vols if e != big], recursive=True)
box = gmsh.model.occ.addBox(0.0, -30, -30, 100, 60, 60)
gmsh.model.occ.intersect([big], [(3, box)])
gmsh.model.occ.synchronize()
fld = gmsh.model.mesh.field
b = fld.add("Box")
for k_, v_ in dict(VIn=H, VOut=2.0, XMin=2.0, XMax=21.0, YMin=-10, YMax=10, ZMin=-8, ZMax=8, Thickness=4).items():
    fld.setNumber(b, k_, v_)
fld.setAsBackgroundMesh(b)
for k_, v_ in (("Mesh.MeshSizeExtendFromBoundary", 0), ("Mesh.MeshSizeFromPoints", 0), ("Mesh.MeshSizeFromCurvature", 0),
               ("Mesh.ElementOrder", 2), ("Mesh.SecondOrderLinear", 1), ("Mesh.Algorithm3D", 10), ("General.NumThreads", 16)):
    gmsh.option.setNumber(k_, v_)
gmsh.model.mesh.generate(3)
ntag, xyz, _ = gmsh.model.mesh.getNodes()
X = xyz.reshape(-1, 3)
idx = {int(t): i for i, t in enumerate(ntag)}
et, etags, enodes = gmsh.model.mesh.getElements(3)
assert list(et) == [11], et
T = np.array([idx[int(t)] for t in enodes[0]]).reshape(-1, 10)[:, G2C]
gmsh.finalize()
print(f"mesh: {len(X)} nodes, {len(T)} C3D10, h_fine {H}", flush=True)

clamp = np.where(X[:, 0] < 1e-6)[0]
rig = np.where(X[:, 0] > X_RIGID)[0]
cen = X[T[:, :4]].mean(axis=1)
hz = np.where((cen[:, 0] > 2.5) & (cen[:, 0] < 20.0))[0]
assert len(clamp) > 50 and len(rig) > 50

# ------------------------------------------------------------------ deck
def nums(a, per=12):
    a = list(a)
    return "\n".join(", ".join(str(v) for v in a[i:i + per]) for i in range(0, len(a), per))

ref, rot = len(X) + 1, len(X) + 2
L = ["*HEADING", "knife flexure bench test part, half model", "*NODE"]
L += [f"{i + 1}, {x:.6f}, {y:.6f}, {z:.6f}" for i, (x, y, z) in enumerate(X)]
L += [f"{ref}, {HP[0]}, {HP[1]}, {HP[2]}", f"{rot}, {HP[0]}, {HP[1]}, {HP[2]}"]
L += ["*ELEMENT, TYPE=C3D10, ELSET=EALL"] + [f"{j + 1}, " + ", ".join(str(n + 1) for n in t) for j, t in enumerate(T)]
L += ["*ELSET, ELSET=EHINGE", nums(hz + 1)]
L += ["*NSET, NSET=NCLAMP", nums(clamp + 1), "*NSET, NSET=NRIG", nums(rig + 1)]
L += ["*MATERIAL, NAME=PETG", "*ELASTIC", f"{E}, {NU}", "*SOLID SECTION, ELSET=EALL, MATERIAL=PETG",
      f"*RIGID BODY, NSET=NRIG, REF NODE={ref}, ROT NODE={rot}", "*BOUNDARY", "NCLAMP, 1, 3"]


def step(loads, strain):
    s = ["*STEP", "*STATIC", "*CLOAD, OP=NEW"] + [f"{n}, {d}, {v:.9g}" for n, d, v in loads]
    s += ["*NODE PRINT, NSET=NPIL", "U"]
    if strain:
        s += ["*EL PRINT, ELSET=EHINGE", "E"]
    return s + ["*END STEP"]


L += ["*NSET, NSET=NPIL", f"{ref}, {rot}"]
for ax in (A, S_AX, np.array([0.0, 1.0, 0.0])):                         # 1 N.mm pure moments
    L += step([(rot, d + 1, ax[d]) for d in range(3) if abs(ax[d]) > 1e-12], strain=ax is A)
for _, pt, gr in LOADS:                                                   # weight at pt: force + moment at pilot
    F = np.array([0.0, 0.0, -gr * 1e-3 * 9.81])
    M = np.cross(np.array(pt) - HP, F)
    L += step([(ref, 3, F[2])] + [(rot, d + 1, M[d]) for d in range(3) if abs(M[d]) > 1e-12], strain=True)
(WD / "knife.inp").write_text("\n".join(L) + "\n")

# ------------------------------------------------------------------ run + parse
env = dict(os.environ, OMP_NUM_THREADS="16", CCX_NPROC_EQUATION_SOLVER="16")
r = subprocess.run(["ccx", "-i", "knife"], cwd=WD, env=env, capture_output=True, text=True, timeout=7200)
(WD / "ccx.log").write_text(r.stdout[-20000:] + r.stderr[-5000:])
if "Job finished" not in r.stdout:
    sys.exit("ccx failed, see " + str(WD / "ccx.log"))
steps, cur, mode = [], None, None
for ln in (WD / "knife.dat").read_text().splitlines():
    t = ln.strip()
    if t.startswith("displacements"):
        cur = {"U": {}, "E": []}; steps.append(cur); mode = "U"
    elif t.startswith("strains"):
        if cur is None or cur["E"]:
            cur = {"U": {}, "E": []}; steps.append(cur)
        mode = "E"
    elif t and mode and t[0].isdigit():
        v = t.split()
        if mode == "U":
            cur["U"][int(v[0])] = np.array([float(x) for x in v[1:4]])
        else:
            cur["E"].append([float(x) for x in v[2:8]])
# ccx prints U for a step, then E for the same step (step 1 only)
stp = [s_ for s_ in steps if s_["U"]]
Es = stp[0]["E"]
th = [s_["U"][rot] for s_ in stp]                                         # rotation vectors (rad)
k_h = 1e-3 / float(th[0] @ A)                                             # N.m/rad
k_s = 1e-3 / float(th[1] @ S_AX)
k_y = 1e-3 / float(th[2] @ np.array([0.0, 1.0, 0.0]))
Ev = np.array(Es)
Et = np.zeros((len(Ev), 3, 3))
Et[:, 0, 0], Et[:, 1, 1], Et[:, 2, 2] = Ev[:, 0], Ev[:, 1], Ev[:, 2]
Et[:, 0, 1] = Et[:, 1, 0] = Ev[:, 3]; Et[:, 0, 2] = Et[:, 2, 0] = Ev[:, 4]; Et[:, 1, 2] = Et[:, 2, 1] = Ev[:, 5]
ep = np.abs(np.linalg.eigvalsh(Et)).max(axis=1)                          # |principal| per Gauss point


def pstrain(E_):
    v = np.array(E_)
    t = np.zeros((len(v), 3, 3))
    t[:, 0, 0], t[:, 1, 1], t[:, 2, 2] = v[:, 0], v[:, 1], v[:, 2]
    t[:, 0, 1] = t[:, 1, 0] = v[:, 3]; t[:, 0, 2] = t[:, 2, 0] = v[:, 4]; t[:, 1, 2] = t[:, 2, 1] = v[:, 5]
    e = np.abs(np.linalg.eigvalsh(t)).max(axis=1)
    return float(e.max()), float(np.percentile(e, 99.9))
deg1 = math.radians(1.0) / float(th[0] @ A)                              # moment scale for 1 deg about a
out = dict(mesh=dict(nodes=len(X), elems=len(T), h_fine=H), E_MPa=E, nu=NU,
           k_hinge_Nm_rad=k_h, k_cross_Nm_rad=k_s, k_flap_Nm_rad=k_y,
           strain_per_deg_max=float(ep.max() * deg1), strain_per_deg_p999=float(np.percentile(ep, 99.9) * deg1),
           strain_per_deg_p99=float(np.percentile(ep, 99) * deg1))
for j, (nm, pt, gr) in enumerate(LOADS):
    u_ref, w = stp[3 + j]["U"][ref], stp[3 + j]["U"][rot]
    out[nm] = dict(grams=gr, turn_about_hinge_deg=math.degrees(float(w @ A)),
                   tilt_about_x_deg=math.degrees(float(w[0])), flap_about_y_deg=math.degrees(float(w[1])),
                   drop_mm={k_: float(-(u_ref + np.cross(w, np.array(q) - HP))[2]) for k_, q in PROBES.items()},
                   web_strain_max=pstrain(stp[3 + j]["E"])[0], web_strain_p999=pstrain(stp[3 + j]["E"])[1])
(WD / "summary.json").write_text(json.dumps(out, indent=1))
print(json.dumps(out, indent=1))
