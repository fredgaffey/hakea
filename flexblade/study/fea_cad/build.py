"""Whole-rotor CalculiX model of the SolidCraft CAD of 5in_v8s_2307_m245_301 (runs on the Mac: gmsh + numpy).

Printed bodies: hub.step, blade1.step, blade2.step meshed as C3D10 (gmsh/OCC, mm). Each 9 x 0.05 mm dry stack
is ONE hex20 (C3D20R) lamina of t = 0.05 mm at the stack mid-plane with E x 9 and rho x 9 (bending n.E.w.t^3/12
and axial n.E.w.t both match; per-lamina stress = model stress / 9). Strip frames come from
solidcraft.rotor_layout (layout.json, the same numbers that drew the STEPs). Glue: both lamina faces of each
8 mm land are *TIEd to the slot faces they face (0.325 mm epoxy gap, rigid glue). Aramid strap: SPRINGA chain
hub knot well -> hub channel mouth -> blade channel mouth -> blade knot well; knot ends averaged (*EQUATION) on
the bearing half of the well wall, channel-mouth nodes held laterally to the mouth ring (strand slides axially).

usage: python build.py OUTDIR [h_root_mm=0.5] [nl_free=40] [nw=8]
writes OUTDIR/mesh.inp + sets.json (node/element sets used by cases.py / post.py)
"""
import json
import sys
from pathlib import Path

import gmsh
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = Path(sys.argv[1]); OUT.mkdir(parents=True, exist_ok=True)
H = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
NL = int(sys.argv[3]) if len(sys.argv) > 3 else 40
NW = int(sys.argv[4]) if len(sys.argv) > 4 else 8
LAY = json.loads((HERE / "layout.json").read_text())
P = LAY["prm"]
TL = LAY["t_lam"]                      # 0.05 mm lamina
BMAP = [np.array(b) for b in LAY["blade_map"]]
X_HUB_END = LAY["hinge"][0] - 3.25 / 2   # strap free span (design strap_length 3.25 mm) about the hinge
X_BL_START = LAY["hinge"][0] + 3.25 / 2

# ------------------------------------------------------------------ printed bodies (gmsh)
gmsh.initialize()
gmsh.option.setNumber("General.Terminal", 0)
bodies = {}
for nm in ("hub", "blade1", "blade2"):
    dt = gmsh.model.occ.importShapes(str(HERE / "cad" / f"{nm}.step"))
    bodies[nm] = [t for d, t in dt if d == 3]
gmsh.model.occ.synchronize()
f = gmsh.model.mesh.field.add("Ball")
for k, v in (("Radius", 27.0), ("Thickness", 6.0), ("VIn", H), ("VOut", max(4 * H, 1.6)),
             ("XCenter", 0.0), ("YCenter", 0.0), ("ZCenter", 0.0)):
    gmsh.model.mesh.field.setNumber(f, k, v)
gmsh.model.mesh.field.setAsBackgroundMesh(f)
for k, v in (("Mesh.MeshSizeExtendFromBoundary", 0), ("Mesh.MeshSizeFromPoints", 0),
             ("Mesh.MeshSizeFromCurvature", 0), ("Mesh.MeshSizeMin", 0.08), ("Mesh.ElementOrder", 2),
             ("Mesh.Algorithm3D", 10), ("Mesh.Optimize", 1), ("Mesh.HighOrderOptimize", 0), ("Mesh.SecondOrderLinear", 1),
             ("General.NumThreads", 8)):
    gmsh.option.setNumber(k, v)
gmsh.model.mesh.generate(3)
ntag, ncoord, _ = gmsh.model.mesh.getNodes()
X = np.zeros((int(ntag.max()) + 1, 3))
X[ntag.astype(int)] = ncoord.reshape(-1, 3)
TET = {}
for nm, vols in bodies.items():
    el = []
    for v in vols:
        types, _, nodes = gmsh.model.mesh.getElements(3, v)
        for ty, nn in zip(types, nodes):
            assert ty == 11, ty                                   # 10-node tet
            el.append(nn.reshape(-1, 10).astype(int))
    TET[nm] = np.vstack(el)[:, [0, 1, 2, 3, 4, 5, 6, 7, 9, 8]]  # gmsh -> ccx C3D10 order
gmsh.finalize()
e0 = TET["hub"][0]
assert np.allclose(X[e0[8]], (X[e0[1]] + X[e0[3]]) / 2, atol=1e-6), "tet10 node order"
used = np.unique(np.concatenate([t.ravel() for t in TET.values()]))
print({k: len(v) for k, v in TET.items()}, "nodes", len(used), "bbox", X[used].min(0).round(2), X[used].max(0).round(2))

FACES = np.array([(0, 1, 2), (0, 3, 1), (1, 3, 2), (2, 3, 0)])    # ccx C3D10 faces 1..4 (corners)


def boundary_faces(T, eid0):
    c = T[:, :4][:, FACES]                                        # (ne, 4, 3)
    key = np.sort(c, axis=2).reshape(-1, 3)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    once = cnt[inv.ravel()] == 1
    idx = np.nonzero(once)[0]
    return np.c_[eid0 + idx // 4, idx % 4 + 1], c.reshape(-1, 3)[idx]   # (elem id, face no), corner nodes


eid = {}
nxt = 1
for nm in ("hub", "blade1", "blade2"):
    eid[nm] = nxt
    nxt += len(TET[nm])
BF = {nm: boundary_faces(TET[nm], eid[nm]) for nm in TET}
nodes_of = {nm: np.unique(TET[nm].ravel()) for nm in TET}

# ------------------------------------------------------------------ strips (hex20 laminae)
HEX20 = [(0, 0, 0), (2, 0, 0), (2, 2, 0), (0, 2, 0), (0, 0, 2), (2, 0, 2), (2, 2, 2), (0, 2, 2),
         (1, 0, 0), (2, 1, 0), (1, 2, 0), (0, 1, 0), (1, 0, 2), (2, 1, 2), (1, 2, 2), (0, 1, 2),
         (0, 0, 1), (2, 0, 1), (2, 2, 1), (0, 2, 1)]
land, Lf, wid = P["land"], LAY["strips"][0]["L"], P["w"]
s_c = np.r_[np.linspace(-land, 0, 17)[:-1], np.linspace(0, Lf, NL + 1)[:-1], np.linspace(Lf, Lf + land, 17)]
u_c = np.linspace(-wid / 2, wid / 2, NW + 1)
v_c = np.array([-P["slot_t"] / 2, -TL / 2, TL / 2, P["slot_t"] / 2])   # glue | lamina | glue (land rows only)


def refine(c):
    o = np.empty(2 * len(c) - 1)
    o[::2], o[1::2] = c, (c[:-1] + c[1:]) / 2
    return o


S_, U_, V_ = refine(s_c), refine(u_c), refine(v_c)
nid = int(X.shape[0])
newX, STRIPS, HEXEL = [], [], []
hex_id = nxt
for k in (1, 2):
    B = BMAP[k - 1]
    for i, fr in enumerate(LAY["strips"], 1):
        p0, t, w, n = (B @ np.array(fr[q]) for q in ("p0", "t", "w", "n"))
        if np.cross(t, w) @ n < 0:
            n = -n
        key = {}

        def gid(a, b, c):
            if (a, b, c) not in key:
                global nid
                key[(a, b, c)] = nid
                newX.append(p0 + S_[a] * t + U_[b] * w + V_[c] * n)
                nid += 1
            return key[(a, b, c)]
        els, erow, gl, glrow = [], [], [], []
        nland = 16
        for a in range(len(s_c) - 1):
            for b in range(len(u_c) - 1):
                els.append([gid(2 * a + di, 2 * b + dj, 2 + dk) for di, dj, dk in HEX20])
                erow.append(a)
                if a < nland or a >= len(s_c) - 1 - nland:          # epoxy layers fill the slot gap
                    for c0 in (0, 4):
                        gl.append([gid(2 * a + di, 2 * b + dj, c0 + dk) for di, dj, dk in HEX20])
                        glrow.append((a, b, c0))
        ids = np.arange(hex_id, hex_id + len(els))
        hex_id += len(els)
        gids = np.arange(hex_id, hex_id + len(gl))
        hex_id += len(gl)
        HEXEL.append((ids, np.array(els), gids, np.array(gl)))
        sl = {}
        for side, fit, lo, hi in (("hub", "hub", -land, 0.0), ("bl", f"blade{k}", Lf, Lf + land)):
            for sg, c in ((1, 6), (-1, 0)):
                sl[f"{side}{'p' if sg > 0 else 'm'}"] = [v for (a, b, cc), v in key.items()
                                                       if cc == c and lo - 1e-6 <= S_[a] <= hi + 1e-6]
        STRIPS.append(dict(k=k, i=i, p0=p0, t=t, w=w, n=n, key={f"{a},{b},{c}": v for (a, b, c), v in key.items()},
                           elems=ids.tolist(), erow=erow, glue=gids.tolist(), glrow=glrow, slaves=sl))
X = np.vstack([X, np.array(newX)])

# master faces: boundary faces of the fitting lying on the slot faces facing each land
TIES = []
for st in STRIPS:
    for side, fit, lo, hi in (("hub", "hub", -land - 0.6, 0.6), ("bl", f"blade{st['k']}", Lf - 0.6, Lf + land + 0.6)):
        (ef, cn) = BF[fit]
        C = X[cn] - st["p0"]                                      # (nf, 3, 3)
        s, u, v = C @ st["t"], C @ st["w"], C @ st["n"]
        for sg in (1, -1):
            ok = (np.abs(v - sg * P["slot_t"] / 2) < 0.03).all(1) & ((s > lo) & (s < hi)).all(1) & \
                 (np.abs(u) < P["slot_w"] / 2 + 0.05).all(1)
            nm = f"K{st['k']}S{st['i']}{side}{'P' if sg > 0 else 'M'}"
            TIES.append(dict(name=nm, faces=ef[ok].tolist(), slaves=st["slaves"][f"{side}{'p' if sg > 0 else 'm'}"]))
            print(nm, "master faces", int(ok.sum()), "slaves", len(TIES[-1]["slaves"]))

# ------------------------------------------------------------------ strap: one SPRINGA knot -> knot
# Each knot node sits at the centroid of the bearing half of its well wall and is tied to it by an averaging
# *EQUATION (exactly rigid-rotation invariant at the centroid). A chain with channel-mouth nodes held laterally
# was tried first: the slightly off-axis mouth centroids turned the 13 kN/mm strand into a spurious pitch
# spring (k_strand * e^2), so the strand is one straight tie and its arm e about the hinge axis is reported.
EQ, SPR, STRAPN, STRAP_E = [], [], {}, {}
for k in (1, 2):
    B = BMAP[k - 1]
    ids = []
    Xb = np.linalg.inv(B) @ X.T                                   # blade-1-frame coordinates (3, nn)
    for fit, cx, sgn in (("hub", LAY["x_hw"], 1), (f"blade{k}", LAY["x_bw"], -1)):
        nn = nodes_of[fit]
        r = np.hypot(Xb[0, nn] - cx, Xb[1, nn])
        sel = nn[(np.abs(r - P["well_d"] / 2) < 0.06) & (sgn * (Xb[0, nn] - cx) > 0.4) & (np.abs(Xb[2, nn]) < 2.0)]
        X = np.vstack([X, X[sel].mean(0)])
        ids.append(X.shape[0] - 1)
        EQ.append((ids[-1], (1, 2, 3), sel.tolist()))
        print(f"strap b{k} knot {fit}: {len(sel)} well nodes, centroid (blade frame) {np.round(np.linalg.inv(B) @ X[ids[-1]], 3)}")
    STRAPN[k] = ids
    SPR.append((ids[0], ids[1]))
    h = B @ np.array(LAY["hinge"]); ax = B @ np.array(LAY["hinge_axis"])
    d = X[ids[1]] - X[ids[0]]
    STRAP_E[k] = float(np.cross(ax, X[ids[1]] - h) @ d / np.linalg.norm(d))
    print(f"strap b{k} length {np.linalg.norm(d):.3f} mm, arm about hinge axis e = {STRAP_E[k]:.4f} mm")

# ------------------------------------------------------------------ write
o = ["*NODE"]
alln = np.r_[used, np.arange(int(ntag.max()) + 1, X.shape[0])]
o += [f"{i}, {X[i, 0]:.7g}, {X[i, 1]:.7g}, {X[i, 2]:.7g}" for i in alln if i > 0]
for nm in ("hub", "blade1", "blade2"):
    o.append(f"*ELEMENT, TYPE=C3D10, ELSET={nm.upper()}")
    o += [f"{eid[nm] + j}, " + ", ".join(map(str, e)) for j, e in enumerate(TET[nm])]
for st, (ids, els, gids, gl) in zip(STRIPS, HEXEL):
    for nm_, ii, ee in (("STRIP", ids, els), ("GLUE", gids, gl)):
        o.append(f"*ELEMENT, TYPE=C3D20R, ELSET={nm_}{st['k']}{st['i']}")
        o += [f"{j}, " + ", ".join(map(str, e[:15])) + ",\n" + ", ".join(map(str, e[15:])) for j, e in zip(ii, ee)]
o.append("*ELEMENT, TYPE=SPRINGA, ELSET=STRAP")
o += [f"{hex_id + j}, {a}, {b}" for j, (a, b) in enumerate(SPR)]
o += ["*ELSET, ELSET=STRIPS", "STRIP11, STRIP12, STRIP21, STRIP22", "*ELSET, ELSET=GLUES", "GLUE11, GLUE12, GLUE21, GLUE22", "*ELSET, ELSET=SOLIDS",
      "HUB, BLADE1, BLADE2, STRIPS, GLUES", "*ELSET, ELSET=FITS", "HUB, BLADE1, BLADE2"]


def nset(name, ids):
    ids = list(ids)
    return [f"*NSET, NSET={name}"] + [", ".join(map(str, ids[i:i + 12])) for i in range(0, len(ids), 12)]


bore = [i for i in nodes_of["hub"] if np.hypot(X[i, 0], X[i, 1]) < LAY["bore"] / 2 + 0.03]
o += nset("BORE", bore)
print("bore nodes", len(bore))
for t_ in TIES:
    if not t_["faces"]:
        continue
    o += nset("N" + t_["name"], t_["slaves"])
    o += [f"*SURFACE, NAME=S{t_['name']}, TYPE=NODE", "N" + t_["name"],
          f"*SURFACE, NAME=M{t_['name']}, TYPE=ELEMENT"] + [f"{e}, S{fn}" for e, fn in t_["faces"]]
    o += [f"*TIE, NAME=T{t_['name']}, POSITION TOLERANCE=0.05, ADJUST=NO", f"S{t_['name']}, M{t_['name']}"]
for nd, dofs, sel in EQ:
    for d in dofs:
        terms = [(nd, d, 1.0)] + [(m, d, -1.0 / len(sel)) for m in sel]
        o += ["*EQUATION", str(len(terms))]
        o += [", ".join(f"{a}, {b}, {c:.9g}" for a, b, c in terms[i:i + 4]) for i in range(0, len(terms), 4)]
(OUT / "mesh.inp").write_text("\n".join(o) + "\n")
meta = dict(H=H, NL=NL, NW=NW, eid=eid, ntet={k: len(v) for k, v in TET.items()}, nnodes=int(len(alln)),
            strap=STRAPN, strap_e=STRAP_E, spring_ids=[hex_id, hex_id + len(SPR) - 1], next_node=int(X.shape[0]),
            strips=[{k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in st.items() if k != "slaves"}
                    for st in STRIPS],
            ties={t_["name"]: dict(nf=len(t_["faces"]), slaves=t_["slaves"]) for t_ in TIES},
            nodes_hub=nodes_of["hub"].tolist(), nodes_b1=nodes_of["blade1"].tolist(), nodes_b2=nodes_of["blade2"].tolist())
(OUT / "sets.json").write_text(json.dumps(meta))
np.save(OUT / "X.npy", X)
print("wrote", OUT / "mesh.inp")
