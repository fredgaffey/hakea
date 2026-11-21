"""2D plane-strain (tri6, gmsh) CalculiX model of ONE lamina (t=0.10 mm) held between two jaws whose
exit edge has radius r. Frictionless surface-to-surface contact, jaw bolt pressure P0 on the top jaw, then the
strip free end is loaded with the v8 beam surface bending stress SIGB (linear traction = end moment) + axial SIGA.
Reports K_clamp = max |S_xx| on the strip surface nodes / SIGB and the peak x (mm, <0 = under the jaw).
usage: python study/ccx_clamp_edge.py [out_dir]
"""
import os
import subprocess
import sys
from pathlib import Path

import gmsh
import numpy as np

T = 0.1
SIGB, SIGA, P0 = 143.0, 1.0, 20.0       # MPa: v8 hub-clamp beam surface bending (sm 150/Kt 1.05), axial, jaw bolt pressure
XL, XR, HJ = -4.0, 3.0, 3.0              # strip back end, free end, jaw thickness (mm)
JAWS = {"steel200": (200e3, 0.29), "pa_cf8": (8e3, 0.35), "petg5": (5e3, 0.36), "petg2": (2e3, 0.36)}


def mesh(r, fine):
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    g = gmsh.model.geo
    rr = max(r, 0.02)
    P = lambda x, y: g.addPoint(x, y, 0)
    L = lambda a, b: g.addLine(a, b)

    def surf(lines):
        return g.addPlaneSurface([g.addCurveLoop(lines)])
    a, b, c, d = P(XL, -T / 2), P(XR, -T / 2), P(XR, T / 2), P(XL, T / 2)
    ls = [L(a, b), L(b, c), L(c, d), L(d, a)]
    out = {"strip": surf(ls), "strip_end": ls[1], "strip_left": ls[3], "strip_bot": ls[0], "strip_top": ls[2]}
    for sgn, nm in ((1, "jU"), (-1, "jL")):
        y0 = sgn * T / 2
        p1, p2 = P(XL, y0), P(-rr, y0)
        cen = P(-rr, y0 + sgn * rr)
        p3, p4, p5 = P(0, y0 + sgn * rr), P(0, y0 + sgn * HJ), P(XL, y0 + sgn * HJ)
        l1, arc = L(p1, p2), g.addCircleArc(p2, cen, p3)
        l3, l4, l5 = L(p3, p4), L(p4, p5), L(p5, p1)
        out[nm] = surf([l1, arc, l3, l4, l5])
        out[nm + "_face"] = [l1, arc]
        out[nm + "_left"], out[nm + "_outer"] = l5, l4
    g.synchronize()
    g.mesh.setTransfiniteCurve(ls[3], 7)
    g.mesh.setTransfiniteCurve(ls[1], 9)   # uniform end nodes for consistent nodal forces
    f = gmsh.model.mesh.field
    f.add("Ball", 1)
    for k, v in (("Radius", 0.15), ("VIn", fine), ("VOut", 0.5), ("Thickness", 1.0), ("XCenter", -0.05),
                 ("YCenter", 0.0), ("ZCenter", 0)):
        f.setNumber(1, k, v)
    f.setAsBackgroundMesh(1)
    for k, v in (("Mesh.MeshSizeExtendFromBoundary", 0), ("Mesh.MeshSizeFromPoints", 0), ("Mesh.MeshSizeMax", 0.5),
                 ("Mesh.ElementOrder", 2), ("Mesh.HighOrderOptimize", 0)):
        gmsh.option.setNumber(k, v)
    gmsh.model.mesh.generate(2)
    return out


def build(r, mat, wd: Path, fine=0.012):
    E, nu = mat
    ent = mesh(r, fine)
    nidx, X, elsets = {}, {}, {}

    def nid(tag, body):
        k = (body, tag)
        if k not in nidx:
            nidx[k] = len(nidx) + 1
        return nidx[k]
    for body in ("strip", "jU", "jL"):
        tags, co, _ = gmsh.model.mesh.getNodes(2, ent[body], includeBoundary=True)
        for t_, xyz in zip(tags, np.reshape(co, (-1, 3))):
            X[nid(int(t_), body)] = xyz[:2]
        _, _, en = gmsh.model.mesh.getElements(2, ent[body])
        els = [[nid(int(t_), body) for t_ in e] for e in np.reshape(en[0], (-1, 6))]
        for e in els:
            (x1, y1), (x2, y2), (x3, y3) = X[e[0]], X[e[1]], X[e[2]]
            if (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1) < 0:
                e[:] = [e[0], e[2], e[1], e[5], e[4], e[3]]
        elsets[body] = els

    def line_nodes(body, ln):
        _, _, en = gmsh.model.mesh.getElements(1, ln)
        return {nid(int(t_), body) for t_ in en[0]}
    nsets = {"SLEFT": line_nodes("strip", ent["strip_left"]), "SEND": line_nodes("strip", ent["strip_end"]),
             "JULEFT": line_nodes("jU", ent["jU_left"]), "JLLEFT": line_nodes("jL", ent["jL_left"]),
             "JUOUT": line_nodes("jU", ent["jU_outer"]), "JLOUT": line_nodes("jL", ent["jL_outer"])}
    ucont = set().union(*[line_nodes("jU", l) for l in ent["jU_face"]])
    lcont = set().union(*[line_nodes("jL", l) for l in ent["jL_face"]])
    stop, sbot = line_nodes("strip", ent["strip_top"]), line_nodes("strip", ent["strip_bot"])
    FACE = [(0, 1, 3), (1, 2, 4), (2, 0, 5)]
    base, n0 = {}, 0
    for body in ("strip", "jU", "jL"):
        base[body] = n0
        n0 += len(elsets[body])

    def surface(body, ns, xmax):
        out = {}
        for k, e in enumerate(elsets[body]):
            for fi, (a, b, m) in enumerate(FACE):
                if e[a] in ns and e[b] in ns and e[m] in ns and X[e[m]][0] <= xmax:
                    out.setdefault(fi + 1, []).append(k + 1 + base[body])
        return out
    gmsh.finalize()
    o = ["*NODE"] + [f"{i}, {x[0]:.9g}, {x[1]:.9g}" for i, x in sorted(X.items())]
    for body, nm in (("strip", "STRIP"), ("jU", "JU"), ("jL", "JL")):
        o.append(f"*ELEMENT, TYPE=CPE6, ELSET={nm}")
        o += [f"{base[body] + k + 1}, " + ", ".join(map(str, e)) for k, e in enumerate(elsets[body])]
    for nm, ns in nsets.items():
        o.append(f"*NSET, NSET={nm}")
        s_ = sorted(ns)
        o += [", ".join(map(str, s_[i:i + 12])) for i in range(0, len(s_), 12)]
    for nm, body, ns, xm in (("SUP", "strip", stop, 0.4), ("SLO", "strip", sbot, 0.4), ("MUP", "jU", ucont, 9),
                             ("MLO", "jL", lcont, 9)):
        o.append(f"*SURFACE, NAME={nm}, TYPE=ELEMENT")
        for fi, els in surface(body, ns, xm).items():
            o += [f"{e}, S{fi}" for e in els]
    o += ["*MATERIAL, NAME=ST", "*ELASTIC", "200000, 0.29", "*MATERIAL, NAME=JM", "*ELASTIC", f"{E}, {nu}",
          "*SOLID SECTION, ELSET=STRIP, MATERIAL=ST", "*SOLID SECTION, ELSET=JU, MATERIAL=JM",
          "*SOLID SECTION, ELSET=JL, MATERIAL=JM",
          "*SURFACE INTERACTION, NAME=SI", "*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=LINEAR", "5e6",
          "*CONTACT PAIR, INTERACTION=SI, TYPE=SURFACE TO SURFACE", "SUP, MUP", "SLO, MLO",
          "*BOUNDARY", "SLEFT, 1, 2, 0.", "JULEFT, 1, 1, 0.", "JLLEFT, 1, 1, 0.", "JLOUT, 2, 2, 0."]
    o += ["*STEP, NLGEOM, INC=200", "*STATIC", "0.1, 1., 1e-4, 0.25", "*DLOAD"]
    o += [f"{e}, P{fi}, {P0}" for fi, els in surface("jU", nsets["JUOUT"], 99).items() for e in els]
    o += ["*END STEP"]
    ids = sorted(nsets["SEND"], key=lambda i: X[i][1])
    d = X[ids[1]][1] - X[ids[0]][1]
    w = np.full(len(ids), 2 * d / 3)
    w[1::2] = 4 * d / 3
    w[0] = w[-1] = d / 3
    o += ["*STEP, NLGEOM, INC=200", "*STATIC", "0.1, 1., 1e-4, 0.25", "*CLOAD"]
    o += [f"{i}, 1, {wi * (SIGB * X[i][1] / (T / 2) + SIGA):.9g}" for i, wi in zip(ids, w)]
    o += ["*NODE FILE", "S", "*END STEP"]
    wd.mkdir(parents=True, exist_ok=True)
    (wd / "m.inp").write_text("\n".join(o) + "\n")
    rc = subprocess.run(["nice", "-n", "15", "ccx", "-i", "m"], cwd=wd, capture_output=True, text=True,
                        env={**os.environ, "OMP_NUM_THREADS": "1", "CCX_NPROC_EQUATION_SOLVER": "1"})
    if "*ERROR" in rc.stdout + rc.stderr or rc.returncode:
        raise RuntimeError("\n".join((rc.stdout + rc.stderr).splitlines()[-8:]))
    txt = (wd / "m.frd").read_text().splitlines()
    idx = [i for i, l in enumerate(txt) if l.startswith(" -4  STRESS")]
    S, i = {}, idx[-1] + 7
    while txt[i].startswith(" -1"):
        S[int(txt[i][3:13])] = float(txt[i][13:25])
        i += 1
    surf = [(X[n][0], S[n]) for n in (stop | sbot) if n in S and -2.0 < X[n][0] < 1.0]
    x, s = max(surf, key=lambda p: abs(p[1]))
    return abs(s) / SIGB, x


if __name__ == "__main__":
    base = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/clampedge")
    print("jaw  r_mm  K_clamp  x_peak_mm(<0 under jaw)")
    for jn, jm in JAWS.items():
        for r in (0.0, 0.5, 1.0, 2.0):
            try:
                k, x = build(r, jm, base / f"{jn}_{r}")
                print(f"{jn} {r} {k:.3f} {x:+.3f}", flush=True)
            except Exception as e:
                print(f"{jn} {r} FAIL {str(e)[-300:]}", flush=True)
