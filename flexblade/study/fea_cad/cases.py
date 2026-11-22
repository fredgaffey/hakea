"""Write the ccx decks for a mesh dir made by build.py (units mm, N, t, s, MPa).

c1_<MAT>.inp  hinge stiffness: hub bore encastre, blade-1 outboard of r_aero + 3 mm rigid (pilot at the hinge
              point), three linear steps with a 1 N.mm pure moment about a (hinge axis), s (= y x a), y (flap)
c2_<MAT>.inp  hover static, NLGEOM: CENTRIF at the design hover rpm + lumped thrust (T/2 per blade over
              0.65-0.85 R, +z)
c3_<MAT>.inp  step 1 = c2 loads (CF only), step 2 = *FREQUENCY 8 modes (stress-stiffened, rotating frame,
              no Coriolis);  c3s_<MAT>.inp = same at rest (no pre-stress)
usage: python cases.py MESHDIR
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MD = Path(sys.argv[1])
LAY = json.loads((HERE / "layout.json").read_text())
BR = json.loads((HERE / "beam_ref.json").read_text())
SETS = json.loads((MD / "sets.json").read_text())
X = np.load(MD / "X.npy")
NLAM = LAY["n_lam"]
ST = LAY["materials"]["steel_301_fh"]
AR = LAY["materials"]["aramid_yarn"]
FITMAT = {"STIFF": (210e3, 0.3, 7.9e-9), "PACF": (7.5e3, 0.35, 1.2e-9), "PETG": (2.0e3, 0.38, 1.27e-9)}
RPM = BR["design"]["rpm"]
OM2 = (RPM * 2 * np.pi / 60) ** 2
THRUST = BR["design"]["thrust_N"]
BMAP = [np.array(b) for b in LAY["blade_map"]]
R = LAY["R"]
a = np.array(LAY["hinge_axis"]); a /= np.linalg.norm(a)
s = np.cross([0.0, 1.0, 0.0], a); s /= np.linalg.norm(s)
y = np.array([0.0, 1.0, 0.0])
nn = SETS["next_node"]
REF, ROT = nn, nn + 1


def nset(name, ids):
    ids = list(map(int, ids))
    return [f"*NSET, NSET={name}"] + [", ".join(map(str, ids[i:i + 12])) for i in range(0, len(ids), 12)]


def blade_local(k, ids):
    return (np.linalg.inv(BMAP[k - 1]) @ X[ids].T).T


def common(mat, strap=True):
    E, nu, rho = FITMAT[mat]
    A = np.pi / 4 * LAY["prm"]["strand_d"] ** 2
    o = ["*INCLUDE, INPUT=mesh.inp"]
    sp0 = SETS["spring_ids"][0]
    o += ["*ELSET, ELSET=STRAPS", f"{sp0}, {sp0 + 1}"]
    hk, bk = SETS["strap"]["1"]
    Ls = float(np.linalg.norm(X[bk] - X[hk]))
    o += ["*MATERIAL, NAME=FIT", "*ELASTIC", f"{E:.6g}, {nu}", "*DENSITY", f"{rho:.6g}",
          "*MATERIAL, NAME=STEEL9", "*ELASTIC", f"{NLAM * ST['E'] / 1e6:.6g}, {ST['nu']}", "*DENSITY", f"{NLAM * ST['rho'] * 1e-12:.6g}",
          "*MATERIAL, NAME=EPOXY", "*ELASTIC", "3000, 0.35", "*DENSITY", "1.15e-9",
          "*SOLID SECTION, ELSET=FITS, MATERIAL=FIT", "*SOLID SECTION, ELSET=STRIPS, MATERIAL=STEEL9",
          "*SOLID SECTION, ELSET=GLUES, MATERIAL=EPOXY"]
    o += ["*SPRING, ELSET=STRAPS", "", f"{AR['E'] / 1e6 * A / Ls * (1.0 if strap else 1e-9):.6g}"]
    tips = []
    for k in (1, 2):
        ids = np.array(SETS[f"nodes_b{k}"])
        xl = blade_local(k, ids)
        m = xl[:, 0] > 0.93 * R
        tips += [int(ids[m][np.argmax(xl[m, 1])]), int(ids[m][np.argmin(xl[m, 1])])]
    o += nset("TIPS", tips) + ["*BOUNDARY", "BORE, 1, 3, 0."]
    return o, tips


def c1(mat, strap=False):
    o, _ = common(mat, strap)
    ids = np.array(SETS["nodes_b1"])
    rig = ids[blade_local(1, ids)[:, 0] > LAY["r_aero"] + 3.0]
    h = BMAP[0] @ np.array(LAY["hinge"])
    o += ["*NODE", f"{REF}, {h[0]:.7g}, {h[1]:.7g}, {h[2]:.7g}", f"{ROT}, {h[0]:.7g}, {h[1]:.7g}, {h[2]:.7g}"]
    o += nset("RIG1", rig) + [f"*RIGID BODY, NSET=RIG1, REF NODE={REF}, ROT NODE={ROT}"] + nset("PIL", [REF, ROT])
    for v in (a, s, y):
        o += ["*STEP", "*STATIC", "*CLOAD, OP=NEW"] + [f"{ROT}, {i + 1}, {v[i]:.9g}" for i in range(3)]
        o += ["*NODE PRINT, NSET=PIL", "U", "*END STEP"]
    return o


def hover_loads(thrust=True):
    o = ["*DLOAD", f"SOLIDS, CENTRIF, {OM2:.7g}, 0., 0., 0., 0., 0., 1."]
    if thrust:
        o.append("*CLOAD")
        for k in (1, 2):
            ids = np.array(SETS[f"nodes_b{k}"])
            r = blade_local(k, ids)[:, 0]
            sel = ids[(r > 0.65 * R) & (r < 0.85 * R)]
            o += [f"{n}, 3, {THRUST / 2 / len(sel):.6g}" for n in sel]
    return o


def c2(mat):
    o, _ = common(mat)
    o += nset("STRAPN", [n for k in ("1", "2") for n in SETS["strap"][k]])
    o += ["*STEP, NLGEOM, INC=200", "*STATIC", "0.25, 1., 1e-5, 0.5"] + hover_loads()
    o += ["*NODE FILE, FREQUENCY=1000", "U", "*EL FILE, FREQUENCY=1000", "S",
          "*EL PRINT, ELSET=STRIPS, FREQUENCY=1000", "S", "*EL PRINT, ELSET=GLUES, FREQUENCY=1000", "S",
          "*NODE PRINT, NSET=STRAPN, FREQUENCY=1000", "U", "*NODE PRINT, NSET=TIPS, FREQUENCY=1000", "U",
          "*NODE PRINT, NSET=BORE, TOTALS=ONLY, FREQUENCY=1000", "RF", "*END STEP"]
    return o


def c3(mat, rotating=True):
    o, _ = common(mat)
    if rotating:
        o += ["*STEP, NLGEOM, INC=200", "*STATIC", "0.25, 1., 1e-5, 0.5"] + hover_loads(False) + ["*END STEP"]
    o += ["*STEP", "*FREQUENCY", "8", "*NODE PRINT, NSET=TIPS", "U", "*END STEP"]
    return o


if __name__ == "__main__":
    for mat in ("STIFF", "PACF", "PETG"):
        (MD / f"c1_{mat}.inp").write_text("\n".join(c1(mat)) + "\n")
    (MD / "c1s_STIFF.inp").write_text("\n".join(c1("STIFF", True)) + "\n")
    (MD / "c1s_PACF.inp").write_text("\n".join(c1("PACF", True)) + "\n")
    for mat in ("PACF", "PETG"):
        (MD / f"c2_{mat}.inp").write_text("\n".join(c2(mat)) + "\n")
        (MD / f"c3_{mat}.inp").write_text("\n".join(c3(mat)) + "\n")
        (MD / f"c3s_{mat}.inp").write_text("\n".join(c3(mat, False)) + "\n")
    print("rpm", RPM, "Omega^2", OM2, "thrust", THRUST, "ref/rot", REF, ROT)
