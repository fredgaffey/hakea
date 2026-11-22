"""Post-process the ccx runs of a mesh dir (Mac, numpy only) -> MESHDIR/summary.json.

c1*: rotation of the pilot ROT node under unit pure moments about a / s / y -> k_hinge, k_cross, k_flap (N.m/rad)
c2*: strip lamina surface stress (Gauss S, linear fit through the lamina, along the strip axis, /n_lam = per
     lamina) along s; epoxy land shear / peel on the strip mid-plane; fitting von Mises + max principal from
     the .frd nodal stresses (per body: max, location, 99.9th pct); strap tension; bore reaction check
c3*: frequencies + blade-tip motion class per mode
usage: python post.py MESHDIR
"""
import json
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MD = Path(sys.argv[1])
LAY = json.loads((HERE / "layout.json").read_text())
SETS = json.loads((MD / "sets.json").read_text())
X = np.load(MD / "X.npy")
NLAM = LAY["n_lam"]
BMAP = [np.array(b) for b in LAY["blade_map"]]
a = np.array(LAY["hinge_axis"]); a /= np.linalg.norm(a)
s_ax = np.cross([0.0, 1.0, 0.0], a); s_ax /= np.linalg.norm(s_ax)
y_ax = np.array([0.0, 1.0, 0.0])
G3 = 1 / np.sqrt(3)
IPS = np.array([(x, y, z) for z in (-G3, G3) for y in (-G3, G3) for x in (-G3, G3)])
LF, LAND = LAY["strips"][0]["L"], LAY["prm"]["land"]
S_C = np.r_[np.linspace(-LAND, 0, 17)[:-1], np.linspace(0, LF, SETS["NL"] + 1)[:-1], np.linspace(LF, LF + LAND, 17)]
U_C = np.linspace(-LAY["prm"]["w"] / 2, LAY["prm"]["w"] / 2, SETS["NW"] + 1)


def blocks(dat, kind):
    """[(time, {id: values})] for every '<kind> ... for set ... and time t' block of a .dat"""
    out, cur = [], None
    for ln in open(dat):
        if kind in ln and "time" in ln:
            cur = {}
            out.append((float(ln.split()[-1]), cur))
            continue
        p = ln.split()
        if cur is not None and p and re.match(r"^-?\d", p[0]):
            try:
                vals = list(map(float, p))
            except ValueError:
                continue
            key = (int(vals[0]), int(vals[1])) if kind.startswith("stresses") else int(vals[0])
            cur[key] = np.array(vals[2:] if kind.startswith("stresses") else vals[1:])
        elif cur is not None and p and not re.match(r"^-?\d", p[0]):
            cur = None if "for set" not in ln else cur
    return out


def case1(f):
    U = blocks(f, "displacements")
    rot = SETS["next_node"] + 1
    th = [b[rot] for _, b in U[-3:]]
    return dict(k_hinge=1e-3 / (a @ th[0]), k_cross=1e-3 / (s_ax @ th[1]), k_flap=1e-3 / (y_ax @ th[2]),
                axis_rot_dir=(th[0] / np.linalg.norm(th[0])).tolist())


def strip_stress(S):
    out = []
    for st in SETS["strips"]:
        t, n = np.array(st["t"]), np.array(st["n"])
        rows = []
        for e, ar in zip(st["elems"], st["erow"]):
            s0, s1 = S_C[ar], S_C[ar + 1]
            for g in range(4):           # (xi, eta) pairs, zeta = -/+ at g and g + 4
                v = []
                for gz in (g, g + 4):
                    sx, sy, sz, sxy, sxz, syz = S[(e, gz + 1)][:6]
                    T = np.array([[sx, sxy, sxz], [sxy, sy, syz], [sxz, syz, sz]])
                    v.append(t @ T @ t)
                m, d = (v[0] + v[1]) / 2, (v[1] - v[0]) * np.sqrt(3) / 2
                rows.append((s0 + (1 + IPS[g][0]) / 2 * (s1 - s0), max(abs(m + d), abs(m - d)), m))
        r = np.array(rows)
        hub = r[(r[:, 0] > -0.5) & (r[:, 0] < 1.0)]
        bl = r[(r[:, 0] > LF - 1.0) & (r[:, 0] < LF + 0.5)]
        free = r[(r[:, 0] > 0) & (r[:, 0] < LF)]
        mid = r[np.abs(r[:, 0] - LF / 2) < 0.6]
        # profile: max over width per s (per lamina)
        ss = np.unique(np.round(r[:, 0], 4))
        prof = [(float(x), float(r[np.abs(r[:, 0] - x) < 1e-3, 1].max() / NLAM)) for x in ss]
        out.append(dict(tag=f"b{st['k']}.flex{st['i'] - 1}", hub_mouth=hub[:, 1].max() / NLAM, blade_mouth=bl[:, 1].max() / NLAM,
                        free_max=free[:, 1].max() / NLAM, s_at_free_max=float(free[np.argmax(free[:, 1]), 0]),
                        membrane_mid=float(np.median(mid[:, 2])) / NLAM, profile=prof))
    return out


def glue_stress(S):
    out = []
    for st in SETS["strips"]:
        n = np.array(st["n"])
        res = {"hub": [], "blade": []}
        for e, (ar, br, c0) in zip(st["glue"], st["glrow"]):
            s0, s1 = S_C[ar], S_C[ar + 1]
            for g in range(8):
                sx, sy, sz, sxy, sxz, syz = S[(e, g + 1)][:6]
                T = np.array([[sx, sxy, sxz], [sxy, sy, syz], [sxz, syz, sz]])
                tr = T @ n
                pn = tr @ n
                res["hub" if ar < 16 else "blade"].append((s0 + (1 + IPS[g][0]) / 2 * (s1 - s0),
                                                         np.linalg.norm(tr - pn * n), pn))
        d = {}
        for k_, v in res.items():
            v = np.array(v)
            d[k_] = dict(tau_max=float(v[:, 1].max()), tau_mean=float(v[:, 1].mean()), s_tau_max=float(v[np.argmax(v[:, 1]), 0]),
                         peel_max=float(v[:, 2].max()), peel_min=float(v[:, 2].min()))
        out.append(dict(tag=f"b{st['k']}.flex{st['i'] - 1}", **d))
    return out


def frd_stress(f):
    last, cur, mode = None, None, None
    for ln in open(f):
        if ln.startswith(" -4"):
            mode = ln.split()[1]
            cur = {} if mode == "STRESS" else None
        elif ln.startswith(" -1") and cur is not None:
            nd = int(ln[3:13])
            cur[nd] = [float(ln[13 + 12 * i:25 + 12 * i]) for i in range(6)]
        elif ln.startswith(" -3") and cur is not None:
            last, cur = cur, None
    return last


def fittings(f):
    St = frd_stress(f)
    out = {}
    mouths = [(f"b{st['k']}.flex{st['i'] - 1} {sd}", np.array(st["p0"]) + (0 if sd == "hub mouth" else LF) * np.array(st["t"]))
              for st in SETS["strips"] for sd in ("hub mouth", "blade mouth")]
    for nm, key in (("hub", "nodes_hub"), ("blade1", "nodes_b1"), ("blade2", "nodes_b2")):
        ids = np.array([i for i in SETS[key] if i in St])
        A = np.array([St[i] for i in ids])
        sx, sy, sz, sxy, syz, szx = A.T          # frd order: SXX SYY SZZ SXY SYZ SZX
        vm = np.sqrt(0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2) + 3 * (sxy ** 2 + syz ** 2 + szx ** 2))
        T = np.zeros((len(ids), 3, 3))
        T[:, 0, 0], T[:, 1, 1], T[:, 2, 2] = sx, sy, sz
        T[:, 0, 1] = T[:, 1, 0] = sxy; T[:, 1, 2] = T[:, 2, 1] = syz; T[:, 0, 2] = T[:, 2, 0] = szx
        p1 = np.linalg.eigvalsh(T)[:, -1]
        i = int(np.argmax(vm)); j = int(np.argmax(p1))
        near = lambda p: min(((np.linalg.norm(p - q), lb) for lb, q in mouths))
        out[nm] = dict(vm_max=float(vm[i]), vm_p999=float(np.percentile(vm, 99.9)), vm_p99=float(np.percentile(vm, 99)),
                       vm_at=X[ids[i]].round(2).tolist(), vm_at_r=float(np.hypot(*X[ids[i], :2])),
                       vm_near=f"{near(X[ids[i]])[1]} ({near(X[ids[i]])[0]:.2f} mm)",
                       p1_max=float(p1[j]), p1_p999=float(np.percentile(p1, 99.9)), p1_at=X[ids[j]].round(2).tolist(),
                       p1_near=f"{near(X[ids[j]])[1]} ({near(X[ids[j]])[0]:.2f} mm)")
    return out


def case2(stem):
    dat = MD / f"{stem}.dat"
    S = blocks(dat, "stresses (elem")
    # strips and glue are separate *EL PRINT blocks at the same time: merge
    Sall = {}
    for _, b in S:
        Sall.update(b)
    U = {}
    for _, b in blocks(dat, "displacements"):
        U.update(b)
    strap = {}
    k_ = None
    for k in ("1", "2"):
        h, b = SETS["strap"][k]
        L0 = np.linalg.norm(X[b] - X[h]); L1 = np.linalg.norm(X[b] + U[b] - X[h] - U[h])
        strap[k] = dict(stretch_mm=float(L1 - L0), L0=float(L0))
    return dict(strips=strip_stress(Sall), glue=glue_stress(Sall), fittings=fittings(MD / f"{stem}.frd"), strap=strap,
                tips={int(k): v.tolist() for k, v in U.items() if k in set(sum([[t] for t in []], []))})


def case3(stem):
    txt = open(MD / f"{stem}.dat").read()
    fr = []
    if "E I G E N V A L U E   O U T P U T" in txt:
        sec = txt.split("E I G E N V A L U E   O U T P U T")[1].split("P A R T I C I P")[0]
        for ln in sec.splitlines():
            p = ln.split()
            if len(p) >= 4 and p[0].isdigit():
                fr.append(float(p[3]))
    U = blocks(MD / f"{stem}.dat", "displacements")
    tips = None
    modes = []
    for f_, (_, b) in zip(fr, U[-len(fr):] if fr else []):
        ids = sorted(b)
        if tips is None:
            tips = ids
        q = {}
        for k in (1, 2):
            le, te = ids[2 * (k - 1)], ids[2 * (k - 1) + 1]   # TIPS order: b1 LE, b1 TE, b2 LE, b2 TE (sorted ids!)
            Bi = np.linalg.inv(BMAP[k - 1])
            ul, ut = Bi @ b[le], Bi @ b[te]
            xl, xt = Bi @ X[le], Bi @ X[te]
            ch = np.linalg.norm(xl - xt)
            q[k] = dict(flap=b[le][2] / 2 + b[te][2] / 2, lag=(ul[1] + ut[1]) / 2, pitch=(ul[2] - ut[2]) / ch * 25.0)
        amp = {c: abs(q[1][c]) + abs(q[2][c]) for c in ("flap", "lag", "pitch")}
        c = max(amp, key=amp.get)
        sym = "sym" if q[1][c] * q[2][c] * (1 if c == "flap" else -1) > 0 else "antisym"
        modes.append(dict(f_hz=f_, label=f"{c} {sym}", parts={k_: round(v / max(amp.values()), 3) for k_, v in amp.items()}))
    return modes


if __name__ == "__main__":
    out = dict(mesh=dict(H=SETS["H"], NL=SETS["NL"], NW=SETS["NW"], ntet=SETS["ntet"], nnodes=SETS["nnodes"]),
               strap_e_mm=SETS.get("strap_e"))
    for f in sorted(MD.glob("c*.dat")):
        stem = f.stem
        try:
            if stem.startswith("c1"):
                out[stem] = case1(f)
            elif stem.startswith("c2"):
                out[stem] = case2(stem)
            elif stem.startswith("c3"):
                out[stem] = case3(stem)
        except Exception as ex:       # incomplete run
            out[stem] = dict(error=repr(ex))
        print(stem, json.dumps({k: v for k, v in out[stem].items() if k not in ("strips", "glue")} if isinstance(out[stem], dict) else out[stem], default=float)[:600])
    (MD / "summary.json").write_text(json.dumps(out, indent=1, default=float))
