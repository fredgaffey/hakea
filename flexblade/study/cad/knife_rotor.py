r"""Knife-flexure 5" flexblade rotor: ONE watertight print, strips dropped in at pauses (2026-11-22).

Topology = the FACT ideal for the lead (two sheets whose planes both contain the skewed hinge axis a),
with the lead's kinematics (hinge point, axis, strip lines, mouths, free length L, 105 deg crossing).
Realised as one printed part. Section across the hinge axis (blade-1 frame, u across, v ~ up):

                  knife (blade root)
          ______  \              /  ______
           hub     \  web    web  /    hub        knife: a V that sits in a V-groove of the hub
          (stop)    \     /\     /   (stop)       webs: printed plates along the two strip lines,
                     \  /    \  /                 each with an internal slit holding the steel/G10
                  web \/ cradle\/ web             strip (strip = the spring, web = the body round it)
          hub          (hub)           hub        kerfs between knife flank and hub = travel stops

  * webs are offset along a (strip 0 at +a, strip 1 at -a): no fused crossing (cartwheel = 4x stiffer);
  * each slit's top end is cut on a level line of the strip plane, so the strip slides in along its own
    length from the paused layer, sits wholly below it, and the next layers seal it (pause per height);
  * end gaps (planes normal to a) free the knife from the hub at its inboard end and the hub cradle from
    the blade at its outboard end, so hub and blade meet only through the two webs.

usage: python study/cad/knife_rotor.py DESIGN.json OUT.json [key=value ...]
       python study/cad/knife_rotor.py DESIGN.json OUT.json free=1    # cavities without webs: must split in 3
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clean_rotor import blade_loft, frame  # noqa: E402
from flexblade.cad.solidcraft import naca4  # noqa: E402

from flexblade.cad.solidcraft import _Script, _blade_map, _r, rotor_layout  # noqa: E402

OV = dict(n_lam=3, t_lam=0.10, width=4.0, slot_clear=0.25, blade_extra=0.0, mouth_chamfer=0.5, boss_h=6.5)
P = dict(tw=1.6,        # printed web thickness, mm (slit + two skins)
         c=0.4,         # web clearance to knife / hub (> mid-web sway theta*L/8 = 0.33 at 15 deg)
         d2=1.2,        # knife wraps the blade land this far below the strip line past the mouth
         stop=12.0,     # stair sizing angle, deg (checked: clear at +-10, stair contacts at +-11)
         land=3.0,      # strip land past the top mouth at mid-width (level-cut end: +-1.74 across width)
         land_b=3.0,    # strip land past the bottom mouth (level-cut too)
         e=0.6,         # web edge wall past the strip along a
         g=0.8,         # end-gap width (planes normal to a)
         a_end=5.8,     # knife inboard end / cradle outboard end, |a| mm
         t_stack=0.45,  # strip: 9 x 0.05 steel (or one G10 strip of the same EI)
         slit_t=0.6, slit_w=4.3, zc=0.15,   # slit thickness, width; top clearance above strip (z)
         W=9.0, X=22.5, r_plan=3.0, zb=-7.2, zt=7.2,   # hub bar: |y| <= W, |x| <= X, z range
         chisel=46.0, x_ch=16.5,           # end taper (deg from horizontal; < 45 deg overhang) from |x|
         bore=5.1, nut_af=8.0, nut_seat=6.0,
         te=0.02, tip=7.0, free=0,
         slits=1,       # 0: solid printed flexure (no shim slits, no strips, no pauses)
         paddle=0,      # 1: flat T-section test paddles instead of the aero blades
         pad_x0=19.5, pad_x1=60.0, pad_w=12.0, pad_t=3.0, pad_rib=23.0, pad_rw=6.0, pad_hole=2.5, pad_hx=55.0, pad_hy=4.5,
         inb=(("in0", 20.0, 15.0, 0.8, 0.0, 0.3),),
         blade=2,       # 1: clean_rotor.blade_loft (9 sections, 4412, 2 % TE); 2: blade_loft2 below
         n_sec=18, tc_root=15, tc_tip=9, tc_in=20, te_abs=0.45, tip_min=0.3)


def section(lay, p):
    """2D section polylines (u, v) of blade 1, about the hinge point."""
    h, u, v, a = frame(lay)
    q = lay["strips"][0]["p1"] - h
    um, vm = float(q @ u), float(q @ v)
    th, rm = math.atan2(vm, um), math.hypot(um, vm)
    c, s = math.cos(th), math.sin(th)
    e0, n0 = np.array([c, s]), np.array([-s, c])
    d = p["tw"] / 2 + p["c"]
    A = np.array([0.0, d / c])
    P1, P2, P3 = rm * e0 + d * n0, rm * e0 - p["d2"] * n0, 40 * e0 - p["d2"] * n0
    # hub-side kerf as a STAIR of treads parallel to the strip line and risers square to it: every face then
    # stays <= 45 deg overhang for blade 1 AND for its point-mirror blade 2 (a turned kerf plane would be a
    # 55 deg ceiling on blade 2). Tread depth = what the knife/web underside sweeps when turned `stop` deg.
    k = math.radians(p["stop"])
    need = lambda s_, b: b * math.cos(k) + s_ * math.sin(k) + 0.3
    steps = [(2.0, need(2.0, p["tw"] / 2)), (rm, need(rm, p["tw"] / 2)), (8.0, need(8.0, p["d2"])), (40.0, need(14.0, p["d2"]))]
    o1 = max(steps[0][1], d)
    pts = [np.array([0.0, -o1 / c])]
    for j, (sj, oj) in enumerate(steps):
        oj = max(oj, d)
        if j:
            pts.append(steps[j - 1][0] * e0 - oj * n0)
        pts.append(sj * e0 - oj * n0)
    mi = lambda X: np.array([-X[0], X[1]])
    line = [mi(P3), mi(P2), mi(P1), A, P1, P2, P3]
    kerf = pts[::-1] + [mi(q) for q in pts[1:]]
    return dict(th=th, rm=rm, d=d, line=line, kerf=kerf, stair=[(sj, max(oj, d)) for sj, oj in steps], big=45.0)


def sk_prism(S, name, Lm, origin, e1, e2, poly, dist):
    """Polygon in the (e1, e2) plane through `origin` (blade-1 frame), extruded `dist` along e1 x e2, placed
    by Lm. A mirroring Lm keeps a right-handed sketch frame by flipping x_dir and the polygon's x."""
    sg = 1.0 if np.linalg.det(Lm) > 0 else -1.0
    S.add("sketch.create", plane={"origin": _r(Lm @ np.asarray(origin, float)), "x_dir": _r(sg * (Lm @ e1), 9),
                                  "y_dir": _r(Lm @ e2, 9)}, name=name + "_sk")
    pts = [(sg * x, y) for x, y in poly]
    S.add("sketch.line", points=[_r(q) for q in (pts if sg > 0 else pts[::-1])], closed=True)
    S.add("sketch.finish")
    S.add("solid.extrude", sketch=name + "_sk", distance=round(dist, 5), operation="new", body_name=name)


def strip_geom(lay, Lm, f, p, grow_w=0.0, top_ext=0.0, bot_ext=0.0):
    """Strip/slit outline in its own plane (sigma along t from the crossing, omega along w), with the top end
    on a level line (world z) so it slides in along t from the paused layer. Returns poly, centre, pause z."""
    L2 = lay["prm"].L / 2
    tz, wz = float((Lm @ f["t"])[2]), float((Lm @ f["w"])[2])
    sg = 1.0 if tz > 0 else -1.0
    c = (f["p0"] + f["p1"]) / 2
    st = sg * (L2 + p["land"]) + sg * top_ext / abs(tz)
    hw = lay["prm"].w / 2 + grow_w
    top = lambda om: st - om * wz / tz
    bot = lambda om: -sg * (L2 + p["land_b"] + bot_ext) - om * wz / tz
    poly = [(bot(-hw), -hw), (top(-hw), -hw), (top(hw), hw), (bot(hw), hw)]
    z_top = float((Lm @ c)[2] + st * tz)
    return poly, c, z_top


def plate_in_strip_plane(S, name, Lm, f, poly, c, thk):
    t, w = np.asarray(f["t"], float), np.asarray(f["w"], float)
    nn = np.cross(t, w)
    sk_prism(S, name, Lm, c - thk / 2 * nn, t, w, poly, thk)


def hinge_cuts(S, lay, k, p, target):
    """Cut blade k's hinge into `target`: knife/cradle band, lower web slots, end gaps (webs kept)."""
    h, u, v, a = frame(lay)
    Lm = _blade_map(k, lay["n"])
    sec = section(lay, p)
    L2, hw = lay["prm"].L / 2, lay["prm"].w / 2
    a0, a1, g, B = -p["a_end"], p["a_end"], p["g"], sec["big"]
    webs = []
    for i, f in enumerate(lay["strips"]):
        nm = f"web{k}{i}"
        S.obox(nm, Lm, (f["p0"] + f["p1"]) / 2, f["t"], f["w"], np.cross(f["t"], f["w"]),
               (-L2 - 0.6, L2 + 0.6), (-hw - p["e"], hw + p["e"]), (-p["tw"] / 2, p["tw"] / 2))
        webs.append(nm)
    tools = []
    band = [tuple(q) for q in sec["line"]] + [tuple(q) for q in sec["kerf"]]
    sk_prism(S, f"band{k}", Lm, h + a0 * a, u, v, band, a1 - a0)
    tools.append(f"band{k}")
    for i, f in enumerate(lay["strips"]):
        nm = f"lslot{k}{i}"
        S.obox(nm, Lm, (f["p0"] + f["p1"]) / 2, f["t"], f["w"], np.cross(f["t"], f["w"]),
               (-L2, 0.0), (-hw - p["e"] - p["c"], hw + p["e"] + p["c"]), (-sec["d"], sec["d"]))
        tools.append(nm)
    if not p["free"]:
        for tl in tools:
            S.cut(tl, webs, keep=True)
    knife = [tuple(q) for q in sec["line"]] + [(sec["line"][-1][0], B), (sec["line"][0][0], B)]
    below = [tuple(q) for q in sec["line"]] + [(sec["line"][-1][0], -B), (sec["line"][0][0], -B)]
    sk_prism(S, f"gapK{k}", Lm, h + (a0 - g) * a, u, v, knife, g + 0.1)   # +0.1 overlap: no knife/cradle edge contact
    sk_prism(S, f"gapC{k}", Lm, h + a1 * a, u, v, below, g)
    tools += [f"gapK{k}", f"gapC{k}"]
    S.cut(target, tools)
    S.add("solid.remove", bodies=webs)


def slit_cuts(S, lay, k, p, target):
    """Strip slit bodies (through web and both lands), top end level at the pause height. Returned, not cut:
    SMLib takes the four internal voids only as ONE cut after the hinge cavities (a second separate void cut
    fails its gap check)."""
    Lm = _blade_map(k, lay["n"])
    pz = []
    for i, f in enumerate(lay["strips"]):
        poly, c, zp = strip_geom(lay, Lm, f, p, grow_w=(p["slit_w"] - lay["prm"].w) / 2, top_ext=p["zc"], bot_ext=0.1)
        plate_in_strip_plane(S, f"slit{k}{i}", Lm, f, poly, c, p["slit_t"])
        pz.append(zp)
    return pz


def envelope(S, p):
    S.add("sketch.create", plane={"origin": [0, 0, p["zb"]], "x_dir": [1, 0, 0], "y_dir": [0, 1, 0]}, name="env_sk")
    S.add("sketch.rectangle.rounded", p0=[-p["X"], -p["W"]], p1=[p["X"], p["W"]], radius=p["r_plan"])
    S.add("sketch.finish")
    S.add("solid.extrude", sketch="env_sk", distance=round(p["zt"] - p["zb"], 5), operation="new", body_name="rotor")
    # chisel ends: top and bottom faces fall at `chisel` deg from |x| = x_ch toward the blade root
    ca, sa = math.cos(math.radians(p["chisel"])), math.sin(math.radians(p["chisel"]))
    I = np.eye(3)
    for sx in (1, -1):
        for sz in (1, -1):
            nm = f"ch{'p' if sx > 0 else 'm'}{'p' if sz > 0 else 'm'}"
            e1 = np.array([sx * ca, 0.0, -sz * sa])            # down the taper, outboard
            e2 = np.array([0.0, 1.0, 0.0])
            e3 = np.cross(e1, e2)
            o = np.array([sx * p["x_ch"], 0.0, sz * p["zt"]])
            out = 1.0 if (e3 @ np.array([sx * sa, 0, sz * ca])) > 0 else -1.0   # side away from the part
            S.obox(nm, I, o, e1, e2, e3, (-5, 40), (-30, 30), (0, 30) if out > 0 else (-30, 0))
            S.cut("rotor", [nm])


def hub_bore(S, p):
    S.cyl("bore", np.eye(3), (0, 0, p["zb"] - 1), (0, 0, p["zt"] + 1), p["bore"])
    S.cut("rotor", ["bore"])
    rh = (p["nut_af"] + 0.3) / math.sqrt(3)
    S.plate("nut", [(rh * math.cos(math.pi / 6 + j * math.pi / 3), rh * math.sin(math.pi / 6 + j * math.pi / 3))
                    for j in range(6)], p["zb"] + p["nut_seat"], 20.0)
    S.cut("rotor", ["nut"])


def _inside(poly, pts):
    """Even-odd point-in-polygon for an (N, 2) array."""
    poly = np.asarray(poly, float)
    x, y = pts[:, 0], pts[:, 1]
    ins = np.zeros(len(pts), bool)
    for (x0, y0), (x1, y1) in zip(poly, np.roll(poly, -1, axis=0)):
        cr = (y0 > y) != (y1 > y)
        xi = x0 + (y - y0) * (x1 - x0) / np.where(y1 == y0, 1e-12, y1 - y0)
        ins ^= cr & (x < xi)
    return ins


def checks(lay, p):
    """Numbers that decide printability/assembly before any CAD: cavity distance from the shaft (nut wall),
    slit-corner walls to the outside, pause heights."""
    h, u, v, a = frame(lay)
    sec = section(lay, p)
    band = [tuple(q) for q in sec["line"]] + [tuple(q) for q in sec["kerf"]]
    g = np.mgrid[-12:12:0.1, -12:12:0.1].reshape(2, -1).T
    gi = g[_inside(band, g)]
    rmin = 1e9
    for aa in np.arange(-p["a_end"], p["a_end"] + 1e-9, 0.2):
        X = h + np.outer(gi[:, 0], u) + np.outer(gi[:, 1], v) + aa * a
        X = X[(X[:, 2] > p["zb"] + p["nut_seat"] - 0.5) & (X[:, 2] < p["zt"]) & (np.abs(X[:, 1]) < p["W"])]
        if len(X):
            rmin = min(rmin, float(np.min(np.hypot(X[:, 0], X[:, 1]))))
    out = {"band_min_r_from_shaft": round(rmin, 2), "nut_circumradius": round((p["nut_af"] + 0.3) / math.sqrt(3), 2)}
    tch = math.tan(math.radians(p["chisel"]))
    walls = []
    for k in range(1, lay["n"] + 1):
        Lm = _blade_map(k, lay["n"])
        for i, f in enumerate(lay["strips"]):
            poly, c, zp = strip_geom(lay, Lm, f, p, grow_w=(p["slit_w"] - lay["prm"].w) / 2, top_ext=p["zc"], bot_ext=0.1)
            t, w = np.asarray(f["t"], float), np.asarray(f["w"], float)
            nn = np.cross(t, w)
            for sgm, om in poly:
                for sn in (-1, 1):
                    q = Lm @ (c + sgm * t + om * w + sn * p["slit_t"] / 2 * nn)
                    zc = p["zt"] - max(abs(q[0]) - p["x_ch"], 0) * tch
                    walls.append((min(p["zt"] - abs(q[2]) if q[2] > 0 else q[2] - p["zb"], p["W"] - abs(q[1]),
                                      zc - abs(q[2])), k, i))
            out[f"pause_z_b{k}s{i}"] = round(zp, 2)
    wmin = min(walls)
    out["slit_min_wall"] = (round(wmin[0], 2), f"blade{wmin[1]} strip{wmin[2]}")
    return out


def paddle(S, p, k, join=True):
    """Flat test paddle on blade k's side (blade 2 = -x): plate |y|<=pad_w/2, |z|<=pad_t/2 from |x|=pad_x0 (inside
    the blade-side end chunk: outboard of the cradle gap plane for |z|<=1.5 is |x|>19.02), plus a 45-deg gusset and
    rib down to the bed from |x|=pad_rib (clear of the envelope) so it prints without support. Two vertical holes
    at (pad_hx, +-pad_hy) to hang weights (torque about the ~radial hinge axis)."""
    sg = 1.0 if k == 1 else -1.0
    hw, ht, zb = p["pad_w"] / 2, p["pad_t"] / 2, p["zb"]
    ey, ez = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
    I3 = np.eye(3)
    x0 = p["pad_x0"] if k == 1 else -p["pad_x1"]
    sk_prism(S, f"pad{k}", I3, np.array([x0, 0.0, 0.0]), ey, ez,
             [(-hw, -ht), (hw, -ht), (hw, ht), (-hw, ht)], p["pad_x1"] - p["pad_x0"])
    xr = p["pad_rib"] if k == 1 else -p["pad_x1"]
    rw = p["pad_rw"] / 2
    rib = [(-hw, ht), (-hw, -ht), (-rw, -ht - (hw - rw)), (-rw, zb), (rw, zb), (rw, -ht - (hw - rw)), (hw, -ht), (hw, ht)]
    sk_prism(S, f"padr{k}", I3, np.array([xr, 0.0, 0.0]), ey, ez, rib, p["pad_x1"] - p["pad_rib"])
    S.join(f"pad{k}", [f"padr{k}"])
    for j, yy in enumerate((p["pad_hy"], -p["pad_hy"])):
        S.cyl(f"padh{k}{j}", I3, (sg * p["pad_hx"], yy, zb - 1), (sg * p["pad_hx"], yy, ht + 1), p["pad_hole"])
    S.cut(f"pad{k}", [f"padh{k}0", f"padh{k}1"])
    if join:
        S.join("rotor", [f"pad{k}"])


def blade_loft2(S, lay, k, name, p):
    """Blade v2: same design chord/twist, smoother + printable.
    * n_sec sections, sine-spaced: even at the root (uneven spacing there made the loft spline overshoot into a
      hook under the root), dense at the tip
    * thickness tapers tc_root -> tc_tip % (NACA 44tt, same camber line as 4412): stiff root, thin fast tip
    * trailing edge a fixed te_abs mm thick (two 0.4 mm lines can print it; a 2 % TE is 0.25 mm = not printable)
    * root fairing: the in-bar section is a tc_in % thick section, so the blade grows out of the bar end
      as a smooth thick-to-thin taper instead of a step
    * elliptic tip over the last `tip` mm down to tip_min x chord"""
    D, R, n = lay["D"], lay["R"], lay["n"]
    st = sorted(D.blade.stations, key=lambda s_: s_.r_frac)
    rf = np.array([s_.r_frac * R for s_ in st])
    ch = np.array([s_.chord * 1e3 for s_ in st])
    tw = np.array([s_.twist_deg for s_ in st])

    def txt(tc, chord):
        te = min(p["te_abs"] / chord, 0.06)
        pts = naca4(f"44{int(round(tc)):02d}", n=41)
        m = len(pts) // 2
        pts = [(x, y + te * x / 2) for x, y in pts[:m + 1]] + [(x, y - te * x / 2) for x, y in pts[m + 1:]]
        return "NACA44\n" + "\n".join(f"{x:.6f} {y:.6f}" for x, y in pts) + "\n"

    ang = 2 * math.pi * (k - 1) / n
    er = np.array([math.cos(ang), math.sin(ang), 0.0])
    xd = -np.cross([0, 0, 1.0], er)
    r0, tip = lay["r_aero"], p["tip"]
    t = np.linspace(0.0, 1.0, p["n_sec"])
    rr = r0 + (R - r0) * (0.5 * t + 0.5 * np.sin(np.pi / 2 * t))   # ~3 mm steps at the root, ~1.2 at the tip
    def chs(r):
        c = float(np.interp(r, rf, ch))
        if r > R - tip:
            c *= math.sqrt(max(1 - ((r - (R - tip)) / tip) ** 2, 0.0)) * (1 - p["tip_min"]) + p["tip_min"]
        return c
    zs = -1.0 if k % 2 == 0 else 1.0
    tw0 = float(np.interp(r0, rf, tw))
    secs = []
    for nm, r, c, f, z, pv in p["inb"]:
        S.add("sketch.create", plane={"origin": _r(r * er + np.array([0, 0, zs * z])), "x_dir": _r(xd, 9), "y_dir": [0, 0, 1]},
              name=f"{name}_{nm}")
        S.add("sketch.insert_airfoil", text=txt(p["tc_in"], c), chord=c, pivot=pv, at=[0, 0], angle=round(-f * tw0, 4))
        S.add("sketch.finish")
        secs.append({"sketch": f"{name}_{nm}"})
    for j, r in enumerate(rr):
        sk = f"{name}_af{j}"
        c = chs(r)
        tc = p["tc_root"] + (p["tc_tip"] - p["tc_root"]) * (r - r0) / (R - r0)
        S.add("sketch.create", plane={"origin": _r(r * er), "x_dir": _r(xd, 9), "y_dir": [0, 0, 1]}, name=sk)
        S.add("sketch.insert_airfoil", text=txt(tc, c), chord=round(c, 4), pivot=0.25,
              at=[0, 0], angle=round(-float(np.interp(r, rf, tw)), 4))
        S.add("sketch.finish")
        secs.append({"sketch": sk})
    S.add("solid.loft", sections=secs, body_name=name)


def build(design, p=P, ov=OV, blades=True, strips=True):
    lay = rotor_layout(design, ov)
    S = _Script()
    envelope(S, p)
    pauses = {}
    for k in range(1, lay["n"] + 1):
        hinge_cuts(S, lay, k, p, "rotor")
    if p["slits"]:
        for k in range(1, lay["n"] + 1):
            pauses[k] = slit_cuts(S, lay, k, p, "rotor")
        S.cut("rotor", [f"slit{k}{i}" for k in range(1, lay["n"] + 1) for i in range(len(lay["strips"]))])
    else:
        strips = False
    hub_bore(S, p)
    if p["paddle"]:
        for k in (1, 2):
            paddle(S, p, k, join=not p["free"])
    elif blades:
        for k in range(1, lay["n"] + 1):
            (blade_loft2 if p["blade"] == 2 else blade_loft)(S, lay, k, f"aero{k}", p)
            S.join("rotor", [f"aero{k}"])
    if strips:
        for k in range(1, lay["n"] + 1):
            Lm = _blade_map(k, lay["n"])
            for i, f in enumerate(lay["strips"]):
                poly, c, _ = strip_geom(lay, Lm, f, p)
                plate_in_strip_plane(S, f"strip{k}{i}", Lm, f, poly, c, p["t_stack"])
    return S.cmds, lay, pauses


if __name__ == "__main__":
    d = json.load(open(sys.argv[1]))
    p = dict(P)
    for kv in sys.argv[3:]:
        k_, v_ = kv.split("=")
        p[k_] = (tuple(("in%d" % j, *[float(x) for x in sec.split(":")][:3], 0.0, float(sec.split(":")[3]))
                       for j, sec in enumerate(v_.split(","))) if k_ == "inb" else type(P[k_])(v_))
    cmds, lay, pauses = build(d, p, blades=not p["free"], strips=not p["free"])
    json.dump(cmds, open(sys.argv[2], "w"))
    sec = section(lay, p)
    print(len(cmds), "commands; theta", round(math.degrees(sec["th"]), 2), "rm", round(sec["rm"], 3), "d", sec["d"])
    print(checks(lay, p))
