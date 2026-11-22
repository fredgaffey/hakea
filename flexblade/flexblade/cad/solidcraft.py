"""SolidCraft (the kernel inside Keel CAD) model of a crossed-strip flexure rotor.

`rotor_commands(design)` returns a pure modelling script ({"commands": [...]}, no file I/O, no paths),
so it replays unchanged in the CLI (`solidcraft-cli run`) and in Keel's page
(`solidcraft.execute(command, params)`). Bodies, each a single solid:

  hub            printed hub: motor boss + bore, per-blade spine (strap channel + knot well) and two
                 arm plates carrying the hub-side U jaws (slot open on the crossing edge, datum wall on
                 the other), arm roots filleted to the boss
  blade{k}       printed blade root fitting: shank (strap channel + knot well), two arm plates and the
                 blade-side U jaws (blind, over-deep slots), joined to a lofted blade (`blade_loft_commands`)
  strip_b{k}_{i} 9-lamina strip stack (one solid), sitting in its two slots: land in the hub jaw, free span,
                 land in the blade jaw
  strap_b{k}     aramid strand (centrifugal tie) from the hub knot well to the blade knot well

Strip frames (exit points, directions, crossing) come from the analysis model (fittings.strip_members), so
the CAD and the beam model share the same flexure. Every dimension is the 5in_301 print package's
(study/build/build_5in_301.py, BUILD.md, fittings.StripParams); see the constants below. Blade k>1 is
blade 1 rotated about z, with even blades' flexure + fittings mirrored in z (geometry.members_for_blade);
the aero blade is only rotated (airfoils are not mirrored). Frame: mm, hub axis z, blade 1 along +x,
leading edge +y.
"""
from __future__ import annotations

import copy
import math

import numpy as np

from ..schema import Design
from .. import fittings as FT

# print-package numbers (build_5in_301.py / BUILD.md / fittings.py), none new
BOSS_OD, BOSS_H, BORE_D = 10.0, 12.0, 5.1     # M5 motor boss, washer faces z +-6
SLOT_CLEAR = 0.25                             # slot = stack + 0.25 (easy build), width = w + 2 x 0.1
ARM_T = FT.WEB_T_SLIM                         # 2.0 mm web / arm plate
OPEN_LIP = FT.OPEN_LIP                        # 0.5 mm strip overhang past an open (U) jaw edge
STRIP_CLEAR = FT.STRIP_CLEAR                  # 0.5 mm keep-out round the free span
OVERDEEP = FT.OVERDEEP                        # 3.0 mm blade slot depth beyond the land
CH_LEN = FT.CH_LEN                            # 8.0 mm strand channel to the knot well


def _r(x, nd=5):
    return [round(float(v), nd) for v in np.ravel(x)]


def _hull2(pts):
    """2-D convex hull (monotone chain), CCW, no repeated end point."""
    P = sorted({(round(float(x), 6), round(float(y), 6)) for x, y in pts})
    if len(P) < 3:
        return P

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def naca4(code: str = "4412", n: int = 31):
    """Selig-ordered (TE, upper, LE, lower, TE) closed-TE NACA 4-digit section, unit chord.
    4412 is the design airfoil and is Clark-Y-like (12 %, 4 % camber, near-flat lower surface)."""
    m, p, tt = int(code[0]) / 100, int(code[1]) / 10, int(code[2:]) / 100
    x = (1 - np.cos(np.linspace(0, np.pi, n))) / 2
    yt = 5 * tt * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4)
    if m > 0 and p > 0:
        yc = np.where(x < p, m / p**2 * (2 * p * x - x**2), m / (1 - p)**2 * ((1 - 2 * p) + 2 * p * x - x**2))
        dy = np.where(x < p, 2 * m / p**2 * (p - x), 2 * m / (1 - p)**2 * (p - x))
    else:
        yc = dy = np.zeros_like(x)
    th = np.arctan(dy)
    xu, yu = x - yt * np.sin(th), yc + yt * np.cos(th)
    xl, yl = x + yt * np.sin(th), yc - yt * np.cos(th)
    return list(zip(xu[::-1], yu[::-1])) + list(zip(xl[1:], yl[1:]))


class _Script:
    def __init__(self):
        self.cmds = []

    def add(self, command, **params):
        self.cmds.append({"command": command, "params": params})

    def obox(self, name, L, origin, e1, e2, e3, r1, r2, r3):
        """Box spanning r1/r2/r3 (lo, hi) along local axes e1/e2/e3 about `origin` (blade-1 frame),
        then placed for blade k by the linear map L (rotation, or rotation x mirror)."""
        E = L @ np.column_stack([e1, e2, e3])
        o = L @ np.asarray(origin, float)
        if np.linalg.det(E) < 0:                                  # mirrored: keep the frame right-handed
            E[:, 2] *= -1
            r3 = (-r3[1], -r3[0])
        U, _, Vt = np.linalg.svd(E)
        E = U @ Vt                                                # exact rotation for solid.move
        rr = (r1, r2, r3)
        c = o + E @ np.array([(a + b) / 2 for a, b in rr])
        sz = [round(b - a, 5) for a, b in rr]
        self.add("solid.box", length=sz[0], width=sz[1], height=sz[2], center=[0, 0, 0], body_name=name)
        M = np.column_stack([E, c])
        self.add("solid.move", bodies=[name], matrix=[[float(v) for v in row] for row in M], tolerance=1e-9)

    def cyl(self, name, L, p0, p1, d):
        a, b = L @ np.asarray(p0, float), L @ np.asarray(p1, float)
        h = float(np.linalg.norm(b - a))
        self.add("solid.cylinder", diameter=round(d, 5), height=round(h, 5), base=_r(a),
                 axis=_r((b - a) / h, 9), body_name=name)

    def plate(self, name, pts_xy, z0, t):
        """Prism: CCW polygon on the plane z = z0, extruded +t along +z."""
        self.add("sketch.create", plane={"origin": [0, 0, round(z0, 5)], "x_dir": [1, 0, 0], "y_dir": [0, 1, 0]},
                 name=name + "_sk")
        self.add("sketch.line", points=[_r(p) for p in pts_xy], closed=True)
        self.add("sketch.finish")
        self.add("solid.extrude", sketch=name + "_sk", distance=round(t, 5), operation="new", body_name=name)

    def join(self, target, tools):
        for tl in tools:                       # one tool at a time: a failing union names its tool
            self.add("solid.combine", target=target, tools=[tl], operation="join")

    def cut(self, target, tools, keep=False):
        self.add("solid.combine", target=target, tools=list(tools), operation="cut", keep_tools=keep)


def _blade_map(k: int, n: int) -> np.ndarray:
    """Blade-1 frame -> blade k (mirror z for even blades, then rotate about z), as fittings._mirror_rot."""
    a = 2 * math.pi * (k - 1) / n
    R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
    return R @ np.diag([1, 1, -1.0]) if k % 2 == 0 else R


OVERRIDE_KEYS = {"n_lam": "laminae per stack", "t_lam": "lamina thickness, mm", "width": "strip width, mm",
                 "slot_clear": "slot thickness clearance over the stack, mm", "blade_extra": "blade slot depth past the land, mm",
                 "mouth_chamfer": "45 deg slot-mouth chamfer, mm", "boss_h": "boss height, mm", "boss_od": "boss OD, mm",
                 "bore": "bore, mm"}


def rotor_layout(design: dict | Design, overrides: dict | None = None) -> dict:
    """Blade-1 frame geometry (mm) shared by the script and the checks. `overrides` (OVERRIDE_KEYS)
    change the strip stack / slots / boss only; free length, skew, cross angle and hinge radius stay."""
    d = design.model_dump() if isinstance(design, Design) else copy.deepcopy(design)
    ov = dict(overrides or {})
    bad = set(ov) - set(OVERRIDE_KEYS)
    if bad:
        raise KeyError(f"unknown overrides {sorted(bad)}; known {sorted(OVERRIDE_KEYS)}")
    fx = d["flexure"]
    if "n_lam" in ov:
        fx["strip_laminae"] = int(ov["n_lam"])
    if "t_lam" in ov:
        fx["thickness"] = float(ov["t_lam"]) * 1e-3
    if "width" in ov:
        fx["width"] = float(ov["width"]) * 1e-3
    if "slot_clear" in ov:
        fx["strip_slot_clear"] = float(ov["slot_clear"])
    if float(d["flexure"].get("strip_slot_clear", -1.0)) < 0:
        d["flexure"]["strip_slot_clear"] = SLOT_CLEAR
    D = Design.model_validate(d)
    prm = FT.strip_params(D)
    strips, wires, hinge = FT.strip_members(D)
    fr = []
    for m in strips:
        p0, p1, L, t, w, nrm = FT._frame(m)
        fr.append(dict(p0=p0, p1=p1, L=L, t=t, w=w, n=nrm, c=(p0 + p1) / 2))
    for i, f in enumerate(fr):                    # crossing edge: the side facing the other strip
        o = fr[1 - i] if len(fr) == 2 else fr[i]
        f["sc"] = float(np.sign((o["c"] - f["c"]) @ f["w"])) or 1.0
    xw = sorted(float(v) for v in (np.asarray(wires[0].p0) * 1e3)[:1].tolist() + (np.asarray(wires[0].p1) * 1e3)[:1].tolist()) \
        if wires else [hinge[0][0] * 1e3 - 1.6, hinge[0][0] * 1e3 + 1.6]
    hw = prm.well_d / 2 + prm.wall                # spine / shank half-width round the knot well
    x_hw = BOSS_OD / 2 + prm.well_d / 2           # hub knot well, tangent to the boss
    x_bw = xw[1] + CH_LEN                         # blade knot well, CH_LEN out along the channel
    r_aero = x_bw + prm.well_d / 2 + prm.wall     # aero loft starts outboard of the blade well
    return dict(D=D, prm=prm, strips=fr, hinge=np.asarray(hinge[0]) * 1e3, hinge_axis=np.asarray(hinge[1]),
                x_hub_end=xw[0], x_blade_start=xw[1], hw=hw, x_hw=x_hw, x_bw=x_bw, r_aero=r_aero,
                ch_h=prm.channel_d / 2 + prm.wall, n=D.blade.n_blades, R=D.blade.diameter_in * 25.4 / 2,
                boss_h=float(ov.get("boss_h", BOSS_H)), boss_od=float(ov.get("boss_od", BOSS_OD)),
                bore=float(ov.get("bore", BORE_D)), blade_extra=float(ov.get("blade_extra", OVERDEEP)),
                chamfer=float(ov.get("mouth_chamfer", 0.0)), overrides=ov)


def _jaw_ranges(f, prm, side, extra=OVERDEEP):
    """(t, u, v) ranges about p0 of the jaw block and the slot cutter, hub (side 0) or blade (side 1)."""
    sc, L = f["sc"], f["L"]
    u_jaw = tuple(sorted((sc * (prm.w / 2 - OPEN_LIP), -sc * (prm.slot_w / 2 + prm.wall))))
    u_slot = tuple(sorted((sc * (prm.w / 2 + 2.0), -sc * prm.slot_w / 2)))     # open on the crossing edge
    v_jaw = (-(prm.slot_t / 2 + prm.wall), prm.slot_t / 2 + prm.wall)
    v_slot = (-prm.slot_t / 2, prm.slot_t / 2)
    if side == 0:      # hub: slot open through the back (stacks go in from the hub side)
        return ((-prm.land, 0.0), u_jaw, v_jaw), ((-prm.land - 2.0, 0.5), u_slot, v_slot)
    deep = prm.land + extra
    return ((L, L + deep + prm.wall), u_jaw, v_jaw), ((L - 0.5, L + deep), u_slot, v_slot)


def _box_corners_xy(f, rr):
    pts = []
    for a in rr[0]:
        for b in rr[1]:
            for c in rr[2]:
                p = f["p0"] + a * f["t"] + b * f["w"] + c * f["n"]
                pts.append(p[:2])
    return pts


def blade_loft_commands(S: _Script, lay: dict, k: int, name: str, n_sec: int = 5):
    """Aero blade k as a loft of airfoil sections from lay['r_aero'] to the tip (swap this for Keel's
    propeller script later). Quarter chord on the pitch axis (z = 0), chord / twist from the design
    stations, leading edge toward the rotation (+y for blade 1)."""
    D, R, n = lay["D"], lay["R"], lay["n"]
    st = sorted(D.blade.stations, key=lambda s: s.r_frac)
    rf = np.array([s.r_frac * R for s in st])
    ch = np.array([s.chord * 1e3 for s in st])
    tw = np.array([s.twist_deg for s in st])
    af = D.blade.airfoil
    code = af.naca if getattr(af, "kind", "") == "naca4" and getattr(af, "naca", None) else "4412"
    txt = "NACA" + code + "\n" + "\n".join(f"{x:.6f} {y:.6f}" for x, y in naca4(code)) + "\n"
    a = 2 * math.pi * (k - 1) / n
    er = np.array([math.cos(a), math.sin(a), 0.0])
    xd = -np.cross([0, 0, 1.0], er)                      # chord direction LE -> TE
    secs = []
    for j, r in enumerate(np.linspace(lay["r_aero"], R, n_sec)):
        sk = f"{name}_af{j}"
        S.add("sketch.create", plane={"origin": _r(r * er), "x_dir": _r(xd, 9), "y_dir": [0, 0, 1]}, name=sk)
        S.add("sketch.insert_airfoil", text=txt, chord=round(float(np.interp(r, rf, ch)), 4), pivot=0.25,
              at=[0, 0], angle=round(-float(np.interp(r, rf, tw)), 4))
        S.add("sketch.finish")
        secs.append({"sketch": sk})
    S.add("solid.loft", sections=secs, body_name=name)


def _mouth(S: _Script, name, L, f, t0, sgn, prm, c):
    """45 deg slot-mouth chamfer cutter: frustum lofted from the exit plane (slot + 2c, started 0.2 mm
    outside it) to depth c into the jaw (slot size). sgn: +1 into the blade jaw, -1 into the hub jaw."""
    secs = []
    for j, (dt, grow) in enumerate(((-0.2 * sgn, c + 0.2), (c * sgn, 0.0))):
        o = L @ (f["p0"] + (t0 + dt) * f["t"])
        sk = f"{name}_s{j}"
        S.add("sketch.create", plane={"origin": _r(o), "x_dir": _r(L @ f["w"], 9), "y_dir": _r(L @ f["n"], 9)}, name=sk)
        S.add("sketch.rectangle.center", center=[0, 0], corner=[round(prm.slot_w / 2 + grow, 5), round(prm.slot_t / 2 + grow, 5)])
        S.add("sketch.finish")
        secs.append({"sketch": sk})
    S.add("solid.loft", sections=secs, body_name=name)


def rotor_commands(design: dict, overrides: dict | None = None, fillet: bool = True, n_sec: int = 5) -> dict:
    """SolidCraft script for the whole rotor (see module doc). Pure modelling commands only.
    overrides: see OVERRIDE_KEYS (e.g. the Ender/PETG spin-test variant in study/cad/build_5in_ender_tonight.py)."""
    lay = rotor_layout(design, overrides)
    prm, fr, n, hw = lay["prm"], lay["strips"], lay["n"], lay["hw"]
    boss_h, BOSS_OD, BORE_D, ex_d, cham = lay["boss_h"], lay["boss_od"], lay["bore"], lay["blade_extra"], lay["chamfer"]
    zb = BOSS_H / 2 - ARM_T                            # arm plates: hub z in [-6, -4], blade z in [4, 6]
    ex, ey, ez = np.eye(3)
    S = _Script()
    # boss bottom stays on the lead's flat face (z = -BOSS_H/2); arms / spines keep the lead's z (boss_h only shortens the boss)
    S.add("solid.cylinder", diameter=BOSS_OD, height=boss_h, base=[0, 0, -BOSS_H / 2], body_name="hub")
    r_in = BOSS_OD / 2 - prm.wall / 2         # arm outline from inside the boss: no tangent contact
    circ = [(r_in * math.cos(q), r_in * math.sin(q)) for q in np.linspace(0, 2 * math.pi, 36, endpoint=False)]
    x_be = lay["r_aero"] + 1.0                          # shank runs 1 mm into the aero loft
    fil = []
    for k in range(1, n + 1):
        L = _blade_map(k, n)
        mz = np.sign(np.linalg.det(L))                  # z sign of this blade's fitting side
        # ---- hub side of blade k
        hub_parts = [f"hub_spine{k}"]
        S.obox(f"hub_spine{k}", L, [0, 0, 0], ex, ey, ez, (0.0, lay["x_hub_end"]), (-hw, hw),
               (-(BOSS_H / 2 - ARM_T / 2), lay["ch_h"]))
        for i, f in enumerate(fr, 1):
            jr, _ = _jaw_ranges(f, prm, 0)
            S.obox(f"hub_jaw{k}{i}", L, f["p0"], f["t"], f["w"], f["n"], *jr)
            poly = _hull2([(L @ [x, y, 0])[:2] for x, y in circ + _box_corners_xy(f, jr)])
            S.plate(f"hub_arm{k}{i}", poly, -BOSS_H / 2 if mz > 0 else zb, ARM_T)
            hub_parts += [f"hub_jaw{k}{i}", f"hub_arm{k}{i}"]
            yc = float(np.mean([p[1] for p in _box_corners_xy(f, jr)]))
            s = math.copysign(1.0, yc)
            q = 0.5 * (math.asin(min(0.95, (hw + 0.3) / (BOSS_OD / 2))) + math.radians(75))   # mid-arc, clear of spine + arm edges
            fil.append(L @ np.array([BOSS_OD / 2 * math.cos(q), s * BOSS_OD / 2 * math.sin(q), -zb]))
        S.join("hub", hub_parts)
        # ---- blade root fitting k
        S.obox(f"blade{k}", L, [0, 0, 0], ex, ey, ez, (lay["x_blade_start"], x_be), (-hw, hw),
               (-lay["ch_h"], BOSS_H / 2 - ARM_T / 2))
        bl_parts = []
        for i, f in enumerate(fr, 1):
            jr, _ = _jaw_ranges(f, prm, 1, ex_d)
            S.obox(f"blade_jaw{k}{i}", L, f["p0"], f["t"], f["w"], f["n"], *jr)
            sh = [(x, y) for x in (lay["x_blade_start"], lay["r_aero"]) for y in (-hw, hw)]   # plate stops at the loft root
            poly = _hull2([(L @ [x, y, 0])[:2] for x, y in sh + _box_corners_xy(f, jr)])
            S.plate(f"blade_arm{k}{i}", poly, zb if mz > 0 else -BOSS_H / 2, ARM_T)
            bl_parts += [f"blade_jaw{k}{i}", f"blade_arm{k}{i}"]
        S.join(f"blade{k}", bl_parts)
    fil = [p for p in fil if -BOSS_H / 2 < p[2] < -BOSS_H / 2 + boss_h]     # arms that meet the boss
    if fillet and fil:  # arm roots: arm plate face meeting the boss cylinder
        S.add("solid.fillet", body="hub", edges=[_r(p) for p in fil],
              radius=round(min(prm.wall, 0.25 * (boss_h - ARM_T)), 4))   # short boss: band above the arm limits R
    S.cyl("bore", np.eye(3), [0, 0, -BOSS_H], [0, 0, BOSS_H], BORE_D)
    S.cut("hub", ["bore"])
    if boss_h < BOSS_H:  # short boss: keep the washer / nut seat on its top face clear of the upper arms
        S.cyl("seat_clear", np.eye(3), [0, 0, -BOSS_H / 2 + boss_h], [0, 0, BOSS_H + 10], BOSS_OD + 1.0)
        S.cut("hub", ["seat_clear"])
    for k in range(1, n + 1):
        L = _blade_map(k, n)
        ko, hs, bs = [], [], []
        for i, f in enumerate(fr, 1):
            _, hr = _jaw_ranges(f, prm, 0)
            _, br = _jaw_ranges(f, prm, 1, ex_d)
            S.obox(f"hub_slot{k}{i}", L, f["p0"], f["t"], f["w"], f["n"], *hr)
            S.obox(f"blade_slot{k}{i}", L, f["p0"], f["t"], f["w"], f["n"], *br)
            S.obox(f"keepout{k}{i}", L, f["p0"], f["t"], f["w"], f["n"], (0.0, f["L"]),
                   (-prm.w / 2 - STRIP_CLEAR, prm.w / 2 + STRIP_CLEAR), (-prm.t / 2 - STRIP_CLEAR, prm.t / 2 + STRIP_CLEAR))
            ko.append(f"keepout{k}{i}"); hs.append(f"hub_slot{k}{i}"); bs.append(f"blade_slot{k}{i}")
            if cham > 0:
                _mouth(S, f"hub_mouth{k}{i}", L, f, 0.0, -1, prm, cham)
                _mouth(S, f"blade_mouth{k}{i}", L, f, f["L"], 1, prm, cham)
                hs.append(f"hub_mouth{k}{i}"); bs.append(f"blade_mouth{k}{i}")
        # strap channel + knot wells (wells open through the arm side for knotting)
        zt = BOSS_H / 2 + 1
        S.cyl(f"hub_well{k}", L, [lay["x_hw"], 0, -zt], [lay["x_hw"], 0, lay["ch_h"] + 1], prm.well_d)
        S.cyl(f"hub_chan{k}", L, [lay["x_hw"], 0, 0], [lay["x_hub_end"] + 1, 0, 0], prm.channel_d)
        S.cyl(f"blade_well{k}", L, [lay["x_bw"], 0, -lay["ch_h"] - 1], [lay["x_bw"], 0, zt], prm.well_d)
        S.cyl(f"blade_chan{k}", L, [lay["x_blade_start"] - 1, 0, 0], [lay["x_bw"], 0, 0], prm.channel_d)
        S.cut("hub", hs + [f"hub_well{k}", f"hub_chan{k}"])
        S.cut("hub", ko, keep=True)
        S.cut(f"blade{k}", bs + ko + [f"blade_well{k}", f"blade_chan{k}"])
        blade_loft_commands(S, lay, k, f"aero{k}", n_sec)
        S.join(f"blade{k}", [f"aero{k}"])
        for i, f in enumerate(fr, 1):
            S.obox(f"strip_b{k}_{i}", L, f["p0"], f["t"], f["w"], f["n"], (-prm.land, f["L"] + prm.land),
                   (-prm.w / 2, prm.w / 2), (-prm.t / 2, prm.t / 2))
        S.cyl(f"strap_b{k}", L, [lay["x_hw"], 0, 0], [lay["x_bw"], 0, 0], prm.strand_d)
    return {"commands": S.cmds}


def part_names(design: dict) -> list[str]:
    n = int(design["blade"]["n_blades"])
    return (["hub"] + [f"blade{k}" for k in range(1, n + 1)]
            + [f"strip_b{k}_{i}" for k in range(1, n + 1) for i in (1, 2)]
            + [f"strap_b{k}" for k in range(1, n + 1)])
