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
  strap_b{k}     aramid strand (centrifugal tie) from the hub knot well to the blade knot well; it clips the
                 crossing stacks (see _strap_route). overrides strap_route="pair_z": strap_b{k}_1 / _2 at z = +-dz

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
                 "bore": "bore, mm",
                 "flat_base": "support-free flat motor face (bool, default off): see _flat_base_doc",
                 "supports": "designed breakaway supports (bool, default off; needs meshes=, see support_commands)",
                 "strap_route": "'axis' (default: one strand on the pitch axis) | 'pair_z': see _strap_route"}
STRAP_CLEAR = 0.15              # strap_route pair_z: min strand-surface to strip-stack clearance, mm
NOTCH_X, NOTCH_Y = 2.6, 1.0     # pair_z: blade arm-plate notch past the hub rib (x: 1 mm + the 15 deg flap sweep), mm
RIB_THIN, NOTCH_JAW = 0.5, 3.0  # pair_z: rib wall on the -y (blade-jaw) side and over the channel; notch -y edge (jaw at -3.04), mm


def _stack_dist(fr, prm, y0, z0, x0, x1, step=0.01):
    """Min distance (mm) from the x-parallel line (y0, z0), x in [x0, x1], to the strip stacks (lands included)."""
    xs = np.arange(x0, x1 + 1e-9, step)
    P = np.stack([xs, np.full_like(xs, y0), np.full_like(xs, z0)], 1)
    out = []
    for f in fr:
        d = P - f["p0"]
        a, b, c = d @ f["t"], d @ f["w"], d @ f["n"]
        da = np.maximum(0, np.maximum(-prm.land - a, a - f["L"] - prm.land))
        out.append(float(np.sqrt(da ** 2 + np.maximum(0, abs(b) - prm.w / 2) ** 2
                                 + np.maximum(0, abs(c) - prm.t / 2) ** 2).min()))
    return min(out)


def _strap_route(fr, prm, ov, x_hw, x_bw, ch_h, x_bs):
    """Strand route(s) of the tension-torsion strap, blade-1 frame. 'axis': the analysis strap, one strand of
    strand_d on the pitch axis (through the hinge point). It clips the crossing stacks: the stacks sit side by
    side along the hinge axis (28 deg off the pitch axis) with only cross_gap between their crossing edges, so
    the clear radius round the pitch axis is ~0.06 mm (lead) and no load-carrying strand fits there.
    'pair_z': two area-equivalent strands (strand_d / sqrt 2) at z = +-dz, y = 0, dz the smallest 0.05 mm step
    that clears both stacks by STRAP_CLEAR over the whole knot-to-knot run. An offset along z (the pitch-axis
    normal in the plane of pitch and hinge axes) is the only one with no first-order stretch / moment about the
    hinge axis (w . (e x x) = 0); a y offset stretches the strand at 0.47 dy per rad of hinge rotation. The
    pair adds trapeze pitch stiffness ~ T (0.88 dz)^2 / L_free (= flexure.strap_strands 2, strap_radius dz in
    the beam model). The +dz strand leaves the hub through a narrow rib on the spine (channel + wall), runs in the
    blade shank; the -dz strand runs in the hub spine, passes under the shank and knots in a short lug at the
    blade well (kept outboard of the hub arm plates so +-15 deg swing stays clear)."""
    route = ov.get("strap_route", "axis") or "axis"
    if route == "axis":
        return dict(route="axis", d=prm.strand_d, ch_d=prm.channel_d, zs=(0.0,), dz=0.0, top=ch_h,
                    bx0=(x_bs,))
    if route != "pair_z":
        raise ValueError(f"strap_route {route!r}: 'axis' | 'pair_z'")
    d = prm.strand_d / math.sqrt(2.0)
    ch_d = d + FT.CH_CLEAR
    dz = round(math.ceil((ch_d / 2) / 0.05) * 0.05, 4)
    while min(_stack_dist(fr, prm, 0.0, s * dz, x_hw, x_bw) for s in (1, -1)) < d / 2 + STRAP_CLEAR:
        dz = round(dz + 0.05, 4)
    lug_x0 = x_bw - CH_LEN / 2
    return dict(route="pair_z", d=d, ch_d=ch_d, zs=(dz, -dz), dz=dz, top=max(ch_h, dz + ch_d / 2 + RIB_THIN),
                rib_y=(-(ch_d / 2 + RIB_THIN), ch_d / 2 + prm.wall), rib_hw=ch_d / 2 + prm.wall, lug_x0=lug_x0, bx0=(x_bs, lug_x0))
FB_GAP = 2.0                    # flat_base: min xy gap from an upper hub jaw's shadow to its blade fitting, mm
FB_SLOT_EXT = 8.0               # flat_base: hub slot cutter run-out past the jaw back, so it exits the pedestal


def _flat_base_doc():
    """flat_base=True (CAD only; the beam model is unchanged). With the skewed crossed strips the hub jaws of
    blade 1 hang ~4.9 mm below the lead's boss face, so: (1) the boss bottom (motor face) moves down to the lowest
    hub-jaw point, or FB_BELL_GAP below the mirrored blade's lower jaw where it passes over the bell (z_base,
    -12.55 for the lead): the whole rotor sits that much higher on the shaft, a rigid translation the analysis
    frame does not see; (2) every hub jaw whose vertical shadow is clear of its own blade fitting becomes
    a pedestal = the jaw extended straight down to z_base (jaw box cut by its upward face planes, over its xy
    hull), so its jaw faces (all <= 44.4 deg from vertical at skew 62 / cross 105) and slot / keep-out cuts
    (strip-aligned) print without supports; the hub-down blade's spine and arm plates become prisms from z_base;
    (3) hub slots run out through the pedestal (stack still loads from the hub back); knot wells open at z_base.
    The upper (mirrored, even-blade) hub jaw whose shadow falls on its own blade fitting cannot be propped
    without entering the blade's swing envelope; it keeps the lead's arm plate (reported by the overhang check)."""
    return _flat_base_doc.__doc__


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
                chamfer=float(ov.get("mouth_chamfer", 0.0)), flat_base=bool(ov.get("flat_base", False)),
                z_base=_z_base(fr, prm, D.blade.n_blades, float(ov.get("blade_extra", OVERDEEP))), overrides=ov,
                strap=_strap_route(fr, prm, ov, x_hw, x_bw, prm.channel_d / 2 + prm.wall, xw[1]))


FB_BELL_R, FB_BELL_GAP = 14.5, 1.0   # flat_base: 2307 bell radius; blade-fitting clearance above the motor face there, mm


def _z_base(fr, prm, n, extra=OVERDEEP):
    """flat_base motor face (mm, 0.05 grid): the lowest hub-jaw point over all blades, and at least FB_BELL_GAP
    below any blade-jaw point inside the bell radius (the mirrored blade's lower jaw passes over the bell)."""
    z = min(float((_blade_map(k, n) @ np.asarray(p))[2]) for k in range(1, n + 1) for f in fr
            for p in _box_corners(f, _jaw_ranges(f, prm, 0)[0]))
    g = np.linspace(0, 1, 13)
    for k in range(1, n + 1):
        L = _blade_map(k, n)
        for f in fr:
            rr = _jaw_ranges(f, prm, 1, extra)[0]
            P = np.array([L @ (f["p0"] + a * f["t"] + b * f["w"] + c * f["n"]) for a in np.interp(g, [0, 1], rr[0])
                          for b in np.interp(g, [0, 1], rr[1]) for c in np.interp(g, [0, 1], rr[2])])
            P = P[np.hypot(P[:, 0], P[:, 1]) < FB_BELL_R]
            if len(P):
                z = min(z, float(P[:, 2].min()) - FB_BELL_GAP)
    return round(math.floor(z / 0.05) * 0.05, 4)


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


def _box_corners(f, rr):
    return [f["p0"] + a * f["t"] + b * f["w"] + c * f["n"] for a in rr[0] for b in rr[1] for c in rr[2]]


def _poly_gap(P, Q):
    """Min distance between two convex CCW polygons (0 if they overlap; separating-axis test)."""
    P, Q = np.asarray(P, float), np.asarray(Q, float)
    for A, B in ((P, Q), (Q, P)):
        for i in range(len(A)):
            e = A[(i + 1) % len(A)] - A[i]
            nrm = np.array([e[1], -e[0]]) / np.linalg.norm(e)          # outward for CCW
            if ((B - A[i]) @ nrm).min() > 0:
                break
        else:
            continue
        break
    else:
        return 0.0

    def seg(p, a, b):
        ab = b - a
        u = np.clip((p - a) @ ab / (ab @ ab), 0, 1)
        return float(np.linalg.norm(p - a - u * ab))
    return min(seg(p, A[i], A[(i + 1) % len(A)]) for A, B in ((P, Q), (Q, P)) for p in B for i in range(len(A)))


def _pedestal(S: "_Script", name, L, f, rr, z_base):
    """Jaw box (ranges rr about p0 in the strip frame, blade-1 frame, placed by L) extended straight down to
    z = z_base: prism over the xy hull of the placed corners, cut by the box's upward-facing face planes."""
    P = [L @ p for p in _box_corners(f, rr)]
    poly = _hull2([p[:2] for p in P])
    z_hi = max(p[2] for p in P) + 1.0
    S.plate(name, poly, z_base, z_hi - z_base)
    big, cut = 60.0, []
    axes = (f["t"], f["w"], f["n"])
    for j, e in enumerate(axes):
        up = float((L @ e)[2])
        if abs(up) < 1e-9:
            continue
        lo, hi = rr[j]
        r = [(-big, big)] * 3
        r[j] = (hi, hi + big) if up > 0 else (lo - big, lo)
        S.obox(f"{name}_c{j}", L, f["p0"], *axes, *r)
        cut.append(f"{name}_c{j}")
    if float((L @ f["t"])[2]) < 0:      # upper jaw: nothing past the mouth plane (free span stays free; 32.5 deg face)
        S.obox(f"{name}_cm", L, f["p0"], *axes, (rr[0][1], rr[0][1] + big), (-big, big), (-big, big))
        cut.append(f"{name}_cm")
    S.cut(name, cut)


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


def rotor_commands(design: dict, overrides: dict | None = None, fillet: bool = True, n_sec: int = 5,
                   meshes: dict | None = None) -> dict:
    """SolidCraft script for the whole rotor (see module doc). Pure modelling commands only.
    overrides: see OVERRIDE_KEYS (e.g. the Ender/PETG spin-test variant in study/cad/build_5in_ender_tonight.py)."""
    lay = rotor_layout(design, overrides)
    prm, fr, n, hw, st = lay["prm"], lay["strips"], lay["n"], lay["hw"], lay["strap"]
    boss_h, BOSS_OD, BORE_D, ex_d, cham = lay["boss_h"], lay["boss_od"], lay["bore"], lay["blade_extra"], lay["chamfer"]
    zb = BOSS_H / 2 - ARM_T                            # arm plates: hub z in [-6, -4], blade z in [4, 6]
    fb, zB = lay["flat_base"], lay["z_base"]
    z_boss0 = zB if fb else -BOSS_H / 2                 # flat_base: motor face at the lowest hub-jaw point
    ex, ey, ez = np.eye(3)
    S = _Script()
    # boss bottom stays on the lead's flat face (z = -BOSS_H/2); arms / spines keep the lead's z (boss_h only shortens the boss)
    S.add("solid.cylinder", diameter=BOSS_OD, height=round(-BOSS_H / 2 + boss_h - z_boss0, 5), base=[0, 0, z_boss0],
          body_name="hub")
    r_in = BOSS_OD / 2 - prm.wall / 2         # arm outline from inside the boss: no tangent contact
    circ = [(r_in * math.cos(q), r_in * math.sin(q)) for q in np.linspace(0, 2 * math.pi, 36, endpoint=False)]
    x_be = lay["r_aero"] + 1.0                          # shank runs 1 mm into the aero loft
    fil = []
    for k in range(1, n + 1):
        L = _blade_map(k, n)
        mz = np.sign(np.linalg.det(L))                  # z sign of this blade's fitting side
        # ---- hub side of blade k
        hub_parts = [f"hub_spine{k}"]
        down = fb and mz > 0                            # flat_base, hub side hanging down: everything on pedestals
        S.obox(f"hub_spine{k}", L, [0, 0, 0], ex, ey, ez, (0.0, lay["x_hub_end"]), (-hw, hw),
               (zB if down else -(BOSS_H / 2 - ARM_T / 2), lay["ch_h"]))
        if st["route"] == "pair_z":                     # rib walling the outer (+dz) strand from the knot well
            S.obox(f"hub_rib{k}", L, [0, 0, 0], ex, ey, ez, (lay["x_hw"], lay["x_hub_end"]), st["rib_y"],
                   (lay["ch_h"] - 0.5, st["top"]))
            hub_parts.append(f"hub_rib{k}")
        if fb and not down:                             # blade k fitting footprint (it lies under the upper hub jaws)
            bfp = _hull2([(L @ [x, y, 0])[:2] for x in (lay["x_blade_start"], x_be) for y in (-hw, hw)]
                         + [(L @ p)[:2] for f in fr for p in _box_corners(f, _jaw_ranges(f, prm, 1, ex_d)[0])])
        for i, f in enumerate(fr, 1):
            jr, _ = _jaw_ranges(f, prm, 0)
            poly = _hull2([(L @ [x, y, 0])[:2] for x, y in circ + _box_corners_xy(f, jr)])
            ped = down or (fb and _poly_gap(_hull2([(L @ p)[:2] for p in _box_corners(f, jr)]), bfp) >= FB_GAP)
            if ped:
                _pedestal(S, f"hub_jaw{k}{i}", L, f, jr, zB)
                hub_parts.append(f"hub_jaw{k}{i}")
                if not down:                            # upper jaw on its own pedestal (meets the boss): no arm plate
                    continue
                S.plate(f"hub_arm{k}{i}", poly, zB, -BOSS_H / 2 + ARM_T - zB)
            else:
                S.obox(f"hub_jaw{k}{i}", L, f["p0"], f["t"], f["w"], f["n"], *jr)
                S.plate(f"hub_arm{k}{i}", poly, -BOSS_H / 2 if mz > 0 else zb, ARM_T)
            hub_parts += [f"hub_jaw{k}{i}", f"hub_arm{k}{i}"] if not ped else [f"hub_arm{k}{i}"]
            yc = float(np.mean([p[1] for p in _box_corners_xy(f, jr)]))
            s = math.copysign(1.0, yc)
            q = 0.5 * (math.asin(min(0.95, (hw + 0.3) / (BOSS_OD / 2))) + math.radians(75))   # mid-arc, clear of spine + arm edges
            if not down:                                # flat_base pedestal arms are solid to the face: no root fillet
                fil.append(L @ np.array([BOSS_OD / 2 * math.cos(q), s * BOSS_OD / 2 * math.sin(q), -zb]))
        S.join("hub", hub_parts)
        # ---- blade root fitting k
        S.obox(f"blade{k}", L, [0, 0, 0], ex, ey, ez, (lay["x_blade_start"], x_be), (-hw, hw),
               (-lay["ch_h"], BOSS_H / 2 - ARM_T / 2))
        bl_parts = []
        if st["route"] == "pair_z":                     # lug holding the outer (-dz) strand's knot, clear of the hub arms
            S.obox(f"blade_lug{k}", L, [0, 0, 0], ex, ey, ez, (st["lug_x0"], lay["x_bw"]), (-st["rib_hw"], st["rib_hw"]),
                   (-st["top"], -lay["ch_h"] + 0.5))
            bl_parts.append(f"blade_lug{k}")
        for i, f in enumerate(fr, 1):
            jr, _ = _jaw_ranges(f, prm, 1, ex_d)
            S.obox(f"blade_jaw{k}{i}", L, f["p0"], f["t"], f["w"], f["n"], *jr)
            sh = [(x, y) for x in (lay["x_blade_start"], lay["r_aero"]) for y in (-hw, hw)]   # plate stops at the loft root
            poly = _hull2([(L @ [x, y, 0])[:2] for x, y in sh + _box_corners_xy(f, jr)])
            S.plate(f"blade_arm{k}{i}", poly, zb if mz > 0 else -BOSS_H / 2, ARM_T)
            bl_parts += [f"blade_jaw{k}{i}", f"blade_arm{k}{i}"]
        S.join(f"blade{k}", bl_parts)
        if st["route"] == "pair_z":                     # arm-plate notch over the hub rib (rest + 15 deg swing clearance)
            S.obox(f"blade_notch{k}", L, [0, 0, 0], ex, ey, ez, (lay["x_hw"] - 5.0, lay["x_hub_end"] + NOTCH_X),
                   (-NOTCH_JAW, st["rib_y"][1] + NOTCH_Y), (lay["ch_h"], BOSS_H / 2 + 1.0))
            S.cut(f"blade{k}", [f"blade_notch{k}"])
    fil = [p for p in fil if z_boss0 < p[2] < -BOSS_H / 2 + boss_h]     # arms that meet the boss
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
            if fb:                                      # run the hub slot out through the pedestal
                hr = ((hr[0][0] - FB_SLOT_EXT, hr[0][1]),) + tuple(hr[1:])
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
        z_w0 = zB - 1 if (fb and mz > 0) else -zt       # flat_base: the hub well opens through the motor face
        S.cyl(f"hub_well{k}", L, [lay["x_hw"], 0, z_w0], [lay["x_hw"], 0, st["top"] + 1], prm.well_d)
        sfx = [""] if len(st["zs"]) == 1 else [f"_{j}" for j in range(1, len(st["zs"]) + 1)]
        for j, z in zip(sfx, st["zs"]):
            S.cyl(f"hub_chan{k}{j}", L, [lay["x_hw"], 0, z], [lay["x_hub_end"] + 1, 0, z], st["ch_d"])
        S.cyl(f"blade_well{k}", L, [lay["x_bw"], 0, -st["top"] - 1], [lay["x_bw"], 0, zt], prm.well_d)
        for j, z, x0 in zip(sfx, st["zs"], st["bx0"]):
            S.cyl(f"blade_chan{k}{j}", L, [x0 - 1, 0, z], [lay["x_bw"], 0, z], st["ch_d"])
        S.cut("hub", hs + [f"hub_well{k}"] + [f"hub_chan{k}{j}" for j in sfx])
        S.cut("hub", ko, keep=True)
        S.cut(f"blade{k}", bs + ko + [f"blade_well{k}"] + [f"blade_chan{k}{j}" for j in sfx])
        blade_loft_commands(S, lay, k, f"aero{k}", n_sec)
        S.join(f"blade{k}", [f"aero{k}"])
        for i, f in enumerate(fr, 1):
            S.obox(f"strip_b{k}_{i}", L, f["p0"], f["t"], f["w"], f["n"], (-prm.land, f["L"] + prm.land),
                   (-prm.w / 2, prm.w / 2), (-prm.t / 2, prm.t / 2))
        for j, z in zip(sfx, st["zs"]):
            S.cyl(f"strap_b{k}{j}", L, [lay["x_hw"], 0, z], [lay["x_bw"], 0, z], st["d"])
    if lay["overrides"].get("supports") and meshes:     # print-frame parts + breakaway supports (second pass)
        for part, T in meshes.items():
            R, dz = print_frame(T)
            S.add("solid.move", bodies=[part], matrix=[list(map(float, R[0])) + [0.0], list(map(float, R[1])) + [0.0],
                                                        list(map(float, R[2])) + [float(dz)]], tolerance=1e-9)
            Tp = T @ R.T + [0, 0, dz]
            ex = [(R @ o + [0, 0, dz], R @ E, rr) for o, E, rr in slot_boxes(lay, part)]
            S.cmds += support_commands(Tp, f"{part}_sup", ex)[0]
    return {"commands": S.cmds}


# ------------------------------------------------------------------ designed breakaway supports
SUP_WALL, SUP_PITCH, SUP_ZGAP, SUP_ANG, SUP_CLEAR, SUP_STEP = 0.8, 3.5, 0.2, 40.0, 1.0, 0.25


def slot_boxes(lay: dict, part: str):
    """Strip slot cutters (slot + glue land + mouth chamfer) of one printed part, rotor frame:
    [(origin, axes 3x3 columns t/w/n, ranges)], for the support keep-out."""
    prm, out = lay["prm"], []
    ks = range(1, lay["n"] + 1) if part == "hub" else [int(part[5:])]
    for k in ks:
        L = _blade_map(k, lay["n"])
        for f in lay["strips"]:
            _, rr = _jaw_ranges(f, prm, 0 if part == "hub" else 1, lay["blade_extra"])
            c = lay["chamfer"]
            rr = (rr[0], (rr[1][0] - c, rr[1][1] + c), (rr[2][0] - c, rr[2][1] + c))
            out.append((L @ f["p0"], L @ np.column_stack([f["t"], f["w"], f["n"]]), rr))
    return out


def _facet_nz(T):
    N = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    return N[:, 2] / np.maximum(np.linalg.norm(N, axis=1), 1e-30)


def ray_first_hit(T, Q):
    """Lowest surface z of the mesh T (n,3,3) on vertical lines through Q (m,2), and that facet's normal z
    (inf / 0 where the line misses)."""
    nz = _facet_nz(T)
    lo, hi = T[:, :, :2].min(1), T[:, :, :2].max(1)
    zq, nq = np.full(len(Q), np.inf), np.zeros(len(Q))
    for s0 in range(0, len(Q), 128):
        q = Q[s0:s0 + 128]
        ii, jj = np.nonzero((q[:, None, 0] >= lo[None, :, 0]) & (q[:, None, 0] <= hi[None, :, 0])
                            & (q[:, None, 1] >= lo[None, :, 1]) & (q[:, None, 1] <= hi[None, :, 1]))
        if not len(ii):
            continue
        a, b, c, p = T[jj, 0], T[jj, 1], T[jj, 2], q[ii]
        d = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
        ok = np.abs(d) > 1e-12
        dd = np.where(ok, d, 1.0)
        w1 = ((p[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (p[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])) / dd
        w2 = ((b[:, 0] - a[:, 0]) * (p[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (p[:, 0] - a[:, 0])) / dd
        w0 = 1 - w1 - w2
        z = np.where(ok & (w0 >= -1e-9) & (w1 >= -1e-9) & (w2 >= -1e-9), w0 * a[:, 2] + w1 * b[:, 2] + w2 * c[:, 2], np.inf)
        zm = np.full(len(q), np.inf)
        np.minimum.at(zm, ii, z)
        sel = np.isfinite(z) & (z <= zm[ii] + 1e-9)
        nm = np.zeros(len(q))
        nm[ii[sel]] = nz[jj[sel]]
        zq[s0:s0 + len(q)], nq[s0:s0 + len(q)] = zm, nm
    return zq, nq


def print_frame(T):
    """Print orientation of a part mesh (rotor frame): z up or flipped 180 deg about x, whichever leaves less
    >50 deg overhang off the plate; returns (R, dz) with the part's lowest point on z = 0."""
    best = None
    for R in (np.eye(3), np.diag([1.0, -1.0, -1.0])):
        Tp = T @ R.T
        z0 = Tp[:, :, 2].min()
        A = np.linalg.norm(np.cross(Tp[:, 1] - Tp[:, 0], Tp[:, 2] - Tp[:, 0]), axis=1) / 2
        bad = (_facet_nz(Tp) < -math.sin(math.radians(50))) & ~(np.abs(Tp[:, :, 2] - z0) < 0.05).all(axis=1)
        if best is None or A[bad].sum() < best[0] - 1.0:
            best = (A[bad].sum(), R, -z0)
    return best[1], best[2]


def support_commands(T, name, exclude=(), pitch=SUP_PITCH):
    """Breakaway support walls under a part mesh T (n,3,3; print frame, plate z = 0), one body `name`.
    A grid of SUP_WALL-thick vertical walls (pitch apart, both ways: cross-braced) rises from the plate to every
    exposed downward surface steeper than SUP_ANG from vertical, stopping SUP_ZGAP (one 0.2 mm layer) below it:
    the slicer bridges the gap and the walls snap off. Columns passing within SUP_CLEAR of an `exclude` box
    (origin, axes, ranges: slots, lands, mouths) are dropped. Returns (commands, columns (x, y, top))."""
    S, cols, thr = _Script(), [], -math.sin(math.radians(SUP_ANG))
    lo, hi = T[:, :, :2].reshape(-1, 2).min(0), T[:, :, :2].reshape(-1, 2).max(0)
    zz = np.arange(0.0, T[:, :, 2].max() + 0.25, 0.25)
    walls = []
    for ax in (0, 1):
        o = 1 - ax
        s = np.arange(lo[ax], hi[ax] + SUP_STEP, SUP_STEP)
        for cpos in np.arange(lo[o] + pitch / 2, hi[o], pitch):
            top, need = np.full(len(s), np.inf), np.zeros(len(s), bool)
            for dc in (-SUP_WALL / 2, 0.0, SUP_WALL / 2):
                Q = np.zeros((len(s), 2)); Q[:, ax] = s; Q[:, o] = cpos + dc
                z, nz = ray_first_hit(T, Q)
                top = np.minimum(top, z)
                need |= np.isfinite(z) & (nz < thr) & (z > SUP_ZGAP + 0.4)
            top = np.minimum(top, np.minimum(np.r_[top[1:], np.inf], np.r_[np.inf, top[:-1]])) - SUP_ZGAP
            need &= np.isfinite(top) & (top > 0.4)
            for org, E, rr in exclude:                       # keep-out round slots / lands / mouths
                idx = np.nonzero(need)[0]
                if not len(idx):
                    break
                P = np.zeros((len(idx), len(zz), 3)); P[:, :, ax] = s[idx, None]; P[:, :, o] = cpos; P[:, :, 2] = zz[None]
                loc = (P - org) @ E
                lo_, hi_ = np.array([r[0] for r in rr]), np.array([r[1] for r in rr])
                d = np.linalg.norm(np.maximum(np.maximum(lo_ - loc, loc - hi_), 0.0), axis=2)
                bad = ((d < SUP_CLEAR + SUP_WALL / 2) & (zz[None] <= top[idx, None])).any(axis=1)
                need[idx[bad]] = False
            j = 0
            while j < len(s):
                if not need[j]:
                    j += 1
                    continue
                e = j
                while e + 1 < len(s) and need[e + 1]:
                    e += 1
                if e > j:
                    walls.append((ax, cpos, s[j:e + 1], top[j:e + 1]))
                    cols += [(*(np.array([sv, cpos]) if ax == 0 else np.array([cpos, sv])), tv)
                             for sv, tv in zip(s[j:e + 1], top[j:e + 1])]
                j = e + 1
    names = []
    for i, (ax, cpos, sv, tv) in enumerate(walls):
        nm = name if i == 0 else f"{name}_w{i}"
        if ax == 0:      # along x at y = cpos: plane normal -y, extrude 0.8 from y = cpos + 0.4
            pl = {"origin": [0, round(float(cpos) + SUP_WALL / 2, 5), 0], "x_dir": [1, 0, 0], "y_dir": [0, 0, 1]}
        else:            # along y at x = cpos: plane normal +x
            pl = {"origin": [round(float(cpos) - SUP_WALL / 2, 5), 0, 0], "x_dir": [0, 1, 0], "y_dir": [0, 0, 1]}
        pts = [(sv[0], 0.0), (sv[-1], 0.0)] + list(zip(sv[::-1], tv[::-1]))
        S.add("sketch.create", plane=pl, name=nm + "_sk")
        S.add("sketch.line", points=[_r(q, 4) for q in pts], closed=True)
        S.add("sketch.finish")
        S.add("solid.extrude", sketch=nm + "_sk", distance=SUP_WALL, operation="new", body_name=nm)
        names.append(nm)
    if len(names) > 1:
        S.join(name, names[1:])
    return S.cmds, np.array(cols).reshape(-1, 3)



def part_names(design: dict, overrides: dict | None = None) -> list[str]:
    n = int(design["blade"]["n_blades"])
    pair = (overrides or {}).get("strap_route", "axis") == "pair_z"
    return (["hub"] + [f"blade{k}" for k in range(1, n + 1)]
            + [f"strip_b{k}_{i}" for k in range(1, n + 1) for i in (1, 2)]
            + [f"strap_b{k}{j}" for k in range(1, n + 1) for j in (("_1", "_2") if pair else ("",))])
