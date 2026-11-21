"""I30: steel XY flexure stage motor mount (two nested orthogonal parallelograms of 1095 shim blades + TPU shear pads).
Blades: horizontal (length along x or y), width w along the motor axis z, thickness t in-plane-soft.  Inner block carries
the motor; middle frame is stage-1 (moves in y), outer frame is ground.  4 TPU pad columns in shear between ground plate
and inner block give the lateral stiffness and the damping; the blades only guide (kill tilt/yaw/axial).  Units mm in API.
`python study/build/mount_xy.py [sizes] [build]` prints props; build writes STLs to study/build/<size>/xy/.
Run from flexblade/ with PYTHONPATH=$PWD:$PWD/study:$PWD/study/build."""
import json, sys
from pathlib import Path
import numpy as np
import trimesh as tm
from mount_twin import cyl, box, diff, un

G, INF, E, GS, NU = 6.5e6, 0.4, 200e9, 77e9, 0.29
ETA_PAD, ETA_BL = 0.15, 0.001
HERE = Path(__file__).resolve().parent

SIZES = {  # k_lat target N/m (twin), motor+inner mass kg, thrust N, Omega_max rad/s, J_p, rule k_t, 1P Hz
    "7in_v6": dict(k_lat=35.6e3, m=0.055, T=15.0, Om=1636.0, Jp=7.64e-6, kt_need=166.0, f1p=183.0,
                   t=0.08, w=10.0, L=12.0, s=24.0, hp=3.0),
    "10in_v5": dict(k_lat=24.0e3, m=0.126, T=25.0, Om=952.0, Jp=2.55e-5, kt_need=128.0, f1p=108.0,
                    t=0.08, w=10.0, L=12.0, s=24.0, hp=3.0),
}
NB = 4          # blades per stage
LEV2 = 21.0     # lever (mm) of stage-2 blades for tilt about the in-plane axes


def kz_blade(t, w, L):  # blade loaded in its own plane (axial z): bending + shear
    t, w, L = t * 1e-3, w * 1e-3, L * 1e-3
    return 1 / (L**3 / (E * t * w**3) + L / (0.85 * GS * t * w))


def props(k_lat, m, T, Om, Jp, kt_need, f1p, t, w, L, s, hp, mm=0.010, N=4, **_):
    tb, wb, Lb, sb = t * 1e-3, w * 1e-3, L * 1e-3, s * 1e-3
    kb = NB * E * wb * tb**3 / Lb**3                         # stage lateral stiffness (guided-guided)
    k_pad = k_lat - kb
    A = k_pad * hp * 1e-3 / (INF * G)                        # total pad area m^2
    Dp = 2 * np.sqrt(A / N / np.pi) * 1e3
    kz = kz_blade(t, w, L)
    k1 = NB * kz * (sb / 2) ** 2
    k2 = NB * kz * (LEV2 * 1e-3) ** 2
    kt = 1 / (1 / k1 + 1 / k2)
    kax = NB * kz / 2
    ka = E * wb * tb / Lb                                   # blade axial
    kth = NB * ka * (sb / 2) ** 2 / 2
    I_stat = m * 0.015**2                                   # stator+inner block about z (generous)
    out = dict(kb=kb, k_pad=k_pad, f_pad=k_pad / k_lat, Dp=Dp, A_pad_mm2=A * 1e6, kt=kt, kt_need=kt_need, kt_margin=kt / kt_need,
               k_ax=kax, k_theta=kth, f_theta_hz=np.sqrt(kth / (I_stat + Jp)) / 2 / np.pi,
               f_theta_derate10_hz=np.sqrt(kth / 10 / (I_stat + Jp)) / 2 / np.pi)
    zeta = (ETA_PAD * k_pad + ETA_BL * kb) / (2 * k_lat)
    d3 = 3 * 9.81 * (m + mm) / k_lat
    out.update(zeta=zeta, f_x=np.sqrt(k_lat / m) / 2 / np.pi, f_y=np.sqrt(k_lat / (m + mm)) / 2 / np.pi,
               d3g_mm=d3 * 1e3, sig3g_mpa=3 * E * tb * d3 / Lb**2 / 1e6, pad_strain_3g=d3 / (hp * 1e-3))
    r = f1p / out["f_y"]
    u = 1e-6 / (m + mm) * r * r / abs(r * r - 1)            # residual imbalance 1 g mm at 1P
    out.update(u_1p_um=u * 1e6, sig_fat_mpa=3 * E * tb * u / Lb**2 / 1e6,
               tau_thrust_mpa=T / (NB * tb * Lb) / 1e6)
    b, a = min(w, L), max(w, L)
    tcr = (5.35 + 4 * (b / a) ** 2) * np.pi**2 * E / (12 * (1 - NU**2)) * (t / b) ** 2 / 1e6
    out.update(tau_cr_mpa=tcr, buckle_margin=tcr / out["tau_thrust_mpa"],
               euler_blade_n=4 * np.pi**2 * E * (wb * tb**3 / 12) / Lb**2, axial_lock_n_at3g=E * wb * tb * 0.6 * d3**2 / Lb**2)
    return out


# ---- geometry (mm).  z: base plate 0..2, pads 2..5, blades/frames 5..5+w
ZB, ZP = 2.0, 5.0
SLOT_D, CL = 4.0, 1.5


def sl(x0, x1, y0, y1, z0, z1):
    return box(x1 - x0, y1 - y0, z1 - z0, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def build(out: Path, L=12.0, w=10.0, t=0.08, s=24.0, hp=3.0, Dp=3.5, N=4, **_):
    out.mkdir(parents=True, exist_ok=True)
    zt = ZP + w
    ts = t + 0.30                       # slot width
    sx = s / 2
    hi = 15.0                           # inner block half
    yc = hi + L                         # middle crossbar inner face
    xm = hi + CL                        # middle cavity half-x
    xw = xm + 5.0                       # middle outer x
    xo = xw + L                         # outer wall inner face
    ym = yc + 6.0                       # middle outer y
    inner = sl(-hi, hi, -hi, hi, ZP, zt)
    cut = [cyl(5.5, w + 2, z0=ZP - 1)]
    for a in range(4):
        th = a * np.pi / 2
        cut.append(cyl(1.6, w + 2, z0=ZP - 1, pos=(9.5 * np.cos(th), 9.5 * np.sin(th))))
        th += np.pi / 4
        cut.append(cyl(1.6, w + 2, z0=ZP - 1, pos=(8 * np.cos(th), 8 * np.sin(th))))
    pads = [(12 * np.cos(a * np.pi / 2), 12 * np.sin(a * np.pi / 2)) for a in range(N)]
    for p in pads:
        cut.append(cyl(Dp / 2 - 0.4, 1.6, z0=ZP - 0.01, pos=p))
    for sy in (-1, 1):
        for xx in (-sx, sx):
            y0, y1 = (hi - SLOT_D, hi + 0.01) if sy > 0 else (-hi - 0.01, -hi + SLOT_D)
            cut.append(sl(xx - ts / 2, xx + ts / 2, y0, y1, ZP + 1.0, zt + 1))
    inner = diff(inner, *cut)
    mid = un(sl(-xw, -xm, -ym, ym, ZP, zt), sl(xm, xw, -ym, ym, ZP, zt))
    mid = un(mid, un(sl(-xw, xw, yc, ym, ZP, zt), sl(-xw, xw, -ym, -yc, ZP, zt)))
    cut = []
    for sy in (-1, 1):
        for xx in (-sx, sx):
            y0, y1 = (yc - 0.01, yc + SLOT_D) if sy > 0 else (-yc - SLOT_D, -yc + 0.01)
            cut.append(sl(xx - ts / 2, xx + ts / 2, y0, y1, ZP + 1.0, zt + 1))
    for sx_ in (-1, 1):
        for yy in (-sx, sx):
            x0, x1 = (xw - SLOT_D, xw + 0.01) if sx_ > 0 else (-xw - 0.01, -xw + SLOT_D)
            cut.append(sl(x0, x1, yy - ts / 2, yy + ts / 2, ZP + 1.0, zt + 1))
    mid = diff(mid, *cut)
    xp, yp = xo + 6.0, ym + 4.0
    outer = un(sl(-xp, xp, -yp, yp, 0, ZB), un(sl(-xp, -xo, -yp, yp, ZB, zt), sl(xo, xp, -yp, yp, ZB, zt)))
    cut = []
    for sx_ in (-1, 1):
        for yy in (-sx, sx):
            x0, x1 = (xo - 0.01, xo + SLOT_D) if sx_ > 0 else (-xo - SLOT_D, -xo + 0.01)
            cut.append(sl(x0, x1, yy - ts / 2, yy + ts / 2, ZP + 1.0, zt + 1))
    for p in pads:
        cut.append(cyl(Dp / 2 - 0.4, 1.6, z0=ZB - 1.59, pos=p))
    for xx in (-xp + 4, xp - 4):
        for yy in (-yp + 4, yp - 4, 0):
            cut.append(cyl(1.7, ZB + 2, z0=-1, pos=(xx, yy)))
    outer = diff(outer, *cut)
    pad = un(cyl(Dp / 2, hp), cyl(Dp / 2 - 0.5, hp + 2.8, z0=-1.4))
    blade = box(L + 2 * (SLOT_D - 0.5), w - 0.4, t, (0, 0, 0))
    parts = dict(inner_block=inner, middle_frame=mid, outer_frame=outer, tpu_pad=pad, shim_blade=blade)
    info = {}
    for n, m in parts.items():
        m.export(out / f"{n}.stl")
        info[n] = dict(watertight=bool(m.is_watertight), vol_mm3=round(float(m.volume)), bbox=[round(float(x), 1) for x in m.extents])
    info["footprint_mm"] = [round(2 * xp, 1), round(2 * yp, 1)]
    info["height_mm"] = zt
    info["blades"] = dict(n=2 * NB, t=t, w=w - 0.4, L_free=L, strip_len=L + 2 * (SLOT_D - 0.5), material="steel_1095_shim")
    info["pads"] = dict(n=N, D=Dp, h=hp, mat="TPU 85A", note="glued 1 mm-radius tenons in sockets of inner block and base plate")
    (out / "xy_info.json").write_text(json.dumps(info, indent=1))
    return info


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if a != "build"]
    for n in (args or SIZES):
        p = props(**SIZES[n])
        print(n, {k: float(f"{v:.4g}") for k, v in p.items()})
        if "build" in sys.argv:
            P = {k: SIZES[n][k] for k in ("L", "w", "t", "s", "hp")}
            P["Dp"] = round(p["Dp"], 2)
            print(json.dumps(build(HERE / n / "xy", **P)))
