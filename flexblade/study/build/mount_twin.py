"""Shared parametric twin-washer motor mount (TPU 85A washers, decoupled lateral/tilt). Units mm.
Lateral: k_lat = 2 infill G pi (ro^2-ri^2)/t.  Tilt: k_t = 2 Ec pi (ro^2-ri^2)/t (d/2)^2 + 2 Ec pi (ro^4-ri^4)/(4t),
Ec = 3.4 G infill (both washers preloaded so both carry thrust).  Sizing rule (never tilt-soft a 2-blade rotor):
k_t >= margin * Omega_max^2 * (I_t - J_p), checked at 2x the estimated motor I_t = 0.5 J_p + m (15 mm)^2.
Used by 7in_v6/make_mount.py and 10in_v5/make_mount.py (run from flexblade/ with PYTHONPATH=.)."""
import json
from pathlib import Path
import numpy as np
import trimesh as tm

G, INF = 6.5e6, 0.4
EC = 3.4 * G * INF


def props(ri, ro, t, d):
    a = np.pi * ((ro * 1e-3) ** 2 - (ri * 1e-3) ** 2) / (t * 1e-3)
    k_lat = 2 * INF * G * a
    k_loc = 2 * EC * np.pi * ((ro * 1e-3) ** 4 - (ri * 1e-3) ** 4) / (4 * t * 1e-3)
    k_t = 2 * EC * a * (d * 1e-3 / 2) ** 2 + k_loc
    return dict(k_lat=float(k_lat), k_t=float(k_t), k_ax=float(2 * EC * a))


def rule(Om_rad, I_t, J_p):
    """minimum k_t at margin 1 = Omega^2 (I_t - J_p)"""
    return Om_rad ** 2 * max(I_t - J_p, 0.0)


def cyl(r, h, z0=0.0, pos=(0, 0), sections=64):
    c = tm.creation.cylinder(radius=r, height=h, sections=sections)
    c.apply_translation([pos[0], pos[1], z0 + h / 2])
    return c


def box(sx, sy, sz, c=(0, 0, 0)):
    b = tm.creation.box((sx, sy, sz))
    b.apply_translation(c)
    return b


def diff(a, *bs):
    for b in bs:
        a = a.difference(b, engine="manifold")
    return a


def un(a, b):
    return a.union(b, engine="manifold")


def build(out: Path, ri, ro, t, d):
    """washer 1 z -t..0, washer 2 z -(d+t)..-d (centres d apart)."""
    assert d > t + 6
    washer = diff(cyl(ro, t), cyl(ri, t))
    rb = ro + 0.4                       # skirt bore
    plate = box(30, 30, 4, (0, 0, 2))
    holes = [cyl(1.6, 4, pos=(sx * s / 2, sy * s / 2), sections=24) for s in (16, 19) for sx in (-1, 1) for sy in (-1, 1)]
    L = d + t
    cup = diff(cyl(rb + 1.8, L + 4, z0=-L), cyl(rb, L + 4, z0=-L))
    carrier = diff(un(plate, cup), cyl(4.5, 4), *holes)
    carrier = diff(carrier, cyl(rb, t + 2, z0=-t))
    carrier.export(out / "mount_carrier.stl")
    cap = diff(cyl(rb + 1.8, 2, z0=-L - 2), cyl(4.6, 2, z0=-L - 2))   # retainer ring (separate print, M2 x3)
    cap.export(out / "mount_cap.stl")
    tpost = cyl(3.9, L + 6, z0=-L - 2)
    tpost = un(un(tpost, cyl(ro, 2, z0=-t - 2)), cyl(ro, 2, z0=-d))    # flanges on the washer inner faces
    tpost.export(out / "mount_post.stl")
    blk = box(22, 24, 20, (0, 0, 10))
    bore = tm.creation.cylinder(radius=5.05, height=30, sections=48)
    bore.apply_transform(tm.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    bore.apply_translation([0, 0, 10])
    slit = box(22, 24, 1.0, (0, 0, 17.5))
    pinch = tm.creation.cylinder(radius=1.6, height=30, sections=24)
    pinch.apply_transform(tm.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    pinch.apply_translation([0, 0, 17.5])
    clamp = diff(blk, bore, slit, pinch)
    clamp = un(un(clamp, cyl(9, 2, z0=20)), cyl(3.9, 12, z0=22))
    clamp = diff(clamp, cyl(1.7, 20, z0=14, sections=24))
    clamp.export(out / "mount_tube_clamp.stl")
    washer.export(out / "mount_tpu_washer.stl")
    info = {}
    for n, m in (("carrier", carrier), ("cap", cap), ("clamp", clamp), ("washer", washer), ("post", tpost)):
        info[n] = dict(watertight=bool(m.is_watertight), extents=np.round(m.extents, 1).tolist(),
                       cm3=round(m.volume / 1000, 2))
        print(n, info[n])
    (out / "mount_parts.json").write_text(json.dumps(dict(ri=ri, ro=ro, t=t, d=d, **props(ri, ro, t, d), parts=info), indent=1))
    return info
