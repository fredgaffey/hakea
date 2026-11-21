"""CSG of the damped isotropic motor mount (carrier, tube clamp + post, TPU washer). Units mm.
Run: PYTHONPATH=. python study/build/10in_v5/make_mount.py  (from flexblade/)."""
from pathlib import Path
import numpy as np
import trimesh as tm

OUT = Path(__file__).parent


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


# v5b: DECOUPLED twin-washer mount (study/mount_soft_10in.py). Two TPU 85A washers ri 4 / ro 5.8 / t 12, centres 30 mm apart
# (lateral k 24 kN/m total -> hub mode 70 Hz; tilt k 19 N m/rad -> 0.5-0.9 deg at 0.17-0.3 N m). Preload the M4 so BOTH carry thrust.
# Carrier = plate (motor) + cup skirt; post (clamp-side) has two flanges d 12 that bear on the inner faces of the washers.
washer = diff(cyl(5.8, 12), cyl(4, 12))
plate = box(30, 30, 4, (0, 0, 2))
holes = [cyl(1.6, 4, pos=(sx * d / 2, sy * d / 2), sections=24) for d in (16, 19) for sx in (-1, 1) for sy in (-1, 1)]
cup = diff(cyl(8, 46, z0=-42), cyl(6.2, 46, z0=-42))      # skirt hanging below plate: washer 1 z -12..0, washer 2 z -42..-30
cap = diff(cyl(8, 2, z0=-44), cyl(4.6, 2, z0=-44))       # retainer ring under washer 2 (separate print, M2 x3 not modelled)
carrier = diff(plate.union(cup, engine="manifold"), cyl(4.5, 4), *holes)
carrier = diff(carrier, cyl(6.2, 14, z0=-12))             # keep bore for washer 1 (open)
carrier.export(OUT / "mount_carrier.stl")
# post: rod d 7.8, flanges d 12 x 2 at z -12..-14 and -28..-30 (washer inner faces, 16 mm web between)
tpost = cyl(3.9, 48, z0=-44).union(cyl(6, 2, z0=-14), engine="manifold").union(cyl(6, 2, z0=-30), engine="manifold")
tpost.export(OUT / "mount_post.stl")

# tube clamp: 22x22x20 block (tube axis along Y), 10.1 bore, slit + M3 pinch bolt, central post d 7.8 w flange d 18
blk = box(22, 24, 20, (0, 0, 10))
bore = tm.creation.cylinder(radius=5.05, height=30, sections=48)
bore.apply_transform(tm.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
bore.apply_translation([0, 0, 10])
slit = box(22, 24, 1.0, (0, 0, 17.5))
pinch = tm.creation.cylinder(radius=1.6, height=30, sections=24)
pinch.apply_transform(tm.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
pinch.apply_translation([0, 0, 17.5])
clamp = diff(blk, bore, slit, pinch)
flange = cyl(9, 2, z0=20)
post = cyl(3.9, 12, z0=22)         # post through washer bore (ri 4) and carrier, M4 screw + washer on top
clamp = clamp.union(flange, engine="manifold").union(post, engine="manifold")
clamp = diff(clamp, cyl(1.7, 20, z0=14, sections=24))   # M4 tap
clamp.export(OUT / "mount_tube_clamp.stl")
washer.export(OUT / "mount_tpu_washer.stl")
for n, m in (("carrier", carrier), ("clamp", clamp), ("washer", washer), ("tpost", tpost)):
    print(n, m.is_watertight, np.round(m.extents, 1), round(m.volume / 1000, 2), "cm3")
