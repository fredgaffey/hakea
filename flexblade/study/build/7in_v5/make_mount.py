"""CSG of the damped isotropic motor mount (carrier, tube clamp + post, TPU washer). Units mm.
Run: PYTHONPATH=. python study/build/7in_v5/make_mount.py  (from flexblade/)."""
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


# TPU washer (print 85A, 40 % infill-equivalent): ri 4, ro 9, t 10
washer = diff(cyl(9, 10), cyl(4, 10))

# carrier: 30x30x4 plate, 20 mm dia pocket (washer seat, 3 mm deep from below), M3 patterns 16x16 and 19x19
plate = box(30, 30, 4, (0, 0, 2))
holes = [cyl(1.6, 4, pos=(sx * d / 2, sy * d / 2), sections=24) for d in (16, 19) for sx in (-1, 1) for sy in (-1, 1)]
pocket = cyl(9.2, 3)
shaft_clear = cyl(4.5, 4)  # motor shaft / bearing boss clearance (through)
carrier = diff(plate, pocket, shaft_clear, *holes)
# retaining collar lip around the washer (carrier hangs from washer; shear gap = washer thickness)
carrier = carrier.union(diff(cyl(11.5, 3, z0=-3), cyl(9.2, 3, z0=-3)), engine="manifold")
carrier.export(OUT / "mount_carrier.stl")

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
flange = cyl(9, 2, z0=20)          # flange the washer bears on (annulus ro 9)
post = cyl(3.9, 12, z0=22)         # post through washer bore (ri 4) and carrier, M4 screw + washer on top
clamp = clamp.union(flange, engine="manifold").union(post, engine="manifold")
clamp = diff(clamp, cyl(1.7, 20, z0=14, sections=24))   # M4 tap
clamp.export(OUT / "mount_tube_clamp.stl")
washer.export(OUT / "mount_tpu_washer.stl")
for n, m in (("carrier", carrier), ("clamp", clamp), ("washer", washer)):
    print(n, m.is_watertight, np.round(m.extents, 1), round(m.volume / 1000, 2), "cm3")
