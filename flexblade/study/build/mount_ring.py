"""Planar TPU pad-ring motor mount: N round pads on radius R between a motor plate and an arm plate.
Tilt loads pads in compression/tension: k_t = (N/2) k_c R^2 (+ N Ec I_pad/h), k_c = Ec A/h, Ec = 3.4 G inf (1+2 S^2),
S = D/(4h) (round pad).  Lateral loads pads in shear: k_lat = N inf G A/h.  Units mm in the API.
Run from flexblade/ with PYTHONPATH=$PWD:$PWD/study:$PWD/study/build."""
import json
from pathlib import Path
import numpy as np
import trimesh as tm
from mount_twin import cyl, box, diff, un

G, INF = 6.5e6, 0.4

SIZES = {  # N, R, D, h (mm; free pad length between plates), T = assumed max thrust (N) for sag
    "7in_v6": dict(N=4, R=52.0, D=5.0, h=6.0, T=15.0),
    "10in_v5": dict(N=4, R=56.0, D=5.0, h=8.0, T=25.0),
}


def props(N, R, D, h, T=0.0, inf=INF):
    A = np.pi * (D * 1e-3) ** 2 / 4
    hh, RR = h * 1e-3, R * 1e-3
    S = D / (4 * h)
    Ec = 3.4 * G * inf * (1 + 2 * S * S)
    kc = Ec * A / hh
    ks = inf * G * A / hh
    kt = N / 2 * kc * RR ** 2 + N * 3.4 * G * inf * (np.pi * (D * 1e-3) ** 4 / 64) / hh
    return dict(k_t=float(kt), k_lat=float(N * ks), k_ax=float(N * kc), S=float(S),
                sag_mm=float(T / (N * kc) * 1e3), slender=float(h / D))


def mount_kwargs(N, D, h, inf=INF, **_):
    """stability.Mount fields giving the same lateral stiffness (equivalent annulus, r_in ~ 0)"""
    return dict(r_in_mm=1e-6, r_out_mm=float(np.sqrt(N) * D / 2), length_mm=float(h), infill=inf)


def build(out: Path, N, R, D, h, T=0.0, **_):
    out.mkdir(parents=True, exist_ok=True)
    F, tf, nk, tp = D + 5.0, 1.2, 0.6, 3.0     # flange dia, flange thick, neck depth in plate, plate thick
    Rp = R + F / 2 + 3
    pos = [2 * np.pi * i / N + np.pi / 4 for i in range(N)]
    L = Rp - R + 2

    def slot_cuts():
        """T-slots (depth measured from the gap-side plate face into the plate), open at the plate rim"""
        cs = []
        for a in pos:
            for w, d0, d1 in ((F + 0.4, nk, nk + tf + 0.4), (D + 0.4, -0.01, nk + 0.01)):
                b = box(L, w, d1 - d0, (0, 0, 0))
                b.apply_translation([R + L / 2, 0, 0])
                b.apply_transform(tm.transformations.rotation_matrix(a, [0, 0, 1]))
                cs.append((b, (d0 + d1) / 2))
        return cs
    cs = slot_cuts()
    # arm plate: z -tp..0, gap face at z=0, slots cut downward
    bcut = []
    for c, dm in cs:
        c = c.copy()
        c.apply_translation([0, 0, -dm])
        bcut.append(c)
    bot = diff(cyl(Rp, tp, z0=-tp), cyl(9.0, tp + 2, z0=-tp - 1), *bcut)
    blk = box(22, 24, 20, (0, 0, -tp - 10))
    bore = tm.creation.cylinder(radius=5.05, height=30, sections=48)
    bore.apply_transform(tm.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    bore.apply_translation([0, 0, -tp - 10])
    slit = box(22, 24, 1.0, (0, 0, -tp - 2.5))
    pinch = tm.creation.cylinder(radius=1.6, height=30, sections=24)
    pinch.apply_transform(tm.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    pinch.apply_translation([0, 0, -tp - 2.5])
    arm_plate = un(bot, diff(blk, bore, slit, pinch))
    # motor plate: z h..h+tp, slots cut upward from z=h
    tcut = []
    for c, dm in cs:
        c = c.copy()
        c.apply_translation([0, 0, h + dm])
        tcut.append(c)
    mh = [cyl(1.6, tp + 2, z0=h - 1, pos=(sx * s / 2, sy * s / 2), sections=24)
          for s in (16, 19) for sx in (-1, 1) for sy in (-1, 1)]
    top = diff(cyl(Rp, tp, z0=h), cyl(4.5, tp + 2, z0=h - 1), *tcut, *mh)
    pad = un(un(cyl(F / 2, tf, z0=-nk - tf), cyl(D / 2, h + 2 * nk, z0=-nk)), cyl(F / 2, tf, z0=h + nk))
    arm_plate.export(out / "ring_arm_plate.stl")
    top.export(out / "ring_motor_plate.stl")
    pad.export(out / "ring_tpu_pad.stl")
    info = {}
    for n, m in (("arm_plate", arm_plate), ("motor_plate", top), ("pad", pad)):
        info[n] = dict(watertight=bool(m.is_watertight), extents=np.round(m.extents, 1).tolist(),
                       cm3=round(m.volume / 1000, 2))
        print(n, info[n])
    (out / "ring_parts.json").write_text(json.dumps(dict(N=N, R=R, D=D, h=h, **props(N, R, D, h, T), parts=info), indent=1))
    return info


if __name__ == "__main__":
    import sys
    here = Path(__file__).parent
    for k, P in SIZES.items():
        print(k, props(**P))
        if "build" in sys.argv:
            build(here / k / "ring", **P)
