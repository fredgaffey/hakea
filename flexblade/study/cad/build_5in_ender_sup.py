"""Tonight's Ender/PETG variant (3 x 0.10 mm can-Al, boss 6.5 mm) with DESIGNED BREAKAWAY SUPPORTS (run from flexblade/).

Two passes: (1) the tonight parts (study/cad/5in_ender_tonight/*.stl, rotor frame); (2) rotor_commands(design,
{..., "supports": True}, meshes=...) puts each part in its print frame (z up or flipped about x, lowest point on
z = 0) and adds `<part>_sup`: 0.8 mm walls on a 3.5 mm grid (cross-braced) from the plate to every exposed
downward surface steeper than 40 deg, ending 0.2 mm (one layer) below it; 1 mm clear of slots, lands, mouths.
Outputs study/cad/5in_ender_tonight_sup/: hub_sup.stl, blade_a_sup.stl, blade_b_sup.stl (part + support, one file
per part, print frame), <part>_iso.png, <part>_bottom.png, checks.json.
Usage: python study/cad/build_5in_ender_sup.py [hub|blade1|blade2 ...]
"""
import json
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_5in as C                                            # noqa: E402
from flexblade.cad.solidcraft import (rotor_commands, rotor_layout, part_names, print_frame, slot_boxes,  # noqa: E402
                                      support_commands)

OV = dict(n_lam=3, t_lam=0.10, width=4.0, slot_clear=0.25, blade_extra=0.0, mouth_chamfer=0.5, boss_h=6.5)
SRC, OUT = C.HERE / "5in_ender_tonight", C.HERE / "5in_ender_tonight_sup"
FILES = {"hub": "hub", "blade1": "blade_a", "blade2": "blade_b"}


def one(part, sc, names, meshes, lay):
    fn = FILES[part]
    keep = [part, f"{part}_sup"]
    s = C.plus(sc, ("solid.remove", {"bodies": [o for o in names if o not in keep]}))
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "s.json"
        p.write_text(json.dumps(s))
        subprocess.run([C.CLI, "run", str(p), "--quality", "fine", "--out", str(OUT / f"{fn}_sup.stl"), "--quiet"],
                       check=True, capture_output=True)
        for v in ("iso", "bottom"):
            subprocess.run([C.CLI, "snapshot", str(p), "--out", str(OUT / f"{fn}_{v}.png"), "--width", "1000", "--height",
                            "800", "--view", v, "--body", f"{part}:color=#3b7dd8", "--body", f"{part}_sup:color=#e0a020"],
                           check=True, capture_output=True)
    ev = C.cli(s)
    by = {b["name"]: b for b in ev["bodies"]}
    r = C.cli(C.plus(s, ("inspect.clearance", {"bodies": [part], "against": [f"{part}_sup"], "clearance": 3, "nearest": 1})),
              results=True)[-1]
    gap = (-max(o["volume_mm3"] for o in r["overlaps"]) if r["overlaps"] else min(x["distance_mm"] for x in r["near"]))
    T = meshes[part]
    R, dz = print_frame(T)
    Tp = T @ R.T + [0, 0, dz]
    ex = [(R @ o + [0, 0, dz], R @ E, rr) for o, E, rr in slot_boxes(lay, part)]
    _, cols = support_commands(Tp, f"{part}_sup", ex)
    cov = C.support_coverage(Tp, cols)
    sb = by.get(f"{part}_sup")
    return fn, dict(flipped=bool(R[2, 2] < 0), part_shells=by[part]["kernel"]["shells"],
                    sup_shells=sb["kernel"]["shells"] if sb else 0, sup_mm3=sb["volume_mm3"] if sb else 0.0,
                    sup_height=sb["bbox"]["max"][2] if sb else 0.0, part_sup_gap_mm=gap, **cov)


def main(parts):
    design = json.load(open(C.DESIGN))
    names = part_names(design)
    meshes = {p: C.stl_tris(SRC / f"{FILES[p]}.stl") for p in parts}
    ov = dict(OV, supports=True)
    lay = rotor_layout(design, ov)
    sc = rotor_commands(design, ov, meshes=meshes)
    names = names + [f"{p}_sup" for p in parts]
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "rotor_script_sup.json").write_text(json.dumps(sc))
    with ThreadPoolExecutor(3) as ex:
        res = dict(ex.map(lambda p: one(p, sc, names, meshes, lay), parts))
    for fn, m in res.items():
        print(f"{fn:8s} {'flipped' if m['flipped'] else 'z-up':7s} part shells {m['part_shells']}, support shells "
              f"{m['sup_shells']} ({m['sup_mm3']:.0f} mm3, {m['sup_height']:.1f} mm tall), part-support gap "
              f"{m['part_sup_gap_mm']:.2f} mm | >50 deg overhang {m['overhang_mm2']:.1f} mm2: unsupported exposed "
              f"{m['unsupported_mm2']:.1f}, over-part roofs {m['hidden_mm2']:.1f}")
    (OUT / "checks.json").write_text(json.dumps(dict(overrides=ov, parts=res), indent=1, default=float))


if __name__ == "__main__":
    main(sys.argv[1:] or ["hub", "blade1", "blade2"])
