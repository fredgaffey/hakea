"""Flat-base (support-free motor face) variants of the 5in lead (run from flexblade/).

solidcraft.rotor_commands(design, {"flat_base": True, ...}): see solidcraft._flat_base_doc. Strips, free length,
skew, cross angle and hinge radius are the lead's, so the beam model / scoring is unchanged.
  lead    -> study/cad/5in_flatbase/         (lead strips, 9 x 0.05 mm 301)
  ender   -> study/cad/5in_flatbase_ender/   (tonight's: 3 x 0.10 mm can-Al, boss 6.5 mm, 0.5 mm mouth chamfers)
  before_lead / before_ender: printability of the same parts without flat_base (no checks), for comparison.
Each writes rotor_script.json, hub.stl, blade_a.stl, blade_b.stl (rotor frame = print frame, z up),
iso.png, bottom.png, root.png, checks.json and prints the check table.
Usage: python study/cad/build_5in_flatbase.py lead|ender|before_lead|before_ender [--print-only]
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_5in as C                                            # noqa: E402
from flexblade.cad.solidcraft import rotor_commands, rotor_layout, part_names   # noqa: E402

ENDER = dict(n_lam=3, t_lam=0.10, width=4.0, slot_clear=0.25, blade_extra=0.0, mouth_chamfer=0.5, boss_h=6.5)
VARIANTS = {"lead": ({"flat_base": True}, "5in_flatbase"), "ender": (dict(ENDER, flat_base=True), "5in_flatbase_ender"),
            "before_lead": ({}, None), "before_ender": (ENDER, None)}
SWING_OK = 1.0                                                   # brief: swing gap >= 1.0 mm


def export(base, names, out, lay):
    stls = {}
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "s.json"
        for nm, fn in (("hub", "hub"), ("blade1", "blade_a"), ("blade2", "blade_b")):
            p.write_text(json.dumps(C.plus(base, ("solid.remove", {"bodies": [o for o in names if o != nm]}))))
            stls[fn] = out / f"{fn}.stl"
            subprocess.run([C.CLI, "run", str(p), "--quality", "fine", "--out", str(stls[fn]), "--quiet"],
                           check=True, capture_output=True)
        return stls


def snapshots(base, names, out, lay):
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "s.json"
        p.write_text(json.dumps(base))
        H = [round(v, 4) for v in lay["hinge"].tolist()]
        views = {"iso.png": ["--view", "iso"], "bottom.png": ["--view", "bottom"],
                 "root.png": ["--camera", json.dumps({"target": [H[0] * 0.6, 0, H[2]], "yaw": 0.7, "pitch": -0.3,
                                                      "distance": 42}), "--no-grid"]}
        for fn, v in views.items():
            subprocess.run([C.CLI, "snapshot", str(p), "--out", str(out / fn), "--width", "1200", "--height", "900"]
                           + v + C.body_args(names), check=True, capture_output=True)


def main(tag, print_only=False):
    ov, sub = VARIANTS[tag]
    design = json.load(open(C.DESIGN))
    base = rotor_commands(design, ov)
    lay = rotor_layout(design, ov)
    names = part_names(design)
    z_face = lay["z_base"] if lay["flat_base"] else -6.0
    out = C.HERE / sub if sub else Path(tempfile.mkdtemp(prefix=f"fb_{tag}_"))
    out.mkdir(parents=True, exist_ok=True)
    (out / "rotor_script.json").write_text(json.dumps(base, indent=1))
    rows = []
    if not print_only:
        ev = C.cli(base)
        by = {b["name"]: b for b in ev["bodies"]}
        rows = [(f"one_body {nm}", nm in by and by[nm]["kernel"]["shells"] == 1 and by[nm]["closed"],
                 f"shells {by[nm]['kernel']['shells']}, {by[nm]['volume_mm3']:.1f} mm3, bbox z "
                 f"{by[nm]['bbox']['min'][2]:.2f}..{by[nm]['bbox']['max'][2]:.2f}" if nm in by else "missing") for nm in names]
        rows.append(("no stray bodies", set(by) == set(names), f"{len(by)} bodies"))
        rows += C.land_checks(base, lay, float(design["flexure"]["length"]) * 1e3)
        r = C.cli(C.plus(base, ("inspect.interference", {})), results=True)[-1]
        itf = [(x["a"], x["b"], round(x.get("volume_mm3", 0.0), 3)) for x in r.get("interferences", [])]
        real = [x for x in itf if x[2] >= 0.01 and not ({x[0][:5], x[1][:5]} == {"strip", "strap"})]
        rows.append(("interference (fittings)", not real, str(real) if real else "none between printed parts and strips"))
        rows += [(a, ok and float(d.split("gap ")[1].split(" ")[0]) >= SWING_OK, d) for a, ok, d in C.swing_checks(base, lay)]
    stls = export(base, names, out, lay)
    prow, m = C.print_rows(stls, z_face)
    rows += prow
    if sub and not print_only:
        snapshots(base, names, out, lay)
    w = max(len(a) for a, _, _ in rows)
    for a, ok, det in rows:
        print(f"{a:<{w}}  {'PASS' if ok else 'FAIL':<6}  {det}")
    print(f"[{tag}] {sum(ok for _, ok, _ in rows)}/{len(rows)} PASS; z_base {lay['z_base']:.2f}, boss "
          f"{z_face:.2f}..{-6.0 + lay['boss_h']:.2f} ({-6.0 + lay['boss_h'] - z_face:.2f} mm) -> {out}")
    (out / "checks.json").write_text(json.dumps(dict(overrides=ov, z_base=lay["z_base"], printability=m,
                                                     rows=[dict(check=a, ok=bool(o), detail=d) for a, o, d in rows]),
                                                indent=1, default=str))
    return rows


if __name__ == "__main__":
    main(sys.argv[1], "--print-only" in sys.argv)
