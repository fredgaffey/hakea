"""Ender 3 S1 / PETG structural spin-test variant of the 5in lead (run from flexblade/).

Overrides on 5in_v8s_2307_m245_301 (free length, skew, cross angle, hinge radius unchanged): drink-can
aluminium strips N_LAM x T_LAM mm x 4.0 mm, slot = n t + 0.25, blade slot depth = the glue land (stack
bottoms out: sets the free length), 0.5 mm 45 deg slot-mouth chamfers, boss OD 10 x 6.5 mm, bore 5.1.
Outputs study/cad/5in_ender_tonight/: rotor_script.json, hub.stl, blade_a.stl, blade_b.stl (--quality fine,
print frame = rotor frame, z up), strip_template.svg (1:1), iso.png, slot.png, checks.json; prints the table.
Usage: python study/cad/build_5in_ender_tonight.py [t_lam_mm] [n_lam]
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_5in as C                                            # noqa: E402
from flexblade.cad.solidcraft import rotor_commands, rotor_layout, part_names   # noqa: E402

T_LAM = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10        # Fred measures the can wall
N_LAM = int(sys.argv[2]) if len(sys.argv) > 2 else 3
OV = dict(n_lam=N_LAM, t_lam=T_LAM, width=4.0, slot_clear=0.25, blade_extra=0.0, mouth_chamfer=0.5, boss_h=6.5)
OUT = C.HERE / "5in_ender_tonight"


def template_svg(L, land, w, path):
    tot = L + 2 * land
    W, H = tot + 20, w + 30
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">',
         f'<rect x="10" y="8" width="{tot:.3f}" height="{w:.3f}" fill="none" stroke="black" stroke-width="0.1"/>',
         *[f'<line x1="{10 + x:.3f}" y1="6" x2="{10 + x:.3f}" y2="{10 + w:.3f}" stroke="red" stroke-width="0.1"/>'
           for x in (land, land + L)],
         f'<text x="10" y="{14 + w:.1f}" font-size="2.2">strip {tot:.2f} x {w:.1f} mm: land {land:.1f} | free {L:.2f} | land {land:.1f}'
         f' (red = slot exits). {4 * N_LAM} needed (4 stacks of {N_LAM}) + spares, grain along.</text>',
         f'<rect x="10" y="{18 + w:.1f}" width="10" height="1.5" fill="black"/>',
         f'<text x="22" y="{19.5 + w:.1f}" font-size="2.2">10 mm scale check: print at 100 %, measure this bar</text>',
         '</svg>']
    path.write_text("\n".join(s))


def main():
    design = json.load(open(C.DESIGN))
    base = rotor_commands(design, OV)
    lay = rotor_layout(design, OV)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "rotor_script.json").write_text(json.dumps(base, indent=1))
    names = part_names(design)
    ev = C.cli(base)
    by = {b["name"]: b for b in ev["bodies"]}
    rows = [(f"one_body {nm}", nm in by and by[nm]["kernel"]["shells"] == 1 and by[nm]["closed"],
             f"shells {by[nm]['kernel']['shells']}, {by[nm]['volume_mm3']:.1f} mm3, bbox z "
             f"{by[nm]['bbox']['min'][2]:.1f}..{by[nm]['bbox']['max'][2]:.1f}" if nm in by else "missing") for nm in names]
    rows.append(("no stray bodies", set(by) == set(names), f"{len(by)} bodies"))
    rows += C.land_checks(base, lay, float(design["flexure"]["length"]) * 1e3)
    r = C.cli(C.plus(base, ("inspect.interference", {})), results=True)[-1]
    itf = [(x["a"], x["b"], round(x.get("volume_mm3", 0.0), 3)) for x in r.get("interferences", [])]
    real = [x for x in itf if x[2] >= 0.01 and not ({x[0][:5], x[1][:5]} == {"strip", "strap"})]
    rows.append(("interference (fittings)", not real, str(real) if real else "none between printed parts and strips"))
    rows.append(("strap vs strips (design)", not any(x not in real for x in itf if x[2] >= 0.01),
                 str([x for x in itf if x not in real]) or "none"))
    rows += C.swing_checks(base, lay)
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "s.json"
        for nm, fn in (("hub", "hub"), ("blade1", "blade_a"), ("blade2", "blade_b")):
            p.write_text(json.dumps(C.plus(base, ("solid.remove", {"bodies": [o for o in names if o != nm]}))))
            subprocess.run([C.CLI, "run", str(p), "--quality", "fine", "--out", str(OUT / f"{fn}.stl"), "--quiet"],
                           check=True, capture_output=True)
        p.write_text(json.dumps(base))
        f1 = lay["strips"][0]
        mouth = (f1["p0"] + 0.0 * f1["t"]).tolist()
        for fn, v in (("iso.png", ["--view", "iso"]),
                      ("slot.png", ["--camera", json.dumps({"target": mouth, "yaw": 2.2, "pitch": -0.35, "distance": 26}), "--no-grid"])):
            subprocess.run([C.CLI, "snapshot", str(p), "--out", str(OUT / fn), "--width", "1200", "--height", "900"]
                           + v + C.body_args(names), check=True, capture_output=True)
    prm = lay["prm"]
    template_svg(f1["L"], prm.land, prm.w, OUT / "strip_template.svg")
    w = max(len(a) for a, _, _ in rows)
    for a, ok, det in rows:
        print(f"{a:<{w}}  {'PASS' if ok else 'FAIL':<6}  {det}")
    print(f"slot {prm.slot_t:.2f} x {prm.slot_w:.2f} mm, land {prm.land:.1f}, free {f1['L']:.3f}, stack {prm.t:.2f} mm")
    (OUT / "checks.json").write_text(json.dumps(dict(overrides=OV, slot_t=prm.slot_t, slot_w=prm.slot_w, land=prm.land,
                                                     free=f1["L"], rows=[dict(check=a, ok=bool(o), detail=d) for a, o, d in rows]), indent=1))


if __name__ == "__main__":
    main()
