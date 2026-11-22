"""SolidCraft CAD of 5in_v8s_2307_m245_301: build outputs + connectivity checks (run from flexblade/).

Writes study/cad/5in_301/: rotor_script.json (pure modelling commands, replayable in Keel), per-part
<name>.step / <name>.stl (CLI --out only), iso.png / top.png / root.png. Prints a PASS/FAIL table:
  one_body      every part is ONE closed solid (kernel shell count 1), no stray bodies
  land/free     each strip stack, measured on the B-rep with inspect.classify_points (points inside both
                jaw walls = clamped): hub land and blade land >= the glue land, free length (hub exit face
                to blade exit face) within 0.1 mm of the design, land clamped continuously
  interference  inspect.interference over all bodies: no overlaps (strips sit in slot clearance)
  swing         hub-to-blade-fitting gap >= 0.5 mm with the fitting turned +-15 deg about lag (z), flap and
                the flexure hinge axis through the hinge point (solid.move + inspect.clearance)
Usage: python study/cad/check_5in.py [--no-export]
"""
import json
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

from flexblade.cad.solidcraft import (rotor_commands, rotor_layout, part_names, _blade_map, OVERDEEP)

CLI = os.environ.get("SOLIDCRAFT_CLI", str(Path.home() / "opt/solidcraft-0.46.2-linux-x86_64/bin/solidcraft-cli"))
HERE = Path(__file__).resolve().parent
OUT = HERE / "5in_301"
DESIGN = HERE.parents[0] / "designs/5in_v8s_2307_m245_301.json"
SWING_DEG, MIN_GAP, FREE_TOL, STEP = 15.0, 0.5, 0.1, 0.01
COLORS = {"hub": "#3b7dd8", "blade": "#e08a2c", "strip": "#202020", "strap": "#20a040"}


def cli(script, *args, results=False):
    """Run a script; returns eval's measurement or run's per-command results."""
    with tempfile.TemporaryDirectory() as td:
        p, jo = Path(td) / "s.json", Path(td) / "o.json"
        p.write_text(json.dumps(script))
        cmd = [CLI, "run" if results else "eval", str(p), "--json-out", str(jo)] + (["--results"] if results else []) + list(args)
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr[-3000:])
        return json.loads(jo.read_text())


def plus(base, *cmds):
    return {"commands": base["commands"] + [{"command": c, "params": p} for c, p in cmds]}


def strip_probe(lay, k, i):
    """World-frame point builder along strip (k, i): p(t, v) = p0 + t t_hat + v n_hat (u = 0, strip middle)."""
    f, L = lay["strips"][i - 1], _blade_map(k, lay["n"])
    return lambda t, v=0.0: (L @ (f["p0"] + t * f["t"] + v * f["n"])).round(6).tolist(), f["L"]


def land_checks(base, lay, design_L):
    prm, n = lay["prm"], lay["n"]
    off = prm.slot_t / 2 + prm.wall / 2                      # mid-wall, each side of the slot
    fine = lambda c: [round(c + STEP * j, 4) for j in range(-30, 31)]
    coarse = lambda a, b: [round(a + 0.25 * j, 4) for j in range(int((b - a) / 0.25) + 1)]
    cmds, plan = [], []
    for k in range(1, n + 1):
        for i in (1, 2):
            P, L = strip_probe(lay, k, i)
            hub_t = coarse(-prm.land - 1, 1) + fine(0.0)
            bl_t = coarse(L - 1, L + prm.land + OVERDEEP + 1) + fine(L)
            st_t = fine(-prm.land) + fine(L + prm.land)
            for body, ts, offs in (("hub", hub_t, (-off, off)), (f"blade{k}", bl_t, (-off, off)), (f"strip_b{k}_{i}", st_t, (0.0,))):
                pts = [P(t, v) for t in ts for v in offs]
                cmds.append(("inspect.classify_points", {"body": body, "points": pts}))
                plan.append((k, i, body, ts, len(offs)))
    res = cli(plus(base, *cmds), results=True)[-len(cmds):]
    rows, per = [], {}
    for (k, i, body, ts, no), r in zip(plan, res):
        cls = [c["class"] in ("inside", "on") for c in r["results"]]
        ins = {t: all(cls[j * no:(j + 1) * no]) for j, t in enumerate(ts)}
        per.setdefault((k, i), {})[body.split("_")[0].rstrip("0123456789")] = ins
    for (k, i), d in per.items():
        L = strip_probe(lay, k, i)[1]
        hub, bl, st = d["hub"], d["blade"], d["strip"]
        t_hx = max(t for t, v in hub.items() if v and abs(t) <= 0.31)                   # hub exit face
        t_bx = min(t for t, v in bl.items() if v and abs(t - L) <= 0.31)                # blade exit face
        t_s0 = min(t for t, v in st.items() if v)                                        # strip ends
        t_s1 = max(t for t, v in st.items() if v)
        land_h, land_b, free = t_hx - t_s0, t_s1 - t_bx, t_bx - t_hx
        cont_h = all(v for t, v in hub.items() if t_s0 + 0.25 <= t <= t_hx - 0.25)
        cont_b = all(v for t, v in bl.items() if t_bx + 0.25 <= t <= t_s1 - 0.25)
        free_open = not any(v for t, v in hub.items() if t > t_hx + 0.05) and not any(v for t, v in bl.items() if t < t_bx - 0.05)
        ok = (land_h >= prm.land - 2 * STEP and land_b >= prm.land - 2 * STEP and abs(free - design_L) <= FREE_TOL
              and cont_h and cont_b and free_open)
        rows.append((f"strip_b{k}_{i}", ok, f"land hub {land_h:.2f} / blade {land_b:.2f} mm (glue land {prm.land:.1f}), "
                     f"free {free:.3f} mm (design {design_L:.3f}), clamped continuously {cont_h and cont_b}"))
    return rows


def swing_checks(base, lay):
    rows = []
    for k in range(1, lay["n"] + 1):
        L = _blade_map(k, lay["n"])
        H = (L @ lay["hinge"]).tolist()
        er = L @ np.array([1.0, 0, 0])
        axes = {"lag (z)": [0, 0, 1.0], "flap": np.cross([0, 0, 1.0], er).tolist(), "hinge axis": (L @ lay["hinge_axis"]).tolist()}
        gaps = []
        for an, ax in axes.items():
            for s in (-1, 0, 1):
                if s == 0 and an != "lag (z)":
                    continue
                cmds = ([("solid.move", {"bodies": [f"blade{k}"], "axis": [round(a, 9) for a in ax], "angle": s * SWING_DEG,
                                          "origin": [round(h, 6) for h in H]})] if s else [])
                cmds.append(("inspect.clearance", {"bodies": ["hub"], "against": [f"blade{k}"], "clearance": 30, "nearest": 1}))
                r = cli(plus(base, *cmds), results=True)[-1]
                g = -max(o["volume_mm3"] for o in r["overlaps"]) if r["overlaps"] else min(x["distance_mm"] for x in r["near"])
                gaps.append((g, f"{an} {s * SWING_DEG:+.0f}"))
        g, at = min(gaps)
        rest = next(x for x, a in gaps if a.endswith("+0"))
        rows.append((f"swing blade{k}", g >= MIN_GAP, f"min hub gap {g:.2f} mm at {at} deg (rest {rest:.2f}); "
                     + ", ".join(f"{a} {x:.2f}" for x, a in gaps if not a.endswith("+0"))))
    return rows


def body_args(names):
    a = []
    for nm in names:
        key = next(c for c in COLORS if nm.startswith(c))
        a += ["--body", f"{nm}:color={COLORS[key]}"]
    return a


def export(base, names, lay):
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "s.json"
        for nm in names:
            p.write_text(json.dumps(plus(base, ("solid.remove", {"bodies": [o for o in names if o != nm]}))))
            subprocess.run([CLI, "run", str(p), "--out", str(OUT / f"{nm}.step"), "--out", str(OUT / f"{nm}.stl"), "--quiet"],
                           check=True, capture_output=True)
        p.write_text(json.dumps(base))
        H = lay["hinge"].tolist()
        views = {"iso.png": ["--view", "iso"], "top.png": ["--view", "top"],
                 "root.png": ["--camera", json.dumps({"target": H, "yaw": 0.9, "pitch": 0.45, "distance": 48}), "--no-grid"],
                 "root_section.png": ["--camera", json.dumps({"target": H, "yaw": 0.0, "pitch": 0.0, "distance": 45}),
                                      "--section", "XZ", "--no-grid"]}
        for fn, v in views.items():
            subprocess.run([CLI, "snapshot", str(p), "--out", str(OUT / fn), "--width", "1200", "--height", "900"]
                           + v + body_args(names), check=True, capture_output=True)


def main(do_export=True):
    design = json.load(open(DESIGN))
    base = rotor_commands(design)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "rotor_script.json").write_text(json.dumps(base, indent=1))
    lay = rotor_layout(design)
    names = part_names(design)
    ev = cli(base)
    by = {b["name"]: b for b in ev["bodies"]}
    rows = []
    for nm in names:
        b = by.get(nm)
        ok = b is not None and b["kernel"]["shells"] == 1 and b["closed"]
        rows.append((f"one_body {nm}", ok, f"shells {b['kernel']['shells'] if b else '-'}, "
                     f"{b['volume_mm3']:.1f} mm3" if b else "missing"))
    stray = sorted(set(by) - set(names))
    rows.append(("no stray bodies", not stray, ", ".join(stray) or f"{len(by)} bodies"))
    rows += land_checks(base, lay, float(design["flexure"]["length"]) * 1e3)
    r = cli(plus(base, ("inspect.interference", {})), results=True)[-1]
    itf = [(x["a"], x["b"], round(x.get("volume_mm3", 0.0), 3)) for x in r.get("interferences", [])]
    rows.append(("interference", not itf and not r.get("unchecked"), str(itf) if itf else
                 f"none over all {len(by)} bodies (unchecked {len(r.get('unchecked', []))})"))
    rows += swing_checks(base, lay)
    if do_export:
        export(base, names, lay)
    w = max(len(a) for a, _, _ in rows)
    print(f"{'check':<{w}}  result  detail")
    for a, ok, det in rows:
        print(f"{a:<{w}}  {'PASS' if ok else 'FAIL':<6}  {det}")
    n_fail = sum(not ok for _, ok, _ in rows)
    print(f"{len(rows) - n_fail}/{len(rows)} PASS")
    (OUT / "checks.json").write_text(json.dumps([dict(check=a, ok=bool(ok), detail=d) for a, ok, d in rows], indent=1))
    return n_fail


if __name__ == "__main__":
    sys.exit(1 if main(do_export="--no-export" not in sys.argv) else 0)
