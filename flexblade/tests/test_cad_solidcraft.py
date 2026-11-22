"""SolidCraft rotor script: pure modelling commands (Keel-replayable) and, when the CLI is installed,
it builds every part as one closed solid with the strip stacks clear of the fittings."""
import json
import os
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from flexblade.cad.solidcraft import rotor_commands, part_names

ROOT = Path(__file__).resolve().parents[1]
DESIGN = ROOT / "study/designs/5in_v8s_2307_m245_301.json"
CLI = os.environ.get("SOLIDCRAFT_CLI", str(Path.home() / "opt/solidcraft-0.46.2-linux-x86_64/bin/solidcraft-cli"))
HAVE_CLI = Path(CLI).exists() or shutil.which("solidcraft-cli") is not None


@pytest.fixture(scope="module")
def design():
    if not DESIGN.exists():
        pytest.skip("design file missing")
    return json.load(open(DESIGN))


def test_script_is_pure_modelling(design):
    s = rotor_commands(design)
    cmds = s["commands"]
    assert cmds and all(c["command"].split(".")[0] in ("solid", "sketch") for c in cmds)
    txt = json.dumps(s)
    assert "file." not in txt and "/home" not in txt and "\\\\" not in txt
    made = {c["params"].get("body_name") for c in cmds}
    assert set(part_names(design)) <= made


@pytest.mark.skipif(not HAVE_CLI, reason="solidcraft-cli not installed")
def test_script_builds_one_solid_per_part(design, tmp_path):
    p, jo = tmp_path / "rotor.json", tmp_path / "eval.json"
    p.write_text(json.dumps(rotor_commands(design)))
    cli = CLI if Path(CLI).exists() else shutil.which("solidcraft-cli")
    r = subprocess.run([cli, "eval", str(p), "--json-out", str(jo)], capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stderr[-2000:]
    bodies = {b["name"]: b for b in json.loads(jo.read_text())["bodies"]}
    assert set(bodies) == set(part_names(design))
    for name, b in bodies.items():
        assert b["closed"] and b["kernel"]["shells"] == 1, name
        assert b["volume_mm3"] > 1.0, name
    # strip stack volume = (2 lands + free length) x width x stack thickness
    f = design["flexure"]
    v = (f["length"] * 1e3 + 16.0) * f["width"] * 1e3 * f.get("strip_laminae", 1) * f["thickness"] * 1e3
    assert abs(bodies["strip_b1_1"]["volume_mm3"] - v) < 0.05 * v


def test_flat_base_is_off_by_default(design):
    from flexblade.cad.solidcraft import rotor_layout
    assert rotor_commands(design) == rotor_commands(design, {"flat_base": False})
    fb = rotor_commands(design, {"flat_base": True})["commands"]
    boss = next(c["params"] for c in fb if c["params"].get("body_name") == "hub")
    zb = rotor_layout(design, {"flat_base": True})["z_base"]
    assert boss["base"][2] == pytest.approx(zb) and zb < -6.0          # motor face moved down to the lowest hub jaw


@pytest.mark.skipif(not HAVE_CLI, reason="solidcraft-cli not installed")
def test_flat_base_hub_sits_flat(design, tmp_path):
    """flat_base hub: one solid, lowest point = boss face, and no >50 deg overhang below z = -2.5 (the hub-down
    blade's pedestals, arms and spine; the upper blade's outer jaw group is a known residual above z = 2)."""
    import sys
    sys.path.insert(0, str(ROOT / "study/cad"))
    import check_5in as C
    from flexblade.cad.solidcraft import rotor_layout
    ov = {"flat_base": True}
    names = part_names(design)
    p, stl = tmp_path / "hub.json", tmp_path / "hub.stl"
    p.write_text(json.dumps(C.plus(rotor_commands(design, ov), ("solid.remove", {"bodies": [o for o in names if o != "hub"]}))))
    cli = CLI if Path(CLI).exists() else shutil.which("solidcraft-cli")
    r = subprocess.run([cli, "run", str(p), "--out", str(stl), "--quiet"], capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stderr[-2000:]
    zb = rotor_layout(design, ov)["z_base"]
    m = C.printability(stl)
    assert abs(m["z_min"] - zb) <= 0.05 and m["plate_mm2"] > 50.0
    T = C.stl_tris(stl)
    low = T[T[:, :, 2].max(axis=1) < -2.5]
    N = np.cross(low[:, 1] - low[:, 0], low[:, 2] - low[:, 0])
    A = np.linalg.norm(N, axis=1) / 2
    on_plate = (np.abs(low[:, :, 2] - zb) < 0.05).all(axis=1)
    bad = (N[:, 2] / np.maximum(2 * A, 1e-30) < -0.766) & ~on_plate
    assert A[bad].sum() < 1.0


def test_strap_route_is_off_by_default(design):
    """Default (and explicit 'axis') script is unchanged: the committed study/cad/5in_301 build replays byte for byte."""
    from flexblade.cad.solidcraft import rotor_layout, _stack_dist
    base = rotor_commands(design)
    assert base == rotor_commands(design, {"strap_route": "axis"})
    ref = ROOT / "study/cad/5in_301/rotor_script.json"
    if ref.exists():
        assert json.dumps(base, indent=1) == ref.read_text()
    lay = rotor_layout(design)
    assert part_names(design) == part_names(design, {"strap_route": "axis"})
    # the pitch-axis strand clips the crossing stacks (clear radius ~0.06 mm << strand radius 0.65 mm)
    assert _stack_dist(lay["strips"], lay["prm"], 0.0, 0.0, lay["x_hw"], lay["x_bw"]) < lay["prm"].strand_d / 2
    st = rotor_layout(design, {"strap_route": "pair_z"})["strap"]
    assert st["zs"] == (st["dz"], -st["dz"]) and 2.5 < st["dz"] < 4.0
    assert st["d"] == pytest.approx(lay["prm"].strand_d / 2 ** 0.5)      # area-equivalent pair
    with pytest.raises(ValueError):
        rotor_layout(design, {"strap_route": "bogus"})


@pytest.mark.skipif(not HAVE_CLI, reason="solidcraft-cli not installed")
def test_strap_route_pair_clears_strips(design):
    """pair_z: two strands per blade, every part one solid, and no interference at all (the axis strand clips)."""
    import sys
    sys.path.insert(0, str(ROOT / "study/cad"))
    import check_5in as C
    ov = {"strap_route": "pair_z"}
    names = part_names(design, ov)
    assert {"strap_b1_1", "strap_b1_2", "strap_b2_1", "strap_b2_2"} <= set(names) and "strap_b1" not in names
    base = rotor_commands(design, ov)
    bodies = {b["name"]: b for b in C.cli(base)["bodies"]}
    assert set(bodies) == set(names)
    assert all(b["closed"] and b["kernel"]["shells"] == 1 for b in bodies.values())
    itf = C.cli(C.plus(base, ("inspect.interference", {})), results=True)[-1]
    assert not itf.get("interferences") and not itf.get("unchecked"), itf.get("interferences")
