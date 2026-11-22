"""SolidCraft rotor script: pure modelling commands (Keel-replayable) and, when the CLI is installed,
it builds every part as one closed solid with the strip stacks clear of the fittings."""
import json
import os
import shutil
import subprocess
from pathlib import Path

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
