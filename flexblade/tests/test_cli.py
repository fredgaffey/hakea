"""CLI tests against the fake backend (tests/fake_backend.py)."""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

import fake_backend
from flexblade import cli

PRESET = "preset:fake_5in_tpu_64d"
FLEXBLADE = Path(sys.executable).with_name("flexblade")


@pytest.fixture(autouse=True)
def fake(monkeypatch):
    fake_backend.install(monkeypatch)
    monkeypatch.delenv("FLEXBLADE_FAKE", raising=False)


def run(capfd, *argv):
    code = cli.main(list(argv))
    out, err = capfd.readouterr()
    return code, out, err


def one_json(out):
    lines = [ln for ln in out.splitlines() if ln.strip()]
    assert len(lines) == 1, out
    return json.loads(lines[0])


def test_schema(capfd):
    code, out, _ = run(capfd, "schema", "--which", "Design")
    assert code == 0
    s = one_json(out)
    assert "flexure" in s["properties"]


def test_presets_list_and_one(capfd):
    code, out, _ = run(capfd, "presets")
    names = [p["name"] for p in one_json(out)["presets"]]
    assert "fake_5in_tpu_64d" in names
    code, out, _ = run(capfd, "presets", "fake_5in_tpu_64d")
    assert one_json(out)["name"] == "fake_5in_tpu_64d"
    code, out, _ = run(capfd, "presets", "nope")
    assert code == 1 and "unknown preset" in one_json(out)["error"]


def test_materials(capfd):
    code, out, _ = run(capfd, "materials")
    assert code == 0 and any(m["name"] == "tpu_64d" for m in one_json(out)["materials"])


def test_evaluate_full_compact_digest(capfd, tmp_path):
    code, out, _ = run(capfd, "evaluate", PRESET)
    r = one_json(out)
    assert code == 0 and r["harmonic"]["nodes"] and r["trim"]["rpm"] > 0
    code, out, _ = run(capfd, "evaluate", PRESET, "--compact", "--what", "trim,harmonic")
    r = one_json(out)
    assert "nodes" not in r["harmonic"]
    code, out, _ = run(capfd, "evaluate", PRESET, "--digest", "--set", "flexure.thickness=0.002")
    d = one_json(out)
    assert "HARMONIC" in d["digest"] and "CHECKS" in d["digest"]


def test_evaluate_from_file_and_stdin(capfd, tmp_path, monkeypatch):
    code, out, _ = run(capfd, "presets", "fake_7in_tpu_64d")
    p = tmp_path / "d.json"
    p.write_text(out)
    code, out, _ = run(capfd, "evaluate", str(p), "--compact")
    assert code == 0 and one_json(out)["design_hash"]
    import io
    monkeypatch.setattr(sys, "stdin", io.StringIO(p.read_text()))
    code, out, _ = run(capfd, "validate", "-")
    assert one_json(out)["ok"] is True


def test_set_and_get_and_validation_error(capfd):
    code, out, _ = run(capfd, "set", PRESET, "flexure.thickness=0.0015", "blade.layup.material=pa6_cf",
                       "op.rpm=null")
    d = one_json(out)
    assert d["flexure"]["thickness"] == 0.0015 and d["blade"]["layup"]["material"] == "pa6_cf"
    assert d["op"]["rpm"] is None
    code, out, _ = run(capfd, "set", PRESET, "flexure.nonsense=1")
    e = one_json(out)
    assert code == 1 and e["type"] == "ValidationError" and e["details"][0]["loc"] == "flexure.nonsense"
    code, out, _ = run(capfd, "get", PRESET, "flexure.skew_deg", "blade.diameter_in")
    assert one_json(out) == {"flexure.skew_deg": 45.0, "blade.diameter_in": 5.0}


def test_diff(capfd):
    code, out, _ = run(capfd, "diff", PRESET, "preset:fake_5in_pa6_cf")
    d = one_json(out)
    assert "flexure.layup.material" in d


def test_export(capfd, tmp_path):
    for fmt in ("json", "stl", "glb", "step"):
        p = tmp_path / f"x.{fmt}"
        code, out, _ = run(capfd, "export", PRESET, "--fmt", fmt, "-o", str(p))
        assert code == 0 and one_json(out)["path"] == str(p) and p.stat().st_size > 0


def test_sweep_file_and_stdout(capfd, tmp_path):
    p = tmp_path / "s.jsonl"
    code, out, _ = run(capfd, "sweep", PRESET, "--var", "flexure.thickness=0.6e-3:2e-3:3",
                       "--var", "flexure.skew_deg=30,45", "--out", str(p), "--workers", "1")
    s = one_json(out)
    assert code == 0 and s["n"] == 6 and s["errors"] == 0
    rows = [json.loads(x) for x in p.read_text().splitlines()]
    assert len(rows) == 6 and rows[0]["values"] == {"flexure.thickness": 0.0006, "flexure.skew_deg": 30}
    assert "nodes" not in rows[0]["results"]["harmonic"]
    code, out, _ = run(capfd, "sweep", PRESET, "--var", "flexure.width=0.004:0.01:2:log", "--workers", "1")
    lines = [json.loads(x) for x in out.splitlines()]
    assert len(lines) == 2 and abs(lines[1]["values"]["flexure.width"] - 0.01) < 1e-12


def test_sweep_bad_path_fails_fast(capfd):
    code, out, _ = run(capfd, "sweep", PRESET, "--var", "flexure.thicc=1:2:2")
    assert code == 1 and "thicc" in one_json(out)["error"]


def test_optimize(capfd, tmp_path):
    req = {"design": json.loads(fake_backend.fake_presets()["fake_5in_tpu_64d"].model_dump_json()),
           "variables": [{"path": "flexure.thickness", "lo": 5e-4, "hi": 2e-3}],
           "targets": [{"quantity": "harmonic.control_ratio", "kind": "max"}], "max_evals": 8}
    p = tmp_path / "req.json"
    p.write_text(json.dumps(req))
    code, out, err = run(capfd, "optimize", str(p), "--compact", "--progress")
    r = one_json(out)
    assert code == 0 and r["n_evals"] == 8 and "progress" in err


def test_digest_cmd(capfd, tmp_path):
    code, out, _ = run(capfd, "evaluate", PRESET)
    p = tmp_path / "r.json"
    p.write_text(out)
    code, out, _ = run(capfd, "digest", str(p))
    assert "TRIM" in one_json(out)["digest"]


def test_usage_error_is_json(capfd):
    with pytest.raises(SystemExit) as ex:
        cli.main(["evaluate", "--bogus"])
    out, _ = capfd.readouterr()
    assert ex.value.code == 2 and json.loads(out)["type"] == "usage"


def test_physics_missing_hint(capfd, monkeypatch):
    def boom(*a, **k):
        raise ModuleNotFoundError("No module named 'flexblade.rotor'")
    monkeypatch.setattr(fake_backend.api, "evaluate", boom)
    code, out, _ = run(capfd, "evaluate", PRESET)
    assert code == 1 and "hint" in one_json(out)


@pytest.mark.skipif(not FLEXBLADE.exists(), reason="console script not installed")
def test_subprocess_stdout_is_pure_json(tmp_path):
    """Real process: stdout must be exactly one JSON doc even if the backend prints."""
    env = dict(os.environ, FLEXBLADE_FAKE="1")
    r = subprocess.run([str(FLEXBLADE), "evaluate", PRESET, "--digest"], capture_output=True, text=True,
                       env=env, timeout=120)
    assert r.returncode == 0, r.stderr
    assert json.loads(r.stdout)["digest"]
    assert "FAKE backend" in r.stderr


def test_coaxial_digest_and_compact(capfd):
    code, out, _ = run(capfd, "evaluate", PRESET, "--set", "layout=coaxial", "--digest")
    d = one_json(out)["digest"]
    assert "COAXIAL TOTALS" in d and "UPPER ROTOR" in d and "LOWER ROTOR" in d and "yaw_authority" in d
    code, out, _ = run(capfd, "evaluate", PRESET, "--set", "layout=coaxial", "--compact")
    r = one_json(out)
    assert "nodes" not in r["coaxial"]["lower"]["harmonic"] and r["coaxial"]["rpm_lower"] > 0
