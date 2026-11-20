"""GUI server tests (FastAPI TestClient) against the fake backend, plus JS syntax checks."""
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest

import fake_backend

pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402

from flexblade.gui.server import create_app  # noqa: E402

STATIC = Path(__file__).resolve().parents[1] / "flexblade" / "gui" / "static"
KEEL = Path(__file__).resolve().parents[1] / "flexblade" / "keel"
P = "fake_5in_tpu_64d"


@pytest.fixture
def client(monkeypatch):
    fake_backend.install(monkeypatch)
    return TestClient(create_app(["http://keel.example"]))


def test_meta(client):
    info = client.get("/api/info").json()
    assert info["fake_backend"] is True and "http://keel.example" in info["cors_origins"]
    assert "Design" in client.get("/api/schema").json()
    pres = client.get("/api/presets").json()["presets"]
    assert P in pres and pres[P]["flexure"]["thickness"] > 0
    assert client.get(f"/api/presets/{P}").json()["name"] == P
    assert client.get("/api/presets/nope").status_code == 422
    assert client.get("/api/materials").json()["materials"]
    h = client.get("/api/hints").json()
    assert h["fields"]["flexure.thickness"]["unit"] == "mm"


def test_validate_and_set(client):
    v = client.post("/api/validate", json={"design": {}}).json()
    assert v["ok"] and v["design"]["flexure"]["topology"]
    v = client.post("/api/validate", json={"design": {"blade": {"diameter_in": 99}}}).json()
    assert v["ok"] is False and v["errors"][0]["loc"] == "blade.diameter_in"
    d = client.post("/api/set", json={"design": P, "values": {"flexure.width": 0.01}}).json()
    assert d["flexure"]["width"] == 0.01
    r = client.post("/api/set", json={"design": P, "values": {"flexure.bad": 1}})
    assert r.status_code == 422 and r.json()["errors"]


def test_evaluate(client):
    r = client.post("/api/evaluate", json={"design": P})
    assert r.status_code == 200
    res = r.json()
    h = res["harmonic"]
    assert len(h["nodes"]) == len(h["displacement_re"]) == len(h["displacement_im"])
    assert len(h["displacement_re"][0]) == 6 and res["campbell"]["rpm"]
    # cached second call
    assert client.post("/api/evaluate", json={"design": P}).json()["design_hash"] == res["design_hash"]
    assert "TRIM" in client.post("/api/digest", json={"results": res}).json()["digest"]


def test_evaluate_backend_missing(client, monkeypatch):
    def boom(*a, **k):
        raise ModuleNotFoundError("No module named 'flexblade.rotor'")
    monkeypatch.setattr(fake_backend.api, "evaluate", boom)
    r = client.post("/api/evaluate", json={"design": {"preset": P, "set": {"flexure.length": 0.009}}})
    assert r.status_code == 501 and "hint" in r.json()


def test_geometry_and_export(client):
    g = client.post("/api/geometry", json={"design": P}).json()
    assert g["units"] == "m" and g["parts"]
    for part in g["parts"]:
        assert part["vertices"] and part["faces"] and part["color"].startswith("#") and "role" in part
    assert client.get(f"/api/geometry?preset={P}").json()["parts"]
    for fmt in ("step", "stl", "glb", "json"):
        r = client.post(f"/api/export?fmt={fmt}", json={"design": P})
        assert r.status_code == 200 and len(r.content) > 0
        assert f"{P}.{fmt}" in r.headers["content-disposition"]
    assert client.post("/api/export?fmt=dwg", json={"design": P}).status_code == 400


def test_session_design(client):
    d = client.get("/api/session/design").json()
    assert d["name"]
    client.put("/api/session/design", json={"design": {"preset": "fake_7in_pa6_cf"}})
    assert client.get("/api/session/design").json()["name"] == "fake_7in_pa6_cf"
    r = client.get("/api/export?fmt=step")
    assert r.status_code == 200 and r.text.startswith("ISO-10303-21")


def _wait(client, jid, timeout=20):
    t0 = time.time()
    while time.time() - t0 < timeout:
        j = client.get(f"/api/jobs/{jid}").json()
        if j["status"] in ("done", "error"):
            return j
        time.sleep(0.05)
    raise AssertionError("job timeout")


def test_optimize_job(client):
    req = {"design": P, "variables": [{"path": "flexure.thickness", "lo": 5e-4, "hi": 2e-3}],
           "targets": [{"quantity": "harmonic.control_ratio", "kind": "max"}], "max_evals": 10}
    jid = client.post("/api/optimize", json=req).json()["job_id"]
    j = _wait(client, jid)
    assert j["status"] == "done", j["error"]
    assert j["progress"]["i"] == 10 and len(j["history"]) == 10
    assert j["result"]["design"]["flexure"]["thickness"] > 0 and "digest" in j["result"]
    assert client.post("/api/optimize", json={"design": P, "variables": [], "targets": [{"kind": "zz"}]}).status_code == 422
    assert client.get("/api/jobs/nope").status_code == 404


def test_sweep_job(client):
    jid = client.post("/api/sweep", json={"design": P, "variables": {"flexure.thickness": "1e-3:2e-3:3"}}).json()["job_id"]
    j = _wait(client, jid)
    assert j["status"] == "done" and len(j["result"]["rows"]) == 3
    assert client.post("/api/sweep", json={"design": P, "variables": {"flexure.zz": [1]}}).status_code == 422


def test_cors_and_private_network(client):
    h = {"Origin": "http://keel.example", "Access-Control-Request-Method": "GET",
         "Access-Control-Request-Private-Network": "true"}
    r = client.options("/api/export", headers=h)
    assert r.status_code == 200
    assert r.headers["access-control-allow-origin"] == "http://keel.example"
    assert r.headers["access-control-allow-private-network"] == "true"
    r = client.options("/api/export", headers=dict(h, Origin="http://evil.example"))
    assert r.status_code == 400 and r.headers.get("access-control-allow-origin") != "http://evil.example"
    r = client.get("/keel/flexblade-keel.js", headers={"Origin": "http://localhost:8787"})
    assert r.status_code == 200 and "export function install" in r.text
    assert r.headers["access-control-allow-origin"] == "http://localhost:8787"


def test_static_served(client):
    r = client.get("/")
    assert r.status_code == 200 and "importmap" in r.text and "three@0.160.0" in r.text
    for f in ("js/app.js", "js/viewer.js", "js/params.js", "style.css"):
        assert client.get(f"/{f}").status_code == 200


def test_refuses_non_loopback():
    from flexblade.gui.server import _is_loopback, serve
    assert _is_loopback("127.0.0.1") and _is_loopback("::1") and not _is_loopback("0.0.0.0")
    with pytest.raises(SystemExit):
        serve(host="0.0.0.0", port=1)


NODE = shutil.which("node")


@pytest.mark.skipif(not NODE, reason="node not installed")
@pytest.mark.parametrize("js", sorted(list((STATIC / "js").glob("*.js")) + list(KEEL.glob("*.js"))),
                         ids=lambda p: p.name)
def test_js_syntax(js, tmp_path):
    mjs = tmp_path / (js.stem + ".mjs")
    mjs.write_text(js.read_text())
    r = subprocess.run([NODE, "--check", str(mjs)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


@pytest.mark.skipif(not NODE, reason="node not installed")
def test_keel_parameter_map(tmp_path):
    """The Keel bridge's pure helpers produce CAD expressions with units."""
    mod = tmp_path / "k.mjs"
    mod.write_text((KEEL / "flexblade-keel.js").read_text())
    design = fake_backend.fake_presets()[P].model_dump(mode="json")
    script = (f"import {{ parameterMap }} from '{mod.as_uri()}';"
              f"console.log(JSON.stringify(parameterMap({json.dumps(design)})));")
    r = subprocess.run([NODE, "--input-type=module", "-e", script], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    m = json.loads(r.stdout)
    assert m["fb_flex_thickness"]["expression"] == "1.2 mm"
    assert m["fb_skew"]["expression"] == "45 deg" and m["fb_diameter"]["expression"] == "5 in"


@pytest.mark.skipif(os.environ.get("FLEXBLADE_BROWSER_TEST") != "1", reason="set FLEXBLADE_BROWSER_TEST=1")
def test_headless_browser(tmp_path):
    """Full render in headless Chrome (needs network for three.js CDN)."""
    import socket
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    env = dict(os.environ, FLEXBLADE_FAKE="1")
    exe = Path(sys.executable).with_name("flexblade")
    srv = subprocess.Popen([str(exe), "gui", "--port", str(port)], env=env)
    try:
        time.sleep(3)
        shoot = Path(__file__).resolve().parents[1] / "examples" / "shoot.py"
        r = subprocess.run([sys.executable, str(shoot), f"http://127.0.0.1:{port}", str(tmp_path)],
                           capture_output=True, text=True, timeout=240)
        assert r.returncode == 0, r.stdout + r.stderr
    finally:
        srv.terminate()


def test_coaxial_evaluate_and_geometry(client):
    d = {"preset": P, "set": {"layout": "coaxial"}}
    r = client.post("/api/evaluate", json={"design": d}).json()
    c = r["coaxial"]
    assert c["upper"]["harmonic"]["nodes"] and c["lower"]["harmonic"]["nodes"]
    assert c["rpm_upper"] > 0 and "yaw_authority" in c
    g = client.post("/api/geometry", json={"design": d}).json()
    rotors = {p["rotor"] for p in g["parts"]}
    assert rotors == {"upper", "lower"}
    assert any(p["name"].startswith("lower_") for p in g["parts"])
    h = client.get("/api/hints").json()
    assert "op.extra_inflow" in h["advanced"] and any(gr["key"] == "coaxial" for gr in h["groups"])
