"""MCP server tests: in-process client against the fake backend."""
import json

import anyio
import pytest

import fake_backend

mcp_mod = pytest.importorskip("flexblade.mcp_server")
from mcp import Client  # noqa: E402

P = "fake_5in_tpu_64d"


@pytest.fixture(autouse=True)
def fake(monkeypatch):
    fake_backend.install(monkeypatch)


def call(name, args, progress=None):
    async def go():
        async with Client(mcp_mod.mcp) as c:
            kw = {"progress_callback": progress} if progress else {}
            return await c.call_tool(name, args, **kw)
    return anyio.run(go)


def payload(r):
    assert not r.is_error, r.content[0].text
    if r.structured_content is not None and set(r.structured_content) != {"result"}:
        return r.structured_content
    text = r.content[0].text
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return text


def test_tools_listed_with_descriptions():
    async def go():
        async with Client(mcp_mod.mcp) as c:
            return await c.list_tools()
    tools = {t.name: t for t in anyio.run(go).tools}
    for n in ["get_schema", "list_presets", "get_preset", "list_materials", "validate_design",
              "set_design_values", "evaluate_design", "optimize_design", "sweep", "export_geometry",
              "explain_results"]:
        assert n in tools and len(tools[n].description) > 40
    assert "dotted" in tools["set_design_values"].description


def test_schema_presets_materials():
    assert "properties" in payload(call("get_schema", {"which": "Design"}))
    names = [p["name"] for p in payload(call("list_presets", {}))["presets"]]
    assert P in names
    assert payload(call("get_preset", {"name": P}))["name"] == P
    assert any(m["name"] == "pa6_cf" for m in payload(call("list_materials", {}))["materials"])


def test_validate_and_set():
    ok = payload(call("validate_design", {"design": P}))
    assert ok["ok"] and ok["design_hash"]
    bad = payload(call("validate_design", {"design": {"flexure": {"thickness": "x"}}}))
    assert bad["ok"] is False and bad["errors"]
    d = payload(call("set_design_values", {"design": P, "values": {"flexure.skew_deg": 30}}))
    assert d["flexure"]["skew_deg"] == 30
    r = call("set_design_values", {"design": P, "values": {"flexure.nope": 1}})
    assert r.is_error and "flexure.nope" in r.content[0].text


def test_evaluate_and_explain():
    r = payload(call("evaluate_design", {"design": {"preset": P, "set": {"flexure.thickness": 0.001}}}))
    assert "HARMONIC" in r["digest"] and "nodes" not in r["results"]["harmonic"]
    full = payload(call("evaluate_design", {"design": P, "detail": "full"}))
    assert full["results"]["harmonic"]["nodes"]
    txt = payload(call("explain_results", {"results": full["results"]}))
    assert "CHECKS" in txt and "blade 1" in txt


def test_optimize_with_progress():
    seen = []

    async def prog(p, total, msg):
        seen.append(p)
    req = {"design": P, "variables": [{"path": "flexure.thickness", "lo": 5e-4, "hi": 2e-3}],
           "targets": [{"quantity": "harmonic.control_ratio", "kind": "max"}], "max_evals": 6}
    r = payload(call("optimize_design", {"request": req}, progress=prog))
    assert r["n_evals"] == 6 and r["design"]["flexure"]["thickness"] > 0 and "digest" in r
    assert seen and seen[-1] == 6


def test_sweep_and_export(tmp_path):
    out = tmp_path / "s.jsonl"
    r = payload(call("sweep", {"design": P, "variables": {"flexure.thickness": "5e-4:2e-3:3",
                                                          "flexure.skew_deg": [30, 45]},
                               "out_path": str(out)}))
    assert r["n"] == 6 and len(r["rows"]) == 6 and "control_ratio" in r["rows"][0]
    assert len(out.read_text().splitlines()) == 6
    r = call("sweep", {"design": P, "variables": {"flexure.thickness": "1e-3:2e-3:50"}, "max_points": 10})
    assert r.is_error
    e = payload(call("export_geometry", {"design": P, "fmt": "stl", "path": str(tmp_path / "a.stl")}))
    assert (tmp_path / "a.stl").stat().st_size > 0 and e["path"].endswith("a.stl")
    e = payload(call("export_geometry", {"design": P, "fmt": "step"}))
    assert e["path"].endswith(".step")


def test_design_resource():
    async def go():
        async with Client(mcp_mod.mcp) as c:
            res = await c.list_resources()
            txt = await c.read_resource("flexblade://design")
            return res, txt
    res, txt = anyio.run(go)
    assert "flexblade://design" in [str(x.uri) for x in res.resources]
    assert "design contract" in txt.contents[0].text
