"""Material-agnostic resize and material sweep plumbing."""
import numpy as np
import pytest

from flexblade import api
from flexblade.matsweep import (fitness, rotor_mass, set_material, slot_materials, used_slots)
from flexblade.resize import _props, member_section, resize_for_material, section_targets
from flexblade import fact


@pytest.fixture(scope="module")
def presets():
    return api.presets()


def _flex_soft(d):
    m = [m for m in fact.generate_members(d) if m.kind != "rigid"][0]
    return _props(member_section(m))["EI_soft"]


def test_slots(presets):
    d = presets["5in_tpu_cf_skin"]
    s = used_slots(d)
    assert s["skin"] == ["pa6_cf"] and s["body"] == ["tpu_64d"]
    assert "insert" not in s
    d2 = set_material(d, "skin", "petg_cf")
    assert slot_materials(d2, "skin") == ["petg_cf"]
    assert slot_materials(d2, "body") == ["tpu_64d"]
    with pytest.raises(ValueError):
        set_material(d, "insert", "steel_302_spring")


@pytest.mark.parametrize("preset,a,b", [("5in_tpu_monolithic", "tpu_64d", "tpu_95a"),
                                         ("10in_cf_flexure", "pa6_cf", "petg_cf")])
def test_resize_matches_stiffness_and_roundtrips(presets, preset, a, b):
    d = presets[preset]
    tgt = section_targets(d)
    d2, rep = resize_for_material(d, "body", b)
    assert rep["feasible"]
    # governing stiffness reproduced within 2 % for every element
    for e in rep["elements"]:
        assert abs(e["ratio"] - 1) < 0.02, e
    assert abs(_flex_soft(d2) / tgt["members"][0]["EI_soft"] - 1) < 0.02
    t2 = section_targets(d2)
    for s0, s1 in zip(tgt["stations"], t2["stations"]):
        assert abs(s1["EI_soft"] / s0["EI_soft"] - 1) < 0.02
    assert slot_materials(d2, "body") == [b]
    # A -> B -> A returns the original dimensions
    d3, _ = resize_for_material(d2, "body", a)
    assert np.isclose(d3.flexure.thickness, d.flexure.thickness, rtol=0.01)
    assert np.isclose(d3.flexure.width, d.flexure.width, rtol=0.01)
    for s0, s3 in zip(d.blade.stations, d3.blade.stations):
        tc0 = s0.thickness_frac or 0.12
        assert np.isclose(s3.thickness_frac or 0.12, tc0, rtol=0.01)


def test_resize_thin_hits_min_printable_then_width(presets):
    # TPU -> stiff CF filament: thickness would drop below 0.4 mm -> width shrinks instead
    d = presets["5in_tpu_monolithic"]
    d2, rep = resize_for_material(d, "body", "pa6_cf", t_min=0.4e-3)
    fl = [e for e in rep["elements"] if e["element"] == "flexure"][0]
    assert fl["new"]["thickness"] >= 0.4e-3 * 0.999
    if fl["new"]["thickness"] <= 0.4e-3 * 1.001:
        assert fl["new"]["width"] < d.flexure.width
        assert "width" in fl["param"]


def test_rotor_mass_and_fitness(presets):
    m = rotor_mass(presets["5in_tpu_cf_skin"])
    assert 0.5e-3 < m["total"] < 50e-3
    assert set(m["per_material"]) == {"tpu_64d", "pa6_cf"}
    assert fitness(0.01, 0.5, 0.5, 0.5) == pytest.approx(0.01)
    assert fitness(0.01, 2.0, 0.5, 0.5) == pytest.approx(0.0025)
    assert fitness(0.01, 0.5, 0.075, 0.5) == pytest.approx(0.005)
    assert fitness(0.01, 0.5, None, 0.5) == 0.0


def test_cli_resize_and_material_sweep(capfd, tmp_path):
    import json
    from flexblade import cli
    assert cli.main(["resize", "preset:5in_tpu_monolithic", "--slot", "body", "--material", "tpu_95a"]) == 0
    out = json.loads(capfd.readouterr().out)
    assert out["report"]["feasible"] and out["design"]["flexure"]["thickness"] > 0.0025
    p = tmp_path / "ms.csv"
    assert cli.main(["material-sweep", "preset:5in_tpu_monolithic", "--slot", "body",
                     "--materials", "tpu_95a,tpu_64d", "--workers", "1", "--out", str(p)]) == 0
    capfd.readouterr()
    lines = p.read_text().splitlines()
    assert lines[0].startswith("material,") and len(lines) == 3


def test_material_sweep_small(presets):
    recs = api.material_sweep(presets["5in_tpu_monolithic"], "body", "tpu_95a,tpu_64d", workers=1)
    assert [r["material"] for r in recs] == ["tpu_95a", "tpu_64d"]
    for r in recs:
        assert "error" not in r, r.get("error")
        assert r["control_ratio"] is not None and r["max_util"] > 0 and r["rotor_mass_g"] > 0
        assert r["governing"]
