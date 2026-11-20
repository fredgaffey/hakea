"""Filament database: loading, sanity ranges, URLs, derivation bookkeeping."""
import json
import re

import numpy as np
import pytest

from flexblade import filament_db as fdb
from flexblade.materials import (GENERIC_NAMES, LIBRARY, META, get_material, list_materials,
                                 select_materials)

ENTRIES = fdb.load_entries()
URL = re.compile(r"^https?://[A-Za-z0-9.-]+\.[A-Za-z]{2,}(/\S*)?$")

# (E1 range Pa, rho range kg/m3, Xt range Pa) per family
RANGES = {
    "elastomer": ((2e6, 1.5e9), (900, 1400), (3e6, 80e6)),
    "polymer": ((0.25e9, 40e9), (850, 1700), (10e6, 400e6)),
    "insert": ((20e9, 400e9), (1200, 8200), (200e6, 3500e6)),
}


def _family(e):
    if e["class"] in fdb.INSERT_CLASSES:
        return "insert"
    return "elastomer" if fdb.CLASSES[e["class"]].get("elastomer") else "polymer"


def test_database_present_and_big_enough():
    assert len(ENTRIES) >= 60
    brands = {e["brand"] for e in ENTRIES}
    assert len(brands) >= 12
    classes = {e["class"] for e in ENTRIES}
    for c in ("tpu", "pa_cf", "petg", "pla", "cf_rod", "steel_wire"):
        assert c in classes


def test_every_entry_registered_and_generic_kept():
    for g in GENERIC_NAMES:
        assert g in LIBRARY
    for e in ENTRIES:
        m = get_material(e["name"])
        assert m.name == e["name"]
        assert e["name"] in META


def test_no_duplicate_names():
    names = [e["name"] for e in ENTRIES]
    assert len(names) == len(set(names))
    assert not set(names) & set(GENERIC_NAMES)


def test_required_keys_and_schema_enums():
    req = fdb.JSON_SCHEMA["required"]
    for e in ENTRIES:
        for k in req:
            assert k in e, (e["name"], k)
        assert re.fullmatch(r"[a-z0-9_]+", e["name"])
        assert e["class"] in fdb.CLASSES
        assert e["confidence"] in ("high", "medium", "low")
        for k in fdb.JSON_SCHEMA["properties"]["sim"]["required"]:
            assert k in e["sim"], (e["name"], k)


def test_sane_ranges():
    for e in ENTRIES:
        m = get_material(e["name"])
        (elo, ehi), (rlo, rhi), (slo, shi) = RANGES[_family(e)]
        assert elo <= m.E1 <= ehi, (e["name"], m.E1)
        assert rlo <= m.rho <= rhi, (e["name"], m.rho)
        assert slo <= m.Xt <= shi, (e["name"], m.Xt)
        assert 0 < m.E3 <= m.E1 * 1.0001 and 0 < m.E2 <= m.E1 * 1.0001
        assert 0 < m.Zt <= m.Xt * 1.0001
        assert 0 < m.creep_strain_allow < 0.05
        assert 0 < m.fatigue_strain_amp_allow < 0.06
        # positive-definite compliance
        assert np.all(np.linalg.eigvalsh(m.compliance_local()) > 0), e["name"]


def test_urls_well_formed():
    for e in ENTRIES:
        assert URL.match(e["url"]), (e["name"], e["url"])
        if e.get("tds_url"):
            assert URL.match(e["tds_url"]), (e["name"], e["tds_url"])


def test_every_sim_field_has_provenance():
    for e in ENTRIES:
        for k in ("rho", "E1", "E2", "E3", "Xt", "Zt", "creep_strain_allow"):
            assert k in e["derived"] and e["derived"][k], (e["name"], k)
        # a "tds:" provenance must point to a non-null datasheet value
        for k, rule in e["derived"].items():
            if rule.startswith("tds:"):
                key = rule[4:].split()[0]
                src = dict(e["tds"])
                if "[conditioned values]" in rule:
                    src.update({a: b for a, b in (e["tds"].get("conditioned") or {}).items() if b is not None})
                assert src.get(key) is not None, (e["name"], k, rule)


def test_prices_normalised():
    for e in ENTRIES:
        p = e.get("price")
        if p and p.get("value") and p.get("per_kg", True) and p.get("currency") in fdb.FX_TO_USD:
            assert 3 <= p["usd_per_kg"] <= 2000, (e["name"], p)


def test_list_and_select():
    lm = list_materials()
    names = {m["name"] for m in lm}
    assert set(GENERIC_NAMES) <= names
    real = [m for m in lm if m["name"] in META]
    assert all({"brand", "product", "url", "usd_per_kg", "confidence", "group"} <= set(m) for m in real)
    tpu = select_materials("class:tpu")
    assert tpu and all(get_material(n).is_elastomer for n in tpu)
    assert not any(get_material(n).is_elastomer for n in select_materials("rigid"))
    assert select_materials("pla,tpu_95a") == ["pla", "tpu_95a"]


def test_variant_materials():
    base = get_material("tpu_95a")
    v = get_material("tpu_95a@E=0.8,S=1.2")
    assert np.isclose(v.E1, 0.8 * base.E1) and np.isclose(v.Xt, 1.2 * base.Xt)
    assert np.isclose(v.rho, base.rho)
    with pytest.raises(KeyError):
        get_material("tpu_95a@Q=2")


def test_derive_rules_are_used_when_tds_missing():
    rec = {"name": "x_tpu", "brand": "x", "product": "x", "class": "tpu", "url": "https://x.com",
           "tds": {"shore_hardness": "95A", "tensile_strength_xy_MPa": 40}, "confidence": "low"}
    sim, der = fdb.derive(rec, fdb.shore_modulus_fit(ENTRIES))
    assert "Shore" in der["E1"] and der["Xt"] == "tds:tensile_strength_xy_MPa"
    assert 5e6 < sim["E1"] < 200e6
    assert "class" in der["Zt"]


def test_data_file_has_schema():
    doc = json.loads(fdb.DATA_PATH.read_text())
    assert doc["schema_version"] == fdb.SCHEMA_VERSION
    assert doc["json_schema"]["title"].startswith("flexblade")
