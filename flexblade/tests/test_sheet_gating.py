"""Sheet-stock gating (materials.LAMINABLE_SHEETS / SHEET_METALS): any dry sheet stack is n*k1,
Goodman only on metals, steel results unchanged."""
from pathlib import Path

import pytest

from flexblade import api, materials as M, rotor
from flexblade.schema import Design

LEAD = Path(__file__).resolve().parents[1] / "study" / "designs" / "5in_v8s_2307_m245_301.json"
# 5in_v8s_2307_m245_301 under the pre-gating (steel-only) code; run-to-run noise is ~2e-8
STEEL_REF = {"fatigue_strain|flex0": 0.126597846, "fatigue_stress|flex0": 0.142028966,
             "creep_strain|flex0": 0.19535034, "static_stress|flex0": 0.185375565,
             "interlayer|flex0": 0.208550166, "harmonic_jitter|blade": 0.598593314}


def _lead(**u) -> Design:
    d = Design.model_validate_json(LEAD.read_text())
    if "material" in u:
        u["layup"] = d.flexure.layup.model_copy(update=dict(material=u.pop("material")))
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=u)))


def test_gating_tuples():
    assert set(M.SHIM_STEELS) <= set(M.SHEET_METALS) <= set(M.LAMINABLE_SHEETS)
    assert set(M.LAMINABLE_SHEETS) == set(M.SHEET_MATERIALS)
    for n in ("fr4_g10", "cf_ud_pultruded_strip", "cf_woven_sheet", "pom_acetal", "pet_film", "pp_sheet"):
        assert n in M.LAMINABLE_SHEETS and n not in M.SHEET_METALS
    for n in M.SHEET_MATERIALS:
        M.get_material(n)
        assert n in M.SHIM_THICKNESSES


def _flex_EIy(d: Design) -> float:
    m = rotor.build_model(d)
    return min(s.EIy for e in m.fe.elements if e.kind == "flexure" for s in e.sections)


@pytest.mark.parametrize("mat", ["pet_film", "brass_c260_h"])
def test_non_steel_stack_is_n_k1(mat):
    k1 = _flex_EIy(_lead(material=mat, strip_laminae=1, strip_lam_eta=0.0))
    k3 = _flex_EIy(_lead(material=mat, strip_laminae=3, strip_lam_eta=0.0))
    assert k3 / k1 == pytest.approx(3.0, rel=0.02)          # unbonded: n*k1, not n^3*k1


def test_steel_results_unchanged():
    r = api.evaluate(_lead(), ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    got = {f"{c['name']}|{c['member']}": c["utilisation"] for c in r["checks"]}
    for k, v in STEEL_REF.items():
        assert got[k] == pytest.approx(v, rel=1e-6), k


def test_goodman_metals_only():
    assert "brass_c260_h".startswith(M.SHEET_METALS)
    assert not "pom_acetal".startswith(M.SHEET_METALS)
    assert rotor.STRIP_SU.get("brass_c260_h", M.get_material("brass_c260_h").Xt) == 420e6
