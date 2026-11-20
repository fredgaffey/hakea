import pytest

from flexblade.aero import AeroModel
from flexblade.motor import capability
from flexblade.presets import CONCEPTS, SIZES_IN, all_presets, scale_design
from flexblade.schema import Design


def test_all_presets_names():
    P = all_presets()
    assert "hakea_5in_hinge_equiv" in P
    for D in SIZES_IN:
        for c in CONCEPTS:
            d = P[f"{D}in_{c}"]
            assert isinstance(d, Design) and d.blade.diameter_in == D
            Design.model_validate(d.model_dump(mode="json"))


def test_concept_materials():
    P = all_presets()
    assert P["7in_tpu_rod"].flexure.layup.insert_material == "cf_rod_pultruded"
    assert P["7in_tpu_cf_skin"].blade.layup.skin_material == "pa6_cf"
    assert P["7in_cf_flexure"].flexure.layup.material == "pa6_cf"
    assert P["7in_tpu_monolithic"].flexure.topology == "integral_taper"


def test_scale_design():
    d5 = all_presets()["5in_tpu_rod"]
    d10 = scale_design(d5, 10)
    assert d10.name == "10in_tpu_rod"
    assert d10.blade.diameter_in == 10
    assert abs(d10.flexure.length / d5.flexure.length - 2) < 1e-9
    assert abs(d10.blade.stations[3].chord / d5.blade.stations[3].chord - 2) < 1e-9
    assert d10.op.vehicle_mass > d5.op.vehicle_mass
    assert d10.motor.name != d5.motor.name


@pytest.mark.parametrize("D", SIZES_IN)
def test_presets_hover_feasible(D):
    d = all_presets()[f"{D}in_tpu_monolithic"]
    t = AeroModel(d).trim()
    assert 0.55 < t.FoM < 0.85
    assert t.mach[-1] < 0.6
    c = capability(d.motor, t.rpm, t.Q)
    assert c.back_emf < 0.8 * c.V_max
    assert c.dQ_max_1p > 0.3 * t.Q      # can pulse ≥ ±30 % of hover torque
