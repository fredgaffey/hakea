import numpy as np

from flexblade.schema import BatterySpec, VehicleSpec
from flexblade.vehicle import (CELLS, discharge, evaluate_mission, make_pack)


def test_pack_li_ion_21700():
    p = make_pack(BatterySpec(chemistry="liion_21700", S=6, P=2))
    assert np.isclose(p.Ah, 9.0)
    assert np.isclose(p.energy_Wh, 9.0 * 3.6 * 6)
    assert np.isclose(p.I_cont, 90.0)
    cells = 12 * CELLS["liion_21700"]["kg"]
    assert cells < p.mass < cells * 1.3
    assert 190 < p.specific_energy_Wh_kg() < 240  # P45B cell 231 Wh/kg less overhead


def test_pack_lipo_specific_energy_and_ir():
    p = make_pack(BatterySpec(chemistry="lipo", S=4, capacity_Ah=1.5))
    assert 110 < p.specific_energy_Wh_kg() < 140
    p2 = make_pack(BatterySpec(chemistry="lipo", S=4, capacity_Ah=3.0))
    assert p2.R < p.R and p2.I_cont == 2 * p.I_cont


def test_ocv_and_sag():
    p = make_pack(BatterySpec(chemistry="lipo", S=4, capacity_Ah=1.5))
    assert p.ocv(1.0) > p.ocv(0.5) > p.ocv(0.1)
    assert np.isclose(p.ocv(1.0), 16.8)
    assert p.v_loaded(80.0, 0.5) < p.ocv(0.5)
    assert p.v_loaded(80.0, 0.5) > p.v_loaded(160.0, 0.5)
    assert p.current(1e6, 0.5) == float("inf")


def test_discharge_energy_scaling():
    p = make_pack(BatterySpec(chemistry="liion_21700", S=6, P=1))
    d1 = discharge(p, 100.0)
    d2 = discharge(p, 200.0)
    assert d1["endurance_s"] > 2 * d2["endurance_s"]          # sag losses grow with power
    assert d1["V_start"] > d1["V_mid"] > d1["V_end"]
    assert d1["energy_Wh"] <= p.energy_Wh * p.usable * 1.1


def test_mass_budget_consistency_single():
    spec = VehicleSpec(layout="single", diameter_in=6.0, payload_kg=0.05,
                       battery=BatterySpec(chemistry="lipo", S=4, capacity_Ah=1.5))
    r = evaluate_mission(spec, full=False)
    assert np.isclose(r.auw, sum(r.mass.values()))
    assert np.isclose(r.mass["battery"], r.pack.mass)
    assert np.isclose(r.payload_fraction, 0.05 / r.auw)
    # thrust-weight consistency: aero hover at the budgeted AUW
    assert 40 < r.P_shaft < 120
    assert r.P_elec > r.P_shaft
    assert 0.5 < r.headroom["emf_frac_mid"] < 0.65
    assert r.endurance_min_derated < r.endurance_min


def test_full_mission_sets_vehicle_mass_and_coax_heavier():
    b = BatterySpec(chemistry="lipo", S=4, capacity_Ah=1.5)
    s = evaluate_mission(VehicleSpec(layout="single", diameter_in=5.0, battery=b), full=True)
    assert np.isclose(s.design.op.vehicle_mass, s.auw)
    assert s.control_ratio is not None and s.control_ratio > 0
    c = evaluate_mission(VehicleSpec(layout="coaxial", diameter_in=5.0, battery=b), full=False)
    assert c.auw > s.auw and len(c.rpm) == 2
