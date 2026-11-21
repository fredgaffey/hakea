"""--motor / --vehicle-mass overrides in run_study (fast: no rotor solve)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "study"))
import run_study as rs  # noqa: E402
from flexblade import motor as mot, presets  # noqa: E402


def test_crm2204_preset():
    m = mot.get_motor("crm2204_1400kv")
    assert m.kv == 1400 and m.supply_V == 11.1 and m.max_current == 15.0 and m.J_rotor < 4.17e-6


def test_override_motor_and_mass(monkeypatch):
    monkeypatch.setattr(rs, "MOTOR", "crm2204_1400kv")
    monkeypatch.setattr(rs, "MASS_OVERRIDE", 0.18)
    d = rs._override(presets.make_preset(7.0, "cf_flexure"))
    assert d.motor.name == "crm2204_1400kv" and d.op.vehicle_mass == 0.18
    monkeypatch.setattr(rs, "MOTOR", None)
    monkeypatch.setattr(rs, "MASS_OVERRIDE", None)
    d0 = presets.make_preset(7.0, "cf_flexure")
    assert rs._override(d0) is d0
