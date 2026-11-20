import numpy as np

from flexblade.motor import (HAKEA_REFERENCE, MOTOR_LIBRARY, capability, hub_speed_amp, kt,
                             max_rpm, motor_for_diameter)
from flexblade.schema import Motor


def test_kt():
    assert np.isclose(kt(Motor(kv=1300)), 0.007346, rtol=1e-3)


def test_capability_hakea_hover():
    m = Motor()
    c = capability(m, 18500, 0.058)
    assert c.dQ_limit == "current"
    assert 0.1 < c.dQ_max_1p < 0.4
    assert 5 < c.I_mean < 12
    assert 5 < c.copper_W_mean < 20
    assert c.bandwidth_gain < 1 and c.bandwidth_phase_deg < 0


def test_voltage_limit():
    m = Motor(max_current=1000)
    c = capability(m, 26000, 0.05)
    assert c.dQ_limit == "voltage"
    assert np.isclose(c.dQ_max_1p, c.dQ_voltage_limit)
    assert capability(m, max_rpm(m, 0.05) * 1.01, 0.05).dQ_max_1p == 0


def test_hub_speed():
    J = HAKEA_REFERENCE["J_m"] + HAKEA_REFERENCE["J_p"]
    assert 5 < hub_speed_amp(0.03, J, 15000) < 500


def test_library_and_sizes():
    for name, (mot, note) in MOTOR_LIBRARY.items():
        assert mot.kv > 0 and mot.J_rotor > 0 and note
    assert motor_for_diameter(5).name == "tmotor_2307_1300kv"
    assert motor_for_diameter(12).J_rotor > motor_for_diameter(4).J_rotor
