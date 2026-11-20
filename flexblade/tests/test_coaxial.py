import json

import numpy as np
import pytest

from flexblade import coaxial
from flexblade.aero import G0, AeroModel
from flexblade.motor import get_motor
from flexblade.presets import all_presets, make_coax_preset, make_preset
from flexblade.schema import (BladeHarmonic, Coaxial, HarmonicResult, InflowProfile, Results)


@pytest.fixture(scope="module")
def coax5():
    return make_coax_preset(5, "tpu_rod")


@pytest.fixture(scope="module")
def pair5(coax5):
    return coaxial.pair_trim(coax5)


@pytest.fixture(scope="module")
def res5(coax5):
    return coaxial.evaluate(coax5, single=coaxial.aero_only_evaluate)


# ------------------------------------------------------------------ aero hooks

def test_thrust_share_default():
    d = make_preset(5, "tpu_rod")
    d = d.model_copy(update=dict(op=d.op.model_copy(update=dict(thrust_share=0.4))))
    t = AeroModel(d).trim()
    assert np.isclose(t.T, 0.4 * d.op.vehicle_mass * G0, rtol=1e-6)


def test_extra_inflow_momentum_and_power():
    d = make_preset(5, "tpu_rod")
    t0 = AeroModel(d).trim()
    prof = InflowProfile(r_frac=[0.0, 1.0], v=[5.0, 5.0])
    d2 = d.model_copy(update=dict(op=d.op.model_copy(update=dict(extra_inflow=prof))))
    m2 = AeroModel(d2)
    t2 = m2.trim(thrust=t0.T)
    assert np.allclose(t2.v_ext, 5.0)
    assert t2.P > t0.P            # climb-like extra mass flow costs power
    assert t2.rpm > t0.rpm
    # annulus momentum with external velocity: dT = 4πρ r F (v_e + v) v
    dT = 4 * np.pi * t2.rho * t2.r * t2.F_loss * t2.U_P * t2.v_own
    assert np.allclose(dT[1:-1], t2.dT_dr[1:-1], rtol=1e-4, atol=1e-3)
    # empty profile = no effect
    d3 = d.model_copy(update=dict(op=d.op.model_copy(update=dict(extra_inflow=InflowProfile()))))
    assert np.isclose(AeroModel(d3).trim().rpm, t0.rpm)


# ------------------------------------------------------------------ interference

def test_wake_contraction(pair5):
    t = pair5.upper
    a = coaxial.wake_contraction(t, 0.1 * t.R)
    b = coaxial.wake_contraction(t, 0.5 * t.R)
    assert 0.78 < b < a < 1.0
    assert 0.80 < pair5.r_c < 0.9


def test_wake_profile(pair5):
    tU = pair5.upper
    r = np.linspace(0, tU.R, 200)
    v = coaxial.upper_wake_on_lower(tU, r, 0.25 * tU.R)
    rc = coaxial.wake_contraction(tU, 0.25 * tU.R) * tU.R
    assert np.all(v[r > rc + 0.1 * tU.R] < 1e-3 * v.max())
    # stream-tube continuity: wake volume flux = upper's annulus flux
    q_up = np.sum(tU.w_int * 2 * np.pi * tU.r * tU.F_loss * tU.v_own)
    q_w = np.trapezoid(2 * np.pi * r * v, r)
    assert abs(q_w / q_up - 1) < 0.1
    vu = coaxial.lower_upstream_on_upper(pair5.lower, r, 0.25 * tU.R)
    assert np.all(vu >= 0) and vu.max() < v.max()


# ------------------------------------------------------------------ trim

def test_torque_balance(coax5, pair5):
    W = coax5.op.vehicle_mass * G0
    assert pair5.converged
    assert np.isclose(pair5.upper.T + pair5.lower.T, W, rtol=1e-5)
    assert np.isclose(pair5.upper.Q, pair5.lower.Q, rtol=2e-3)
    assert 0.5 < pair5.share_upper < 0.6        # upper carries more at equal torque
    assert pair5.lower.rpm > pair5.upper.rpm      # lower works in the upper wake
    assert np.all(pair5.v_ext_lower >= 0) and pair5.v_ext_lower.max() > pair5.v_ext_upper.max()


def test_equal_rpm_and_fixed(coax5):
    d = coax5.model_copy(update=dict(coaxial=Coaxial(spacing_frac=0.25, trim="equal_rpm")))
    p = coaxial.pair_trim(d)
    assert np.isclose(p.upper.rpm, p.lower.rpm)
    assert np.isclose(p.upper.T + p.lower.T, d.op.vehicle_mass * G0, rtol=1e-5)
    assert p.upper.Q > p.lower.Q               # equal rpm: upper absorbs more torque
    with pytest.raises(ValueError):
        coaxial.pair_trim(coax5.model_copy(update=dict(coaxial=Coaxial(trim="fixed_rpm"))))


def test_lower_design_overrides(coax5):
    cx = Coaxial(lower_collective_offset_deg=2.0, lower_motor=get_motor("2004_1700kv_6s"),
                 lower_torque_mod_frac=0.1)
    d = coaxial.lower_design(coax5.model_copy(update=dict(coaxial=cx)), rpm=1000.0, share=0.4)
    assert d.layout == "single" and d.op.rpm == 1000.0 and d.op.thrust_share == 0.4
    assert d.op.collective_deg == coax5.op.collective_deg + 2.0
    assert d.motor.name == "2004_1700kv_6s" and d.op.torque_mod_frac == 0.1


# ------------------------------------------------------------------ evaluate

def test_evaluate_aero_only(coax5, res5):
    c = res5.coaxial
    W = coax5.op.vehicle_mass * G0
    assert np.isclose(c.thrust_total, W, rtol=1e-4)
    assert abs(c.yaw_torque_net) < 2e-3 * c.upper.trim.torque
    assert 1.1 < c.interference_power_factor < 1.45     # Leishman ideal κ_int ≈ 1.28
    assert 0.5 < c.figure_of_merit < 0.9
    assert c.yaw_authority > 0.2 * c.upper.trim.torque
    assert res5.trim is not None and res5.trim.rpm == c.rpm_upper
    assert any(w.startswith("upper:") for w in res5.warnings)
    assert any(w.startswith("lower:") for w in res5.warnings)
    for k in ("power_total", "rpm_lower", "yaw_authority", "power_single_rotor"):
        assert k in res5.summary
    json.dumps(res5.model_dump(mode="json"))
    Results.model_validate(res5.model_dump(mode="json"))


def test_coax_vs_single_power(res5):
    s = res5.summary
    # coax pair at the same weight: within ±25 % of one rotor carrying it all
    assert 0.75 < s["power_total"] / s["power_single_rotor"] < 1.25
    assert s["rpm_upper"] < s["rpm_single_rotor"]


def test_mirror_results():
    h = HarmonicResult(torque_amp=0.01, hub_speed_amp_rpm=1.0,
                       blades=[BladeHarmonic(blade=1, pitch_amp_deg=1, pitch_phase_deg=30,
                                             lag_amp_deg=1, lag_phase_deg=-10, flap_amp_deg=0,
                                             flap_phase_deg=5)],
                       hub_moment_mean=0.01, hub_moment_phase_deg=40, hub_moment_2p=0.001,
                       thrust_1p=0.0, moment_per_torque=1, control_ratio=0.01, tilt_equiv_deg=1,
                       nodes=[(0.01, 0.02, 0.0)], displacement_re=[[1, 2, 3, 4, 5, 6]])
    r = coaxial._mirror_results(Results(design_hash="x", harmonic=h), 0.008)
    hm = r.harmonic
    assert hm.hub_moment_phase_deg == -40 and hm.blades[0].pitch_phase_deg == -30
    assert hm.nodes[0] == (0.01, -0.02, -0.008)
    assert hm.displacement_re[0] == [1, -2, 3, -4, 5, -6]


def test_coax_presets():
    P = all_presets()
    for D in (5, 7, 10):
        d = P[f"{D}in_coax_tpu_rod"]
        assert d.layout == "coaxial" and 0.2 <= d.coaxial.spacing_frac <= 0.35
