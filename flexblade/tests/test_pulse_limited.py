"""Pulse-limited scoring: scaling classification and cr_pl arithmetic (no solver runs)."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "study"))
import run_study as rs  # noqa: E402
from flexblade import forced as frc, motor as mot, robust  # noqa: E402


def _chk(name, u, value=0.0, note="", member="m"):
    return dict(name=name, member=member, utilisation=u, value=value, allowable=1.0, note=note)


@pytest.fixture(scope="module")
def design():
    return rs.base_design(7.0, "tt_strap_steelx")[0]


def test_classification():
    assert rs.PULSE_EXP["motor_thermal"] == 2
    for k in ("motor_torque_1p", "esc_duty_slew", "esc_duty_peak", "fatigue_strain", "fatigue_stress"):
        assert rs.PULSE_EXP[k] == 1
    for k in ("static_stress", "creep_strain", "interlayer", "strap_tension", "strip_clamp_wrap",
              "aeroelastic_stability", "air_resonance", "pitch_cf_ratio", "harmonic_jitter"):
        assert k not in rs.PULSE_EXP


def test_linear_scale_and_cr_pl(design):
    r = dict(checks=[_chk("motor_torque_1p", 0.5), _chk("fatigue_stress", 0.8), _chk("static_stress", 3.0)],
             summary={})
    out = dict(control_ratio=0.1, viol={"check:fatigue_stress": 0.0, "check:static_stress": math.log(3)})
    rs.add_pulse_limited(design, r, out)
    assert out["pulse_scale"] == pytest.approx(1 / 0.8, rel=1e-6)
    assert out["pulse_binding"] == "fatigue_stress"
    assert out["cr_pl"] == pytest.approx(0.1 / 0.8, rel=1e-6)
    assert set(out["viol_hard"]) == {"check:static_stress"}      # non-scaling stays hard
    assert not out["feasible_pl"]


def test_over_utilised_scales_down(design):
    r = dict(checks=[_chk("fatigue_strain", 2.0), _chk("motor_torque_1p", 0.5)], summary={})
    out = dict(control_ratio=0.2, viol={"check:fatigue_strain": math.log(2)})
    rs.add_pulse_limited(design, r, out)
    assert out["cr_pl"] == pytest.approx(0.1, rel=1e-6) and out["feasible_pl"]


def test_motor_cap_binds():
    s, b = rs.pulse_scale([("motor_torque_1p", lambda s: 0.95 * s), ("fatigue_stress", lambda s: 0.2 * s)])
    assert s == pytest.approx(1 / 0.95, rel=1e-6) and b == "motor_torque_1p"


def test_thermal_affine_in_s2(design):
    rpm, Q = 9000.0, 0.15
    u0 = mot.thermal(design.motor, rpm, Q, 0.0, design.op.temperature_C)["utilisation"]
    u1 = u0 + 0.3
    fns = rs.pulse_util_fns(design, dict(checks=[_chk("motor_thermal", u1)], summary=dict(rpm=rpm),
                                         trim=dict(torque=Q)))
    (_, f), = fns
    assert f(1.0) == pytest.approx(u1) and f(2.0) == pytest.approx(u0 + 0.3 * 4)
    s, _ = rs.pulse_scale(fns)
    assert u0 + 0.3 * s * s == pytest.approx(1.0, rel=1e-6)


def test_fatigue_np_part_conservative(design):
    note = "1/rev stress amp / (ratio × strength) srss nP: 1P 0.600 + nP 0.300"
    (_, f), = rs.pulse_util_fns(design, dict(checks=[_chk("fatigue_stress", 0.67, note=note)], summary={}))
    assert f(1.0) == pytest.approx(math.hypot(0.6, 0.3))
    assert f(2.0) == pytest.approx(math.hypot(1.2, 0.6))       # nP grows with s above 1
    assert f(0.5) == pytest.approx(math.hypot(0.3, 0.3))       # nP aero part kept when scaling down


def test_jitter_above_floor_is_hard_not_pulse_scaled(design):
    j = 1.5 * frc.JITTER_MAX_DEG     # jitter fails at s=1: hard violation, pulse scaling untouched
    r = dict(checks=[_chk("harmonic_jitter", 1.0, value=j)], summary=dict(pitch_amp_deg=0.1))
    out = dict(control_ratio=0.1, viol={})
    rs.add_pulse_limited(design, r, out)
    assert out["pulse_scale"] > 0.0 and "pulse_infeasible" not in out["viol_hard"]


def test_combine_per_throttle_schedule_and_j_variants(design):
    tags = [t for t, _ in robust.variants(design, (0.7, 1.4), None, (1e-6, 5e-6))]
    assert len(tags) == 4 and tags[0].endswith(":J1e-06")
    ds = [d for _, d in robust.variants(design, (1.0,), None, (1e-6, 5e-6))]
    assert [d.motor.J_hub for d in ds] == [1e-6, 5e-6]
    ms = [dict(control_ratio=0.1, cr_pl=0.08, pulse_scale=0.8, viol={}, viol_hard={}, util={}),
          dict(control_ratio=0.05, cr_pl=0.06, pulse_scale=1.2, viol={"check:fatigue_stress": 0.1},
               viol_hard={}, util={})]
    c = robust.combine(ms, ["T0.7", "T1.4"])
    assert c["cr_pl"] == 0.06 and c["feasible_pl"] and not c["feasible"]
    assert [p["pulse_scale"] for p in c["points"]] == [0.8, 1.2]
