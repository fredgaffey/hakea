import numpy as np
import pytest

from flexblade import api, presets, robust, rotor


def _with(d, **kw):
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=kw)))


def test_pre_lag_geometry_mirrored():
    d = presets.make_preset(5, "cf_flexure")
    m0 = rotor.build_model(d)
    m = rotor.build_model(_with(d, pre_lag_deg=5.0))
    X0, X = m0.fe.X0, m.fe.X0
    lag = []
    for b0, b in zip(m0.blades, m.blades):
        tip0, tip = b0.chain_nodes[-1], b.chain_nodes[-1]
        dl = b.Rk.T @ (X[tip] - X0[tip])
        lag.append(dl[1])
        # rotation about the blade's own hinge axis through the hinge point: distance kept
        assert np.linalg.norm(X[tip] - b.hinge_point) == pytest.approx(
            np.linalg.norm(X0[tip] - b0.hinge_point), rel=1e-9)
    assert lag[0] == pytest.approx(lag[1], rel=1e-9) and abs(lag[0]) > 1e-4   # same lag
    pp = rotor.preset_pitch(m)
    assert pp[0] == pytest.approx(-pp[1]) and abs(pp[0]) > 0                 # opposite pitch


def test_pre_lag_pitch_axis_keeps_positions():
    d = presets.make_preset(5, "cf_flexure")
    m0 = rotor.build_model(d)
    m = rotor.build_model(_with(d, pre_lag_deg=5.0, pre_lag_axis="pitch"))
    for b0, b in zip(m0.blades, m.blades):
        assert np.allclose(np.asarray(m.fe.X0)[b.chain_nodes], np.asarray(m0.fe.X0)[b0.chain_nodes], atol=1e-12)
    pp = rotor.preset_pitch(m)
    assert pp[0] == pytest.approx(-pp[1]) and abs(pp[0]) > 0


def test_robust_variants_pin_pre_lag_reference():
    d = _with(presets.make_preset(5, "cf_flexure"), pre_lag_deg="auto")
    for _, v in robust.variants(d):
        assert v.flexure.pre_lag_ref_mass == pytest.approx(d.op.vehicle_mass)


def test_auto_pitch_preset_cancels_split():
    d = _with(presets.make_preset(5, "cf_flexure"), pre_lag_deg="auto", pre_lag_axis="pitch")
    r = api.evaluate(d, what=["trim"])
    assert abs(r.summary["trim_diff_pitch_deg"]) < 0.1
    r0 = api.evaluate(_with(d, pre_lag_deg=0.0), what=["trim"])
    assert abs(r0.summary["trim_diff_pitch_deg"]) > abs(r.summary["trim_diff_pitch_deg"])
