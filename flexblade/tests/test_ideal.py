"""topology == "ideal": material-free 6×6 root spring + mass knobs."""
import numpy as np
import pytest

from flexblade import fact, presets, rotor
from flexblade.optimize import set_design
from flexblade.schema import IdealFlexure


def _ideal(d, **kw):
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(
        topology="ideal", ideal=IdealFlexure(**kw)))))


@pytest.fixture(scope="module")
def base():
    return set_design(presets.make_preset(5, "cf_flexure"), {"blade.elastic_axis_chord_frac": 0.45})


def _cr(model):
    aero = rotor.make_aero(model)
    ss = rotor.static_trim(model, aero)
    return rotor.harmonic_1p(model, ss, aero).result.control_ratio


def test_ideal_K6_principal_axes(base):
    d = _ideal(base, k_hinge=0.1, k_cross=30.0, k_flap=50.0)
    K = fact.ideal_K6(d)
    _, a = fact.hinge_axis(d, 1)
    assert a @ K[3:, 3:] @ a == pytest.approx(0.1)
    assert K[4, 4] == pytest.approx(50.0)
    assert np.allclose(K, K.T)
    assert np.allclose(K[:3, 3:], 0.0)


def test_ideal_reduces_to_skewed_pin_hinge(base):
    """Given the skewed pin hinge's K (soft about a, k_rot about the other axes, k_trans in
    translation), the ideal topology reproduces the pin-hinge reference model."""
    base = base.model_copy(update=dict(analysis=base.analysis.model_copy(update=dict(root_drag=False))))
    kh = dict(k_hinge=0.05, k_rot=1e3, k_trans=1e7)   # root-hardware drag differs by topology
    cr_pin = _cr(rotor.build_model(base, ideal_hinge=kh))
    d = _ideal(base, k_hinge=0.05, k_cross=1e3, k_flap=1e3, k_axial=1e7, k_inplane=1e7, k_outplane=1e7)
    r = rotor.evaluate(d)
    assert r.summary["control_ratio"] == pytest.approx(cr_pin, rel=1e-3)
    assert r.summary["hinge_k_free"] == pytest.approx(0.05, rel=1e-2)
    # no flexure-member (material) checks; aero / motor / ESC checks remain
    names = {c.member for c in r.checks}
    assert {"esc", "motor"} <= names
    assert not any(m.startswith("flex") for m in names)


def test_ideal_explicit_K6_and_mass_knobs(base):
    d = _ideal(base, k_hinge=0.05, k_cross=1e3, k_flap=1e3)
    d2 = _ideal(base, K6=fact.ideal_K6(d).tolist())
    assert np.allclose(fact.ideal_K6(d2), fact.ideal_K6(d))
    m0 = rotor.build_model(d)
    m1 = rotor.build_model(set_design(d, {"flexure.ideal.tip_mass": 1e-3,
                                          "flexure.ideal.lag_inertia": 1e-6,
                                          "flexure.ideal.loss_eta": 0.1}))
    assert m1.fe.elements[[e.kind for e in m1.fe.elements].index("spring")].loss_eta == 0.1
    assert _cr(m1) != pytest.approx(_cr(m0), rel=1e-3)
