import numpy as np
import pytest

from flexblade import presets, rotor


def _cone(d, b0):
    return d.model_copy(update=dict(blade=d.blade.model_copy(update=dict(precone_deg=float(b0)))))


def _my(d):
    m = rotor.build_model(d)
    ss = rotor.static_trim(m, rotor.make_aero(m))
    return m, ss, np.array([M[1] for _, M in rotor.flexure_root_loads(m, ss)])


@pytest.fixture(scope="module")
def base():
    d = presets.make_preset(5, "cf_flexure")
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(pre_lag_deg=0.0))))


def test_precone_geometry_same_cone_on_mirrored_blades(base):
    m0, m = rotor.build_model(base), rotor.build_model(_cone(base, 3.0))
    assert np.array_equal(m0.fe.X0, rotor.build_model(_cone(base, 0.0)).fe.X0)
    for b0, b in zip(m0.blades, m.blades):
        tip = b.chain_nodes[-1]
        rel, rel0 = b.Rk.T @ (m.fe.X0[tip] - b.hinge_point), b0.Rk.T @ (m0.fe.X0[tip] - b0.hinge_point)
        assert np.degrees(np.arctan2(rel[2], rel[0])) == pytest.approx(3.0, abs=1e-9)   # tip up (+z) on both
        assert np.linalg.norm(rel) == pytest.approx(np.linalg.norm(rel0), rel=1e-12)
        for ids in b.member_elems.values():   # flexure itself not moved
            assert np.allclose(m.fe.X0[m.fe.elements[ids[0]].n0], m0.fe.X0[m0.fe.elements[ids[0]].n0])


def test_precone_unloads_hover_flap_moment(base):
    _, ss0, my0 = _my(base)
    _, ss00, my00 = _my(_cone(base, 0.0))
    assert np.allclose(my0, my00, rtol=0, atol=1e-12) and ss0.rpm == ss00.rpm
    _, _, my1 = _my(_cone(base, 1.0))
    slope = (my1.mean() - my0.mean()) / 1.0
    assert slope < 0                                  # CF on a coned blade pushes the flap moment down
    b_star = -my0.mean() / slope
    _, _, ms = _my(_cone(base, b_star))
    assert abs(ms.mean()) < 0.3 * abs(my0.mean())
