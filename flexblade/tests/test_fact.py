import numpy as np
import pytest

from flexblade import fact, presets


@pytest.fixture(scope="module")
def design():
    return presets.all_presets()["hakea_5in_hinge_equiv"]


def test_screw_spaces(design):
    T = fact.freedom_space(design)
    C = fact.constraint_space(design)
    assert C.shape == (6, 5)
    assert np.allclose(T.T @ fact._DELTA @ C, 0, atol=1e-12)
    assert fact.constraint_space(design, radial=True).shape == (6, 4)
    # radial line through the hinge point and any line parallel to the axis are constraints
    p, a = fact.hinge_axis(design)
    assert abs(fact.reciprocal(T[:, 0], fact.line_wrench(p, [1, 0, 0]))) < 1e-12
    assert abs(fact.reciprocal(T[:, 0], fact.line_wrench(p + [0, 0.01, 0], a))) < 1e-12


def test_hinge_axis_sign(design):
    """Positive rotation about the returned axis = lag (rotation about −z) with
    pitch (about +x) = +tanδ·lag for blade 1, −tanδ·lag for blade 2."""
    d = np.radians(design.flexure.skew_deg)
    for k, s in ((1, 1.0), (2, -1.0)):
        _, a = fact.hinge_axis(design, k)
        lag, pitch = -a[2], a[0]
        assert lag > 0
        assert pitch / lag == pytest.approx(s * np.tan(d))


@pytest.mark.parametrize("topo,dim,min_ratio", [("skewed_cross_blade", 5, 20), ("wire_triplet", 5, 20),
                                                ("skewed_single_blade", 3, 1.5), ("integral_taper", 3, 1.5)])
def test_layouts(design, topo, dim, min_ratio):
    d = design.model_copy(deep=True)
    d.flexure.topology = topo
    if topo == "wire_triplet":
        d.flexure.thickness = 0.0006
    members = fact.generate_members(d)
    W = fact.layout_constraint_space(members)
    assert W.shape[1] == dim
    c = fact.check_layout(d, members)
    assert c.ideal_constraints_reciprocal
    assert c.axis_error_deg < 0.5
    assert c.stiffness_ratio > min_ratio


def test_mirror_member(design):
    m = fact.generate_members(design)[0]
    mm = fact.mirror_member(m)
    assert mm.p0[2] == -m.p0[2] and mm.p0[0] == m.p0[0]
