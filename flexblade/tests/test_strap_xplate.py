"""Strap + axially released crossed plates (concept tt_strap_xplate)."""
import numpy as np
import pytest

from flexblade import fact, presets
from flexblade.rotor import build_model, flexure_stiffness


def _design(**kw):
    d = presets.make_preset(5.0, "cf_flexure")
    u = dict(topology="skewed_cross_blade", thickness=0.0005, width=0.02, length=0.016,
             cross_angle_deg=90.0, cross_gap=0.0012, strap_strands=1, strap_strand_diameter=0.0007,
             strap_radius=0.0, strap_length=0.004, axial_release=True)
    f = d.flexure.model_copy(update={**u, **kw})
    return d.model_copy(update=dict(flexure=f))


def test_strand_on_crossing_line_through_gap():
    d = _design()
    mem = fact.generate_members(d)
    cab = [m for m in mem if m.cable]
    plates = [m for m in mem if not m.cable]
    assert len(cab) == 1 and len(plates) == 2
    H0, a = fact.hinge_axis(d)
    p0, p1 = np.array(cab[0].p0), np.array(cab[0].p1)
    assert np.allclose(0.5 * (p0 + p1), H0)                       # centred on the hinge point
    assert np.linalg.norm(p1 - p0) == pytest.approx(0.004)
    for m in plates:                                              # plates clear of the strand
        c = 0.5 * (np.array(m.p0) + np.array(m.p1))
        assert abs((c - H0) @ a) - 0.5 * m.width >= 0.5 * 0.0007


def test_thin_crossed_plates_unloaded_ratio_scales_with_aspect():
    # unloaded cross-axis pivot: rotation about a is plate bending (∝ t³), the cross rotation
    # loads the plates in-plane (∝ w³) → k_cross/k_hinge grows ~ (w/t)²
    def ratio(t):
        d = _design(strap_strands=0, thickness=t)
        K = flexure_stiffness(build_model(d))
        _, a = fact.hinge_axis(d)
        b = np.cross(a, [0.0, 1.0, 0.0])
        C = np.linalg.inv(K)[3:, 3:]
        return (b @ C @ b) ** -1 / (a @ C @ a) ** -1
    r_thick, r_thin = ratio(0.0008), ratio(0.0004)
    assert r_thin > 300
    assert r_thin > 2.0 * r_thick
