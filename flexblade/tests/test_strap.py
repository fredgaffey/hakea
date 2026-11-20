"""Tension-torsion strap: strap materials, cable members, axial release, strap checks."""
import numpy as np
import pytest

from flexblade import fact, presets
from flexblade.materials import STRAP_MATERIALS, get_material
from flexblade.rotor import build_model, flexure_stiffness, strap_anchor_capacity


def _design(**kw):
    d = presets.make_preset(5.0, "cf_flexure")
    u = dict(topology="skewed_cruciform", thickness=0.0005, width=0.02, bar_length=0.02,
             cruciform_rod_diameter=0.0, strap_strands=4, strap_strand_diameter=0.0006,
             strap_radius=0.0007, axial_release=True)
    f = d.flexure.model_copy(update={**u, **kw})
    return d.model_copy(update=dict(flexure=f))


def test_strap_materials():
    for n in STRAP_MATERIALS:
        m = get_material(n)
        assert m.E1 > 50e9 and m.Xt > 1e9
    # UHMWPE is creep-limited: smallest sustained strain allowable of the three
    assert get_material("uhmwpe_braid").creep_strain_allow < min(
        get_material(n).creep_strain_allow for n in ("aramid_yarn", "cf_tow"))


def test_strap_members_radial_through_hinge():
    d = _design()
    mem = fact.generate_members(d)
    cab = [m for m in mem if m.cable]
    assert len(cab) == 4
    H0, _ = fact.hinge_axis(d)
    for m in cab:
        v = np.subtract(m.p1, m.p0)
        assert abs(v[1]) < 1e-12 and abs(v[2]) < 1e-12           # radial
        off = np.hypot(m.p0[1] - H0[1], m.p0[2] - H0[2])
        assert off == pytest.approx(0.0007)
        assert m.p0[0] == pytest.approx(d.flexure.hub_radius)


def test_cable_section_and_release_stiffness():
    d = _design()
    model = build_model(d)
    cab = [e for e in model.fe.elements if e.blade == 1 and e.tag.endswith(("flex2", "flex3"))]
    s = cab[0].sections[0]
    A = np.pi * 0.0006 ** 2 / 4
    assert s.EA == pytest.approx(get_material("aramid_yarn").E1 * A, rel=0.05)
    assert s.GJ < 1e-3 * s.EA * (0.0006 ** 2) / 8 * 1.01                # bend factor applied
    K = flexure_stiffness(model)
    # strap alone carries radial load: radial stiffness ≈ n EA / L
    L = max(max(m.p0[0], m.p1[0]) for m in fact.generate_members(d)) - d.flexure.hub_radius
    assert K[0, 0] == pytest.approx(4 * s.EA / L, rel=0.2)
    # no strap, released cruciform: ~no radial stiffness
    d0 = _design(strap_strands=0)
    K0 = flexure_stiffness(build_model(d0))
    assert K0[0, 0] < 1e-3 * K[0, 0]


def test_anchor_capacity_ordering():
    f = _design().flexure
    pin, _ = strap_anchor_capacity(f)
    knot, _ = strap_anchor_capacity(f.model_copy(update=dict(strap_anchor="knot")))
    P = get_material("aramid_yarn").Xt * np.pi * 0.0006 ** 2 / 4
    assert knot < pin < P
    pot_u, _ = strap_anchor_capacity(f.model_copy(update=dict(strap_anchor="epoxy_pot",
                                                              strap_material="uhmwpe_braid")))
    pot_a, _ = strap_anchor_capacity(f.model_copy(update=dict(strap_anchor="epoxy_pot")))
    assert pot_u < pot_a                                       # UHMWPE bonds poorly
