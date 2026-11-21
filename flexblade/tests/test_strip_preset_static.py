"""I34 zero-stress trim: flexure.strip_preset_deg / Element.d7_0 initial strain."""
from pathlib import Path

import numpy as np
import pytest

from flexblade import rotor
from flexblade.beam import assemble, element_end_forces
from flexblade.schema import Design

V5 = Path(__file__).resolve().parents[1] / "study" / "designs" / "10in_tt_strap_steelx_v5.json"


@pytest.fixture(scope="module")
def hover():
    d = rotor.resolve_pre_lag(Design.model_validate_json(V5.read_text()))
    m = rotor.build_model(d)
    return d, m, rotor.static_trim(m, rotor.make_aero(m))


def test_static_preset_zero_is_legacy(hover):
    d, m, ss = hover
    z = [[[0.0] * 6 for _ in d.flexure.members or range(3)] for _ in range(d.blade.n_blades)]
    dz = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(strip_preset_deg=z))))
    mz = rotor.build_model(dz)
    a = assemble(m.fe, ss.state, ss.Omega, want_tangent=True)
    b = assemble(mz.fe, ss.state, ss.Omega, want_tangent=True)
    assert np.array_equal(a.fint, b.fint) and np.array_equal(a.Kgeo, b.Kgeo)


def test_static_preset_equal_to_trim_deflection_zero_bending(hover):
    """d7_0 = elastic hover deformation of every strip element -> no strip bending at hover,
    and the end-rotation report reads zero."""
    d, m, ss = hover
    sa = assemble(m.fe, ss.state, ss.Omega, want_tangent=False)
    flex = [i for i, e in enumerate(m.fe.elements) if e.kind == "flexure"]
    M0 = max(np.abs(np.r_[element_end_forces(m.fe.elements[i], sa.cr[i])][[4, 5, 10, 11]]).max() for i in flex)
    assert M0 > 1e-4                      # the legacy hover state does bend the strips
    m.fe._pack = None
    for i in flex:
        p = sa.cr[i].d7.copy()
        p[0] = 0.0                        # keep the axial (CF) strain
        m.fe.elements[i].d7_0 = p
    sb = assemble(m.fe, ss.state, ss.Omega, want_tangent=True)
    for i in flex:
        fa, fb = element_end_forces(m.fe.elements[i], sb.cr[i])
        assert np.abs([fa[4], fa[5], fb[4], fb[5]]).max() < 1e-9 * M0 + 1e-12
        assert abs(fa[0] - element_end_forces(m.fe.elements[i], sa.cr[i])[0][0]) < 1e-9
    r = rotor.strip_end_rotations(m, ss)
    assert max(abs(row[c]) for b in r for row in b for c in (1, 2, 4, 5)) < 1e-9
    m.fe._pack = None
    for i in flex:
        m.fe.elements[i].d7_0 = None
