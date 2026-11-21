"""steelx fitting mass/inertia (fittings.fitting_mass_props) and its lumping in rotor.build_model."""
import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("manifold3d")

from flexblade import fittings as FT, rotor  # noqa: E402
from flexblade.schema import Design  # noqa: E402

V4 = Path(__file__).resolve().parents[1] / "study/designs/7in_tt_strap_steelx_v4.json"


def _fl(d, **kw):
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=kw)))


@pytest.fixture(scope="module")
def design():
    return Design.model_validate(json.loads(V4.read_text()))


def test_mass_properties_box():
    import manifold3d as mf
    v, c, C = FT.mass_properties(mf.Manifold.cube((2.0, 4.0, 6.0)).translate((1.0, 0.0, 0.0)))
    assert v == pytest.approx(48.0)
    assert np.allclose(c, [2.0, 2.0, 3.0])
    assert np.allclose(np.diag(C), 48.0 * np.array([4.0, 16.0, 36.0]) / 12.0)


def test_fitting_props_and_slim(design):
    mp = FT.fitting_mass_props(design)
    assert 3.0e-3 < mp["hub"][0] < 3.8e-3 and 3.0e-3 < mp["root"][0] < 3.9e-3      # PA6-CF, ~3.4 g each
    for m, c, I in mp.values():
        lam = np.linalg.eigvalsh(I)
        assert lam.min() > 0 and lam.max() <= lam.sum() - lam.max() + 1e-15     # triangle inequality
    ms = FT.fitting_mass_props(_fl(design, fitting_slim=True))
    assert ms["hub"][0] < 0.7 * mp["hub"][0] and ms["root"][0] < 0.7 * mp["root"][0]
    f = FT.build_strip_fittings(_fl(design, fitting_slim=True))
    assert len(f.hub.decompose()) == 1 and len(f.blade.decompose()) == 1


def test_model_lumps_fittings(design):
    def tot(m):
        mp = sum(e.mpts[:, 3].sum() for e in m.fe.elements if len(e.mpts))
        jz = sum(Mn[5, 5] for n, Mn in m.fe.lumped if n == m.hub)
        return mp, jz
    m0, j0 = tot(rotor.build_model(_fl(design, fitting_mass="none")))
    m1, j1 = tot(rotor.build_model(design))
    mp = FT.fitting_mass_props(design)
    B = design.blade.n_blades
    assert m1 - m0 == pytest.approx(B * mp["root"][0], rel=1e-9)
    m_h, c_h, I_h = mp["hub"]
    assert j1 - j0 == pytest.approx(B * (I_h[2, 2] + m_h * (c_h[0] ** 2 + c_h[1] ** 2)), rel=1e-9)
