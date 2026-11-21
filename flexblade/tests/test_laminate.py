"""I28 laminated (unbonded) shim-stack strips: stiffness, per-lamina strain, friction zeta_eq."""
import math

import numpy as np
import pytest

from flexblade import laminate as lam
from flexblade.section import RectShape, section_properties
from flexblade.schema import Layup, PrintOrientation


def _sec(w, t):
    lay = Layup(kind="solid", material="steel_1095_shim")
    ori = PrintOrientation(build_dir=(0.0, 0.0, 1.0), raster_dir=(1.0, 0.0, 0.0))
    return section_properties(RectShape(w, t), lay, ori, np.array([1.0, 0, 0]),
                              axes=(np.array([0, 1.0, 0]), np.array([0, 0, 1.0])))


@pytest.mark.parametrize("n", [2, 3, 4])
def test_stiffness_unbonded_vs_bonded(n):
    w, t = 7e-3, 0.08e-3
    s1, sb = _sec(w, t), _sec(w, n * t)
    sl = lam.laminate_section(s1, n)
    assert sl.EIy / sb.EIy == pytest.approx(lam.bending_ratio_unbonded(n), rel=1e-9)
    assert sl.EIy / sb.EIy == pytest.approx(1 / n**2, rel=1e-9)
    # axial / in-plane / mass carry the full n t
    assert sl.EA == pytest.approx(sb.EA, rel=1e-9)
    assert sl.EIz == pytest.approx(sb.EIz, rel=1e-9)
    assert sl.mass_per_len == pytest.approx(sb.mass_per_len, rel=1e-9)
    assert lam.laminate_section(s1, 1) is s1


def test_strain_per_lamina():
    n, w, t, M = 3, 7e-3, 0.08e-3, 1e-3
    sl = lam.laminate_section(_sec(w, t), n)
    kap = M / sl.EIy
    eps_fibre = max(abs(fb.z) for fb in sl.fibres) * kap
    assert eps_fibre == pytest.approx(lam.lamina_strain(kap, t), rel=1e-9)
    assert lam.lamina_strain(kap, t) == pytest.approx(t * kap / 2)
    # same stiffness as a single strip of t1: t_lam = t1 / n^(1/3), strain lower by n^(1/3)
    t1 = 0.15e-3
    tl = t1 / n ** (1 / 3)
    assert n * tl**3 == pytest.approx(t1**3)
    assert lam.lamina_strain(10.0, tl) / lam.lamina_strain(10.0, t1) == pytest.approx(n ** (-1 / 3))


def test_zeta_eq_formula():
    assert lam.zeta_eq(2 * math.pi * 2 * 1.0, 1.0) == pytest.approx(1.0)
    n, E, w, t, L, k, mu, N, a = 3, 200e9, 7e-3, 0.08e-3, 0.015, 40.0, 0.15, 2.0, 0.5e-3
    Ed = lam.friction_energy(n, t, k, mu, N, a)
    assert Ed == pytest.approx((n - 1) * 2 * 4 * mu * N * (t * k * a / 2))
    U = lam.strain_energy(n, E, w, t, L, k)
    assert U == pytest.approx(0.5 * n * E * w * t**3 / 12 * k**2 * L)
    eta = lam.friction_eta(n, E, w, t, L, k, mu, N, a)
    assert eta == pytest.approx(2 * Ed / (4 * math.pi * U))
    # Coulomb: zeta ~ 1/amplitude; no laminae -> no friction
    assert lam.friction_eta(n, E, w, t, L, 2 * k, mu, N, a) == pytest.approx(eta / 2)
    assert lam.friction_eta(1, E, w, t, L, k) == 0.0
