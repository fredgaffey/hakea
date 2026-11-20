"""FEA validation: cantilever stiffness / frequency vs closed form, orthotropy."""
import math

import numpy as np
import pytest

from flexblade import fea
from flexblade.schema import PrintOrientation


def _cantilever(E=2e9, nu=0.3, rho=1200.0, L=0.1, b=0.01, t=0.005, n=(24, 3, 2)):
    pts, tets = fea.box_mesh(L, b, t, *n)
    m = fea.model_from_arrays(pts, tets, C=fea.isotropic_C(E, nu), rho=rho)
    fixed = m.doflocs[:, 0] < 1e-12
    T, _ = fea._transform(m, fixed)
    return m, T


def test_cantilever_tip_stiffness():
    E, nu, L, b, t = 2e9, 0.3, 0.1, 0.01, 0.005
    m, T = _cantilever(E, nu, L=L, b=b, t=t)
    K = m.stiffness()
    tip = m.doflocs[:, 0] > L - 1e-12
    f = np.zeros(m.ndof)
    P = 1.0
    f[3 * np.where(tip)[0] + 2] = P / tip.sum()   # (lumped) shear at tip, z direction
    u = T @ fea._solve((T.T @ K @ T).tocsc(), T.T @ f)
    w = u[3 * np.where(tip)[0] + 2].mean()
    I = b * t ** 3 / 12
    G = E / (2 * (1 + nu))
    w_ref = P * L ** 3 / (3 * E * I) + P * L / (5 / 6 * G * b * t)   # Timoshenko
    assert w == pytest.approx(w_ref, rel=0.03)


def test_cantilever_first_frequency():
    E, nu, rho, L, b, t = 2e9, 0.3, 1200.0, 0.1, 0.01, 0.005
    m, T = _cantilever(E, nu, rho, L, b, t)
    import scipy.sparse.linalg as spla
    K = (T.T @ m.stiffness() @ T).tocsc()
    M = (T.T @ m.mass() @ T).tocsc()
    lam = np.sort(spla.eigsh(K, k=3, M=M, sigma=0, which="LM")[0])
    f1 = math.sqrt(lam[0]) / (2 * math.pi)
    I = t ** 3 * b / 12
    f_ref = 1.875104 ** 2 / (2 * math.pi) * math.sqrt(E * I / (rho * b * t * L ** 4))
    assert f1 == pytest.approx(f_ref, rel=0.03)


def test_mass_and_centrifugal_load():
    m, _ = _cantilever()
    assert m.total_mass() == pytest.approx(1200 * 0.1 * 0.01 * 0.005)
    om = 100.0
    f = m.centrifugal_load(om)
    # resultant = Ω² ∫ρ x dV = Ω² m x_cg
    assert f[0::3].sum() == pytest.approx(om ** 2 * m.total_mass() * 0.05, rel=1e-9)
    assert abs(f[2::3]).sum() == 0.0


@pytest.mark.parametrize("raster,expect", [((0, 1, 0), "E1"), ((1, 0, 0), "E2")])
def test_orthotropic_rotation(raster, expect):
    """Bar along y, uniaxial tension: apparent modulus = E1 if raster ∥ y else E2."""
    from flexblade.materials import get_material
    mat = get_material("pa6_cf")
    pts, tets = fea.box_mesh(0.02, 0.004, 0.004, 6, 2, 2)
    pts = pts[:, [1, 0, 2]]                      # bar along y
    o = PrintOrientation(build_dir=(0, 0, 1), raster_dir=raster)
    m = fea.model_from_arrays(pts, tets, material="pa6_cf", orientation=o)
    y = m.doflocs[:, 1]
    # roller at y=0 (uy=0), pin one corner fully to kill rigid modes
    dofs_fix = list(3 * np.where(y < 1e-12)[0] + 1)
    corner = np.where((y < 1e-12) & (m.doflocs[:, 0] < -0.004 / 2 + 1e-12)
                      & (m.doflocs[:, 2] < -0.004 / 2 + 1e-12))[0][0]
    dofs_fix += [3 * corner, 3 * corner + 2]
    corner2 = np.where((y < 1e-12) & (m.doflocs[:, 0] > 0.004 / 2 - 1e-12)
                       & (m.doflocs[:, 2] < -0.004 / 2 + 1e-12))[0][0]
    dofs_fix += [3 * corner2 + 2]
    free = np.setdiff1d(np.arange(m.ndof), dofs_fix)
    K = m.stiffness().tocsr()
    end = np.where(y > 0.02 - 1e-12)[0]
    f = np.zeros(m.ndof)
    # consistent traction: use uniform displacement instead -> reaction
    d = 1e-5
    pres = np.zeros(m.ndof)
    pres[3 * end + 1] = d
    pres_dofs = 3 * end + 1
    free = np.setdiff1d(free, pres_dofs)
    u = pres.copy()
    u[free] = fea._solve(K[free][:, free], -(K[free] @ pres))
    F = (K @ u)[pres_dofs].sum()
    E_app = F / (0.004 * 0.004) / (d / 0.02)
    E_ref = mat.E1 if expect == "E1" else mat.E2
    assert E_app == pytest.approx(E_ref, rel=0.02)


def test_stiffness_matrix_rotation_invariants():
    from flexblade.materials import get_material
    mat = get_material("pa6_cf")
    o = PrintOrientation(build_dir=(1, 0, 0), raster_dir=(0, 1, 0))
    C = mat.stiffness_matrix(o)
    S = np.linalg.inv(C)
    assert 1 / S[1, 1] == pytest.approx(mat.E1, rel=1e-9)
    assert 1 / S[0, 0] == pytest.approx(mat.E3, rel=1e-9)
