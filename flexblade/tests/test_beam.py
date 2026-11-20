import numpy as np
import pytest
import scipy.linalg as sl

from flexblade.beam import Element, FEModel, assemble, assemble_Kmat, euler_load, solve_static
from flexblade.schema import Layup
from flexblade.section import RectShape, section_properties

SEC = section_properties(RectShape(0.01, 0.002), Layup(material="steel_302_spring"), None)


def cantilever(n=10, L=0.1, sec=SEC):
    m = FEModel()
    for i in range(n + 1):
        m.add_node([i * L / n, 0, 0])
    for i in range(n):
        m.add_element(Element(i, i + 1, np.array([0, 1.0, 0]), [sec]))
    m.fix(0)
    m.prepare()
    return m


def test_cantilever_static_and_modal():
    m = cantilever()
    f = m.free_dofs()
    sa = assemble(m, m.initial_state(), 0.0, True, True)
    K = assemble_Kmat(m, sa)
    F = np.zeros(m.ndof)
    F[6 * 10 + 2] = 1.0
    u = np.linalg.solve(K[np.ix_(f, f)], F[f])
    assert u[list(f).index(62)] == pytest.approx(0.1**3 / (3 * SEC.EIy), rel=2e-3)   # incl. shear
    w2 = sl.eigh(K[np.ix_(f, f)], sa.M[np.ix_(f, f)], eigvals_only=True)
    f1 = np.sqrt(w2[0]) / (2 * np.pi)
    ref = 1.8751**2 * np.sqrt(SEC.EIy / SEC.mass_per_len / 0.1**4) / (2 * np.pi)
    assert f1 == pytest.approx(ref, rel=1e-3)


def _flap_reference(n, EI, m, L, Om):
    """Independent Euler–Bernoulli FE with exact centrifugal N(x) (no spin softening in flap)."""
    l = L / n
    N = 2 * (n + 1)
    K = np.zeros((N, N))
    M = np.zeros((N, N))
    for i in range(n):
        x0 = i * l
        ke = EI / l**3 * np.array([[12, 6*l, -12, 6*l], [6*l, 4*l*l, -6*l, 2*l*l],
                                   [-12, -6*l, 12, -6*l], [6*l, 2*l*l, -6*l, 4*l*l]])
        me = m * l / 420 * np.array([[156, 22*l, 54, -13*l], [22*l, 4*l*l, 13*l, -3*l*l],
                                     [54, 13*l, 156, -22*l], [-13*l, -3*l*l, -22*l, 4*l*l]])
        kg = np.zeros((4, 4))
        for xi, w in zip(*np.polynomial.legendre.leggauss(5)):
            s = (xi + 1) / 2
            Nx = m * Om**2 * (L**2 - (x0 + s * l)**2) / 2
            dH = np.array([-6*s + 6*s*s, l*(1 - 4*s + 3*s*s), 6*s - 6*s*s, l*(-2*s + 3*s*s)]) / l
            kg += w / 2 * l * Nx * np.outer(dH, dH)
        K[2*i:2*i+4, 2*i:2*i+4] += ke + kg
        M[2*i:2*i+4, 2*i:2*i+4] += me
    return np.sqrt(sl.eigh(K[2:, 2:], M[2:, 2:], eigvals_only=True)[0]) / Om


@pytest.mark.parametrize("ratio", [0.2, 2.0])
def test_rotating_cantilever_southwell(ratio):
    """Flap of a rotating cantilever (hub at the axis): CR + centrifugal potential vs an
    independent FE; at low speed also vs Southwell ν² = ν0² + 1.193 Ω²."""
    m = cantilever()
    f = m.free_dofs()
    w0 = 1.8751**2 * np.sqrt(SEC.EIy / SEC.mass_per_len / 0.1**4)
    Om = w0 / ratio
    st, info = solve_static(m, Om)
    assert info.converged
    sa = assemble(m, st, Om, True, True)
    lam = sl.eigh((assemble_Kmat(m, sa) + sa.Kgeo)[np.ix_(f, f)], sa.M[np.ix_(f, f)], eigvals_only=True)
    nus = np.sqrt(np.abs(lam)) / Om
    ref = _flap_reference(40, SEC.EIy, SEC.mass_per_len, 0.1, Om)
    assert min(abs(nus - ref)) / ref < 2e-3
    if ratio >= 2:   # Southwell coefficient 1.193 is the low-speed (ν0 ≫ Ω) value
        south = np.sqrt(ratio**2 + 1.193)
        assert min(abs(nus - south)) / south < 0.003


def test_rigid_spin_is_stiffness_free_and_euler_sign():
    """Two-blade beam star with a free hub rotation: at centrifugal equilibrium the rigid
    rotation about z has (numerically) zero stiffness; the Euler load of a hub
    acceleration pushes blade tips toward −y (lag)."""
    m = FEModel()
    hub = m.add_node([0, 0, 0])
    nodes = {}
    for s in (1, -1):
        prev = hub
        for i in range(1, 9):
            n = m.add_node([s * 0.01 * i, 0, 0])
            m.add_element(Element(prev, n, np.array([0, 1.0, 0]), [SEC]))
            prev = n
        nodes[s] = prev
    m.fix(hub, range(5))
    m.prepare()
    Om = 500.0
    st, info = solve_static(m, Om, extra_fixed=(5,))
    assert info.converged
    sa = assemble(m, st, Om, True, True)
    K = assemble_Kmat(m, sa) + sa.Kgeo
    v = np.zeros(m.ndof)
    for i, x in enumerate(st.x):
        v[6*i:6*i+3] = np.cross([0, 0, 1.0], x)
        v[6*i+3:6*i+6] = [0, 0, 1.0]
    v[:5] = 0
    k_spin = v @ K @ v
    k_bend = (v * (np.arange(m.ndof) % 6 == 5)) @ assemble_Kmat(m, sa) @ (v * (np.arange(m.ndof) % 6 == 5))
    assert abs(k_spin) < 1e-6 * (abs(v @ sa.Kgeo @ v) + k_bend + 1.0) * 1e3
    F = euler_load(m, st, 100.0)
    assert F[6 * nodes[1] + 1] < 0 and F[6 * nodes[-1] + 1] > 0
