"""Section-force recovery along a thin strip carrying a large in-plane (strong-axis) moment.

Cantilever strip, tip moment only: the spatial moment is the same vector at every cross-section,
so the recovered section moments must (i) match across every interior node and (ii) have norm
|M_tip|. The legacy CR-frame recovery K12·d7 (log-rotation conjugates) leaks ~½θ×M3 into the soft
out-of-plane M2 and jumps at nodes; the spatial recovery (st given) must not.
"""
import numpy as np

from flexblade.beam import Element, FEModel, assemble, element_end_forces, solve_static
from flexblade.schema import Layup
from flexblade.section import RectShape, section_properties


def _strip_tip_moment(Mtip, n=6, L=0.025):
    sec = section_properties(RectShape(0.009, 0.0002), Layup(material="steel_1095_shim"), None)
    m = FEModel()
    for i in range(n + 1):
        m.add_node([i * L / n, 0, 0])
    for i in range(n):
        m.add_element(Element(i, i + 1, np.array([0, 1.0, 0]), [sec]))
    m.fix(0)
    m.prepare()
    F = np.zeros(m.ndof)
    F[6 * n + 3: 6 * n + 6] = Mtip
    st, info = solve_static(m, 0.0, F, n_steps=4)
    assert info.converged, info.message
    return m, st, assemble(m, st, 0.0, want_tangent=False)


def test_strip_moment_recovery_continuous_and_spatial():
    Mtip = np.array([0.004, 0.008, 0.05])          # torsion, out-of-plane, in-plane (N·m)
    m, st, sa = _strip_tip_moment(Mtip)
    new = [element_end_forces(e, c, st) for e, c in zip(m.elements, sa.cr)]
    old = [element_end_forces(e, c) for e, c in zip(m.elements, sa.cr)]
    for k in range(len(m.elements)):
        assert abs(np.linalg.norm(new[k][0][3:]) - np.linalg.norm(Mtip)) < 1e-3 * np.linalg.norm(Mtip)
        assert abs(np.linalg.norm(new[k][1][3:]) - np.linalg.norm(Mtip)) < 1e-3 * np.linalg.norm(Mtip)
    jn = max(np.abs(new[k][1][3:] - new[k + 1][0][3:]).max() for k in range(len(m.elements) - 1))
    jo = max(np.abs(old[k][1][3:] - old[k + 1][0][3:]).max() for k in range(len(m.elements) - 1))
    assert jn < 2e-3 * np.linalg.norm(Mtip)
    assert jo > 10 * jn                              # legacy recovery jumps at nodes
    # clamp: section moment equals the applied tip moment resolved in the (fixed) root frame
    assert np.allclose(new[0][0][3:], Mtip, rtol=0, atol=2e-3 * np.linalg.norm(Mtip))
