"""flexure_stiffness under preload: the 6×6 transported from the blade master node to the
hinge point must include the second-order rigid-offset term, else a tension T acting a
distance e outboard of the hinge shows a spurious pendulum stiffness T·e."""
import numpy as np
import pytest

from flexblade import rotor
from flexblade.beam import Element, FEModel, assemble_internal, assemble_Kmat, solve_static


def _pin_link(e, T=100.0, k=0.05, hr=0.0135):
    fe = FEModel()
    hub = fe.add_node([0, 0, 0])
    fe.fix(hub)
    A = fe.add_node([hr, 0, 0])
    B = fe.add_node([hr, 0, 0])
    stiff = rotor.stiff_section(1e8, 1e3, 1e3)
    fe.add_element(Element(hub, A, np.array([0, 0, 1.0]), [stiff], kind="rigid", massless=True))
    fe.add_element(Element(A, B, np.zeros(3), [], kind="spring",
                           spring_K=np.diag([1e8, 1e8, 1e8, 1e4, 1e4, k]), spring_frame=np.eye(3)))
    M = B
    if e > 0:
        M = fe.add_node([hr + e, 0, 0])
        fe.add_element(Element(B, M, np.array([0, 0, 1.0]), [stiff], kind="rigid", massless=True))
    fe.prepare()
    f = np.zeros(fe.ndof)
    f[6 * M] = T
    st, _ = solve_static(fe, 0.0, f)
    sa = assemble_internal(fe, st, True)
    K = assemble_Kmat(fe, sa) + sa.Kgeo
    hp = np.array([hr, 0, 0])
    fm = sa.fint[6 * M:6 * M + 6]
    old = rotor.condense_to_point(fe, K, M, [hub], st.x[M], hp)[5, 5]
    new = rotor.condense_to_point(fe, K, M, [hub], st.x[M], hp, fm)[5, 5]
    return old, new


@pytest.mark.parametrize("e", [0.0, 0.002, 0.005])
def test_pin_plus_link_reports_pin_stiffness(e):
    old, new = _pin_link(e)
    assert new == pytest.approx(0.05, rel=0.01)            # consistent: the pin only
    assert old - 0.05 == pytest.approx(100.0 * e, abs=2e-3)  # preload-free transport: + T·e
