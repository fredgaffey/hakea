"""Strip lateral-torsional buckling (rotor.strip_ltb_factor): a clamped thin strip under uniform
in-plane moment must buckle near the classical M_cr = 2 pi/L sqrt(EIy GJ) (fixed ends, L_e = L/2),
and a moment falling to zero along the strip must buckle at a higher peak moment."""
import numpy as np

from flexblade.beam import Element, FEModel, State
from flexblade.rotor import strip_ltb_factor
from flexblade.schema import Layup
from flexblade.section import RectShape, section_properties

L, N = 0.025, 6


def _strip_state(kappa_of_s):
    """Strip along x, width along y (in-plane), state = in-plane bending arc with curvature kappa(s)."""
    sec = section_properties(RectShape(0.009, 0.0002), Layup(material="steel_1095_shim"), None)
    m = FEModel()
    xs = np.linspace(0, L, N + 1)
    for s in xs:
        m.add_node([s, 0, 0])
    for i in range(N):
        m.add_element(Element(i, i + 1, np.array([0, 1.0, 0]), [sec]))
    m.prepare()
    ss = np.linspace(0, L, 40 * N + 1)
    th = np.concatenate([[0.0], np.cumsum(0.5 * (kappa_of_s(ss[1:]) + kappa_of_s(ss[:-1])) * np.diff(ss))])
    x = np.concatenate([[0.0], np.cumsum(np.cos(th[:-1] / 2 + th[1:] / 2) * np.diff(ss))])
    y = np.concatenate([[0.0], np.cumsum(np.sin(th[:-1] / 2 + th[1:] / 2) * np.diff(ss))])
    k = np.searchsorted(ss, xs - 1e-12)
    X = np.c_[x[k], y[k], np.zeros(N + 1)]
    R = np.array([[[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]] for t in th[k]])
    return m, State(X, R), sec


def test_uniform_moment_matches_classical_mcr():
    m, _, sec = _strip_state(lambda s: 0 * s)
    mcr = 2 * np.pi / L * np.sqrt(sec.EIy * sec.GJ)
    M = 0.3 * mcr
    m, st, _ = _strip_state(lambda s: M / sec.EIz + 0 * s)
    a = strip_ltb_factor(m, st, m.elements)
    assert 0.95 < a * M / mcr < 1.2, a * M / mcr          # 6 elements: ~ +9 %


def test_moment_gradient_raises_mcr_and_unstressed_is_stable():
    m, st0, sec = _strip_state(lambda s: 0 * s)
    assert strip_ltb_factor(m, st0, m.elements) > 1e3
    mcr = 2 * np.pi / L * np.sqrt(sec.EIy * sec.GJ)
    M = 0.3 * mcr
    m, st, _ = _strip_state(lambda s: M / sec.EIz * (1 - s / L))
    a = strip_ltb_factor(m, st, m.elements)
    assert a * M / mcr > 1.4
