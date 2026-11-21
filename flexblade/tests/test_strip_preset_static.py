"""I34 zero-stress trim: flexure.strip_preset_deg / Element.d7_0 = pre-angled clamp slots
(end-frame rotation, beam.PRESET_MODE "slot")."""
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
        # axial: the strip is now straight between the slots -> no arc-length (bow) term; the CF
        # tension is unchanged up to EA·bow/L (small vs the bending removed being exact)
    r = rotor.strip_end_rotations(m, ss)
    assert max(abs(row[c]) for b in r for row in b for c in (1, 2, 4, 5)) < 1e-9
    m.fe._pack = None
    for i in flex:
        m.fe.elements[i].d7_0 = None


def test_slot_preset_equals_rotated_mesh():
    """A strip in a slot pre-angled by p == the same strip meshed straight along the rotated line
    (stress-free pre-shaped geometry), under the same tension: same equilibrium and same modes.
    The legacy initial-strain model (pre-curved end element) misses the axial mode by ~30 %."""
    from flexblade import beam
    from flexblade.beam import FEModel, Element, solve_static
    from flexblade.section import SectionProps
    L, ne, p, E = 0.025, 6, np.radians(8.0), 200e9
    sec = SectionProps(EA=E * 0.009 * 8e-4, EIy=E * 8 * 0.009 * 1e-12 / 12, EIz=E * 8e-4 * 0.009 ** 3 / 12,
                       EIyz=0.0, GJ=80e9 * 8 * 0.009 * 1e-12 / 3, GAy=1e9, GAz=1e9, mass_per_len=0.06,
                       Ip_mass=1e-9, Iyy_mass=5e-10, Izz_mass=5e-10, Iyz_mass=0.0, centroid=(0.0, 0.0),
                       shear_centre=(0.0, 0.0), mass_centre=(0.0, 0.0), area=0.0, method="test")
    ax = np.array([np.cos(p), 0.0, -np.sin(p)])

    def run(axis, pre):
        fe = FEModel()
        ns = [fe.add_node(axis * L * i / ne) for i in range(ne + 1)]
        for i in range(ne):
            fe.add_element(Element(ns[i], ns[i + 1], np.array([0.0, 1.0, 0.0]), [sec], kind="flexure"))
        if pre is not None:
            fe.elements[0].d7_0 = np.r_[0.0, pre]
        fe.fix(ns[0])
        fe.add_lumped_inertia(ns[-1], np.diag([0.01, 0.01, 0.01, 2e-6, 2e-6, 2e-6]))
        F = np.zeros(fe.ndof)
        F[6 * ns[-1]:6 * ns[-1] + 3] = 300.0 * ax
        out = solve_static(fe, 0.0, F, n_steps=8)
        st = out[0] if isinstance(out, tuple) else out.state
        sa = assemble(fe, st, 0.0, want_tangent=True, want_mass=True)
        K = beam.assemble_Kmat(fe, sa) + sa.Kgeo
        fr = fe.free_dofs()
        w2 = np.linalg.eigvals(np.linalg.solve(sa.M[np.ix_(fr, fr)], K[np.ix_(fr, fr)])).real
        return st.x[ns[-1]], np.sort(np.sqrt(np.abs(w2)))[:4]

    old = beam.PRESET_MODE
    try:
        beam.PRESET_MODE = "slot"
        xm, fm = run(ax, None)
        xs, fs = run(np.array([1.0, 0.0, 0.0]), [0.0, -p, 0.0, 0.0, 0.0, 0.0])
        assert np.allclose(xs, xm, atol=1e-9) and np.allclose(fs, fm, rtol=1e-6)
        beam.PRESET_MODE = "strain"
        _, fl = run(np.array([1.0, 0.0, 0.0]), [0.0, -p, 0.0, 0.0, 0.0, 0.0])
        assert abs(fl[3] / fm[3] - 1) > 0.1
    finally:
        beam.PRESET_MODE = old
