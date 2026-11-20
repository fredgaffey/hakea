import numpy as np
import pytest

from flexblade import presets, rotor
from flexblade.beam import assemble, assemble_Kmat, euler_load
from flexblade.schema import (AnalysisOptions, Airfoil, Blade, BladeStation, Design, Flexure, Layup,
                              OperatingPoint)

R = 0.127
E_HINGE = 0.15
SKEW = 30.0


def hinge_design(**an):
    return Design(
        blade=Blade(diameter_in=10, root_cutout_frac=E_HINGE, airfoil=Airfoil(naca="0012"),
                    stations=[BladeStation(r_frac=E_HINGE, chord=0.02, twist_deg=0),
                              BladeStation(r_frac=1, chord=0.02, twist_deg=0)],
                    layup=Layup(material="steel_302_spring"), elastic_axis_chord_frac=0.42),
        flexure=Flexure(hinge_radius=E_HINGE * R, skew_deg=SKEW),
        op=OperatingPoint(rpm=3000, drive="torque"),
        analysis=AnalysisOptions(aero="none", n_blade_elements=12, **an))


@pytest.fixture(scope="module")
def hinge():
    d = hinge_design()
    m = rotor.build_model(d, ideal_hinge=dict(k_hinge=1e-6, k_rot=1e5, k_trans=1e9))
    ss = rotor.static_trim(m, None)
    assert ss.converged
    return m, ss


def test_a_pin_hinge_lag_frequency(hinge):
    """Rigid uniform blade on an ideal skewed pin hinge (hub held): ν_ζ/Ω = √(3e/2(1−e))."""
    m, ss = hinge
    m.fe.fix(m.hub, [5])
    try:
        md = rotor.modes(m, ss, None, 4)
    finally:
        m.fe.fixed.discard(6 * m.hub + 5)
    nu = md.freqs_hz[0] / (ss.Omega / (2 * np.pi))
    assert "lag" in md.labels[0]
    assert nu == pytest.approx(np.sqrt(3 * E_HINGE / (2 * (1 - E_HINGE))), rel=2e-3)


def test_a_pitch_lag_coupling(hinge):
    m, ss = hinge
    hd = rotor.harmonic_1p(m, ss, None, unit_torque=True)
    t = np.tan(np.radians(SKEW))
    a1, a2 = hd.blade_angles
    assert np.real(a1["pitch_root"] / a1["lag"]) == pytest.approx(t, rel=1e-4)
    assert np.real(a2["pitch_root"] / a2["lag"]) == pytest.approx(-t, rel=1e-4)


def test_d_hub_acceleration_signs(hinge):
    """Hub accelerating (+z) → both blades lag (ζ > 0); blade 1 pitches up, blade 2 down."""
    m, ss = hinge
    fe = m.fe
    free = [i for i in fe.free_dofs() if i != 6 * m.hub + 5]
    sa = assemble(fe, ss.state, ss.Omega, True)
    K = (assemble_Kmat(fe, sa) + sa.Kgeo)[np.ix_(free, free)]
    F = euler_load(fe, ss.state, 1000.0)
    u = np.zeros(fe.ndof)
    u[free] = np.linalg.solve(K, F[free])
    (u1, r1), (u2, r2) = [rotor.blade_local_motion_c(m, u, bi) for bi in m.blades]
    assert -r1[0, 2] > 0 and -r2[0, 2] > 0
    assert r1[0, 0] > 0 > r2[0, 0]
    assert r1[0, 0] / -r1[0, 2] == pytest.approx(np.tan(np.radians(SKEW)), rel=1e-3)


def test_flexure_stiffness_and_evaluate_smoke():
    d = presets.all_presets()["hakea_5in_hinge_equiv"].model_copy(deep=True)
    d.analysis.n_blade_elements = 8
    d.analysis.n_flexure_elements = 3
    res = rotor.evaluate(d)
    assert res.trim is not None and res.trim.rpm > 1000
    K6 = np.array(res.flexure_stiffness)
    assert K6.shape == (6, 6) and np.allclose(K6, K6.T, rtol=1e-6, atol=1e-9 * np.abs(K6).max())
    h = res.harmonic
    assert h is not None and len(h.blades) == 2
    # opposite-skew blades: pitch responses in antiphase
    dp = abs(((h.blades[0].pitch_phase_deg - h.blades[1].pitch_phase_deg) + 180) % 360 - 180)
    assert dp > 150
    assert h.hub_moment_mean > 0 and h.control_ratio > 0
    assert res.checks and "max_utilisation" in res.summary
    assert len(h.nodes) == len(h.displacement_re)
    # aero-only mean moment (aero.mean_hub_moment convention) ≈ structural hub moment
    assert res.summary["aero_moment_mean"] == pytest.approx(h.hub_moment_mean, rel=0.3)


def test_dynamic_inflow_reduces_moment():
    d = presets.all_presets()["hakea_5in_hinge_equiv"].model_copy(deep=True)
    d.analysis.n_blade_elements = 8
    d.analysis.n_flexure_elements = 3
    m = rotor.build_model(d)
    aero = rotor.make_aero(m)
    ss = rotor.static_trim(m, aero)
    lin = rotor.linearize(m, ss)
    on = rotor.harmonic_1p(m, ss, aero, lin=lin).result.hub_moment_mean
    d.analysis.dynamic_inflow = False
    off = rotor.harmonic_1p(m, ss, aero, lin=lin).result.hub_moment_mean
    assert on < off
