import time

import numpy as np
import pytest

from flexblade.aero import AeroModel, theodorsen, unsteady_summary
from flexblade.presets import make_preset
from flexblade.schema import Design


@pytest.fixture(scope="module")
def model():
    return AeroModel(make_preset(5, "tpu_monolithic"))


@pytest.fixture(scope="module")
def trim(model):
    return model.trim()


def test_theodorsen_values():
    C = theodorsen(np.array([0.0, 0.1, 0.5, 50.0]))
    assert C[0] == 1
    assert abs(C[1] - (0.8319 - 0.1723j)) < 2e-3
    assert abs(C[2] - (0.5979 - 0.1507j)) < 2e-3
    assert abs(C[3].real - 0.5) < 0.01


def test_hover_trim_5in(model, trim):
    assert abs(trim.T - 0.6 * 9.80665) < 1e-3
    assert 15000 < trim.rpm < 22000
    assert 0.5 < trim.FoM < 0.85
    assert trim.converged
    # signs: thrust up, drag on blade 1 toward -y, torque positive
    assert np.all(trim.Fz[1:-1] > 0)
    assert np.all(trim.Fy < 0)
    assert trim.Q > 0 and trim.P > 0
    # tip and root loss drive end loads to ~0
    assert abs(trim.Fz[-1]) < 0.02 * trim.Fz.max()
    # cambered NACA 4412: nose-down cm
    assert np.median(trim.Mx) < 0
    assert set(trim.stations_dict()) >= {"r", "alpha_deg", "cl", "cd", "Re", "dT_dr"}


def test_trim_speed(model):
    model.trim()
    t0 = time.perf_counter()
    model.trim()
    assert time.perf_counter() - t0 < 0.3


def test_trim_modes_consistent(model, trim):
    t2 = model.trim(rpm=trim.rpm)
    assert abs(t2.T - trim.T) < 1e-6
    # solve collective at higher rpm for same thrust → collective negative
    t3 = model.trim(rpm=trim.rpm * 1.1, thrust=trim.T)
    assert abs(t3.T - trim.T) < 1e-4 and t3.collective < 0
    # elastic nose-up pitch raises thrust at fixed rpm
    t4 = model.trim(rpm=trim.rpm, blade_shape={"theta": np.full_like(trim.r, np.radians(1.0))})
    assert t4.T > trim.T
    # prescribed inflow equal to the BEMT inflow reproduces loads
    t5 = model.trim(rpm=trim.rpm, inflow=trim.v_i)
    assert np.allclose(t5.Fz, trim.Fz)


def test_momentum_consistency(trim):
    # BEMT total thrust vs annulus momentum with loss factor
    dT_mom = 4 * np.pi * trim.r * trim.rho * trim.F_loss * trim.v_i**2
    assert np.allclose(dT_mom[1:-1], trim.dT_dr[1:-1], rtol=1e-4, atol=1e-3)


def _fd(model, trim, dUT=0.0, dUP=0.0, dth=0.0, h=1e-6):
    p = np.array(model.section_loads(trim.U_T + dUT * h, trim.U_P + dUP * h, trim.theta + dth * h))
    m = np.array(model.section_loads(trim.U_T - dUT * h, trim.U_P - dUP * h, trim.theta - dth * h))
    return (p - m) / (2 * h)   # (3, n)


@pytest.mark.parametrize("kind", ["quasi_steady", "theodorsen"])
def test_linearized_matches_finite_difference(model, trim, kind):
    w = 1e-6
    A = model.linearized_array(trim, w, kind)
    assert A.shape == (len(trim.r), 3, 4)
    refs = [_fd(model, trim, dUP=1) * 1j * w,   # uz: δU_P = iω uz
            _fd(model, trim, dUT=1) * 1j * w,   # uy: δU_T = iω uy
            _fd(model, trim, dth=1),            # θ
            _fd(model, trim, dUP=1)]            # dλ
    for j, ref in enumerate(refs):
        got = A[:, :, j].T
        scale = np.abs(ref).max(axis=1, keepdims=True) + 1e-30
        assert np.max(np.abs(got - ref) / scale) < 1e-4, j


def test_linearized_physics_signs(model, trim):
    A = model.linearized_array(trim, trim.omega)
    i = len(trim.r) * 2 // 3
    assert A[i, 0, 0].imag < 0          # plunge (heave) damping
    assert A[i, 1, 1].imag < 0          # lag damping (in-plane rate → more drag)
    assert A[i, 0, 2].real > 0          # nose-up pitch → more lift
    assert A[i, 0, 3].real < 0          # more downwash → less lift
    # apparent mass dominates heave at very high frequency: force = −m_a ü → Re > 0
    Ah = model.linearized_array(trim, 200 * trim.omega)
    assert Ah[i, 0, 0].real > 0
    # 'none' gives zeros
    assert np.all(model.linearized_array(trim, trim.omega, "none") == 0)


def test_pitt_peters(model, trim):
    G = model.pitt_peters_hover(trim, 0.0)
    assert G.shape == (2, 2)
    # steady gain = 1/λ0, physical signs: λs ∝ +C_Mx, λc ∝ −C_My
    assert np.isclose(G[1, 0], 1 / trim.lambda0) and np.isclose(G[0, 1], -1 / trim.lambda0)
    # extra lift on the ψ=0 side gives My<0 and more downwash at ψ=0 (λc>0)
    ci = model.cyclic_inflow(trim, 0.0, -0.01)
    assert ci.lam_c.real > 0 and abs(ci.lam_s) < 1e-12
    assert ci.field(trim.R, 0.0) > 0 and ci.field(trim.R, np.pi) < 0
    # dynamic: lag grows with frequency
    assert abs(model.pitt_peters_hover(trim, trim.omega)[1, 0]) < abs(G[1, 0])
    # rotating-frame view: blade 1 at ψ=Ωt sees max downwash at t=0, blade 2 opposite
    b1 = ci.blade_1p(trim.r, 1, 2)
    b2 = ci.blade_1p(trim.r, 2, 2)
    assert np.allclose(b1.imag, 0) and np.all(b1.real[trim.r > 0] > 0) and np.allclose(b2, -b1)
    assert model.pitt_peters_hover_full(trim).shape == (3, 3)


def test_mean_hub_moment_convention(model, trim):
    # momentum check of Pitt–Peters steady gain: the 1/rev lift that a cyclic
    # inflow would need for annulus momentum, ΔFz = 4ρ v0 Δv·(2πr)/B, maps back
    # to C_M = −λ0 λc.  Use blade loads peaking at ψ=0 (cos) → My < 0, Mx = 0.
    B = trim.n_blades
    F1 = np.ones_like(trim.r) + 0j
    F = np.stack([F1 * np.exp(1j * 2 * np.pi * k / B) for k in range(B)])
    Mx, My = model.mean_hub_moment(trim, F)
    assert abs(Mx) < 1e-12 and My < 0
    assert np.isclose(My, -0.5 * B * np.sum(trim.w_int * trim.r))


def test_unsteady_summary(model, trim):
    s = unsteady_summary(trim)
    assert np.all((s["k"] > 0) & (s["k"] < 1))
    assert 0.05 < s["k_75"] < 0.3


def test_default_design_runs():
    m = AeroModel(Design())
    t = m.trim(rpm=20000)
    assert t.T > 0
