"""Damped rotating-frame eigenvalues, air/ground resonance (stability.py)."""
import json
from pathlib import Path

import numpy as np
import pytest

from flexblade import rotor
from flexblade import stability as S
from flexblade.matsweep import _margin
from flexblade.schema import Airframe, Design, Drive

OM = 2 * np.pi * 50
B, E, RG, MB, NU = 3, 0.02, 0.4, 1.0, 0.3
DESIGN5 = Path(__file__).resolve().parents[1] / "study" / "designs" / "5in_tt_strap_steelx_hand.json"


def af_of(f, z=0.0, m=5.0, fy=None, nb=B):
    mt = m + nb * MB
    w, wy = 2 * np.pi * f, 2 * np.pi * (fy or f)
    return S.Airframe(m, mt * w * w, mt * wy * wy, 2 * z * mt * w, 2 * z * mt * wy)


def maxre_col(f, c=0.0, z=0.0):
    return max(S.coleman_roots(B, E, RG, MB, NU, OM, c, af_of(f, z)).real)


def maxre_toy(f, c=0.0, z=0.0, floquet=False):
    rs = S.articulated_toy(B, E, RG, MB, NU, OM, c)
    return max(S.coupled_eigs(rs, af_of(f, z), floquet=floquet)[0].real)


def _edge(fun, lo, hi):
    """bisect the stability boundary between a stable (lo) and unstable (hi) point"""
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if fun(mid) > 1e-9:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def test_toy_matches_coleman_roots():
    rs = S.articulated_toy(B, E, RG, MB, NU, OM, 3.0)
    s_t = S.coupled_eigs(rs, af_of(30.0, 0.02), floquet=False)[0]
    for sc in S.coleman_roots(B, E, RG, MB, NU, OM, 3.0, af_of(30.0, 0.02)):
        # fixed-frame Coleman root = rotating-frame root + iΩ
        assert np.min(np.abs(s_t + 1j * OM - sc)) < 1e-6 * abs(sc)


def test_coleman_band_and_damped_boundary():
    fs = np.linspace(5, 150, 146)
    un = fs[np.array([maxre_col(f) for f in fs]) > 1e-6]
    assert len(un) > 3          # classic undamped ground-resonance band near (1-ν)Ω coupling
    for f_in, f_out in ((un.min(), un.min() - 1.0), (un.max(), un.max() + 1.0)):
        ec = _edge(lambda f: maxre_col(f), f_out, f_in)
        et = _edge(lambda f: maxre_toy(f), f_out, f_in)
        assert abs(et - ec) < 0.01 * ec
    # damped boundary (Deutsch-type: lag damper vs hub damping) at mid-band
    f0 = 0.5 * (un.min() + un.max())
    c_lag = 2.0
    zc = _edge(lambda z: -maxre_col(f0, c_lag, z), 2.0, 0.0)
    zt = _edge(lambda z: -maxre_toy(f0, c_lag, z), 2.0, 0.0)
    assert 0 < zc < 2.0 and abs(zt - zc) < 0.02 * zc


def test_floquet_isotropic_equals_lti_and_aniso_b2():
    rs = S.articulated_toy(B, E, RG, MB, NU, OM, 3.0)
    s_l = S.coupled_eigs(rs, af_of(40.0, 0.03), floquet=False)[0]
    s_f = S.coupled_eigs(rs, af_of(40.0, 0.03), floquet=True)[0]
    assert np.max(np.abs(np.sort_complex(s_l) - np.sort_complex(s_f))) < 1e-4 * OM
    rs2 = S.articulated_toy(2, E, RG, MB, NU, OM, 0.0)
    s2 = S.coupled_eigs(rs2, af_of(30.0, 0.0, fy=36.0, nb=2), floquet=True)[0]
    assert abs(max(s2.real)) < 1e-6 * OM            # conservative periodic system: |μ| = 1


def test_two_blade_aniso_support_window_near_1p():
    """2-bladed rotor on anisotropic support: unstable when the support frequencies straddle
    Ω (f_x < Ω < f_y, 2/rev parametric), stable away from it with the same damping."""
    rs2 = S.articulated_toy(2, E, RG, MB, NU, OM, 3.0)
    f1 = OM / (2 * np.pi)
    s_in = S.coupled_eigs(rs2, af_of(0.96 * f1, 0.02, fy=1.1 * f1, nb=2), floquet=True)[0]
    s_out = S.coupled_eigs(rs2, af_of(0.7 * f1, 0.02, fy=0.8 * f1, nb=2), floquet=True)[0]
    assert max(s_in.real) > 0.0 > max(s_out.real)


def test_margin_1p_eff():
    assert S.margin_1p_eff(1.15, 0.0) == pytest.approx(0.15, rel=1e-6)
    assert S.margin_1p_eff(0.9, 0.0) > 0.1
    assert S.margin_1p_eff(0.95, 0.2) > S.margin_1p_eff(0.95, 0.02)
    assert _margin(0.95, 0.3) > _margin(0.95) == pytest.approx(0.05)


@pytest.fixture(scope="module")
def d5():
    d = Design.model_validate(json.loads(DESIGN5.read_text()))
    return rotor.resolve_pre_lag(d) if d.flexure.pre_lag_deg == "auto" else d


def test_fe_hub_translation_is_consistent(d5):
    m = rotor.build_model(d5)
    ss = rotor.static_trim(m, rotor.make_aero(m))
    lin = rotor.linearize(m, ss, True)
    _, Bm = S._basis(m, ss, lin, 12, hub=True)
    rs = S.reduced_system(m, ss, lin, None, Bm, ss.Omega / (2 * np.pi), hub=True)
    ih = np.ix_(rs.ih, rs.ih)
    mr = rs.m_rotor
    assert np.allclose(rs.M[ih], mr * np.eye(2), rtol=1e-6)
    assert np.allclose(rs.G[ih], 2 * ss.Omega * mr * S.J2, rtol=1e-3, atol=1e-6 * ss.Omega * mr)
    assert np.allclose(rs.K[ih], -ss.Omega ** 2 * mr * np.eye(2), rtol=1e-3, atol=1e-3 * ss.Omega ** 2 * mr)


def test_evaluate_stability_checks(d5):
    d = d5.model_copy(deep=True)
    d.op.airframe = Airframe(f_hz=150.0, f_y_hz=190.0)
    d.analysis.stability = "floquet"
    r = rotor.evaluate(d)
    names = {c.name: c for c in r.checks if c.member == "stability"}
    assert set(names) == {"aeroelastic_stability", "air_resonance"}
    assert names["aeroelastic_stability"].value > 0.0 and names["air_resonance"].value > 0.0
    assert "zeta_energy" in r.modes[1].participation
    assert 0.5 < r.summary["near_1p_per_rev"] < 1.5
    d.analysis.stability = "energy"
    r2 = rotor.evaluate(d)
    assert [c.name for c in r2.checks if c.member == "stability"] == ["aeroelastic_stability"]


# ------------------------------------------------------------------ hub tilt (soft mount)


def _tilt_only(Om, It, Jp, kt, ct=0.0, mr=0.02):
    """No rotor: hub translation (mass mr) + tilt DOFs with zero rotor tilt inertia."""
    M = np.diag([mr, mr, 0.0, 0.0])
    G = np.zeros((4, 4))
    K = np.zeros((4, 4))
    G[:2, :2] = 2 * Om * mr * S.J2
    K[:2, :2] = -Om ** 2 * mr * np.eye(2)
    rs = S.RotSys(M, G, K, Om, (0, 1), None, mr, (2, 3))
    k = 0.1 * (2 * np.pi * 70) ** 2
    return rs, S.Airframe(0.1 - mr, k, k, 0.0, 0.0, It, Jp, kt, ct)


def test_tilt_whirl_split_analytic():
    """Rotor removed: fixed-frame whirl roots of I_t ω² ∓ J_pΩ ω − k = 0."""
    Om, It, Jp, kt = 2 * np.pi * 128, 8e-5, 4e-5, 19.4
    rs, af = _tilt_only(Om, It, Jp, kt)
    s, V = S.coupled_eigs(rs, af)
    a = Jp * Om / It
    wf = 0.5 * (a + np.sqrt(a * a + 4 * kt / It))      # forward
    wb = 0.5 * (-a + np.sqrt(a * a + 4 * kt / It))     # backward
    assert wf > wb and abs(wf - wb - a) < 1e-9 * wf
    fixed = np.concatenate([s + 1j * Om, s - 1j * Om])
    for w in (wf, wb):
        assert np.min(np.abs(fixed - 1j * w)) < 1e-6 * w
        assert np.min(np.abs(fixed + 1j * w)) < 1e-6 * w
    # mount damping: both whirl modes damped (rotating frame stable)
    rs, af = _tilt_only(Om, It, Jp, kt, ct=0.15 * kt / np.sqrt(kt / It))
    assert max(S.coupled_eigs(rs, af)[0].real) < 0


def test_tilt_stiff_limit_recovers_translation_model():
    """k_t -> inf: the tilt DOFs freeze and the rotor + hub roots return to the old model."""
    rs0 = S.articulated_toy(B, E, RG, MB, NU, OM, 3.0)
    n = len(rs0.M)
    M = np.zeros((n + 2, n + 2)); G = M.copy(); K = M.copy()
    M[:n, :n], G[:n, :n], K[:n, :n] = rs0.M, rs0.G, rs0.K
    c = np.array([[0.0, 0.05], [-0.05, 0.0]])          # tilt-translation inertia coupling (offset CG)
    M[np.ix_(rs0.ih, [n, n + 1])] = c
    M[np.ix_([n, n + 1], rs0.ih)] = c.T
    M[n, n] = M[n + 1, n + 1] = 0.01
    rs = S.RotSys(M, G, K, OM, rs0.ih, None, rs0.m_rotor, (n, n + 1))
    af0 = af_of(30.0, 0.02)
    s0 = np.sort_complex(S.coupled_eigs(rs0, af0)[0])
    for kt, tol in ((1e6, 2e-2), (1e9, 2e-5)):
        af = S.Airframe(af0.m, af0.kx, af0.ky, af0.cx, af0.cy, 1e-3, 5e-4, kt, 0.1 * np.sqrt(kt * 1e-3))
        s = S.coupled_eigs(rs, af)[0]
        s = np.sort_complex(s[np.abs(s) < 10 * np.max(np.abs(s0))])
        assert len(s) == len(s0)
        assert np.max(np.abs(s - s0)) < tol * np.max(np.abs(s0))


def test_fe_hub_tilt_cf_restoring(d5):
    """Rigid flap tilt (θy, blades on ±x) in rotating axes: CF restoring ≈ Ω² × tilt inertia."""
    m = rotor.build_model(d5)
    ss = rotor.static_trim(m, rotor.make_aero(m))
    lin = rotor.linearize(m, ss, True)
    _, Bm = S._basis(m, ss, lin, 12, hub=True, tilt=True)
    rs = S.reduced_system(m, ss, lin, None, Bm, ss.Omega / (2 * np.pi), hub=True, tilt=True)
    ty = rs.it[1]
    print("tilt M", np.diag(rs.M)[list(rs.it)], "K/Om2", np.diag(rs.K)[list(rs.it)] / ss.Omega ** 2)
    assert rs.M[ty, ty] > 0 and abs(rs.K[ty, ty] / (ss.Omega ** 2 * rs.M[ty, ty]) - 1) < 0.1
    assert np.allclose(rs.M[np.ix_(rs.ih, rs.ih)], rs.m_rotor * np.eye(2), rtol=1e-6)


def _toy_rs_af(k_rot, c_rot=0.0, Omega=100.0):
    """1-DOF-per-axis toy: rotor hub (x, y) with stiffness k_rot (can be negative: divergence)."""
    n = 2
    M = np.eye(n)
    K = k_rot * np.eye(n)
    G = c_rot * np.eye(n)
    rs = S.RotSys(M=M, G=G, K=K, Omega=Omega, ih=(0, 1), m_rotor=1.0)
    af = S.Airframe(m=1e-9, kx=1e-9, ky=1e-9, cx=0.0, cy=0.0)
    return rs, af


def test_frame_zeta_real_root_is_divergence():
    rs, af = _toy_rs_af(-49.0)                    # s = +/-7 real
    s = np.array([7.0 + 0j, -7.0 + 0j])
    V = np.zeros((4, 2), complex)
    V[0, :] = 1.0
    z, div = S.frame_zeta(rs, af, s, V, return_div=True)
    assert z[0] == pytest.approx(-1.0) or z[0] < 0
    assert div[0] and div[1]
    assert z[0] < 0 < z[1]
    assert S.min_zeta(s, 100.0, z=z)[0] < 0


def test_frame_zeta_oscillatory_root_not_divergence():
    rs, af = _toy_rs_af(400.0)
    s = np.array([-1.0 + 20j, -1.0 - 20j])
    V = np.zeros((4, 2), complex)
    V[0, :] = 1.0
    z, div = S.frame_zeta(rs, af, s, V, return_div=True)       # hub-dominated => fixed-frame branch
    assert not div.any() and (z > 0).all() and (z < 1).all()
    zr, dr = S.frame_zeta(rs, af, s, V, hub_frac=2.0, return_div=True)   # rotating-frame branch
    assert not dr.any() and zr[0] == pytest.approx(1 / np.hypot(1, 20))
    zr, dr = S.frame_zeta(rs_div := _toy_rs_af(-49.0)[0], af, np.array([7.0 + 0j]), V[:, :1], hub_frac=2.0, return_div=True)
    assert dr[0] and zr[0] == pytest.approx(-1.0)


def test_rigid_spin_has_no_stiffness_with_aero(d5):
    """Hover is axisymmetric: the azimuth is cyclic, so K r = 0 for the rigid spin r.  The dead trim
    aero loads (missing follower term in K_geo) used to give r a negative stiffness ~1e-4 Omega^2 J ->
    spurious real roots s = +O(10) rad/s (fake divergence in air_resonance)."""
    m = rotor.build_model(d5)
    aero = rotor.make_aero(m)
    ss = rotor.static_trim(m, aero)
    lin = rotor.linearize(m, ss, True)
    lam, Bm = S._basis(m, ss, lin, 12, hub=False)
    rs = S.reduced_system(m, ss, lin, aero, Bm, float(np.sqrt(abs(lam[1])) / (2 * np.pi)))
    P = S._spin_projector(m, ss, lin, Bm, rs.M)
    assert P is not None
    r = np.linalg.svd(np.eye(len(P)) - P)[0][:, 0]
    assert np.linalg.norm(rs.K @ r) < 1e-9 * ss.Omega ** 2 * np.linalg.norm(rs.M @ r)
    s = np.linalg.eigvals(S.state_matrix(rs.M, rs.G, rs.K))
    assert not np.any((s.real > 1e-3 * ss.Omega) & (np.abs(s.imag) <= s.real))


def _rigid_dp(J=2e-5):
    w = 2 * np.pi * 50
    return dict(c=1e-4, kp=J * w, ki=J * w * w * 0.25, tau=1e-3, tl=2e-4, J=J)


def test_drive_rigid_rotor_pade_roots():
    """Rigid rotor J on the drive: roots of J s + Z_pade(s) = 0 (cubic) plus the inert ξ root s = 0."""
    dp = _rigid_dp()
    J, te = dp["J"], dp["tau"] + dp["tl"]
    rs = S.RotSys(np.array([[J]]), np.zeros((1, 1)), np.zeros((1, 1)), 500.0)
    r = S.add_drive(rs, np.array([1.0]), dp)
    s = np.linalg.eigvals(S.state_matrix(r.M, r.G, r.K))
    p = np.polyadd(np.polymul([J, dp["c"], 0], [te / 2, 1]), np.polymul([dp["kp"], dp["ki"]], [-te / 2, 1]))
    assert np.allclose(np.sort_complex(s[np.abs(s) > 1e-6]), np.sort_complex(np.roots(p)), rtol=1e-8)
    assert np.min(np.abs(s)) < 1e-6


def test_drive_rigid_rotor_frequency_response():
    """Augmented (θ, ξ) system response to a unit hub torque = 1/(−JΩ² + jΩ Z_pade(jΩ))."""
    dp = _rigid_dp()
    J = dp["J"]
    rs = S.RotSys(np.array([[J]]), np.zeros((1, 1)), np.zeros((1, 1)), 500.0)
    r = S.add_drive(rs, np.array([1.0]), dp)
    for w in (50.0, 700.0, 3000.0):
        x = np.linalg.solve(r.K - w * w * r.M + 1j * w * r.G, np.array([1.0, 0.0]))
        th = 1.0 / (-J * w * w + 1j * w * S.drive_impedance(dp, 1j * w, pade=True))
        assert abs(x[0] / th - 1) < 1e-10
    # exact delay reduces to Padé at low frequency
    assert abs(S.drive_impedance(dp, 10j) / S.drive_impedance(dp, 10j, pade=True) - 1) < 1e-3


def test_drive_default_free_spin_unchanged(d5):
    """motor.esc None (default) = free spin: reduced system and 1P harmonic unchanged; Drive() with no speed
    loop in duty mode leaves the harmonic unchanged too (back-EMF is already in the duty k_hub)."""
    assert d5.motor.esc is None
    m = rotor.build_model(d5)
    aero = rotor.make_aero(m)
    ss = rotor.static_trim(m, aero)
    lin = rotor.linearize(m, ss, True)
    lam, Bm = S._basis(m, ss, lin, 12, hub=False)
    f = float(np.sqrt(abs(lam[1])) / (2 * np.pi))
    a = S.reduced_system(m, ss, lin, aero, Bm, f)
    b = S.reduced_system(m, ss, lin, aero, Bm, f, drive=False)
    assert np.array_equal(a.G, b.G) and np.array_equal(a.K, b.K)
    h0 = rotor.harmonic_1p(m, ss, aero, lin=lin).result
    m.design = d5.model_copy(update=dict(motor=d5.motor.model_copy(update=dict(esc=Drive()))))
    h1 = rotor.harmonic_1p(m, ss, aero, lin=lin).result
    assert d5.op.drive == "duty"
    assert np.isclose(h1.hub_moment_mean, h0.hub_moment_mean, rtol=1e-12)
    # with Drive() the stability system gains c_emf on the spin DOF only
    c = S.reduced_system(m, ss, lin, aero, Bm, f)
    bh = Bm[6 * m.hub + 5]
    dp = S.drive_params(m.design.motor, S.spin_inertia(m, ss, lin))
    assert np.allclose(c.G - a.G, dp["c"] * np.outer(bh, bh))
    # a stiff, delay-free governor holds the hub: spin rate ≈ unit torque / kp
    J = S.spin_inertia(m, ss, lin)
    kp = 200 * J * ss.Omega
    esc = Drive(current_control=True, k_p=kp, k_i=0.0, latency_s=0.0)
    m.design = d5.model_copy(update=dict(motor=d5.motor.model_copy(update=dict(esc=esc))))
    h2 = rotor.harmonic_1p(m, ss, aero, lin=lin, unit_torque=True).result
    rate = h2.hub_speed_amp_rpm * 2 * np.pi / 60
    assert abs(rate * kp - 1) < 0.02
