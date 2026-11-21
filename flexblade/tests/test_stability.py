"""Damped rotating-frame eigenvalues, air/ground resonance (stability.py)."""
import json
from pathlib import Path

import numpy as np
import pytest

from flexblade import rotor
from flexblade import stability as S
from flexblade.matsweep import _margin
from flexblade.schema import Airframe, Design

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
