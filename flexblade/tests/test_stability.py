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
