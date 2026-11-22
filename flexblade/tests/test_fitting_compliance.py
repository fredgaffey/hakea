"""FITTING_COMPLIANCE: series glue-land + printed fitting-arm springs on clamped-strip flexures."""
import numpy as np
import pytest

from flexblade import fitting_compliance as FC
from flexblade import geometry as G
from flexblade import rotor as R
from flexblade.fittings import is_clamped_strip
from flexblade.schema import Design
from pathlib import Path

LEAD = Path(__file__).resolve().parents[1] / "study" / "designs" / "5in_v8s_2307_m245_301.json"


@pytest.fixture(scope="module")
def lead():
    d = Design.model_validate_json(LEAD.read_text())
    # clean strips-only basis (as the CalculiX case 1): no slot presets (their unequilibrated misfit
    # state adds a geometric stiffness), no axial release, no strap
    d.flexure.strip_preset_deg = [[[0.0] * 6 for _ in b] for b in d.flexure.strip_preset_deg]
    d.flexure.axial_release = False
    d.flexure.strap_strands = 0
    if d.flexure.pre_lag_deg == "auto":
        d = R.resolve_pre_lag(d)
    assert is_clamped_strip(d)
    return d


def _k(d, monkeypatch, **env):
    for k, v in env.items():
        monkeypatch.setenv(k, str(v))
    members, (hp, ax), _ = G.get_members(d)
    a = np.asarray(ax, float) / np.linalg.norm(ax)
    members = [m for m in members if not m.cable]
    K = R.flexure_only_model(d, members).stiffness_at(np.asarray(hp, float))
    Cr = np.linalg.inv(K)[3:, 3:]
    y = np.array([0.0, 1.0, 0.0])
    s = np.cross(y, a)
    s /= np.linalg.norm(s)
    return K, {n: 1.0 / (v @ Cr @ v) for n, v in (("hinge", a), ("cross", s), ("flap", y))}


def test_default_off_and_rigid_limit_is_exact(lead, monkeypatch):
    monkeypatch.delenv("FITTING_COMPLIANCE", raising=False)
    assert not FC.enabled()
    K0, _ = _k(lead, monkeypatch)
    K1, _ = _k(lead, monkeypatch, FITTING_COMPLIANCE=1, FITTING_E="inf", FITTING_GLUE_E="inf")
    assert np.array_equal(K0, K1)


def test_softer_fittings_monotonically_softer(lead, monkeypatch):
    _, k0 = _k(lead, monkeypatch, FITTING_COMPLIANCE=0)
    prev = k0
    for E in (210e9, 7.5e9, 2.0e9):
        _, k = _k(lead, monkeypatch, FITTING_COMPLIANCE=1, FITTING_E=E)
        for q in ("cross", "flap"):
            assert k[q] < prev[q]
        assert k["hinge"] <= prev["hinge"] * (1 + 1e-9)
        assert k["hinge"] > 0.9 * k0["hinge"]          # pitch barely moves (CalculiX: -1.5 % PETG)
        prev = k


def test_glue_spring_limits():
    from types import SimpleNamespace
    sp = SimpleNamespace(EA=3.5e5, EIy=7.2e-5, EIz=0.46, GJ=1.1e-4)
    Ks = FC.glue_end_spring(sp, 4e-3, 1.0, (3e9, 0.35, 0.1e-3, 8e-3))
    Kt = FC.glue_end_spring(sp, 4e-3, 1.0, (3e9, 0.35, 0.3e-3, 8e-3))
    assert np.all(np.linalg.eigvalsh(Ks) > 0)
    assert np.all(np.diag(Kt) < np.diag(Ks))          # thicker glue is softer
    assert FC.glue_end_spring(sp, 4e-3, 1.0, (np.inf, 0.35, 0.1e-3, 8e-3)) is None
