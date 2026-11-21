"""fact.stopped_state: rotor stopped on the bench, strips self-stressed by the pre-angled slots
(strip_preset_deg) with no CF / aero (study/results/stopped_state.md)."""
from pathlib import Path

import pytest

from flexblade import rotor
from flexblade.fact import STOPPED_BUCKLING_MIN, STOPPED_YIELD_FRAC, stopped_state
from flexblade.materials import get_material
from flexblade.schema import Design

V8 = Path(__file__).resolve().parents[1] / "study" / "designs" / "10in_tt_strap_steelx_v8.json"


@pytest.fixture(scope="module")
def v8():
    return rotor.resolve_pre_lag(Design.model_validate_json(V8.read_text()))


def test_stopped_state_v8_safe_and_path_independent(v8):
    r = stopped_state(v8, n_steps=2)
    assert r["converged"]
    assert r["buckling_multiplier"] > STOPPED_BUCKLING_MIN
    assert r["sigma_max"] < STOPPED_YIELD_FRAC * get_material(v8.flexure.layup.material).Xt
    assert 10e6 < r["sigma_max"] < 150e6                 # misfit does stress the strips (~67 MPa)
    assert all(abs(s["N"]) < 0.05 * s["Pcr"] for s in r["strips"])   # radial slider frees the axial misfit
    r4 = stopped_state(v8, n_steps=4)
    assert r4["sigma_max"] == pytest.approx(r["sigma_max"], rel=1e-3)
    assert r4["k"][2]["k_2nd"] == pytest.approx(r["k"][2]["k_2nd"], rel=1e-3)


def test_stopped_state_no_presets_is_unstressed(v8):
    d = v8.model_copy(deep=True)
    d.flexure.strip_preset_deg = []
    r = stopped_state(d)
    assert r["sigma_max"] < 1e3 and r["buckling_multiplier"] > 1e6
    assert r["k"][1]["k_cross"] == pytest.approx(r["k"][1]["k_cross0"])


def test_stopped_state_without_axial_release_is_marginal(v8):
    """Without the blade-end radial slider the slot misfit loads the strips axially (~0.1 Pcr) and the
    misfit state is within 2x of buckling: the slider is what makes the presets benign when stopped."""
    d = v8.model_copy(deep=True)
    d.flexure.axial_release = False
    r = stopped_state(d, n_steps=4)     # 2 steps jumps to a spurious branch (>= 4 agree to 4 digits)
    assert r["buckling_multiplier"] < STOPPED_BUCKLING_MIN
    assert min(s["N"] for s in r["strips"]) < -2.0
