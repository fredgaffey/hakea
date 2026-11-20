"""Temperature derating of moduli/allowables and the HDT check."""
import json
from pathlib import Path

import numpy as np

from flexblade.materials import LIBRARY, META, get_material, part_temperature, temp_knockdown
from flexblade.rotor import evaluate
from flexblade.schema import Design

WINNER = Path(__file__).parents[1] / "study" / "designs" / "5in_tpu_rod_winner.json"


def _design(T):
    d = json.loads(WINNER.read_text())
    d.setdefault("op", {})["ambient_C"] = T
    return Design.model_validate(d)


def test_knockdown_monotonic_and_bounded():
    Ts = np.linspace(-10, 200, 60)
    k = [temp_knockdown(T, 70.0) for T in Ts]
    assert all(a >= b for a, b in zip(k, k[1:]))
    assert k[0] == 1.0 and min(k) == 0.3
    assert np.isclose(temp_knockdown(70.0, 70.0), 0.3)
    assert temp_knockdown(100.0, 0.0) == 1.0   # hdt n/a


def test_derated_material_monotonic():
    name = "bambu_tpu_95a_hf"
    prev = None
    for T in (25, 40, 60, 80, 120):
        with part_temperature(T):
            m = get_material(name)
        cur = (m.E1, m.Xt, m.creep_strain_allow, m.fatigue_strain_amp_allow)
        if prev:
            assert all(c <= p for c, p in zip(cur, prev))
        prev = cur
    assert get_material(name).E1 == LIBRARY[name].E1   # context restored


def test_brand_allowables_reach_checks():
    name = "bambu_tpu_95a_hf"
    assert LIBRARY[name].Xt == 27.3e6 and LIBRARY[name].hdt_C > 0
    assert LIBRARY[name].allow_source in ("class default", "brand")
    d = json.loads(WINNER.read_text())
    d["flexure"]["layup"]["material"] = name
    r = evaluate(Design.model_validate(d))
    notes = " ".join(c.note for c in r.checks if c.name == "static_stress")
    assert name in notes and "26.2 MPa" in notes   # brand Xt/Yt/Zt-derived strength reaches the check


def test_winner_evaluates_and_derates():
    r25 = evaluate(_design(25.0))
    r40 = evaluate(_design(40.0))
    u = lambda r: max(c.utilisation for c in r.checks if c.name != "hdt_temperature")
    assert np.isfinite(u(r25)) and u(r40) >= u(r25)
    assert any(c.name == "hdt_temperature" for c in r40.checks)
    hot = evaluate(_design(120.0))
    assert max(c.utilisation for c in hot.checks if c.name == "hdt_temperature") > 1.0
