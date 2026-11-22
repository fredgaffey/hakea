"""Opt-in CABLE_STRAIN=axial: a bought cord's fatigue/creep strain is eps0 + kappa * r_filament
(filaments bend individually) instead of the solid-section fibre strain. Default unchanged."""
from pathlib import Path

import numpy as np

from flexblade import rotor
from flexblade.schema import Design, Layup, PrintOrientation
from flexblade.section import CircleShape, RectShape, section_properties

LEAD = Path(__file__).resolve().parents[1] / "study/designs/5in_v8s_2307_m245_301.json"
ORI = PrintOrientation(build_dir=(0.0, 0.0, 1.0), raster_dir=(1.0, 0.0, 0.0))
E1, E2, E3 = np.eye(3)
T_HAT = np.array([1.0, 0.0, 0.0])


def _sec(shape, material):
    return section_properties(shape, Layup(kind="solid", material=material), ORI, E1, axes=(E2, E3))


def _both(fn, monkeypatch):
    monkeypatch.delenv("CABLE_STRAIN", raising=False)
    a = fn()
    monkeypatch.setenv("CABLE_STRAIN", "axial")
    b = fn()
    monkeypatch.delenv("CABLE_STRAIN", raising=False)
    return a, b


def test_printed_member_bit_identical(monkeypatch):
    sec = _sec(RectShape(4e-3, 0.6e-3), "pla")
    sf = np.array([40.0, 0.3, 0.0, 1e-4, 2e-3, -1e-3])
    sc = sf * (1 + 0.4j)
    a, b = _both(lambda: (rotor._fibre_checks(None, sec, sf, T_HAT, False),
                          rotor._fibre_amp(sec, sc, T_HAT, 200.0, False)), monkeypatch)
    assert repr(a) == repr(b)


def test_cable_default_unchanged_and_axial_lower(monkeypatch):
    sec = rotor._cable_section(_sec(CircleShape(1.3e-3), "aramid_yarn"), 1e-3)
    sf = np.array([80.0, 0.0, 0.0, 0.0, 2e-6, 1e-6])            # tension + cord bending
    sc = np.array([30.0, 0, 0, 0, 3e-6, 0]) * (1 + 0.2j)
    (c0, a0), (c1, a1) = _both(lambda: (rotor._fibre_checks(None, sec, sf, T_HAT, True),
                                        rotor._fibre_amp(sec, sc, T_HAT, 200.0, True)), monkeypatch)
    eps = np.linalg.solve(sec.stiffness6(), sf)
    ref = max(eps[0] + fb.z * eps[4] - fb.y * eps[5] for fb in sec.fibres)
    assert c0[2] == ref                                          # default: section fibre strain
    k = np.hypot(eps[4], eps[5])
    assert np.isclose(c1[2], eps[0] + rotor.CABLE_FILAMENT_RADIUS * k)
    assert c1[0] == c0[0]                                        # static strength unchanged
    assert c1[2] < c0[2] and c1[4] < c0[4] and a1[0] < a0[0] and a1[2] < a0[2]


def test_lead_axial_lowers_strap_only(monkeypatch):
    d = Design.model_validate_json(LEAD.read_text())

    def run():
        r = rotor.evaluate(d, ["trim", "checks"])
        return {(c.name, c.member): c.utilisation for c in r.checks}

    a, b = _both(run, monkeypatch)
    assert a.keys() == b.keys()
    for k in a:
        if k[1] != "flex2":
            assert a[k] == b[k], k
    assert b[("fatigue_strain", "flex2")] < a[("fatigue_strain", "flex2")]
    assert b[("creep_strain", "flex2")] < a[("creep_strain", "flex2")]
