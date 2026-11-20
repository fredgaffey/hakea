import json
from pathlib import Path

from flexblade import rotor
from flexblade.aero import AeroModel
from flexblade.rootdrag import envelope, hardware_profile
from flexblade.schema import Design

D = Path(__file__).resolve().parents[1] / "study" / "designs" / "5in_tt_strap_hand.json"


def _design():
    return Design.model_validate(json.load(open(D)))


def test_root_drag_adds_power_and_fairing_reduces_it():
    d = _design()
    off = d.model_copy(update=dict(analysis=d.analysis.model_copy(update=dict(root_drag=False))))
    fair = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(fairing=True))))
    t0, t1, t2 = (AeroModel(x).trim(rpm=19000) for x in (off, d, fair))
    assert t0.Q_root == 0.0 and t1.Q_root > t2.Q_root > 0
    assert abs(t1.P - t0.P - t1.Q_root * t1.omega) < 1e-9
    assert abs(t1.T - t0.T) < 1e-9


def test_root_envelope_soft_check():
    d = _design()
    h, allow = envelope(d)
    assert len(hardware_profile(d)[0]) > 0 and h > allow > 0
    r = rotor.evaluate(d)
    c = [c for c in r.checks if c.name == "root_envelope"]
    assert c and c[0].utilisation > 1.0


def test_faired_geometry_exports_and_fitting_walls(tmp_path):
    from flexblade import geometry as g
    d = _design()
    f = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(fairing=True))))
    geo = g.build_geometry(f, blades=[1], n_span=6)
    names = [p.name for p in geo.parts]
    assert "blade1_fairing_top" in names and "blade1_fairing_bot" in names
    assert not any("fairing" in p.name for p in g.build_geometry(d, blades=[1], n_span=6).parts)
    man = g.export_print_parts(f, tmp_path, blades="one", fmt="3mf")
    assert man["files"]["3mf"]
    assert g._clamp_length(d, g.get_members(d)[0], 0.03) >= 2 * g.WALL_MIN
    assert envelope(f)[0] < 1.5 * envelope(d)[0]
