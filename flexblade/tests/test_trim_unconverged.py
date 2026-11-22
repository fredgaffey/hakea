"""Out-of-bracket trim must be flagged (trim_ok False + warning), never silently returned as a trim."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "study"))
import run_study as R  # noqa: E402
from flexblade import rotor  # noqa: E402

D5 = Path(__file__).resolve().parents[1] / "study" / "designs" / "5in_v8s_2307_m245_301.json"


def test_out_of_bracket_trim_flagged():
    d0 = R.load_design(D5)
    d = d0.model_copy(update=dict(op=d0.op.model_copy(update=dict(vehicle_mass=d0.op.vehicle_mass * 40, rpm=None))))
    m = rotor.build_model(d)
    ss = rotor.static_trim(m, rotor.make_aero(m))
    assert not ss.trim_ok
    assert any("failed" in w or "not converge" in w for w in ss.warnings)
