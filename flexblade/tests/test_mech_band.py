import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "study"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_study as R  # noqa: E402
from test_control_quality import _modes_result  # noqa: E402


def _viol(mech_hz, criterion):
    r = _modes_result()
    r["modes"][1]["freq_hz"] = 30.0       # other mode clear of 1P/2P
    r["modes"][2]["freq_hz"] = mech_hz    # mechanism mode (largest 1P response), 1P = 10 Hz
    d, _ = R.base_design(10.0, "tt_strap_steelx")
    return R.metrics(d, r, criterion)["viol"]


def test_mech_band_passes_092_where_margin_fails():
    assert "resonance" in _viol(9.2, "margin")
    v = _viol(9.2, "mech_band")
    assert "resonance" not in v and "mech_band" not in v


def test_mech_band_fails_095():
    v = _viol(9.5, "mech_band")
    assert "mech_band" in v
