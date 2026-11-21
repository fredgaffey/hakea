import sys
from pathlib import Path

import numpy as np
import pytest

from flexblade import robust

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "study"))
import run_study as R  # noqa: E402


def test_phase_spread_about_median_wraps_and_per_throttle():
    tags = ["T0.7:J1", "T0.7:p0:J1", "T0.7:p1:J1", "T1:J1", "T1:p0:J1", "T1:p1:J1"]
    # T0.7 straddles +-180 (median 178, spread 10 deg across the wrap); T1 offset 90 is trimmed out
    ph = [178.0, -172.0, 175.0, 90.0, 100.0, 85.0]
    cq = robust.control_quality(tags, [1.0] * 6, ph)
    assert cq["per_throttle"]["T0.7"]["phase_med"] == pytest.approx(178.0)
    assert cq["per_throttle"]["T0.7"]["phase_spread"] == pytest.approx(10.0)
    assert cq["per_throttle"]["T1"]["phase_spread"] == pytest.approx(10.0)
    assert cq["phase_spread_deg"] == pytest.approx(10.0)
    assert robust.control_quality_viol(cq) == {}
    cq = robust.control_quality(tags, [1.0] * 6, [0, 20, -10, 0, 0, 0])
    assert cq["phase_spread_deg"] == pytest.approx(20.0)
    assert robust.control_quality_viol(cq)["cq_phase"] == pytest.approx(20 / 15 - 1)


def test_gain_spread_normalised_by_j_schedule():
    # gain 2x at the high J is scheduled out; only the tolerance spread at each J counts
    tags = ["T1:J1", "T1:p0:J1", "T1:p1:J1", "T1:J2", "T1:p0:J2", "T1:p1:J2"]
    cq = robust.control_quality(tags, [1.0, 1.1, 0.9, 2.0, 2.6, 2.0], [0.0] * 6)
    assert cq["gain_spread"] == pytest.approx(0.3)
    assert robust.control_quality_viol(cq)["cq_gain"] == pytest.approx(0.3 / 0.25 - 1)
    cq = robust.control_quality(tags, [1.0, 1.1, 0.9, 2.0, 2.2, 1.8], [0.0] * 6)
    assert cq["gain_spread"] == pytest.approx(0.1)
    bad = robust.control_quality(tags, [1.0, float("nan"), 1, 1, 1, 1], [0.0] * 6)
    assert set(robust.control_quality_viol(bad)) == {"cq_phase", "cq_gain"}


def test_combine_adds_cq_violation_to_hard():
    ms = [dict(control_ratio=0.1, cr_pl=0.1, viol={}, viol_hard={}, util={}, criterion="control_quality",
               cyc_gain=g, cyc_phase_deg=p) for g, p in ((1.0, 0.0), (1.0, 40.0), (1.0, -40.0))]
    out = robust.combine(ms, ["T1", "T1:p0", "T1:p1"])
    assert out["cq"]["phase_spread_deg"] == pytest.approx(40.0)
    assert "cq_phase" in out["viol_hard"] and out["feasible_pl"] is False


def _modes_result():
    """3 nodes x 6 dof; modes: rigid spin, A (near 1P, NOT labelled pitch), B; response mostly B."""
    rng = np.random.default_rng(0)
    shapes = [rng.standard_normal((3, 6)) for _ in range(3)]
    U = 0.2 * shapes[1] + (1.0 + 0.5j) * shapes[2] + 5.0 * shapes[0]   # big rigid-spin content
    modes = [dict(freq_hz=0.0, damping_ratio=0.0, label="rigid spin", participation={}, shape=shapes[0].tolist()),
             dict(freq_hz=9.5, damping_ratio=0.02, label="pitch antisym+hub", participation={"hub": 0.3},
                  shape=shapes[1].tolist()),
             dict(freq_hz=8.0, damping_ratio=0.02, label="flap sym", participation={"hub": 0.0},
                  shape=shapes[2].tolist())]
    return {"summary": {"rpm": 600.0}, "modes": modes,
            "harmonic": {"displacement_re": U.real.tolist(), "displacement_im": U.imag.tolist(),
                         "nodes": [[0.01, 0, 0], [0.05, 0, 0], [0.1, 0, 0]]}}


def test_mechanism_mode_by_participation_not_label():
    r = _modes_result()
    assert R.mechanism_mode(r) == 2              # largest response share, rigid spin excluded
    c = R.modal_participation(np.array([1.0, 2.0j]), np.eye(2))
    assert c == pytest.approx([1 / 3, 2 / 3])
    # the mechanism mode (0.8/rev) is exempt from the 1P margin; the other elastic mode still counts
    mg_all, _ = R.resonance_margin(r)
    mg, wh = R.resonance_margin(r, skip_1p=2)
    assert mg_all <= mg and "pitch antisym+hub" in wh
    m_ = r["modes"]
    m_[1]["freq_hz"], m_[2]["freq_hz"] = 30.0, 9.6
    mg, wh = R.resonance_margin(r, skip_1p=R.mechanism_mode(r))
    assert mg > 0.5          # only the 3/rev mode (and its 2P hub check) remains
