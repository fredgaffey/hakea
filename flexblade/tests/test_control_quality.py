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
    assert [p["tag"] for p in out["points"]] == ["T1"]          # tolerance samples: spread only


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


# ---------------------------------------------------------------- control_quality_cal
def _m(g, ph, mpr=0.9, cr=0.1):
    return dict(control_ratio=cr, cr_pl=cr, viol={}, viol_hard={}, util={}, criterion="control_quality_cal",
                cyc_gain=g, cyc_phase_deg=ph, mech_per_rev=mpr)


def _cal_set(build_gain=1.0, build_phase=0.0, rt=None, build_mpr=0.9):
    """nominal build + one build corner p0 (calibrated out), each with the runtime samples."""
    rt = rt or {"rpm+": (1.05, 3.0), "rpm-": (0.97, -2.0), "temp+": (1.02, 1.0), "creep": (0.95, 2.0),
                "J+": (1.0, 4.0), "J-": (1.0, -1.0)}
    tags, ms = [], []
    for p, g0, ph0, mpr in (("", 1.0, 0.0, 0.9), (":p0", build_gain, build_phase, build_mpr)):
        tags.append(f"T1{p}:J1")
        ms.append(_m(g0, ph0, mpr, cr=0.1 * g0))
        for r, (gm, dph) in rt.items():
            tags.append(f"T1{p}:r{r}:J1")
            ms.append(_m(g0 * gm, ph0 + dph, mpr))
    return tags, ms


def test_cal_residual_is_linear_stack_and_build_offset_is_calibrated_out():
    # build corner: gain 0.6x, phase +80 deg -> FAILS control_quality, but is calibrated out here
    tags, ms = _cal_set(build_gain=0.6, build_phase=80.0)
    c = robust.control_quality_cal(tags, ms)
    # rpm 3 + temp (1 + 0.3*2) + creep 2 + J 4 = 10.6 deg; gain 5 + (2 + 0.3*5) + 5 + 0 = 13.5 %
    assert c["phase_res_deg"] == pytest.approx(10.6)
    assert c["gain_res"] == pytest.approx(0.135)
    assert c["groups"]["rpm"] == pytest.approx([3.0, 0.05])
    assert c["authority_min"] == pytest.approx(0.6) and c["cr_pl_ratio_min"] == pytest.approx(0.6)
    assert c["mech_margin_min"] == pytest.approx(0.1) and not c["mech_side_flip"]
    assert robust.control_quality_cal_viol(c) == {}
    nom = [(t, m) for t, m in zip(tags, ms) if ":r" not in t]
    cq = robust.control_quality([t for t, _ in nom], [m["cyc_gain"] for _, m in nom],
                                [m["cyc_phase_deg"] for _, m in nom])
    assert robust.control_quality_viol(cq)            # the uncalibrated criterion rejects it


def test_cal_limits_residual_and_calibratability():
    big = {"rpm+": (1.0, 12.0), "rpm-": (1.0, -2.0), "creep": (1.3, 0.0)}
    c = robust.control_quality_cal(*_cal_set(rt=big))
    v = robust.control_quality_cal_viol(c)
    assert c["phase_res_deg"] == pytest.approx(12.0) and "cqc_phase" not in v
    assert c["gain_res"] == pytest.approx(0.3 * 1.3)          # creep 30 % + temp- 0.3 x 30 %
    assert v["cqc_gain"] == pytest.approx(0.39 / 0.25 - 1)
    v = robust.control_quality_cal_viol(robust.control_quality_cal(*_cal_set(build_gain=0.4)))
    assert v["cqc_authority"] == pytest.approx(0.2)
    v = robust.control_quality_cal_viol(robust.control_quality_cal(*_cal_set(build_mpr=0.98)))
    assert v["cqc_mech_margin"] == pytest.approx(1 - 0.02 / 0.03)
    v = robust.control_quality_cal_viol(robust.control_quality_cal(*_cal_set(build_mpr=1.05)))
    assert "cqc_mech_side" in v and "cqc_mech_margin" not in v   # crossed 1P: phase flips


def test_cal_combine_uses_nominal_points_for_hard_checks():
    tags, ms = _cal_set(build_gain=0.4)
    out = robust.combine(ms, tags)
    assert [p["tag"] for p in out["points"]] == ["T1:J1"]
    assert out["cr_pl"] == pytest.approx(0.1)
    assert "cqc_authority" in out["viol_hard"] and out["feasible_pl"] is False


def test_runtime_variants_tags_and_perturbations():
    from flexblade.schema import Design
    p = Path(__file__).resolve().parents[1] / "study" / "designs" / "7in_tt_strap_steelx_v6.json"
    d = Design.model_validate_json(p.read_text())
    v = dict(robust.variants(d, (1.0,), [{"flexure.thickness": 1.05}], (2e-6,), list(robust.CQ_RUNTIME)))
    assert len(v) == 2 * (1 + len(robust.CQ_RUNTIME))
    n, r = v["T1:J2e-06"], v["T1:p0:rcreep:J2e-06"]
    assert r.flexure.width == pytest.approx(0.98 * n.flexure.width)
    assert r.flexure.thickness == pytest.approx(1.05 * n.flexure.thickness)
    assert v["T1:rJ+:J2e-06"].motor.J_hub == pytest.approx(2.2e-6)
    assert v["T1:rrpm+:J2e-06"].op.vehicle_mass == pytest.approx(1.1025 * n.op.vehicle_mass)
    assert v["T1:rtemp+:J2e-06"].op.ambient_C == 45.0 and n.op.ambient_C == d.op.ambient_C
    # without runtime samples the variant set (and so the evaluation cache keys) is unchanged
    assert [t for t, _ in robust.variants(d, (1.0,), None, (2e-6,))] == ["T1:J2e-06"]
    assert robust.variants(d, (1.0,), None, (2e-6,))[0][1] == n
