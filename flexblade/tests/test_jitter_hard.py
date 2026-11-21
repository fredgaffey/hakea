"""harmonic_jitter is a hard check at s=1, not a pulse-scaled utilisation."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "study"))
import run_study as rs  # noqa: E402


def _r(j, p1):
    return {"summary": {"rpm": None, "pitch_amp_deg": p1},
            "checks": [{"name": "harmonic_jitter", "utilisation": j / max(0.3, 0.1 * p1),
                        "value": j}]}


class _D:
    class op:
        pulse_capability_margin = None


def test_jitter_not_pulse_scaled():
    # jitter 0.5 deg > 0.3 floor but < 0.1*p1 = 1.0 deg: passes at s=1
    fns = rs.pulse_util_fns(_D, _r(0.5, 10.0))
    assert fns == []
    s, _ = rs.pulse_scale(fns)
    assert s > 0


def test_cr_pl_positive_and_violation_kept():
    out = {"control_ratio": 0.2, "viol": {}}
    rs.add_pulse_limited(_D, _r(0.5, 10.0), out)
    assert out["cr_pl"] > 0 and out["feasible_pl"]
    assert "check:harmonic_jitter" not in out["viol_hard"]
    out = {"control_ratio": 0.2, "viol": {"check:harmonic_jitter": 0.5}}
    rs.add_pulse_limited(_D, _r(2.0, 10.0), out)
    assert out["viol_hard"].get("check:harmonic_jitter") == 0.5 and out["cr_pl"] > 0
