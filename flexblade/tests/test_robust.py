import pytest

from flexblade import presets, robust


class _Pool:
    def map(self, ds):
        return [dict(results=dict(mass=d.op.vehicle_mass)) for d in ds]


def _m(d, r):
    return dict(control_ratio=r["mass"], util={"a": r["mass"]}, viol={}, res_margin=1.0 / r["mass"],
                res_which="x", rpm=0, binding=("a", r["mass"]))


def test_variants_and_combine():
    d = presets.make_preset(5, "cf_flexure")
    vs = robust.variants(d, (0.7, 1.0, 1.4))
    assert [round(v.op.vehicle_mass / d.op.vehicle_mass, 3) for _, v in vs] == [0.7, 1.0, 1.4]
    assert all(v.op.rpm is None for _, v in vs)
    ((m, err),) = robust.map_metrics(_Pool(), [d], _m, (0.7, 1.0, 1.4))
    assert err is None and len(m["points"]) == 3
    assert m["control_ratio"] == pytest.approx(0.7 * d.op.vehicle_mass)
    assert m["util"]["a"] == pytest.approx(1.4 * d.op.vehicle_mass)
    assert m["res_margin"] == pytest.approx(1 / (1.4 * d.op.vehicle_mass))


def test_damped_resonance_criterion():
    a, ph = robust.sdof_1p(1.0, 0.04)
    assert a == pytest.approx(12.5) and ph == pytest.approx(90.0)
    assert robust.DAMPED_AMP_CAP == pytest.approx(4.10, abs=0.01)
    assert not robust.damped_resonance(0.6, 0.04, delta=0.02)["viol"]
    near = robust.damped_resonance(0.95, 0.04, delta=0.05)
    assert {"damped_amp", "damped_gain", "damped_phase"} <= set(near["viol"])
    # more damping -> smaller required margin (monotone); below 1P still needs a margin
    m = [robust.damped_required_margin(z, delta=0.02) for z in (0.02, 0.04, 0.1, 0.2)]
    assert all(x >= y for x, y in zip(m, m[1:])) and m[0] > 0.05
    c = dict(viol_hard={"cqc_mech_margin": 0.5, "cqc_mech_side": 1.0, "x": 0.1},
             per_node={"T1:nom:J": dict(mech_per_rev=0.6, mech_zeta=0.05),
                       "T1:p0:J": dict(mech_per_rev=0.97, mech_zeta=0.04)})
    v = robust.damped_gates(c, delta=0.02)
    assert "cqc_mech_margin" not in v and "cqc_mech_side" not in v and v["x"] == 0.1
    assert "damped_amp" in v and c["damped"]["node"] == "T1:p0:J"
    c["per_node"]["T1:p0:J"]["mech_zeta"] = float("nan")
    assert robust.damped_gates(c)["damped_missing"] == 1.0
