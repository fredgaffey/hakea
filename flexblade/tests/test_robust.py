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
