"""encode/decode round trip for the DE seeding (run_study --init-designs)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "study"))
import run_study as rs  # noqa: E402
from flexblade.optimize import Objective, set_design  # noqa: E402
from flexblade.schema import OptimizeRequest, Target  # noqa: E402

LEADS = (("7in_tt_strap_steelx_v6", 7.0), ("10in_tt_strap_steelx_v5", 10.0))


def _obj(base, D):
    return Objective(OptimizeRequest(design=base, variables=rs.variables("tt_strap_steelx", D),
                                     targets=[Target(quantity="harmonic.control_ratio", kind="max")]))


def test_encode_decode_roundtrip():
    for n, D in LEADS:
        lead = rs.load_design(rs.DESIGNS / f"{n}.json")
        obj = _obj(lead, D)
        u = rs.encode_design(obj, lead)
        assert u.shape == (len(obj.variables),) and (u >= 0).all() and (u <= 1).all()
        d2 = set_design(lead, obj.to_values(u))   # lead as fixed base: only out-of-range values may change
        for v in obj.variables:
            want = min(max(rs._get(lead, v["path"]), v["lo"]), v["hi"])
            assert abs(rs._get(d2, v["path"]) - want) <= 1e-9 * max(1.0, abs(want))
        a, b = d2.model_dump(mode="json"), lead.model_dump(mode="json")
        for k in ("blade", "flexure"):   # fixed fields (laminae, clamp, balance pins) untouched
            for f in ("balance_masses", "strip_laminae", "strip_clamp", "fitting_slim"):
                if f in b[k]:
                    assert a[k][f] == b[k][f]


def test_seeded_population_shape_and_bounds():
    lead = rs.load_design(rs.DESIGNS / "7in_tt_strap_steelx_v6.json")
    obj = _obj(lead, 7.0)
    pop = rs.seeded_population(obj, [lead], 72, 0.3, seed=1)
    assert pop.shape == (72, len(obj.variables)) and (pop >= 0).all() and (pop <= 1).all()
    assert np.allclose(pop[0], rs.encode_design(obj, lead))
    assert np.abs(pop[1] - pop[0]).max() < 0.5


def test_motor_override_keeps_size_jhub_and_slew():
    lead = rs.load_design(rs.DESIGNS / "7in_tt_strap_steelx_v6.json")
    d = rs.with_motor(lead, "2307_1300kv_hakea")
    assert d.motor.name == "2307_1300kv_hakea" and d.motor.kv == 1300.0 and d.motor.R == 0.1542
    assert d.motor.J_hub == lead.motor.J_hub and d.motor.duty_slew_max == lead.motor.duty_slew_max
    assert rs.with_motor(lead) is lead   # MOTOR unset -> unchanged
