import json
from pathlib import Path

import numpy as np

import flexblade.rotor as R
from flexblade.schema import Design

D = Path(__file__).resolve().parent.parent / "study" / "designs"


def _trim(name):
    d = Design.model_validate(json.loads((D / name).read_text()))
    # stored trims predate the steelx fitting-mass model: compare against the legacy model
    d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(fitting_mass="none"))))
    m = R.build_model(d)
    return d, R.static_trim(m, R.make_aero(m))


def test_trim_converged_design_unchanged():
    d, ss = _trim("7in_tt_strap_steelx_v2.json")
    assert ss.trim_ok
    assert abs(ss.rpm / 13265.8 - 1) < 1e-3
    assert abs(ss.trim.T - d.op.vehicle_mass * 9.80665) < 2e-3 * d.op.vehicle_mass * 9.80665


def test_trim_failure_is_flagged_not_silent():
    # 10in strap: thrust(rpm) never reaches W (torsional collapse branch) -> explicit failure flag
    d, ss = _trim("10in_tt_strap_steelx_v2.json")
    assert not ss.trim_ok
    assert any("trim failed" in w for w in ss.warnings)
