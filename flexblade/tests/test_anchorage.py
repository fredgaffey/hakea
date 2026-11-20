"""Member-end anchorage compliance (anchorage.py, rotor member_anchorage)."""
import json
from pathlib import Path

import numpy as np

from flexblade import anchorage, fact, rotor
from flexblade.materials import get_material
from flexblade.schema import Design, FlexureMember, Layup

DESIGNS = Path(__file__).parents[1] / "study" / "designs"


def _with(d, anc):
    return d.model_copy(update=dict(analysis=d.analysis.model_copy(update=dict(member_anchorage=anc))))


def test_circular_punch_closed_form():
    """Steel bar on steel half-space: k_n = 2 a E*, k_tilt = 4/3 a^3 E*, E* = E / (2 (1 - nu^2))."""
    m = FlexureMember(kind="wire", p0=(0.0, 0, 0), p1=(0.01, 0, 0), width_dir=(0, 1.0, 0), width=0.002, thickness=0.002,
                      shape="circle", layup=Layup(material="steel_302_spring"))
    e = np.eye(3)
    K = anchorage.end_spring(m, None, e[0], e[1], e[2], "steel_302_spring", None)
    s = get_material("steel_302_spring")
    Es = s.E1 / (2 * (1 - s.nu12 ** 2))
    a = 0.001
    assert np.isclose(K[0, 0], 2 * a * Es, rtol=1e-6)
    assert np.isclose(K[4, 4], 4 / 3 * a ** 3 * Es, rtol=1e-6)
    assert np.all(np.linalg.eigvalsh(K) > 0)


def test_winner_hinge_matches_calculix():
    """5in_tpu_rod_winner: rigid member ends gave beam k_hinge 10.2 N m/rad vs CalculiX 2.99
    (E02/E08). With the hub-end punch anchorage (ccx basis: blade side rigid) the beam is
    within 25 %; adding the blade end softens further."""
    d = Design.model_validate(json.loads((DESIGNS / "5in_tpu_rod_winner.json").read_text()))
    a = fact.hinge_axis(d)[1]
    k = {}
    for anc in ("none", "hub", "both"):
        K = rotor.flexure_stiffness(rotor.build_model(_with(d, anc)))
        k[anc] = 1.0 / (a @ np.linalg.inv(K)[3:, 3:] @ a)
    assert k["none"] > 2.5 * k["hub"] > 2.5 * k["both"]
    assert abs(2.99 / k["hub"] - 1.0) < 0.25
