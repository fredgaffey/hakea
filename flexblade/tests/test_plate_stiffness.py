"""Wide-plate (crossed-blade flexure) bending modulus vs ccx solid reference.

ccx rigid-fitting k_hinge_free (N m/rad, measured with study/campaign/experiments/e08_root_compliance.py
variant 'rigid'): 5in_cf_flexure 31.35 (beam before the plate-modulus fix: 16.98, ratio 0.54);
see CAMPAIGN.md E08."""
import json
from pathlib import Path

import numpy as np
import pytest

from flexblade import fact, rotor
from flexblade.materials import get_material
from flexblade.schema import Design, PrintOrientation

DESIGNS = Path(__file__).parents[1] / "study" / "designs"


def test_plate_modulus_isotropic_and_shear_coupling():
    m = get_material("pa6_cf")
    e = np.eye(3)
    # on-axis: width fully restrained -> E1/(1 - nu12 nu21) (plane stress)
    q = m.plate_modulus(PrintOrientation(), e[0], e[1], e[2])
    assert np.isclose(q, m.E1 / (1 - m.nu12 * m.nu12 * m.E2 / m.E1), rtol=1e-6)
    # 35 deg off-axis: shear coupling makes the restrained modulus much larger than 1/S11
    ph = np.radians(35.49)
    L, W = np.array([np.cos(ph), 0, np.sin(ph)]), np.array([np.sin(ph), 0, -np.cos(ph)])
    ratio = m.plate_modulus(PrintOrientation(), L, W, e[1]) / m.axial_modulus(PrintOrientation(), L)
    assert 1.5 < ratio < 1.8


def _k_free(d):
    d = d.model_copy(update=dict(analysis=d.analysis.model_copy(update=dict(member_anchorage="none"))))
    a = fact.hinge_axis(d)[1]
    K = rotor.flexure_stiffness(rotor.build_model(d))
    return 1.0 / (a @ np.linalg.inv(K)[3:, 3:] @ a)


def test_cf_flexure_beam_vs_ccx_rigid():
    d = Design.model_validate(json.load(open(DESIGNS / "5in_cf_flexure.json")))
    ratio = _k_free(d) / 31.35          # ccx rigid-fitting reference
    assert 0.65 < ratio < 1.1           # was 0.54 before the plate-modulus fix; ~0.70 now
