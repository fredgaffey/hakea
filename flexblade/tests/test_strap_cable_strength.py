"""Bought unbonded strand (strap cable): strength is the along-fibre Xt whatever the strand's
deflected direction vs the print-orientation frame, and there is no interlayer check."""
import numpy as np

from flexblade import rotor
from flexblade.materials import get_material
from flexblade.schema import Layup, PrintOrientation
from flexblade.section import CircleShape, section_properties


def _strand(d=0.0016):
    ori = PrintOrientation(build_dir=(0.0, 0.0, 1.0), raster_dir=(1.0, 0.0, 0.0))
    e1, e2, e3 = np.eye(3)
    sp = section_properties(CircleShape(d), Layup(kind="solid", material="aramid_yarn"), ori, e1,
                            axes=(e2, e3))
    return rotor._cable_section(sp, 1e-3)


def test_cable_uses_fibre_strength_off_axis():
    sec = _strand()
    T = 800.0
    sf = np.array([T, 0, 0, 0, 0, 0], float)
    t_hat = np.array([np.cos(0.06), 0.0, np.sin(0.06)])     # strand tilted 3.4 deg
    Xt = get_material("aramid_yarn").Xt
    u_s, u_il, *_ = rotor._fibre_checks(None, sec, sf, t_hat, cable=True)
    sig = T / (np.pi * 0.0016 ** 2 / 4)
    assert abs(u_s - sig / Xt) / (sig / Xt) < 0.05
    assert u_il == 0.0
    # printed-member path knocks the strength down off-axis (Hill) and checks interlayer
    u_p, il_p, *_ = rotor._fibre_checks(None, sec, sf, t_hat, cable=False)
    assert u_p > 1.5 * u_s and il_p > 0.0
