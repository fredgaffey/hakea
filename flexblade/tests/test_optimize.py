import pytest

from flexblade import optimize
from flexblade.schema import (AnalysisOptions, Airfoil, Blade, BladeStation, Design, Flexure, Layup,
                              OperatingPoint, OptimizeRequest, Target, Variable)


def _design():
    R, e = 0.127, 0.15
    return Design(
        blade=Blade(diameter_in=10, root_cutout_frac=e, airfoil=Airfoil(naca="0012"),
                    stations=[BladeStation(r_frac=e, chord=0.02, twist_deg=0),
                              BladeStation(r_frac=1, chord=0.02, twist_deg=0)],
                    layup=Layup(material="steel_302_spring"), elastic_axis_chord_frac=0.42),
        flexure=Flexure(topology="skewed_cross_blade", hinge_radius=e * R, skew_deg=30.0, length=0.01,
                        thickness=0.0003, width=0.008, layup=Layup(material="steel_302_spring")),
        op=OperatingPoint(rpm=3000, drive="torque"),
        analysis=AnalysisOptions(aero="none", n_blade_elements=6, n_flexure_elements=2))


def test_paths():
    d = _design()
    d2 = optimize.set_design(d, {"flexure.thickness": 0.0004, "analysis.n_blade_elements": 7.2})
    assert d2.flexure.thickness == 0.0004 and d2.analysis.n_blade_elements == 7
    assert optimize.get_path(d2.model_dump(), "flexure.thickness") == 0.0004


def test_nelder_mead_hits_lag_frequency():
    """Solve flexure thickness for a target lag frequency (cheap structural model)."""
    req = OptimizeRequest(design=_design(),
                          variables=[Variable(path="flexure.thickness", lo=0.0002, hi=0.0008, log=True)],
                          targets=[Target(quantity="summary.lag_freq_per_rev", kind="eq", value=0.8)],
                          method="nelder_mead", max_evals=25)
    res = optimize.run(req)
    assert res.n_evals >= 5
    assert res.results.summary["lag_freq_per_rev"] == pytest.approx(0.8, rel=0.05)
    assert len(res.history) >= 5
