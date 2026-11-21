import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "study"))
import run_study as rs
from flexblade.optimize import set_design, get_path, BAL_KEYS
from flexblade.schema import Design


def test_bal_mapping_roundtrip():
    d = Design()
    vals = {"bal.mass": 1e-3, "bal.r_frac": 0.25, "bal.chord": 0.01, "bal.normal": 0.005}
    d2 = set_design(d, vals)
    bm = d2.blade.balance_masses
    assert len(bm) == 2
    assert bm[0].chord_offset == -bm[1].chord_offset == 0.01
    assert bm[0].normal_offset == -bm[1].normal_offset == 0.005
    dj = d2.model_dump(mode="json")
    assert all(abs(get_path(dj, k) - vals[k]) < 1e-12 for k in BAL_KEYS)
    d3 = set_design(d2, {"bal.mass": 2e-3})   # partial update keeps the other fields
    assert d3.blade.balance_masses[1].normal_offset == -0.005


def test_variables_declared():
    rs.BAL_VARS = True
    try:
        V = rs.variables("tt_strap_steelx", 10.0)
    finally:
        rs.BAL_VARS = False
    p = {v.path: v for v in V}
    assert p["bal.chord"].hi == 0.012 and p["flexure.strap_length"].lo == 0.006
    assert "flexure.skew_deg" in p


def test_one_sided_flags_sign_change():
    sc, dr, pen = rs.one_sided_check([6500, 7700, 9100], [0.97, 0.99, 1.02])
    assert sc
    sc, dr, pen = rs.one_sided_check([6500, 7700, 9100], [0.958, 0.948, 0.942])
    assert not sc and dr < 0.07 and pen == 0.0
    sc, dr, pen = rs.one_sided_check([6500, 9100], [1.10, 1.02])
    assert not sc and pen > 0


def test_blade_vars_and_core_fraction():
    from flexblade.schema import Layup, BladeStation
    from flexblade.section import section_properties, RectShape
    rs.BLADE_VARS = True
    try:
        p = {v.path for v in rs.variables("tt_strap_steelx", 10.0)}
    finally:
        rs.BLADE_VARS = False
    assert {"blade.chord_scale", "blade.layup.core_fraction", "blade.layup.skin_thickness"} <= p
    assert "blade.chord_scale" not in {v.path for v in rs.variables("tt_strap_steelx", 10.0)}
    d = Design()
    d = d.model_copy(update=dict(blade=d.blade.model_copy(update=dict(
        stations=[BladeStation(r_frac=0.5, chord=0.04)]))))
    d2 = set_design(set_design(d, {"blade.chord_scale": 1.3}), {"blade.chord_scale": 1.3})
    assert abs(d2.blade.stations[0].chord - 0.052) < 1e-12

    def sp(f):
        return section_properties(RectShape(width=0.04, thickness=0.006), Layup(
            kind="core_skin", material="pa6_cf", skin_material="pa6_cf", skin_thickness=0.0008,
            core_fraction=f), None, force_grid=True)
    a, b, c = sp(1.0), sp(0.0), sp(0.5)
    assert b.mass_per_len < c.mass_per_len < a.mass_per_len and b.EIy < c.EIy < a.EIy
