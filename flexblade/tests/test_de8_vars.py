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
        stations=[BladeStation(r_frac=0.5, chord=0.04, twist_deg=10.0)]))))
    d2 = set_design(set_design(d, {"blade.chord_scale": 1.3}), {"blade.chord_scale": 1.3})
    assert abs(d2.blade.stations[0].chord - 0.052) < 1e-12

    def sp(f):
        return section_properties(RectShape(width=0.04, thickness=0.006), Layup(
            kind="core_skin", material="pa6_cf", skin_material="pa6_cf", skin_thickness=0.0008,
            core_fraction=f), None, force_grid=True)
    a, b, c = sp(1.0), sp(0.0), sp(0.5)
    assert b.mass_per_len < c.mass_per_len < a.mass_per_len and b.EIy < c.EIy < a.EIy


def test_laminae_vars_and_snap():
    from flexblade.optimize import LAMINA_STOCK
    rs.LAMINAE_VARS = True
    try:
        p = {v.path for v in rs.variables("tt_strap_steelx", 10.0)}
    finally:
        rs.LAMINAE_VARS = False
    assert {"flexure.strip_laminae", "flexure.lamina_idx"} <= p and "flexure.thickness" not in p
    assert "flexure.strip_laminae" not in {v.path for v in rs.variables("tt_strap_steelx", 10.0)}
    d = rs.base_design(10.0, "tt_strap_steelx", False)[0]
    d2 = set_design(d, {"flexure.strip_laminae": 6.4, "flexure.lamina_idx": 2.6})
    assert isinstance(d2.flexure.strip_laminae, int) and d2.flexure.strip_laminae == 6
    assert d2.flexure.thickness == LAMINA_STOCK[3]
    assert set_design(d, {"flexure.strip_laminae": 1.5}).flexure.strip_laminae == 2


def test_lamina_idx_get_path_roundtrip():
    """run_study reads every DE var's start value with get_path; the virtual lamina_idx must resolve."""
    import json
    from pathlib import Path
    from flexblade.optimize import get_path, set_design, LAMINA_STOCK
    from flexblade.schema import Design
    d = Design.model_validate(json.loads(Path("study/designs/6in_tt_strap_steelx_2307.json").read_text()))
    i = get_path(d.model_dump(mode="json"), "flexure.lamina_idx")
    assert 0 <= i <= len(LAMINA_STOCK) - 1 and i == int(i)
    d2 = set_design(d, {"flexure.lamina_idx": 2.0})
    assert get_path(d2.model_dump(mode="json"), "flexure.lamina_idx") == 2.0


def _lead():
    p = Path(__file__).resolve().parents[1] / "study/designs/5in_v8s_2307_m245_301.json"
    return Design.model_validate_json(p.read_text())


def test_width_idx_roundtrip():
    d = _lead()
    assert get_path(d.model_dump(mode="json"), "flexure.width_idx") == 1.0
    d2 = set_design(d, {"flexure.width_idx": 2.0})
    assert abs(d2.flexure.width - 5.0e-3) < 1e-12


def test_material_override_on_seed():
    d = _lead()
    rs.MATERIAL = "steel_1095_shim"
    try:
        assert rs._material(d).flexure.layup.material == "steel_1095_shim"
    finally:
        rs.MATERIAL = None
    assert rs._material(d).flexure.layup.material == "steel_301_fh"


_BASE = (0.05e-3, 0.075e-3, 0.10e-3, 0.127e-3, 0.15e-3)


def _reload(monkeypatch, ext):
    import importlib
    import flexblade.optimize as opt
    if ext:
        monkeypatch.setenv("LAMINA_STOCK_EXT", "1")
    else:
        monkeypatch.delenv("LAMINA_STOCK_EXT", raising=False)
    return importlib.reload(opt)


def _lam_bounds(monkeypatch):
    monkeypatch.setattr(rs, "LAMINAE_VARS", True)
    V = {v.path: v for v in rs.variables("tt_strap_steelx", 10.0)}
    return V["flexure.strip_laminae"], V["flexure.lamina_idx"]


def test_lamina_stock_default_unchanged(monkeypatch):
    opt = _reload(monkeypatch, False)
    assert opt.LAMINA_STOCK == _BASE
    n, i = _lam_bounds(monkeypatch)
    assert (n.hi, i.lo, i.hi) == (12.5, -0.5, 4.5)


def test_lamina_stock_ext(monkeypatch):
    opt = _reload(monkeypatch, True)
    try:
        assert opt.LAMINA_STOCK[:5] == _BASE and opt.LAMINA_STOCK[5] == 0.038e-3
        n, i = _lam_bounds(monkeypatch)
        assert (n.hi, i.hi) == (24.5, 6.5)
        d = opt.set_design(Design(), {"flexure.lamina_idx": 5.0, "flexure.strip_laminae": 24})
        assert d.flexure.thickness == 0.038e-3 and d.flexure.strip_laminae == 24
        assert opt.get_path(d.model_dump(mode="json"), "flexure.lamina_idx") == 5.0
    finally:
        _reload(monkeypatch, False)


def _with_mat(d, m):
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(
        layup=d.flexure.layup.model_copy(update=dict(material=m))))))


def test_steel_lamina_mapping_unchanged(monkeypatch):
    """steel strips (running DEs) keep LAMINA_STOCK indices 0-4, n >= 2 and bounds 1.5..12.5."""
    opt = _reload(monkeypatch, False)
    d = rs.base_design(10.0, "tt_strap_steelx", False)[0]
    for m in ("steel_1095_shim", "steel_301_fh"):
        dm = _with_mat(d, m)
        for i, t in enumerate(_BASE):
            d2 = opt.set_design(dm, {"flexure.lamina_idx": float(i), "flexure.strip_laminae": 1.0})
            assert d2.flexure.thickness == t and d2.flexure.strip_laminae == 2
            assert opt.get_path(d2.model_dump(mode="json"), "flexure.lamina_idx") == float(i)
        assert opt.set_design(dm, {"flexure.lamina_idx": 9.0}).flexure.thickness == _BASE[-1]
        assert opt.set_design(dm, {"flexure.strip_laminae": 30}).flexure.strip_laminae == 12
    monkeypatch.setattr(rs, "MATERIAL", "steel_301_fh")
    n, i = _lam_bounds(monkeypatch)
    assert (n.lo, n.hi, i.lo, i.hi) == (1.5, 12.5, -0.5, 4.5)


def test_g10_lamina_mapping(monkeypatch):
    opt = _reload(monkeypatch, False)
    from flexblade.materials import SHIM_THICKNESSES, G10_STOCK
    assert opt.G10_STOCK == G10_STOCK == SHIM_THICKNESSES["fr4_g10"] == (1e-4, 2e-4, 3e-4, 4e-4, 5e-4, 8e-4)
    d = _with_mat(rs.base_design(10.0, "tt_strap_steelx", False)[0], "fr4_g10")
    for i, t in enumerate(G10_STOCK):
        d2 = opt.set_design(d, {"flexure.lamina_idx": i + 0.3})
        assert d2.flexure.thickness == t
        assert opt.get_path(d2.model_dump(mode="json"), "flexure.lamina_idx") == float(i)
        assert opt.get_path(d2, "flexure.lamina_idx") == float(i)
    assert opt.set_design(d, {"flexure.lamina_idx": -0.5}).flexure.thickness == 1e-4
    assert opt.set_design(d, {"flexure.lamina_idx": 5.5}).flexure.thickness == 8e-4
    assert opt.LAMINA_STOCK == _BASE   # G10 has its own index space


def test_g10_single_strip_allowed(monkeypatch):
    opt = _reload(monkeypatch, False)
    d = _with_mat(rs.base_design(10.0, "tt_strap_steelx", False)[0], "fr4_g10")
    assert opt.set_design(d, {"flexure.strip_laminae": 0.6}).flexure.strip_laminae == 1
    assert opt.set_design(d, {"flexure.strip_laminae": 6.4}).flexure.strip_laminae == 6
    monkeypatch.setattr(rs, "MATERIAL", "fr4_g10")
    n, i = _lam_bounds(monkeypatch)
    assert (n.lo, n.hi, i.lo, i.hi) == (0.5, 6.5, -0.5, 5.5)


def test_stencil_304_single_strip(monkeypatch):
    opt = _reload(monkeypatch, False)
    from flexblade.materials import SHIM_THICKNESSES
    st = SHIM_THICKNESSES["steel_304_hta"]
    assert opt.STENCIL_STOCK == st == (0.8e-4, 1.0e-4, 1.2e-4, 1.3e-4, 1.5e-4, 1.8e-4, 2.0e-4)
    d = _with_mat(rs.base_design(10.0, "tt_strap_steelx", False)[0], "steel_304_hta")
    for i, t in enumerate(st):
        d2 = opt.set_design(d, {"flexure.lamina_idx": i + 0.3})
        assert d2.flexure.thickness == t
        assert opt.get_path(d2, "flexure.lamina_idx") == float(i)
    assert opt.set_design(d, {"flexure.strip_laminae": 0.6}).flexure.strip_laminae == 1
    assert opt.set_design(d, {"flexure.strip_laminae": 9.0}).flexure.strip_laminae == 3
    assert opt.LAMINA_STOCK == _BASE
    monkeypatch.setattr(rs, "MATERIAL", "steel_304_hta")
    n, i = _lam_bounds(monkeypatch)
    assert (n.lo, n.hi, i.lo, i.hi) == (0.5, 3.5, -0.5, 6.5)
