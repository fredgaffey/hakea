import numpy as np
import pytest

from flexblade import api, presets
from flexblade.printsetup import (apply_print_setup, print_checks, region_orientations, resolve_build_dir,
                                  snap_diameter, tilt_build_dir)
from flexblade.schema import Layup, PrintSetup


@pytest.fixture(scope="module")
def tbar():
    d = presets.make_preset(5, "tpu_rod").model_copy(deep=True)
    f = d.flexure
    f.topology, f.bar_diameter, f.bar_length, f.thickness = "torsion_bar", 0.0015, 0.012, 0.0025
    f.layup = Layup(kind="solid", material="tpu_95a")
    return d


def with_ps(d, **kw):
    return d.model_copy(update=dict(print_setup=PrintSetup(**kw)))


def test_none_unchanged(tbar):
    assert apply_print_setup(tbar) is tbar


def test_presets_build_dirs(tbar):
    assert np.allclose(resolve_build_dir(with_ps(tbar, preset="flat")), [0, 0, 1])
    assert np.allclose(resolve_build_dir(with_ps(tbar, preset="on_edge")), [0, 1, 0])
    a = np.array([np.sin(np.radians(45)), 0, -np.cos(np.radians(45))])
    b = resolve_build_dir(with_ps(tbar, preset="hinge_in_plane"))
    assert abs(a @ b) < 1e-9
    assert np.allclose(tilt_build_dir(45, 0), [np.sqrt(.5), 0, np.sqrt(.5)])


def test_orientations_follow_single_build_dir(tbar):
    d = with_ps(tbar, preset="on_edge", blade_raster_deg=30.0)
    ro = region_orientations(d)
    assert np.allclose(ro["blade"].build_dir, [0, 1, 0]) and np.allclose(ro["flexure"].build_dir, [0, 1, 0])
    assert abs(np.dot(ro["blade"].raster_dir, ro["flexure"].raster_dir) - np.cos(np.radians(30))) < 1e-9
    from flexblade import fact
    mem = fact.generate_members(d)
    assert all(abs(np.dot(m.orientation.build_dir, [0, 1, 0])) > 0.999 for m in mem)


def test_interlayer_changes_with_build_dir(tbar):
    ut = {}
    for pre in ("flat", "on_edge", "hinge_in_plane"):
        r = api.evaluate(with_ps(tbar, preset=pre, install={"bar": "pause"}), ["trim", "harmonic", "checks"])
        ut[pre] = max(c.utilisation for c in r.checks if c.name == "interlayer")
    assert len({round(v, 3) for v in ut.values()}) == 3


def test_insert_in_layer_and_overhang(tbar):
    P = {"bar": "pause"}
    flat, _, _ = print_checks(with_ps(tbar, preset="flat", install=P))
    ins = [c for c in flat if c.name == "insert_in_layer"]
    assert ins and abs(ins[0].value - 45.0) < 1e-6 and ins[0].utilisation > 1       # bar 45 deg out of plane
    hip, s, _ = print_checks(with_ps(tbar, preset="hinge_in_plane", install=P))
    assert s["print_insert_err_deg"] < 1e-6 and s["print_overhang_max_deg"] < 1e-6
    assert max(c.utilisation for c in hip) <= 1.0
    # drilled: no layer-plane constraint, drill checks present
    dr, _, _ = print_checks(with_ps(tbar, preset="drilled_torsion_bar"))
    names = {c.name for c in dr}
    assert "insert_in_layer" not in names and {"drill_access", "drill_length", "stock_diameter"} <= names


def test_formed_checks(tbar):
    ch, s, w = print_checks(with_ps(tbar, preset="formed_torsion_bar"))
    assert s["print_insert_err_deg"] < 1e-6                              # as-printed in-plane
    twist = [c for c in ch if c.name == "form_insert" and c.member.endswith(".twist")]
    assert twist and twist[0].utilisation <= 1.0                          # twist axis == rod axis
    assert any(c.name == "form_bend_radius" and c.utilisation <= 1 for c in ch)
    t = [c for c in ch if c.name == "form_temp"]
    assert t and t[0].utilisation <= 1.0 and w                            # tpu_95a 100 C < rod Tg 110 C


def test_snap_diameter():
    assert abs(snap_diameter(0.0016) - 0.0015) < 1e-12 and abs(snap_diameter(0.00235) - 0.00238) < 1e-12
