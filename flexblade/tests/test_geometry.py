"""Geometry: watertight parts, layups, blade/skew symmetry, exports, solid mesh, animation."""
import json

import numpy as np
import pytest
import trimesh

from flexblade import geometry as g
from flexblade.schema import Design, HarmonicResult, Layup


def _design(**blade_kw):
    d = Design()
    if blade_kw:
        d = d.model_copy(update=dict(blade=d.blade.model_copy(update=blade_kw)))
    return d


def _check_watertight(surf):
    for p in surf["parts"]:
        m = trimesh.Trimesh(p["vertices"], p["faces"], process=True)
        assert m.is_watertight, p["name"]
        assert m.is_winding_consistent, p["name"]
        assert m.volume > 0, p["name"]


def test_default_surface_watertight_and_tagged():
    s = g.build_surface(Design())
    _check_watertight(s)
    names = [p["name"] for p in s["parts"]]
    assert "blade1" in names and "blade2" in names and "hub" in names
    assert all(p["material"] for p in s["parts"])
    # blade 2 is blade 1 rotated by pi
    b1 = next(p for p in s["parts"] if p["name"] == "blade1")["vertices"]
    b2 = next(p for p in s["parts"] if p["name"] == "blade2")["vertices"]
    R = np.diag([-1.0, -1.0, 1.0])
    assert np.allclose(b1 @ R.T, b2, atol=1e-12)


def test_skew_mirrored_on_even_blade():
    """Axis convention a = (sinδ,0,−cosδ) for blade 1 (fact.hinge_axis). Blade 2 has the
    mirrored skew in its own frame; rotated into H both axes are parallel, i.e. the top of
    blade 1's axis leans toward the hub, blade 2's toward its own tip (opposite skew)."""
    s = g.build_surface(Design())
    h = {x["blade"]: x for x in s["hinges"]}
    d1, d2 = np.asarray(h[1]["dir"]), np.asarray(h[2]["dir"])
    assert d1[0] > 0 and d1[2] < 0
    assert np.allclose(d1, d2, atol=1e-12)
    p1, p2 = np.asarray(h[1]["point"]), np.asarray(h[2]["point"])
    assert np.allclose(p2, -p1 * [1, 1, -1])


def test_core_skin_and_rod_parts():
    d = _design(layup=Layup(kind="core_skin", material="tpu_95a", skin_material="pa6_cf",
                            skin_thickness=0.0003))
    s = g.build_surface(d)
    _check_watertight(s)
    mats = {p["name"]: p["material"] for p in s["parts"]}
    assert mats["blade1_skin"] == "pa6_cf" and mats["blade1_core"] == "tpu_95a"
    v_skin = trimesh.Trimesh(*[next(p for p in s["parts"] if p["name"] == "blade1_skin")[k]
                               for k in ("vertices", "faces")]).volume
    v_core = trimesh.Trimesh(*[next(p for p in s["parts"] if p["name"] == "blade1_core")[k]
                               for k in ("vertices", "faces")]).volume
    assert 0 < v_core < v_skin
    d = _design(layup=Layup(kind="rod_insert", material="tpu_64d", insert_material="cf_rod_pultruded",
                            insert_diameter=0.0008, insert_positions=[0.3]))
    s = g.build_surface(d)
    _check_watertight(s)
    rod = next(p for p in s["parts"] if p["name"] == "blade1_rod0")
    assert rod["material"] == "cf_rod_pultruded"


@pytest.mark.parametrize("topo", ["skewed_cross_blade", "skewed_single_blade", "wire_triplet",
                                  "integral_taper", "rigid"])
def test_topologies(topo):
    d = Design()
    d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(topology=topo))))
    _check_watertight(g.build_surface(d))


def test_exports(tmp_path):
    d = Design()
    for fmt in ("glb", "stl", "obj", "json"):
        p = g.export(d, fmt, tmp_path / f"x.{fmt}")
        assert (tmp_path / f"x.{fmt}").stat().st_size > 1000, p
    js = json.loads((tmp_path / "x.json").read_text())
    assert js["parts"][0]["vertices"]
    sc = trimesh.load(tmp_path / "x.glb")
    assert len(sc.geometry) == len(js["parts"])


def test_step_export_named_mm(tmp_path):
    p = g.export(Design(), "step", tmp_path / "x.step")
    txt = open(p).read()
    assert "SI_UNIT(.MILLI.,.METRE.)" in txt
    assert "PRODUCT('blade1_tpu_64d'" in txt and "PRODUCT('hub_pa6_cf'" in txt
    import gmsh
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.occ.importShapes(p)
        gmsh.model.occ.synchronize()
        vols = gmsh.model.getEntities(3)
        assert len(vols) >= 3
        xmax = max(gmsh.model.getBoundingBox(*v)[3] for v in vols)
        assert xmax == pytest.approx(Design().blade.radius * 1000, rel=0.05)
    finally:
        gmsh.finalize()


def test_coaxial_surface():
    d = Design(layout="coaxial")
    s = g.build_surface(d)
    _check_watertight(s)
    up = next(p for p in s["parts"] if p["name"] == "upper_blade1")["vertices"]
    lo = next(p for p in s["parts"] if p["name"] == "lower_blade1")["vertices"]
    assert np.allclose(lo[:, 1], -up[:, 1]) and np.allclose(lo[:, 0], up[:, 0])
    assert np.allclose(lo[:, 2] - up[:, 2], -d.coaxial.spacing_frac * d.blade.radius)


def test_solid_mesh_root_and_tags():
    sm = g.build_solid_mesh(Design(), "root")
    assert sm.n_elements > 500
    assert set(np.unique(sm.elem_part)) <= set(range(len(sm.parts)))
    assert all(p.member >= 0 for p in sm.parts)


def test_deformed_frames_rigid_rotation():
    """Pure rigid pitch rotation θ about +x of every node -> vertex disp = θ × x."""
    d = Design()
    s = g.build_surface(d)
    R = d.blade.radius
    xs = np.linspace(0.0, R, 30)
    nodes = np.vstack([np.column_stack([xs, 0 * xs, 0 * xs]), np.column_stack([-xs, 0 * xs, 0 * xs])])
    th = 0.01
    U = np.zeros((len(nodes), 6))
    U[:, 3] = th
    U[:, 1] = 0.0
    h = HarmonicResult(torque_amp=1, hub_speed_amp_rpm=0, blades=[], hub_moment_mean=0,
                       hub_moment_phase_deg=0, hub_moment_2p=0, thrust_1p=0, moment_per_torque=0,
                       control_ratio=0, tilt_equiv_deg=0, nodes=[tuple(n) for n in nodes],
                       displacement_re=U.tolist(), displacement_im=np.zeros_like(U).tolist())
    fr = g.deformed_frames(d, h, n_frames=4, surface=s)
    p = next(q for q in fr["parts"] if q["name"] == "blade1")
    X = next(q for q in s["parts"] if q["name"] == "blade1")["vertices"]
    expect = np.cross([th, 0, 0], X)
    assert np.allclose(p["frames"][0], expect, atol=1e-9)
    assert np.allclose(p["frames"][2], -expect, atol=1e-9)


def _union_volume_mm3(design, blades="one"):
    """Independent check: OCC fuse of all raw part solids (overlaps counted once)."""
    import gmsh
    geo = g._step_geometry(design)
    parts = [p for p in geo.parts if blades == "all" or (p.blade == 1 and p.role != "spoke")]
    with g._Gmsh() as gm:
        dts = [d for p in parts for d in g.occ_part(gm.model.occ, p)]
        out, _ = gm.model.occ.fuse(dts[:1], dts[1:])
        gm.model.occ.synchronize()
        return sum(gm.model.occ.getMass(*d) for d in out if d[0] == 3) * 1e9


@pytest.mark.parametrize("layup", [
    Layup(kind="rod_insert", material="tpu_64d", insert_material="cf_rod_pultruded",
          insert_diameter=0.0008, insert_positions=[0.3]),
    Layup(kind="core_skin", material="tpu_95a", skin_material="pa6_cf", skin_thickness=0.0004),
])
def test_print_parts_watertight_disjoint(tmp_path, layup):
    d = _design(layup=layup)
    man = g.export_print_parts(d, tmp_path, blades="one", fmt="stl")
    vols = []
    for key, f in man["files"].items():
        m = trimesh.load(f)
        assert m.is_watertight, key
        assert m.bounds[0][2] >= -1e-6                       # on the bed, mm
        if key == "rod_channels":   # rods are simple solids: OCC mass is reliable here
            assert m.volume == pytest.approx(man["parts"][key]["occ_volume_mm3"], rel=0.03)
        vols.append(m.volume)
    assert len(vols) >= 2
    assert sum(vols) == pytest.approx(_union_volume_mm3(d), rel=0.02)
    if layup.kind == "rod_insert":
        assert "rod_channels" in man["files"]


def test_print_parts_3mf(tmp_path):
    import zipfile
    man = g.export_print_parts(Design(), tmp_path, blades="one", fmt="3mf")
    z = zipfile.ZipFile(man["files"]["3mf"])
    assert "3D/3dmodel.model" in z.namelist()
    assert 'name="tpu_64d"' in z.read("3D/3dmodel.model").decode()
