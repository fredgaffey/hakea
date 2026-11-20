import numpy as np
import pytest

from flexblade.bladegeom import naca4_coords
from flexblade.materials import get_material
from flexblade.schema import Layup
from flexblade.section import (CircleShape, RectShape, airfoil_shape, rect_torsion_constant,
                               section_properties)

STEEL = Layup(material="steel_302_spring")
E = get_material("steel_302_spring").E1
G = get_material("steel_302_spring").G12


@pytest.mark.parametrize("b,t", [(0.008, 0.0012), (0.004, 0.004), (0.02, 0.0005)])
def test_rect_grid_vs_closed_form(b, t):
    g = section_properties(RectShape(b, t), STEEL, None, force_grid=True)
    assert g.EA == pytest.approx(E * b * t, rel=1e-9)
    assert g.EIy == pytest.approx(E * b * t**3 / 12, rel=0.01)
    assert g.EIz == pytest.approx(E * t * b**3 / 12, rel=0.01)
    assert g.GJ == pytest.approx(G * rect_torsion_constant(b, t), rel=0.02)


def test_thin_strip_gj():
    b, t = 0.02, 0.0004
    g = section_properties(RectShape(b, t), STEEL, None, force_grid=True)
    # thin strip b t^3 G/3 with end correction (1 − 0.63 t/b)
    assert g.GJ == pytest.approx(G * b * t**3 / 3 * (1 - 0.63 * t / b), rel=0.02)


def test_circle():
    d = 0.003
    g = section_properties(CircleShape(d), STEEL, None, force_grid=True)
    assert g.EA == pytest.approx(E * np.pi * d**2 / 4, rel=0.01)
    assert g.EIy == pytest.approx(E * np.pi * d**4 / 64, rel=0.02)
    assert g.GJ == pytest.approx(G * np.pi * d**4 / 32, rel=0.02)


def test_offsets_and_mass():
    c = section_properties(RectShape(0.01, 0.002, cy=0.003, cz=-0.001), STEEL, None, force_grid=True)
    assert c.centroid == pytest.approx((0.003, -0.001), abs=1e-6)
    assert c.mass_centre == pytest.approx((0.003, -0.001), abs=1e-6)
    S = c.stiffness6()
    # axial–bending coupling from the offset: ∂N/∂κ3 = −EA·yc
    assert S[0, 5] == pytest.approx(-c.EA * 0.003, rel=1e-6)
    assert S[0, 4] == pytest.approx(c.EA * -0.001, rel=1e-6)


def test_airfoil_and_composites():
    sh = airfoil_shape(naca4_coords("4412"), 0.012, 0.25)
    p = section_properties(sh, Layup(material="tpu_64d"), None)
    assert p.area / 0.012**2 == pytest.approx(0.0822, rel=0.02)
    # NACA 4-digit area centroid ≈ 0.42 c → 2 mm behind the quarter chord (y < 0)
    assert -0.0024 < p.centroid[0] < -0.0016
    rod = section_properties(sh, Layup(kind="rod_insert", material="tpu_64d", insert_material="cf_rod_pultruded",
                                        insert_diameter=0.001, insert_positions=[0.3]), None)
    cf = get_material("cf_rod_pultruded").E1
    assert rod.EA == pytest.approx(p.EA + cf * np.pi * 0.001**2 / 4, rel=0.15)
    skin = section_properties(sh, Layup(kind="core_skin", material="tpu_64d", skin_material="pa6_cf",
                                         skin_thickness=0.0003), None)
    assert skin.EA > 20 * p.EA and skin.GJ > p.GJ
