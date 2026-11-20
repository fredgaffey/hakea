import numpy as np
from flexblade.materials import get_material
from flexblade.schema import PrintOrientation


def test_axial_modulus_axes():
    m = get_material("pa6_cf")
    o = PrintOrientation(build_dir=(0, 0, 1), raster_dir=(1, 0, 0))
    assert np.isclose(m.axial_modulus(o, (1, 0, 0)), m.E1)
    assert np.isclose(m.axial_modulus(o, (0, 1, 0)), m.E2)
    assert np.isclose(m.axial_modulus(o, (0, 0, 1)), m.E3)


def test_stiffness_rotation_consistent():
    m = get_material("pa6_cf")
    o = PrintOrientation(build_dir=(1, 0, 0), raster_dir=(0, 0, 1))
    C = m.stiffness_matrix(o)
    S = np.linalg.inv(C)
    assert np.isclose(1 / S[2, 2], m.E1)   # raster along z
    assert np.isclose(1 / S[0, 0], m.E3)   # build along x
    assert np.allclose(C, C.T)


def test_shear_and_freq():
    m = get_material("tpu_64d")
    o = PrintOrientation()
    assert np.isclose(m.shear_modulus(o, (1, 0, 0), (0, 0, 1)), m.G13)
    assert abs(m.complex_scale(400.0)) > 1.5
