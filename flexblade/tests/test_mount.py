import numpy as np
from flexblade.schema import Airframe, Mount
from flexblade.stability import mount_stiffness, mount_modes, make_airframe, isotropic


def test_mount_zeta_matches_strain_energy_and_series():
    mt = Mount(infill=0.4, length_mm=10)
    km = mount_stiffness(mt)
    assert abs(km - 0.4 * 6.5e6 * np.pi * (81 - 16) * 1e-6 / 0.01) < 1
    f, z, mm, zse, _, _ = mount_modes(0.075, 0.2, 0.28 * (2 * np.pi * 200) ** 2, 0.02, km, 0.15)
    assert abs(z - zse) < 0.02 * zse + 1e-4
    assert z > 0.05 and f < 0.7 * 185
    # very stiff mount -> arm damping, rigid-mount frequency
    f2, z2, *_ = mount_modes(0.075, 0.2, 0.28 * (2 * np.pi * 200) ** 2, 0.02, 1e3 * km, 0.15)
    assert abs(z2 - 0.02) < 0.003 and abs(f2 - 200) < 3


def test_mount_isotropic_airframe():
    af = Airframe(f_hz=200, f_y_hz=None, zeta=0.02, mount=Mount())
    a = make_airframe(af, 1.0, 0.03)
    assert isotropic(a)
    off = make_airframe(Airframe(f_hz=200, zeta=0.02), 1.0, 0.03)
    assert a.cx > off.cx * 0 and a.kx < off.kx
