"""CFD: case generation (no solver run) and post-processing parsers on synthetic data."""
import math

import numpy as np
import pytest

from flexblade import cfd
from flexblade.cfd import post
from flexblade.schema import Design


def test_pitching_case_generation(tmp_path):
    c = cfd.pitching_airfoil_case(Design(), tmp_path / "p", r_frac=0.75, rpm=15000,
                                  alpha_mean_deg=5.0, n_periods=1)
    for f in ("Allrun", "mesh.msh", "0/U", "0/p", "0/k", "0/omega", "0/nut",
              "constant/dynamicMeshDict", "system/controlDict", "system/fvSchemes",
              "system/fvSolution", "system/decomposeParDict", "flexblade_case.json"):
        assert (tmp_path / "p" / f).exists(), f
    msh = (tmp_path / "p" / "mesh.msh").read_text()
    for name in ('"airfoil"', '"farfield"', '"frontAndBack"', '"fluid"'):
        assert name in msh
    assert c.n_cells > 2000
    assert c.k == pytest.approx(c.omega * c.chord / 2 / c.V)
    cd = (tmp_path / "p" / "system/controlDict").read_text()
    assert "forceCoeffs" in cd and "pimpleFoam" in cd
    dm = (tmp_path / "p" / "constant/dynamicMeshDict").read_text()
    assert "oscillatingRotatingMotion" in dm and f"{c.omega:.6g}"[:5] in dm


def test_pitching_case_laminar(tmp_path):
    cfd.pitching_airfoil_case(Design(), tmp_path / "l", rpm=15000, alpha_mean_deg=3.0,
                              turbulence="laminar")
    assert "laminar" in (tmp_path / "l/constant/turbulenceProperties").read_text()
    assert not (tmp_path / "l/0/k").exists()


def test_rotor_case_generation(tmp_path):
    r = cfd.rotor_sector_case(Design(), tmp_path / "r", rpm=15000)
    for f in ("Allrun", "system/blockMeshDict", "system/snappyHexMeshDict",
              "constant/triSurface/blade.stl", "constant/MRFProperties", "system/topoSetDict", "0/U", "0/p"):
        assert (tmp_path / "r" / f).exists(), f
    bm = (tmp_path / "r/system/blockMeshDict").read_text()
    assert bm.count("hex (") == 4 and "cyclic" in bm and "rotationAngle 180" in bm
    assert "omega" in (tmp_path / "r/constant/MRFProperties").read_text()
    assert r.sector_deg == 180.0


def _write_coeffs(path, t, cols):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        f.write("# Force coefficients\n# dragDir : (1 0 0)\n# Time Cd Cl CmPitch\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.8g}\t" + "\t".join(f"{c[i]:.8g}" for c in cols) + "\n")


def test_harmonic_fit_and_pitching_response(tmp_path):
    om = 2 * math.pi * 300.0
    t = np.linspace(0, 6 * 2 * math.pi / om, 3000)
    A = math.radians(2.0)
    H = 5.5 * np.exp(-1j * math.radians(12.0))          # Cl/α transfer, lags 12°
    alpha_c = -1j * A                                     # α = A sin ωt
    cl = 0.6 + np.real(H * alpha_c * np.exp(1j * om * t)) + 0.01 * np.cos(2 * om * t)
    cd = 0.02 + 0 * t
    cm = -0.05 + 0 * t
    _write_coeffs(tmp_path / "postProcessing/forceCoeffs/0/coefficient.dat", t, [cd, cl, cm])
    r = post.pitching_response(tmp_path, 2.0, om)
    assert r["Cl"]["mean"] == pytest.approx(0.6, abs=1e-6)
    assert r["Cl"]["H_abs"] == pytest.approx(5.5, rel=1e-6)
    assert r["Cl"]["phase_H_deg"] == pytest.approx(-12.0, abs=1e-4)


def test_read_dat_vector_format(tmp_path):
    p = tmp_path / "force.dat"
    p.write_text("# Forces\n# Time total pressure viscous\n"
                 "1 (1 2 3) (0.5 1 1.5) (0.5 1 1.5)\n2 (2 4 6) (1 2 3) (1 2 3)\n")
    names, A = post.read_dat(p)
    assert names[:4] == ["Time", "total_x", "total_y", "total_z"]
    assert A.shape == (2, 10)


def test_hover_forces(tmp_path):
    base = tmp_path / "postProcessing/forces/0"
    base.mkdir(parents=True)
    (base / "force.dat").write_text("# Time total_x total_y total_z\n"
                                    + "".join(f"{i} 0 0 3.0\n" for i in range(1, 301)))
    (base / "moment.dat").write_text("# Time total_x total_y total_z\n"
                                     + "".join(f"{i} 0 0 -0.03\n" for i in range(1, 301)))
    r = post.hover_forces(tmp_path, n_blades=2)
    assert r["thrust"] == pytest.approx(6.0) and r["torque"] == pytest.approx(0.06)


def test_theodorsen_limits():
    assert post.theodorsen_pitch_cl(0.0) == pytest.approx(2 * math.pi)
    v = post.theodorsen_pitch_cl(0.1)
    assert abs(v) < 2 * math.pi * 1.05 and np.angle(v) != 0
