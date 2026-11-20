"""Validation of the independent CalculiX cross-check (flexblade/ccx.py) on closed-form cases."""
import math
import shutil

import numpy as np
import pytest

from flexblade import ccx
from flexblade import materials as M
from flexblade.schema import PrintOrientation

pytestmark = pytest.mark.skipif(shutil.which("ccx") is None and not __import__("os").path.exists(ccx.CCX),
                                reason="ccx not installed")


def _steel():
    return M._iso("steel", 7850.0, 200e9, 0.3)


def _tip_load(model, L):
    """Distribute a unit total force on the XMAX face evenly over its nodes (quadratic tets:
    corner/mid-node weighting is ignored -- error is far below the 3 % tolerance)."""
    return model.nsets["XMAX"]


def test_cantilever_tip_deflection_and_frequency(tmp_path):
    L, b, t = 100.0, 10.0, 5.0                        # mm, bending about the weak (t) axis -> z load
    mat = _steel()
    model = ccx.box_model((L, b, t), h=1.6, material=mat)
    E = mat.E1 * 1e-6
    I = b * t**3 / 12
    P = 10.0
    n = len(model.nsets["XMAX"])
    A = ccx.write_inp_model(model, tmp_path / "s", "static", nlgeom=False,
                            fixed=[("XMIN", 1, 3)], loads=[("XMAX", 3, -P / n)])
    r = ccx.run(tmp_path / "s")
    U = r["disp"][-1]
    tip = U[np.isin(U[:, 0] - 1, model.nsets["XMAX"])][:, 3].mean()
    # Timoshenko correction (shear): + P L / (kappa G A)
    G = E / 2.6
    ref = P * L**3 / (3 * E * I) + P * L / (5 / 6 * G * b * t)
    assert abs(abs(tip) - ref) / ref < 0.03, (tip, ref)

    # first bending frequency (z direction), Euler-Bernoulli
    model2 = ccx.box_model((L, b, t), h=1.6, material=mat)
    ccx.write_inp_model(model2, tmp_path / "m", "modal", rpm=None, fixed=[("XMIN", 1, 3)], n_modes=3)
    f = ccx.run(tmp_path / "m")["freqs"][0]
    rho = mat.rho * 1e-12
    m_per_len = rho * b * t * 1e0   # t/mm^2 -> t/mm... consistent mm units
    w1 = 1.8751**2 * math.sqrt(E * I / (m_per_len * L**4))
    fref = w1 / (2 * math.pi)
    assert abs(f - fref) / fref < 0.03, (f, fref)


def test_rotating_bar_centrifugal_stress(tmp_path):
    R, w_, t = 100.0, 10.0, 6.0                       # mm; bar from r=0 to R, fixed at r=0 end
    mat = _steel()
    model = ccx.box_model((R, w_, t), h=2.0, material=mat)
    # shift so bar axis passes... the bar is radial along +x, rotation axis z through origin
    model.nodes[:, 1] -= w_ / 2
    model.nodes[:, 2] -= t / 2
    rpm = 6000.0
    om = rpm * 2 * math.pi / 60
    # fix the root face only in x (Poisson-free axial), pin y/z of the axis-line nodes
    xmin = model.nsets["XMIN"]
    ccx.write_inp_model(model, tmp_path / "c", "centrifugal", rpm=rpm, nlgeom=False,
                        fixed=[("XMIN", 1, 3)])
    r = ccx.run(tmp_path / "c")
    stats = r["stress"][-1]
    eid = stats[:, 0].astype(int) - 1
    cen = model.nodes[model.elems[:, :4]].mean(1)
    sxx = stats[:, 2]
    rho = mat.rho * 1e-12
    # compare mid-span slices (away from the clamped root and the free end)
    for r0 in (35.0, 55.0, 75.0):
        sel = np.abs(cen[eid, 0] - r0) < 4.0
        ref = rho * om**2 * (R**2 - r0**2) / 2
        assert abs(sxx[sel].mean() - ref) / ref < 0.03, (r0, sxx[sel].mean(), ref)


def _axial_strain(model, res):
    """Least-squares d(ux)/dx over the mid 60 % of the block (away from clamp / load-face effects)."""
    U = res["disp"][-1]
    x = model.nodes[(U[:, 0] - 1).astype(int), 0]
    Lx = model.nodes[:, 0].max()
    sel = (x > 0.2 * Lx) & (x < 0.8 * Lx)
    return np.polyfit(x[sel], U[sel, 1], 1)[0]


def test_orthotropic_rotated_block_reproduces_E(tmp_path):
    """Uniaxial test along global x of a block whose raster axis is rotated 30 deg about z:
    E_x from ccx must equal materials.Material.axial_modulus."""
    mat = M.get_material("pa6_cf")
    th = math.radians(30.0)
    ori = PrintOrientation(build_dir=(0, 0, 1), raster_dir=(math.cos(th), math.sin(th), 0))
    L, b, t = 40.0, 8.0, 8.0
    model = ccx.box_model((L, b, t), h=3.0, material=mat, orientation=ori)
    # free lateral expansion: fix x on XMIN, pin one node's y/z to remove rigid motion
    corner = int(np.argmin(np.linalg.norm(model.nodes, axis=1)))
    model.nsets["PIN"] = np.array([corner])
    n = len(model.nsets["XMAX"])
    sig = 10.0                                         # MPa
    ccx.write_inp_model(model, tmp_path / "o", "static", nlgeom=False,
                        fixed=[("XMIN", 1, 1), ("PIN", 2, 3)],
                        loads=[("XMAX", 1, sig * b * t / n)])
    r = ccx.run(tmp_path / "o")
    Ex = sig / _axial_strain(model, r)
    # off-axis orthotropic: shear-extension coupling shears the block, but the x-slope of ux
    # over the middle of the block still gives E_x (x fixed on one face, lateral/shear free)
    Ex_ref = mat.axial_modulus(ori, (1, 0, 0)) * 1e-6
    assert abs(Ex - Ex_ref) / Ex_ref < 0.03, (Ex, Ex_ref)
    # and the unrotated block gives E1 exactly
    m0 = ccx.box_model((L, b, t), h=3.0, material=mat, orientation=PrintOrientation())
    m0.nsets["PIN"] = np.array([int(np.argmin(np.linalg.norm(m0.nodes, axis=1)))])
    ccx.write_inp_model(m0, tmp_path / "o0", "static", nlgeom=False,
                        fixed=[("XMIN", 1, 1), ("PIN", 2, 3)],
                        loads=[("XMAX", 1, sig * b * t / len(m0.nsets["XMAX"]))])
    E1 = sig / _axial_strain(m0, ccx.run(tmp_path / "o0"))
    assert abs(E1 - mat.E1 * 1e-6) / (mat.E1 * 1e-6) < 0.03, (E1, mat.E1 * 1e-6)


def test_root_stiffness_rigid_tip_bar(tmp_path):
    """Isotropic bar clamped at x=0, rigid body on the x=L face with ref node at its centre:
    K[axial]=EA/L, K[torsion]~GJ/L (square section), K[tip bending]=3EI/L^3 (z) -- the 6x6
    from the 6 unit-load cases at the free-end ref node."""
    L, b = 40.0, 4.0                                   # mm
    mat = M._iso("steel_nu0", 7850.0, 200e9, 0.0)     # nu=0: no clamp/rigid-end Poisson restraint
    model = ccx.box_model((L, b, b), h=1.0, material=mat)
    model.nodes[:, 1] -= b / 2
    model.nodes[:, 2] -= b / 2
    ccx.write_inp_model(model, tmp_path / "r", "root_stiffness", fixed=[("XMIN", 1, 3)],
                        rigid=dict(nset="XMAX", ref=(L, 0.0, 0.0)))
    K = ccx.stiffness_from_run(ccx.run(tmp_path / "r"))
    E, G = mat.E1, mat.E1 / 2.0
    A, I = (b * 1e-3) ** 2, (b * 1e-3) ** 4 / 12
    J = 0.1406 * (b * 1e-3) ** 4                       # square-section torsion constant
    Lm = L * 1e-3
    assert abs(K[0, 0] - E * A / Lm) / (E * A / Lm) < 0.03
    assert abs(K[3, 3] - G * J / Lm) / (G * J / Lm) < 0.05
    kb = 12 * E * I / Lm**3                    # rotation of the rigid end is held at 0 in K[2,2]
    assert abs(K[2, 2] - kb) / kb < 0.05       # (shear flexibility of a stubby bar softens ~2 %)


@pytest.mark.slow
def test_design_root_stiffness_smoke(tmp_path):
    """Whole pipeline on the winner design (mesh -> 1-blade root stiffness): K is symmetric
    positive definite, diagonal sensible, and the hinge-axis rotation is the softest DOF."""
    import json
    from pathlib import Path
    from flexblade.schema import Design
    f = Path(__file__).parents[1] / "study" / "designs" / "5in_tpu_rod_winner.json"
    d = Design.model_validate_json(f.read_text())
    out = ccx.check(d, rpm=18500.0, workdir=tmp_path, do_modal=False, do_centrifugal=False)
    K = np.array(out["K6"])
    assert np.all(np.linalg.eigvalsh(K) > 0)
    # N m/rad. Was ~0.32 when the hub fitting was TPU; ca47135 made it rigid PA6-CF (the soft
    # fitting was a real design flaw), so ccx now gives ~2.99. Beam model gives ~10.2 (our fea 7.7):
    # ccx/beam ~0.29, residual gap is fitting/anchorage compliance that beam/fea idealise as rigid.
    assert 1.5 < out["k_hinge_Nm_rad"] < 6.0
    assert out["k_hinge_Nm_rad"] < min(np.diag(K)[3:])
