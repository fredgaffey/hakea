"""Topology optimisation: element, solver, adjoint sensitivities (FD), cantilever vs fea.py,
GPU/CPU parity, MMA, export, rotor hook."""
import math

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from flexblade.materials import get_material  # noqa: E402
from flexblade.presets import all_presets  # noqa: E402
from flexblade.schema import PrintOrientation  # noqa: E402
from flexblade.topopt import grid  # noqa: E402
from flexblade.topopt.fem import MGSolver, Level, prolong, restrict  # noqa: E402

CUDA = torch.cuda.is_available()
ORI = PrintOrientation(build_dir=(0.0, 0.3, 1.0), raster_dir=(1.0, 0.2, 0.0))


# --------------------------------------------------------------------------- element


@pytest.mark.parametrize("incompatible", [False, True])
def test_hex_element_rigid_modes_and_patch(incompatible):
    C = get_material("pa6_cf").stiffness_matrix(ORI)
    h = (1e-3, 0.7e-3, 1.3e-3)
    K = grid.hex_stiffness(C, h, incompatible)
    ev = np.linalg.eigvalsh(K)
    assert np.sum(np.abs(ev) < 1e-9 * ev.max()) == 6
    assert ev.min() > -1e-9 * ev.max()
    # constant strain → energy = ½ εᵀCε V (patch test)
    eps = np.array([1e-3, -2e-4, 3e-4, 1e-4, -2e-4, 5e-4])
    X = np.array([[a * h[0], b * h[1], c * h[2]] for a, b, c in grid.OFFSETS])
    E = np.array([[eps[0], eps[5] / 2, eps[4] / 2], [eps[5] / 2, eps[1], eps[3] / 2],
                  [eps[4] / 2, eps[3] / 2, eps[2]]])
    u = (X @ E.T).ravel()
    assert 0.5 * u @ K @ u == pytest.approx(0.5 * eps @ C @ eps * np.prod(h), rel=1e-10)


def test_incompatible_modes_remove_shear_locking():
    """Pure bending of a single element row: Q1 locks, QM6 matches beam theory closely."""
    E = 1e9
    C = np.zeros((6, 6))
    nu = 0.0
    lam, mu = E * nu / ((1 + nu) * (1 - 2 * nu)), E / (2 * (1 + nu))
    C[:3, :3] = lam
    C[np.arange(3), np.arange(3)] += 2 * mu
    C[3:, 3:] = np.eye(3) * mu
    h = (1e-3, 1e-3, 1e-3)
    ratios = {}
    for inc in (False, True):
        K = grid.hex_stiffness(C, h, inc)
        # pure bending mode about y: u_x = −κ z (x − ½), w = ½ κ x²-ish → energy ratio
        X = np.array([[a * h[0], b * h[1], c * h[2]] for a, b, c in grid.OFFSETS])
        kap = 1.0
        u = np.zeros((8, 3))
        u[:, 0] = -kap * (X[:, 2] - h[2] / 2) * (X[:, 0] - h[0] / 2)
        u[:, 2] = 0.5 * kap * (X[:, 0] - h[0] / 2) ** 2
        U = 0.5 * u.ravel() @ K @ u.ravel()
        exact = 0.5 * E * kap ** 2 * h[1] * h[2] ** 3 / 12 * h[0]
        ratios[inc] = U / exact
    assert ratios[False] > 1.5
    assert ratios[True] == pytest.approx(1.0, rel=0.02)


# --------------------------------------------------------------------------- solver


def _small_solver(device, shape=(8, 4, 4), levels=3, dtype=torch.float64, seed=0):
    Cs = [get_material("tpu_64d").stiffness_matrix(ORI), get_material("pa6_cf").stiffness_matrix(ORI)]
    fixed = np.zeros(tuple(s + 1 for s in shape), bool)
    fixed[0] = True
    S = MGSolver(shape, (2e-4, 2e-4, 2e-4), Cs, fixed, levels=levels, device=torch.device(device),
                 op_dtype=dtype)
    rng = np.random.default_rng(seed)
    s = torch.as_tensor(np.stack([0.05 + rng.random(shape), rng.random(shape) ** 3]), device=device)
    S.set_scales(s)
    return S, s


def test_transfer_operators_adjoint():
    rng = np.random.default_rng(1)
    uc = torch.as_tensor(rng.standard_normal((2, 3, 3, 4, 5)))
    rf = torch.as_tensor(rng.standard_normal((2, 3, 5, 7, 9)))
    assert float((prolong(uc) * rf).sum()) == pytest.approx(float((uc * restrict(rf)).sum()), rel=1e-12)


def test_matrix_free_vs_assembled_and_pcg():
    S, s = _small_solver("cpu")
    f = S.fine
    K = f.dense().numpy()
    rng = np.random.default_rng(2)
    u = torch.as_tensor(rng.standard_normal((1, 3) + f.nshape)) * f.free_f
    Ku = f.matvec(u)
    assert np.allclose(Ku.numpy().ravel(), K @ u.numpy().ravel(), rtol=1e-10, atol=1e-6 * np.abs(K).max())
    b = torch.zeros((2, 3) + f.nshape, dtype=torch.float64)
    b[0, 2, -1] = 1.0
    b[1, 1, -1, :, -1] = 1.0
    x, it, rel = S.solve(b, tol=1e-10, maxit=500)
    xd = np.linalg.solve(K, b.reshape(2, -1).numpy().T).T
    assert np.allclose(x.reshape(2, -1).numpy(), xd, rtol=1e-6, atol=1e-8 * np.abs(xd).max())
    assert it < 60          # MG-preconditioned


def test_galerkin_coarse_operator_exact():
    """Element-wise Galerkin coarse matrices equal Pᵀ K P of the assembled fine operator."""
    S, s = _small_solver("cpu", shape=(4, 4, 2), levels=2)
    f, c = S.levels
    Kf = f.dense().numpy()
    Kc = c.dense().numpy()
    nc = int(np.prod(c.nshape))
    P = np.zeros((3 * int(np.prod(f.nshape)), 3 * nc))
    for j in range(3 * nc):
        e = torch.zeros(3 * nc, dtype=torch.float64)
        e[j] = 1
        P[:, j] = prolong(e.reshape((1, 3) + c.nshape)).reshape(-1).numpy()
    free_f = ~np.tile(f.fixed.numpy().ravel(), 3)
    free_c = ~np.tile(c.fixed.numpy().ravel(), 3)
    G = (P[free_f][:, free_c]).T @ Kf[np.ix_(free_f, free_f)] @ P[free_f][:, free_c]
    assert np.allclose(G, Kc[np.ix_(free_c, free_c)], rtol=1e-9, atol=1e-9 * np.abs(G).max())


# --------------------------------------------------------------------------- cantilever vs fea


def _cantilever_domain(L, b, t, n):
    shape = (n * 4, max(2, n), max(2, n // 2))
    h = np.array([L / shape[0], b / shape[1], t / shape[2]])
    plate = np.zeros(shape, bool)
    plate[-2:] = True
    z = np.zeros(shape, bool)
    return grid.Domain(origin=np.array([0.0, -b / 2, -t / 2]), h=h, shape=shape,
                       hinge_point=np.array([L, 0.0, 0.0]), hinge_axis=np.array([0, 0, 1.0]),
                       x_hub=0.0, x_root=L - 2 * h[0], hub=z.copy(), plate=plate, design=~plate,
                       seed_rho=z.astype(float), seed_phi=z.astype(float), mg_levels=2)


def test_cantilever_compliance_vs_fea():
    """6×6 at the tip of a clamped PA6-CF bar: voxel model (Q1+incompatible, stiff plate)
    vs fea.py P2 tets (RBE2 tip) vs Timoshenko beam (bending)."""
    from flexblade.fea import FEModel, _transform, box_mesh, factorize
    L, b, t = 8e-3, 2e-3, 1e-3
    mat = get_material("pa6_cf")
    ori = PrintOrientation(build_dir=(0, 0, 1.0), raster_dir=(1.0, 0, 0))
    C = mat.stiffness_matrix(ori)
    dom = _cantilever_domain(L, b, t, 4)
    S = MGSolver(dom.shape, dom.h, [C], dom.fixed_nodes(), levels=2, device=torch.device("cpu"),
                 op_dtype=torch.float64)
    sc = torch.ones((1,) + dom.shape, dtype=torch.float64)
    sc[0][torch.as_tensor(dom.plate)] = 1e3
    S.set_scales(sc)
    N = torch.as_tensor(dom.rigid_load_operator())
    U, it, rel = S.solve(N, tol=1e-11, maxit=2000)
    Cv = torch.einsum("bcxyz,dcxyz->bd", N, U).numpy()
    Kv = np.linalg.inv(0.5 * (Cv + Cv.T))
    # fea: P2 tets of the same flexible length (L − plate), tip face RBE2 at x = L_f, then
    # transfer to the point (L, 0, 0) through the rigid plate
    Lf = L - 2 * dom.h[0]
    pts, tets = box_mesh(Lf, b, t, 16, 4, 2)
    pts[:, 0] += 0.0
    model = FEModel(pts, tets, np.broadcast_to(C, (len(tets), 6, 6)).copy(), np.full(len(tets), mat.rho))
    X = model.doflocs
    fixed = X[:, 0] < 1e-9
    slaves = X[:, 0] > Lf - 1e-9
    K = model.stiffness()
    T, md = _transform(model, fixed, slaves, np.array([L, 0.0, 0.0]))
    Kr = (T.T @ K @ T).tocsc()
    ii = np.setdiff1d(np.arange(Kr.shape[0]), md)
    Kim = Kr[ii][:, md].toarray()
    sol = factorize(Kr[ii][:, ii])
    Xs = np.column_stack([sol(Kim[:, j]) for j in range(6)])
    Kf = Kr[md][:, md].toarray() - Kim.T @ Xs
    Cf = np.linalg.inv(0.5 * (Kf + Kf.T))
    Cvd, Cfd = np.diag(np.linalg.inv(Kv)), np.diag(Cf)
    # axial, shear/bending (1, 2, 4, 5) within 1 % here; torsion (3) is the weakest with only
    # 2-4 voxels across the section (measured −7 %)
    for i, tol in zip(range(6), (0.02, 0.02, 0.02, 0.10, 0.02, 0.02)):
        assert Cvd[i] == pytest.approx(Cfd[i], rel=tol), (i, Cvd[i], Cfd[i])
    # Timoshenko check: tip force z at x = Lf, point at L (rigid lever): w = F Lf³/3EI (+ shear)
    E1 = mat.E1
    I = b * t ** 3 / 12
    a = L - Lf
    w_end = Lf ** 3 / (3 * E1 * I) + a * Lf ** 2 / (2 * E1 * I)
    th_end = Lf ** 2 / (2 * E1 * I) + a * Lf / (E1 * I)
    w_beam = w_end + th_end * a
    assert Cfd[2] == pytest.approx(w_beam, rel=0.1)


# --------------------------------------------------------------------------- sensitivities


def _small_problem(device="cpu", n_elem=2500, dtype="float64", **kw):
    from flexblade.topopt.problem import Problem, TopoptOptions
    d = all_presets()["5in_cf_flexure"]
    o = TopoptOptions(n_elem=n_elem, device=device, op_dtype=dtype, cg_tol=1e-12, cg_maxit=3000,
                      rpm=18000.0, min_feature=0.6e-3, verbose=False, **kw)
    P = Problem(d, o)
    return P


def _responses(P, x, beta=4.0):
    r = P.responses(x, beta)
    f0, g, names = P.constraints(r)
    return [f0] + g, ["f0"] + names


@pytest.mark.parametrize("overhang", [False, True])
def test_adjoint_sensitivities_match_finite_differences(overhang):
    P = _small_problem(overhang=overhang, build_dir=(0.0, 0.0, 1.0))
    rng = np.random.default_rng(3)
    x0 = P.x0().numpy()
    x0 = np.clip(0.5 * x0 + 0.5 * rng.random(x0.shape), 0.05, 0.95)
    r = P.responses(P.T(x0), 4.0)
    P.set_reference(dict(c_stiff0=r["c_stiff"].detach(), c_target=float(r["c_soft"]) * 1.2,
                         stretch_limit=float(r["stretch"]) * 0.9))
    P.stress_corr = 1.0
    x = P.T(x0).requires_grad_(True)
    vals, names = _responses(P, x)
    grads = [torch.autograd.grad(v, x, retain_graph=True, allow_unused=True)[0] for v in vals]
    d = rng.standard_normal(x0.shape)
    d /= np.linalg.norm(d)
    eps = 1e-4
    with torch.no_grad():
        vp, _ = _responses(P, P.T(x0 + eps * d))
        vm, _ = _responses(P, P.T(x0 - eps * d))
    for name, g, a, b in zip(names, grads, vp, vm):
        fd = (float(a) - float(b)) / (2 * eps)
        ad = float(g @ P.T(d)) if g is not None else 0.0
        assert ad == pytest.approx(fd, rel=2e-4, abs=1e-8 * max(1.0, abs(float(a)))), name


@pytest.mark.skipif(not CUDA, reason="no CUDA")
def test_gpu_cpu_parity():
    Pc = _small_problem("cpu", dtype="float64")
    Pg = _small_problem("cuda", dtype="float32")
    Pg.state["tol"] = 1e-8
    x0 = Pc.x0().numpy()
    rc = Pc.responses(Pc.T(x0), 4.0)
    rg = Pg.responses(Pg.T(x0), 4.0)
    for k in ("c_soft", "stretch", "volume", "stress_pnorm"):
        assert float(rg[k]) == pytest.approx(float(rc[k]), rel=2e-4), k
    assert np.allclose(rg["C"].cpu().numpy(), rc["C"].numpy(), rtol=1e-3,
                       atol=1e-4 * np.abs(rc["C"].numpy()).max())
    xg = Pg.T(x0).requires_grad_(True)
    xc = Pc.T(x0).requires_grad_(True)
    gg = torch.autograd.grad(Pg.responses(xg, 4.0)["c_soft"], xg)[0].cpu().numpy()
    gc = torch.autograd.grad(Pc.responses(xc, 4.0)["c_soft"], xc)[0].numpy()
    assert np.abs(gg - gc).max() < 2e-3 * np.abs(gc).max()


# --------------------------------------------------------------------------- MMA


def test_mma_constrained_quadratic():
    from flexblade.topopt.mma import MMA
    # min Σ (x − 0.8)²  s.t.  mean(x) ≤ 0.5, x ∈ [0,1] → x = 0.5
    n = 50
    x = torch.full((n,), 0.2, dtype=torch.float64)
    opt = MMA(n, 1, move=0.3)
    for _ in range(60):
        f0 = float(((x - 0.8) ** 2).sum())
        df0 = 2 * (x - 0.8)
        g = np.array([float(x.mean()) / 0.5 - 1])
        dg = torch.full((1, n), 1.0 / (0.5 * n), dtype=torch.float64)
        x = opt.update(x, f0, df0, g, dg)
    assert np.allclose(x.numpy(), 0.5, atol=2e-3)


# --------------------------------------------------------------------------- export / hooks


def test_marching_tets_watertight_and_step(tmp_path):
    from flexblade.topopt import export as ex
    g = np.indices((10, 10, 10)) - 4.5
    f = (np.sqrt((g ** 2).sum(0)) < 3.5).astype(float)
    V, F = ex.marching_tets(ex.nodal_field(f), 0.5, np.zeros(3), np.full(3, 1e-3))
    e = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), 1)
    _, cnt = np.unique(e, axis=0, return_counts=True)
    assert (cnt == 2).all()
    P = V[F]
    vol = np.einsum("ij,ij->i", P[:, 0], np.cross(P[:, 1], P[:, 2])).sum() / 6
    assert vol == pytest.approx(f.sum() * 1e-9, rel=0.15) and vol > 0
    ex.write_stl(tmp_path / "b.stl", V, F)
    import trimesh
    assert trimesh.load(tmp_path / "b.stl").is_watertight
    p = ex.write_step(tmp_path / "b.step", {"stiff": (V, F)})
    gmsh = pytest.importorskip("gmsh")
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.occ.importShapes(p)
        gmsh.model.occ.synchronize()
        assert len(gmsh.model.getEntities(2)) >= len(F) * 0.9 or len(gmsh.model.getEntities(3)) == 1
    finally:
        gmsh.finalize()


def test_voxelised_fact_seed_and_domain():
    d = all_presets()["5in_cf_flexure"]
    dom = grid.build_domain(d, 20000)
    assert dom.seed_rho.sum() > 0 and (dom.seed_phi <= dom.seed_rho).all()
    assert np.all(np.array(dom.shape) % 2 ** (dom.mg_levels - 1) == 0)
    N = dom.rigid_load_operator()
    # Nᵀ G = I: a rigid motion of the plate is read back exactly
    X = dom.node_coords()
    r = X - dom.hinge_point
    q = np.array([1e-3, -2e-3, 0.5e-3, 0.01, -0.02, 0.03])
    u = q[:3] + np.cross(q[3:], r)
    assert np.allclose(np.einsum("bcxyz,xyzc->b", N, u), q, rtol=1e-9, atol=1e-12)


def test_rotor_hook_full_K6_spring():
    """A 6×6 spring + topology 'topopt' reproduces the flexure stiffness of the FACT design
    in rotor.flexure_stiffness (both blades, mirrored), and evaluate() runs."""
    from flexblade.rotor import build_model, flexure_stiffness
    from flexblade.schema import TopoptFlexure
    d = all_presets()["5in_cf_flexure"]
    m0 = build_model(d)
    K1, K2 = flexure_stiffness(m0, blade=1), flexure_stiffness(m0, blade=2)
    d2 = d.model_copy(deep=True)
    d2.flexure.topology = "topopt"
    d2.flexure.topopt = TopoptFlexure(K6=K1.tolist(), mass=1e-4, com=(0.01, 0.0, 0.0))
    m1 = build_model(d2)
    from flexblade.topopt.problem import layout_metrics
    # the spring sits between rotor.py's stiff (1e3×) spoke/connector links, which add ~1 %
    # compliance in series
    for b, Kref in ((1, K1), (2, K2)):
        Kb = flexure_stiffness(m1, blade=b)
        assert np.allclose(np.diag(Kb), np.diag(Kref), rtol=0.03)
        a = m0.blades[b - 1].hinge_dir                 # frame H (blade 2 rotated by π)
        mb, mr = layout_metrics(Kb, a, d.flexure.length), layout_metrics(Kref, a, d.flexure.length)
        assert mb["k_axis"] == pytest.approx(mr["k_axis"], rel=0.02)
        assert mb["stiffness_ratio"] == pytest.approx(mr["stiffness_ratio"], rel=0.03)
        assert mb["axis_error_deg"] < 0.5
    from flexblade.rotor import evaluate
    r = evaluate(d2, ["trim"])
    assert r.trim is not None and r.flexure_stiffness is not None
