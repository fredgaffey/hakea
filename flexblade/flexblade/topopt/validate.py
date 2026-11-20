"""Independent re-analysis of a topopt result with fea.py's P2 tetrahedra.

The crisp voxel solid (design ∪ hub ∪ plate) is re-meshed into conforming quadratic
tetrahedra (6 Kuhn tets per voxel), material per voxel (soft / stiff phase, Voigt C from
``Material.stiffness_matrix``), hub voxels clamped, plate voxels tied *exactly* rigidly to
a master node at the hinge point (RBE2 by elimination, ``fea._transform``) — i.e. the same
boundary-condition model as ``fea.root_stiffness`` — and K6 = Schur complement on the
master. Differences vs. the topopt model: element (P2 tet vs. Q1 + incompatible modes),
plate (exactly rigid vs. 30× stiff phase), void (absent vs. ε).

The Schur solve uses SuperLU (fea.factorize; pypardiso if installed); "pcg" (Jacobi-PCG on
the GPU) and "ilu" (ILU-GMRES) are available but slower on these high-contrast systems."""
from __future__ import annotations

import time

import numpy as np
import scipy.sparse as sp

from .grid import OFFSETS, Domain


def voxel_tets(dom: Domain, solid: np.ndarray, coarsen: int = 1):
    """(points, tets, voxel index of each tet) for the solid voxels (6 Kuhn tets each)."""
    from .export import KUHN
    nx, ny, nz = solid.shape
    ijk = np.argwhere(solid)
    offs = np.array(OFFSETS)
    gn = ijk[:, None, :] + offs[None]                                    # (nv,8,3)
    Y, Z = ny + 1, nz + 1
    gid = (gn[..., 0] * Y + gn[..., 1]) * Z + gn[..., 2]
    tets_g = gid[:, KUHN].reshape(-1, 4)
    used, inv = np.unique(tets_g, return_inverse=True)
    tets = inv.reshape(-1, 4)
    k = used % Z
    j = (used // Z) % Y
    i = used // (Y * Z)
    pts = dom.origin[None] + np.stack([i, j, k], -1) * dom.h[None]
    vox = np.repeat(np.arange(len(ijk)), 6)
    return pts, tets, ijk, vox


def _pcg_gpu(A: sp.csr_matrix, B: np.ndarray, tol=1e-9, maxit=20000):
    import torch
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    A = A.tocsr()
    At = torch.sparse_csr_tensor(torch.as_tensor(A.indptr, dtype=torch.int64),
                                 torch.as_tensor(A.indices, dtype=torch.int64),
                                 torch.as_tensor(A.data), size=A.shape, dtype=torch.float64).to(dev)
    b = torch.as_tensor(B, dtype=torch.float64, device=dev)
    dinv = 1.0 / torch.as_tensor(A.diagonal(), device=dev)[:, None]
    x = torch.zeros_like(b)
    r = b.clone()
    z = dinv * r
    p = z.clone()
    rz = (r * z).sum(0)
    bn = torch.linalg.vector_norm(b, dim=0).clamp_min(1e-300)
    it = 0
    for it in range(1, maxit + 1):
        Ap = At @ p
        alpha = rz / (p * Ap).sum(0)
        x += alpha * p
        r -= alpha * Ap
        if bool((torch.linalg.vector_norm(r, dim=0) / bn).max() < tol):
            break
        z = dinv * r
        rz_new = (r * z).sum(0)
        p = z + (rz_new / rz) * p
        rz = rz_new
    rel = float((torch.linalg.vector_norm(r, dim=0) / bn).max())
    return x.cpu().numpy(), it, rel


def remesh_stiffness(prob, rho: np.ndarray, phi: np.ndarray, solver: str = "auto",
                     tol: float = 1e-9) -> dict:
    """K6 at the hinge point of the crisp voxel design via fea.FEModel (P2 tets)."""
    from ..fea import FEModel, _transform
    t0 = time.perf_counter()
    dom = prob.dom
    solid_design = (rho > 0.5) & dom.design
    solid = solid_design | dom.hub | dom.plate
    pts, tets, ijk, vox = voxel_tets(dom, solid)
    Cs, Ch = prob.Cs
    vi = tuple(ijk[vox].T)
    is_stiff = (phi[vi] > 0.5)
    pas = (dom.hub | dom.plate)[vi]
    C = np.where(is_stiff[:, None, None], Ch[None], Cs[None])
    C = np.where(pas[:, None, None], prob.opts.plate_factor * Ch[None], C)
    rho_m = np.where(is_stiff, prob.mats[1].rho, prob.mats[0].rho)
    model = FEModel(pts, tets, C, rho_m)
    X = model.doflocs
    tolx = 1e-9 + 1e-6 * dom.h[0]
    # hub voxels clamped; plate voxels → rigid master at the hinge point
    i_hub = np.where(dom.hub.any((1, 2)))[0]
    i_pl = np.where(dom.plate.any((1, 2)))[0]
    x_hub_end = dom.origin[0] + (i_hub.max() + 1) * dom.h[0] if len(i_hub) else dom.origin[0]
    x_pl0 = dom.origin[0] + i_pl.min() * dom.h[0]
    fixed = X[:, 0] <= x_hub_end + tolx
    slaves = X[:, 0] >= x_pl0 - tolx
    t1 = time.perf_counter()
    K = model.stiffness()
    T, md = _transform(model, fixed, slaves, dom.hinge_point)
    Kr = (T.T @ K @ T).tocsr()
    ii = np.setdiff1d(np.arange(Kr.shape[0]), md)
    Kii = Kr[ii][:, ii].tocsr()
    Kim = Kr[ii][:, md].toarray()
    Kmm = Kr[md][:, md].toarray()
    t2 = time.perf_counter()
    info = {}
    if solver == "auto":
        # SuperLU (COLAMD): ~8 s at 60k free dofs; Jacobi-PCG / ILU-GMRES were slower on
        # these high-contrast P2 systems
        solver = "splu"
    if solver == "ilu":
        import scipy.sparse.linalg as spla
        A = Kii.tocsc()
        ilu = spla.spilu(A, drop_tol=1e-5, fill_factor=8, permc_spec="COLAMD")
        M = spla.LinearOperator(A.shape, ilu.solve)
        cols, its = [], []
        for j in range(6):
            n_it = [0]
            xj, inf = spla.gmres(A, Kim[:, j], M=M, rtol=tol, restart=50, maxiter=200,
                                 callback=lambda *_: n_it.__setitem__(0, n_it[0] + 1),
                                 callback_type="pr_norm")
            cols.append(xj)
            its.append(n_it[0])
        Xs = np.column_stack(cols)
        info.update(gmres_iters=its)
    elif solver == "pcg":
        Xs, it, rel = _pcg_gpu(Kii, Kim, tol=tol)
        info.update(pcg_iters=it, pcg_relres=rel)
    else:
        from ..fea import factorize
        f = factorize(Kii.tocsc())
        Xs = np.column_stack([f(Kim[:, j]) for j in range(6)])
    K6 = Kmm - Kim.T @ Xs
    K6 = 0.5 * (K6 + K6.T)
    return dict(K6=K6, n_dof=int(model.ndof), n_tets=int(len(tets)), n_free=int(len(ii)),
                solver=solver, timings=dict(mesh=t1 - t0, assemble=t2 - t1,
                                            solve=time.perf_counter() - t2), **info)


def compare(K_a: np.ndarray, K_b: np.ndarray, axis: np.ndarray) -> dict:
    """Relative differences of the hinge-frame diagonal compliances (other dofs free)."""
    R = np.zeros((6, 6))
    a = np.asarray(axis, float)
    xp = np.array([1.0, 0, 0]) - a * a[0]
    xp /= np.linalg.norm(xp)
    Rf = np.column_stack([a, xp, np.cross(a, xp)])
    R[:3, :3] = R[3:, 3:] = Rf
    ca = np.diag(R.T @ np.linalg.inv(K_a) @ R)
    cb = np.diag(R.T @ np.linalg.inv(K_b) @ R)
    names = ["t_a", "t_x'", "t_n", "r_a(hinge)", "r_x'", "r_n"]
    return {n: float(x / y - 1.0) for n, x, y in zip(names, ca, cb)}
