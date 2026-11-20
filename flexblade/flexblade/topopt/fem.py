"""Matrix-free structured-hex elasticity on GPU/CPU (torch): operator, geometric multigrid
V-cycle preconditioner, batched PCG and a differentiable linear solve (adjoint backward).

Layout: nodal vector fields u (B, 3, X, Y, Z) (B right-hand sides); element scale fields
s (P, nx, ny, nz) — element stiffness = Σ_p s_p Ke_p (P material phases). Dirichlet nodes
(mask (X,Y,Z)) carry identity rows; all vectors keep zeros there.

Precision: operator/V-cycle in `op_dtype` (float32 on GPU), PCG vectors, dot products and
residuals in float64 ("float32 matvec with float64 accumulation").

Multigrid: re-discretised coarse levels (element counts halved; Ke for the 2^l element
size; coarse s = mean of the 8 children per phase), trilinear prolongation P, restriction
Pᵀ, damped-Jacobi smoothing, dense Cholesky on the coarsest level. Symmetric V(ν,ν) → SPD
preconditioner.
"""
from __future__ import annotations

import time

import numpy as np
import torch

from .grid import OFFSETS, hex_stiffness


def default_device() -> torch.device:
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


# =========================================================================== one level


class Level:
    def __init__(self, shape, h, Cs: list[np.ndarray], fixed: torch.Tensor, device, dtype,
                 mem_budget: float = 1.5e9):
        self.shape = tuple(int(v) for v in shape)
        self.nshape = tuple(v + 1 for v in self.shape)
        self.h = np.asarray(h, float)
        self.device, self.dtype = device, dtype
        Ke = np.stack([hex_stiffness(C, self.h) for C in Cs])            # (P,24,24)
        self.P = len(Cs)
        self.Ke64 = torch.as_tensor(Ke, dtype=torch.float64, device=device)
        self.Ke = self.Ke64.to(dtype)
        self.Kall = self.Ke.reshape(self.P * 24, 24)
        self.diagKe = torch.diagonal(self.Ke, dim1=1, dim2=2)              # (P,24)
        self.l1Ke = self.Ke.abs().sum(2)                                   # (P,24) row |·| sums
        self.l1 = True                     # ℓ1-Jacobi smoother (unconditionally convergent)
        self.omega = 1.0
        self.fixed = fixed.to(device)                                      # (X,Y,Z) bool
        self.free_f = (~self.fixed).to(dtype)
        self.ne = int(np.prod(self.shape))
        per_rhs = self.ne * 24 * (2 + self.P) * torch.finfo(dtype).bits / 8
        self.bchunk = max(1, int(mem_budget // per_rhs))
        self.s = None
        self.Kel = None          # (ne,24,24) Galerkin element matrices (coarse levels)
        self.diag = None

    # --------------------------------------------------------------- gather/scatter
    def gather(self, u):
        """(B,3,X,Y,Z) → (B,24,ne)."""
        nx, ny, nz = self.shape
        B = u.shape[0]
        ue = torch.stack([u[:, :, a:a + nx, b:b + ny, c:c + nz] for a, b, c in OFFSETS], 1)
        return ue.reshape(B, 24, self.ne)

    def scatter(self, fe):
        """(B,24,ne) → (B,3,X,Y,Z) (sum over elements)."""
        nx, ny, nz = self.shape
        B = fe.shape[0]
        fe = fe.reshape(B, 8, 3, nx, ny, nz)
        out = torch.zeros((B, 3) + self.nshape, dtype=fe.dtype, device=fe.device)
        for n, (a, b, c) in enumerate(OFFSETS):
            out[:, :, a:a + nx, b:b + ny, c:c + nz] += fe[:, n]
        return out

    def set_scales(self, s: torch.Tensor):
        self.s = s.to(self.dtype)
        self.Kel = None
        sf = self.s.reshape(self.P, 1, self.ne)
        dk = self.l1Ke if self.l1 else self.diagKe
        d = (dk[:, :, None] * sf).sum(0)                                    # (24,ne)
        self._set_diag(d)

    def set_element_matrices(self, Kel: torch.Tensor):
        self.Kel = Kel.to(self.dtype)
        d = self.Kel.abs().sum(2).T if self.l1 else torch.diagonal(self.Kel, dim1=1, dim2=2).T
        self._set_diag(d)

    def _set_diag(self, d):
        diag = self.scatter(d[None])[0]
        self.diag = torch.where(self.fixed[None], torch.ones_like(diag), diag)
        self.dinv = 1.0 / self.diag

    def lmax_jacobi(self, iters: int = 12, seed: int = 0) -> float:
        """Power-iteration estimate of λmax(D⁻¹K) (for the Jacobi damping)."""
        g = torch.Generator(device=self.device).manual_seed(seed)
        x = torch.rand((1, 3) + self.nshape, generator=g, device=self.device, dtype=self.dtype) * self.free_f
        lam = 1.0
        for _ in range(iters):
            y = self.dinv * self.matvec(x) * self.free_f
            lam = float(torch.linalg.vector_norm(y) / torch.linalg.vector_norm(x))
            x = y / torch.linalg.vector_norm(y)
        return lam

    def element_forces(self, ue, s=None, Kall=None):
        if self.Kel is not None and s is None:
            return torch.bmm(self.Kel, ue.permute(2, 1, 0)).permute(2, 1, 0)
        s = self.s if s is None else s
        Kall = self.Kall if Kall is None else Kall
        B = ue.shape[0]
        f = torch.matmul(Kall, ue).reshape(B, self.P, 24, self.ne)
        return (f * s.reshape(1, self.P, 1, self.ne)).sum(1)

    def centered(self, ue):
        """Remove the element-mean translation (B,24,ne) and cast to the level dtype."""
        B = ue.shape[0]
        ue = ue.reshape(B, 8, 3, self.ne)
        return (ue - ue.mean(1, keepdim=True)).reshape(B, 24, self.ne).to(self.dtype)

    def matvec(self, u):
        """K u in this level's dtype. If u has higher precision (float64 PCG vectors), the
        element displacement vectors are gathered in that precision and their element-mean
        translation (a rigid mode, Ke·t = 0) removed before casting: floating void regions
        carry large rigid offsets that float32 would otherwise swamp."""
        out = []
        hi = u.dtype != self.dtype
        for i in range(0, u.shape[0], self.bchunk):
            ub = u[i:i + self.bchunk]
            ue = self.gather(ub)
            if hi:
                ue = self.centered(ue)
            f = self.scatter(self.element_forces(ue)).to(u.dtype)
            out.append(f * self.free_f + ub * (1 - self.free_f))
        return torch.cat(out, 0)

    def dense(self) -> torch.Tensor:
        """Assembled float64 matrix (coarsest level), identity rows on fixed dofs."""
        X, Y, Z = self.nshape
        nn = X * Y * Z
        idx = torch.arange(nn, device=self.device).reshape(X, Y, Z)
        nx, ny, nz = self.shape
        en = torch.stack([idx[a:a + nx, b:b + ny, c:c + nz].reshape(-1) for a, b, c in OFFSETS], 1)
        ed = (torch.arange(3, device=self.device)[None, None, :] * nn + en[:, :, None]).reshape(-1, 24)
        if self.Kel is not None:
            Ke = self.Kel.to(torch.float64)
        else:
            s = self.s.to(torch.float64).reshape(self.P, -1)
            Ke = torch.einsum("pe,pij->eij", s, self.Ke64)
        N = 3 * nn
        K = torch.zeros((N * N,), dtype=torch.float64, device=self.device)
        lin = (ed[:, :, None] * N + ed[:, None, :]).reshape(-1)
        K.index_add_(0, lin, Ke.reshape(-1))
        K = K.reshape(N, N)
        fx = self.fixed.reshape(-1).repeat(3)
        K[fx, :] = 0
        K[:, fx] = 0
        K[fx, fx] = 1.0
        return K


# =========================================================================== transfer


def _prolong_axis(x, d):
    n = x.shape[d]
    a = x.narrow(d, 0, n - 1)
    b = x.narrow(d, 1, n - 1)
    inter = torch.stack([a, 0.5 * (a + b)], d + 1).flatten(d, d + 1)
    return torch.cat([inter, x.narrow(d, n - 1, 1)], d)


def _restrict_axis(x, d):
    xm = x.movedim(d, -1)
    ev = xm[..., 0::2].clone()
    od = xm[..., 1::2]
    ev[..., :-1] += 0.5 * od
    ev[..., 1:] += 0.5 * od
    return ev.movedim(-1, d)


def child_prolongation() -> np.ndarray:
    """Pc (8 children, 24 fine-element dofs, 24 coarse-element dofs): trilinear
    interpolation of the coarse element's nodes onto child (ox,oy,oz)'s nodes."""
    Pc = np.zeros((8, 24, 24))
    for ci, (ox, oy, oz) in enumerate(OFFSETS):
        for n, (a, b, c) in enumerate(OFFSETS):
            t = ((ox + a) / 2, (oy + b) / 2, (oz + c) / 2)
            for N, (A, B, C) in enumerate(OFFSETS):
                w = (t[0] if A else 1 - t[0]) * (t[1] if B else 1 - t[1]) * (t[2] if C else 1 - t[2])
                for comp in range(3):
                    Pc[ci, 3 * n + comp, 3 * N + comp] = w
    return Pc


def prolong(uc):
    for d in (2, 3, 4):
        uc = _prolong_axis(uc, d)
    return uc


def restrict(rf):
    for d in (2, 3, 4):
        rf = _restrict_axis(rf, d)
    return rf


# =========================================================================== solver


class MGSolver:
    """Hierarchy + PCG. `Cs`: list of P Voigt 6×6 phase matrices (frame H)."""

    def __init__(self, shape, h, Cs, fixed_nodes: np.ndarray, levels: int = 4, device=None,
                 op_dtype=torch.float32, nu: int = 2, omega_factor: float = 4.0 / 3.0,
                 mem_budget: float = 1.5e9, galerkin: bool = True,
                 coarse_dtype=torch.float64):
        self.device = device or default_device()
        self.op_dtype = op_dtype
        self.levels = []
        fixed = torch.as_tensor(fixed_nodes, device=self.device)
        shape = np.asarray(shape, int)
        h = np.asarray(h, float)
        for l in range(levels):
            self.levels.append(Level(shape, h, Cs, fixed, self.device,
                                     op_dtype if l == 0 else coarse_dtype, mem_budget))
            if l < levels - 1:
                if (shape % 2).any():
                    raise ValueError(f"grid {shape} not divisible for {levels} levels")
                shape = shape // 2
                h = h * 2
                # coarse fixed node = fine fixed node at even index
                fixed = fixed[::2, ::2, ::2]
        self.nu, self.omega_factor = nu, omega_factor
        self.galerkin = galerkin
        Pc = torch.as_tensor(child_prolongation(), dtype=torch.float64, device=self.device)
        self.coarse_dtype = coarse_dtype
        self.Pc = Pc.to(coarse_dtype)
        f = self.levels[0]
        # Q[c,p] = Pcᵀ Ke_p Pc for the first coarsening (from phase scales)
        self.Q = torch.einsum("cki,pkl,clj->cpij", Pc, f.Ke64, Pc).to(coarse_dtype)
        self.stats = dict(cg_iters=[], solve_time=0.0, n_solves=0)

    @property
    def fine(self) -> Level:
        return self.levels[0]

    def set_scales(self, s: torch.Tensor):
        """s (P,nx,ny,nz) fine element phase scales."""
        s = s.detach()
        Kel = None
        for l, lev in enumerate(self.levels):
            if l == 0 or not self.galerkin:
                lev.set_scales(s)
            else:
                lev.set_element_matrices(Kel)
            if l < len(self.levels) - 1:
                if not lev.l1:
                    lev.omega = self.omega_factor / (1.05 * lev.lmax_jacobi())
                if self.galerkin:
                    Kel = self._coarsen(lev, s if l == 0 else None)
                else:
                    s = torch.nn.functional.avg_pool3d(s[None].to(torch.float32), 2)[0].to(s.dtype)
        Kc = self.levels[-1].dense()
        try:
            self.Lc = torch.linalg.cholesky(Kc)
        except RuntimeError:
            Kc = Kc + 1e-10 * torch.diagonal(Kc).mean() * torch.eye(len(Kc), device=Kc.device, dtype=Kc.dtype)
            self.Lc = torch.linalg.cholesky(Kc)

    def _coarsen(self, lev: Level, s=None) -> torch.Tensor:
        """Galerkin element matrices of the next level: K_E = Σ_children Pcᵀ K_child Pc
        (exact PᵀKP, since trilinear prolongation is local to each coarse element)."""
        nx, ny, nz = lev.shape
        nc = (nx // 2) * (ny // 2) * (nz // 2)
        out = torch.zeros((nc, 24, 24), dtype=self.coarse_dtype, device=self.device)
        for ci, (ox, oy, oz) in enumerate(OFFSETS):
            if s is not None:
                sc = s[:, ox::2, oy::2, oz::2].reshape(lev.P, nc).to(self.coarse_dtype)
                out += torch.einsum("pe,pij->eij", sc, self.Q[ci])
            else:
                K = lev.Kel.reshape(nx, ny, nz, 24, 24)[ox::2, oy::2, oz::2].reshape(nc, 24, 24)
                out += self.Pc[ci].T @ K @ self.Pc[ci]
        return out

    def vcycle(self, r, l=0):
        lev = self.levels[l]
        if l == len(self.levels) - 1:
            B = r.shape[0]
            rr = r.reshape(B, -1).to(torch.float64).T
            x = torch.cholesky_solve(rr, self.Lc).T.reshape(r.shape)
            return x.to(r.dtype)
        w = lev.omega
        x = w * lev.dinv * r
        for _ in range(self.nu - 1):
            x = x + w * lev.dinv * (r - lev.matvec(x))
        res = (r - lev.matvec(x)) * lev.free_f
        nxt = self.levels[l + 1]
        rc = restrict(res).to(nxt.dtype) * nxt.free_f
        xc = self.vcycle(rc, l + 1)
        x = x + prolong(xc).to(lev.dtype) * lev.free_f
        for _ in range(self.nu):
            x = x + w * lev.dinv * (r - lev.matvec(x))
        return x

    def precond(self, r64):
        return self.vcycle(r64.to(self.op_dtype)).to(torch.float64)

    def A64(self, x64):
        return self.fine.matvec(x64.to(torch.float64))

    def solve(self, b, x0=None, tol: float = 1e-6, maxit: int = 400):
        """Batched PCG on K x = b; b (B,3,X,Y,Z) float64. Returns (x, iters, relres)."""
        t0 = time.perf_counter()
        b = b.to(torch.float64) * self.fine.free_f.to(torch.float64)
        B = b.shape[0]
        dims = tuple(range(1, 5))
        bn = torch.linalg.vector_norm(b, dim=dims)
        x = torch.zeros_like(b) if x0 is None else x0.to(torch.float64).clone() * self.fine.free_f.to(torch.float64)
        r = b - self.A64(x)
        active = bn > 0
        bn = torch.where(active, bn, torch.ones_like(bn))
        rel = torch.linalg.vector_norm(r, dim=dims) / bn
        it = 0
        if bool((rel[active] > tol).any()):
            z = self.precond(r)
            p = z.clone()
            rz = (r * z).sum(dims)
            for it in range(1, maxit + 1):
                Ap = self.A64(p)
                pAp = (p * Ap).sum(dims)
                go = (rel > tol) & active
                alpha = torch.where(go, rz / torch.where(pAp != 0, pAp, torch.ones_like(pAp)),
                                    torch.zeros_like(rz))
                x = x + alpha[:, None, None, None, None] * p
                r = r - alpha[:, None, None, None, None] * Ap
                rel = torch.linalg.vector_norm(r, dim=dims) / bn
                if not bool(((rel > tol) & active).any()):
                    break
                z = self.precond(r)
                rz_new = (r * z).sum(dims)
                beta = torch.where(go, rz_new / torch.where(rz != 0, rz, torch.ones_like(rz)),
                                   torch.zeros_like(rz))
                p = z + beta[:, None, None, None, None] * p
                rz = rz_new
        self.stats["cg_iters"].append(it)
        self.stats["n_solves"] += 1
        if self.device.type == "cuda":
            torch.cuda.synchronize()
        self.stats["solve_time"] += time.perf_counter() - t0
        rel = torch.where(active, rel, torch.zeros_like(rel))
        return x, it, rel

    # ----------------------------------------------------------------- sensitivities
    def scale_grad(self, lam, U):
        """∂(λᵀ K U)/∂s_p per element: (P,nx,ny,nz) = Σ_b λ_eᵀ Ke_p U_e (float64 accumulate)."""
        lev = self.fine
        g = torch.zeros((lev.P, lev.ne), dtype=torch.float64, device=self.device)
        for i in range(0, U.shape[0], lev.bchunk):
            ue = lev.centered(lev.gather(U[i:i + lev.bchunk].to(torch.float64)))
            le = lev.centered(lev.gather(lam[i:i + lev.bchunk].to(torch.float64)))
            B = ue.shape[0]
            f = torch.matmul(lev.Kall, ue).reshape(B, lev.P, 24, lev.ne)
            g += (f * le[:, None]).to(torch.float64).sum((0, 2))
        return g.reshape((lev.P,) + lev.shape)


class LinearSolve(torch.autograd.Function):
    """U = K(s)⁻¹ F with adjoint backward.

    backward: λ = K⁻¹ ḡ_U. ḡ_U is first projected on span(F) (least squares over the
    batch): ḡ = F A + R → λ = U A + K⁻¹ R, so compliance-type responses reuse the forward
    solutions (self-adjoint) and only the residual part costs a solve.
    ∂L/∂s_p = −Σ_b λ_bᵀ Ke_p U_b (element-wise), ∂L/∂F = λ."""

    @staticmethod
    def forward(ctx, s, F, solver: MGSolver, state: dict):
        solver.set_scales(s)
        U, it, rel = solver.solve(F, state.get("U0"), state.get("tol", 1e-6), state.get("maxit", 400))
        state["U0"] = U.detach()
        state["iters"] = it
        state["relres"] = rel.max().item()
        ctx.solver, ctx.state, ctx.sdtype = solver, state, s.dtype
        ctx.save_for_backward(U, F)
        return U

    @staticmethod
    def backward(ctx, gU):
        U, F = ctx.saved_tensors
        solver, state = ctx.solver, ctx.state
        free = solver.fine.free_f.to(torch.float64)
        gU = gU.to(torch.float64) * free
        B = F.shape[0]
        Fm = F.reshape(B, -1)
        Gm = gU.reshape(gU.shape[0], -1)
        nrm = torch.linalg.vector_norm(Fm, dim=1).clamp_min(1e-300)
        Fn = Fm / nrm[:, None]
        A = (torch.linalg.pinv(Fn @ Fn.T, hermitian=True) @ (Fn @ Gm.T)) / nrm[:, None]                 # (B,Bg)
        R = Gm - A.T @ Fm
        lam = (A.T @ U.reshape(B, -1)).reshape(gU.shape)
        rn = torch.linalg.vector_norm(R, dim=1)
        gn = torch.linalg.vector_norm(Gm, dim=1)
        need = rn > 1e-9 * torch.clamp(gn, min=1e-300)
        if bool(need.any()):
            idx = torch.nonzero(need).flatten()
            Rb = R[idx].reshape((len(idx),) + gU.shape[1:])
            x0 = state.get("L0")
            if x0 is not None and x0.shape[0] != len(idx):
                x0 = None
            lr, it, rel = solver.solve(Rb, x0, state.get("tol", 1e-6), state.get("maxit", 400))
            state["L0"] = lr.detach()
            state["adj_iters"] = it
            lam[idx] = lam[idx] + lr
        gs = -solver.scale_grad(lam, U).to(ctx.sdtype)
        return gs, lam, None, None


def linear_solve(s, F, solver, state):
    return LinearSolve.apply(s, F, solver, state)
