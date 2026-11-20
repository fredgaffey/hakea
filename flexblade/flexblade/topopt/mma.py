"""Method of Moving Asymptotes (Svanberg 1987/2002 form), written from the paper.

    min f0(x) + Σ_i (c_i y_i + ½ d_i y_i²)   s.t.  f_i(x) − y_i ≤ 0,  xmin ≤ x ≤ xmax, y ≥ 0

Each iteration builds the convex separable approximation with asymptotes L < x < U and
solves its dual in the m multipliers λ ≥ 0: for fixed λ the primal minimiser is closed
form, x_j = (L_j √P_j + U_j √Q_j)/(√P_j + √Q_j) clipped to [α_j, β_j], y_i =
max(0, (λ_i − c_i)/d_i); the concave dual W(λ) is maximised with L-BFGS-B (m is small).
Works on torch tensors (GPU) for the n-sized arrays."""
from __future__ import annotations

import numpy as np
import scipy.optimize as so
import torch


class MMA:
    def __init__(self, n: int, m: int, xmin=0.0, xmax=1.0, move: float = 0.2, asyinit: float = 0.5,
                 asyincr: float = 1.2, asydecr: float = 0.7, c: float = 1.0e3, d: float = 1.0,
                 device=None):
        self.n, self.m = n, m
        self.move, self.asyinit, self.asyincr, self.asydecr = move, asyinit, asyincr, asydecr
        self.device = device
        dt = torch.float64
        self.xmin = torch.full((n,), float(xmin), dtype=dt, device=device) if np.isscalar(xmin) else xmin
        self.xmax = torch.full((n,), float(xmax), dtype=dt, device=device) if np.isscalar(xmax) else xmax
        self.c = np.full(m, c, float)
        self.d = np.full(m, d, float)
        self.k = 0
        self.xold1 = self.xold2 = None
        self.low = self.upp = None
        self.lam = np.ones(m)

    def update(self, x, f0, df0, g, dg):
        """x (n,), f0 float, df0 (n,), g (m,) array, dg (m,n) tensor → new x (n,)."""
        x = x.detach().to(torch.float64)
        df0 = df0.detach().to(torch.float64)
        dg = dg.detach().to(torch.float64)
        g = np.asarray(g, float)
        xmin, xmax = self.xmin, self.xmax
        dx = xmax - xmin
        self.k += 1
        if self.k <= 2:
            low = x - self.asyinit * dx
            upp = x + self.asyinit * dx
        else:
            sgn = (x - self.xold1) * (self.xold1 - self.xold2)
            fac = torch.ones_like(x)
            fac[sgn > 0] = self.asyincr
            fac[sgn < 0] = self.asydecr
            low = x - fac * (self.xold1 - self.low)
            upp = x + fac * (self.upp - self.xold1)
            low = torch.minimum(torch.maximum(low, x - 10 * dx), x - 0.01 * dx)
            upp = torch.maximum(torch.minimum(upp, x + 10 * dx), x + 0.01 * dx)
        alpha = torch.maximum(torch.maximum(xmin, low + 0.1 * (x - low)), x - self.move * dx)
        beta = torch.minimum(torch.minimum(xmax, upp - 0.1 * (upp - x)), x + self.move * dx)
        ux2, xl2 = (upp - x) ** 2, (x - low) ** 2
        raa = 1e-5 / dx

        def pq(grad):
            gp, gm = grad.clamp_min(0), (-grad).clamp_min(0)
            return ux2 * (1.001 * gp + 0.001 * gm + raa), xl2 * (0.001 * gp + 1.001 * gm + raa)

        p0, q0 = pq(df0)
        Pm, Qm = pq(dg)                                                  # (m,n)
        b = ((Pm / (upp - x) + Qm / (x - low)).sum(1)).cpu().numpy() - g
        m = self.m

        def xopt(lam_t):
            P = p0 + lam_t @ Pm
            Q = q0 + lam_t @ Qm
            sP, sQ = torch.sqrt(P), torch.sqrt(Q)
            xs = (low * sP + upp * sQ) / (sP + sQ)
            return torch.minimum(torch.maximum(xs, alpha), beta), P, Q

        def negW(lam):
            lam_t = torch.as_tensor(lam, dtype=torch.float64, device=x.device)
            xs, P, Q = xopt(lam_t)
            y = np.maximum(0.0, (lam - self.c) / self.d)
            W = float((P / (upp - xs) + Q / (xs - low)).sum()) \
                + float(np.sum(self.c * y + 0.5 * self.d * y ** 2 - lam * y)) - float(lam @ b)
            gi = (Pm / (upp - xs) + Qm / (xs - low)).sum(1).cpu().numpy() - y - b
            return -W, -gi

        res = so.minimize(negW, np.maximum(self.lam, 1e-6), jac=True, method="L-BFGS-B",
                          bounds=[(0.0, None)] * m, options=dict(maxiter=500, ftol=1e-14, gtol=1e-10))
        self.lam = res.x
        xnew, _, _ = xopt(torch.as_tensor(res.x, dtype=torch.float64, device=x.device))
        self.xold2, self.xold1 = self.xold1, x.clone()
        self.low, self.upp = low, upp
        return xnew
