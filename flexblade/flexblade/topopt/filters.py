"""Density filter, Heaviside projection and Langelaar's AM overhang filter (torch, autograd)."""
from __future__ import annotations

import math

import numpy as np
import torch
import torch.nn.functional as Fnn


class DensityFilter:
    """Linear cone ("hat") filter of radius r (m) on an anisotropic voxel grid, normalised
    by the kernel mass inside the grid: x̃ = (w * x) / (w * 1). Self-adjoint up to the
    normalisation, which autograd handles exactly."""

    def __init__(self, shape, h, radius: float, device, dtype=torch.float64):
        h = np.asarray(h, float)
        k = [int(math.floor(radius / h[d] - 1e-9)) for d in range(3)]
        ax = [np.arange(-k[d], k[d] + 1) * h[d] for d in range(3)]
        X = np.stack(np.meshgrid(*ax, indexing="ij"), -1)
        w = np.maximum(0.0, radius - np.linalg.norm(X, axis=-1))
        self.pad = tuple(k)
        self.w = torch.as_tensor(w, dtype=dtype, device=device)[None, None]
        ones = torch.ones((1, 1) + tuple(shape), dtype=dtype, device=device)
        self.norm = Fnn.conv3d(ones, self.w, padding=self.pad)[0, 0]
        self.size = tuple(2 * v + 1 for v in k)

    def __call__(self, x: torch.Tensor) -> torch.Tensor:
        if self.size == (1, 1, 1):
            return x
        return Fnn.conv3d(x[None, None], self.w, padding=self.pad)[0, 0] / self.norm


def heaviside(x: torch.Tensor, beta: float, eta: float = 0.5) -> torch.Tensor:
    if beta <= 0:
        return x
    tb = math.tanh(beta * eta)
    return (tb + torch.tanh(beta * (x - eta))) / (tb + math.tanh(beta * (1 - eta)))


def build_axis(build_dir, tol_deg: float = 2.0):
    """(axis index, sign) if build_dir is within tol of ±x/±y/±z, else None."""
    b = np.asarray(build_dir, float)
    b = b / np.linalg.norm(b)
    i = int(np.argmax(np.abs(b)))
    if abs(b[i]) < math.cos(math.radians(tol_deg)):
        return None
    return i, (1 if b[i] > 0 else -1)


def overhang_filter(x: torch.Tensor, axis: int, sign: int, P: float = 40.0, eps: float = 1e-4,
                    xi0: float = 0.5, base: torch.Tensor | None = None) -> torch.Tensor:
    """Langelaar (2016/2017) AM filter: printed density ξ_i = smin(x_i, smax(ξ of the 5
    supporting elements in the layer below)), layers stacked along `axis` in direction
    `sign`; the first layer sits on the build plate (fully supported). 45° self-supporting
    angle for cubic voxels (support stencil = element below + its 4 face neighbours).
    base: optional (layer-shaped) support of the first layer (default 1)."""
    xm = x.movedim(axis, 0)
    if sign < 0:
        xm = xm.flip(0)
    nl = xm.shape[0]
    Q = P + math.log(5) / math.log(xi0)
    out = [xm[0] if base is None else _smin(xm[0], base, eps)]
    for i in range(1, nl):
        s = out[-1]
        sp = Fnn.pad(s[None, None], (1, 1, 1, 1), value=0.0)[0, 0]
        stack = torch.stack([s, sp[:-2, 1:-1], sp[2:, 1:-1], sp[1:-1, :-2], sp[1:-1, 2:]])
        smax = (stack.clamp_min(1e-12) ** P).sum(0) ** (1.0 / Q)
        out.append(_smin(xm[i], smax, eps))
    y = torch.stack(out)
    if sign < 0:
        y = y.flip(0)
    return y.movedim(0, axis)


def _smin(x, s, eps):
    return 0.5 * (x + s - torch.sqrt((x - s) ** 2 + eps) + math.sqrt(eps))
