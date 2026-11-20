"""Topology-optimisation problem: two-field multi-material SIMP on the voxel domain,
FACT-target responses with autograd + adjoint solves.

Fields (design elements only): ρ (solid density) and φ (stiff-phase fraction of the solid).
Each is density-filtered (cone, radius r) and Heaviside-projected (β, η = 0.5); optional
Langelaar overhang filter on ρ. Element stiffness (ordered two-phase SIMP):

    C_e = (ε + ρ^p (1 − φ^p)) C_soft + ρ^p φ^p C_stiff      (void = ε C_soft)

hub/plate passive: plate_factor · C_stiff. Mass density ρ((1−φ) ρ_soft + φ ρ_stiff).

Load cases (one batched solve, B = 7): six unit wrenches at the hinge point through the
plate (N = G(GᵀG)⁻¹), and centrifugal: outboard-blade CF force (radial through the hinge
point) + design-dependent body force ρ_m Ω² (x, y, 0) (lumped, 1/8 per node).

Responses (C = NᵀU, 6×6 compliance at the hinge point; Ch in the hinge frame
(a, x', n) for both translations and rotations):
  c_soft     = Ch[θa, θa]                         compliance about the hinge axis
  c_stiff    = Ch[t_a,t_a], Ch[t_x',t_x'], Ch[t_n,t_n], Ch[θx',θx'], Ch[θn,θn]
  tilt       = |Ch[θa, θx',θn]| / c_soft           (soft-axis direction error, rad)
  offset     = |Ch[t_x',t_n ; θa]| / c_soft / Lc    (soft-axis position error / Lc)
  stretch    = (Nᵀ U_cf)_x                          radial stretch at the hinge point
  stress     = p-norm of √FI_e, FI = Tsai-Hill (incl. interlayer Zt) of the centrifugal
               case, ρ^(2q)-relaxed, phase-mixed, adaptive max-correction
  volume     = mean ρ over design elements
Objective: f0 = (mean_i (c_stiff_i / c_stiff_i,seed)^4)^(1/4) (soft max of the five
stiff-direction compliances normalised by the FACT seed). Constraints (≤ 0):
  k_target/k_soft − 1 (c_soft ≥ 1/k_target), k_soft_band: c_soft/(band c_target) − 1,
  (tilt² + offset²)/axis_tol² − 1, stretch/limit − 1, stress − 1, volume/volfrac − 1,
  [mass/limit − 1], [stiff-phase volume/limit − 1].
"""
from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field
from typing import Optional

import numpy as np
import torch

from .. import fact
from ..materials import get_material
from ..schema import Design, PrintOrientation
from .fem import MGSolver, default_device, linear_solve
from .filters import DensityFilter, build_axis, heaviside, overhang_filter
from .grid import B_matrix, Domain, OFFSETS, build_domain, tsai_hill_form


@dataclass
class TopoptOptions:
    n_elem: int = 100_000                  # target element count (ignored if h given)
    h: Optional[float] = None              # voxel size, m
    envelope: str = "box"                  # box | airfoil
    soft_material: str = "tpu_64d"
    stiff_material: str = "pa6_cf"
    build_dir: Optional[tuple] = None      # None → design.flexure.orientation
    raster_dir: Optional[tuple] = None
    penal: float = 3.0
    eps_void: float = 1e-6
    plate_factor: float = 30.0             # plate/hub stiffness = factor × stiff phase
    min_feature: float = 0.8e-3            # ≈ 2 nozzle widths; filter radius = min_feature
    beta0: float = 2.0
    beta_max: float = 32.0
    beta_every: int = 30
    overhang: bool = False                 # Langelaar AM filter (build_dir must be ±x/±y/±z)
    k_hinge_target: Optional[float] = None  # N m/rad about the hinge axis; None → FACT design
    k_hinge_band: float = 1.3              # c_soft ≤ band × target compliance
    axis_tol: float = 0.05                 # tilt (rad) / offset (fraction of Lc)
    stretch_limit: Optional[float] = None  # m; None → seed value
    stress_sf: float = 1.5
    stress_P: float = 8.0
    stress_q: float = 0.5
    stress_exclude_layers: int = 1         # skip design layers next to hub/plate (singular)
    volfrac: Optional[float] = None        # of design domain; None → clip(2 × seed, .15, .4)
    mass_limit: Optional[float] = None     # kg
    stiff_frac_limit: Optional[float] = None
    rpm: Optional[float] = None            # None → design rpm / trim
    max_iter: int = 200
    move: float = 0.1
    tol_change: float = 2e-3
    cg_tol: float = 1e-6
    cg_maxit: int = 500
    device: Optional[str] = None
    op_dtype: str = "float32"
    seed_init: bool = True                 # False → uniform volfrac init
    verbose: bool = True

    def to_dict(self):
        return asdict(self)


def _rpm(design: Design, opts: TopoptOptions, warnings: list) -> float:
    if opts.rpm is not None:
        return float(opts.rpm)
    from ..fea import _rpm_or_default
    return float(_rpm_or_default(design, None, warnings))


def _outboard_cf(design: Design, Omega: float, warnings: list) -> float:
    """Centrifugal force of the blade outboard of the flexure (N)."""
    try:
        from ..fea import _blade_outboard_mass
        m, mx = _blade_outboard_mass(design)
        return float(Omega ** 2 * mx)
    except Exception as e:  # noqa: BLE001
        warnings.append(f"outboard blade mass from beam model ({e})")
        from ..rotor import build_model
        mdl = build_model(design)
        fe = mdl.fe
        bi = mdl.blades[0]
        mx = 0.0
        for i in bi.blade_elems:
            el = fe.elements[i]
            x0, x1 = fe.X0[el.n0], fe.X0[el.n1]
            mx += el.sections[0].mass_per_len * np.linalg.norm(x1 - x0) * 0.5 * (x0[0] + x1[0])
        return float(Omega ** 2 * mx)


def fact_reference(design: Design) -> dict:
    """6×6 of the parametric FACT flexure from rotor.py's beam model (rpm 0, no prestress)."""
    from ..rotor import build_model, flexure_stiffness
    K6 = flexure_stiffness(build_model(design))
    return dict(K6=K6, **layout_metrics(K6, design))


def layout_metrics(K6: np.ndarray, design_or_axis, Lc: Optional[float] = None) -> dict:
    """fact.check_layout's metrics from a 6×6 at the hinge point: k about the target axis
    (other dofs free), softest-rotation stiffness, stiffness ratio, axis error/offset."""
    if isinstance(design_or_axis, Design):
        H0, a = fact.hinge_axis(design_or_axis)
        Lc = max(design_or_axis.flexure.length, 1e-4)
    else:
        a = np.asarray(design_or_axis, float)
    K6 = 0.5 * (np.asarray(K6) + np.asarray(K6).T)
    Ktt, Ktr, Krt, Krr = K6[:3, :3], K6[:3, 3:], K6[3:, :3], K6[3:, 3:]
    Kr = Krr - Krt @ np.linalg.solve(Ktt, Ktr)
    lam, V = np.linalg.eigh(0.5 * (Kr + Kr.T))
    w = V[:, 0] * (1 if V[:, 0] @ a >= 0 else -1)
    v = -np.linalg.solve(Ktt, Ktr @ w)
    p_rel = np.cross(w, v)
    p_perp = p_rel - a * (a @ p_rel)
    k_tt = float(np.linalg.eigvalsh(Ktt).min()) * Lc ** 2
    C = np.linalg.inv(K6)
    e = np.r_[np.zeros(3), a]
    return dict(k_axis=float(1.0 / (e @ C @ e)), k_soft=float(lam[0]), k_rot2=float(lam[1]),
                k_trans_Lc2=k_tt, stiffness_ratio=float(min(lam[1], k_tt) / max(lam[0], 1e-30)),
                axis_error_deg=float(np.degrees(np.arccos(np.clip(abs(w @ a), 0, 1)))),
                axis_offset=float(np.linalg.norm(p_perp)), soft_axis=w.tolist())


class Problem:
    def __init__(self, design: Design, opts: TopoptOptions | None = None, domain: Domain | None = None):
        self.design = design
        self.opts = o = opts or TopoptOptions()
        self.warnings: list[str] = []
        t0 = time.perf_counter()
        self.dom = dom = domain or build_domain(design, o.n_elem, o.h, o.envelope)
        self.device = torch.device(o.device) if o.device else default_device()
        dev = self.device
        ori = design.flexure.orientation
        if o.build_dir is not None or o.raster_dir is not None:
            ori = PrintOrientation(build_dir=tuple(o.build_dir or ori.build_dir),
                                   raster_dir=tuple(o.raster_dir or ori.raster_dir))
        self.orientation = ori
        ms, mh = get_material(o.soft_material), get_material(o.stiff_material)
        self.mats = (ms, mh)
        Cs = [ms.stiffness_matrix(ori), mh.stiffness_matrix(ori)]
        self.Cs = Cs
        op_dtype = getattr(torch, o.op_dtype)
        self.solver = MGSolver(dom.shape, dom.h, Cs, dom.fixed_nodes(), levels=dom.mg_levels,
                               device=dev, op_dtype=op_dtype)
        f64 = torch.float64
        T = lambda a, dt=f64: torch.as_tensor(np.ascontiguousarray(a), dtype=dt, device=dev)  # noqa: E731
        self.T = T
        self.N = T(dom.rigid_load_operator())                                  # (6,3,X,Y,Z)
        self.design_mask = T(dom.design, torch.bool)
        self.passive = T(dom.hub | dom.plate, torch.bool)
        self.nd = int(dom.design.sum())
        self.vol_e = float(np.prod(dom.h))
        Xc = dom.elem_centers()
        self.Xc = T(Xc)
        self.filt = DensityFilter(dom.shape, dom.h, max(o.min_feature, 1.01 * float(np.max(dom.h))), dev)
        # hinge frame / 6×6 rotation
        R = dom.hinge_frame()
        self.T6 = T(np.block([[R, np.zeros((3, 3))], [np.zeros((3, 3)), R]]))
        # centrifugal
        self.rpm = _rpm(design, o, self.warnings)
        self.Omega = self.rpm * 2 * math.pi / 60
        self.F_cf = _outboard_cf(design, self.Omega, self.warnings)
        # stress
        self.Bc = T(B_matrix((0.5, 0.5, 0.5), dom.h))                          # (6,24)
        # FI = εᵀ (Cᵀ M_σ C) ε  (strain-based Tsai-Hill quadratic form per phase)
        self.Mfi = T(np.stack([C.T @ tsai_hill_form(mn, ori, o.stress_sf) @ C for C, mn in
                               zip(Cs, (o.soft_material, o.stiff_material))]))
        sm = dom.design.copy()
        k = o.stress_exclude_layers
        if k > 0:
            near = np.zeros_like(sm)
            ix = np.where(dom.hub.any((1, 2)))[0]
            ip = np.where(dom.plate.any((1, 2)))[0]
            i0 = (ix.max() + 1) if len(ix) else 0
            i1 = ip.min() if len(ip) else dom.shape[0]
            near[i0:i0 + k] = True
            near[max(i1 - k, 0):i1] = True
            sm &= ~near
        self.stress_mask = T(sm, torch.bool)
        self.stress_corr = 1.0
        self.oh = None
        if o.overhang:
            ax = build_axis(ori.build_dir)
            if ax is None:
                self.warnings.append("overhang filter skipped: build_dir not aligned with a grid axis")
            else:
                self.oh = ax
        self.state = dict(tol=o.cg_tol, maxit=o.cg_maxit)
        self.setup_time = time.perf_counter() - t0
        # FACT reference and targets
        self.fact = None
        try:
            self.fact = fact_reference(design)
        except Exception as e:  # noqa: BLE001
            self.warnings.append(f"FACT beam reference failed: {e}")
        self.ref = None

    # ------------------------------------------------------------------ fields
    def x0(self) -> torch.Tensor:
        d = self.dom
        if self.opts.seed_init and d.seed_rho[d.design].sum() > 0:
            r = 0.1 + 0.8 * d.seed_rho[d.design]
            p = np.where(d.seed_rho[d.design] > 0, 0.05 + 0.9 * d.seed_phi[d.design], 0.5)
        else:
            vf = self.volfrac()
            r = np.full(self.nd, vf)
            p = np.full(self.nd, 0.5)
        return self.T(np.r_[r, p])

    def volfrac(self) -> float:
        if self.opts.volfrac is not None:
            return self.opts.volfrac
        return float(np.clip(2.0 * self.dom.info["seed_volume_fraction"], 0.15, 0.4))

    def fields(self, x: torch.Tensor, beta: float):
        """design vector → (ρ, φ) grids (projected, passive applied)."""
        nd = self.nd
        dm = self.design_mask
        zr = torch.zeros(self.dom.shape, dtype=x.dtype, device=x.device)
        rin = zr.masked_scatter(dm, x[:nd]) + self.passive.to(x.dtype)
        pin = zr.masked_scatter(dm, x[nd:]) + 0.5 * self.passive.to(x.dtype)
        rho = heaviside(self.filt(rin), beta)
        if self.oh is not None:
            rho = overhang_filter(rho, *self.oh)
        phi = heaviside(self.filt(pin), beta)
        rho = torch.where(dm, rho, torch.zeros_like(rho))
        phi = torch.where(dm, phi, torch.zeros_like(phi))
        return rho, phi

    def scales(self, rho, phi):
        p, o = self.opts.penal, self.opts
        rp = rho ** p
        fp = phi ** p
        s_soft = o.eps_void + rp * (1 - fp)
        s_stiff = rp * fp
        pas = self.passive
        s_soft = torch.where(pas, torch.zeros_like(s_soft), s_soft)
        s_stiff = torch.where(pas, torch.full_like(s_stiff, o.plate_factor), s_stiff)
        return torch.stack([s_soft, s_stiff])

    def centrifugal_load(self, rho, phi):
        ms, mh = self.mats
        dens = rho * ((1 - phi) * ms.rho + phi * mh.rho) * self.vol_e * self.Omega ** 2 / 8.0
        fe = torch.stack([dens * self.Xc[..., 0], dens * self.Xc[..., 1], torch.zeros_like(dens)])
        nx, ny, nz = self.dom.shape
        out = torch.zeros((3,) + self.dom.node_shape, dtype=rho.dtype, device=rho.device)
        for a, b, c in OFFSETS:
            out = out + torch.nn.functional.pad(fe, (c, 1 - c, b, 1 - b, a, 1 - a))
        return out + self.F_cf * self.N[0]

    # ------------------------------------------------------------------ responses
    def responses(self, x: torch.Tensor, beta: float, need_stress: bool = True) -> dict:
        rho, phi = self.fields(x, beta)
        return self.responses_fields(rho, phi, need_stress)

    def responses_fields(self, rho, phi, need_stress: bool = True) -> dict:
        s = self.scales(rho, phi)
        F = torch.cat([self.N, self.centrifugal_load(rho, phi)[None]])
        U = linear_solve(s, F, self.solver, self.state)
        C = torch.einsum("bcxyz,dcxyz->bd", self.N, U[:6])
        C = 0.5 * (C + C.T)
        Ch = self.T6.T @ C @ self.T6
        out = dict(rho=rho, phi=phi, C=C, Ch=Ch)
        out["c_soft"] = Ch[3, 3]
        out["c_stiff"] = torch.stack([Ch[0, 0], Ch[1, 1], Ch[2, 2], Ch[4, 4], Ch[5, 5]])
        out["tilt2"] = (Ch[3, 4] ** 2 + Ch[3, 5] ** 2) / Ch[3, 3] ** 2
        out["offset2"] = (Ch[1, 3] ** 2 + Ch[2, 3] ** 2) / Ch[3, 3] ** 2 / self.dom.Lc ** 2
        qcf = torch.einsum("bcxyz,cxyz->b", self.N, U[6])
        out["stretch"] = qcf[0]
        out["q_cf"] = qcf
        dm = self.design_mask
        out["volume"] = rho[dm].sum() / self.nd
        ms, mh = self.mats
        out["mass"] = (rho * ((1 - phi) * ms.rho + phi * mh.rho))[dm].sum() * self.vol_e
        out["stiff_volume"] = (rho * phi)[dm].sum() / self.nd
        if need_stress:
            lev = self.solver.fine
            ue = lev.gather(U[6:7].to(torch.float64))[0]                       # (24,ne)
            eps = (self.Bc @ ue).reshape((6,) + self.dom.shape)
            fi = torch.einsum("axyz,pab,bxyz->pxyz", eps, self.Mfi, eps)
            fi_e = rho ** (2 * self.opts.stress_q) * ((1 - phi) * fi[0] + phi * fi[1])
            util = torch.sqrt(fi_e[self.stress_mask].clamp_min(0) + 1e-16)
            P = self.opts.stress_P
            umax = util.max()
            pn = (umax.detach() * ((util / umax.detach()) ** P).sum() ** (1 / P))
            out["stress_pnorm"] = pn
            out["stress_max"] = umax.detach()
            out["util"] = torch.zeros_like(rho).masked_scatter(self.stress_mask, util.detach())
        out["U"] = U
        return out

    # ------------------------------------------------------------------ targets
    def set_reference(self, ref: dict):
        """ref: c_stiff0 (5,), c_target, stretch_limit."""
        self.ref = ref

    def constraints(self, r: dict) -> tuple[torch.Tensor, list[torch.Tensor], list[str]]:
        o, ref = self.opts, self.ref
        f0 = (((r["c_stiff"] / ref["c_stiff0"]) ** 4).mean()) ** 0.25
        g, names = [], []
        g.append(ref["c_target"] / r["c_soft"] - 1.0); names.append("hinge_soft_enough")
        g.append(r["c_soft"] / (o.k_hinge_band * ref["c_target"]) - 1.0); names.append("hinge_not_too_soft")
        g.append((r["tilt2"] + r["offset2"]) / o.axis_tol ** 2 - 1.0); names.append("axis")
        g.append(r["stretch"] / ref["stretch_limit"] - 1.0); names.append("stretch")
        if "stress_pnorm" in r:
            g.append(self.stress_corr * r["stress_pnorm"] - 1.0); names.append("stress")
        g.append(r["volume"] / self.volfrac() - 1.0); names.append("volume")
        if o.mass_limit is not None:
            g.append(r["mass"] / o.mass_limit - 1.0); names.append("mass")
        if o.stiff_frac_limit is not None:
            g.append(r["stiff_volume"] / o.stiff_frac_limit - 1.0); names.append("stiff_volume")
        return f0, g, names
