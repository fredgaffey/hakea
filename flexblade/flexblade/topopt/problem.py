"""Topology-optimisation problem: two-field multi-material SIMP on the voxel domain,
FACT-target responses with autograd + adjoint solves.

Fields (design elements only): ρ (solid density) and φ (stiff-phase fraction of the solid).
Each is density-filtered (cone, radius r ≥ min feature ≈ 2 nozzle widths) and
Heaviside-projected (β continuation, η = 0.5); optional Langelaar overhang filter on ρ.
Element stiffness (ordered two-phase SIMP):

    C_e = (ε + ρ^p (1 − φ^p)) C_soft + ρ^p φ^p C_stiff      (void = ε C_soft)

hub/plate passive: plate_factor · C_stiff. Mass density ρ((1−φ) ρ_soft + φ ρ_stiff).

Solves (one batched PCG, B = 7): six unit wrenches at the hinge point through the plate
(N = G(GᵀG)⁻¹) and the design-dependent centrifugal body force ρ_m Ω² (x, y, 0) of the
flexure itself (lumped, 1/8 per node). Load states are combinations (no extra solves):

  sustained  U_s = U_body + Σ_i W_s,i U_i     W_s: steady hinge-point wrench of the blade on
             the flexure at hover trim (root CF pull + steady moments after CF relief), from
             the rotor model with the target-stiffness hinge (loads.py)
  1/rev      U_re/im = Σ_i Re/Im(W_1p,i) U_i  complex 1/rev wrench amplitude (design torque)

Responses (C = NᵀU, 6×6 compliance at the hinge point; Ch in the hinge frame (a, x', n)):
  c_soft     = Ch[θa, θa]                         compliance about the hinge axis
  c_stiff    = Ch[t_a,t_a], Ch[t_x',t_x'], Ch[t_n,t_n], Ch[θx',θx'], Ch[θn,θn]
  tilt/offset  soft-axis direction / position error
  stretch    = (Nᵀ U_s)_x                         radial stretch at the hinge point
  stress     = p-norm of √FI_e, FI = Tsai-Hill (incl. interlayer Zt), sustained state
  creep      = p-norm of ε_eq(U_s) / creep_strain_allow(phase)              (materials.py)
  fatigue    = p-norm of √(ε_eq(U_re)² + ε_eq(U_im)²) / fatigue_strain_amp_allow(phase)
  interlayer = p-norm of √FI_il(U_s) + √(FI_il(U_re) + FI_il(U_im)), FI_il = (σ_n/Zt)² +
               (τ/S)² on the layer plane (build direction), peak of sustained + 1/rev
  volume     = mean ρ over design elements
Phase mixing of the squared utilisations: ρ^(2q) ((1−φ^p) u_soft² + φ^p u_stiff²) (qp-relaxation;
φ^p as in the stiffness, so a gray φ is judged like the phase that carries its stiffness).
Each p-norm carries an adaptive max-correction (ratio of true max to p-norm, damped).
Objective: f0 = (mean_i (c_stiff_i / c_stiff_i,seed)^4)^(1/4) (soft max of the five
stiff-direction compliances normalised by the FACT seed). Constraints (≤ 0):
  k_target/k_soft − 1, c_soft/(band c_target) − 1, (tilt² + offset²)/axis_tol² − 1,
  stretch/limit − 1, stress − 1, creep − 1, fatigue − 1, interlayer − 1,
  volume/volfrac − 1, [mass/limit − 1], [stiff-phase volume/limit − 1].
"""
from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from typing import Optional

import numpy as np
import torch

from .. import fact
from ..materials import get_material
from ..schema import Design, PrintOrientation
from .fem import MGSolver, default_device, linear_solve
from .filters import DensityFilter, build_axis, heaviside, overhang_filter
from .grid import (B_matrix, Domain, OFFSETS, build_domain, equiv_strain_form, interlayer_form,
                   tsai_hill_form)

STRAIN_CONSTRAINTS = ("creep", "fatigue", "interlayer")


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
    eps_void: float = 1e-4                 # × soft phase (TPU): void E ~ kPa; floating islands stay solvable
    plate_factor: float = 30.0             # plate/hub stiffness = factor × stiff phase
    min_feature: float = 0.8e-3            # ≈ 2 nozzle widths; filter radius = min_feature
    beta0: float = 2.0
    beta_max: float = 32.0
    beta_every: int = 30
    overhang: bool = False                 # Langelaar AM filter (build_dir must be ±x/±y/±z)
    k_hinge_target: Optional[float] = None  # N m/rad about the hinge axis; None → lag_target
    lag_target: Optional[float] = 0.7      # /rev at hover (rotor model); None → FACT k_axis
    k_hinge_band: float = 1.3              # c_soft ≤ band × target compliance
    axis_tol: float = 0.05                 # tilt (rad) / offset (fraction of Lc)
    min_ratio: Optional[float] = None      # constrained/hinge stiffness ratio floor (smooth max
                                           # of the 5 constrained compliances, transl. × 1/Lc²)
    stretch_limit: Optional[float] = None  # m; None → FACT beam under the same steady wrench
    loads: str = "rotor"                   # rotor (trim + 1/rev wrenches) | cf (CF pull only)
    stress_sf: float = 1.5
    stress_P: float = 8.0
    stress_q: float = 0.5
    strain_sf: float = 1.0                 # on creep / fatigue strain allowables
    creep: bool = True
    fatigue: bool = True
    interlayer: bool = True
    stress_exclude_layers: int = 2         # skip design layers next to hub/plate (singular corners)
    volfrac: Optional[float] = None        # of design domain; None → clip(2 × seed, .15, .4)
    mass_limit: Optional[float] = None     # kg
    stiff_frac_limit: Optional[float] = None
    rpm: Optional[float] = None            # None → rotor trim rpm (loads=rotor) / design rpm
    max_iter: int = 200
    move: float = 0.1
    move_final: float = 0.03               # MMA move limit once β = β_max (damps oscillation)
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


def intent_K6(K6_fact: np.ndarray, axis, k_hinge: float) -> np.ndarray:
    """The FACT rigid-body intent as a 6×6 at the hinge point: rotation about `axis` with
    stiffness k_hinge, the other five directions decoupled at the FACT design's constrained
    stiffness (translations λmin(Ktt), rotations the second-softest rotation of K6_fact)."""
    K = 0.5 * (np.asarray(K6_fact) + np.asarray(K6_fact).T)
    Ktt, Ktr, Krr = K[:3, :3], K[:3, 3:], K[3:, 3:]
    Kr = Krr - Ktr.T @ np.linalg.solve(Ktt, Ktr)
    k_r = float(np.linalg.eigvalsh(0.5 * (Kr + Kr.T))[1])
    k_t = float(np.linalg.eigvalsh(Ktt).min())
    a = np.asarray(axis, float)
    xp = np.array([1.0, 0, 0]) - a * a[0]
    xp /= np.linalg.norm(xp)
    R = np.column_stack([a, xp, np.cross(a, xp)])
    T = np.block([[R, np.zeros((3, 3))], [np.zeros((3, 3)), R]])
    return T @ np.diag([k_t, k_t, k_t, k_hinge, max(k_r, 5 * k_hinge), max(k_r, 5 * k_hinge)]) @ T.T


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
    # stiffness of the five constrained directions, measured against the hinge: the
    # softest direction orthogonal (in compliance) to the hinge rotation
    k_axis = float(1.0 / (e @ C @ e))
    return dict(k_axis=k_axis, k_soft=float(lam[0]), k_rot2=float(lam[1]),
                k_trans_Lc2=k_tt, stiffness_ratio=float(min(lam[1], k_tt) / max(lam[0], 1e-30)),
                constrained_ratio=float(min(_k_constrained(K6, a), k_tt) / k_axis),
                axis_error_deg=float(np.degrees(np.arccos(np.clip(abs(w @ a), 0, 1)))),
                axis_offset=float(np.linalg.norm(p_perp)), soft_axis=w.tolist())


def _k_constrained(K6: np.ndarray, a) -> float:
    """Softest rotation stiffness ⟂ the intended hinge axis `a` (translations free): the
    stiffest the FACT 'constrained' rotations are, independent of where the soft axis is."""
    Ktt, Ktr, Krr = K6[:3, :3], K6[:3, 3:], K6[3:, 3:]
    Kr = Krr - Ktr.T @ np.linalg.solve(Ktt, Ktr)
    Cr = np.linalg.inv(Kr)
    a = np.asarray(a, float)
    xp = np.array([1.0, 0, 0]) - a * a[0]
    xp /= np.linalg.norm(xp)
    Q = np.column_stack([xp, np.cross(a, xp)])
    # rotations ⟂ a with the a-rotation free: compliance block of the ⟂ plane
    Cp = Q.T @ Cr @ Q
    return float(1.0 / np.linalg.eigvalsh(0.5 * (Cp + Cp.T)).max())


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
                               device=dev, op_dtype=op_dtype, mid_dtype=op_dtype)
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
        R = dom.hinge_frame()
        self.T6 = T(np.block([[R, np.zeros((3, 3))], [np.zeros((3, 3)), R]]))
        # strain/stress quadratic forms per phase: FI = εᵀ M ε
        self.Bc = T(B_matrix((0.5, 0.5, 0.5), dom.h))                          # (6,24)
        names = (o.soft_material, o.stiff_material)
        self.Mfi = T(np.stack([C.T @ tsai_hill_form(mn, ori, o.stress_sf) @ C for C, mn in zip(Cs, names)]))
        self.Mcreep = T(np.stack([equiv_strain_form(0.5 if m.is_elastomer else m.nu12)
                                  / (m.creep_strain_allow / o.strain_sf) ** 2 for m in self.mats]))
        self.Mfat = T(np.stack([equiv_strain_form(0.5 if m.is_elastomer else m.nu12)
                                / (m.fatigue_strain_amp_allow / o.strain_sf) ** 2 for m in self.mats]))
        self.Mil = T(np.stack([C.T @ interlayer_form(mn, ori, o.stress_sf) @ C for C, mn in zip(Cs, names)]))
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
        self.corr = {n: 1.0 for n in ("stress",) + STRAIN_CONSTRAINTS}
        self.oh = None
        if o.overhang:
            ax = build_axis(ori.build_dir)
            if ax is None:
                self.warnings.append("overhang filter skipped: build_dir not aligned with a grid axis")
            else:
                self.oh = ax
        self.state = dict(tol=o.cg_tol, maxit=o.cg_maxit)
        # FACT reference, hinge-stiffness target and load cases
        self.fact = None
        try:
            self.fact = fact_reference(design)
        except Exception as e:  # noqa: BLE001
            self.warnings.append(f"FACT beam reference failed: {e}")
        self._targets_and_loads()
        self.Omega = self.rpm * 2 * math.pi / 60
        self.ref = None
        Lc2 = dom.Lc ** 2
        self._ratio_scale = T(np.array([1 / Lc2, 1 / Lc2, 1 / Lc2, 1.0, 1.0]))
        self.setup_time = time.perf_counter() - t0

    @property
    def stress_corr(self):
        return self.corr["stress"]

    @stress_corr.setter
    def stress_corr(self, v):
        self.corr["stress"] = v

    # ------------------------------------------------------------------ targets / loads
    def _targets_and_loads(self):
        o, dom, d = self.opts, self.dom, self.design
        a = dom.hinge_axis
        self.lag_info = None
        Kf = self.fact["K6"] if self.fact else None
        if o.k_hinge_target is not None:
            k_t = float(o.k_hinge_target)
        elif o.lag_target is not None and Kf is not None:
            from .loads import k_hinge_for_lag
            try:
                k0 = self.fact["k_soft"]
                self.lag_info = k_hinge_for_lag(d, intent_K6(Kf, a, k0), a, o.lag_target)
                k_t = self.lag_info["k_hinge"]
            except Exception as e:  # noqa: BLE001
                self.warnings.append(f"lag-target k_hinge failed ({e}); using the FACT k_soft")
                k_t = self.fact["k_soft"]
        elif self.fact:
            k_t = self.fact["k_axis"]
        else:
            k_t = None                         # → voxel seed (run.py)
        self.k_target = k_t
        self.loads_info = {}
        Ws = W1 = None
        rpm = None
        if o.loads == "rotor" and Kf is not None and k_t is not None:
            from .loads import rotor_hinge_loads
            try:
                Lt = rotor_hinge_loads(d, intent_K6(Kf, a, k_t))
                Lf = rotor_hinge_loads(d, Kf)
                Ws, W1, rpm = Lt["W_steady"], Lt["W_1p"], Lt["rpm"]
                self.loads_info = dict(target=Lt, fact=Lf)
            except Exception as e:  # noqa: BLE001
                self.warnings.append(f"rotor loads failed ({e}); CF pull only")
        elif o.loads not in ("rotor", "cf"):
            raise ValueError(f"loads {o.loads!r}: rotor | cf")
        self.rpm = float(o.rpm) if o.rpm is not None else (rpm or _rpm(d, o, self.warnings))
        Om = self.rpm * 2 * math.pi / 60
        self.F_cf = _outboard_cf(d, Om, self.warnings)
        if Ws is None:
            Ws, W1 = np.r_[self.F_cf, 0, 0, 0, 0, 0], np.zeros(6, complex)
            self.loads_info.setdefault("fact", dict(W_steady=Ws, W_1p=W1))
        self.W = self.wrenches(Ws, W1)
        lf = self.loads_info.get("fact")
        self.W_fact = self.wrenches(lf["W_steady"], lf["W_1p"]) if lf else self.W

    def wrenches(self, Ws, W1) -> dict:
        W1 = np.asarray(W1, complex)
        return dict(s=self.T(np.asarray(Ws, float)), re=self.T(W1.real), im=self.T(W1.imag),
                    Ws=np.asarray(Ws, float), W1=W1)

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

    def body_load(self, rho, phi):
        """Nodal centrifugal body force of the flexure material, ρ_m Ω² (x, y, 0)."""
        ms, mh = self.mats
        dens = rho * ((1 - phi) * ms.rho + phi * mh.rho) * self.vol_e * self.Omega ** 2 / 8.0
        fe = torch.stack([dens * self.Xc[..., 0], dens * self.Xc[..., 1], torch.zeros_like(dens)])
        out = torch.zeros((3,) + self.dom.node_shape, dtype=rho.dtype, device=rho.device)
        for a, b, c in OFFSETS:
            out = out + torch.nn.functional.pad(fe, (c, 1 - c, b, 1 - b, a, 1 - a))
        return out

    def centrifugal_load(self, rho, phi):
        """Body force + steady wrench (legacy name)."""
        return self.body_load(rho, phi) + torch.einsum("i,icxyz->cxyz", self.W["s"], self.N)

    # ------------------------------------------------------------------ responses
    def responses(self, x: torch.Tensor, beta: float, need_stress: bool = True) -> dict:
        rho, phi = self.fields(x, beta)
        return self.responses_fields(rho, phi, need_stress)

    def _pnorm(self, util2: torch.Tensor, rho, phi, name: str, out: dict):
        """util2 (2, nx,ny,nz): squared utilisation per phase → relaxed, mixed p-norm."""
        q, p = self.opts.stress_q, self.opts.penal
        fp = phi ** p                       # same phase weight as the stiffness interpolation
        u2 = rho ** (2 * q) * ((1 - fp) * util2[0] + fp * util2[1])
        util = torch.sqrt(u2[self.stress_mask].clamp_min(0) + 1e-16)
        P = self.opts.stress_P
        umax = util.max()
        out[name + "_pnorm"] = umax.detach() * ((util / umax.detach()) ** P).sum() ** (1 / P)
        out[name + "_max"] = umax.detach()
        out[name + "_field"] = torch.zeros_like(rho).masked_scatter(self.stress_mask, util.detach())

    def responses_fields(self, rho, phi, need_stress: bool = True, wrenches: dict | None = None) -> dict:
        s = self.scales(rho, phi)
        F = torch.cat([self.N, self.body_load(rho, phi)[None]])
        U = linear_solve(s, F, self.solver, self.state)
        return self.responses_from_U(rho, phi, U, need_stress, wrenches)

    def responses_from_U(self, rho, phi, U, need_stress: bool = True, wrenches: dict | None = None) -> dict:
        W = wrenches or self.W
        C = torch.einsum("bcxyz,dcxyz->bd", self.N, U[:6])
        C = 0.5 * (C + C.T)
        Ch = self.T6.T @ C @ self.T6
        out = dict(rho=rho, phi=phi, C=C, Ch=Ch)
        out["c_soft"] = Ch[3, 3]
        out["c_stiff"] = torch.stack([Ch[0, 0], Ch[1, 1], Ch[2, 2], Ch[4, 4], Ch[5, 5]])
        out["tilt2"] = (Ch[3, 4] ** 2 + Ch[3, 5] ** 2) / Ch[3, 3] ** 2
        out["offset2"] = (Ch[1, 3] ** 2 + Ch[2, 3] ** 2) / Ch[3, 3] ** 2 / self.dom.Lc ** 2
        Us = U[6] + torch.einsum("i,icxyz->cxyz", W["s"], U[:6])
        qcf = torch.einsum("bcxyz,cxyz->b", self.N, Us)
        out["stretch"] = qcf[0]
        out["q_cf"] = qcf
        dm = self.design_mask
        out["volume"] = rho[dm].sum() / self.nd
        ms, mh = self.mats
        out["mass"] = (rho * ((1 - phi) * ms.rho + phi * mh.rho))[dm].sum() * self.vol_e
        out["stiff_volume"] = (rho * phi)[dm].sum() / self.nd
        if need_stress:
            o = self.opts
            lev = self.solver.fine
            shp = (6,) + self.dom.shape

            def strain(u):
                return (self.Bc @ lev.gather(u[None].to(torch.float64))[0]).reshape(shp)

            def qf(e, M):
                return torch.einsum("axyz,pab,bxyz->pxyz", e, M, e)

            es = strain(Us)
            self._pnorm(qf(es, self.Mfi), rho, phi, "stress", out)
            fat = o.fatigue or o.interlayer
            if fat:
                ere = strain(torch.einsum("i,icxyz->cxyz", W["re"], U[:6]))
                eim = strain(torch.einsum("i,icxyz->cxyz", W["im"], U[:6]))
            if o.creep:
                self._pnorm(qf(es, self.Mcreep), rho, phi, "creep", out)
            if o.fatigue:
                self._pnorm(qf(ere, self.Mfat) + qf(eim, self.Mfat), rho, phi, "fatigue", out)
            if o.interlayer:
                fs = qf(es, self.Mil).clamp_min(0)
                fa = (qf(ere, self.Mil) + qf(eim, self.Mil)).clamp_min(0)
                self._pnorm((torch.sqrt(fs + 1e-20) + torch.sqrt(fa + 1e-20)) ** 2, rho, phi,
                            "interlayer", out)
            out["util"] = out["stress_field"]
        out["U"] = U
        return out

    # ------------------------------------------------------------------ batched adjoint
    def value_and_grads(self, x: torch.Tensor, beta: float, need_stress: bool = True):
        """Responses, (f0, g, names) and d/dx of every function with ONE forward solve and
        ONE batched adjoint solve (instead of one adjoint per function): the adjoint loads
        ∂f_j/∂U of all functions are stacked, reduced to an orthonormal basis (Gram
        eigen-decomposition; strain p-norms share a few fields), their part in span(F) is
        answered by U itself, and only the remainder is solved."""
        xv = x.detach().clone().requires_grad_(True)
        rho, phi = self.fields(xv, beta)
        s = self.scales(rho, phi)
        F = torch.cat([self.N, self.body_load(rho, phi)[None]])
        sv = self.solver
        st = self.state
        with torch.no_grad():
            sv.set_scales(s.detach())
            U, it, rel = sv.solve(F.detach(), st.get("U0"), st.get("tol", 1e-6), st.get("maxit", 400))
        st["U0"], st["iters"], st["relres"] = U.detach(), it, rel.max().item()
        Ul = U.detach().requires_grad_(True)
        r = self.responses_from_U(rho, phi, Ul, need_stress)
        f0, g, names = self.constraints(r)
        vals = [f0] + list(g)
        free = sv.fine.free_f.to(torch.float64)
        gUs = []
        for v in vals:
            gu = torch.autograd.grad(v, Ul, retain_graph=True, allow_unused=True)[0]
            gUs.append(None if gu is None else gu * free)
        lams = self._adjoint_many(gUs, U, F.detach())
        grads = []
        for v, lam in zip(vals, lams):
            if lam is None:
                gr = torch.autograd.grad(v, xv, retain_graph=True, allow_unused=True)[0]
            else:
                gs = -sv.scale_grad(lam, U).to(s.dtype)
                gr = torch.autograd.grad([v, s, F], xv, grad_outputs=[torch.ones_like(v), gs, lam],
                                         retain_graph=True, allow_unused=True)[0]
            grads.append(torch.zeros_like(xv) if gr is None else gr.detach())
        return r, f0, g, names, grads

    def _adjoint_many(self, gUs, U, F):
        B = F.shape[0]
        idx = [j for j, gu in enumerate(gUs) if gu is not None and bool(gu.abs().max() > 0)]
        out = [None] * len(gUs)
        if not idx:
            return out
        n = U[0].numel()
        G = torch.cat([gUs[j].reshape(B, n) for j in idx])                         # (J·B, n)
        Gram = G @ G.T
        lam_e, V = torch.linalg.eigh(Gram)
        keep = lam_e > 1e-13 * lam_e.max().clamp_min(1e-300)
        V, lam_e = V[:, keep], lam_e[keep]
        Q = (V / lam_e.sqrt()).T @ G                                                # (r, n) orthonormal
        coef = G @ Q.T                                                              # G = coef Q
        Fm = F.reshape(B, n)
        Fn = torch.linalg.vector_norm(Fm, dim=1).clamp_min(1e-300)
        Fh = Fm / Fn[:, None]
        A = torch.linalg.pinv(Fh @ Fh.T, hermitian=True) @ (Fh @ Q.T) / Fn[:, None]  # (B, r)
        R = Q - A.T @ Fm
        LQ = A.T @ U.reshape(B, n)
        rn = torch.linalg.vector_norm(R, dim=1)
        need = rn > 1e-9
        if bool(need.any()):
            ii = torch.nonzero(need).flatten()
            Rb = R[ii].reshape((len(ii),) + U.shape[1:])
            x0 = self.state.get("L0")
            if x0 is not None and x0.shape[0] != len(ii):
                x0 = None
            lr, it, rel = self.solver.solve(Rb, x0, self.state.get("tol", 1e-6), self.state.get("maxit", 400))
            self.state["L0"] = lr.detach()
            self.state["adj_iters"] = it
            self.state["adj_rhs"] = len(ii)
            LQ[ii] = LQ[ii] + lr.reshape(len(ii), n)
        lam = (coef @ LQ).reshape((len(idx), B) + U.shape[1:])
        for k, j in enumerate(idx):
            out[j] = lam[k]
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
        if o.min_ratio:
            cs = r["c_stiff"] * self._ratio_scale
            cmax = ((cs ** 8).mean()) ** (1 / 8)
            g.append(o.min_ratio * cmax / r["c_soft"] - 1.0); names.append("ratio")
        for n in ("stress",) + STRAIN_CONSTRAINTS:
            if n + "_pnorm" in r:
                g.append(self.corr[n] * r[n + "_pnorm"] - 1.0); names.append(n)
        g.append(r["volume"] / self.volfrac() - 1.0); names.append("volume")
        if o.mass_limit is not None:
            g.append(r["mass"] / o.mass_limit - 1.0); names.append("mass")
        if o.stiff_frac_limit is not None:
            g.append(r["stiff_volume"] / o.stiff_frac_limit - 1.0); names.append("stiff_volume")
        return f0, g, names

    def update_corrections(self, r: dict, first: bool):
        for n in self.corr:
            if n + "_pnorm" in r:
                ratio = float(r[n + "_max"]) / max(float(r[n + "_pnorm"]), 1e-30)
                self.corr[n] = ratio if first else 0.5 * self.corr[n] + 0.5 * ratio
