"""Optimisation driver: seed evaluation, targets, MMA loop with β continuation, final crisp
evaluation, outputs."""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import torch

from ..schema import Design
from .mma import MMA
from .problem import Problem, TopoptOptions, layout_metrics


@dataclass
class TopoptResult:
    design_name: str
    options: dict
    K6: list                      # 6×6 at the hinge point (crisp 0/1 design), frame H
    K6_gray: list                 # at the final projected (β_max) fields
    metrics: dict                 # layout_metrics of K6
    seed_metrics: dict            # voxelised FACT seed
    fact_metrics: dict            # FACT beam model (rotor.flexure_stiffness)
    fact_K6: list
    responses: dict               # stretch, stress, volume, mass of the crisp design
    mass_props: dict              # mass, com, inertia (design region, crisp)
    targets: dict
    history: list
    timings: dict
    grid: dict
    warnings: list = field(default_factory=list)
    rho: Optional[np.ndarray] = None          # crisp (0/1) fields the K6 refers to
    phi: Optional[np.ndarray] = None
    rho_gray: Optional[np.ndarray] = None     # final projected fields (smooth iso-surfaces)
    phi_gray: Optional[np.ndarray] = None

    def summary(self) -> dict:
        return dict(name=self.design_name, k_axis=self.metrics["k_axis"],
                    stiffness_ratio=self.metrics["stiffness_ratio"],
                    axis_error_deg=self.metrics["axis_error_deg"],
                    fact_stiffness_ratio=self.fact_metrics.get("stiffness_ratio"),
                    fact_k_axis=self.fact_metrics.get("k_axis"),
                    seed_stiffness_ratio=self.seed_metrics["stiffness_ratio"],
                    stretch=self.responses["stretch"], stress_util_max=self.responses["stress_max"],
                    volume=self.responses["volume"], mass=self.mass_props["mass"],
                    iterations=len(self.history), s_per_iter=self.timings.get("s_per_iter"))

    def to_json(self) -> dict:
        d = {k: v for k, v in self.__dict__.items() if k not in ("rho", "phi", "rho_gray", "phi_gray")}
        return json.loads(json.dumps(d, default=_jsonable))


def _jsonable(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, torch.Tensor):
        return o.detach().cpu().tolist()
    return str(o)


def _f(t) -> float:
    return float(t.detach().cpu()) if isinstance(t, torch.Tensor) else float(t)


def mass_properties(prob: Problem, rho: np.ndarray, phi: np.ndarray) -> dict:
    """Mass, centre of mass and inertia (about the COM, frame H) of the design region."""
    ms, mh = prob.mats
    dm = prob.dom.design
    m_e = np.where(dm, rho * ((1 - phi) * ms.rho + phi * mh.rho), 0.0) * prob.vol_e
    X = prob.dom.elem_centers()
    M = float(m_e.sum())
    if M <= 0:
        return dict(mass=0.0, com=[0.0, 0.0, 0.0], inertia=np.zeros((3, 3)).tolist())
    c = (m_e[..., None] * X).sum((0, 1, 2)) / M
    d = X - c
    r2 = (d ** 2).sum(-1)
    I = np.einsum("xyz,xyz,ij->ij", m_e, r2, np.eye(3)) - np.einsum("xyz,xyzi,xyzj->ij", m_e, d, d)
    return dict(mass=M, com=c.tolist(), inertia=I.tolist())


def evaluate_fields(prob: Problem, rho, phi, need_stress=True) -> dict:
    T = prob.T
    with torch.no_grad():
        r = prob.responses_fields(T(rho), T(phi), need_stress)
    C = r["C"].cpu().numpy()
    K6 = np.linalg.inv(C)
    K6 = 0.5 * (K6 + K6.T)
    out = dict(K6=K6, C=C, stretch=_f(r["stretch"]), volume=_f(r["volume"]), mass=_f(r["mass"]),
               stiff_volume=_f(r["stiff_volume"]), c_soft=_f(r["c_soft"]),
               c_stiff=r["c_stiff"].cpu().numpy(), tilt=math.sqrt(_f(r["tilt2"])),
               offset=math.sqrt(_f(r["offset2"])), q_cf=r["q_cf"].cpu().numpy().tolist())
    if need_stress:
        out.update(stress_max=_f(r["stress_max"]), stress_pnorm=_f(r["stress_pnorm"]),
                   util=r["util"].cpu().numpy())
    out["metrics"] = layout_metrics(K6, prob.dom.hinge_axis, prob.dom.Lc)
    return out


def run(design: Design, opts: TopoptOptions | None = None, callback: Callable | None = None,
        problem: Problem | None = None) -> TopoptResult:
    opts = opts or TopoptOptions()
    t0 = time.perf_counter()
    prob = problem or Problem(design, opts)
    dom = prob.dom
    log = print if opts.verbose else (lambda *a, **k: None)
    log(f"[topopt] grid {dom.shape} h={np.round(dom.h * 1e3, 4)} mm, {dom.n_elem} elem, "
        f"{prob.nd} design, MG levels {dom.mg_levels}, device {prob.device}, rpm {prob.rpm:.0f}, "
        f"F_cf {prob.F_cf:.1f} N")
    # ---------------------------------------------------------------- seed / targets
    seed = evaluate_fields(prob, dom.seed_rho, dom.seed_phi)
    fact_m = prob.fact or {}
    if opts.k_hinge_target is not None:
        k_t = opts.k_hinge_target
    elif fact_m:
        k_t = fact_m["k_axis"]
    else:
        k_t = seed["metrics"]["k_axis"]
    if opts.stretch_limit is not None:
        stretch_lim = opts.stretch_limit
    elif fact_m:
        # FACT beam flexure under the outboard-blade centrifugal force (its own body load
        # is small): the parametric design's stretch is the benchmark
        stretch_lim = float((np.linalg.inv(fact_m["K6"]) @ np.r_[prob.F_cf, 0, 0, 0, 0, 0])[0])
    else:
        stretch_lim = max(seed["stretch"], 1e-7)
    ref = dict(c_stiff0=prob.T(seed["c_stiff"]), c_target=1.0 / k_t, stretch_limit=stretch_lim)
    prob.set_reference(ref)
    targets = dict(k_hinge_target=k_t, stretch_limit=stretch_lim, volfrac=prob.volfrac(),
                   axis_tol=opts.axis_tol, stress_sf=opts.stress_sf, mass_limit=opts.mass_limit,
                   c_stiff_seed=seed["c_stiff"].tolist())
    log(f"[topopt] FACT beam: k_axis {fact_m.get('k_axis', float('nan')):.4g} N m/rad, ratio "
        f"{fact_m.get('stiffness_ratio', float('nan')):.3g}; voxel seed: k_axis "
        f"{seed['metrics']['k_axis']:.4g}, ratio {seed['metrics']['stiffness_ratio']:.3g}, stretch "
        f"{seed['stretch'] * 1e6:.1f} µm, stress util {seed['stress_max']:.3g}; target k {k_t:.4g}")
    # ---------------------------------------------------------------- MMA loop
    x = prob.x0()
    n = x.numel()
    _, g0, names = prob.constraints(prob.responses(x.clone(), opts.beta0))
    mma = MMA(n, len(g0), 0.0, 1.0, move=opts.move, device=prob.device)
    beta = opts.beta0
    hist = []
    since_beta = 0
    t_loop = time.perf_counter()
    for it in range(1, opts.max_iter + 1):
        ti = time.perf_counter()
        xv = x.clone().requires_grad_(True)
        r = prob.responses(xv, beta)
        f0, g, names = prob.constraints(r)
        grads = []
        for val in [f0] + g:
            gr = torch.autograd.grad(val, xv, retain_graph=True, allow_unused=True)[0]
            grads.append(torch.zeros_like(x) if gr is None else gr)
        # adaptive p-norm correction (stress)
        if "stress_pnorm" in r:
            ratio = _f(r["stress_max"]) / max(_f(r["stress_pnorm"]), 1e-30)
            prob.stress_corr = 0.5 * prob.stress_corr + 0.5 * ratio if it > 1 else ratio
        gv = np.array([_f(v) for v in g])
        xnew = mma.update(x, _f(f0), grads[0], gv, torch.stack(grads[1:]))
        change = float((xnew - x).abs().max())
        x = xnew.detach()
        rec = dict(it=it, beta=beta, f0=_f(f0), change=change,
                   k_axis=1.0 / _f(r["c_soft"]), **{"g_" + n_: float(v) for n_, v in zip(names, gv)},
                   stress_max=_f(r.get("stress_max", 0.0)), volume=_f(r["volume"]),
                   cg_iters=prob.state.get("iters"), adj_iters=prob.state.get("adj_iters"),
                   time=time.perf_counter() - ti)
        hist.append(rec)
        if callback:
            callback(rec)
        if it % 5 == 1 or it == opts.max_iter:
            log(f"[topopt] it {it:3d} β {beta:4.0f} f0 {rec['f0']:.4f} k {rec['k_axis']:.4g} "
                f"g[{' '.join(f'{v:+.2f}' for v in gv)}] Δ {change:.3f} cg {rec['cg_iters']} "
                f"{rec['time']:.2f}s")
        since_beta += 1
        if (since_beta >= opts.beta_every or change < opts.tol_change) and beta < opts.beta_max:
            beta = min(2 * beta, opts.beta_max)
            since_beta = 0
        elif beta >= opts.beta_max and change < opts.tol_change and gv.max() < 0.02:
            break
    t_opt = time.perf_counter() - t_loop
    # ---------------------------------------------------------------- final
    with torch.no_grad():
        rho_g, phi_g = prob.fields(x, beta)
    rho_g = rho_g.cpu().numpy()
    phi_g = phi_g.cpu().numpy()
    gray = evaluate_fields(prob, rho_g, phi_g, need_stress=False)
    rho_c = (rho_g > 0.5).astype(float)
    phi_c = (phi_g > 0.5).astype(float) * rho_c
    fin = evaluate_fields(prob, rho_c, phi_c)
    mp = mass_properties(prob, rho_c, phi_c)
    timings = dict(setup=prob.setup_time, optimise=t_opt, total=time.perf_counter() - t0,
                   s_per_iter=t_opt / max(len(hist), 1),
                   gpu_mem_peak_MB=(torch.cuda.max_memory_allocated() / 1e6
                                    if prob.device.type == "cuda" else None),
                   mean_cg_iters=float(np.mean([h["cg_iters"] for h in hist])) if hist else None)
    log(f"[topopt] done {len(hist)} it, {timings['s_per_iter']:.2f} s/it; crisp design k_axis "
        f"{fin['metrics']['k_axis']:.4g} ratio {fin['metrics']['stiffness_ratio']:.3g} axis err "
        f"{fin['metrics']['axis_error_deg']:.2f}° stretch {fin['stretch'] * 1e6:.1f} µm stress "
        f"{fin['stress_max']:.3g} vol {fin['volume']:.3f}")
    return TopoptResult(
        design_name=design.name, options=opts.to_dict(), K6=fin["K6"].tolist(), K6_gray=gray["K6"].tolist(),
        metrics=fin["metrics"], seed_metrics=seed["metrics"],
        fact_metrics={k: v for k, v in fact_m.items() if k != "K6"},
        fact_K6=np.asarray(fact_m.get("K6", np.full((6, 6), np.nan))).tolist(),
        responses=dict(stretch=fin["stretch"], stress_max=fin["stress_max"], volume=fin["volume"],
                       mass=fin["mass"], stiff_volume=fin["stiff_volume"], tilt=fin["tilt"],
                       offset=fin["offset"], seed_stretch=seed["stretch"], seed_stress_max=seed["stress_max"],
                       gray_metrics=gray["metrics"]),
        mass_props=mp, targets=targets, history=hist, timings=timings,
        grid=dict(origin=dom.origin.tolist(), h=dom.h.tolist(), shape=list(dom.shape),
                  mg_levels=dom.mg_levels, n_design=prob.nd, hinge_point=dom.hinge_point.tolist(),
                  hinge_axis=dom.hinge_axis.tolist(), x_hub=dom.x_hub, x_root=dom.x_root,
                  envelope=dom.envelope),
        warnings=prob.warnings + dom.info.get("warnings", []), rho=rho_c, phi=phi_c,
        rho_gray=rho_g, phi_gray=phi_g)
