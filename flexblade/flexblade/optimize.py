"""Parametric solve of flexure/blade parameters against targets (OWNER: structures).

Variables are dotted paths into the Design (e.g. ``flexure.thickness``,
``flexure.members.0.width``); targets are dotted paths into Results
(``harmonic.control_ratio``, ``summary.lag_freq_per_rev``, ``checks.max_utilisation``
(alias of summary.max_utilisation), ``trim.coning_deg``, ``modes.2.freq_hz`` …).

Objective (minimised) = Σ weight · term, with s = |value_target| or 1:
    min:  v / s0        max: −v / s0       (s0 = |v| at the initial design, or 1)
    eq:   ((v − t)/s)²  le: (max(0, v − t)/s)²   ge: (max(0, t − v)/s)²
Failed evaluations score 1e6. Variables are optimised in normalised [0, 1] coordinates
(log-scaled if Variable.log).

Methods: differential_evolution (scipy; parallel via `workers` processes or a
`mapper`), nelder_mead, slsqp.

mapper contract (differential_evolution only):
  * map-like callable `mapper(func, iterable_of_u) -> list[float]` (scipy `workers`
    contract; func is the picklable objective on normalised u), or
  * a Design mapper: any object with `map_designs(list[Design]) -> list[dict]` returning
    records {"results": Results-json} | {"error": str} in order — e.g. an open
    `flexblade.remote.HostPool` (laptop + Mac). The u → Design adapter (path setter) and
    scoring then run in this process; every evaluation is recorded in `history`.
"""
from __future__ import annotations

import math
import os
import time
from typing import Any, Callable, Optional

import numpy as np

from .schema import Design, OptimizeRequest, OptimizeResult, Results

WHAT = ["trim", "modes", "harmonic", "checks"]
FAIL = 1.0e6

# --------------------------------------------------------------------------- dotted paths


def _set_path(d: dict, path: str, value: Any) -> None:
    keys = path.split(".")
    cur: Any = d
    for k in keys[:-1]:
        cur = cur[int(k)] if isinstance(cur, list) else cur[k]
    last = keys[-1]
    if isinstance(cur, list):
        cur[int(last)] = value
    else:
        old = cur.get(last)
        if isinstance(old, int) and not isinstance(old, bool):
            value = int(round(value))
        cur[last] = value


BAL_KEYS = ("bal.mass", "bal.r_frac", "bal.chord", "bal.normal")


def bal_to_masses(mass: float, r_frac: float, chord: float, normal: float) -> list[dict]:
    """Symmetric balance-mass pair: (+chord, +normal) and (-chord, -normal), kg / m."""
    return [dict(mass=mass, r_frac=r_frac, chord_offset=sg * chord, normal_offset=sg * normal)
            for sg in (1.0, -1.0)]


def _bal_get(obj: Any, key: str) -> float:
    bm = get_path(obj, "blade.balance_masses")
    if not bm:
        return 0.0
    b = bm[0]
    f = {"bal.mass": "mass", "bal.r_frac": "r_frac", "bal.chord": "chord_offset",
         "bal.normal": "normal_offset"}[key]
    return abs(float(b[f] if isinstance(b, dict) else getattr(b, f)))


def get_path(obj: Any, path: str) -> Any:
    if path in BAL_KEYS:
        return _bal_get(obj, path)
    if path == "flexure.lamina_idx":   # virtual DE var: index of the nearest stock lamina thickness
        t = float(get_path(obj, "flexure.thickness"))
        return float(min(range(len(LAMINA_STOCK)), key=lambda i: abs(LAMINA_STOCK[i] - t)))
    cur = obj
    for k in path.split("."):
        if isinstance(cur, list):
            cur = cur[int(k)]
        elif isinstance(cur, dict):
            cur = cur[k]
        else:
            cur = getattr(cur, k)
    return cur


LAMINA_STOCK = (0.05e-3, 0.075e-3, 0.10e-3, 0.127e-3, 0.15e-3)   # shim stock, m


def set_design(design: Design, values: dict[str, float]) -> Design:
    d = design.model_dump(mode="json")
    bal = {}
    for p, v in values.items():
        if p == "blade.chord_scale":
            k = float(v) / float(d["blade"].get("chord_scale", 1.0))
            for st in d["blade"]["stations"]:
                st["chord"] *= k
            d["blade"]["chord_scale"] = float(v)
        elif p == "flexure.lamina_idx":   # continuous index -> nearest stock lamina thickness
            i = min(max(int(round(float(v))), 0), len(LAMINA_STOCK) - 1)
            d["flexure"]["thickness"] = LAMINA_STOCK[i]
        elif p == "flexure.strip_laminae":
            d["flexure"]["strip_laminae"] = min(max(int(round(float(v))), 2), 12)
        elif p in BAL_KEYS:
            bal[p] = float(v)
        else:
            _set_path(d, p, float(v))
    if bal:
        cur = {k: _bal_get(d, k) for k in BAL_KEYS}
        cur.update(bal)
        d["blade"]["balance_masses"] = bal_to_masses(*(cur[k] for k in BAL_KEYS))
    return Design.model_validate(d)


def quantity(results: Results | dict, path: str) -> float:
    r = results.model_dump(mode="json") if isinstance(results, Results) else results
    if path in ("checks.max_utilisation", "max_utilisation"):
        path = "summary.max_utilisation"
    if path.startswith("checks.") and path.count(".") == 2:
        # checks.<name>.<member> → utilisation
        _, name, member = path.split(".")
        vals = [c["utilisation"] for c in r["checks"] if c["name"] == name and
                (member in ("*", "max") or c["member"] == member)]
        return float(max(vals)) if vals else float("nan")
    v = get_path(r, path)
    return float(v) if v is not None else float("nan")


# --------------------------------------------------------------------------- objective


class Objective:
    """Picklable objective on normalised variables u ∈ [0,1]^n."""

    def __init__(self, req: OptimizeRequest, scales0: dict[int, float] | None = None):
        self.design_json = req.design.model_dump(mode="json")
        self.variables = [v.model_dump() for v in req.variables]
        self.targets = [t.model_dump() for t in req.targets]
        self.scales0 = scales0 or {}

    def to_values(self, u) -> dict[str, float]:
        out = {}
        for ui, v in zip(np.clip(np.asarray(u, float), 0.0, 1.0), self.variables):
            if v["log"]:
                out[v["path"]] = float(math.exp(math.log(v["lo"]) + ui * (math.log(v["hi"]) - math.log(v["lo"]))))
            else:
                out[v["path"]] = float(v["lo"] + ui * (v["hi"] - v["lo"]))
        return out

    def to_u(self, values: dict[str, float]) -> np.ndarray:
        u = []
        for v in self.variables:
            x = values[v["path"]]
            if v["log"]:
                u.append((math.log(x) - math.log(v["lo"])) / (math.log(v["hi"]) - math.log(v["lo"])))
            else:
                u.append((x - v["lo"]) / (v["hi"] - v["lo"]))
        return np.clip(np.array(u, float), 0.0, 1.0)

    def evaluate(self, u) -> tuple[float, dict, Optional[Results]]:
        from . import api
        vals = self.to_values(u)
        try:
            design = set_design(Design.model_validate(self.design_json), vals)
            res = api.evaluate(design, WHAT)      # single or coaxial
        except Exception as ex:  # noqa: BLE001
            return FAIL, {"error": str(ex)[:200], **vals}, None
        return self.score(res), vals, res

    def score(self, res: Results) -> float:
        J = 0.0
        for i, t in enumerate(self.targets):
            try:
                v = quantity(res, t["quantity"])
            except Exception:  # noqa: BLE001
                return FAIL
            if not math.isfinite(v):
                return FAIL
            s = abs(t["value"]) if t.get("value") not in (None, 0) else 1.0
            s0 = self.scales0.get(i, 1.0) or 1.0
            k = t["kind"]
            if k == "min":
                term = v / s0
            elif k == "max":
                term = -v / s0
            elif k == "eq":
                term = ((v - t["value"]) / s) ** 2
            elif k == "le":
                term = (max(0.0, v - t["value"]) / s) ** 2
            else:
                term = (max(0.0, t["value"] - v) / s) ** 2
            J += t["weight"] * term
        return float(J)

    def __call__(self, u) -> float:
        return self.evaluate(u)[0]


def _init_worker():
    os.environ["NUMBA_NUM_THREADS"] = "1"
    try:
        import numba
        numba.set_num_threads(1)
    except Exception:  # noqa: BLE001
        pass


# --------------------------------------------------------------------------- driver


def run(req: OptimizeRequest, progress: Callable | None = None, mapper: Callable | None = None) -> OptimizeResult:
    """Run the optimisation. progress(i, n, objective, best) is called per evaluation
    (serial) or per generation (parallel). mapper: map-like callable used for parallel
    evaluation by differential_evolution (overrides req.workers)."""
    from scipy import optimize as so

    t0 = time.perf_counter()
    obj0 = Objective(req)
    init_vals = {}
    for v in req.variables:
        try:
            x = float(get_path(req.design.model_dump(mode="json"), v.path))
        except Exception:  # noqa: BLE001
            x = 0.5 * (v.lo + v.hi)
        init_vals[v.path] = min(max(x, v.lo), v.hi)
    u0 = obj0.to_u(init_vals)
    # initial evaluation → scales for min/max targets
    J0, vals0, res0 = obj0.evaluate(u0)
    scales0 = {}
    if res0 is not None:
        for i, t in enumerate(req.targets):
            if t.kind in ("min", "max"):
                try:
                    scales0[i] = abs(quantity(res0, t.quantity)) or 1.0
                except Exception:  # noqa: BLE001
                    scales0[i] = 1.0
    obj = Objective(req, scales0)
    J0 = obj.score(res0) if res0 is not None else FAIL
    history: list[dict] = [dict(i=0, objective=J0, **vals0)]
    best = dict(J=J0, u=u0, res=res0)
    n_eval = [1]

    def record(u, J, res=None):
        n_eval[0] += 1
        vals = obj.to_values(u)
        history.append(dict(i=n_eval[0], objective=float(J), **vals))
        if J < best["J"]:
            best.update(J=J, u=np.array(u, float), res=res)
        if progress is not None:
            try:
                progress(n_eval[0], req.max_evals, float(J), float(best["J"]))
            except Exception:  # noqa: BLE001
                pass

    def serial_f(u):
        J, _, res = obj.evaluate(u)
        record(u, J, res)
        return J

    n = len(req.variables)
    bounds = [(0.0, 1.0)] * n
    converged, message = False, ""
    pool = None
    try:
        if req.method == "differential_evolution":
            popsize = max(5, min(15, req.max_evals // max(1, 4 * n)))
            maxiter = max(1, req.max_evals // (popsize * n) - 1)
            workers: Any = 1
            design_mapper = mapper is not None and hasattr(mapper, "map_designs")
            if design_mapper:
                def workers(_func, pop):        # u → Design → remote evaluate → score
                    pop = [np.asarray(u, float) for u in pop]
                    designs, idx = [], []
                    Js = [FAIL] * len(pop)
                    for i, u in enumerate(pop):
                        try:
                            designs.append(set_design(req.design, obj.to_values(u)))
                            idx.append(i)
                        except Exception:  # noqa: BLE001 (invalid design → FAIL)
                            pass
                    recs = mapper.map_designs(designs) if designs else []
                    for i, rec in zip(idx, recs):
                        if "results" in rec:
                            Js[i] = obj.score(rec["results"])
                    for u, J in zip(pop, Js):
                        record(u, J, None)
                    return Js
            elif mapper is not None:
                workers = mapper
            elif req.workers > 1:
                import multiprocessing as mp
                ctx = mp.get_context("spawn")
                pool = ctx.Pool(req.workers, initializer=_init_worker)
                workers = pool.map
            if workers == 1:
                r = so.differential_evolution(serial_f, bounds, maxiter=maxiter, popsize=popsize,
                                              seed=req.seed, x0=u0, polish=False, tol=1e-6)
            else:
                gen = [0]

                def cb(xk, convergence=None):
                    gen[0] += 1      # scipy does not pass f(xk); log the best-so-far point
                    history.append(dict(i=-gen[0], generation=gen[0], **obj.to_values(xk)))
                    if progress is not None:
                        try:
                            progress(gen[0] * popsize * n, req.max_evals, float("nan"), float("nan"))
                        except Exception:  # noqa: BLE001
                            pass
                r = so.differential_evolution(obj, bounds, maxiter=maxiter, popsize=popsize, seed=req.seed,
                                              x0=u0, polish=False, tol=1e-6, workers=workers,
                                              updating="deferred", callback=cb)
                if not design_mapper:
                    n_eval[0] += int(r.nfev)
                Jr, _, resr = obj.evaluate(r.x)
                record(r.x, Jr, resr)
            converged, message = bool(r.success), str(r.message)
        elif req.method == "nelder_mead":
            r = so.minimize(lambda u: serial_f(np.clip(u, 0, 1)) + 1e3 * float(np.sum(np.clip(-u, 0, None) + np.clip(u - 1, 0, None))),
                            u0, method="Nelder-Mead",
                            options=dict(maxfev=req.max_evals, xatol=1e-4, fatol=1e-8, initial_simplex=None))
            converged, message = bool(r.success), str(r.message)
        else:
            r = so.minimize(serial_f, u0, method="SLSQP", bounds=bounds,
                            options=dict(maxiter=max(1, req.max_evals // (n + 1)), eps=1e-3))
            converged, message = bool(r.success), str(r.message)
    finally:
        if pool is not None:
            pool.close()
            pool.join()
    u_best = best["u"]
    design = set_design(req.design, obj.to_values(u_best))
    res = best["res"]
    if res is None:
        from . import api
        res = api.evaluate(design, WHAT)
    message += f" ({n_eval[0]} evals, {time.perf_counter() - t0:.1f} s)"
    return OptimizeResult(design=design, results=res, objective=float(best["J"]), n_evals=int(n_eval[0]),
                          history=history, converged=converged, message=message)
