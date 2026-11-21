"""Surrogate-assisted robust design search around 7in_tt_strap_steelx_v3.

  PYTHONPATH=flexblade python flexblade/study/surrogate.py run [--n 360] [--rounds 3] [--workers 8]

Stages (resumable; every evaluation goes through flexblade.cache):
  sample : Latin hypercube (+-20 %, 9 DE knobs + blade density) robustly evaluated (T0.7/1.0/1.4)
  rounds : fit GPs -> DE on the surrogate (P20 cr under +-tol Monte Carlo, margin/fatigue/thermal
           constraints with safety) -> verify top picks with real evaluations + 8 random tol samples
  final  : 24-sample tolerance check of best vs v3, Sobol/tolerance sensitivities, report + design.
No scikit-learn here: numpy ARD-RBF Gaussian processes (marginal-likelihood fit by L-BFGS).
"""
from __future__ import annotations

import os
os.environ.setdefault("NUMBA_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import argparse
import json
import math
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution, minimize
from scipy.linalg import cho_factor, cho_solve
from scipy.stats import qmc

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust                       # noqa: E402
from flexblade.optimize import get_path, set_design     # noqa: E402
from flexblade.schema import Design                     # noqa: E402
from flexblade.stability import margin_1p_eff          # noqa: E402
import run_study as rs                                  # noqa: E402

KINDS = ["trim", "modes", "harmonic", "checks"]
NOMF = HERE / "designs" / "7in_tt_strap_steelx_v3.json"
DATA = HERE / "results" / "surrogate_data.jsonl"
REPORT = HERE / "results" / "surrogate_7in.md"
OUTD = HERE / "designs" / "7in_tt_strap_steelx_surr.json"
BOX = 0.20

# DE knobs and tolerance (kind, amount): rel = x(1+U(-a,a)); abs = x+U(-a,a)
KNOBS = [("flexure.skew_deg", "abs", 1.0), ("flexure.hinge_radius", "rel", .05),
         ("flexure.length", "rel", .05), ("blade.elastic_axis_chord_frac", "abs", .01),
         ("flexure.thickness", "rel", .05), ("flexure.width", "rel", .05),
         ("flexure.cross_angle_deg", "abs", 1.0), ("flexure.strap_strand_diameter", "rel", .025),
         ("flexure.strap_length", "rel", .05)]
NK = len(KNOBS)
NAMES = [k[0].split(".")[-1] for k in KNOBS] + ["blade_rho"]
RHO_TOL, RHO_BOX = .05, .10
TARGETS = ["cr", "m1", "m2", "m3", "lfat", "ltherm", "lumax", "split"]
HEAD = ["cr", "m1", "m3", "lfat", "ltherm"]
# surrogate-side acceptance (stricter than the real checks: RES_BAND .15, util 1, split 2)
THR = dict(m=0.17, util=0.95, split=1.9)
KSIG = 1.0


def load_nom() -> Design:
    return Design.model_validate_json(NOMF.read_text())


def x_of(d: Design) -> np.ndarray:
    return np.array([float(get_path(d, k[0])) for k in KNOBS] + [1.0])


def build(base: Design, x: np.ndarray) -> Design:
    d = set_design(base, {k[0]: float(v) for k, v in zip(KNOBS, x[:NK])})
    if abs(x[NK] - 1.0) > 1e-9:
        lay = d.blade.layup.model_copy(update=dict(material=f"pa6_cf@rho={round(float(x[NK]), 4)}"))
        d = d.model_copy(update=dict(blade=d.blade.model_copy(update=dict(layup=lay))))
    return d


def box(x0: np.ndarray):
    V = {v.path: v for v in rs.variables("tt_strap_steelx", 7.0)}
    lo, hi = x0.copy(), x0.copy()
    for i, k in enumerate(KNOBS):
        lo[i], hi[i] = x0[i] * (1 - BOX), x0[i] * (1 + BOX)
        if V[k[0]].lo <= x0[i] <= V[k[0]].hi:       # clip to DE range only if v3 lies inside it
            lo[i], hi[i] = max(lo[i], V[k[0]].lo), min(hi[i], V[k[0]].hi)
    lo[NK], hi[NK] = 1 - RHO_BOX, 1 + RHO_BOX
    return lo, hi


def tol_draw(x: np.ndarray, rng, n: int, only: int | None = None) -> np.ndarray:
    """n tolerance perturbations of x (abs space)."""
    X = np.tile(x, (n, 1))
    for i in range(NK + 1):
        if only is not None and i != only:
            continue
        u = rng.uniform(-1, 1, n)
        if i == NK:
            X[:, i] = x[i] * (1 + RHO_TOL * u)
        elif KNOBS[i][1] == "rel":
            X[:, i] = x[i] * (1 + KNOBS[i][2] * u)
        else:
            X[:, i] = x[i] + KNOBS[i][2] * u
    return X


# --------------------------------------------------------------------------- evaluation


def _mgk(r: dict) -> list[float]:
    rpm = (r.get("summary") or {}).get("rpm") or (r.get("trim") or {}).get("rpm")
    out = [9.0, 9.0, 9.0]
    if not rpm or not r.get("modes"):
        return out
    f1 = rpm / 60.0
    for m in r["modes"]:
        f = m["freq_hz"]
        if f < 0.05 * f1:
            continue
        z = m.get("damping_ratio")
        for j, k in enumerate((1, 2, 3)):
            mg = abs(f / (k * f1) - 1.0)
            if z is not None and math.isfinite(z) and z > 0:
                mg = max(mg, margin_1p_eff(f / (k * f1), z))
            out[j] = min(out[j], mg)
    return out


def _point(job):
    did, tag, djson = job
    try:
        os.nice(10)
    except OSError:
        pass
    try:
        d = Design.model_validate_json(djson)
        r = api.evaluate(d, KINDS).model_dump(mode="json")
        m = rs.metrics(d, r)
        return did, tag, dict(cr=m["control_ratio"], mg=_mgk(r), util=m["util"], viol=m["viol"],
                              split=m["split_deg"], rpm=m["rpm"])
    except Exception as e:  # noqa: BLE001
        return did, tag, dict(err=repr(e)[:200])


def aggregate(pts: dict) -> dict:
    ok = {t: p for t, p in pts.items() if "err" not in p}
    if len(ok) < len(pts) or not ok:
        return dict(ok=False, err=next((p["err"] for p in pts.values() if "err" in p), "?"))
    P = list(ok.values())
    util: dict = {}
    viol: dict = {}
    for p in P:
        for k, v in p["util"].items():
            util[k] = max(util.get(k, 0.0), v)
        for k, v in p["viol"].items():
            viol[k] = max(viol.get(k, 0.0), v)
    umax = max([v for k, v in util.items() if not k.startswith("member:") and k not in rs.SOFT_CHECKS] or [1e3])
    fat = max(util.get("fatigue_stress", 0), util.get("fatigue_strain", 0))
    mg = [min(p["mg"][j] for p in P) for j in range(3)]
    sp = max([p["split"] for p in P if math.isfinite(p["split"])] or [99.0])
    cr = min(p["cr"] for p in P)
    return dict(ok=True, cr=cr, m1=min(mg[0], 1.0), m2=min(mg[1], 1.0), m3=min(mg[2], 1.0),
                fat=fat, therm=util.get("motor_thermal", 0.0), umax=umax, split=sp,
                feas=(not viol), viol=sorted(viol), cr_pts={t: p["cr"] for t, p in ok.items()})


def eval_designs(pool, base: Design, X: np.ndarray) -> list[dict]:
    jobs = []
    for i, x in enumerate(X):
        for tag, dv in robust.variants(build(base, x)):
            jobs.append((i, tag, dv.model_dump_json()))
    pts = [dict() for _ in X]
    for did, tag, p in pool.imap_unordered(_point, jobs, chunksize=1):
        pts[did][tag] = p
    return [aggregate(p) for p in pts]


def load_data() -> list[dict]:
    return [json.loads(l) for l in DATA.read_text().splitlines()] if DATA.exists() else []


def add_data(rows: list[dict]) -> None:
    DATA.parent.mkdir(exist_ok=True)
    with DATA.open("a") as f:
        for r in rows:
            f.write(json.dumps(r, default=float) + "\n")


def eval_and_store(pool, base, X, src, rnd) -> list[dict]:
    recs = eval_designs(pool, base, X)
    rows = [dict(x=list(map(float, x)), src=src, round=rnd, **rec) for x, rec in zip(X, recs)]
    add_data(rows)
    return rows


def cache_cases(base: Design) -> np.ndarray:
    """verify_v3 one-at-a-time tolerance cases (cache hits) as extra training data."""
    x0 = x_of(base)
    X = []
    idx = {k[0]: i for i, k in enumerate(KNOBS)}
    for p in ("flexure.width", "flexure.length", "flexure.hinge_radius"):
        for m in (0.9, 1.1):
            x = x0.copy(); x[idx[p]] *= m; X.append(x)
    for t in (0.00013, 0.0002):
        x = x0.copy(); x[idx["flexure.thickness"]] = t; X.append(x)
    for dk in (-2.0, 2.0):
        x = x0.copy(); x[0] += dk; X.append(x)
    for dk in (-.02, .02):
        x = x0.copy(); x[3] += dk; X.append(x)
    for m in (math.sqrt(.8), math.sqrt(1.2)):
        x = x0.copy(); x[7] *= m; X.append(x)
    for m in (0.9, 1.1):
        x = x0.copy(); x[NK] = m; X.append(x)
    return np.array(X)


# --------------------------------------------------------------------------- GP


class GP:
    def __init__(self, lo, hi):
        self.mid, self.half = (lo + hi) / 2, (hi - lo) / 2

    def z(self, X):
        return (np.atleast_2d(X) - self.mid) / self.half

    @staticmethod
    def _K(A, B, ell, amp):
        d = (A[:, None, :] - B[None, :, :]) / ell
        return amp * np.exp(-0.5 * (d ** 2).sum(-1))

    def _nll(self, th, Z, y):
        ell, amp, nz = np.exp(th[:-2]), np.exp(th[-2]), np.exp(th[-1])
        d2 = ((Z[:, None, :] - Z[None, :, :]) / ell) ** 2
        Kf = amp * np.exp(-0.5 * d2.sum(-1))
        K = Kf + (nz + 1e-8) * np.eye(len(Z))
        try:
            c = cho_factor(K, lower=True)
        except np.linalg.LinAlgError:
            return 1e10, np.zeros_like(th)
        a = cho_solve(c, y)
        nll = 0.5 * y @ a + np.log(np.diag(c[0])).sum()
        W = cho_solve(c, np.eye(len(Z))) - np.outer(a, a)
        g = np.empty_like(th)
        WK = W * Kf
        g[:-2] = 0.5 * np.einsum("ij,ijk->k", WK, d2)
        g[-2] = 0.5 * WK.sum()
        g[-1] = 0.5 * nz * np.trace(W)
        return nll / len(Z), g / len(Z)

    def fit(self, X, y, th0=None, restarts=2, maxiter=150, seed=0):
        Z = self.z(X)
        self.ym, self.ys = y.mean(), y.std() + 1e-12
        yn = (y - self.ym) / self.ys
        d = Z.shape[1]
        bnds = [(np.log(.15), np.log(30))] * d + [(np.log(.1), np.log(10)), (np.log(1e-7), np.log(.3))]
        rng = np.random.default_rng(seed)
        best = None
        for r in range(restarts):
            t0 = th0 if (th0 is not None and r == 0) else np.r_[np.log(rng.uniform(1.5, 5, d)), 0.0, np.log(1e-3)]
            res = minimize(self._nll, t0, args=(Z, yn), method="L-BFGS-B", jac=True, bounds=bnds,
                           options=dict(maxiter=maxiter))
            if best is None or res.fun < best.fun:
                best = res
        self.th = best.x
        ell, amp, nz = np.exp(self.th[:-2]), np.exp(self.th[-2]), np.exp(self.th[-1])
        self.ell, self.amp, self.Z = ell, amp, Z
        self.c = cho_factor(self._K(Z, Z, ell, amp) + (nz + 1e-8) * np.eye(len(Z)), lower=True)
        self.alpha = cho_solve(self.c, yn)
        return self

    def predict(self, X, std=False):
        Zs = self.z(X)
        Ks = self._K(Zs, self.Z, self.ell, self.amp)
        mu = Ks @ self.alpha * self.ys + self.ym
        if not std:
            return mu
        v = cho_solve(self.c, Ks.T)
        var = np.maximum(self.amp - (Ks * v.T).sum(1), 1e-12)
        return mu, np.sqrt(var) * self.ys


def targets_of(rows: list[dict]):
    """-> X, dict name -> y for trainable rows (valid trim)."""
    R = [r for r in rows if r.get("ok") and r["cr"] > 0.005 and math.isfinite(r["split"])]
    X = np.array([r["x"] for r in R])
    Y = dict(cr=[r["cr"] for r in R], m1=[r["m1"] for r in R], m2=[r["m2"] for r in R],
             m3=[r["m3"] for r in R], lfat=[math.log(max(r["fat"], 1e-3)) for r in R],
             ltherm=[math.log(max(r["therm"], 1e-3)) for r in R],
             lumax=[math.log(max(min(r["umax"], 50), 1e-3)) for r in R],
             split=[min(r["split"], 20.0) for r in R])
    return X, {k: np.array(v) for k, v in Y.items()}


def fit_all(X, Y, lo, hi, prev=None):
    gps = {}
    for t in TARGETS:
        gps[t] = GP(lo, hi).fit(X, Y[t], th0=(prev[t].th if prev else None), seed=1)
    return gps


def cv_r2(X, Y, lo, hi, gps, k=5, seed=0):
    idx = np.random.default_rng(seed).permutation(len(X))
    folds = np.array_split(idx, k)
    out = {}
    for t in TARGETS:
        pred = np.zeros(len(X))
        for f in folds:
            tr = np.setdiff1d(idx, f)
            g = GP(lo, hi).fit(X[tr], Y[t][tr], th0=gps[t].th, restarts=1, maxiter=40)
            pred[f] = g.predict(X[f])
        out[t] = 1 - ((Y[t] - pred) ** 2).sum() / ((Y[t] - Y[t].mean()) ** 2).sum()
    return out


# --------------------------------------------------------------------------- surrogate MC + DE


def mc_eval(gps, Xs, ksig=KSIG):
    """Xs (M,10) -> cr_eff (M,), pass (M,), violation (M,) (normalised, >=0)."""
    P = {}
    for t in TARGETS:
        P[t] = gps[t].predict(Xs, std=True)
    cr = P["cr"][0] - 0.5 * P["cr"][1]
    pen = np.zeros(len(Xs))
    for t in ("m1", "m2", "m3"):
        pen += np.maximum(0, THR["m"] - (P[t][0] - ksig * P[t][1])) / 0.1
    for t in ("lfat", "ltherm", "lumax"):
        pen += np.maximum(0, P[t][0] + ksig * P[t][1] - math.log(THR["util"])) / 0.1
    pen += np.maximum(0, P["split"][0] + ksig * P["split"][1] - THR["split"])
    return cr, pen <= 0, pen


def make_obj(gps, lo, hi, S=48, seed=0):
    rng = np.random.default_rng(seed)
    U = rng.uniform(-1, 1, (S, NK + 1))   # common random numbers

    def tolX(xk):  # xk (P,NK) abs -> (P*S, NK+1)
        P = len(xk)
        X = np.ones((P, S, NK + 1))
        X[:, :, :NK] = xk[:, None, :]
        for i in range(NK):
            kind, a = KNOBS[i][1], KNOBS[i][2]
            X[:, :, i] = xk[:, None, i] * (1 + a * U[None, :, i]) if kind == "rel" else xk[:, None, i] + a * U[None, :, i]
        X[:, :, NK] = 1 + RHO_TOL * U[None, :, NK]
        return X.reshape(P * S, NK + 1)

    def f(zt):  # DE vectorised: zt (NK, P) in [0,1]
        xk = lo[:NK] + zt.T * (hi[:NK] - lo[:NK])
        cr, ok, pen = mc_eval(gps, tolX(xk))
        cr, ok, pen = cr.reshape(-1, S), ok.reshape(-1, S), pen.reshape(-1, S)
        eff = np.where(ok, cr, 0.0)
        p20 = np.percentile(eff, 20, axis=1)
        return -p20 + 0.05 * pen.mean(1) + 0.5 * np.maximum(0, 0.9 - ok.mean(1))
    return f, tolX


def optimise(gps, lo, hi, seeds=range(8), maxiter=120):
    res = []
    for s in seeds:
        f, _ = make_obj(gps, lo, hi, seed=100 + s)
        r = differential_evolution(f, [(0, 1)] * NK, seed=s, vectorized=True, updating="deferred",
                                   popsize=12, maxiter=maxiter, tol=1e-6, polish=False)
        xk = lo[:NK] + r.x * (hi[:NK] - lo[:NK])
        res.append((r.fun, np.r_[xk, 1.0]))
    return sorted(res, key=lambda t: t[0])


def surrogate_stats(gps, x, lo, hi, n=2000, seed=5):
    _, tolX = make_obj(gps, lo, hi, S=1)
    rng = np.random.default_rng(seed)
    Xs = tol_draw(x, rng, n)
    cr, ok, _ = mc_eval(gps, Xs)
    eff = np.where(ok, cr, 0.0)
    return dict(p20=float(np.percentile(eff, 20)), pass_rate=float(ok.mean()),
                cr_nom=float(gps["cr"].predict(x[None])[0]))


def pick(res, evaluated, lo, hi, n=5, mind=0.06):
    out = []
    for fun, x in res:
        z = (x[:NK] - lo[:NK]) / (hi[:NK] - lo[:NK])
        if all(np.linalg.norm(z - zz) > mind for zz in out) and all(
                np.linalg.norm(z - (e[:NK] - lo[:NK]) / (hi[:NK] - lo[:NK])) > 0.02 for e in evaluated):
            out.append(z)
            yield fun, x
        if len(out) >= n:
            return


# --------------------------------------------------------------------------- real verification


def real_stats(rows: list[dict]) -> dict:
    """rows[0] nominal, rest tolerance samples -> pass rate / P20 cr over all."""
    eff = [r["cr"] if r.get("ok") and r["feas"] else 0.0 for r in rows]
    return dict(n=len(rows), pass_rate=float(np.mean([e > 0 for e in eff])),
                p20=float(np.percentile(eff, 20)), nom_cr=rows[0].get("cr", 0.0),
                nom_feas=bool(rows[0].get("feas")), cr_min_tol=float(min(eff)),
                viol=sorted({v for r in rows for v in r.get("viol", [])}))


def verify(pool, base, x, n_tol, rnd, tag, seed):
    rng = np.random.default_rng(seed)
    X = np.vstack([x[None], tol_draw(x, rng, n_tol)])
    rows = eval_and_store(pool, base, X, tag, rnd)
    return real_stats(rows), rows


# --------------------------------------------------------------------------- sensitivities


def sobol_first(gps, lo, hi, tgt, N=3000, seed=0):
    rng = np.random.default_rng(seed)
    A = lo + rng.uniform(size=(N, NK + 1)) * (hi - lo)
    B = lo + rng.uniform(size=(N, NK + 1)) * (hi - lo)
    A[:, NK] = B[:, NK] = 1.0
    fa, fb = gps[tgt].predict(A), gps[tgt].predict(B)
    var = np.var(np.r_[fa, fb])
    S = []
    for i in range(NK):
        ABi = A.copy(); ABi[:, i] = B[:, i]
        S.append(float(np.mean(fb * (gps[tgt].predict(ABi) - fa)) / var))
    return S


def tol_variance(gps, x, tgt, n=1500, seed=3):
    rng = np.random.default_rng(seed)
    v = []
    for i in range(NK + 1):
        v.append(float(np.var(gps[tgt].predict(tol_draw(x, rng, n, only=i)))))
    tot = sum(v) or 1.0
    return [a / tot for a in v], v


def elasticity(gps, x, tgt):
    """d f / d ln x_i * (tolerance in ln units) -> change of f over one full tolerance step."""
    g = []
    for i in range(NK + 1):
        h = 1e-3 * x[i]
        xp, xm = x.copy(), x.copy()
        xp[i] += h; xm[i] -= h
        d = (gps[tgt].predict(xp[None])[0] - gps[tgt].predict(xm[None])[0]) / (2 * h)
        tol = (RHO_TOL * x[i] if i == NK else (KNOBS[i][2] * x[i] if KNOBS[i][1] == "rel" else KNOBS[i][2]))
        g.append(float(d * tol))
    return g


# --------------------------------------------------------------------------- driver


def run(a):
    base = load_nom()
    x0 = x_of(base)
    lo, hi = box(x0)
    t0 = time.time()
    log = lambda *s: print(f"[{time.time() - t0:6.0f}s]", *s, flush=True)
    rounds_log = []
    with Pool(a.workers) as pool:
        rows = load_data()
        X = lo + qmc.LatinHypercube(d=NK + 1, seed=11).random(a.n) * (hi - lo)
        done = sum(r["src"] == "lhs" for r in rows)       # resume: LHS is deterministic, chunked
        for c in range(done, a.n, 40):
            log(f"evaluating LHS designs {c}..{min(c + 40, a.n)} of {a.n}")
            eval_and_store(pool, base, X[c:c + 40], "lhs", 0)
        if not any(r["src"] == "cache" for r in rows):
            log("evaluating base design + cached one-at-a-time cases")
            eval_and_store(pool, base, np.vstack([x0[None], cache_cases(base)]), "cache", 0)
        rows = load_data()
        v3_rows = None
        for rnd in range(1, a.rounds + 1):
            rows = load_data()
            X, Y = targets_of(rows)
            log(f"round {rnd}: {len(rows)} rows, {len(X)} trainable; fitting")
            gps = fit_all(X, Y, lo, hi)
            r2 = cv_r2(X, Y, lo, hi, gps)
            log("CV R2", {k: round(v, 3) for k, v in r2.items()})
            res = optimise(gps, lo, hi)
            ev = [np.array(r["x"]) for r in rows]
            picks = list(pick(res, ev, lo, hi))
            log(f"  surrogate best objective {-res[0][0]:.4f}; {len(picks)} picks")
            entry = dict(round=rnd, n_rows=len(rows), n_train=len(X), r2=r2, picks=[])
            for j, (fun, x) in enumerate(picks):
                ss = surrogate_stats(gps, x, lo, hi)
                vs, vrows = verify(pool, base, x, 8, rnd, f"pick{rnd}.{j}", 1000 * rnd + j)
                entry["picks"].append(dict(x=list(map(float, x)), obj=-fun, surr=ss, real=vs,
                                           real_cr_nom_pred=float(gps["cr"].predict(x[None])[0])))
                log(f"  pick{j}: pred cr {ss['cr_nom']:.3f} p20 {ss['p20']:.3f} pass {ss['pass_rate']:.2f} | "
                    f"real nom {vs['nom_cr']:.3f} feas {vs['nom_feas']} p20 {vs['p20']:.3f} pass {vs['pass_rate']:.2f}")
            rounds_log.append(entry)
        # final
        rows = load_data()
        X, Y = targets_of(rows)
        gps = fit_all(X, Y, lo, hi)
        r2 = cv_r2(X, Y, lo, hi, gps)
        cands = [(e["round"], j, np.array(p["x"]), p["real"]) for e in rounds_log for j, p in enumerate(e["picks"])]
        cands.sort(key=lambda c: (-c[3]["pass_rate"], -c[3]["p20"]))
        best = cands[0] if cands else None
        log("final tolerance check (24 samples) best vs v3")
        v3s, _ = verify(pool, base, x0, 24, 9, "v3_tol24", 777)
        bs = bs_rows = None
        if best:
            bs, bs_rows = verify(pool, base, best[2], 24, 9, "best_tol24", 778)
        json.dump(dict(rounds=rounds_log, v3=v3s, best=bs, r2_final=r2),
                  open(HERE / "results" / f"surrogate_summary_{NOMF.stem}.json", "w"), indent=1, default=float)
        write_report(base, x0, lo, hi, gps, rounds_log, r2, v3s, best, bs, len(rows), len(X))
    log("done")


def write_report(base, x0, lo, hi, gps, rounds_log, r2, v3s, best, bs, nrows, ntrain):
    L = ["# Surrogate-assisted robust search, 7in tt_strap_steelx", "",
         f"Data: {nrows} robustly-evaluated designs ({ntrain} trainable) = LHS +-{BOX:.0%} over 9 DE knobs + "
         "blade density, v3, v3 one-at-a-time tolerance cases (cache hits), and verified picks/tol samples. "
         "Each design = 3 throttle points (T0.7/1.0/1.4). GP: numpy ARD-RBF (no scikit-learn installed).", "",
         "## Cross-validated R2 (5-fold, final data)", "", "| target | R2 |", "|---|---|"]
    for t in TARGETS:
        L.append(f"| {t} | {r2[t]:.3f} |")
    L += ["", "Targets: cr = min over throttle of control ratio; m1/m2/m3 = min over throttle and modes of "
          "damping-aware |f/kP-1| (capped 1); lfat/ltherm/lumax = log of max fatigue / motor-thermal / "
          "any-check utilisation; split = max trim differential pitch (deg).", "",
          "## Rounds (surrogate-optimised picks, verified)", "",
          "| round | pick | pred cr nom | pred P20 | pred pass | real cr nom | real feas | real P20 (9) | real pass (9) |",
          "|---|---|---|---|---|---|---|---|---|"]
    for e in rounds_log:
        for j, p in enumerate(e["picks"]):
            s, r = p["surr"], p["real"]
            L.append(f"| {e['round']} | {j} | {s['cr_nom']:.3f} | {s['p20']:.3f} | {s['pass_rate']:.2f} | "
                     f"{r['nom_cr']:.3f} | {r['nom_feas']} | {r['p20']:.3f} | {r['pass_rate']:.2f} |")
        L.append(f"|  | R2 round {e['round']} | " + ", ".join(f"{k} {v:.2f}" for k, v in e["r2"].items()) + " | | | | | |")
    L += ["", "## Best vs v3 (nominal + 24 random tolerance samples: knobs per `KNOBS`, density +-5 %)", "",
          "| design | nominal cr | nominal feasible | tol pass rate | P20 cr (eff.) | min cr | violations |",
          "|---|---|---|---|---|---|---|"]
    for name, s in (("v3", v3s), ("surrogate best", bs)):
        if s:
            L.append(f"| {name} | {s['nom_cr']:.3f} | {s['nom_feas']} | {s['pass_rate']:.2f} | {s['p20']:.3f} | "
                     f"{s['cr_min_tol']:.3f} | {', '.join(s['viol']) or '-'} |")
    xb = best[2] if best else x0
    L += ["", "Knob values (v3 -> best):", "", "| knob | v3 | best | change |", "|---|---|---|---|"]
    for i, n in enumerate(NAMES[:NK]):
        L.append(f"| {n} | {x0[i]:.5g} | {xb[i]:.5g} | {100 * (xb[i] / x0[i] - 1):+.1f}% |")
    L += ["", "## Sensitivity", "",
          "Global first-order Sobol indices over the +-20 % box (surrogate, density fixed):", "",
          "| knob | " + " | ".join(HEAD + ["m2"]) + " |", "|---|" + "---|" * (len(HEAD) + 1)]
    S = {t: sobol_first(gps, lo, hi, t) for t in HEAD + ["m2"]}
    for i in range(NK):
        L.append(f"| {NAMES[i]} | " + " | ".join(f"{S[t][i]:.2f}" for t in HEAD + ["m2"]) + " |")
    L += ["", "Tolerance sensitivity at the best design: share of tolerance-induced variance of each "
          "target (one source at a time at +-tol, surrogate) and the change of the target across one full "
          "tolerance step (elasticity x tol).", "",
          "| source (tol) | " + " | ".join(f"{t} var share | d{t}" for t in ["cr", "m1", "m3", "lfat"]) + " |",
          "|---|" + "---|---|" * 4]
    V = {t: tol_variance(gps, xb, t)[0] for t in ["cr", "m1", "m3", "lfat"]}
    E = {t: elasticity(gps, xb, t) for t in ["cr", "m1", "m3", "lfat"]}
    for i in range(NK + 1):
        tl = (f"{KNOBS[i][2]:g}{'%' if KNOBS[i][1] == 'rel' and False else ''}" if i < NK else f"{RHO_TOL}")
        L.append(f"| {NAMES[i]} ({'+-' + str(tl)}{' rel' if i < NK and KNOBS[i][1] == 'rel' else ''}) | " +
                 " | ".join(f"{V[t][i]:.2f} | {E[t][i]:+.3f}" for t in ["cr", "m1", "m3", "lfat"]) + " |")
    REPORT.write_text("\n".join(L) + "\n")
    if best:
        d = build(base, xb)
        d = d.model_copy(update=dict(name=OUTD.stem))
        OUTD.write_text(d.model_dump_json(indent=1))


def main():
    global NOMF, DATA, REPORT, OUTD
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run"])
    ap.add_argument("--design", default=str(NOMF), help="base design json (e.g. study/designs/7in_tt_strap_steelx_v5.json)")
    ap.add_argument("--n", type=int, default=360)
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()
    NOMF = Path(a.design)
    stem = NOMF.stem
    DATA = HERE / "results" / f"surrogate_data_{stem}.jsonl"   # resumable; cache hits are per model hash
    REPORT = HERE / "results" / f"surrogate_{stem}.md"
    OUTD = HERE / "designs" / f"{stem}_surr.json"
    run(a)


if __name__ == "__main__":
    main()
