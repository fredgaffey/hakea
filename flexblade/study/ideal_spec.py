"""Idealise then realise: the IDEAL flexure per rotor size, free of material and
manufacturing limits.

For each size (default 5, 7, 10 in, default preset motor, same 1/rev pulse as run_study:
0.95 × motor capability, AM32 max_ramp 255), differential evolution maximises
harmonic.control_ratio over the `ideal` flexure topology (flexblade.schema.IdealFlexure:
zero-length 6×6 spring at the hinge point = skew angle + principal stiffnesses about the
skewed hinge axis / the cross axis / flap + three translational stiffnesses, hysteretic loss)
plus the hinge radius and blade mass placement (elastic axis / chordwise CG, tip mass and its
chordwise offset, lag inertia at the hinge). Constraints as run_study (aero/trim
convergence, ESC duty peak/slew, motor 1/rev torque + thermal, blade checks, resonance
±15 % of 1–3P, static pitch split) plus aeroelastic stability (no negatively damped mode);
the flexure member strain / creep / fatigue checks do not exist for the ideal spring.

Outputs per size: study/results/ideal_spec_<D>in.json (spec card: optimum K6 at the hinge
point, principal stiffnesses, skew, coupling, mass placement, control_ratio, binding
constraint, and the target stiffness ratios a real flexure must hit), study/designs/
ideal_<D>in.json (loadable Design), study/results/ideal_evals.jsonl.gz (every evaluation).
`distance(K6_real, card)` scores a real concept (its 6×6 at the hinge point, e.g.
Results.flexure_stiffness) by log-distance to the ideal targets.

Full search (Fred runs this):
  PYTHONPATH=<worktree>/flexblade ~/venv-flexblade/bin/python study/ideal_spec.py \\
      --hosts local:28,mac:10 --sizes 5,7,10 --popmult 10 --maxiter 60
Smoke:
  ... study/ideal_spec.py --hosts local:8 --sizes 5 --popmult 2 --maxiter 2
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import os
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import run_study as R                                   # noqa: E402
from flexblade import fact, presets, robust            # noqa: E402
from flexblade.optimize import set_design               # noqa: E402
from flexblade.schema import Design, IdealFlexure       # noqa: E402

RESULTS = HERE / "results"
DESIGNS = HERE / "designs"
STAB_MIN = -1e-4          # mode damping ratio below this = unstable
# motor_thermal: the run_study pulse (0.95 x current-limited capability, ~3.8 x hover torque at
# 5") overheats the motor continuously whatever the flexure, so by default it is reported but not
# enforced (it would make every design infeasible). --enforce-thermal turns it on.
ENFORCE_THERMAL = False


# --------------------------------------------------------------------------- variables


def variables(D: float) -> list[tuple[str, float, float, bool]]:
    """(path, lo, hi, log) for size D (inch); stiffness ranges are wide on purpose."""
    s = D / 5.0
    return [
        ("flexure.skew_deg", 0.0, 80.0, False),
        ("flexure.hinge_radius", 0.006 * s, 0.0135 * s, False),
        ("flexure.ideal.k_hinge", 1e-3 * s ** 3, 10.0 * s ** 3, True),
        ("flexure.ideal.k_cross", 0.1 * s ** 3, 1e4 * s ** 3, True),
        ("flexure.ideal.k_flap", 0.05 * s ** 3, 1e4 * s ** 3, True),
        ("flexure.ideal.k_axial", 1e3 * s, 1e8 * s, True),
        ("flexure.ideal.k_inplane", 1e3 * s, 1e8 * s, True),
        ("flexure.ideal.k_outplane", 1e3 * s, 1e8 * s, True),
        ("flexure.ideal.loss_eta", 0.0, 0.3, False),
        ("blade.elastic_axis_chord_frac", 0.1, 0.6, False),
        ("flexure.ideal.tip_mass", 0.0, 1.5e-3 * s ** 2, False),
        ("flexure.ideal.tip_mass_chord", -0.006 * s, 0.006 * s, False),
        ("flexure.ideal.lag_inertia", 1e-10 * s ** 4, 1e-5 * s ** 4, True),
    ]


def to_values(V, u) -> dict:
    out = {}
    for (p, lo, hi, lg), x in zip(V, np.clip(u, 0.0, 1.0)):
        out[p] = float(math.exp(math.log(lo) + x * (math.log(hi) - math.log(lo)))) if lg \
            else float(lo + x * (hi - lo))
    return out


def to_u(V, vals: dict) -> np.ndarray:
    u = []
    for p, lo, hi, lg in V:
        v = min(max(vals[p], lo), hi)
        u.append((math.log(v) - math.log(lo)) / (math.log(hi) - math.log(lo)) if lg else (v - lo) / (hi - lo))
    return np.array(u)


def base_design(D: float) -> tuple[Design, dict]:
    d, cap = R.base_design(D, "cf_flexure")
    d = d.model_copy(update=dict(
        name=f"{presets._fmt(D)}in_ideal",
        flexure=d.flexure.model_copy(update=dict(topology="ideal", members=[],
                                                 ideal=IdealFlexure()))))
    return d, cap


# --------------------------------------------------------------------------- constraints


def metrics(d: Design, r: dict) -> dict:
    m = R.metrics(d, r)
    m["viol"] = {k: v for k, v in m["viol"].items() if not k.startswith("geom:")
                 and (ENFORCE_THERMAL or k != "check:motor_thermal")}
    m["motor_thermal_util"] = float(m.get("util", {}).get("motor_thermal", float("nan")))
    zmin = min((md.get("damping_ratio", 0.0) for md in r.get("modes", [])
                if md.get("freq_hz", 0.0) > 0.05 * m["rpm"] / 60.0), default=0.0)
    m["min_damping"] = float(zmin)
    if zmin < STAB_MIN:
        m["viol"]["stability"] = min(1.0, -zmin * 100.0)
    m["feasible"] = not m["viol"]
    m["binding"] = binding(m)
    return m


def binding(m: dict) -> tuple[str, float]:
    """The constraint closest to (or furthest past) its limit, as utilisation (1 = at limit)."""
    u = {f"check:{k}": v for k, v in m.get("util", {}).items() if not k.startswith("member:")
         and (ENFORCE_THERMAL or k != "motor_thermal")}
    if math.isfinite(m.get("res_margin", float("nan"))):
        u[f"resonance ({m.get('res_which', '')})"] = R.RES_BAND / max(m["res_margin"], 1e-6)
    if math.isfinite(m.get("split_deg", float("nan"))):
        u["pitch_split"] = m["split_deg"] / R.SPLIT_MAX
    u["stability"] = 1.0 + max(0.0, -m.get("min_damping", 0.0)) * 100 if m.get("min_damping", 0.0) < STAB_MIN \
        else 0.0
    k = max(u, key=u.get)
    return k, float(u[k])


# --------------------------------------------------------------------------- spec card


def card(D: float, d: Design, m: dict, cap: dict) -> dict:
    ip = d.flexure.ideal
    K = fact.ideal_K6(d)
    _, a = fact.hinge_axis(d, 1)
    Kr = K[3:, 3:]
    k_pitch, k_flap, k_lag, k_lp = Kr[0, 0], Kr[1, 1], Kr[2, 2], Kr[0, 2]
    return dict(
        diameter_in=D, control_ratio=m["control_ratio"], feasible=m["feasible"],
        binding_constraint=m["binding"][0], binding_utilisation=m["binding"][1],
        violations=m["viol"], motor_thermal_util=m["motor_thermal_util"],
        motor_thermal_enforced=ENFORCE_THERMAL, rpm=m["rpm"], thrust=m["thrust"],
        torque_mod_frac=d.op.torque_mod_frac, capability=cap,
        stiffness=dict(
            frame="blade-1 frame H at the hinge point: x radial, y in-plane, z rotor axis; "
                  "[ux,uy,uz,rx,ry,rz] -> [F;M], N/m, N m/rad",
            hinge_point=[float(x) for x in fact.hinge_axis(d, 1)[0]], hinge_axis=a.tolist(),
            skew_deg=d.flexure.skew_deg, k_hinge=ip.k_hinge, k_cross=ip.k_cross, k_flap=ip.k_flap,
            k_axial=ip.k_axial, k_inplane=ip.k_inplane, k_outplane=ip.k_outplane,
            k_pitch=k_pitch, k_lag=k_lag, k_lag_pitch=k_lp, loss_eta=ip.loss_eta, K6=K.tolist()),
        mass=dict(elastic_axis_chord_frac=d.blade.elastic_axis_chord_frac,
                  tip_mass=ip.tip_mass, tip_mass_chord=ip.tip_mass_chord,
                  lag_inertia=ip.lag_inertia, hinge_radius=d.flexure.hinge_radius),
        dynamics=dict(lag_per_rev=m["lag_per_rev"], flap_per_rev=m["flap_per_rev"],
                      pitch_per_rev=m["pitch_per_rev"], pitch_amp_deg=m["pitch_amp_deg"],
                      lag_amp_deg=m["lag_amp_deg"], moment_phase_deg=m["moment_phase_deg"],
                      min_damping=m["min_damping"], res_margin=m["res_margin"]),
        # what a real flexure must hit (scale-free where possible)
        targets=dict(skew_deg=d.flexure.skew_deg,
                     k_hinge=ip.k_hinge,
                     k_cross_over_k_hinge=ip.k_cross / ip.k_hinge,
                     k_flap_over_k_hinge=ip.k_flap / ip.k_hinge,
                     k_lag_over_k_flap=k_lag / k_flap,
                     k_lag_pitch_over_k_lag=k_lp / k_lag,
                     k_axial=ip.k_axial, k_inplane=ip.k_inplane, k_outplane=ip.k_outplane,
                     lag_per_rev=m["lag_per_rev"]),
    )


def _soft_skew_deg(C: np.ndarray) -> float:
    """Skew (from the rotor axis toward span) of the softest rotation axis of compliance C."""
    w, v = np.linalg.eigh(0.5 * (C + C.T))
    s = v[:, -1]
    return float(math.degrees(math.atan2(abs(s[0]), abs(s[2]))))


def distance(K6_real, spec: dict) -> dict:
    """RMS log10 distance of a real flexure's 6×6 (hinge point, frame H) to the ideal
    targets (ratios + k_hinge; translations only count when softer than ideal)."""
    K = np.asarray(K6_real, float)
    t = spec["targets"]
    a = np.asarray(spec["stiffness"]["hinge_axis"], float)
    y = np.array([0.0, 1.0, 0.0])
    b = np.cross(a, y)
    C = np.linalg.inv(K)[3:, 3:]                      # free (translations released) compliance
    # abs + floor: an indefinite/ill-conditioned real K6 must not crash the scoring
    kh, kc, kf = (max(abs(1 / v), 1e-12) for v in (a @ C @ a, b @ C @ b, y @ C @ y))
    Kr = K[3:, 3:]
    terms = dict(k_hinge=math.log10(kh / t["k_hinge"]),
                 k_cross_over_k_hinge=math.log10(kc / kh / t["k_cross_over_k_hinge"]),
                 k_flap_over_k_hinge=math.log10(kf / kh / t["k_flap_over_k_hinge"]),
                 skew_deg=(_soft_skew_deg(C) - t["skew_deg"]) / 45.0,
                 k_lag_over_k_flap=math.log10(abs(Kr[2, 2] / Kr[1, 1]) / t["k_lag_over_k_flap"]))
    for i, k in enumerate(("k_axial", "k_inplane", "k_outplane")):
        terms[k] = min(0.0, math.log10(K[i, i] / t[k]))
    return dict(rms=float(np.sqrt(np.mean([v * v for v in terms.values()]))), terms=terms)


def reference(specs: list[dict], concept: str, pool) -> None:
    """Evaluate the stored real design per size with the current code (same constraints) and
    add its control_ratio and distance to the ideal card."""
    todo = []
    for s in specs:
        p = DESIGNS / f"{presets._fmt(s['diameter_in'])}in_{concept}.json"
        if p.exists():
            todo.append((s, Design.model_validate_json(p.read_text())))
    recs = pool.map([d for _, d in todo]) if todo else []
    for (s, d), rec in zip(todo, recs):
        if "results" not in rec:
            s["reference"] = dict(design=d.name, error=rec.get("error", "?")[:200])
        else:
            r = rec["results"]
            m = metrics(d, r)
            ref = dict(design=d.name, control_ratio=m["control_ratio"], feasible=m["feasible"],
                       binding=m["binding"], lag_per_rev=m["lag_per_rev"])
            if r.get("flexure_stiffness"):
                ref["distance"] = distance(r["flexure_stiffness"], s)
            s["reference"] = ref
        (RESULTS / f"ideal_spec_{presets._fmt(s['diameter_in'])}in.json").write_text(
            json.dumps(s, indent=1, default=float))


# --------------------------------------------------------------------------- driver


def run_size(D: float, pool, log, popmult: int, maxiter: int, seed: int) -> dict:
    from scipy.optimize import differential_evolution

    t0 = time.time()
    case = f"{presets._fmt(D)}in_ideal"
    base, cap = base_design(D)
    V = variables(D)
    best = dict(J=float("inf"))
    n = [0]
    lock = threading.Lock()

    def mapper(_f, pop):
        pop = [np.asarray(u, float) for u in pop]
        vals = [to_values(V, u) for u in pop]
        designs = [set_design(base, v) for v in vals]
        recs = robust.map_metrics(pool, designs, metrics, R.ROBUST, None)
        Js = []
        for v, dsg, (mm, err) in zip(vals, designs, recs):
            if mm is None or "error" in mm:
                J, m = R.FAIL, dict(error=err or mm["error"])
            else:
                m = mm
                J = R.score(m)
            Js.append(J)
            with lock:
                n[0] += 1
                if J < best["J"]:
                    best.update(J=J, vals=v, m=m)
            log.write(dict(case=case, J=J, x=R._round(v), **{k: x for k, x in m.items() if k != "util"}))
        return Js

    u0 = to_u(V, {p: float(R._get(base, p)) if p.startswith(("blade.", "flexure.skew", "flexure.hinge"))
                  else float(getattr(IdealFlexure(), p.split(".")[-1])) for p, *_ in V})
    gen = [0]

    def cb(xk, convergence=None):
        gen[0] += 1
        b = best.get("m", {})
        print(f"[{case}] gen {gen[0]:3d} evals {n[0]:5d} best J {best['J']:+.4f} "
              f"cr {b.get('control_ratio', float('nan')):.4f} feas {b.get('feasible')} "
              f"bind {b.get('binding')} viol {R._short(b.get('viol', {}))}  {time.time() - t0:.0f}s",
              flush=True)

    r = differential_evolution(lambda u: 0.0, [(0.0, 1.0)] * len(V), maxiter=maxiter, popsize=popmult,
                               seed=seed, x0=u0, polish=False, tol=1e-4, atol=0.0,
                               init="latinhypercube", mutation=(0.5, 1.0), recombination=0.7,
                               workers=mapper, updating="deferred", callback=cb)
    if "vals" not in best:
        raise RuntimeError(f"{case}: no successful evaluation ({best})")
    bd = set_design(base, best["vals"]).model_copy(update=dict(name=f"ideal_{presets._fmt(D)}in"))
    spec = card(D, bd, best["m"], cap)
    spec.update(J=best["J"], n_evals=n[0], generations=gen[0], de_message=str(r.message),
                wall_s=time.time() - t0, x=R._round(best["vals"]))
    DESIGNS.mkdir(parents=True, exist_ok=True)
    (DESIGNS / f"ideal_{presets._fmt(D)}in.json").write_text(json.dumps(bd.model_dump(mode="json"), indent=1))
    (RESULTS / f"ideal_spec_{presets._fmt(D)}in.json").write_text(json.dumps(spec, indent=1, default=float))
    return spec


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--hosts", default="local:28,mac:10")
    ap.add_argument("--sizes", default="5,7,10")
    ap.add_argument("--popmult", type=int, default=10)
    ap.add_argument("--maxiter", type=int, default=60)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--robust", nargs="?", const="0.7,1.0,1.4", default=None,
                    help="thrust factors, e.g. 0.7,1,1.4: min control_ratio, constraints at every point")
    ap.add_argument("--enforce-thermal", action="store_true")
    ap.add_argument("--reference", default="tpu_rod",
                    help="stored study/designs/<D>in_<concept>.json to compare against ('' = none)")
    a = ap.parse_args(argv)
    global ENFORCE_THERMAL
    ENFORCE_THERMAL = a.enforce_thermal
    R.ROBUST = tuple(float(x) for x in a.robust.split(",")) if a.robust else None
    os.environ.setdefault("FLEXBLADE_REMOTE_TAG", "study")
    RESULTS.mkdir(parents=True, exist_ok=True)
    from flexblade import remote
    log = R.Logger(RESULTS / "ideal_evals.jsonl.gz")
    sizes = [float(x) for x in a.sizes.split(",")]
    with remote.HostPool(a.hosts) as pool:
        with ThreadPoolExecutor(max_workers=len(sizes)) as ex:
            specs = list(ex.map(lambda D: run_size(D, pool, log, a.popmult, a.maxiter, a.seed), sizes))
        if a.reference:
            try:
                reference(specs, a.reference, pool)
            except Exception as ex:          # scoring a real design must not lose the ideal run
                print(f"reference scoring failed: {ex!r}")
    log.close()
    for s in specs:
        st = s["stiffness"]
        print(f"{s['diameter_in']:g}in: cr {s['control_ratio']:.4f} feasible {s['feasible']} "
              f"bind {s['binding_constraint']} ({s['binding_utilisation']:.2f}) | skew {st['skew_deg']:.1f} "
              f"k_hinge {st['k_hinge']:.3g} k_cross {st['k_cross']:.3g} k_flap {st['k_flap']:.3g} "
              f"k_lp {st['k_lag_pitch']:.3g} | ea {s['mass']['elastic_axis_chord_frac']:.2f} "
              f"tip {s['mass']['tip_mass']*1e3:.2f} g @ {s['mass']['tip_mass_chord']*1e3:+.1f} mm | "
              f"lag {s['dynamics']['lag_per_rev']:.2f}/rev | ref {s.get('reference', {}).get('control_ratio')}",
              flush=True)


if __name__ == "__main__":
    main()
