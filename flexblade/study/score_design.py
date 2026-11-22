"""Canonical sa.py-basis "margin" score of one design (replaces the ad-hoc materials_5in /
cad/strap_score / i34tools/sa.py variants that disagreed on the 5" lead: 0.169 / 0.180 / 0.203).

Basis: robust.variants at thrust factors T 0.7 / 1.0 / 1.25 x motor.J_hub 0.34e-6 / 0.91e-6 / 2.0e-6
(nominal build, no perturbation / runtime samples), api.evaluate(trim, modes, harmonic, checks),
run_study.metrics criterion "margin", ground resonance on (FLEXBLADE_GROUND_RES=1, as sa.py).
Worst case over J per T and over everything overall. Presets as stored unless --resolve
(run_study.resolve_candidate(d, 6): zero-stress slot presets re-solved at T1.0, x PRESET_ALPHA).

usage: score_design.py <design.json> [--resolve] [--set k=v ...] [--workers N] [--save out.json]
  --set takes dotted paths into the design dict (value parsed as JSON, else string), e.g.
  --set flexure.strap_bend_factor=1e-4  (alias: strap.ei_factor=1e-4; strap.X -> flexure.strap_X)
Prints one JSON object: cr_pl per T (min over J), cr_pl/feasible overall, top hard violations,
and per T (min over J) hinge_k_free, pitch_k_eff, pitch/flap/lag per rev.
Lead 5in_v8s_2307_m245_301: cr_pl 0.321 / 0.212 / 0.169 (T0.7/1/1.25), feasible; --resolve 0.167.
Caveat: strap.ei_factor < ~1e-5 gives spurious strand-local rotational modes (beam DOFs of the
strand's inner nodes, f ~ sqrt(factor)) that wreck the score; use 1e-4 for a braided strand."""
from __future__ import annotations
import argparse, json, math, os, sys, time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

os.environ.setdefault("FLEXBLADE_GROUND_RES", "1")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust            # noqa: E402
from flexblade.schema import Design           # noqa: E402
import run_study as rs                        # noqa: E402

TF = (0.7, 1.0, 1.25)
JS = (0.34e-6, 0.91e-6, 2.0e-6)
ALIAS = {"strap.ei_factor": "flexure.strap_bend_factor"}
SKEYS = ("hinge_k_free", "pitch_k_eff", "pitch_k_struct", "pitch_k_prop", "pitch_freq_per_rev",
         "flap_freq_per_rev", "lag_freq_per_rev")


def apply_sets(dd: dict, sets: list[str]) -> dict:
    for s in sets:
        k, v = s.split("=", 1)
        k = ALIAS.get(k, "flexure.strap_" + k[6:] if k.startswith("strap.") else k)
        try:
            v = json.loads(v)
        except json.JSONDecodeError:
            pass
        node, *path = k.split(".")
        cur = dd
        for p in [node] + path[:-1]:
            cur = cur[int(p)] if isinstance(cur, list) else cur[p]
        last = path[-1] if path else node
        if isinstance(cur, dict) and last not in cur:
            raise KeyError(f"--set {k}: no field {last!r}")
        cur[int(last) if isinstance(cur, list) else last] = v
    return dd


def ev(dv: Design) -> dict:
    try:
        r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    except Exception as e:  # noqa: BLE001
        return dict(err=repr(e)[:300])
    m = rs.metrics(dv, r, criterion="margin")
    S = r.get("summary", {})
    top = sorted(((c["name"] + ":" + str(c.get("member")), round(c["utilisation"], 3))
                  for c in r.get("checks", [])), key=lambda x: -x[1])[:3]
    return dict(cr_pl=m.get("cr_pl"), feas=bool(m.get("feasible_pl")), viol=m.get("viol_hard") or {},
                top=top, **{k: S.get(k) for k in SKEYS})


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("design")
    ap.add_argument("--resolve", action="store_true")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--save")
    a = ap.parse_args(argv)
    dd = apply_sets(json.loads(Path(a.design).read_text()), a.set)
    d = Design.model_validate(dd)
    t0 = time.time()
    if a.resolve:
        d = rs.resolve_candidate(d, 6)
    if a.save:
        Path(a.save).write_text(d.model_dump_json(indent=1))
    v = robust.variants(d, TF, None, list(JS))
    if a.workers > 1:
        with ProcessPoolExecutor(a.workers) as ex:
            ms = list(ex.map(ev, [dv for _, dv in v], chunksize=1))
    else:
        ms = [ev(dv) for _, dv in v]
    pts = dict(zip((t for t, _ in v), ms))
    out = dict(design=Path(a.design).name, resolve=a.resolve, set=a.set, cr_pl_T={}, per_T={})
    viol: dict[str, float] = {}
    for f in TF:
        g = [m for t, m in pts.items() if t.startswith(f"T{f:g}:")]
        ok = [m for m in g if "err" not in m and m["cr_pl"] is not None]
        out["cr_pl_T"][f"{f:g}"] = round(min(m["cr_pl"] for m in ok), 4) if len(ok) == len(g) else None
        row = {k: (round(min(x), 5) if (x := [m[k] for m in ok if m[k] is not None and math.isfinite(m[k])])
                   else None) for k in SKEYS}
        row["feas"] = len(ok) == len(g) and all(m["feas"] for m in ok)
        row["top"] = max((m["top"] for m in ok), key=lambda x: x[0][1] if x else 0, default=None)
        out["per_T"][f"{f:g}"] = row
        for m in ok:
            for k, x in m["viol"].items():
                viol[k] = max(viol.get(k, 0.0), round(float(x), 4))
    errs = {t: m["err"] for t, m in pts.items() if "err" in m}
    crs = [c for c in out["cr_pl_T"].values()]
    out.update(cr_pl=min(crs) if None not in crs else None,
               feasible=all(r["feas"] for r in out["per_T"].values()),
               viol=dict(sorted(viol.items(), key=lambda kv: -kv[1])[:5]), errors=errs or None,
               secs=round(time.time() - t0))
    print(json.dumps(out))
    return out


if __name__ == "__main__":
    main()
