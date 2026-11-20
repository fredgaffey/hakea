"""E04: re-rank grid leaders by FACT intent — hinge-axis error (softest rotation vs intended axis).
Top 60 feasible per method (+ top 150 overall), re-evaluated with the new summary metrics."""
import json, sys, warnings, multiprocessing as mp
from pathlib import Path
warnings.filterwarnings("ignore")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent.parent))
import full_grid as fg, reinforce_grid as rg
from flexblade import api

def run(args):
    tag, d = args
    try:
        r = api.evaluate(d)
        s = r.summary
        return dict(tag=tag, ctrl=r.harmonic.control_ratio, max_util=max(c.utilisation for c in r.checks),
                    axis_err=s.get("hinge_axis_err_deg"), k_ratio=s.get("hinge_k_ratio"),
                    k_free=s.get("hinge_k_free"), lag=s.get("lag_freq_per_rev"))
    except Exception as e:
        return dict(tag=tag, error=str(e)[:120])

if __name__ == "__main__":
    rows = [json.loads(l) for l in open(HERE.parent / "full_grid/results.jsonl")]
    feas = [r for r in rows if r.get("ok") and r["converged"] and (r["max_util"] or 9) <= 1 and abs(r.get("split_deg") or 0) <= 2]
    pick = set(r["tag"] for r in sorted(feas, key=lambda r: -(r["control_ratio"] or 0))[:150])
    for m in {r["method"] for r in feas}:
        pick |= set(r["tag"] for r in sorted([r for r in feas if r["method"] == m], key=lambda r: -(r["control_ratio"] or 0))[:60])
    want = {}
    for D in sorted({int(t.split("|")[0]) for t in pick}):
        b = rg.base_for(D); b.blade.elastic_axis_chord_frac = 0.43
        for meta, d in fg.variants(b, D):
            if meta["tag"] in pick: want[meta["tag"]] = d
    with mp.get_context("fork").Pool(6) as p:
        out = p.map(run, list(want.items()))
    (HERE / "e04_results.json").write_text(json.dumps(out, indent=1))
    ok = [o for o in out if "error" not in o]
    good = sorted([o for o in ok if (o["axis_err"] or 90) <= 20], key=lambda o: -o["ctrl"])
    lines = [f"{len(ok)} evaluated; {len(good)} with hinge-axis error ≤ 20° (FACT-true)", "",
             "| tag | ctrl | axis err° | k_ratio | lag/rev | max util |", "|---|---|---|---|---|---|"]
    lines += [f"| {o['tag']} | {o['ctrl']:.3f} | {o['axis_err']:.1f} | {o['k_ratio']:.1f} | {(o['lag'] or 0):.2f} | {o['max_util']:.2f} |" for o in good[:30]]
    by = {}
    for o in ok: by.setdefault(o["tag"].split("|")[1], []).append(o["axis_err"] or 90)
    lines += ["", "median axis error by method: " + ", ".join(f"{m} {sorted(v)[len(v)//2]:.0f}°" for m, v in by.items())]
    (HERE / "e04_summary.md").write_text("\n".join(lines) + "\n")
    print("done")
