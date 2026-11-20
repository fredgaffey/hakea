"""E03: robustness of grid leaders to flexure stiffness (CalculiX suggests real hinge may be
2-3x softer than beam). Re-evaluate top feasible designs with TPU modulus x {0.4, 0.7, 1.0, 1.15}."""
import dataclasses, json, sys, warnings, multiprocessing as mp
from pathlib import Path
warnings.filterwarnings("ignore")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent.parent))
import full_grid as fg, reinforce_grid as rg
from flexblade import api, materials
SCALES = (0.4, 0.7, 1.0, 1.15)
for base in ("tpu_95a", "tpu_64d"):
    m = materials.LIBRARY[base]
    for s in SCALES:
        materials.LIBRARY[f"{base}_x{s}"] = dataclasses.replace(
            m, name=f"{base}_x{s}", E1=m.E1*s, E2=m.E2*s, E3=m.E3*s, G12=m.G12*s, G13=m.G13*s, G23=m.G23*s)

def designs_for_tags(tags):
    want = {t: None for t in tags}
    for D in sorted({int(t.split("|")[0]) for t in tags}):
        b = rg.base_for(D); b.blade.elastic_axis_chord_frac = 0.43
        for meta, d in fg.variants(b, D):
            if meta["tag"] in want: want[meta["tag"]] = d
    return want

def run(args):
    tag, d, s = args
    d = d.model_copy(deep=True); lay = d.flexure.layup
    if lay.material.startswith("tpu"): lay.material = f"{lay.material}_x{s}"
    try:
        r = api.evaluate(d); u = max(c.utilisation for c in r.checks)
        return dict(tag=tag, scale=s, ctrl=r.harmonic.control_ratio, max_util=u,
                    lag=r.summary.get("lag_freq_per_rev"), warn=len(r.warnings))
    except Exception as e:
        return dict(tag=tag, scale=s, error=str(e)[:120])

if __name__ == "__main__":
    rows = [json.loads(l) for l in open(HERE.parent / "full_grid/results.jsonl")]
    feas = sorted([r for r in rows if r.get("ok") and r["converged"] and (r["max_util"] or 9) <= 1
                   and abs(r.get("split_deg") or 0) <= 2], key=lambda r: -(r["control_ratio"] or 0))[:25]
    ds = designs_for_tags([r["tag"] for r in feas])
    jobs = [(t, d, s) for t, d in ds.items() if d is not None for s in SCALES]
    with mp.get_context("fork").Pool(6) as p:
        out = p.map(run, jobs)
    (HERE / "e03_results.json").write_text(json.dumps(out, indent=1))
    by = {}
    for o in out: by.setdefault(o["tag"], {})[o["scale"]] = o
    lines = ["| tag | ctrl@0.4 | ctrl@0.7 | ctrl@1.0 | ctrl@1.15 | worst util | lag@0.4 |", "|---|---|---|---|---|---|---|"]
    for t, v in by.items():
        g = lambda s, k: (v.get(s) or {}).get(k)
        lines.append(f"| {t} | " + " | ".join(f"{(g(s,'ctrl') or 0):.3f}" for s in SCALES)
                     + f" | {max((g(s,'max_util') or 9) for s in SCALES):.2f} | {(g(0.4,'lag') or 0):.2f} |")
    (HERE / "e03_summary.md").write_text("\n".join(lines) + "\n")
    print("done")
