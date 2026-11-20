"""E07: IDEALISE-THEN-REALISE. Replace the flexure by an ideal skewed hinge spring (k_hinge about
the skewed axis, everything else very stiff) and map which hinge stiffness / skew / hinge radius /
blade mass balance gives robust control authority. Output = the target spec any manufacturable
flexure must realise. 2307 motor, 0.6 kg, pulse at capability, PA6-CF blade (reinforce_grid.base_for)."""
import itertools, json, sys, warnings, multiprocessing as mp
from pathlib import Path
import numpy as np
warnings.filterwarnings("ignore")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent))
import reinforce_grid as rg
rg.MOTOR, rg.MASS = "tmotor_2307_1300kv", 0.6
from flexblade import rotor

def ctrl(d, k):
    m = rotor.build_model(d, ideal_hinge=dict(k_hinge=k, k_rot=1e5, k_trans=1e9))
    aero = rotor.make_aero(m); ss = rotor.static_trim(m, aero); hd = rotor.harmonic_1p(m, ss, aero)
    return hd, ss

def run(args):
    D, sk, hr, ea, k = args
    try:
        d = BASE[D].model_copy(deep=True)
        d.flexure.skew_deg, d.flexure.hinge_radius = sk, BASE[D].flexure.hinge_radius * hr
        d.blade.elastic_axis_chord_frac = ea
        out = dict(D=D, skew=sk, hr=hr, ea=ea, k=k)
        for f in (0.6, 1.0, 1.5):          # robustness band on hinge stiffness
            hd, ss = ctrl(d, k * f)
            out[f"ctrl_{f}"] = float(hd.result.control_ratio)
            if f == 1.0:
                out["pitch"] = float(hd.result.blades[0].pitch_amp_deg)
                out["rpm"] = float(ss.rpm)
        out["ctrl_robust"] = min(out["ctrl_0.6"], out["ctrl_1.0"], out["ctrl_1.5"])
        return out
    except Exception as e:
        return dict(D=D, skew=sk, hr=hr, ea=ea, k=k, error=str(e)[:100])

BASE = {}
if __name__ == "__main__":
    for D in (5, 6, 7):
        BASE[D] = rg.base_for(D)
    ks = list(np.round(np.geomspace(0.1, 30, 16), 4))
    jobs = list(itertools.product((5, 6, 7), (30, 40, 50, 60), (0.7, 1.0, 1.3), (0.38, 0.43, 0.48), ks))
    with mp.get_context("fork").Pool(6) as p:
        out = p.map(run, jobs, chunksize=4)
    (HERE / "e07_results.json").write_text(json.dumps(out))
    ok = [o for o in out if "error" not in o]
    lines = ["Ideal-hinge target map: best ROBUST control (min over hinge stiffness ×0.6/1.0/1.5)", "",
             "| size | skew | hinge r× | EA x/c | k_hinge N·m/rad | ctrl robust | ctrl nominal | pitch° |", "|---|---|---|---|---|---|---|---|"]
    for D in (5, 6, 7):
        best = sorted([o for o in ok if o["D"] == D], key=lambda o: -o["ctrl_robust"])[:5]
        lines += [f"| {o['D']}\" | {o['skew']} | {o['hr']} | {o['ea']} | {o['k']} | {o['ctrl_robust']:.3f} | {o['ctrl_1.0']:.3f} | {o.get('pitch',0):.2f} |" for o in best]
    lines.append(f"\n{len(ok)} ok / {len(out)} runs")
    (HERE / "e07_summary.md").write_text("\n".join(lines) + "\n")
    print("done")
