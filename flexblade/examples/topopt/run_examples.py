"""Topology-optimisation example runs (GPU): 5in_cf_flexure and 10in_tpu_rod presets.

    PYTHONPATH=flexblade python flexblade/examples/topopt/run_examples.py [n_elem] [max_iter]

Writes per preset into flexblade/examples/topopt/<preset>/: render.png, summary.json,
result.json, design_topopt.json (Flexure.topology = "topopt", loadable by `flexblade
evaluate`), <preset>_soft.stl / _stiff.stl (mm), <preset>_topopt.step (faceted, mm),
fields.npz (git-ignored). Equivalent CLI:

    flexblade topopt preset:5in_cf_flexure --out flexblade/examples/topopt/5in_cf_flexure --evaluate
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

from flexblade.presets import all_presets
from flexblade.topopt import TopoptOptions, optimise

HERE = Path(__file__).resolve().parent


def main():
    n_elem = int(sys.argv[1]) if len(sys.argv) > 1 else 100_000
    max_iter = int(sys.argv[2]) if len(sys.argv) > 2 else 200
    names = sys.argv[3:] or ["5in_cf_flexure", "10in_tpu_rod"]
    P = all_presets()
    for name in names:
        t0 = time.time()
        s = optimise(P[name], HERE / name, TopoptOptions(n_elem=n_elem, max_iter=max_iter),
                     validate=True, evaluate=True)
        s.pop("result")
        s.pop("design_topopt")
        s["wall_s"] = time.time() - t0
        print(json.dumps(s, indent=1, default=str))


if __name__ == "__main__":
    main()
