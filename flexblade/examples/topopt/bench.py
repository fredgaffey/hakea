"""Timing / GPU memory of the topopt iteration vs resolution (a few MMA iterations each).

    PYTHONPATH=flexblade python flexblade/examples/topopt/bench.py [preset] [n_elem ...]
"""
from __future__ import annotations

import json
import sys

import torch

from flexblade.presets import all_presets
from flexblade.topopt import TopoptOptions
from flexblade.topopt.run import run


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "5in_cf_flexure"
    sizes = [int(v) for v in sys.argv[2:]] or [50_000, 150_000, 400_000, 1_000_000]
    d = all_presets()[name]
    out = []
    for n in sizes:
        torch.cuda.empty_cache()
        torch.cuda.reset_peak_memory_stats()
        res = run(d, TopoptOptions(n_elem=n, max_iter=6, verbose=False))
        h = res.history[1:]          # skip the first (cold-start) iteration
        out.append(dict(n_elem=int(res.grid["shape"][0] * res.grid["shape"][1] * res.grid["shape"][2]),
                        shape=res.grid["shape"], mg_levels=res.grid["mg_levels"],
                        s_per_iter=sum(r["time"] for r in h) / len(h),
                        cg_iters=[r["cg_iters"] for r in h], adj_iters=[r["adj_iters"] for r in h],
                        gpu_mem_peak_MB=torch.cuda.max_memory_allocated() / 1e6))
        print(json.dumps(out[-1]), flush=True)


if __name__ == "__main__":
    main()
