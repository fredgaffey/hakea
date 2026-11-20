"""BLAS thread policy.

Measured (32-core laptop, idle): dense solve n=300: 1 thread 0.48 ms ≈ 16 threads;
n=1000: 13.9 → 4.9 ms; n=2000: 91 → 30 ms. Under load, threads on small matrices
were ~500× SLOWER (contention). So: 1 thread by default (set in __init__), and
`blas_threads_for(n)` raises it for big dense work — unless this process is one of
many parallel workers (FLEXBLADE_WORKER=1), where processes already fill the cores.

    with blas_threads_for(K.shape[0]):
        x = np.linalg.solve(K, f)
"""
from __future__ import annotations

import contextlib
import os

SMALL_N = 500


def _limits(n: int):
    try:
        from threadpoolctl import threadpool_limits
        return threadpool_limits(n)
    except Exception:  # noqa: BLE001
        return contextlib.nullcontext()


def blas_threads_for(n_dof: int, max_threads: int | None = None):
    if os.environ.get("FLEXBLADE_WORKER") == "1" or n_dof < SMALL_N:
        return _limits(1)
    cap = max_threads or min(16, os.cpu_count() or 1)
    return _limits(max(1, min(cap, n_dof // 125)))
