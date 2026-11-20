"""flexblade: flexure / compliant-blade simulator for swashplateless cyclic rotors."""
import os as _os

# Our matrices are small (hundreds of DOFs). Multithreaded BLAS on them is ~500x slower
# than single-threaded (thread spin-up/contention, worse on a busy 32-core box), and we
# parallelise across designs with processes instead. Must run before numpy is imported;
# FLEXBLADE_BLAS_THREADS overrides.
_n = _os.environ.get("FLEXBLADE_BLAS_THREADS", "1")
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
           "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
    _os.environ.setdefault(_v, _n)
try:  # numpy already imported by the host process → limit the live pools too
    import sys as _sys
    if "numpy" in _sys.modules:
        from threadpoolctl import threadpool_limits as _tl
        _tl(int(_n))
except Exception:  # noqa: BLE001
    pass

from .schema import MODEL_VERSION, Design, Results  # noqa: E402,F401

__version__ = MODEL_VERSION
