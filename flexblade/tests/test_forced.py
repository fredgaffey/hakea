"""nP forced response at a mode/harmonic crossing (forced.py) + check plumbing."""
import sys
from pathlib import Path

import numpy as np
import scipy.linalg as sl

from flexblade import forced as F


def test_sdof_resonant_amplitude():
    m, k, z, f0 = 0.3, 1.2e4, 0.02, 2.5
    w = np.sqrt(k / m)
    q = F.resonant_amplitude(np.array([1.0 + 0j]), np.array([[m]]), np.array([f0]), w, z)
    assert np.isclose(q, f0 / (2 * z * k))
    # exact SDOF: |X| = F / |k - m w^2 + i c w| with c = 2 z sqrt(k m)
    c = 2 * z * np.sqrt(k * m)
    assert np.isclose(q, f0 / abs(k - m * w * w + 1j * c * w))
    # shape scaling does not change the physical response
    q2 = F.resonant_amplitude(np.array([3.0 + 0j]), np.array([[m]]), np.array([f0]), w, z)
    assert np.isclose(3 * q2, q)


def test_mdof_modal_participation():
    # 2-DOF: response at mode-1 resonance from the modal projection vs a direct solve
    M = np.diag([1.0, 2.0])
    K = np.array([[3.0, -1.0], [-1.0, 1.0]]) * 1e3
    lam, X = sl.eigh(K, M)
    x = X[:, 0]
    w, z = np.sqrt(lam[0]), 0.01
    Fv = np.array([1.0, 0.0])
    q = F.resonant_amplitude(x.astype(complex), M, Fv, w, z)
    C = M @ X @ np.diag(2 * z * np.sqrt(lam)) @ X.T @ M     # modal damping
    U = np.linalg.solve(K - w * w * M + 1j * w * C, Fv)
    assert np.isclose(np.linalg.norm(x * q), np.linalg.norm(U), rtol=2e-2)


def test_combine_and_constants():
    assert np.isclose(F.combine(0.3, 0.4, "srss"), 0.5)
    assert np.isclose(F.combine(0.3, 0.4, "sum"), 0.7)
    assert F.torque_frac(2) == 0.03 and F.torque_frac(3) == 0.01 and F.torque_frac(5) == 0.003


def test_resonance_margin_ignores_np_blade_modes():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "study"))
    import run_study as R
    r = {"summary": {"rpm": 600.0}, "modes": [
        {"freq_hz": 30.0, "damping_ratio": 0.01, "label": "pitch sym", "participation": {"hub": 0.0}},
        {"freq_hz": 25.0, "damping_ratio": 0.01, "label": "lag sym", "participation": {"hub": 0.0}}]}
    mg, wh = R.resonance_margin(r)            # 3.0/rev pitch on 3P, 2.5/rev lag: only 1P counts
    assert mg > 1.0, wh
    assert any("3P" in s for s in R.np_crossings_info(r))
    r["modes"][1]["participation"]["hub"] = 0.5   # hub mode near 2P: the 2P margin applies
    r["modes"][1]["freq_hz"] = 20.5
    mg, wh = R.resonance_margin(r)
    assert mg < 0.05 and "2P" in wh
