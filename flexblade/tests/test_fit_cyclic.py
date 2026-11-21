import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "study" / "calib"))
import fit_cyclic as fc   # noqa: E402
import synth              # noqa: E402


def test_lockin_known_sinusoid():
    psi = np.linspace(0, 2 * np.pi * 50, 5000, endpoint=False)
    y = 0.7 * np.cos(psi + 0.3) + 0.4 + 0.2 * np.cos(2 * psi)       # DC and 2P must be rejected
    a = fc.lockin(y, psi, 1)
    assert abs(abs(a) - 0.7) < 1e-9 and abs(np.angle(a) - 0.3) < 1e-9


def test_window_regression_recovers_G_with_vibration():
    rng = np.random.default_rng(1)
    n = 4000
    t = np.arange(n) / 2000
    psi = 2 * np.pi * 110 * t
    ph = 2 * np.pi * 2 * t
    G = 0.6 * np.exp(1j * np.radians(97))
    z = G * 0.1 * np.exp(1j * ph) + 0.3 * np.exp(1j * psi) + 0.1 + 0.01 * (rng.standard_normal(n) + 1j * rng.standard_normal(n))
    g, s, _ = fc.window_G(z, psi, np.full(n, 0.1), ph)
    assert abs(g / G - 1) < 0.01


def test_fit_recovers_synthetic_sweep():
    rows, gerr, perr = synth.validate(5, dur=20.0, amp=0.1, imu_noise=0.05)
    assert len(rows) == 5 and gerr < 5 and perr < 3
    assert all(r["gain_ci"] > 0 and r["phase_ci"] > 0 for r in rows)
    assert all(a["erpm"] < b["erpm"] for a, b in zip(rows, rows[1:]))
