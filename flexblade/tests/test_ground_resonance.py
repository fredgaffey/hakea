import math
from flexblade import ground_resonance as gr


def test_default_ranges():
    assert gr.default_arm_range(3.5 * 0.0254) == (200.0, 300.0)
    assert gr.default_arm_range(5.0 * 0.0254) == (120.0, 250.0)
    lo, hi = gr.default_arm_range(2.0 * 0.0254)
    assert lo > 200 and hi > 300


def test_lag_mode_pick():
    modes = [dict(freq_hz=0.0, participation=dict(lag=0.5)), dict(freq_hz=100.0, participation=dict(lag=0.9)),
             dict(freq_hz=300.0, participation=dict(lag=0.1))]
    assert abs(gr.lag_per_rev(modes, 6000.0) - 1.0) < 1e-9
    assert math.isnan(gr.lag_per_rev([], 6000.0))


def test_known_bands_7in_and_10in():
    # 7in v6, T1.4: 0.522/rev at 15622 rpm -> f_reg 125 Hz; known unstable arms 75-175 Hz, spec 200-300 Hz
    kw = dict(lag_per_rev=0.522, rpm=15622.0)
    for f in (115, 125, 140):
        assert gr.violation(arm_range=(f, f), **kw) > 0.3
    assert gr.violation(arm_range=(200, 300), **kw) == 0.0
    # 10in v5, T1.4: 0.547/rev at 9093 rpm -> 69 Hz; known band 50-95, spec >= 120
    kw = dict(lag_per_rev=0.547, rpm=9093.0)
    assert gr.violation(arm_range=(70, 70), **kw) > 0.9
    assert gr.violation(arm_range=(120, 250), **kw) == 0.0


def test_nan_is_ignored():
    assert gr.violation(float("nan"), 6000.0, (200, 300)) == 0.0
