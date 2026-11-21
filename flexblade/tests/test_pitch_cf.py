"""pitch_cf: CF propeller-moment vs pitch-stiffness ("CF divergence") ratio + balance masses."""
import math
from pathlib import Path

import numpy as np

from flexblade import rotor
from flexblade.schema import BalanceMass, Design

D7 = Path(__file__).resolve().parents[1] / "study" / "designs" / "7in_tt_strap_steelx_v3.json"


def _design(balance=()):
    d = Design.model_validate_json(D7.read_text())
    return d.model_copy(update=dict(
        flexure=d.flexure.model_copy(update=dict(pre_lag_deg=0.0)),
        blade=d.blade.model_copy(update=dict(balance_masses=list(balance)))))


def test_pitch_cf_propeller_term_and_chinese_weight():
    Om = 13000 * math.pi / 30
    m0 = rotor.build_model(_design())
    K6 = rotor.flexure_stiffness(m0, None)
    a = rotor.pitch_cf(m0, Om, K6)
    b = rotor.pitch_cf(m0, 2 * Om, K6)
    # propeller-moment stiffness: spring to flat pitch, ∝ Ω²
    assert a["pitch_k_prop"] > 0
    assert math.isclose(b["pitch_k_prop"], 4 * a["pitch_k_prop"], rel_tol=1e-9)
    assert a["pitch_k_aero"] < 0          # EA (0.38c) aft of the AC: destabilising
    assert 0 < a["pitch_cf_ratio"] < rotor.PITCH_CF_RATIO_MAX
    # a ±h pair normal to the chord at the root lowers k_prop by 2 m h² Ω² cos 2θ (θ: section pitch)
    m, h, rf = 0.001, 0.01, 0.2
    bal = [BalanceMass(mass=m, r_frac=rf, normal_offset=s * h) for s in (1, -1)]
    m1 = rotor.build_model(_design(bal))
    c = rotor.pitch_cf(m1, Om, K6)
    el = next(e for e in m1.fe.elements if e.blade == 1 and e.extra_mpts is not None)
    th = math.atan2(el.E0[2, 1], el.E0[1, 1])      # section pitch of the carrying element
    dk = a["pitch_k_prop"] - c["pitch_k_prop"]
    assert math.isclose(dk, 2 * m * h * h * Om ** 2 * math.cos(2 * th), rel_tol=0.05)
    # the balance masses are real mass points (CF + inertia) of the blade-1 elements
    xm = [e.mpts[-2:] for e in m1.fe.elements if e.blade == 1 and e.extra_mpts is not None and e.kind != "rigid"]
    assert len(xm) == 1 and np.allclose(sorted(xm[0][:, 2]), [-h, h])
