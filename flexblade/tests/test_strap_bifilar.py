"""Bifilar tension-torsion strap: two strands at +-b about the radial line through the hinge point.
Their geometric (tension) stiffness about the radial (pitch) axis is k = T b²/L (T = total strand
tension ∝ Ω², L = free length), i.e. a pendulum-like pitch spring whose /rev contribution is
rpm-invariant. Checked against the co-rotational FE model (strap-only condensed stiffness, the
other rotations condensed out) at two rpms."""
import numpy as np
import pytest

from flexblade import presets, rotor
from flexblade.beam import assemble_internal, element_end_forces

B, L = 0.0015, 0.004


def _design(r, rpm):
    d = presets.make_preset(5.0, "cf_flexure")
    u = dict(topology="skewed_cruciform", skew_deg=0.0, thickness=0.0005, width=0.02, bar_length=0.02,
             cruciform_rod_diameter=0.0, strap_strands=2, strap_strand_diameter=0.0006,
             strap_radius=r, strap_length=L, axial_release=True)
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=u),
                                    op=d.op.model_copy(update=dict(rpm=rpm)),
                                    analysis=d.analysis.model_copy(update=dict(aero="none"))))


def _strap_pitch(r, rpm):
    m = rotor.build_model(_design(r, rpm))
    ss = rotor.static_trim(m, None, rpm=rpm)
    K = rotor.flexure_stiffness(m, ss, cables_only=True)[3:, 3:]
    k_pitch = 1.0 / np.linalg.inv(K)[0, 0]          # pitch with flap/lag rotation free
    cab = {f"flex{j}" for j, mm in enumerate(m.blades[0].members) if mm.cable}
    sa = assemble_internal(m.fe, ss.state, False)
    T = {}
    for e, c in zip(m.fe.elements, sa.cr):
        if e.blade == 1 and e.kind == "flexure" and e.tag.split(".", 1)[-1] in cab:
            T.setdefault(e.tag, abs(float(element_end_forces(e, c)[0][0])))
    return k_pitch, sum(T.values())


def test_bifilar_pitch_stiffness_vs_omega2():
    ks = []
    for rpm in (6000, 12000):
        k0, _ = _strap_pitch(0.0, rpm)
        k, T = _strap_pitch(B, rpm)
        an = T * B ** 2 / L
        assert k - k0 == pytest.approx(an, rel=0.25)   # FE: +21 % / +9 % (blade CF deflection)
        ks.append((k - k0) / rpm ** 2)
    # ∝ Ω²: the per-rev contribution does not drift with rpm
    assert ks[1] == pytest.approx(ks[0], rel=0.3)
