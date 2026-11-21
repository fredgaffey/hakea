from flexblade import rotor
from flexblade.schema import Flexure


def test_strip_clamp_factor_ordering():
    t = 1.5e-4
    sharp, _ = rotor.strip_clamp_factor(Flexure(thickness=t, strip_clamp="sharp"))
    rad, R = rotor.strip_clamp_factor(Flexure(thickness=t, strip_clamp="radiused"))
    tiny, _ = rotor.strip_clamp_factor(Flexure(thickness=t, strip_clamp="radiused",
                                               strip_clamp_radius=2 * t))
    comp, _ = rotor.strip_clamp_factor(Flexure(thickness=t, strip_clamp="compliant"))
    assert abs(R - 10 * t) < 1e-12                      # default radius = 10 t
    assert abs(rad - rotor.KT_RADIUSED / rotor.FRET_RADIUSED) < 1e-9
    assert comp < rad < tiny < sharp                    # small jaw radius degrades towards sharp
    assert Flexure().strip_clamp == "radiused"
