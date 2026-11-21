from flexblade import robust


def _c(z):
    return {"viol_hard": {"cqc_mech_margin": 1.0, "cqc_mech_side": 1.0, "cqc_gain": 0.5},
            "per_node": {"a": {"mech_zeta": 0.05}, "b": {"mech_zeta": z}}}


def test_drops_margin_side_keeps_others():
    assert set(robust.minimal_gates(_c(0.041))) == {"cqc_gain"}


def test_flags_low_zeta():
    v = robust.minimal_gates(_c(0.01))
    assert abs(v["cqc_mech_zeta"] - 0.5) < 1e-9


def test_missing_zeta_is_violation():
    assert "cqc_mech_zeta" in robust.minimal_gates({"viol_hard": {}, "per_node": {}})
