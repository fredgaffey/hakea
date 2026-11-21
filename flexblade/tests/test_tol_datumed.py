import importlib
from flexblade import robust


def test_default_unchanged():
    assert robust.CQ_PERTURB[0]["flexure.hinge_radius"] == 1.05
    assert robust.tol_ranges()["width"] == 0.05


def test_datumed(monkeypatch):
    monkeypatch.setenv("CQ_TOL", "datumed")
    try:
        r = importlib.reload(robust)
        p = r.CQ_PERTURB
        assert p[0] == {"flexure.thickness": 1.05, "flexure.width": 1.02,
                        "flexure.length": 0.98, "flexure.hinge_radius": 1.015}
        assert abs(p[1]["flexure.hinge_radius"] - 0.985) < 1e-12 and len(p) == 4
    finally:
        monkeypatch.delenv("CQ_TOL")
        importlib.reload(robust)
