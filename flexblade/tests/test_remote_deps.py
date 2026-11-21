"""Regression (DE8 7in/10in all-fail): a remote venv without manifold3d fails every steelx eval
(rotor -> fittings), fast, so work stealing handed it ~all jobs. Bootstrap must install and probe it."""
from flexblade import remote


def test_deps_include_manifold3d():
    assert "manifold3d" in remote.DEPS.split()


def test_bootstrap_probe_requires_manifold3d(monkeypatch):
    cmds = []

    def fake(host, cmd, timeout=900, check=True):
        cmds.append(cmd)
        class R: stdout = ""
        return R()
    monkeypatch.setattr(remote, "_ssh", fake)
    monkeypatch.setattr(remote, "sync", lambda *a, **k: None)
    remote.bootstrap("h")
    assert "manifold3d" in cmds[0] and "manifold3d" in cmds[1]
