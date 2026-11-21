from flexblade.remote import HostHealth


def test_quarantine_after_20_with_half_errors():
    h = HostHealth()
    fired = [h.record("a", {"error": "No module named manifold3d"}) for _ in range(19)]
    assert not any(fired) and not h.bad
    assert h.record("a", {"error": "x"}) and h.bad == {"a"}
    assert h.first["a"] == "No module named manifold3d"
    assert not h.record("a", {"error": "x"})   # fires once


def test_healthy_host_not_quarantined():
    h = HostHealth()
    for i in range(100):
        assert not h.record("b", {"error": "e"} if i % 10 == 0 else {"ok": 1})
    assert not h.bad
