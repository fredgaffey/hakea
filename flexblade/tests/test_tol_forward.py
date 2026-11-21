import json
from flexblade import cache, remote


def test_cache_key_includes_cq_tol(monkeypatch):
    monkeypatch.delenv("CQ_TOL", raising=False)
    a = cache.make_key({"x": 1}, "k")
    monkeypatch.setenv("CQ_TOL", "datumed")
    b = cache.make_key({"x": 1}, "k")
    assert a != b


def test_forward_env_and_job_applies(monkeypatch):
    monkeypatch.setenv("CQ_TOL", "datumed")
    assert remote.forward_env() == {"CQ_TOL": "datumed"}
    monkeypatch.setenv("CQ_TOL", "other")
    monkeypatch.setattr("flexblade.api.evaluate", lambda d, w: (_ for _ in ()).throw(RuntimeError("x")))
    r = remote._eval_job({"id": 1, "design": {}, "env": {"CQ_TOL": "datumed"}})
    import os
    assert os.environ["CQ_TOL"] == "datumed" and "error" in r
