import pytest

from flexblade import api, cache, optimize, presets, remote


@pytest.fixture
def db(tmp_path, monkeypatch):
    monkeypatch.setenv("FLEXBLADE_CACHE", str(tmp_path / "c.sqlite"))
    for k in list(cache.STATS):
        cache.STATS[k] = 0
    return tmp_path / "c.sqlite"


def _d(t=0.0008):
    return optimize.set_design(presets.make_preset(5, "cf_flexure"), {"flexure.thickness": t, "op.rpm": 3000})


def test_hit_identical_and_design_change_misses(db):
    a = api.evaluate(_d(), ["trim"])
    assert cache.STATS["puts"] == 1
    b = api.evaluate(_d(), ["trim"])
    assert cache.STATS["hits"] == 1
    assert a.model_dump(mode="json") == b.model_dump(mode="json")
    api.evaluate(_d(0.0009), ["trim"])
    assert cache.STATS["hits"] == 1 and cache.STATS["puts"] == 2


def test_model_version_change_misses(db, monkeypatch):
    cache.put(_d(), {"x": 1}, "k")
    assert cache.get(_d(), "k") == {"x": 1}
    with monkeypatch.context() as m:
        m.setattr(cache, "model_version", lambda: "other")
        assert cache.get(_d(), "k") is None
        cache.put(_d(), {"x": 2}, "k")
    assert cache.get(_d(), "k") == {"x": 1}
    assert cache.prune(1) in (0, 1)


def test_off(monkeypatch):
    monkeypatch.setenv("FLEXBLADE_CACHE", "off")
    cache.put(_d(), {"x": 1})
    assert cache.get(_d()) is None and cache.stats() == {"enabled": False}


def test_pool_map_dispatches_unique_misses(db):
    ds = [_d(0.0006), _d(0.0007), _d(0.0006), _d(0.0007), _d(0.0006)]
    seen = []

    class Fake(remote.HostPool):
        def __init__(self):
            self.what, self.full = ["trim"], False

        def imap(self, designs):
            for i, d in enumerate(designs):
                seen.append(d)
                yield i, {"id": i, "results": {"v": d.flexure.thickness}}

    p = Fake()
    out = p.map(ds)
    assert len(seen) == 2
    assert [r["results"]["v"] for r in out] == [d.flexure.thickness for d in ds]
    assert [r["id"] for r in out] == list(range(5))
    seen.clear()
    p.map(ds)
    assert seen == []
