"""HostPool (local host only): repeated and concurrent map calls route results correctly,
and optimize.run accepts it as a Design mapper."""
import threading

from flexblade import optimize, presets, remote
from flexblade.schema import OptimizeRequest, Target, Variable


def test_hostpool_repeated_and_threaded_map():
    d = presets.make_preset(5, "cf_flexure")
    designs = [optimize.set_design(d, {"flexure.thickness": t}) for t in (0.0006, 0.0008, 0.0010)]
    with remote.HostPool("local:2", what=["trim"], setup=False) as pool:
        a = pool.map(designs)
        b = pool.map(designs[::-1])
        assert [r["results"]["design_hash"] for r in a] == [x.hash() for x in designs]
        assert [r["results"]["design_hash"] for r in b] == [x.hash() for x in designs[::-1]]
        out = {}

        def go(k, ds):
            out[k] = pool.map(ds)
        ts = [threading.Thread(target=go, args=(k, designs[k:] + designs[:k])) for k in range(3)]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
        for k in range(3):
            assert [r["results"]["design_hash"] for r in out[k]] == \
                [x.hash() for x in designs[k:] + designs[:k]]


def test_optimize_with_design_mapper():
    d = presets.make_preset(5, "cf_flexure")
    req = OptimizeRequest(design=d, variables=[Variable(path="flexure.thickness", lo=0.0005, hi=0.0012)],
                          targets=[Target(quantity="trim.thrust", kind="max")], max_evals=12)
    with remote.HostPool("local:2", setup=False) as pool:
        res = optimize.run(req, mapper=pool)
    assert res.n_evals >= 10
    assert sum(1 for h in res.history if "objective" in h) >= 10
