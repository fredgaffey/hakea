"""5in_v8s_2307_m245_301 breakaway jig: jaw-exit spacing = design free length (0.05 mm), >=2 sprues/blade."""
import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("manifold3d")

from flexblade.schema import Design  # noqa: E402
from flexblade import fittings as FT  # noqa: E402

ROOT = Path(__file__).resolve().parents[1] / "study/designs"


@pytest.fixture(scope="module")
def jig():
    d = json.loads((ROOT / "5in_v8s_2307_m245_301.json").read_text())
    d["flexure"]["strip_slot_clear"] = 0.25          # as in study/build/build_5in_301.py
    D = Design.model_validate(d)
    return D, FT.strip_print_parts(D, breakaway_jig=True)


def test_jaw_exit_distance_equals_free_length(jig):
    D, parts = jig
    prm = FT.strip_params(D)
    strips, _, _ = FT.strip_members(D)
    fits, rotor = parts["_fits"], parts["rotor"]
    assert sum(c.volume() > 1.0 for c in rotor.decompose()) == 1      # one body (zero-volume slivers are dropped on export)
    H, Rj = prm.slot_t / 2, prm.jaw_r
    found = {}
    for (i, end, d, dw) in fits.slot_axes:
        p0, p1, L, t, w, n = FT._frame(strips[i])
        o = p0 if end == 0 else p1
        nd = n - d * (n @ d); nd /= np.linalg.norm(nd)
        for sg in (1, -1):
            c = o - 3 * d + nd * sg * (H + Rj + 0.6)
            R = np.c_[d, np.cross(d, nd), nd]
            line = FT._box([0, -0.05, -0.05], [5, 0.05, 0.05], R, c)
            for pc in (line ^ rotor).decompose():
                s0 = float(((np.asarray(pc.to_mesh().vert_properties)[:, :3] - o) @ d).min())
                if abs(s0) < 0.05:
                    found.setdefault((i, end), []).append(s0)
    assert set(found) == {(i, e) for i in range(len(strips)) for e in (0, 1)}, found
    assert all(abs(x) < 0.05 for v in found.values() for x in v)
    for m in strips:
        p0, p1, L, *_ = FT._frame(m)
        assert abs(L - prm.L) < 1e-9 and abs(np.linalg.norm(p1 - p0) - prm.L) < 1e-6
    assert abs(prm.L - D.flexure.length * 1e3) < 0.05


def test_sprues_and_slot(jig):
    D, parts = jig
    assert all(2 <= len(i) <= 4 for i in parts["_bridge_info"])
    prm = FT.strip_params(D)
    assert abs(prm.slot_t - (9 * 0.05 + 0.25)) < 1e-6 and abs(prm.slot_w - 4.2) < 1e-6
