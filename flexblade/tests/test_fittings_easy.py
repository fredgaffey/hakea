"""Easy build: breakaway jig. The print holds the root fitting at its assembled pose, so the
jaw-exit spacing is the design free length; slots are over-deep (no stops)."""
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
    D = Design.model_validate(json.loads((ROOT / "10in_easy_steelx.json").read_text()))
    return D, FT.strip_print_parts(D, breakaway_jig=True)


def test_jaw_exit_distance_equals_free_length(jig):
    """In the jig body every jaw exit face (any preset tilt) lies on the plane through its member
    end point p0 / p1 (normal = slot axis) within 0.05 mm, so the jaw-exit spacing along the strip
    is the design free length; the exit points themselves are |p1 - p0| = L apart by construction."""
    D, parts = jig
    prm = FT.strip_params(D)
    strips, _, _ = FT.strip_members(D)
    fits, rotor = parts["_fits"], parts["rotor"]
    assert len(rotor.decompose()) == 1                 # one printed body
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


def test_bridges(jig):
    D, parts = jig
    info = parts["_bridge_info"]
    assert all(2 <= len(i) <= 4 for i in info)
    for i in info:
        assert all(1.6 < g < 20 for _, _, g in i)
    assert abs(FT.strip_params(D).slot_t - (8 * 0.075 + 0.25)) < 1e-6
    assert parts["_fits"].params.blade_extra >= 3.0


def test_jig_default_off():
    D = Design.model_validate(json.loads((ROOT / "10in_tt_strap_steelx_v8.json").read_text()))
    assert FT.strip_params(D).blade_extra == 0.3 and not FT.strip_params(D).stop
