"""Clamped steel-strip fittings (fittings.py): watertight parts, clearance, slot dims."""
import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("manifold3d")
pytest.importorskip("trimesh")

from flexblade.schema import Design  # noqa: E402
from flexblade import fittings as FT  # noqa: E402

V4 = Path(__file__).resolve().parents[1] / "study/designs/7in_tt_strap_steelx_v4.json"


@pytest.fixture(scope="module")
def design():
    return Design.model_validate(json.loads(V4.read_text()))


@pytest.fixture(scope="module")
def fits(design):
    return FT.build_strip_fittings(design)


def test_is_strip(design):
    assert FT.is_clamped_strip(design)


def test_slot_and_land_dims(design, fits):
    p = fits.params
    f = design.flexure
    assert p.slot_t == pytest.approx(f.thickness * 1e3 + 0.05)
    assert p.slot_w == pytest.approx(f.width * 1e3 + 0.2)
    assert p.land >= max(4 * np.sqrt(f.width * 1e3), 5.0) - 1e-9
    assert p.jaw_r == pytest.approx(10 * f.thickness * 1e3)
    assert p.L == pytest.approx(f.length * 1e3, rel=0.01)       # free span = modelled length


def test_slot_holds_strip(design, fits):
    """A strip-sized box over the land sits in the slot (no fitting material) in both fittings,
    and the free span is clear of both fittings."""
    strips, _, _ = FT.strip_members(design)
    p = fits.params
    for m in strips:
        p0, p1, L, t, w, n = FT._frame(m)
        R = np.c_[t, w, n]
        land_hub = FT._box([-p.land + 0.05, -p.w / 2 + 0.6, -p.t / 2], [-0.05, p.w / 2 - 0.6, p.t / 2], R, p0)
        land_bl = FT._box([L + 0.05, -p.w / 2 + 0.6, -p.t / 2], [L + p.land - 0.05, p.w / 2 - 0.6, p.t / 2], R, p0)
        free = FT._box([0.05, -p.w / 2, -p.t / 2], [L - 0.05, p.w / 2, p.t / 2], R, p0)
        for fit in (fits.hub, fits.blade):
            for box in (land_hub, land_bl, free):
                assert (box ^ fit).volume() < 1e-6
        # the jaws do surround the land (clamp): material within a wall of the slot faces
        probe = FT._box([-p.land / 2 - 1, -1, p.slot_t / 2 + 0.1], [-p.land / 2 + 1, 1, p.slot_t / 2 + 1.0], R, p0)
        assert (probe ^ fits.hub).volume() > 0.5 * probe.volume()


def test_parts_watertight(design):
    parts = FT.strip_print_parts(design)
    parts.pop("_fits")
    assert set(parts) == {"hub", "blade1", "blade2"}
    for k, m in parts.items():
        tm, kept, dropped = FT.to_trimesh(m)
        assert tm.is_watertight, k
        assert kept == 1 and dropped == 0, k


def test_clearance_positive(design, fits):
    r = FT.fitting_clearance(design, 5.0, 8.0, n=3, fits=fits)
    assert r["min_gap_mm"] > 0.3
    c = FT.clearance_check(design)
    assert c.name == "fitting_clearance" and c.utilisation < 1.0


def test_export_print_parts_strip(design, tmp_path):
    from flexblade.geometry import export_print_parts
    man = export_print_parts(design, tmp_path, blades="all")
    assert set(man["parts"]) == {"hub", "blade1", "blade2"}
    assert all(p["watertight"] for p in man["parts"].values())


def _v8():
    p = Path(__file__).resolve().parents[1] / "study/designs/10in_tt_strap_steelx_v8.json"
    return Design.model_validate(json.loads(p.read_text()))


@pytest.mark.parametrize("blade", [1, 2])
def test_slot_axis_matches_beam_preset_tangent(blade):
    """Slot cutter axis == beam model's strip end tangent with presets applied (< 0.2 deg)."""
    from flexblade import rotor as R
    from flexblade.beam import slot_frames
    d = _v8()
    fits = FT.build_strip_fittings(d, blade)
    assert len(fits.slot_axes) == 4
    m = R.build_model(d)
    a = 2 * np.pi * (blade - 1) / d.blade.n_blades
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1.0]])
    T = np.eye(3) if blade == 1 else np.diag([1.0, 1.0, -1.0])
    worst = 0.0
    for i, end, ax, _dw in fits.slot_axes:
        ids = [k for k, e in enumerate(m.fe.elements) if e.tag == f"b{blade}.flex{i}"]
        e = m.fe.elements[ids[0] if end == 0 else ids[-1]]
        Qa, Qb = slot_frames(e)
        t = T @ Rz.T @ ((Qa if end == 0 else Qb) @ e.E0)[:, 0]
        tgt = -t if end == 0 else t
        worst = max(worst, np.degrees(np.arccos(np.clip(ax @ tgt, -1, 1))))
    assert worst < 0.2, worst
    r = FT.fitting_clearance(d, 5.0, 8.0, n=3, fits=fits)
    assert r["min_gap_mm"] > 0.3


def test_dowel_holes_clear_of_other_slots_and_channels():
    d = _v8()
    fits = FT.build_strip_fittings(d)
    for end, cuts in ((0, fits.hub_cuts), (1, fits.blade_cuts)):
        dws = [a[3] for a in fits.slot_axes if a[1] == end]
        assert len(dws) == 2
        others = [c for c in cuts if all(c is not x for x in dws)]
        slots = others[0:4:2]                     # per-strip slot boxes (cuts: slot, flare, slot, flare)
        chans = others[4:]
        for k, dw in enumerate(dws):
            assert dw.volume() > 10
            for j, s in enumerate(slots):
                if j != k:
                    assert (dw ^ s).volume() < 1e-3, (end, k, j)
            for c in chans:
                assert (dw ^ c).volume() < 1e-3, (end, k)
