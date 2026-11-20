"""CAD-style drawing sheet: orthographic views + cross-section slices, coloured by material.

    python -m flexblade.drawing design.json out.png [--slices 6,8.5,11,14,0.75R]

Slices are planes x = const in hub frame H (blade 1 spans +x), so each slice shows the
y–z section (chord across, thickness up). Parts come from geometry.build_surface, so the
drawing shows exactly what is exported/printed.
"""
from __future__ import annotations

import sys

import numpy as np

from .geometry import build_surface, color_hex
from .schema import Design


def _meshes(design: Design):
    import trimesh
    out = []
    for p in build_surface(design)["parts"]:
        m = trimesh.Trimesh(np.asarray(p["vertices"]) * 1e3, np.asarray(p["faces"]), process=False)
        out.append((p, m))
    return out


def _proj(ax, meshes, axes, title):
    """Orthographic projection: draw every triangle's outline filled lightly per part."""
    from matplotlib.collections import PolyCollection
    i, j = axes
    for p, m in meshes:
        tri = m.vertices[m.faces][:, :, [i, j]]
        pc = PolyCollection(tri, facecolors=color_hex(p["material"], p.get("role", "")),
                            edgecolors="none", alpha=0.35 if p.get("role") not in ("rod",) else 0.9)
        ax.add_collection(pc)
    v = np.vstack([m.vertices for _, m in meshes])
    ax.set_xlim(v[:, i].min() - 2, v[:, i].max() + 2)
    ax.set_ylim(v[:, j].min() - 2, v[:, j].max() + 2)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.grid(alpha=0.3)


def _slice(ax, meshes, x_mm, title):
    from matplotlib.patches import Polygon
    drawn = []
    # draw matrix first, inserts on top
    order = sorted(meshes, key=lambda pm: pm[0].get("role") in ("rod", "core", "member_core"))
    for p, m in order:
        if not (m.bounds[0][0] <= x_mm <= m.bounds[1][0]):
            continue
        sec = m.section(plane_origin=[x_mm, 0, 0], plane_normal=[1, 0, 0])
        if sec is None:
            continue
        for ent in sec.entities:
            pts = sec.vertices[ent.points][:, [1, 2]]
            if len(pts) < 3:
                continue
            ax.add_patch(Polygon(pts, closed=True, fc=color_hex(p["material"], p.get("role", "")),
                                 ec="k", lw=0.4, alpha=0.95))
            drawn.append(pts)
    if drawn:
        v = np.vstack(drawn)
        pad = 1.0
        ax.set_xlim(v[:, 0].min() - pad, v[:, 0].max() + pad)
        ax.set_ylim(v[:, 1].min() - pad, v[:, 1].max() + pad)
        w, h = np.ptp(v[:, 0]), np.ptp(v[:, 1])
        ax.set_xlabel(f"y (chord) — {w:.1f} mm", fontsize=8)
        ax.set_ylabel(f"z — {h:.1f} mm", fontsize=8)
    else:
        ax.text(0.5, 0.5, "no material", ha="center", va="center", transform=ax.transAxes)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.3)


def sheet(design: Design, out: str, slices_mm: list[float] | None = None) -> str:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    meshes = _meshes(design)
    R = design.blade.radius * 1e3
    f = design.flexure
    if not slices_mm:
        hr, hub = f.hinge_radius * 1e3, f.hub_radius * 1e3
        slices_mm = [hub * 0.5, (hub + hr) / 2, hr, hr + 0.4 * f.length * 1e3,
                     design.blade.root_cutout_frac * R + 1.0, 0.75 * R]
    # blade 1 only for slices / views (blade 2 is mirrored)
    b1 = [(p, m) for p, m in meshes if p.get("blade") in (1, 0, None) or p.get("role") == "hub"]
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(3, 4, height_ratios=[1.0, 0.8, 1.0])
    _proj(fig.add_subplot(gs[0, :2]), meshes, (0, 1), "TOP view (x span → , y chord ↑), mm")
    _proj(fig.add_subplot(gs[0, 2:]), meshes, (0, 2), "SIDE view (x span → , z up ↑), mm")
    _proj(fig.add_subplot(gs[1, :2]), b1, (1, 2), "END view along span (y, z), blade 1 + hub, mm")
    axz = fig.add_subplot(gs[1, 2:])
    _proj(axz, [(p, m) for p, m in b1 if m.bounds[0][0] < (f.hinge_radius + f.length) * 1e3 + 4],
          (0, 2), "SIDE detail: hub + flexure (blade 1), mm")
    for k, x in enumerate(slices_mm[:8]):
        ax = fig.add_subplot(gs[2, k % 4]) if k < 4 else None
        if ax is None:
            break
        _slice(ax, b1, x, f"SECTION x = {x:.1f} mm ({x / R:.2f} R)")
    mats = {}
    for p, _ in meshes:
        mats.setdefault(p["material"], color_hex(p["material"], ""))
    fig.legend(handles=[Patch(fc=c, ec="k", label=m) for m, c in mats.items()],
               loc="upper right", fontsize=9, title="material")
    fig.suptitle(f"{design.name} — {design.blade.diameter_in:g}\" rotor, flexure "
                 f"{f.topology}, skew {f.skew_deg:g}°", fontsize=13)
    fig.tight_layout(rect=(0, 0, 0.93, 0.96))
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def main(argv=None):
    import argparse
    import json
    ap = argparse.ArgumentParser(prog="flexblade.drawing")
    ap.add_argument("design")
    ap.add_argument("out")
    ap.add_argument("--slices", help="comma list of x positions in mm")
    a = ap.parse_args(argv)
    d = Design.model_validate(json.load(open(a.design)))
    sl = [float(s) for s in a.slices.split(",")] if a.slices else None
    print(sheet(d, a.out, sl))


if __name__ == "__main__":
    main(sys.argv[1:])
