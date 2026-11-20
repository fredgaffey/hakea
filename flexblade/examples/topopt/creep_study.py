"""Creep-constrained multi-material topology optimisation vs the FACT plate flexure.

Runs flexblade.topopt on the 5" TPU-rod winner and a 6" variant (2307 motor, 0.6 kg,
study/reinforce_grid.base_for(6) with the winner's flexure settings), then compares
hinge stiffness, constrained/hinge stiffness ratio, max creep/fatigue/interlayer
utilisation (voxel FEA, same solver for both) and the rotor 1/rev response (evaluate).

    PYTHONPATH=flexblade python flexblade/examples/topopt/creep_study.py [n_elem] [max_iter] [min_ratio]
Writes flexblade/examples/topopt/<name>/ (STL per material, STEP, renders, JSON) and
comparison.json / comparison.md here.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "study"))

from flexblade.schema import Design  # noqa: E402
from flexblade.topopt import TopoptOptions, optimise  # noqa: E402


def design_5in() -> Design:
    return Design.model_validate(json.loads((ROOT / "study/designs/5in_tpu_rod_winner.json").read_text()))


def design_6in() -> Design:
    import reinforce_grid as rg
    rg.MOTOR, rg.MASS = "tmotor_2307_1300kv", 0.6
    d = rg.base_for(6)
    w = design_5in()
    for k in ("topology", "skew_deg", "thickness", "width", "layup", "orientation"):
        setattr(d.flexure, k, getattr(w.flexure, k))
    d.name = "6in_tpu_rod_variant"
    return d


def render_viewer_parts(dtop: Design, path: Path, r_max: float = 0.03):
    """Matplotlib view of geometry.build_surface (the GUI's part list) of the topopt
    design: hub, fittings, blade roots and the per-material topopt flexure surfaces."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from flexblade.geometry import build_surface
    geo = build_surface(dtop)
    fig = plt.figure(figsize=(12, 6))
    for k, (el, az) in enumerate(((25, -60), (80, -90))):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        for p in geo["parts"]:
            V, F = np.asarray(p["vertices"]), np.asarray(p["faces"])
            if not len(F):
                continue
            P = V[F] * 1e3
            c = np.linalg.norm(P.mean(1)[:, :2], axis=1) < r_max * 1e3
            P = P[c]
            if not len(P):
                continue
            nrm = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
            nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-30)
            col = np.array(matplotlib.colors.to_rgb(p.get("color") or "#888888"))
            shade = 0.45 + 0.55 * np.clip(nrm @ np.array([0.4, -0.5, 0.75]) / 1.03, 0, 1)
            alpha = 1.0 if "topopt" in p["name"] else 0.35
            ax.add_collection3d(Poly3DCollection(P, facecolors=np.clip(col * shade[:, None], 0, 1),
                                                 edgecolors="none", alpha=alpha))
        L = r_max * 1e3
        ax.set_xlim(-L, L); ax.set_ylim(-L, L); ax.set_zlim(-L / 2, L / 2)
        ax.set_box_aspect((2, 2, 1))
        ax.view_init(elev=el, azim=az)
        ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
    fig.suptitle(f"{dtop.name}: GUI part list (topopt flexure opaque: "
                 + ", ".join(sorted({p['material'] for p in geo['parts'] if 'topopt' in p['name']})) + ")")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return str(path)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 150_000
    it = int(sys.argv[2]) if len(sys.argv) > 2 else 220
    min_ratio = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
    rows = []
    for d in (design_5in(), design_6in()):
        (HERE / f"{d.name}.json").write_text(json.dumps(d.model_dump(mode="json"), indent=1))
        torch.cuda.empty_cache()
        if torch.cuda.is_available():
            torch.cuda.reset_peak_memory_stats()
        soft = d.flexure.layup.material if d.flexure.layup.material.startswith("tpu") else "tpu_64d"
        o = TopoptOptions(n_elem=n, max_iter=it, soft_material=soft, envelope="box",
                          min_ratio=min_ratio or None)
        s = optimise(d, HERE / d.name, o, validate=False, evaluate=True)
        res = s["result"]
        s.setdefault("files", {})["render_viewer"] = render_viewer_parts(s["design_topopt"], HERE / d.name / "viewer_parts.png")
        m, fm, rp = res.metrics, res.fact_metrics, res.responses
        ev, evf = s.get("evaluate", {}), s.get("evaluate_fact", {})
        row = dict(design=d.name, grid=res.grid["shape"], n_elem=int(np.prod(res.grid["shape"])),
                   iterations=len(res.history), s_per_iter=res.timings["s_per_iter"],
                   gpu_mem_peak_MB=res.timings["gpu_mem_peak_MB"],
                   k_target=res.targets["k_hinge_target"],
                   fact=dict(k_axis=fm.get("k_axis"), k_soft=fm.get("k_soft"),
                             stiffness_ratio=fm.get("stiffness_ratio"),
                             constrained_ratio=fm.get("constrained_ratio"),
                             axis_error_deg=fm.get("axis_error_deg"),
                             creep=rp.get("seed_fact_loads_creep_max"),
                             fatigue=rp.get("seed_fact_loads_fatigue_max"),
                             interlayer=rp.get("seed_fact_loads_interlayer_max"),
                             strength=rp.get("seed_fact_loads_stress_max"),
                             control_ratio=evf.get("control_ratio"), lag_per_rev=evf.get("lag_freq_per_rev")),
                   topopt=dict(k_axis=m["k_axis"], k_soft=m["k_soft"], stiffness_ratio=m["stiffness_ratio"],
                               constrained_ratio=m["constrained_ratio"], axis_error_deg=m["axis_error_deg"],
                               creep=rp.get("creep_max"), fatigue=rp.get("fatigue_max"),
                               interlayer=rp.get("interlayer_max"), strength=rp.get("stress_max"),
                               mass_g=res.mass_props["mass"] * 1e3,
                               control_ratio=ev.get("control_ratio"), lag_per_rev=ev.get("lag_freq_per_rev")),
                   files=s.get("files"))
        rows.append(row)
        print(json.dumps(row, default=str), flush=True)
    (HERE / "comparison.json").write_text(json.dumps(rows, indent=1, default=str))
    lines = ["| design | variant | k_hinge (N m/rad) | k_softest | constrained/hinge | creep | fatigue | "
             "interlayer | control_ratio | lag /rev |", "|" + "---|" * 10]
    f = lambda v, p=3: "—" if v is None else f"{v:.{p}g}"  # noqa: E731
    for r in rows:
        for tag in ("fact", "topopt"):
            x = r[tag]
            lines.append(f"| {r['design']} | {tag} | {f(x['k_axis'])} | {f(x['k_soft'])} | "
                         f"{f(x['constrained_ratio'])} | {f(x['creep'])} | {f(x['fatigue'])} | "
                         f"{f(x['interlayer'])} | {f(x['control_ratio'])} | {f(x['lag_per_rev'])} |")
    (HERE / "comparison.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
