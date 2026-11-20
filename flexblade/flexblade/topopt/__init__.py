"""3D multi-material topology optimisation of the hub-to-blade flexure (GPU, torch).

Fallback when the parametric FACT flexure cannot meet the targets: density-based
two-field SIMP (void / soft phase / stiff phase, each anisotropic with the print
orientation) on a structured hex voxel grid of the hub-to-blade-root region, solved
matrix-free with multigrid-preconditioned CG on the GPU, MMA with adjoint sensitivities,
initialised from the voxelised FACT layout.

    from flexblade.topopt import TopoptOptions, optimise
    out = optimise(design, "out/dir", TopoptOptions(n_elem=100_000))

Modules: grid (domain, voxelisation, hex element), fem (matrix-free operator, MG-PCG,
adjoint linear solve), filters, mma, problem (responses), run (driver), export (iso-
surfaces, STL/STEP, renders), validate (independent P2-tet re-analysis via fea.py).
Requires PyTorch (CUDA optional; CPU works for small grids).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ..schema import Design, TopoptFlexure
from .problem import Problem, TopoptOptions, layout_metrics  # noqa: F401
from .run import TopoptResult, run  # noqa: F401


def topopt_design(design: Design, res: TopoptResult, source: str = "") -> Design:
    """Copy of `design` whose flexure is the topopt result (topology "topopt": 6×6 spring +
    lumped mass in rotor.py). FACT members are kept for drawing."""
    from .. import fact
    d = design.model_copy(deep=True)
    members = fact.generate_members(design)
    d.flexure.members = members
    d.flexure.tension_member = None
    d.flexure.topology = "topopt"
    d.flexure.topopt = TopoptFlexure(
        K6=[[float(v) for v in row] for row in res.K6], mass=float(res.mass_props["mass"]),
        com=tuple(float(v) for v in res.mass_props["com"]),
        inertia=[[float(v) for v in row] for row in res.mass_props["inertia"]], source=source,
        notes=f"k_hinge {res.metrics['k_axis']:.4g} N m/rad, stiffness ratio "
              f"{res.metrics['stiffness_ratio']:.3g}, axis error {res.metrics['axis_error_deg']:.2f} deg")
    d.name = f"{design.name}_topopt"
    return d


def optimise(design: Design, out_dir: str | Path | None = None, opts: TopoptOptions | None = None,
             validate: bool = True, export: bool = True, evaluate: bool = False,
             callback=None) -> dict:
    """Run, validate, export. Writes into out_dir: result.json, fields.npz, design_topopt.json,
    <name>_soft/stiff.stl, <name>_topopt.step, render.png. Returns a summary dict."""
    from . import export as ex
    from .validate import compare, remesh_stiffness
    opts = opts or TopoptOptions()
    prob = Problem(design, opts)
    res = run(design, opts, callback=callback, problem=prob)
    summary = dict(res.summary())
    out = Path(out_dir) if out_dir is not None else None
    if validate:
        v = remesh_stiffness(prob, res.rho, res.phi)
        Kt = np.asarray(res.K6)
        val = dict(remesh=dict(K6=v["K6"].tolist(), n_dof=v["n_dof"], n_tets=v["n_tets"],
                               timings=v["timings"], solver=v["solver"],
                               metrics=layout_metrics(v["K6"], prob.dom.hinge_axis, prob.dom.Lc)),
                   rel_compliance_diff_voxel_vs_remesh=compare(Kt, v["K6"], prob.dom.hinge_axis))
        res.responses["validation"] = val
        summary["remesh_stiffness_ratio"] = val["remesh"]["metrics"]["stiffness_ratio"]
        summary["remesh_k_axis"] = val["remesh"]["metrics"]["k_axis"]
    dtop = topopt_design(design, res, str(out) if out else "")
    if evaluate:
        from ..rotor import evaluate as rev
        keys = ("control_ratio", "lag_freq_per_rev", "pitch_freq_per_rev", "flap_freq_per_rev",
                "pitch_lag_coupling", "tan_skew", "max_structural_utilisation")
        for tag, dd in (("evaluate", dtop), ("evaluate_fact", design)):
            try:
                r = rev(dd, ["trim", "modes", "harmonic", "checks"])
                summary[tag] = {k: r.summary.get(k) for k in keys}
            except Exception as e:  # noqa: BLE001
                summary[tag] = dict(error=str(e))
    if out is not None:
        out.mkdir(parents=True, exist_ok=True)
        if export:
            files = ex.export_result(res, prob.dom, out)
            files["render"] = ex.render(res, prob.dom, out / "render.png")
            summary["files"] = files
        np.savez_compressed(out / "fields.npz", rho=res.rho, phi=res.phi,
                            rho_gray=res.rho_gray, phi_gray=res.phi_gray,
                            origin=prob.dom.origin, h=prob.dom.h, hub=prob.dom.hub, plate=prob.dom.plate)
        (out / "result.json").write_text(json.dumps(res.to_json(), indent=1))
        (out / "design_topopt.json").write_text(json.dumps(dtop.model_dump(mode="json"), indent=1))
        (out / "summary.json").write_text(json.dumps(summary, indent=1, default=str))
    summary["design_topopt"] = dtop
    summary["result"] = res
    return summary
