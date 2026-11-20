"""Pitching-airfoil validation sweep: generate, run in parallel (Mac/docker), compare with aero.

    from flexblade.cfd.validate import pitch_sweep
    rows = pitch_sweep(design, specs, out_dir="flexblade/examples/cfd/pitch_5in", where="mac")

Each spec is a dict of `pitching_airfoil_case` kwargs plus 'name'. Cases run one core each,
up to `max_parallel` at a time; only coefficient.dat + case json + log tails are kept.
"""
from __future__ import annotations

import json
import math
import shutil
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

from . import post
from .airfoil2d import pitching_airfoil_case
from .runner import run_case


def aero_reference(design, r: float, omega: float, rpm: float | None = None) -> dict:
    """Cl/α transfer from aero.linearized (quasi-steady and Theodorsen) at radius r."""
    from ..aero import AeroModel
    am = AeroModel(design)
    tr = am.trim(rpm=rpm) if rpm else am.trim()
    i = int(np.argmin(np.abs(tr.r - r)))
    out = dict(r_station=float(tr.r[i]), cl_alpha=float(tr.cl_a[i]))
    q = 0.5 * tr.rho * tr.W[i] ** 2 * tr.chord[i]
    for model in ("quasi_steady", "theodorsen"):
        A = am.linearized(tr, omega, model=model)[i]
        H = A[0, 2] / q / math.cos(tr.phi[i])          # dFz/dθ -> dCl/dα (lift ⟂ W)
        out[model] = dict(H_abs=float(abs(H)), phase_deg=float(np.degrees(np.angle(H))))
    return out


def compare_case(case_dir, design=None) -> dict:
    ci = json.loads(Path(case_dir, "flexblade_case.json").read_text())
    resp = post.pitching_response(case_dir, ci["alpha_amp_deg"], ci["omega"], n_periods=2)
    cl = resp["Cl"]
    th = post.theodorsen_pitch_cl(ci["k"], ci["pivot_chord_frac"])
    row = dict(case=Path(case_dir).name, r=ci["r"], k=ci["k"], Re=ci["Re"], mach=ci["mach"],
               alpha_mean_deg=ci["alpha_mean_deg"], alpha_amp_deg=ci["alpha_amp_deg"],
               turbulence=ci["turbulence"], cfd_cl_mean=cl["mean"], cfd_H_abs=cl["H_abs"],
               cfd_phase_deg=cl["phase_H_deg"], cfd_fit_rms=cl["residual_rms"],
               cfd_cl_2nd_harm=float(abs(cl["harmonics"][1])) if len(cl["harmonics"]) > 1 else None,
               thin_theodorsen_H_abs=float(abs(th)),
               thin_theodorsen_phase_deg=float(np.degrees(np.angle(th))))
    if design is not None:
        try:
            ref = aero_reference(design, ci["r"], ci["omega"])
            row.update(aero_cl_alpha=ref["cl_alpha"],
                       aero_theo_H_abs=ref["theodorsen"]["H_abs"],
                       aero_theo_phase_deg=ref["theodorsen"]["phase_deg"],
                       aero_qs_H_abs=ref["quasi_steady"]["H_abs"],
                       aero_qs_phase_deg=ref["quasi_steady"]["phase_deg"])
        except Exception as e:  # noqa: BLE001
            row["aero_error"] = repr(e)
    return row


def pitch_sweep(design, specs: list[dict], out_dir, where: str = "mac",
                max_parallel: int = 10, timeout: float = 4 * 3600,
                total_cores: int = 10) -> list[dict]:
    """Cases run concurrently; each is decomposed over total_cores // n_cases processes
    (scotch) so the machine stays loaded when there are fewer cases than cores."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix="fb_pitch_"))

    # mesh generation (gmsh) is not thread-safe: generate all cases first, then run
    np_each = max(1, total_cores // max(len(specs), 1))
    for spec in specs:
        sp = dict(spec)
        pitching_airfoil_case(design, work / sp.pop("name"), np_=np_each, **sp)

    def one(spec):
        name = spec["name"]
        cdir = work / name
        r = run_case(cdir, where=where, np=np_each, timeout=timeout)
        keep = out_dir / name
        keep.mkdir(parents=True, exist_ok=True)
        shutil.copy(cdir / "flexblade_case.json", keep)
        for f in (cdir / "postProcessing").rglob("coefficient.dat"):
            (keep / "postProcessing/forceCoeffs/0").mkdir(parents=True, exist_ok=True)
            shutil.copy(f, keep / "postProcessing/forceCoeffs/0/coefficient.dat")
        for lg in cdir.glob("log.*"):
            (keep / lg.name).write_text("\n".join(lg.read_text(errors="ignore").splitlines()[-40:]))
        row = dict(case=name, ok=r["ok"], seconds=r["seconds"])
        if r["ok"]:
            try:
                row.update(compare_case(keep, design))
            except Exception as e:  # noqa: BLE001
                row["post_error"] = repr(e)
        else:
            row["log_tail"] = r["log_tail"][-1500:]
        (keep / "result.json").write_text(json.dumps(row, indent=1, default=str))
        return row

    try:
        with ThreadPoolExecutor(max_parallel) as ex:
            rows = list(ex.map(one, specs))
    finally:
        shutil.rmtree(work, ignore_errors=True)
    (out_dir / "summary.json").write_text(json.dumps(rows, indent=1, default=str))
    return rows
