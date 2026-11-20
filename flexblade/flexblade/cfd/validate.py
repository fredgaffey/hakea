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


# --------------------------------------------------------------------------- 3D hover + batch

def bem_hover(design, rpm: float | None = None) -> dict:
    from ..aero import AeroModel
    am = AeroModel(design.model_copy(update=dict(layout="single")))
    tr = am.trim(rpm=rpm) if rpm else am.trim()
    T, Q, om = float(tr.T), float(tr.Q), float(tr.rpm) * 2 * math.pi / 60
    R = design.blade.radius
    rho = design.op.rho
    A = math.pi * R * R
    P_ideal = T ** 1.5 / math.sqrt(2 * rho * A)
    return dict(rpm=float(tr.rpm), thrust=T, torque=Q, FoM=P_ideal / (Q * om))


def hover_case_run(design, name: str, out_dir, where: str = "mac", np_: int = 10,
                   rpm: float | None = None, timeout: float = 4 * 3600, n_last: int = 200,
                   **case_kw) -> dict:
    """One 3D hover sector case: generate, run, extract thrust/torque/FoM vs BEM."""
    from .post import hover_forces
    from .rotor3d import rotor_sector_case
    out_dir = Path(out_dir)
    keep = out_dir / name
    work = Path(tempfile.mkdtemp(prefix="fb_hover_")) / name
    try:
        info = rotor_sector_case(design, work, rpm=rpm, np_=np_, **case_kw)
        bem = bem_hover(design, info.rpm)
        r = run_case(work, where=where, np=np_, timeout=timeout)
        keep.mkdir(parents=True, exist_ok=True)
        shutil.copy(work / "flexblade_case.json", keep)
        for f in (work / "postProcessing").rglob("*.dat"):
            dst = keep / f.relative_to(work)
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(f, dst)
        for lg in work.glob("log.*"):
            (keep / lg.name).write_text("\n".join(lg.read_text(errors="ignore").splitlines()[-60:]))
        row = dict(case=name, ok=r["ok"], seconds=round(r["seconds"]), rpm=info.rpm,
                   cells_bg=info.n_background_cells, bem_T=bem["thrust"], bem_Q=bem["torque"],
                   bem_FoM=bem["FoM"])
        if r["ok"]:
            try:
                hf = hover_forces(work, info.n_blades, n_last=n_last)
                om = info.rpm * 2 * math.pi / 60
                R = info.R
                Pi = hf["thrust"] ** 1.5 / math.sqrt(2 * design.op.rho * math.pi * R * R)
                row.update(cfd_T=hf["thrust"], cfd_Q=hf["torque"], n_iter=hf["n_iter"],
                           cfd_T_std=hf["thrust_std"], cfd_Q_std=hf["torque_std"],
                           cfd_FoM=Pi / (hf["torque"] * om),
                           T_ratio=hf["thrust"] / bem["thrust"], Q_ratio=hf["torque"] / bem["torque"])
            except Exception as e:  # noqa: BLE001
                row["post_error"] = repr(e)
        else:
            row["log_tail"] = r["log_tail"][-1500:]
        (keep / "result.json").write_text(json.dumps(row, indent=1, default=str))
        return row
    finally:
        shutil.rmtree(work.parent, ignore_errors=True)


def blade_shapes(designs: list, where: str = "mac", out_dir="flexblade/examples/cfd/blade_shapes",
                 pitching: bool = False, np_: int = 10, hover_kw: dict | None = None,
                 timeout: float = 4 * 3600) -> list[dict]:
    """Sequentially validate blade shapes (Design objects, dicts, or json paths) by CFD.

    Per design: 3D hover sector (thrust/torque/FoM vs aero BEM trim at same rpm) and, if
    `pitching`, the 1/rev 0.75R 2D pitching case (|H|, phase vs Theodorsen). One heavy case
    at a time (each uses `np_` cores). Returns a flat list of rows (one per case) and writes
    summary.json + table.md into out_dir. Takes tens of minutes to hours per design.
    """
    from ..schema import Design
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for i, d in enumerate(designs):
        if isinstance(d, (str, Path)):
            d = Design.model_validate_json(Path(d).read_text())
        elif isinstance(d, dict):
            d = Design.model_validate(d)
        tag = "".join(c if c.isalnum() or c in "-_" else "_" for c in (d.name or f"d{i}"))
        try:
            row = hover_case_run(d, f"{tag}_hover", out_dir, where, np_, timeout=timeout,
                                 **(hover_kw or {}))
            row["design"] = d.name
            row["kind"] = "hover"
        except Exception as e:  # noqa: BLE001
            row = dict(design=d.name, kind="hover", ok=False, error=repr(e))
        rows.append(row)
        if pitching:
            try:
                prow = pitch_sweep(d, [dict(name=f"{tag}_pitch75", r_frac=0.75)],
                                   out_dir, where=where, total_cores=np_, timeout=timeout)[0]
                prow["design"] = d.name
                prow["kind"] = "pitch"
            except Exception as e:  # noqa: BLE001
                prow = dict(design=d.name, kind="pitch", ok=False, error=repr(e))
            rows.append(prow)
        (out_dir / "summary.json").write_text(json.dumps(rows, indent=1, default=str))
    (out_dir / "table.md").write_text(blade_table(rows))
    return rows


def blade_table(rows: list[dict]) -> str:
    out = ["| design | case | ok | s | T cfd/BEM | Q cfd/BEM | FoM cfd (BEM) | H cfd (Theo) | phase cfd (Theo) |",
           "|---|---|---|---|---|---|---|---|---|"]

    def g(r, k, f="{:.3g}"):
        return f.format(r[k]) if r.get(k) is not None else "-"
    for r in rows:
        if r.get("kind") == "hover":
            out.append(f"| {r.get('design')} | hover | {r.get('ok')} | {r.get('seconds', '-')} | "
                       f"{g(r, 'T_ratio')} | {g(r, 'Q_ratio')} | {g(r, 'cfd_FoM')} ({g(r, 'bem_FoM')}) | - | - |")
        else:
            out.append(f"| {r.get('design')} | pitch 0.75R | {r.get('ok')} | {r.get('seconds', '-')} | - | - | - | "
                       f"{g(r, 'cfd_H_abs')} ({g(r, 'aero_theo_H_abs')}) | "
                       f"{g(r, 'cfd_phase_deg', '{:.1f}')} ({g(r, 'aero_theo_phase_deg', '{:.1f}')}) |")
    return "\n".join(out) + "\n"
