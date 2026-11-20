"""Figures + tables for the design study (reads study/results, writes study/figures).

    PYTHONPATH=<worktree>/flexblade ~/venv-flexblade/bin/python study/analyze.py [--campbell CASE ...]
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
RES, FIG, DES = HERE / "results", HERE / "figures", HERE / "designs"

CONCEPTS = ["tpu_monolithic", "tpu_rod", "tpu_cf_skin", "cf_flexure", "cf_flexure_sym", "tpu_split",
            "tpu95_split"]
LABEL = {"tpu_monolithic": "TPU monolithic", "tpu_rod": "TPU + CF rod inserts", "tpu_cf_skin": "TPU core + CF skin",
         "cf_flexure": "PA6-CF crossed plates", "cf_flexure_sym": "PA6-CF crossed plates, symmetric",
         "tpu_split": "TPU-64D plate + radial CF rod", "tpu95_split": "TPU-95A plate + radial CF rod"}
COLOR = {"tpu_monolithic": "#2a78d6", "tpu_rod": "#eb6834", "tpu_cf_skin": "#1baf7a",
         "cf_flexure": "#eda100", "cf_flexure_sym": "#e87ba4", "tpu_split": "#008300", "tpu95_split": "#6250d6"}
MARK = {"tpu_monolithic": "o", "tpu_rod": "s", "tpu_cf_skin": "^", "cf_flexure": "D", "cf_flexure_sym": "v",
        "tpu_split": "P", "tpu95_split": "X"}
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
SIZES = [4, 5, 7, 10, 12]
CHECKS = ["static_stress", "interlayer", "creep_strain", "fatigue_strain", "fatigue_stress",
          "esc_duty_slew", "esc_duty_peak", "motor_torque_1p"]

plt.rcParams.update({"font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "figure.facecolor": "#fcfcfb",
                     "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})


def load_cases() -> dict[str, dict]:
    out = {}
    p = RES / "cases.jsonl"
    for line in p.read_text().splitlines():
        if line.strip():
            r = json.loads(line)
            out[r["case"]] = r          # last wins (re-runs)
    return out


def load_ideal() -> list[dict]:
    p = RES / "ideal_hinge.jsonl"
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()] if p.exists() else []


def load_evals():
    p = RES / "evals.jsonl.gz"
    out = []
    try:
        with gzip.open(p, "rt") as fh:
            for line in fh:
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    break
    except EOFError:
        pass
    return out


def relaxed_best(evals, case, creep_factor=2.0):
    """Best control ratio among evaluated designs that meet every constraint except that
    the creep-strain allowable is multiplied by creep_factor (sensitivity, no re-optimisation)."""
    best = None
    for r in evals:
        if r.get("case") != case or "viol" not in r:
            continue
        v = dict(r["viol"])
        cu = r["util"].get("creep_strain", 0.0)
        v.pop("check:creep_strain", None)
        if v or cu > creep_factor:
            continue
        if best is None or r["control_ratio"] > best["control_ratio"]:
            best = r
    return best


def fig_control(cases, ideal, evals):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.0), constrained_layout=True)
    for ax, key, ylab in ((axes[0], "control_ratio", "control ratio  M / (W·R)"),
                          (axes[1], "tilt_equiv_deg", "equivalent thrust tilt (deg)")):
        for c in CONCEPTS:
            xs, ys, feas = [], [], []
            for D in SIZES:
                r = cases.get(f"{D}in_{c}")
                if not r or "error" in r:
                    continue
                xs.append(D)
                ys.append(max(r[key], 1e-5))
                feas.append(r["feasible"])
            if not xs:
                continue
            ax.plot(xs, ys, color=COLOR[c], lw=2, label=LABEL[c], zorder=2)
            for x, y, f in zip(xs, ys, feas):
                ax.plot(x, y, MARK[c], ms=8, mfc=COLOR[c] if f else "#fcfcfb", mec=COLOR[c], mew=2, zorder=3)
            # relaxed-creep sensitivity
            rx, ry = [], []
            for D in SIZES:
                b = relaxed_best(evals, f"{D}in_{c}")
                if b:
                    rx.append(D)
                    ry.append(max(b[key], 1e-5))
            if rx:
                ax.plot(rx, ry, color=COLOR[c], lw=1, ls=":", zorder=1)
        if ideal:
            ix = [r["diameter_in"] for r in ideal if r["elastic_axis_chord_frac"] == 0.45]
            iy = [r[key] for r in ideal if r["elastic_axis_chord_frac"] == 0.45]
            ax.plot(ix, iy, color=INK2, lw=1.5, ls="--", marker="x", ms=7, label="ideal skewed pin hinge")
            for x, y in zip(ix, iy):
                if x == 10:
                    ax.annotate("lag-sym+hub mode at 0.99/rev\n(resonant, not meaningful)", (x, y),
                                textcoords="offset points", xytext=(-150, -4), fontsize=7, color=INK2)
        ax.set_yscale("log")
        ax.set_xticks(SIZES)
        ax.set_xlabel("rotor diameter (in)")
        ax.set_ylabel(ylab)
    axes[0].legend(frameon=False, fontsize=7, loc="upper left", ncol=2)
    fig.suptitle("Control authority at the motor-capability 1/rev pulse (filled = all constraints met; "
                 "hollow = best infeasible; dotted = creep allowable ×2)", fontsize=9, color=INK)
    fig.savefig(FIG / "control_ratio_vs_size.png", dpi=150)
    plt.close(fig)


def fig_util(cases):
    rows = [f"{D}in_{c}" for c in CONCEPTS for D in SIZES if f"{D}in_{c}" in cases]
    rows += sorted(k for k in cases if "coax" in k)
    cols = CHECKS + ["pitch split /2°", "resonance (0.15/margin)"]
    M = np.full((len(rows), len(cols)), np.nan)
    for i, k in enumerate(rows):
        r = cases[k]
        if "util" not in r:
            continue
        for j, c in enumerate(CHECKS):
            M[i, j] = r["util"].get(c, np.nan)
        M[i, len(CHECKS)] = r.get("split_deg", np.nan) / 2.0
        mg = r.get("res_margin", np.nan)
        M[i, len(CHECKS) + 1] = 0.15 / mg if mg and mg > 0 else np.nan
    fig, ax = plt.subplots(figsize=(11, 0.32 * len(rows) + 1.8), constrained_layout=True)
    from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm
    cmap = LinearSegmentedColormap.from_list("u", ["#f3f7fd", "#9cc0ec", "#ffffff", "#f6b39a", "#c8312f"])
    norm = TwoSlopeNorm(vmin=0.0, vcenter=1.0, vmax=3.0)
    ax.imshow(np.clip(M, 0, 3), cmap=cmap, norm=norm, aspect="auto")
    for i in range(len(rows)):
        for j in range(len(cols)):
            v = M[i, j]
            if np.isfinite(v):
                ax.text(j, i, f"{v:.2f}" if v < 10 else f"{v:.0f}", ha="center", va="center", fontsize=7,
                        color=INK, fontweight="bold" if v > 1 else "normal")
    ax.set_xticks(range(len(cols)))
    ax.set_xticklabels(cols, rotation=30, ha="right")
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([k + ("" if cases[k].get("feasible") else "  ✗") for k in rows])
    ax.grid(False)
    ax.set_title("Best design per case: utilisation of each constraint (>1 fails, bold; ✗ = case infeasible)",
                 fontsize=9, color=INK)
    fig.savefig(FIG / "utilisation_breakdown.png", dpi=150)
    plt.close(fig)


def fig_campbell(case_names):
    from flexblade import api
    from flexblade.schema import Design
    n = len(case_names)
    if not n:
        return {}
    fig, axes = plt.subplots(1, n, figsize=(5 * n, 4), constrained_layout=True, squeeze=False)
    out = {}
    for ax, name in zip(axes[0], case_names):
        d = Design.model_validate_json((DES / f"{name}.json").read_text())
        if d.layout == "coaxial":
            d = d.model_copy(update=dict(layout="single"))
        r0 = api.evaluate(d, ["trim"])
        rpm0 = r0.trim.rpm
        grid = list(np.linspace(0.1 * rpm0, 1.3 * rpm0, 14))
        d = d.model_copy(update=dict(op=d.op.model_copy(update=dict(rpm_grid=grid))))
        r = api.evaluate(d, ["trim", "modes", "campbell"])
        cb = r.campbell
        F = np.array(cb.freqs_hz)
        rpm = np.array(cb.rpm)
        labs = cb.labels
        for k, ls in ((1, "-"), (2, "--"), (3, ":")):
            ax.plot(rpm, k * rpm / 60, color=INK2, lw=1, ls=ls)
            ax.fill_between(rpm, 0.85 * k * rpm / 60, 1.15 * k * rpm / 60, color=GRID, alpha=0.5, lw=0)
            ax.text(rpm[-1], k * rpm[-1] / 60, f" {k}P", color=INK2, va="center", fontsize=8)
        fam = {"lag": "#2a78d6", "flap": "#eb6834", "pitch": "#1baf7a"}
        for j in range(F.shape[1]):
            lab = labs[j] if j < len(labs) else ""
            col = next((v for kk, v in fam.items() if lab.startswith(kk)), "#6250d6")
            ax.plot(rpm, F[:, j], color=col, lw=2, marker="o", ms=3)
            if F[-1, j] < 3.6 * rpm[-1] / 60:
                ax.text(rpm[-1] * 1.01, F[-1, j], lab, fontsize=7, color=INK, va="center")
        ax.axvline(rpm0, color=INK, lw=1)
        ax.text(rpm0, ax.get_ylim()[1] * 0.97, " hover", fontsize=8, color=INK, va="top")
        ax.set_ylim(0, 3.6 * rpm[-1] / 60)
        ax.set_xlim(rpm[0], rpm[-1] * 1.22)
        ax.set_xlabel("rpm")
        ax.set_ylabel("rotating-frame frequency (Hz)")
        ax.set_title(name, fontsize=9, color=INK)
        out[name] = dict(rpm=rpm.tolist(), freqs_hz=F.tolist(), labels=labs, hover_rpm=rpm0)
    fig.suptitle("Campbell diagrams (lag blue, flap orange, pitch green, other violet; grey bands ±15 % "
                 "of 1P/2P/3P)", fontsize=9, color=INK)
    fig.savefig(FIG / "campbell_winners.png", dpi=150)
    plt.close(fig)
    (RES / "campbell_winners.json").write_text(json.dumps(out))
    return out


def table(cases, evals):
    hdr = ("| case | feasible | control ratio | tilt° | pitch amp° | lag/rev | rpm | max util (what) | "
           "split° | res. margin | slew util 255 / stock | creep×2 cr |")
    lines = [hdr, "|" + "---|" * 12]
    for c in CONCEPTS + ["coax"]:
        for k, r in cases.items():
            if (c == "coax") != ("coax" in k) or (c != "coax" and k.split("in_", 1)[1] != c) or "util" not in r:
                continue
            u = {kk: v for kk, v in r["util"].items() if not kk.startswith("member:")}
            wk = max(u, key=u.get)
            rb = relaxed_best(evals, k)
            lines.append(
                f"| {k} | {'yes' if r['feasible'] else 'NO'} | {r['control_ratio']:.4f} | "
                f"{r['tilt_equiv_deg']:.2f} | {r['pitch_amp_deg']:.2f} | {r['lag_per_rev']:.2f} | "
                f"{r['rpm']:.0f} | {u[wk]:.2f} ({wk}) | {r['split_deg']:.2f} | {r['res_margin']:.2f} | "
                f"{r['slew_util_ramp255']:.2f} / {r['slew_util_stock']:.2f} | "
                f"{rb['control_ratio']:.4f} |" if rb else
                f"| {k} | {'yes' if r['feasible'] else 'NO'} | {r['control_ratio']:.4f} | "
                f"{r['tilt_equiv_deg']:.2f} | {r['pitch_amp_deg']:.2f} | {r['lag_per_rev']:.2f} | "
                f"{r['rpm']:.0f} | {u[wk]:.2f} ({wk}) | {r['split_deg']:.2f} | {r['res_margin']:.2f} | "
                f"{r['slew_util_ramp255']:.2f} / {r['slew_util_stock']:.2f} | - |")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--campbell", nargs="*", default=[])
    a = ap.parse_args(argv)
    FIG.mkdir(exist_ok=True, parents=True)
    cases = load_cases()
    ideal = load_ideal()
    evals = load_evals()
    fig_control(cases, ideal, evals)
    fig_util(cases)
    if a.campbell:
        fig_campbell(a.campbell)
    t = table(cases, evals)
    (RES / "summary_table.md").write_text(t + "\n")
    print(t)


if __name__ == "__main__":
    main()
