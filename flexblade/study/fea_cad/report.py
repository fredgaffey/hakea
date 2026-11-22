"""Tables + figure from fea_cad/results/{m05,m036}_summary.json and beam_ref.json (laptop, light).
Prints markdown tables (pasted into study/results/fea_cad_5in.md) and writes results/strip_glue_profile.png."""
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
R = HERE / "results"
B = json.loads((HERE / "beam_ref.json").read_text())
S = json.loads((R / "m05_summary.json").read_text())
F = json.loads((R / "m036_summary.json").read_text()) if (R / "m036_summary.json").exists() else {}
LAY = json.loads((HERE / "layout.json").read_text())
bc = B["as_cad"]["flex"]
kh_b = B["as_cad"]["k_frameH_compliance_unloaded"]["b1"]
print("## Case 1 hinge stiffness (N m/rad, pure moment, other freedoms free)\n")
print("| model | k_hinge (pitch axis a) | k_cross (s) | k_flap (y) |\n|---|---|---|---|")
print(f"| beam as-CAD, strips only (flexure_only_model) | {bc['strips_only']['k_hinge']:.4f} | {bc['strips_only']['k_cross']:.1f} | {bc['strips_only']['k_flap']:.1f} |")
print(f"| beam as-CAD, strips + strap | {bc['with_strap']['k_hinge']:.4f} | {bc['with_strap']['k_cross']:.1f} | {bc['with_strap']['k_flap']:.1f} |")
print(f"| beam as-CAD, rotor.flexure_stiffness frame H (bench-plan definition) | {kh_b[0]:.4f} | {kh_b[1]:.1f} | {kh_b[2]:.1f} |")
print(f"| beam DESIGN (presets, axial release; bench plan k_pitch) | {B['design']['k_frameH_compliance_unloaded']['b1'][0]:.4f} | | |")
for src, lab in ((S, "h 0.5 mm, 40 x 8"), (F, "h 0.36 mm, 60 x 12")):
    for c, nm in (("c1_STIFF", "ccx strips, rigid-ish fittings (210 GPa)"), ("c1_PACF", "ccx strips, PA6-CF fittings"),
                  ("c1_PETG", "ccx strips, PETG fittings"), ("c1s_STIFF", "ccx + strand, 210 GPa fittings"),
                  ("c1s_PACF", "ccx + strand, PA6-CF fittings")):
        if c in src and "k_hinge" in src[c]:
            r = src[c]
            print(f"| {nm} [{lab}] | {r['k_hinge']:.4f} ({r['k_hinge'] / bc['strips_only']['k_hinge']:.2f}) | "
                  f"{r['k_cross']:.1f} ({r['k_cross'] / bc['strips_only']['k_cross']:.2f}) | {r['k_flap']:.1f} ({r['k_flap'] / bc['strips_only']['k_flap']:.2f}) |")
print("\n(ratios vs beam as-CAD strips only)\n")

beam = {s["tag"]: s for s in B["as_cad"]["strips"]}
beamd = {s["tag"]: s for s in B["design"]["strips"]}
for c in ("c2_PACF", "c2_PETG"):
    if c not in S or "strips" not in S[c]:
        continue
    r = S[c]
    print(f"## {c}: hover {B['design']['rpm']:.0f} rpm, CF + {B['design']['thrust_N']:.2f} N thrust, NLGEOM\n")
    print("| strip | ccx hub mouth | ccx blade mouth | ccx free-span max (s mm) | ccx membrane mid | beam as-CAD hub / blade | beam design hub / blade |\n|---|---|---|---|---|---|---|")
    for s in r["strips"]:
        b, d = beam[s["tag"]], beamd[s["tag"]]
        print(f"| {s['tag']} | {s['hub_mouth']:.0f} | {s['blade_mouth']:.0f} | {s['free_max']:.0f} ({s['s_at_free_max']:.1f}) | {s['membrane_mid']:.1f} | "
              f"{b['surf_hub']:.0f} / {b['surf_blade']:.0f} | {d['surf_hub']:.0f} / {d['surf_blade']:.0f} |")
    print("\nper-lamina surface stress, MPa (model / 9); 301 FH Xt 1280 MPa\n")
    print("| strip | glue hub tau max / mean (s at max) | glue blade tau max / mean (s at max) | peel max hub / blade |\n|---|---|---|---|")
    for g in r["glue"]:
        print(f"| {g['tag']} | {g['hub']['tau_max']:.2f} / {g['hub']['tau_mean']:.2f} ({g['hub']['s_tau_max']:.2f}) | "
              f"{g['blade']['tau_max']:.2f} / {g['blade']['tau_mean']:.2f} ({g['blade']['s_tau_max']:.2f}) | {g['hub']['peel_max']:.2f} / {g['blade']['peel_max']:.2f} |")
    print("\n| body | vM max (99.9 %ile) | at (x,y,z) mm, near | max principal (99.9 %ile) | SF on Xt (peak / 99.9 %ile) |\n|---|---|---|---|---|")
    Xt = 45.0 if "PETG" in c else 110.0
    for nm, f in r["fittings"].items():
        print(f"| {nm} | {f['vm_max']:.1f} ({f['vm_p999']:.1f}) | {f['vm_at']} {f['vm_near']} | {f['p1_max']:.1f} ({f['p1_p999']:.1f}) | "
              f"{Xt / max(f['p1_max'], 1e-9):.1f} / {Xt / max(f['p1_p999'], 1e-9):.1f} |")
    print(f"\nstrap stretch: {r['strap']}\n")
for c in ("c3_PACF", "c3s_PACF", "c3_PETG"):
    if c in S and isinstance(S[c], list):
        rev = B["design"]["rpm"] / 60
        print(f"## {c} modes\n\n| # | Hz | /rev (hover) | class |\n|---|---|---|---|")
        for i, m in enumerate(S[c], 1):
            print(f"| {i} | {m['f_hz']:.1f} | {m['f_hz'] / rev:.3f} | {m['label']} {m['parts']} |")
print("\nbeam as-CAD modes (/rev):", [(round(m["per_rev"], 3), m["label"]) for m in B["as_cad"]["modes"]])
print("beam design modes (/rev):", [(round(m["per_rev"], 3), m["label"]) for m in B["design"]["modes"]])

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
for c, ls in (("c2_PACF", "-"), ("c2_PETG", "--")):
    if c in S and "strips" in S[c]:
        for s in S[c]["strips"]:
            p = np.array(s["profile"])
            ax[0].plot(p[:, 0], p[:, 1], ls, label=f"{c[3:]} {s['tag']}")
ax[0].axvline(0, color="k", lw=0.5); ax[0].axvline(LAY["strips"][0]["L"], color="k", lw=0.5)
ax[0].set_xlabel("s along strip, mm (0 = hub mouth, 10.05 = blade mouth)"); ax[0].set_ylabel("per-lamina surface stress, MPa")
ax[0].set_title("hover: strip lamina stress (max over width)"); ax[0].legend(fontsize=6); ax[0].grid(alpha=0.3)
labs, vals = [], []
for c in ("c2_PACF", "c2_PETG"):
    if c in S and "fittings" in S[c]:
        for nm, f in S[c]["fittings"].items():
            labs.append(f"{c[3:]}\n{nm}"); vals.append((f["vm_p999"], f["vm_max"]))
if vals:
    v = np.array(vals)
    ax[1].bar(np.arange(len(v)) - 0.2, v[:, 0], 0.4, label="vM 99.9 %ile")
    ax[1].bar(np.arange(len(v)) + 0.2, v[:, 1], 0.4, label="vM peak (node)")
    ax[1].set_xticks(np.arange(len(v))); ax[1].set_xticklabels(labs, fontsize=7)
    ax[1].axhline(45, color="r", ls=":", label="PETG Xt 45"); ax[1].axhline(110, color="g", ls=":", label="PA6-CF Xt 110")
    ax[1].set_ylabel("MPa"); ax[1].legend(fontsize=7); ax[1].set_title("hover: fitting von Mises")
fig.tight_layout(); fig.savefig(R / "hover_strip_fitting.png", dpi=100)
