"""Cheapest off-the-shelf strip material for the 5" crossed-strip flexure (lead 5in_v8s_2307_m245_301:
9 x 0.05 x 4 mm 301 FH, L 10.05 mm). k-matched variants n*E*w*t^3 = K0 * Lf (EI/L kept), presets
re-solved (resolve_candidate(d, 6)), nominal screen at T0.7/1.0/1.25, J_hub 0.91e-6, Goodman
(the 2307_5_6in.md section 6 screen); the leaders then go through i34tools/sa.py.
usage: materials_5in.py screen [name ...] | table | save <name> <out.json>
Raw -> study/results/materials_5in.json"""
from __future__ import annotations
import json, os, sys, traceback
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust                                # noqa: E402
from flexblade.materials import get_material                     # noqa: E402
from flexblade.schema import Design                              # noqa: E402
import run_study as rs                                           # noqa: E402

LEAD = HERE / "designs" / "5in_v8s_2307_m245_301.json"
RES = HERE / "results" / "materials_5in.json"
TF = (0.7, 1.0, 1.25)
J = 0.91e-6
K0 = 9 * 193e9 * 4e-3 * (0.05e-3) ** 3          # lead n*E*w*t^3 (N m^2 * 12)
WORKERS = int(os.environ.get("MAT_WORKERS", 2))

# name: (material, n, t_m, w_m or None (k-matched), Lf, stock note)
V = {
    "lead301_9x4": ("steel_301_fh", 9, 50e-6, 4e-3, 1.0, "lead, slit 301 FH (made to order)"),
    "dental_7x5x050": ("steel_301_fh", 7, 50e-6, 5e-3, 1.0, "dental matrix band 5 mm x 0.05"),
    "dental_6x6x050": ("steel_301_fh", 6, 50e-6, 6e-3, 1.0, "dental matrix band 6 mm x 0.05"),
    "dental_16x5x038": ("steel_301_fh", 16, 38e-6, 5e-3, 1.0, "dental matrix band 5 mm x 0.038"),
    "dental_14x6x038": ("steel_301_fh", 14, 38e-6, 6e-3, 1.0, "dental matrix band 6 mm x 0.038"),
    "clock_2x075": ("steel_1095_shim", 2, 75e-6, None, 1.0, "clock suspension steel 0.075, hand-cut"),
    "clock_3x075_L15": ("steel_1095_shim", 3, 75e-6, None, 1.5, "clock suspension steel 0.075, hand-cut"),
    "smt_1x100": ("steel_301_fh", 1, 100e-6, None, 1.0, "SMT stencil 0.10 laser-cut"),
    "smt_1x100_L15": ("steel_301_fh", 1, 100e-6, None, 1.5, "SMT stencil 0.10 laser-cut"),
    "smt_2x100_L15": ("steel_301_fh", 2, 100e-6, None, 1.5, "SMT stencil 0.10 laser-cut"),
    "smt_1x120_L15": ("steel_301_fh", 1, 120e-6, None, 1.5, "SMT stencil 0.12 laser-cut"),
    "brass_2x075": ("brass_c260_h", 2, 75e-6, None, 1.0, "K&S brass shim 0.075"),
    "brass_3x075_L15": ("brass_c260_h", 3, 75e-6, None, 1.5, "K&S brass shim 0.075"),
    "pb_1x127_L15": ("phosbronze_c510", 1, 127e-6, None, 1.5, "model-railway PB strip 0.127"),
    "pb_2x100_L15": ("phosbronze_c510", 2, 100e-6, None, 1.5, "extra-spring PB strip 0.10"),
    "becu_2x100_L15": ("becu_c17200", 2, 100e-6, None, 1.5, "BeCu TH04 0.10"),
    "ti_2x100_L15": ("ti6al4v", 2, 100e-6, None, 1.5, "Ti-6Al-4V foil 0.10"),
    "pom_3x250_L15": ("pom_acetal", 3, 250e-6, None, 1.5, "Delrin sheet 0.25"),
    "pom_1x500_L15": ("pom_acetal", 1, 500e-6, None, 1.5, "Delrin sheet 0.5 (thickest feasible)"),
    "pet_3x190": ("pet_film", 3, 190e-6, None, 1.0, "Mylar 0.19"),
    "pet_2x250_L15": ("pet_film", 2, 250e-6, None, 1.5, "Mylar 0.25 (thickest feasible)"),
    "pp_1x500_L15": ("pp_sheet", 1, 500e-6, None, 1.5, "PP living-hinge 0.5"),
    "fr4_1x200_L15": ("fr4_g10", 1, 200e-6, None, 1.5, "PCB FR4 0.2, CNC outline"),
    "fr4_1x170_L15": ("fr4_g10", 1, 170e-6, "fr4_1x200_L15", 1.5, "PCB FR4 0.2 at -0.03 mm"),
    "fr4_1x230_L15": ("fr4_g10", 1, 230e-6, "fr4_1x200_L15", 1.5, "PCB FR4 0.2 at +0.03 mm"),
    "cfw_1x250x3_L15": ("cf_woven_sheet", 1, 250e-6, 3e-3, 1.5, "reed-petal CF 0.25, w 3 (k x2.2)"),
    # round 2: hub fit (geom:flexure_inside_hub) caps w at ~6 mm (L0) / ~5.1 mm (L x1.5)
    "dental_8x5x050": ("steel_301_fh", 8, 50e-6, 5e-3, 1.0, "dental matrix band 5 mm x 0.05"),
    "dental_6x5x050": ("steel_301_fh", 6, 50e-6, 5e-3, 1.0, "dental matrix band 5 mm x 0.05"),
    "dental_15x5x038": ("steel_301_fh", 15, 38e-6, 5e-3, 1.0, "dental matrix band 5 mm x 0.038"),
    "dental_17x5x038": ("steel_301_fh", 17, 38e-6, 5e-3, 1.0, "dental matrix band 5 mm x 0.038"),
    "dental_14x5x040": ("steel_301_fh", 14, 40e-6, 5e-3, 1.0, "dental matrix band 5 mm x 0.04"),
    "dental_12x6x040": ("steel_301_fh", 12, 40e-6, 6e-3, 1.0, "dental matrix band 6 mm x 0.04"),
    "clock_9x050": ("steel_1095_shim", 9, 50e-6, None, 1.0, "clock suspension steel 0.05, hand-cut"),
    "clock_3x075": ("steel_1095_shim", 3, 75e-6, None, 1.0, "clock suspension steel 0.075, hand-cut"),
    "ti_3x100_L15": ("ti6al4v", 3, 100e-6, None, 1.5, "Ti-6Al-4V foil 0.10"),
    "becu_2x100": ("becu_c17200", 2, 100e-6, None, 1.0, "BeCu TH04 0.10"),
    "pb_1x127": ("phosbronze_c510", 1, 127e-6, None, 1.0, "model-railway PB strip 0.127"),
    "pb_2x100": ("phosbronze_c510", 2, 100e-6, None, 1.0, "extra-spring PB strip 0.10"),
    "brass_4x075": ("brass_c260_h", 4, 75e-6, None, 1.0, "K&S brass shim 0.075"),
    "pet_5x190": ("pet_film", 5, 190e-6, None, 1.0, "Mylar 0.19"),
    "pom_3x250": ("pom_acetal", 3, 250e-6, None, 1.0, "Delrin sheet 0.25"),
    "pp_2x500_L15": ("pp_sheet", 2, 500e-6, None, 1.5, "PP living-hinge 0.5"),
    "fr4_1x200": ("fr4_g10", 1, 200e-6, None, 1.0, "PCB FR4 0.2, CNC outline"),
    "fr4_1x170": ("fr4_g10", 1, 170e-6, "fr4_1x200", 1.0, "PCB FR4 0.2 at -0.03 mm"),
    "fr4_1x230": ("fr4_g10", 1, 230e-6, "fr4_1x200", 1.0, "PCB FR4 0.2 at +0.03 mm"),
}


def width(name):
    mat, n, t, w, lf, _ = V[name]
    if isinstance(w, str):                       # same width as the referenced variant
        return width(w)
    return w if w is not None else K0 * lf / (n * get_material(mat).E1 * t ** 3)


def k_ratio(name):
    mat, n, t, _, lf, _ = V[name]
    return n * get_material(mat).E1 * width(name) * t ** 3 / (K0 * lf)


def build(name) -> Design:
    mat, n, t, _, lf, _ = V[name]
    d = Design.model_validate_json(LEAD.read_text())
    f = d.flexure
    lay = f.layup.model_copy(update=dict(material=mat))
    u = dict(layup=lay, thickness=t, width=round(width(name), 6), length=f.length * lf,
             strip_laminae=n, strip_lam_eta=f.strip_lam_eta if n > 1 else 0.0, strip_fatigue_mean="goodman")
    d = d.model_copy(update=dict(name=f"5in_{name}", flexure=f.model_copy(update=u)))
    return rs.resolve_candidate(d, 6)


def strip_mass(name):
    """4 strips (2 blades x 2 crossed), free span only, g."""
    mat, n, t, _, lf, _ = V[name]
    L = Design.model_validate_json(LEAD.read_text()).flexure.length * lf
    return 4 * n * t * width(name) * L * get_material(mat).rho * 1e3


def run(name):
    try:
        d = build(name)
        (HERE.parent / "scratch" / "mat5").mkdir(parents=True, exist_ok=True)
        (HERE.parent / "scratch" / "mat5" / f"{name}.json").write_text(d.model_dump_json(indent=1))
        pts = []
        for tag, dv in robust.variants(d, TF, None, [J]):
            r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
            m = rs.metrics(dv, r)
            top = sorted(((c["name"], round(c["utilisation"], 3)) for c in r["checks"]), key=lambda x: -x[1])[:4]
            pts.append(dict(tag=tag, cr_pl=m.get("cr_pl"), bind=m.get("pulse_binding"),
                            feas=m.get("feasible_pl"), hard=sorted((m.get("viol_hard") or {}).keys()), top=top,
                            pitch_pr=r["summary"].get("pitch_freq_per_rev")))
        return dict(name=name, pts=pts)
    except Exception as e:  # noqa: BLE001
        return dict(name=name, err=repr(e), tb=traceback.format_exc()[-800:])


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "screen"
    if mode == "save":
        d = Design.model_validate_json((HERE.parent / "scratch" / "mat5" / f"{sys.argv[2]}.json").read_text())
        Path(sys.argv[3]).write_text(d.model_dump_json(indent=1))
        return
    old = json.loads(RES.read_text()) if RES.exists() else {}
    if mode == "table":
        for n, r in old.items():
            if "err" in r:
                print(n, "ERR", r["err"]); continue
            mn = min(r["pts"], key=lambda p: p["cr_pl"] or 0)
            print(f"{n:18s} w {width(n)*1e3:5.2f} k {k_ratio(n):.2f} m {strip_mass(n):.3f} g  cr_pl "
                  + " / ".join(f"{p['cr_pl']:.3f}" for p in r["pts"])
                  + f"  feas {all(p['feas'] for p in r['pts'])} hard {sorted(set(sum((p['hard'] for p in r['pts']), [])))}"
                  + f"  min@{mn['tag']} bind {mn['bind']} top {mn['top'][:3]}")
        return
    names = sys.argv[2:] or list(V)
    with ProcessPoolExecutor(WORKERS) as ex:
        for r in ex.map(run, names, chunksize=1):
            old[r["name"]] = r
            RES.write_text(json.dumps(old, indent=1, default=float))
            print(r["name"], "ERR " + r["err"] if "err" in r else
                  [(p["tag"], round(p["cr_pl"] or 0, 3), p["bind"], p["hard"]) for p in r["pts"]], flush=True)


if __name__ == "__main__":
    main()
