"""Rank real filaments / inserts with the simulator.

    PYTHONPATH=flexblade python flexblade/study/filament_ranking.py [--workers 28] [--quick]

Studies = (preset design, layup slot, candidate set). Each candidate is evaluated
  * "same":    geometry unchanged, material swapped;
  * "resized": geometry resized so every element keeps the preset's soft bending
               stiffness (resize.py) — the fair comparison;
with the 1/rev torque pulse at motor capability (motor.capability(motor, rpm_hover,
Q_hover).mod_frac_max of the preset, used as op.torque_mod_frac for every candidate).

Sensitivity: the top candidates of each study/mode are re-evaluated with the material
replaced by "<name>@E=0.8 / 1.2" and "@S=0.8 / 1.2" (±20 % moduli / strengths) on the
geometry designed with the nominal datasheet values (i.e. built-with-real-material
scatter, not re-designed).

Outputs (flexblade/study/results/): filament_ranking.jsonl (all records),
filament_ranking.csv, sensitivity.csv, and flexblade/study/FILAMENTS.md.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from flexblade import api  # noqa: E402
from flexblade import filament_db as fdb  # noqa: E402
from flexblade.materials import GENERIC_NAMES, META, get_material, material_class, select_materials  # noqa: E402
from flexblade.matsweep import (_eval_job, _prepare_job, capability_mod_frac, run_jobs,  # noqa: E402
                                set_material, slot_materials, to_csv, with_torque_mod)
from flexblade.schema import Design  # noqa: E402

OUT = HERE / "results"

TPU = "class:tpu+tpe"
RIGID = "rigid"
STUDIES = [
    # id, preset, slot, selector, title
    ("tpu_mono_5", "5in_tpu_monolithic", "body", TPU, "5\" monolithic TPU blade — TPU grade"),
    ("tpu_mono_7", "7in_tpu_monolithic", "body", TPU, "7\" monolithic TPU blade — TPU grade"),
    ("skin_core_5", "5in_tpu_cf_skin", "body", TPU, "5\" TPU core + CF skin — TPU core grade"),
    ("skin_5", "5in_tpu_cf_skin", "skin", RIGID, "5\" TPU core + CF skin — skin filament"),
    ("rod_core_7", "7in_tpu_rod", "body", TPU, "7\" TPU + rod — TPU grade"),
    ("rod_core_10", "10in_tpu_rod", "body", TPU, "10\" TPU + rod — TPU grade"),
    ("rod_7", "7in_tpu_rod", "insert", "inserts", "7\" TPU + rod — insert rod/wire"),
    ("rod_10", "10in_tpu_rod", "insert", "inserts", "10\" TPU + rod — insert rod/wire"),
    ("cf_flex_5", "5in_cf_flexure", "body", RIGID, "5\" all-filament crossed flexure — rigid filament"),
    ("cf_flex_10", "10in_cf_flexure", "body", RIGID, "10\" all-filament crossed flexure — rigid filament"),
    ("flex_only_5", "5in_cf_flexure", "flexure", "filaments",
     "5\" crossed flexure — flexure material only (blade stays PA6-CF)"),
    ("flex_only_10", "10in_cf_flexure", "flexure", "filaments",
     "10\" crossed flexure — flexure material only (blade stays PA6-CF)"),
]
N_SENS = 6


def candidates(selector: str, slot: str) -> list[str]:
    names = select_materials(selector)
    if slot != "insert":
        names = [n for n in names if material_class(n) not in fdb.INSERT_CLASSES]
    return names


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 4))
    ap.add_argument("--quick", action="store_true", help="3 materials per study (smoke test)")
    ap.add_argument("--report-only", action="store_true", help="rebuild FILAMENTS.md from results")
    a = ap.parse_args(argv)
    OUT.mkdir(exist_ok=True)
    if a.report_only:
        recs = [json.loads(l) for l in (OUT / "filament_ranking.jsonl").read_text().splitlines()]
        sens = [json.loads(l) for l in (OUT / "sensitivity.jsonl").read_text().splitlines()]
        write_report(recs, sens, json.loads((OUT / "baselines.json").read_text()))
        return
    t0 = time.time()
    P = api.presets()
    base: dict[str, dict] = {}
    for sid, pre, slot, sel, title in STUDIES:
        if pre not in base:
            d = P[pre]
            frac = capability_mod_frac(d)
            base[pre] = dict(design=with_torque_mod(d, frac).model_dump(mode="json"), mod_frac=frac,
                             materials={s: slot_materials(d, s) for s in ("body", "skin", "insert")})
            print(f"{pre}: capability torque_mod_frac = {frac:.3f}", flush=True)

    # ---------------------------------------------------------------- prepare designs
    jobs, keys = [], []
    for sid, pre, slot, sel, title in STUDIES:
        mats = candidates(sel, slot)
        cur = base[pre]["materials"][slot]
        mats = list(dict.fromkeys(cur + mats))       # baseline material first
        if a.quick:
            mats = mats[:3]
        for mode in ("same", "resized"):
            for m in mats:
                jobs.append((base[pre]["design"], slot, m, mode == "resized", 0.4e-3))
                keys.append((sid, mode, m))
    print(f"preparing {len(jobs)} designs …", flush=True)
    prep = run_jobs(_prepare_job, jobs, workers=a.workers)
    ev_jobs = [{"id": i, "design": p["design"]} for i, p in enumerate(prep) if "design" in p]
    print(f"evaluating {len(ev_jobs)} designs on {a.workers} workers …", flush=True)
    ev = run_jobs(_eval_job, ev_jobs, workers=a.workers,
                  progress=lambda i, n: (i % 50 == 0 or i == n) and print(f"  {i}/{n}  {time.time()-t0:.0f}s", flush=True))
    ev_by_id = {j["id"]: e for j, e in zip(ev_jobs, ev)}
    recs = []
    study = {s[0]: s for s in STUDIES}
    for i, ((sid, mode, m), p) in enumerate(zip(keys, prep)):
        _, pre, slot, sel, title = study[sid]
        meta = META.get(m, {})
        r = dict(study=sid, title=title, preset=pre, slot=slot, mode=mode, material=m,
                 brand=meta.get("brand", "generic"), product=meta.get("product", m),
                 cls=material_class(m), url=meta.get("url"), tds_url=meta.get("tds_url"),
                 usd_per_kg=meta.get("usd_per_kg"), confidence=meta.get("confidence", "low"),
                 E1_MPa=get_material(m).E1 / 1e6, Xt_MPa=get_material(m).Xt / 1e6,
                 rho=get_material(m).rho, baseline=m in base[pre]["materials"][slot])
        if "error" in p:
            r["error"] = p["error"]
        else:
            rep = p.get("resize")
            if rep:
                r["resize_feasible"] = rep["feasible"]
                r["resize_worst_ratio"] = rep["worst_ratio"]
                r["resize_below_min"] = rep["below_min_printable"]
                fl = [e for e in rep["elements"] if not e["element"].startswith("station")]
                st = [e for e in rep["elements"] if e["element"].startswith("station")]
                r["resize_flexure"] = fl[0]["new"] if fl else None
                r["resize_tc_root"] = st[0]["new"] if st and st[0]["param"] == "thickness_frac" else None
                r["resize_notes"] = sorted({e["note"] for e in rep["elements"] if e.get("note")})
            e = ev_by_id.get(i, {"error": "not evaluated"})
            if "error" in e:
                r["error"] = e["error"]
            else:
                r.update(e["metrics"])
                r["prepared_design"] = p["design"]
        recs.append(r)
    # ---------------------------------------------------------------- sensitivity
    sens_jobs, sens_keys = [], []
    for sid, pre, slot, sel, title in STUDIES:
        for mode in ("same", "resized"):
            rows = sorted([r for r in recs if r["study"] == sid and r["mode"] == mode and "fitness" in r],
                          key=lambda r: -r["fitness"])[:N_SENS]
            for r in rows:
                d = Design.model_validate(r["prepared_design"])
                for tag in ("E=0.8", "E=1.2", "S=0.8", "S=1.2"):
                    dv = set_material(d, slot, f"{r['material']}@{tag}")
                    sens_jobs.append({"id": len(sens_jobs), "design": dv.model_dump(mode="json")})
                    sens_keys.append((sid, mode, r["material"], tag))
    print(f"sensitivity: {len(sens_jobs)} evaluations …", flush=True)
    sev = run_jobs(_eval_job, sens_jobs, workers=a.workers)
    sens = []
    for (sid, mode, m, tag), e in zip(sens_keys, sev):
        s = dict(study=sid, mode=mode, material=m, variant=tag)
        if "error" in e:
            s["error"] = e["error"]
        else:
            s.update({k: e["metrics"].get(k) for k in ("control_ratio", "max_util", "governing", "fitness",
                                                         "fitness_nocreep", "lag_per_rev", "pitch_per_rev")})
        sens.append(s)
    for r in recs:
        r.pop("prepared_design", None)
    recost(recs)
    (OUT / "filament_ranking.jsonl").write_text("".join(json.dumps(r, default=str) + "\n" for r in recs))
    (OUT / "sensitivity.jsonl").write_text("".join(json.dumps(s, default=str) + "\n" for s in sens))
    cols = ("study", "mode", "material", "brand", "product", "cls", "control_ratio", "pitch_1p_deg",
            "max_util", "governing", "util_creep", "util_fatigue", "util_interlayer", "util_static",
            "lag_per_rev", "pitch_per_rev", "lag_margin", "pitch_margin", "pitch_split_deg",
            "rotor_mass_g", "filament_cost_usd", "fitness", "fitness_nocreep", "max_util_nocreep",
            "feasible", "resize_feasible",
            "usd_per_kg", "confidence", "url", "error")
    (OUT / "filament_ranking.csv").write_text(to_csv(recs, cols))
    (OUT / "sensitivity.csv").write_text(to_csv(sens, ("study", "mode", "material", "variant", "control_ratio",
                                                       "max_util", "governing", "fitness", "error")))
    bl = {pre: dict(mod_frac=b["mod_frac"], materials=b["materials"]) for pre, b in base.items()}
    (OUT / "baselines.json").write_text(json.dumps(bl, indent=1))
    write_report(recs, sens, bl)
    print(f"done in {time.time()-t0:.0f}s", flush=True)


# --------------------------------------------------------------------------- report

def _f(x, n=3):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "–"
    if isinstance(x, bool):
        return "yes" if x else "no"
    if isinstance(x, (int, float)):
        if x == 0:
            return "0"
        return f"{x:.{n}g}" if abs(x) < 1e4 else f"{x:.3g}"
    return str(x)


def _name(r):
    n = r["material"] if r["brand"] == "generic" else f"{r['brand']} {r['product']}"
    return f"[{n}]({r['url']})" if r.get("url") else f"{n} *(generic)*"


def robust_fitness(sens, sid, mode, m):
    v = [s["fitness"] for s in sens if s["study"] == sid and s["mode"] == mode and s["material"] == m
         and s.get("fitness") is not None]
    return (min(v), max(v)) if v else (None, None)


def recost(recs):
    """Filament cost from the per-material mass and the *current* database prices."""
    for r in recs:
        mm = r.get("mass_by_material_g")
        if not mm:
            continue
        cost, unpriced = 0.0, []
        for mat, g in mm.items():
            p = (META.get(mat.split("@")[0]) or {}).get("usd_per_kg")
            if p is None:
                unpriced.append(mat)
            else:
                cost += g / 1e3 * p
        r["filament_cost_usd"] = cost if not unpriced or cost else None
        r["unpriced"] = unpriced


def write_report(recs, sens, baselines, top=12):
    recost(recs)
    L = []
    A = L.append
    A("# Filament database and simulator ranking\n")
    A("*Generated by `flexblade/study/filament_ranking.py` from `flexblade/flexblade/data/filaments.json`"
      " — rerun `--report-only` after editing the hand-written sections in the script.*\n")
    A(RECOMMENDATIONS)
    A("\n## Method\n")
    A(METHOD)
    A("\nBaseline motor 1/rev torque capability used as `op.torque_mod_frac` (from the preset's hover trim):\n")
    A("| preset | torque_mod_frac | slot materials |\n|---|---|---|")
    for pre, b in baselines.items():
        A(f"| {pre} | {b['mod_frac']:.3f} | " + ", ".join(f"{k}: {'/'.join(v)}" for k, v in b["materials"].items() if v) + " |")
    A(index_tables())
    studies = list(dict.fromkeys(r["study"] for r in recs))
    A("\n## Rankings per design\n")
    A("Columns: **fit** = fitness (higher better), **CR** = control_ratio (hub moment / (W·R)), "
      "**θ1P** = 1/rev blade pitch amplitude (deg), **U** = max structural utilisation and the governing "
      "check@member, **lag/pitch** = mode frequency in /rev (margin to nearest of 1P/2P), **split** = static "
      "blade-1 − blade-2 pitch (deg), **m** = blades+flexures mass (g, solid), **$** = filament cost per rotor "
      "(USD, per-kg priced materials only), **fit±20%** = min–max fitness over E±20 % / strength±20 %.\n")
    for sid in studies:
        rs = [r for r in recs if r["study"] == sid]
        A(f"\n### {rs[0]['title']} (`{rs[0]['preset']}`, slot `{rs[0]['slot']}`)\n")
        for mode in ("resized", "same"):
            rows = [r for r in rs if r["mode"] == mode]
            ok = sorted([r for r in rows if r.get("fitness") is not None], key=lambda r: -r["fitness"])
            bad = [r for r in rows if r.get("fitness") is None]
            base = next((r for r in rows if r.get("baseline")), None)
            A(f"\n**{'Resized to the preset section stiffness (fair)' if mode == 'resized' else 'Same geometry'}**"
              f" — {len(ok)} evaluated, {len(bad)} failed"
              + (f"; preset material `{base['material']}` fitness {_f(base.get('fitness'))}" if base else "") + "\n")
            A("| # | material | class | fit | fit±20% | fit¬creep (U¬creep) | CR | θ1P | U | governs | lag (m) | pitch (m) | split | m g | $ |"
              + (" resize |" if mode == "resized" else "") )
            A("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|" + ("---|" if mode == "resized" else ""))
            for i, r in enumerate(ok[:top]):
                lo, hi = robust_fitness(sens, sid, mode, r["material"])
                rz = ""
                if mode == "resized":
                    fl = r.get("resize_flexure")
                    tc = r.get("resize_tc_root")
                    parts = []
                    if isinstance(fl, dict):
                        parts.append(f"t {fl['thickness']*1e3:.2f} w {fl['width']*1e3:.1f} mm")
                    elif isinstance(fl, (int, float)):
                        parts.append(f"{fl*1e3:.2f} mm")
                    if tc:
                        parts.append(f"t/c {tc:.3f}")
                    if not r.get("resize_feasible", True):
                        parts.append("**bound**")
                    rz = " " + "; ".join(parts) + " |"
                A(f"| {i+1} | {_name(r)} | {r['cls']} | {_f(r['fitness'])} | {_f(lo, 2)}–{_f(hi, 2)} | {_f(r.get('fitness_nocreep'))} ({_f(r.get('max_util_nocreep'), 2)}) | "
                  f"{_f(r['control_ratio'])} | {_f(r.get('pitch_1p_deg'))} | {_f(r['max_util'])} | {r.get('governing')} | "
                  f"{_f(r.get('lag_per_rev'))} ({_f(r.get('lag_margin'), 2)}) | {_f(r.get('pitch_per_rev'))} ({_f(r.get('pitch_margin'), 2)}) | "
                  f"{_f(r.get('pitch_split_deg'))} | {_f(r.get('rotor_mass_g'))} | {_f(r.get('filament_cost_usd'), 2)} |" + rz)
            if bad:
                A(f"\n<details><summary>{len(bad)} not evaluated</summary>\n\n" + "\n".join(
                    f"- `{r['material']}`: {r.get('error')}" for r in bad) + "\n</details>\n")
    # class summary
    A("\n## Best per material class (resized mode, all studies)\n")
    A("| study | class | best product | fitness | CR | U | governs |\n|---|---|---|---|---|---|---|")
    for sid in studies:
        rs = [r for r in recs if r["study"] == sid and r["mode"] == "resized" and r.get("fitness") is not None]
        best = {}
        for r in rs:
            c = r["cls"]
            if c not in best or r["fitness"] > best[c]["fitness"]:
                best[c] = r
        for c, r in sorted(best.items(), key=lambda kv: -kv[1]["fitness"]):
            A(f"| {sid} | {c} | {_name(r)} | {_f(r['fitness'])} | {_f(r['control_ratio'])} | {_f(r['max_util'])} | {r['governing']} |")
    A("\n## Database\n")
    A(database_tables())
    (HERE / "FILAMENTS.md").write_text("\n".join(L) + "\n")
    print("wrote", HERE / "FILAMENTS.md")


def index_tables(top: int = 25) -> str:
    """Material indices (no simulation): the simulator's governing checks are flexure creep
    strain and fatigue strain, i.e. strain-limited — rank flexure materials by σ_f/E first."""
    from flexblade.materials import list_materials
    ms = [m for m in list_materials() if not m.get("variant_of")]
    L = ["\n## Material indices (no simulation)\n",
         "The governing checks in every study are **creep strain and fatigue strain in the flexure** "
         "(strain-limited, not strength-limited). For a flexure of given stiffness the usable rotation "
         "scales with the allowable strain, so flexure materials are ranked by the fatigue strain "
         "capability **ε_f = σ_f/E** (σ_f = fatigue strength at 1e7 cycles = fatigue_stress_ratio × Xt; "
         "elastomers: hardness table / literature) and the sustained (creep) strain allowable **ε_c**, "
         "then by ε_f per density. The blade wants specific stiffness E/ρ (frequency placement, low "
         "centrifugal load per stiffness); a tension member wants specific strength σ/ρ and low creep.\n",
         "Provenance: σ_f and ε_c are literature/class values unless a TDS gives fatigue data — see "
         "`derived.fatigue_stress_ratio` / `derived.creep_strain_allow` per entry.\n"]

    def nm(m):
        n = m["name"] if m.get("brand") in (None, "generic") else f"{m['brand']} {m['product']}"
        return f"[{n}]({m['url']})" if m.get("url") else f"{n} *(generic)*"

    def table(title, rows, cols):
        L.append(f"\n### {title}\n")
        L.append("| # | material | class | " + " | ".join(c[0] for c in cols) + " | conf |")
        L.append("|---|---|---|" + "---|" * len(cols) + "---|")
        for i, m in enumerate(rows):
            L.append(f"| {i+1} | {nm(m)} | {m.get('cls')} | " + " | ".join(c[1](m) for c in cols)
                     + f" | {m.get('confidence', 'low')} |")

    from flexblade.filament_db import INSERT_CLASSES
    flex = [m for m in ms if m.get("cls") not in INSERT_CLASSES]
    rig = [m for m in flex if not m["elastomer"]]
    el = [m for m in flex if m["elastomer"]]
    cols = [("ε_f = σ_f/E %", lambda m: f"{m['fatigue_strain']*100:.2f}"),
            ("ε_c creep %", lambda m: f"{m['creep_strain']*100:.2f}"),
            ("ε_f/ρ %·cm³/g", lambda m: f"{m['fatigue_strain_per_rho']*100:.2f}"),
            ("E MPa", lambda m: f"{m['E1']/1e6:.4g}"), ("σ_t MPa", lambda m: f"{m['Xt']/1e6:.3g}"),
            ("ρ", lambda m: f"{m['rho']:.0f}"), ("$/kg", lambda m: _f(m.get("usd_per_kg")))]
    key = lambda m: (-min(m["fatigue_strain"], 10), -m["creep_strain"], -m["fatigue_strain_per_rho"])  # noqa: E731
    table(f"Flexure — rigid filaments by ε_f, then ε_c (top {top})", sorted(rig, key=key)[:top], cols)
    table("Flexure — elastomers (TPU/TPE) by ε_f, then ε_c", sorted(el, key=key), cols)
    best = {}
    for m in sorted(rig + el, key=key):
        best.setdefault(m.get("cls"), m)
    table("Flexure — best product per class", list(best.values()), cols)
    cols_b = [("E/ρ MPa·cm³/g", lambda m: f"{m['specific_stiffness']:.4g}"),
              ("σ/ρ MPa·cm³/g", lambda m: f"{m['specific_strength']:.3g}"),
              ("ε_c creep %", lambda m: f"{m['creep_strain']*100:.2f}"),
              ("E_z/E", lambda m: f"{m['E3']/m['E1']:.2f}"),
              ("σ_z MPa", lambda m: f"{m['Zt']/1e6:.3g}"), ("$/kg", lambda m: _f(m.get("usd_per_kg")))]
    table(f"Blade (rigid, stiffness-driven) — by E/ρ (top {top})",
          sorted(rig, key=lambda m: -m["specific_stiffness"])[:top], cols_b)
    ins = [m for m in ms if m.get("cls") in INSERT_CLASSES]
    table("Tension member / insert — by σ/ρ",
          sorted(ins + sorted(rig, key=lambda m: -m["specific_strength"])[:8],
                 key=lambda m: -m["specific_strength"]), cols_b[:3] + cols[:1])
    return "\n".join(L)


def database_tables() -> str:
    ents = fdb.load_entries()
    L = []
    A = L.append
    n_real = len([e for e in ents if not e.get("variant_of")])
    A(f"{n_real} products ({len(ents) - n_real} conditioned/aged variants) from "
      f"{len({e['brand'] for e in ents})} brands. Values are datasheet (TDS) values as published; "
      "`derived` fields are listed in the JSON per field. E/σ = XY tensile modulus / strength "
      "(insert: rod/wire modulus), Z = interlayer (TDS value, or *italic* = derived by class ratio). "
      "Prices are list prices seen 2026-11 (native currency; USD/kg converted with fixed approximate FX).\n")
    groups: dict[str, list] = {}
    for e in ents:
        if e.get("variant_of"):
            continue
        groups.setdefault(fdb.CLASSES[e["class"]]["group"], []).append(e)
    for g, es in groups.items():
        A(f"\n### {g}\n")
        A("| product | class | ρ | E MPa | σ MPa | E_z MPa | σ_z MPa | elong % | Shore | price | conf | TDS | notes |")
        A("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
        for e in sorted(es, key=lambda e: (e["class"], e["brand"], e["product"])):
            t, s, d = e["tds"], e["sim"], e["derived"]
            ez = f"{s['E3']/1e6:.4g}" if d["E3"].startswith("tds:") else f"*{s['E3']/1e6:.4g}*"
            sz = f"{s['Zt']/1e6:.3g}" if d["Zt"].startswith("tds:") else f"*{s['Zt']/1e6:.3g}*"
            E = f"{s['E1']/1e6:.4g}" + ("" if d["E1"].startswith("tds:") else "†")
            p = e.get("price") or {}
            price = "–"
            if p.get("value"):
                price = f"{p['value']:g} {p.get('currency', '')}" + ("/kg" if p.get("per_kg", True) else "/m")
            tds = f"[TDS]({e['tds_url']})" if e.get("tds_url") else "–"
            note = (e.get("notes") or "").replace("|", "/").replace("\n", " ")
            if len(note) > 140:
                note = note[:137] + "…"
            A(f"| [{e['brand']} {e['product']}]({e['url']}) | {e['class']} | {s['rho']:.0f} | {E} | "
              f"{s['Xt']/1e6:.3g} | {ez} | {sz} | {_f(t.get('elongation_xy_pct'))} | {t.get('shore_hardness') or '–'} | "
              f"{price} | {e['confidence']} | {tds} | {note} |")
    A("\n† modulus not published as tensile modulus — derived (flexural modulus or Shore-hardness fit); see `derived.E1`.\n")
    return "\n".join(L)


METHOD = """\
* **Designs**: the flexblade presets (`api.presets()`), unchanged apart from the swapped slot.
  `body` = base material of blade and flexure (the TPU of TPU designs, the rigid filament of
  the all-filament crossed flexure); `skin` = the CF skin of the core/skin design; `insert` =
  the embedded rod/wire.
* **Torque**: 1/rev pulse at motor capability: `motor.capability(motor, rpm, Q).mod_frac_max`
  at the preset's hover trim, set as `op.torque_mod_frac` (duty drive) for every candidate.
* **Same geometry** shows what happens if you just load another spool. **Resized** keeps the
  preset's section stiffnesses: each flexure member's thickness (then width if t < 0.4 mm) and
  each blade station's t/c (0.06–0.20), skin thickness or rod diameter is solved so its soft
  bending stiffness EI_soft matches the preset (resize.py; < 0.2 % unless a bound is hit —
  marked **bound**). This isolates the material's own merit: density (inertia, centrifugal
  load), strength / creep / fatigue / interlayer allowables, anisotropy and damping.
* **Metrics** from `api.evaluate`: control_ratio = hub moment / (W·R); θ1P = 1/rev pitch
  amplitude; U = max structural utilisation over static, interlayer, creep, fatigue checks
  (motor/ESC checks excluded) and which check governs; lag/pitch frequency in /rev and margin
  to the nearest of 1P, 2P; static blade pitch split (blade 1 − blade 2, from drag-lag × skew);
  mass of blades + flexures (solid sections) and filament cost per rotor.
* **Fitness** (higher is better):

      F = CR · g(U) · h(m_lag) · h(m_pitch),   g(U) = 1 if U ≤ 1 else 1/U²,   h(m) = min(1, m / 0.15)

  i.e. control authority, kept only while every structural check passes, and penalised when a
  lag or pitch mode is within 0.15/rev of 1P or 2P (a mode the modal labeller cannot identify
  — very soft or mixed — counts as zero margin). *Feasible* = U ≤ 1 and both margins ≥ 0.15.
  Because the presets' flexure dimensions are still placeholders, most candidates have U > 1;
  the 1/U² term then ranks them by how far they are from working.
* **Sensitivity**: the top 6 of every table are re-run with E ×0.8 / ×1.2 and strengths ×0.8 / ×1.2
  on the geometry designed with nominal datasheet values (`material@E=0.8` variants); the
  fit±20% column shows the min–max fitness.
* **Derived properties** (never presented as datasheet values): see `flexblade/filament_db.py`
  docstring. Interlayer (Z) properties are TDS values where published, else class ratios;
  creep and fatigue allowables are class/hardness tables (LOW confidence for every brand —
  coupon tests needed); TPU frequency dependence is the generic DMA table.
"""

RECOMMENDATIONS = """\
## Recommendations

*(hand-written after the run — see the tables below for the evidence)*

RECOMMENDATIONS_PLACEHOLDER
"""


if __name__ == "__main__":
    main()
