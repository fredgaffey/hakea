"""Material slots, material sweeps, rotor mass / filament cost, and a flat metric record.

Slots (where a material sits in a Design's layups):

  body     base `material` of every blade AND flexure layup (monolithic TPU, the TPU
           core of core/skin or rod designs, the CF of an all-CF flexure design)
  blade    base material of the blade layups only (blade.layup, station layups)
  flexure  base material of the flexure layups only (flexure.layup, members, tension member)
  skin     `skin_material` of every core_skin layup
  insert   `insert_material` of every rod_insert layup

`material_sweep` evaluates one design per material for a slot, geometry unchanged
(``resize=False``) or resized to the original section stiffnesses
(``resize=True``, see resize.py) — the latter is the fair comparison.
"""
from __future__ import annotations

import csv
import io
import json
import math
import os
from typing import Any, Callable, Iterable

import numpy as np

from .schema import Design, Layup

SLOTS = ("body", "blade", "flexure", "skin", "insert")
_FIELD = {"body": "material", "blade": "material", "flexure": "material",
          "skin": "skin_material", "insert": "insert_material"}


# --------------------------------------------------------------------------- layup walking

def _blade_layups(dm: dict) -> Iterable[dict]:
    for b in (dm["blade"], (dm.get("coaxial") or {}).get("lower_blade")):
        if not b:
            continue
        yield b["layup"]
        for st in b.get("stations") or []:
            if st.get("layup"):
                yield st["layup"]


def _flexure_layups(dm: dict) -> Iterable[dict]:
    for f in (dm["flexure"], (dm.get("coaxial") or {}).get("lower_flexure")):
        if not f:
            continue
        yield f["layup"]
        for m in f.get("members") or []:
            yield m["layup"]
        if f.get("tension_member"):
            yield f["tension_member"]["layup"]


def _slot_layups(dm: dict, slot: str) -> list[dict]:
    if slot not in SLOTS:
        raise ValueError(f"unknown slot {slot!r}; known {SLOTS}")
    lays = []
    if slot in ("body", "blade", "skin", "insert"):
        lays += list(_blade_layups(dm))
    if slot in ("body", "flexure", "skin", "insert"):
        lays += list(_flexure_layups(dm))
    if slot == "skin":
        lays = [l for l in lays if l["kind"] == "core_skin" and l.get("skin_material")]
    if slot == "insert":
        lays = [l for l in lays if l["kind"] == "rod_insert" and l.get("insert_material")]
    return lays


def slot_materials(design: Design, slot: str) -> list[str]:
    """Materials currently in `slot` (empty → slot unused by this design)."""
    dm = design.model_dump()
    return sorted({l[_FIELD[slot]] for l in _slot_layups(dm, slot)})


def used_slots(design: Design) -> dict[str, list[str]]:
    return {s: m for s in SLOTS if (m := slot_materials(design, s))}


def set_material(design: Design, slot: str, material: str) -> Design:
    """Copy of `design` with every layup in `slot` switched to `material`."""
    from .materials import get_material
    get_material(material)
    dm = design.model_dump()
    lays = _slot_layups(dm, slot)
    if not lays:
        raise ValueError(f"slot {slot!r} is not used by design {design.name!r} "
                         f"(used: {list(used_slots(design))})")
    for l in lays:
        l[_FIELD[slot]] = material
    return Design.model_validate(dm)


# --------------------------------------------------------------------------- mass / cost

def _split_mass(sec, layup: Layup) -> dict[str, float]:
    """kg/m per material for a section (2-material area split from area + mass)."""
    from .materials import get_material
    m_tot = sec.mass_per_len
    if layup.kind == "solid":
        return {layup.material: m_tot}
    other = layup.skin_material if layup.kind == "core_skin" else layup.insert_material
    if not other:
        return {layup.material: m_tot}
    rc, ro = get_material(layup.material).rho, get_material(other).rho
    A = sec.area
    if layup.kind == "rod_insert":
        Ao = min(A, len(layup.insert_positions) * math.pi * layup.insert_diameter ** 2 / 4)
    elif abs(ro - rc) > 1e-9:
        Ao = float(np.clip((m_tot - rc * A) / (ro - rc), 0.0, A))
    else:
        Ao = 0.0
    out = {layup.material: rc * (A - Ao)}
    out[other] = out.get(other, 0.0) + ro * Ao
    return out


def rotor_mass(design: Design) -> dict[str, Any]:
    """Printed/inserted mass of all blades + flexures (solid sections, 100 % infill),
    split per material. Hub not included. Coaxial: both rotors."""
    from . import fact
    from .rotor import _blade_section, build_model
    from .bladegeom import resolve_stations
    designs = [design]
    if design.layout == "coaxial":
        cx = design.coaxial
        lower = design.model_copy(update=dict(
            layout="single", blade=cx.lower_blade or design.blade,
            flexure=cx.lower_flexure or design.flexure), deep=True)
        designs = [design.model_copy(update=dict(layout="single")), lower]
    per: dict[str, float] = {}
    for d in designs:
        model = build_model(d)
        fe = model.fe
        stations = resolve_stations(d.blade)
        st_r = np.array([s.r_frac * d.blade.radius for s in stations])
        coll = np.radians(d.op.collective_deg)
        members = fact.generate_members(d)
        for el in fe.elements:
            if el.massless or el.spring_K is not None or not el.sections or el.blade == 0:
                continue
            x0, x1 = np.asarray(fe.X0[el.n0]), np.asarray(fe.X0[el.n1])
            L = float(np.linalg.norm(x1 - x0))
            secs = el.mass_sections or el.sections
            if el.kind in ("blade", "shank"):
                r = float(np.linalg.norm(0.5 * (x0 + x1)))
                _, _, _, layup = _blade_section(d, r, st_r, stations, coll)
            elif el.kind == "flexure":
                j = int(el.tag.split(".flex")[1])
                layup = members[j].layup
            else:
                continue
            for s in secs:
                for mat, mpl in _split_mass(s, layup).items():
                    per[mat] = per.get(mat, 0.0) + mpl * L / len(secs)
    return {"total": float(sum(per.values())), "per_material": per}


def filament_cost(mass: dict[str, Any]) -> dict[str, Any]:
    """USD cost of the material in `rotor_mass` output (per-kg priced entries only;
    rods/wire priced per metre are reported as unpriced)."""
    from .materials import META
    cost, unpriced = 0.0, []
    for mat, kg in mass["per_material"].items():
        base = mat.split("@", 1)[0]
        p = (META.get(base) or {}).get("usd_per_kg")
        if p is None:
            unpriced.append(mat)
        else:
            cost += kg * p
    return {"usd": cost, "unpriced": unpriced}


# --------------------------------------------------------------------------- metrics

_NON_STRUCT = ("esc", "motor")


def _margin(f_per_rev: float | None) -> float | None:
    if f_per_rev is None or not math.isfinite(f_per_rev):
        return None
    return float(min(abs(f_per_rev - 1.0), abs(f_per_rev - 2.0)))


def fitness(control_ratio: float, util: float, lag_margin: float | None,
            pitch_margin: float | None, margin_ref: float = 0.15) -> float:
    """Combined fitness (higher better):

        F = control_ratio · g(U) · h(m_lag) · h(m_pitch)
        g(U) = 1 for U ≤ 1, else 1/U²       (structural utilisation, all checks)
        h(m) = min(1, m / 0.15)              (distance of the mode from 1P/2P in /rev)

    i.e. control authority, kept only while the part survives (creep, fatigue,
    interlayer, static) and the lag/pitch modes stay ≥ 0.15/rev from 1P and 2P."""
    g = 1.0 if util <= 1 else 1.0 / util ** 2
    h = 1.0
    for m in (lag_margin, pitch_margin):
        if m is not None:
            h *= min(1.0, max(m, 0.0) / margin_ref)
    return float(abs(control_ratio) * g * h)


def metrics(res: dict, design: Design | None = None, with_mass: bool = True) -> dict[str, Any]:
    """Flat record from Results (dict) + optional rotor mass / filament cost."""
    s = res.get("summary") or {}
    if res.get("coaxial"):
        s = dict(res["coaxial"].get("upper", {}).get("summary") or {}, **s)
        s["control_ratio"] = res["coaxial"]["control_ratio"]
    checks = [c for c in res.get("checks") or [] if c["member"] not in _NON_STRUCT]
    gov = max(checks, key=lambda c: c["utilisation"]) if checks else None
    by_kind: dict[str, float] = {}
    for c in checks:
        k = c["name"]
        if k.startswith("fatigue"):
            k = "fatigue"
        elif k.startswith("static"):
            k = "static"
        elif k.startswith("creep"):
            k = "creep"
        by_kind[k] = max(by_kind.get(k, 0.0), c["utilisation"])
    rec = dict(
        control_ratio=s.get("control_ratio"), pitch_1p_deg=s.get("pitch_amp_deg"),
        lag_1p_deg=s.get("lag_amp_deg"), hub_moment=s.get("hub_moment_mean"),
        moment_phase_deg=s.get("hub_moment_phase_deg"),
        max_util=gov["utilisation"] if gov else None,
        governing=f"{gov['name']}@{gov['member']}" if gov else None,
        util_creep=by_kind.get("creep"), util_fatigue=by_kind.get("fatigue"),
        util_interlayer=by_kind.get("interlayer"), util_static=by_kind.get("static"),
        lag_per_rev=s.get("lag_freq_per_rev"), flap_per_rev=s.get("flap_freq_per_rev"),
        pitch_per_rev=s.get("pitch_freq_per_rev"),
        lag_margin=_margin(s.get("lag_freq_per_rev")),
        pitch_margin=_margin(s.get("pitch_freq_per_rev")),
        pitch_split_deg=s.get("trim_diff_pitch_deg"), rpm=s.get("rpm"),
        esc_util=max((c["utilisation"] for c in res.get("checks") or [] if c["member"] in _NON_STRUCT),
                     default=None),
    )
    if rec["control_ratio"] is not None and rec["max_util"] is not None:
        rec["fitness"] = fitness(rec["control_ratio"], rec["max_util"], rec["lag_margin"],
                                 rec["pitch_margin"])
        rec["feasible"] = bool(rec["max_util"] <= 1.0 and (rec["lag_margin"] or 0) >= 0.15
                               and (rec["pitch_margin"] or 0) >= 0.15)
        # creep allowables are class tables (low confidence): also report without creep
        nc = [c["utilisation"] for c in checks if not c["name"].startswith("creep")]
        rec["max_util_nocreep"] = max(nc) if nc else None
        if nc:
            rec["fitness_nocreep"] = fitness(rec["control_ratio"], max(nc), rec["lag_margin"],
                                             rec["pitch_margin"])
    if with_mass and design is not None:
        m = rotor_mass(design)
        rec["rotor_mass_g"] = m["total"] * 1e3
        rec["mass_by_material_g"] = {k: v * 1e3 for k, v in m["per_material"].items()}
        c = filament_cost(m)
        rec["filament_cost_usd"] = c["usd"]
        rec["unpriced"] = c["unpriced"]
    return rec


# --------------------------------------------------------------------------- torque at capability

def capability_mod_frac(design: Design, res: dict | None = None) -> float:
    """1/rev torque modulation the motor can deliver at the trimmed hover point:
    motor.capability(motor, rpm, Q).dQ_max_1p / Q (= mod_frac_max)."""
    from . import api
    from .motor import capability
    if res is None:
        res = api.evaluate(design, ["trim"]).model_dump(mode="json")
    tr = res.get("trim") or res["coaxial"]["upper"]["trim"]
    return float(capability(design.motor, tr["rpm"], tr["torque"]).mod_frac_max)


def with_torque_mod(design: Design, frac: float) -> Design:
    op = design.op.model_copy(update=dict(torque_mod_frac=float(frac)))
    return design.model_copy(update=dict(op=op))


# --------------------------------------------------------------------------- sweep

def _prepare_job(job: tuple) -> dict:
    """(design_json, slot, material, resize, t_min) -> {design, resize_report | error}."""
    dj, slot, mat, resize, t_min = job
    d = Design.model_validate(dj)
    try:
        if resize:
            from .resize import resize_for_material
            d2, rep = resize_for_material(d, slot, mat, t_min=t_min)
        else:
            d2, rep = set_material(d, slot, mat), None
        d2 = d2.model_copy(update=dict(name=f"{d.name}|{slot}={mat}" + ("|resized" if resize else "")))
        return {"material": mat, "design": d2.model_dump(mode="json"), "resize": rep}
    except Exception as e:  # noqa: BLE001
        return {"material": mat, "error": f"{type(e).__name__}: {e}"}


def _eval_job(job: dict) -> dict:
    from . import api
    from .uicommon import strip_heavy
    try:
        d = Design.model_validate(job["design"])
        r = strip_heavy(api.evaluate(d).model_dump(mode="json"))
        return {"id": job["id"], "metrics": metrics(r, d), "warnings": r.get("warnings", [])}
    except Exception as e:  # noqa: BLE001
        return {"id": job["id"], "error": f"{type(e).__name__}: {e}"}


def _pool(workers: int):
    import multiprocessing as mp
    os.environ["FLEXBLADE_WORKER"] = "1"
    return mp.get_context("spawn").Pool(workers)


def run_jobs(fn: Callable, jobs: list, workers: int | None = None, progress=None) -> list:
    workers = workers or max(1, (os.cpu_count() or 2) - 2)
    if workers <= 1 or len(jobs) <= 1:
        out = []
        for i, j in enumerate(jobs):
            out.append(fn(j))
            if progress:
                progress(i + 1, len(jobs))
        return out
    out = [None] * len(jobs)
    with _pool(min(workers, len(jobs))) as pool:
        for n, (i, r) in enumerate(pool.imap_unordered(_indexed, [(fn, i, j) for i, j in enumerate(jobs)])):
            out[i] = r
            if progress:
                progress(n + 1, len(jobs))
    return out


def _indexed(a):
    fn, i, j = a
    return i, fn(j)


def material_sweep(design: Design | dict, slot: str, materials: str | list[str] = "all",
                   resize: bool = False, torque: str | float = "design",
                   hosts: str | None = None, workers: int | None = None,
                   t_min: float = 0.4e-3, progress=None) -> list[dict]:
    """Evaluate `design` with each material in `slot`.

    materials: selector (see materials.select_materials) or list of names.
    resize: False = same geometry; True = resize to the design's own section stiffnesses.
    torque: 'design' (op.torque_mod_frac as given), 'capability' (motor capability at
            the baseline's hover trim, applied to every material), or a number.
    hosts: optional HostPool spec ("local:16,mac:10") for the evaluations; otherwise a
           local process pool of `workers`.
    Returns one record per material: {material, brand, product, cls, slot, resized,
    metrics..., resize (report), error}."""
    from .materials import META, get_material, material_class, select_materials
    design = design if isinstance(design, Design) else Design.model_validate(design)
    if not slot_materials(design, slot):
        raise ValueError(f"slot {slot!r} not used by {design.name!r}; used: {list(used_slots(design))}")
    mats = select_materials(materials)
    if slot != "insert":
        from .filament_db import INSERT_CLASSES
        mats = [m for m in mats if material_class(m) not in INSERT_CLASSES] or mats
    if torque == "capability":
        frac = capability_mod_frac(design)
        design = with_torque_mod(design, frac)
    elif not isinstance(torque, str):
        design = with_torque_mod(design, float(torque))
    dj = design.model_dump(mode="json")
    prep = run_jobs(_prepare_job, [(dj, slot, m, resize, t_min) for m in mats],
                    workers=workers if resize else 1)
    ok = [p for p in prep if "design" in p]
    jobs = [{"id": i, "design": p["design"]} for i, p in enumerate(ok)]
    if hosts:
        from .remote import HostPool
        with HostPool(hosts, what=None) as hp:
            raw = hp.map([j["design"] for j in jobs])
        evals = []
        for j, r in zip(jobs, raw):
            if "results" in r:
                d = Design.model_validate(j["design"])
                evals.append({"metrics": metrics(r["results"], d),
                              "warnings": r["results"].get("warnings", [])})
            else:
                evals.append({"error": r.get("error", "?")})
    else:
        evals = run_jobs(_eval_job, jobs, workers=workers, progress=progress)
    out = []
    by_mat = {p["material"]: p for p in prep}
    ev_by_mat = {p["material"]: e for p, e in zip(ok, evals)}
    for m in mats:
        meta = META.get(m.split("@", 1)[0], {})
        rec = dict(material=m, slot=slot, resized=bool(resize), design=design.name,
                   brand=meta.get("brand", "generic"), product=meta.get("product", m),
                   cls=material_class(m), url=meta.get("url"),
                   usd_per_kg=meta.get("usd_per_kg"), confidence=meta.get("confidence", "low"),
                   E1=get_material(m).E1, rho=get_material(m).rho)
        p = by_mat[m]
        if "error" in p:
            rec["error"] = p["error"]
        else:
            rec["resize"] = p.get("resize")
            e = ev_by_mat[m]
            if "error" in e:
                rec["error"] = e["error"]
            else:
                rec.update(e["metrics"])
        out.append(rec)
    return out


TABLE_COLS = ("material", "brand", "product", "cls", "control_ratio", "pitch_1p_deg", "max_util",
              "governing", "lag_per_rev", "pitch_per_rev", "lag_margin", "pitch_margin",
              "pitch_split_deg", "rotor_mass_g", "filament_cost_usd", "fitness", "feasible",
              "resize_ok", "error")


def to_csv(records: list[dict], cols: tuple[str, ...] = TABLE_COLS) -> str:
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(cols)
    for r in records:
        row = []
        for c in cols:
            v = r.get(c)
            if c == "resize_ok":
                v = None if not r.get("resize") else r["resize"].get("feasible")
            row.append(f"{v:.6g}" if isinstance(v, float) else ("" if v is None else v))
        w.writerow(row)
    return buf.getvalue()


def to_jsonl(records: list[dict]) -> str:
    return "".join(json.dumps(r, default=str) + "\n" for r in records)
