"""Printer profiles and printability checks.

ALL SPECS below are from retailer/review pages, Nov 2026 -- VERIFY before relying on them.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .materials import get_material
from .schema import Check, Design


@dataclass(frozen=True)
class Printer:
    name: str
    toolheads: int
    build: tuple[float, float, float]        # m (x, y, z)
    nozzle: float                            # m, default nozzle
    min_wall: float                          # m
    min_feature: float                       # m
    max_nozzle_temp_C: Optional[float]       # None = unverified
    chamber_temp_C: float = 0.0              # actively heated chamber max (0 = none)
    hardened_nozzle: bool = False            # ships/available with hardened nozzle
    min_tpu_shore_a: float = 0.0             # 0 = no restriction known
    requires_for_cf: tuple[str, ...] = ()
    notes: str = ""


SPEC_NOTE = "from retailer/review pages Nov 2026 - verify"

PRINTERS: dict[str, Printer] = {
    "snapmaker_u1": Printer(
        "snapmaker_u1", toolheads=4, build=(0.270, 0.270, 0.270), nozzle=0.4e-3,
        min_wall=0.8e-3, min_feature=0.4e-3, max_nozzle_temp_C=300.0, chamber_temp_C=0.0,
        min_tpu_shore_a=90.0, requires_for_cf=("hardened nozzle", "top cover"),
        notes=f"CF needs hardened nozzle + top cover; TPU >= 90A officially; {SPEC_NOTE}"),
    "bambu_h3c": Printer(
        "bambu_h3c", toolheads=9, build=(0.330, 0.320, 0.325), nozzle=0.4e-3,
        min_wall=0.8e-3, min_feature=0.4e-3, max_nozzle_temp_C=350.0, chamber_temp_C=65.0,
        hardened_nozzle=True,
        notes=f"hardened HRC55+ nozzles 0.2-0.8 mm, 65 C actively heated chamber; {SPEC_NOTE}"),
    "bambu_h2c": Printer(
        "bambu_h2c", toolheads=2, build=(0.325, 0.320, 0.320), nozzle=0.4e-3,
        min_wall=0.8e-3, min_feature=0.4e-3, max_nozzle_temp_C=350.0, chamber_temp_C=65.0,
        hardened_nozzle=True,
        notes=f"hotend changer (2 active hotends; more via changer) -- VERIFY toolhead count, "
              f"build volume and temps; {SPEC_NOTE}"),
    "prusa_xl_5t": Printer(
        "prusa_xl_5t", toolheads=5, build=(0.360, 0.360, 0.360), nozzle=0.4e-3,
        min_wall=0.8e-3, min_feature=0.4e-3, max_nozzle_temp_C=None, chamber_temp_C=0.0,
        notes=f"max nozzle temp unverified (None: temperature check skipped); open frame, no "
              f"heated chamber; {SPEC_NOTE}"),
}


def get_printer(name: str) -> Printer:
    try:
        return PRINTERS[name]
    except KeyError:
        raise KeyError(f"unknown printer {name!r}; known: {sorted(PRINTERS)}") from None


def _layups(design: Design):
    b, f = design.blade, design.flexure
    ls = [b.layup, f.layup] + [s.layup for s in b.stations if s.layup] + [m.layup for m in f.members]
    if f.tension_member:
        ls.append(f.tension_member.layup)
    return ls


def printed_materials(design: Design) -> list[str]:
    """Distinct printed (print_temp > 0) material names used anywhere in the design."""
    names: list[str] = []
    for l in _layups(design):
        names += [l.material] + ([l.skin_material] if l.kind == "core_skin" else []) + \
                 ([l.insert_material] if l.kind in ("rod_insert", "laminate_strip") else [])
    j = design.blade.joint
    if j is not None:
        names += [j.material_a or design.flexure.layup.material,
                  j.material_b or design.blade.layup.material]
    return sorted({n for n in names if n and get_material(n).print_temp > 0})


def min_fit_extent(length: float, width: float, bed: tuple[float, float]) -> float:
    """Smallest max(bbox_x/bed_x, bbox_y/bed_y) of a length x width rectangle rotated in XY."""
    best = np.inf
    for phi in np.radians(np.linspace(0, 90, 181)):
        bx = length * np.cos(phi) + width * np.sin(phi)
        by = length * np.sin(phi) + width * np.cos(phi)
        best = min(best, max(bx / bed[0], by / bed[1]))
    return float(best)


def printer_checks(design: Design, printer: Optional[Printer] = None) -> list[Check]:
    """Printability checks (utilisation > 1 fails; value/allowable in SI)."""
    if printer is None:
        if design.printer is None:
            return []
        printer = get_printer(design.printer)
    P = printer
    b, f = design.blade, design.flexure
    out: list[Check] = []
    # ---- build volume: blades are slender rectangles, diagonal placement allowed
    chords = [s.chord for s in b.stations] or [0.02]
    cmax = max(chords)
    height = cmax * 0.15 + cmax * np.sin(np.radians(max([abs(s.twist_deg) for s in b.stations] or [0.0])))
    util_z = height / P.build[2]
    R = b.radius
    for label, span in (("rotor", 2 * R), ("blade", R + f.hub_radius)):
        if label == "rotor" and b.n_blades != 2:
            span = 2 * R
        u = min_fit_extent(span, cmax, P.build[:2])
        out.append(Check(name=f"print_fit_{label}", member="printer", value=span,
                         allowable=span / u if u > 0 else 0.0, utilisation=max(u, util_z),
                         note=f"{P.name} bed {P.build[0]*1e3:.0f}x{P.build[1]*1e3:.0f} mm, "
                              "diagonal placement allowed"))
    # ---- min wall
    walls: dict[str, float] = {}
    if f.topology != "rigid":
        walls["flexure"] = f.thickness
    for i, m in enumerate(f.members):
        walls[f"member{i}"] = m.thickness
    for nm, l in (("blade", b.layup), ("flexure", f.layup)):
        if l.kind == "core_skin" and l.skin_thickness > 0:
            walls[f"{nm}_skin"] = l.skin_thickness
    j = b.joint
    if j is not None:
        walls["joint_neck"] = j.neck_width
        try:
            from . import joint as jm
            js = jm.resolve(design)
            walls["joint_gap"] = js.pitch - j.dovetail_width
        except ValueError:
            pass
    for nm, t in walls.items():
        out.append(Check(name="min_wall", member=nm, value=t, allowable=P.min_wall,
                         utilisation=P.min_wall / t if t > 0 else np.inf,
                         note=f"{P.name} min wall {P.min_wall*1e3:.1f} mm (value must be >= allowable)"))
    # ---- materials / toolheads / temps
    mats = printed_materials(design)
    out.append(Check(name="toolheads", member="printer", value=float(len(mats)),
                     allowable=float(P.toolheads), utilisation=len(mats) / P.toolheads,
                     note="printed materials: " + ", ".join(mats)))
    for n in mats:
        m = get_material(n)
        if P.max_nozzle_temp_C is not None:
            out.append(Check(name="nozzle_temp", member=n, value=m.print_temp,
                             allowable=P.max_nozzle_temp_C,
                             utilisation=m.print_temp / P.max_nozzle_temp_C))
        if m.is_elastomer and P.min_tpu_shore_a > 0 and m.shore_a > 0:
            out.append(Check(name="tpu_shore", member=n, value=m.shore_a, allowable=P.min_tpu_shore_a,
                             utilisation=P.min_tpu_shore_a / m.shore_a,
                             note="printer officially supports TPU >= this Shore A"))
        if m.needs_chamber and P.chamber_temp_C <= 0:
            out.append(Check(name="heated_chamber", member=n, value=0.0, allowable=1.0,
                             utilisation=1.0,
                             note=f"{n} wants a heated chamber; {P.name} has none "
                                  "(warping/delamination risk)"))
        if m.abrasive:
            out.append(Check(name="cf_hardware", member=n, value=0.0, allowable=1.0,
                             utilisation=0.0 if P.hardened_nozzle else 0.5,
                             note=("hardened nozzle fitted; " if P.hardened_nozzle else
                                   "requires " + ", ".join(P.requires_for_cf or ("hardened nozzle",))
                                   + "; ")))
    return out
