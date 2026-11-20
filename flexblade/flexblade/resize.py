"""Material-agnostic design: resize geometry so a new material reproduces the original
section stiffnesses.

Workflow (DESIGN: "material-agnostic mode")
  1. `section_targets(design)` — the design's own section stiffnesses (EA, EI_soft,
     EI_stiff, GJ, mass/len) of every flexure member and every blade station. These
     are the *targets*: the design was solved (by hand or the optimiser) in terms of
     stiffness, the material is just how it is realised.
  2. `resize_for_material(design, slot, material)` — switch `slot` to `material` and
     rescale one geometric parameter per element so the element's **governing
     stiffness** (the soft bending stiffness EI_soft = smaller principal EI — the
     flexure hinge compliance, blade flap) matches its target:

        flexure / body slot, plate & wire members  thickness t     (EI ∝ E w t³)
            · if t would go below t_min (default 0.4 mm, min printable): t = t_min
              and the width w is scaled instead (EI ∝ w)
            · if t would exceed 0.5·length (no longer a flexure): t = 0.5 L and w
              is scaled up, limited by the blade root chord
        blade / body slot, airfoil stations         thickness ratio t/c ∈ [0.06, 0.20]
                                                     (chord and planform are fixed by aero)
        skin slot                                   skin thickness (each layup)
        insert slot                                 rod/wire diameter (≤ 0.9 × host thickness)

     A 1-D log-secant solve on the real composite section (section.section_properties
     with the new material, print orientation and anisotropy) — not the closed-form
     power law — so the match is exact to < 0.5 % whenever not bound-limited.
     Elements whose bound is hit are flagged infeasible with the achieved ratio.
  3. The caller re-runs the full simulator on the resized design (mass/inertia →
     frequencies and centrifugal load, strength/creep/fatigue/interlayer utilisation,
     control_ratio). The other stiffnesses (EA, GJ, EI_stiff) are *not* matched
     (one free parameter per element) — the report lists their new/target ratios.
"""
from __future__ import annotations

import math
from typing import Any, Callable

import numpy as np

from .schema import Design, Layup

T_MIN = 0.4e-3
TC_BOUNDS = (0.06, 0.20)
TOL = 0.002


# --------------------------------------------------------------------------- section evaluation

def _soft_stiff(sec) -> tuple[float, float]:
    a, b, c = sec.EIy, sec.EIz, sec.EIyz
    m = 0.5 * (a + b)
    d = math.sqrt(max(0.25 * (a - b) ** 2 + c * c, 0.0))
    return m - d, m + d


def _props(sec) -> dict[str, float]:
    soft, stiff = _soft_stiff(sec)
    return dict(EA=sec.EA, EI_soft=soft, EI_stiff=stiff, GJ=sec.GJ, mass_per_len=sec.mass_per_len)


def _member_frame(m):
    p0, p1 = np.asarray(m.p0, float), np.asarray(m.p1, float)
    e1 = (p1 - p0) / np.linalg.norm(p1 - p0)
    wd = np.asarray(m.width_dir, float)
    e2 = wd - e1 * (e1 @ wd)
    if np.linalg.norm(e2) < 1e-9:
        e2 = np.cross([0.0, 0.0, 1.0], e1) if abs(e1[2]) < 0.9 else np.cross([0.0, 1.0, 0.0], e1)
    e2 = e2 / np.linalg.norm(e2)
    return e1, e2, np.cross(e1, e2)


def member_section(m, thickness: float | None = None, width: float | None = None,
                   layup: Layup | None = None):
    """Mid-length section of a FlexureMember (blade 1, frame H), as rotor.build_model."""
    from .section import CircleShape, RectShape, section_properties
    t = (thickness if thickness is not None else m.thickness) * (1 + (m.taper - 1) * 0.5)
    w = width if width is not None else m.width
    shp = CircleShape(round(t, 9)) if m.shape == "circle" else RectShape(round(w, 9), round(t, 9))
    e1, e2, e3 = _member_frame(m)
    return section_properties(shp, layup or m.layup, m.orientation, e1, axes=(e2, e3))


def _station_section(design: Design, i: int):
    from .bladegeom import resolve_stations
    from .rotor import _blade_section
    stations = resolve_stations(design.blade)
    st_r = np.array([s.r_frac * design.blade.radius for s in stations])
    sec, *_ = _blade_section(design, float(st_r[i]), st_r, stations,
                             np.radians(design.op.collective_deg))
    return sec


def section_targets(design: Design) -> dict[str, Any]:
    """Section stiffnesses of every flexure member (mid-length) and blade station."""
    from . import fact
    from .bladegeom import resolve_stations
    out: dict[str, Any] = {"members": [], "stations": []}
    for j, m in enumerate(fact.generate_members(design)):
        if m.kind == "rigid":
            continue
        out["members"].append(dict(index=j, kind=m.kind, thickness=m.thickness, width=m.width,
                                   material=m.layup.material, **_props(member_section(m))))
    for i, st in enumerate(resolve_stations(design.blade)):
        out["stations"].append(dict(index=i, r_frac=st.r_frac, chord=st.chord,
                                    **_props(_station_section(design, i))))
    return out


# --------------------------------------------------------------------------- solver

def _solve(k_of: Callable[[float], float], target: float, p0: float, lo: float, hi: float,
           exp_guess: float) -> tuple[float, float, bool]:
    """Find p in [lo, hi] with k_of(p) = target (k monotone non-decreasing in p).
    Log-secant from p0 with exponent guess, then bracketing bisection (section.py
    rasterises skins and rods, so k(p) can be a staircase). Returns
    (p, k(p)/target, ok) with ok = |ratio-1| < 1 % (2 % for grid-quantised cases is
    flagged by the caller via the ratio)."""
    cache: dict[float, float] = {}

    def K(p):
        if p not in cache:
            cache[p] = k_of(p)
        return cache[p]

    p = float(np.clip(p0, lo, hi))
    k = K(p)
    n = exp_guess
    hist = [(math.log(p), math.log(k))]
    for _ in range(12):
        if abs(k / target - 1) < TOL:
            return p, k / target, True
        p_new = float(np.clip(p * (target / k) ** (1.0 / n), lo, hi))
        if p_new == p:   # stuck at a bound
            return p, k / target, False
        p = p_new
        k = K(p)
        hist.append((math.log(p), math.log(k)))
        (x0, y0), (x1, y1) = hist[-2], hist[-1]
        if abs(x1 - x0) > 1e-12:
            s = (y1 - y0) / (x1 - x0)
            if 0.2 < s < 8:
                n = s
    # bracketing bisection on the evaluated points
    below = [q for q, v in cache.items() if v < target]
    above = [q for q, v in cache.items() if v >= target]
    if not below:
        a = lo
        if K(a) >= target:
            return a, K(a) / target, abs(K(a) / target - 1) < 0.01
        below = [a]
    if not above:
        b = hi
        if K(b) < target:
            return b, K(b) / target, abs(K(b) / target - 1) < 0.01
        above = [b]
    a, b = max(below), min(above)
    for _ in range(40):
        if b / a - 1 < 1e-5:
            break
        m = math.sqrt(a * b)
        if K(m) < target:
            a = m
        else:
            b = m
    best = min(cache, key=lambda q: abs(cache[q] / target - 1) if lo <= q <= hi else np.inf)
    r = cache[best] / target
    return best, r, abs(r - 1) < 0.01


# --------------------------------------------------------------------------- element resizers

def _layup_set(l: Layup, slot: str, mat: str) -> Layup:
    f = {"body": "material", "blade": "material", "flexure": "material",
         "skin": "skin_material", "insert": "insert_material"}[slot]
    if slot == "skin" and not (l.kind == "core_skin" and l.skin_material):
        return l
    if slot == "insert" and not (l.kind == "rod_insert" and l.insert_material):
        return l
    return l.model_copy(update={f: mat})


def _resize_flexure(design: Design, slot: str, mat: str, t_min: float, rep: list) -> Design:
    """Resize the flexure members (topology-generated members share thickness/width)."""
    from . import fact
    f = design.flexure
    if f.topology == "rigid":
        return design
    gen = fact.generate_members(design)
    custom = f.topology == "custom"
    from .bladegeom import resolve_stations
    chord_max = max(s.chord for s in resolve_stations(design.blade))
    new_f = f.model_copy(deep=True)
    targets = [(j, m) for j, m in enumerate(gen) if m.kind != "rigid"]
    if not custom:
        # generated members are geometrically identical up to placement: use member 0
        # (and the tension member, if any, separately)
        idx = [targets[0]] + ([(len(gen) - 1, gen[-1])] if f.tension_member is not None else [])
    else:
        idx = targets + ([(len(gen) - 1, gen[-1])] if f.tension_member is not None else [])
    for j, m in idx:
        is_tension = f.tension_member is not None and j == len(gen) - 1
        newlay = _layup_set(m.layup, slot, mat)
        if newlay == m.layup and slot not in ("body", "flexure"):
            continue
        tgt = _props(member_section(m))
        L = float(np.linalg.norm(np.subtract(m.p1, m.p0)))
        entry = dict(element=("tension_member" if is_tension else f"member{j}" if custom else "flexure"),
                     target_EI_soft=tgt["EI_soft"])
        if slot in ("body", "flexure"):
            k_t = lambda t: _soft_stiff(member_section(m, thickness=t, layup=newlay))[0]  # noqa: E731
            lo = min(t_min, m.thickness)
            hi = max(0.5 * L, m.thickness) if m.shape == "rect" else max(0.5 * L, m.thickness)
            t, ratio, ok = _solve(k_t, tgt["EI_soft"], m.thickness, lo, hi, 3.0 if m.shape == "rect" else 4.0)
            w = m.width
            note = ""
            if not ok and m.shape == "rect":
                bound = "t_min" if t <= lo * 1.0001 else "t_max=0.5L"
                k_w = lambda ww: _soft_stiff(member_section(m, thickness=t, width=ww, layup=newlay))[0]  # noqa: E731
                w, ratio, ok = _solve(k_w, tgt["EI_soft"], m.width, 0.25 * m.width,
                                      max(chord_max, m.width), 1.0)
                note = f"thickness at {bound}; width rescaled"
            entry.update(param="thickness" + ("+width" if w != m.width else ""),
                         old=dict(thickness=m.thickness, width=m.width),
                         new=dict(thickness=t, width=w), ratio=ratio, feasible=ok, note=note,
                         below_min_printable=t < t_min * 0.999)
            new_sec = member_section(m, thickness=t, width=w, layup=newlay)
            if is_tension:
                new_f.tension_member = new_f.tension_member.model_copy(
                    update=dict(thickness=t, width=w, layup=newlay))
            elif custom:
                new_f.members[j] = new_f.members[j].model_copy(update=dict(thickness=t, width=w, layup=newlay))
            else:
                new_f.thickness, new_f.width = t, w
                new_f.layup = newlay
        elif slot == "skin":
            t_host = m.thickness
            k_s = lambda s: _soft_stiff(member_section(m, layup=newlay.model_copy(update=dict(skin_thickness=s))))[0]  # noqa: E731
            s0 = m.layup.skin_thickness
            s, ratio, ok = _solve(k_s, tgt["EI_soft"], s0, min(0.05e-3, s0), 0.45 * t_host, 1.0)
            newlay = newlay.model_copy(update=dict(skin_thickness=s))
            entry.update(param="skin_thickness", old=s0, new=s, ratio=ratio, feasible=ok,
                         below_min_printable=s < min(t_min, s0) * 0.999,
                         note="" if ok else ("skin bound hit (≤ 0.45 × member thickness)" if s >= 0.45 * t_host * 0.999 else "section-grid quantised skin (closest achievable)"))
            new_sec = member_section(m, layup=newlay)
            _assign_member_layup(new_f, custom, is_tension, j, newlay)
        else:  # insert
            d0 = m.layup.insert_diameter
            k_d = lambda d: _soft_stiff(member_section(m, layup=newlay.model_copy(update=dict(insert_diameter=d))))[0]  # noqa: E731
            d, ratio, ok = _solve(k_d, tgt["EI_soft"], d0, min(0.2e-3, d0), max(0.9 * m.thickness, d0), 2.0)
            newlay = newlay.model_copy(update=dict(insert_diameter=d))
            entry.update(param="insert_diameter", old=d0, new=d, ratio=ratio, feasible=ok,
                         note="" if ok else ("diameter bound hit (≤ 0.9 × member thickness)" if d >= 0.9 * m.thickness * 0.999 else "section-grid quantised rod (closest achievable)"))
            new_sec = member_section(m, layup=newlay)
            _assign_member_layup(new_f, custom, is_tension, j, newlay)
        entry["other_ratios"] = {k: _props(new_sec)[k] / tgt[k] for k in ("EA", "EI_stiff", "GJ", "mass_per_len")}
        rep.append(entry)
    return design.model_copy(update=dict(flexure=new_f))


def _assign_member_layup(new_f, custom, is_tension, j, newlay):
    if is_tension:
        new_f.tension_member = new_f.tension_member.model_copy(update=dict(layup=newlay))
    elif custom:
        new_f.members[j] = new_f.members[j].model_copy(update=dict(layup=newlay))
    else:
        new_f.layup = newlay


def _resize_blade(design: Design, slot: str, mat: str, t_min: float, rep: list) -> Design:
    """Per-station resize (t/c, skin thickness or insert diameter) to the station's EI_soft."""
    from .bladegeom import resolve_stations
    b = design.blade
    stations = resolve_stations(b)
    base_tc = (int(b.airfoil.naca[2:4]) / 100) if b.airfoil.kind == "naca4" else 0.12
    new_st = [s.model_copy(deep=True) for s in stations]
    any_change = False
    for i, st in enumerate(stations):
        lay = st.layup or b.layup
        newlay = _layup_set(lay, slot, mat)
        if newlay == lay and slot not in ("body", "blade"):
            continue
        tgt = _props(_station_section(design, i))

        def k_with(tc=None, layup=newlay):
            st2 = [s.model_copy(deep=True) for s in new_st]
            upd = dict(layup=layup)
            if tc is not None:
                upd["thickness_frac"] = tc
            st2[i] = st2[i].model_copy(update=upd)
            d2 = design.model_copy(update=dict(blade=b.model_copy(update=dict(stations=st2))))
            return _station_section(d2, i)

        tc0 = st.thickness_frac or base_tc
        entry = dict(element=f"station{i}", r_frac=st.r_frac, target_EI_soft=tgt["EI_soft"])
        if slot in ("body", "blade"):
            tc, ratio, ok = _solve(lambda x: _soft_stiff(k_with(tc=x))[0], tgt["EI_soft"], tc0,
                                   min(TC_BOUNDS[0], tc0), max(TC_BOUNDS[1], tc0), 3.0)
            entry.update(param="thickness_frac", old=tc0, new=tc, ratio=ratio, feasible=ok,
                         note="" if ok else f"t/c bound {TC_BOUNDS} hit (aero envelope)")
            new_st[i] = new_st[i].model_copy(update=dict(thickness_frac=tc, layup=newlay))
            new_sec = k_with(tc=tc)
        elif slot == "skin":
            s0 = lay.skin_thickness
            t_local = st.chord * tc0
            s, ratio, ok = _solve(lambda x: _soft_stiff(k_with(layup=newlay.model_copy(update=dict(skin_thickness=x))))[0],
                                  tgt["EI_soft"], s0, min(0.05e-3, s0), max(0.4 * t_local, s0), 1.0)
            lay2 = newlay.model_copy(update=dict(skin_thickness=s))
            entry.update(param="skin_thickness", old=s0, new=s, ratio=ratio, feasible=ok,
                         below_min_printable=s < min(t_min, s0) * 0.999,
                         note="" if ok else ("skin bound hit (≤ 0.4 × local airfoil thickness)" if s >= 0.4 * t_local * 0.999 else "section-grid quantised skin (closest achievable)"))
            new_st[i] = new_st[i].model_copy(update=dict(layup=lay2))
            new_sec = k_with(layup=lay2)
        else:
            d0 = lay.insert_diameter
            t_local = st.chord * tc0
            d, ratio, ok = _solve(lambda x: _soft_stiff(k_with(layup=newlay.model_copy(update=dict(insert_diameter=x))))[0],
                                  tgt["EI_soft"], d0, min(0.2e-3, d0), max(0.8 * t_local, d0), 2.0)
            lay2 = newlay.model_copy(update=dict(insert_diameter=d))
            entry.update(param="insert_diameter", old=d0, new=d, ratio=ratio, feasible=ok,
                         note="" if ok else ("diameter bound hit (≤ 0.8 × local airfoil thickness)" if d >= 0.8 * t_local * 0.999 else "section-grid quantised rod (closest achievable)"))
            new_st[i] = new_st[i].model_copy(update=dict(layup=lay2))
            new_sec = k_with(layup=lay2)
        entry["other_ratios"] = {k: _props(new_sec)[k] / tgt[k] for k in ("EA", "EI_stiff", "GJ", "mass_per_len")}
        rep.append(entry)
        any_change = True
    if not any_change:
        return design
    blade_lay = _layup_set(b.layup, slot, mat)
    return design.model_copy(update=dict(blade=b.model_copy(update=dict(stations=new_st, layup=blade_lay))))


def resize_for_material(design: Design | dict, slot: str, material: str,
                        t_min: float = T_MIN) -> tuple[Design, dict[str, Any]]:
    """Switch `slot` to `material` and resize to reproduce the original section
    stiffnesses. Returns (resized design, report). Report: {slot, material, elements:
    [{element, param, old, new, ratio (achieved/target EI_soft), feasible, note,
    other_ratios{EA, EI_stiff, GJ, mass_per_len}}], feasible, worst_ratio}."""
    from .materials import get_material
    from .matsweep import set_material, slot_materials
    design = design if isinstance(design, Design) else Design.model_validate(design)
    get_material(material)
    if not slot_materials(design, slot):
        raise ValueError(f"slot {slot!r} not used by design {design.name!r}")
    if design.layout == "coaxial" and (design.coaxial.lower_blade or design.coaxial.lower_flexure):
        raise NotImplementedError("resize: coaxial designs with a distinct lower rotor not supported")
    rep: list[dict] = []
    d = design
    if slot in ("body", "flexure", "skin", "insert"):
        d = _resize_flexure(d, slot, material, t_min, rep)
    if slot in ("body", "blade", "skin", "insert"):
        d = _resize_blade(d, slot, material, t_min, rep)
    # make sure every layup in the slot carries the new material (stations without own layup etc.)
    d = set_material(d, slot, material)
    worst = max((abs(math.log(e["ratio"])) for e in rep), default=0.0)
    report = dict(slot=slot, material=material, t_min=t_min, elements=rep,
                  feasible=all(e["feasible"] for e in rep),
                  worst_ratio=math.exp(worst),
                  below_min_printable=any(e.get("below_min_printable") for e in rep))
    return d, report
