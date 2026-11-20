"""Reference designs 4"–12" × material concepts (OWNER: aero; flexure dims are
placeholders for the structures owner to tune).

Assumptions
-----------
* Vehicle mass m = 0.6 kg · (D/5")^1.5.  Hakea (5", one rotor carries the whole
  vehicle) is ~0.6 kg AUW → disk loading ≈ 465 N/m².  Constant disk loading
  (m ∝ D²) makes 10–12" vehicles need >0.5 kW from one motor; real larger builds
  run lower disk loading, so the exponent 1.5 (DL ∝ D^−0.5: 4" 520, 12" 300 N/m²)
  is a compromise.  Hover tip speed then falls slowly with size.
* Blade law (hover-optimised for these disk loadings, see `hover_blade_stations`):
      chord c = R (0.30 − 0.12 r/R),        root cut-out 0.22 R
      twist θ = 8° + atan(0.12 / (r/R))     (design α ≈ 5–8° at the BEMT inflow,
                                              ≈ ideal-twist 1/r shape)
  NACA 4412.  At 5"/0.6 kg this trims to ≈18.5k rpm, Q ≈ 0.058 N m, FoM ≈ 0.71
  (BEMT without swirl/3-D losses: FoM optimistic by ~0.1).
* Motors: `motor.motor_for_diameter` (stock 6S motors, see motor.py notes).
* Flexure: lengths ∝ R from the 5" placeholder (hub r 6 mm, flexure 8 mm,
  hinge at 10 mm, skew 45°); thickness/width per concept chosen only to be
  plausible for the modulus (TPU ~90 MPa vs PA6-CF ~7.5 GPa).
"""
from __future__ import annotations

import re

import numpy as np

from .motor import motor_for_diameter
from .schema import (Airfoil, AnalysisOptions, Blade, BladeStation, Design, Flexure, Layup,
                     OperatingPoint)

SIZES_IN = (4, 5, 7, 10, 12)
CONCEPTS = ("tpu_monolithic", "tpu_rod", "tpu_cf_skin", "cf_flexure")
MASS_5IN = 0.6
MASS_EXPONENT = 1.5
ROOT_CUTOUT = 0.22
CHORD_LAW = (0.30, 0.12)
TWIST_ALPHA_DEG = 8.0
TWIST_LAMBDA = 0.12


def vehicle_mass(diameter_in: float) -> float:
    return MASS_5IN * (diameter_in / 5.0) ** MASS_EXPONENT


def hover_blade_stations(diameter_in: float, n: int = 12, root_cutout: float = ROOT_CUTOUT,
                         chord_law=CHORD_LAW, alpha_deg: float = TWIST_ALPHA_DEG,
                         lam: float = TWIST_LAMBDA) -> list[BladeStation]:
    """Explicit stations: c = R(c0 − c1 x), θ = α_d + atan(λ_d / x), x = r/R."""
    R = diameter_in * 0.0254 / 2
    out = []
    for x in np.linspace(root_cutout, 1.0, n):
        out.append(BladeStation(r_frac=float(x), chord=float(R * (chord_law[0] - chord_law[1] * x)),
                                twist_deg=float(alpha_deg + np.degrees(np.arctan(lam / x)))))
    return out


def _concept(concept: str, s: float) -> tuple[Layup, Flexure]:
    """Blade layup and flexure for a material concept at length scale s = R/R_5in."""
    tpu = Layup(kind="solid", material="tpu_64d")
    base = dict(skew_deg=45.0, hinge_radius=0.010 * s, length=0.008 * s, hub_radius=0.006 * s)
    if concept == "tpu_monolithic":
        return tpu, Flexure(topology="integral_taper", thickness=0.0025 * s, width=0.010 * s,
                            taper=0.7, layup=tpu, **base)
    if concept == "tpu_rod":
        rod = Layup(kind="rod_insert", material="tpu_64d", insert_material="cf_rod_pultruded",
                    insert_diameter=0.0008 * s, insert_positions=[0.3, 0.7])
        blade = Layup(kind="rod_insert", material="tpu_64d", insert_material="cf_rod_pultruded",
                      insert_diameter=0.0010 * s, insert_positions=[0.3])
        return blade, Flexure(topology="skewed_single_blade", thickness=0.0020 * s,
                              width=0.010 * s, layup=rod, **base)
    if concept == "tpu_cf_skin":
        blade = Layup(kind="core_skin", material="tpu_64d", skin_material="pa6_cf",
                      skin_thickness=0.0004 * s)
        flex = Layup(kind="core_skin", material="tpu_64d", skin_material="pa6_cf",
                     skin_thickness=0.00015 * s)
        return blade, Flexure(topology="skewed_single_blade", thickness=0.0015 * s,
                              width=0.010 * s, layup=flex, **base)
    if concept == "cf_flexure":
        cf = Layup(kind="solid", material="pa6_cf")
        return cf, Flexure(topology="skewed_cross_blade", thickness=0.0008 * s, width=0.008 * s,
                           cross_angle_deg=90.0, layup=cf, **base)
    raise KeyError(f"unknown concept {concept!r}; known {CONCEPTS}")


def make_preset(diameter_in: float, concept: str) -> Design:
    s = diameter_in / 5.0
    layup, flex = _concept(concept, s)
    blade = Blade(diameter_in=float(diameter_in), n_blades=2, root_cutout_frac=ROOT_CUTOUT,
                  airfoil=Airfoil(kind="naca4", naca="4412"),
                  stations=hover_blade_stations(diameter_in), layup=layup)
    return Design(
        name=f"{_fmt(diameter_in)}in_{concept}", blade=blade, flexure=flex,
        motor=motor_for_diameter(diameter_in),
        op=OperatingPoint(vehicle_mass=round(vehicle_mass(diameter_in), 4)),
        analysis=AnalysisOptions(),
        notes=f"preset: {concept}, mass ∝ D^{MASS_EXPONENT}, hover blade law; "
              "flexure dimensions are scaled placeholders (structures to tune)")


def _fmt(d: float) -> str:
    return str(int(d)) if float(d).is_integer() else f"{d:g}"


def all_presets() -> dict[str, Design]:
    """{name: Design}: "<D>in_<concept>" for D in 4,5,7,10,12 and every concept,
    plus "hakea_5in_hinge_equiv" (5" with a stiff-material crossed-blade flexure
    standing in for Hakea's skewed lag hinge, skew 45°)."""
    out = {}
    for D in SIZES_IN:
        for c in CONCEPTS:
            d = make_preset(D, c)
            out[d.name] = d
    h = make_preset(5, "cf_flexure")
    h = h.model_copy(update=dict(
        name="hakea_5in_hinge_equiv",
        notes="Hakea 5\" reference: skewed lag hinge represented by a soft crossed-blade "
              "flexure at the same hinge axis (skew 45°, hinge r = 10 mm). Placeholder hinge "
              "geometry — set skew/hinge radius to the as-built hub."), deep=True)
    out[h.name] = h
    return out


def scale_design(design: Design, diameter_in: float) -> Design:
    """Geometrically scale a design to a new diameter.

    Lengths (stations' chords, flexure dims, member endpoints, layup insert/skin
    sizes) scale with s = D_new/D_old; angles, r/R stations and materials are
    kept; vehicle mass scales as s^1.5 (see module notes); motor is replaced by
    `motor_for_diameter`; rpm and rpm_grid are reset (re-trim).  Name prefix
    "<D>in_" is updated (else "_<D>in" appended).
    """
    d0 = design.blade.diameter_in
    s = diameter_in / d0
    dm = design.model_dump()

    def sl(lay):
        if lay is None:
            return lay
        lay["insert_diameter"] *= s
        lay["skin_thickness"] *= s
        return lay

    b = dm["blade"]
    b["diameter_in"] = float(diameter_in)
    for st in b["stations"]:
        st["chord"] *= s
        st["layup"] = sl(st["layup"])
    b["layup"] = sl(b["layup"])
    f = dm["flexure"]
    for k in ("hinge_radius", "length", "thickness", "width", "hub_radius"):
        f[k] *= s
    f["layup"] = sl(f["layup"])
    for mem in f["members"] + ([f["tension_member"]] if f["tension_member"] else []):
        mem["p0"] = tuple(np.asarray(mem["p0"]) * s)
        mem["p1"] = tuple(np.asarray(mem["p1"]) * s)
        mem["width"] *= s
        mem["thickness"] *= s
        mem["layup"] = sl(mem["layup"])
    dm["motor"] = motor_for_diameter(diameter_in).model_dump()
    op = dm["op"]
    op["vehicle_mass"] = round(op["vehicle_mass"] * s ** MASS_EXPONENT, 4)
    op["rpm"] = None
    op["rpm_grid"] = []
    name = dm["name"]
    new = re.sub(r"^\d+(\.\d+)?in_", f"{_fmt(diameter_in)}in_", name)
    dm["name"] = new if new != name else f"{name}_{_fmt(diameter_in)}in"
    return Design.model_validate(dm)
