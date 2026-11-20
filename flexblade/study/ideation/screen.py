"""Ideation screen: many root-compliance + blade-shape concepts, Latin-hypercube screened.

Every concept = builder + variables. Variable paths:
  dotted Design path (e.g. flexure.thickness, blade.diameter_in) -> set directly;
  "@name" -> builder/blade-law parameter (see BLADE_LAW and BUILDERS below);
  {"choices": [...]} -> categorical (LHS coordinate mapped to an index).
Build order per sample: base (5" Hakea, 2307 1300KV, 0.6 kg, PA6-CF blade, EA 0.43c,
AM32 duty_slew_max 2550) -> dotted vars -> blade law (@b_*) -> builder -> pulse at motor
1/rev capability (motor.capability(...).dQ_max_1p / trim torque).

Run (from repo root):
  PYTHONPATH=flexblade ~/venv-flexblade/bin/python flexblade/study/ideation/screen.py screen --workers 14
  ... screen.py fea          # coarse FEA of the top-8 kept concepts
  ... screen.py export       # concepts.json (templates + results)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

os.environ.setdefault("FLEXBLADE_WORKER", "1")

import numpy as np  # noqa: E402

from flexblade import api, fact, motor, presets  # noqa: E402
from flexblade.schema import (Airfoil, BladeStation, Design, FlexureMember, Layup,  # noqa: E402
                              PrintOrientation)

OUT = Path(__file__).resolve().parent
WHAT = ["trim", "modes", "harmonic", "checks"]

# --------------------------------------------------------------------------- base design


def base_design() -> Design:
    d = presets.make_preset(5, "tpu_rod").model_copy(deep=True)
    d.motor = motor.get_motor("tmotor_2307_1300kv")
    d.motor.duty_slew_max = 2550.0
    d.op.vehicle_mass = 0.6
    d.blade.layup = Layup(kind="solid", material="pa6_cf")
    d.blade.elastic_axis_chord_frac = 0.43
    d.name = "ideation_base"
    return d


# --------------------------------------------------------------------------- blade law

BLADE_LAW = dict(  # defaults reproduce presets.hover_blade_stations
    b_c_root=None,      # chord/R at the root cut-out (None = preset law 0.30-0.12x)
    b_taper=None,       # c_tip / c_root (None = preset law)
    b_alpha=8.0,        # twist = alpha + atan(lam / x) + 0
    b_lam=0.12,
    b_camber=4,         # NACA 4-digit max camber, % (integer-rounded)
    b_tc=0.12,          # tip t/c
    b_tc_root=0.12,     # root t/c (linear in x)
)


def apply_blade_law(d: Design, p: dict) -> None:
    q = {**BLADE_LAW, **{k: v for k, v in p.items() if k.startswith("b_")}}
    b = d.blade
    R = b.radius
    xr = b.root_cutout_frac
    xs = np.linspace(xr, 1.0, 12)
    if q["b_c_root"] is None and q["b_taper"] is None:
        chord = R * (0.30 - 0.12 * xs)
    else:
        c0 = q["b_c_root"] if q["b_c_root"] is not None else 0.30 - 0.12 * xr
        tr = q["b_taper"] if q["b_taper"] is not None else (0.18 / (0.30 - 0.12 * xr))
        chord = R * c0 * (1 + (tr - 1) * (xs - xr) / (1 - xr))
    twist = q["b_alpha"] + np.degrees(np.arctan(q["b_lam"] / xs))
    tc = q["b_tc"] + (q["b_tc_root"] - q["b_tc"]) * (1 - xs) / (1 - xr)
    cam = int(round(q["b_camber"]))
    b.airfoil = Airfoil(kind="naca4", naca=f"{cam}{4 if cam else 0}{int(round(100 * q['b_tc'])):02d}")
    old = {round(s.r_frac, 6): s.layup for s in b.stations}
    b.stations = [BladeStation(r_frac=float(x), chord=float(c), twist_deg=float(t),
                               thickness_frac=float(tt), layup=None)
                  for x, c, t, tt in zip(xs, chord, twist, tc)]


# --------------------------------------------------------------------------- helpers


def _frame(d):
    return fact._frame(d)                      # H0, a, xp, n


def fact_plate(d: Design, layup: Layup, topology="skewed_single_blade", **kw) -> list[FlexureMember]:
    t = d.model_copy(deep=True)
    t.flexure.topology = topology
    t.flexure.tension_rod_diameter = 0.0
    t.flexure.tension_member = None
    t.flexure.layup = layup
    for k, v in kw.items():
        setattr(t.flexure, k, v)
    return fact.generate_members(t)


def wire(p0, p1, dia, mat, wdir=(0, 0, 1)) -> FlexureMember:
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    l = (p1 - p0) / np.linalg.norm(p1 - p0)
    b = np.cross(l, [0, 0, 1.0])
    b = b if np.linalg.norm(b) > 1e-6 else np.array([0, 1.0, 0])
    return FlexureMember(kind="wire", p0=tuple(map(float, p0)), p1=tuple(map(float, p1)),
                         width=dia, thickness=dia, width_dir=tuple(map(float, wdir)), shape="circle",
                         layup=Layup(kind="solid", material=mat),
                         orientation=PrintOrientation(build_dir=tuple(map(float, b / np.linalg.norm(b))),
                                                      raster_dir=tuple(map(float, l))))


def rod_layup(mat_base, ins_mat, dia, spread):
    pos = [0.5] if spread < 0.02 else [0.5 - spread, 0.5 + spread]
    return Layup(kind="rod_insert", material=mat_base, insert_material=ins_mat,
                 insert_diameter=dia, insert_positions=pos)


# --------------------------------------------------------------------------- builders


def b_flex_only(d, p):
    """Dotted paths already set everything (named topology)."""
    if "tpu" in p:
        d.flexure.layup = Layup(kind="solid", material=p["tpu"])
    if p.get("bar_mat"):
        d.flexure.bar_material = p["bar_mat"]
        d.flexure.tension_rod_material = p.get("strap_mat", p["bar_mat"])
    if p.get("strap_mat"):
        d.flexure.tension_rod_material = p["strap_mat"]
    if p.get("wire_mat"):
        d.flexure.layup = Layup(kind="solid", material=p["wire_mat"])
    if p.get("plate_mat"):
        d.flexure.layup = Layup(kind="solid", material=p["plate_mat"])


def b_rod_plate(d, p):
    """TPU plate with embedded rods (current best family)."""
    f = d.flexure
    f.topology = p.get("topology", "skewed_single_blade")
    f.layup = rod_layup(p.get("tpu", "tpu_95a"), p.get("ins_mat", "cf_rod_pultruded"),
                        min(p.get("ins_d", 0.0016), 0.8 * f.thickness), p.get("spread", 0.2))
    f.tension_rod_diameter = p.get("strap_d", 0.0)


def b_laminate(d, p):
    """Fred: thin laminate on the bending neutral plane inside a TPU plate."""
    f = d.flexure
    f.topology = "custom"
    tpu = fact_plate(d, Layup(kind="solid", material=p.get("tpu", "tpu_95a")))
    lam = [m.model_copy(update=dict(thickness=p["lam_t"], width=f.width * p.get("lam_wfrac", 0.8),
                                    layup=Layup(kind="solid", material=p["lam_mat"])))
           for m in tpu]
    f.members = tpu + lam
    f.tension_rod_diameter = 0.0


def b_through_rod(d, p):
    """Fred: continuous rod from the hub centre (clamped/embedded) through the hinge point
    into the blade end; TPU plate is the hinge filler/damper."""
    f = d.flexure
    f.topology = "custom"
    tpu = fact_plate(d, Layup(kind="solid", material=p.get("tpu", "tpu_95a")))
    x1 = max(max(m.p0[0], m.p1[0]) for m in tpu)
    x0 = p.get("rod_x0_frac", 0.0) * f.hub_radius
    f.members = tpu + [wire((x0, 0, 0), (x1, 0, 0), p["rod_d"], p.get("rod_mat", "cf_rod_pultruded"))]
    f.tension_rod_diameter = 0.0


def b_integral_graded(d, p):
    """Fred: whole propeller as one graded part: PA6-CF blade with a CF rod running from the
    blade into a thinned TPU root (integral taper) -> rod continuous into the rigid blade."""
    f = d.flexure
    f.topology = "integral_taper"
    f.layup = rod_layup(p.get("tpu", "tpu_95a"), "cf_rod_pultruded",
                        min(p["ins_d"], 0.8 * f.thickness), p.get("spread", 0.2))
    d.blade.layup = Layup(kind="rod_insert", material="pa6_cf", insert_material="cf_rod_pultruded",
                          insert_diameter=min(p["ins_d"], 0.0015), insert_positions=[0.43])


def b_trapezoid(d, p):
    """Isosceles-trapezoid / remote-centre pivot: two plates whose planes contain the skewed
    hinge axis but which meet it only virtually, outboard of the plates."""
    f = d.flexure
    H0, a, xp, n = _frame(d)
    g = math.radians(p["half_angle_deg"])
    y = np.array([0, 1.0, 0])
    L = f.length
    mats = p.get("plate", "pa6_cf")
    lay = (Layup(kind="solid", material="pa6_cf") if mats == "pa6_cf"
           else rod_layup("tpu_95a", "cf_rod_pultruded", min(p.get("ins_d", 0.001), 0.8 * f.thickness), 0.0))
    mem = []
    for sg in (1, -1):
        l = math.cos(g) * xp + sg * math.sin(g) * y
        l = l - a * (a @ l)
        l /= np.linalg.norm(l)
        c = H0 - p["remote"] * l                 # plate centre inboard of the virtual pivot
        p0, p1 = c - 0.5 * L * l, c + 0.5 * L * l
        if p0[0] < f.hub_radius:                  # keep the hub end outside the hub
            sh = (f.hub_radius - p0[0] + 1e-4) / l[0]
            p0, p1 = p0 + sh * l, p1 + sh * l
        mem.append(FlexureMember(kind="blade", p0=tuple(map(float, p0)), p1=tuple(map(float, p1)),
                                 width=f.width, thickness=f.thickness, width_dir=tuple(map(float, a)),
                                 layup=lay))
    f.topology = "custom"
    f.members = mem
    f.tension_rod_diameter = p.get("strap_d", 0.0)


def b_flexbeam_link(d, p):
    """Helicopter bearingless hub: radial flex-beam soft in lag (thin in y), plus an inclined
    pitch link at a vertical offset; lag rotates the blade about the link -> pitch (δ3-like
    coupling by geometry instead of a skewed hinge axis)."""
    f = d.flexure
    x0 = f.hub_radius
    x1 = f.hinge_radius + 0.5 * f.length
    beam = FlexureMember(kind="blade", p0=(x0, 0, 0), p1=(x1, 0, 0), width=f.width, thickness=f.thickness,
                         width_dir=(0, 0, 1), layup=Layup(kind="solid", material=p.get("beam_mat", "pa6_cf")),
                         orientation=PrintOrientation(build_dir=(0, 0, 1), raster_dir=(1, 0, 0)))
    zl, dy = p["link_z"], p["link_dy"]
    link = wire((x0, -0.5 * dy, zl), (x1, 0.5 * dy, zl), p["link_d"], p.get("link_mat", "cf_rod_pultruded"))
    f.topology = "custom"
    f.members = [beam, link]
    f.tension_rod_diameter = 0.0


def b_xwire(d, p):
    """Insect-vein hinge: TPU pad with two CF wires crossing at the hinge point in the plate
    plane (each wire line meets the hinge axis -> FACT-reciprocal), wires carry the pull."""
    f = d.flexure
    H0, a, xp, n = _frame(d)
    tpu = fact_plate(d, Layup(kind="solid", material=p.get("tpu", "tpu_95a")))
    g = math.radians(p["x_angle_deg"])
    L = f.length
    ws = []
    for sg in (1, -1):
        l = math.cos(g) * xp + sg * math.sin(g) * a
        ws.append(wire(H0 - 0.5 * L * l, H0 + 0.5 * L * l, p["wire_d"], p.get("wire_mat", "cf_rod_pultruded"),
                       wdir=tuple(n)))
    f.topology = "custom"
    f.members = tpu + ws
    f.tension_rod_diameter = 0.0


def b_raster(d, p):
    """Anisotropic raster-driven compliance: thin PA6-CF plate printed with its fibre/raster
    at angle psi inside the plate plane (stiff along the fibre, soft across)."""
    f = d.flexure
    H0, a, xp, n = _frame(d)
    psi = math.radians(p["psi_deg"])
    r = math.cos(psi) * xp + math.sin(psi) * a
    f.topology = "skewed_single_blade"
    f.layup = Layup(kind="solid", material=p.get("plate_mat", "pa6_cf"))
    f.orientation = PrintOrientation(build_dir=tuple(map(float, np.cross(xp, a))), raster_dir=tuple(map(float, r)))
    f.tension_rod_diameter = p.get("strap_d", 0.0)


def b_le_mass(d, p):
    """Lag-inertia / mass-balance tuning: steel wire insert near the leading edge of the outer
    blade stations (rod_insert in station layups)."""
    b_rod_plate(d, p)
    lay = Layup(kind="rod_insert", material="pa6_cf", insert_material="steel_302_spring",
                insert_diameter=p["mass_d"], insert_positions=[p["mass_pos"]])
    for s in d.blade.stations:
        if s.r_frac >= p["mass_r0"]:
            s.layup = lay


BUILDERS = dict(flex_only=b_flex_only, rod_plate=b_rod_plate, laminate=b_laminate,
                through_rod=b_through_rod, integral_graded=b_integral_graded, trapezoid=b_trapezoid,
                flexbeam_link=b_flexbeam_link, xwire=b_xwire, raster=b_raster, le_mass=b_le_mass)

# --------------------------------------------------------------------------- concepts

V = lambda path, lo, hi: dict(path=path, lo=lo, hi=hi)          # noqa: E731
C = lambda path, choices: dict(path=path, choices=choices)       # noqa: E731
SKEW = V("flexure.skew_deg", 30, 60)
HR = V("flexure.hinge_radius", 0.008, 0.014)
ROD_ROOT = dict(tpu="tpu_95a", ins_mat="cf_rod_pultruded", ins_d=0.0016, spread=0.2)
ROD_FIX = {"flexure.skew_deg": 45.0, "flexure.hinge_radius": 0.010, "flexure.length": 0.008,
           "flexure.thickness": 0.0025, "flexure.width": 0.010}

# fixed dotted defaults per concept go in "fixed"; params (builder @) defaults in "params"
CONCEPTS: list[dict] = [
    # ---- current best family / Fred seeds
    dict(id="C01", name="TPU plate + embedded CF rods (current best, reference)", persp="baseline",
         fred=False, builder="rod_plate", params=dict(ROD_ROOT),
         variables=[SKEW, HR, V("flexure.length", 0.005, 0.012), V("flexure.thickness", 0.0015, 0.0035),
                    V("flexure.width", 0.007, 0.013), V("@ins_d", 0.0008, 0.002), V("@spread", 0.0, 0.3),
                    C("@tpu", ["tpu_95a", "tpu_64d"])]),
    dict(id="C02", name="Neutral-axis laminate in TPU (CF / spring-steel shim)", persp="Fred / materials",
         fred=True, builder="laminate", params=dict(tpu="tpu_95a", lam_mat="cf_rod_pultruded", lam_t=0.0003),
         variables=[SKEW, HR, V("flexure.length", 0.005, 0.012), V("flexure.thickness", 0.0015, 0.0035),
                    V("flexure.width", 0.006, 0.014), V("@lam_t", 0.0001, 0.0006), V("@lam_wfrac", 0.4, 0.9),
                    C("@lam_mat", ["cf_rod_pultruded", "steel_302_spring"])]),
    dict(id="C03", name="Torsion bar on skewed axis + TPU plate + radial strap", persp="Fred / compliant mech",
         fred=True, builder="flex_only", params=dict(tpu="tpu_95a"),
         fixed={"flexure.topology": "torsion_bar", "flexure.bar_material": "cf_rod_pultruded"},
         variables=[SKEW, HR, V("flexure.bar_diameter", 0.001, 0.0025), V("flexure.bar_length", 0.008, 0.02),
                    V("flexure.tension_rod_diameter", 0.0, 0.0012), V("flexure.thickness", 0.001, 0.002),
                    C("@tpu", ["tpu_95a", "tpu_64d"])]),
    dict(id="C04", name="Continuous rod hub-centre -> blade (embedded, capped) + TPU filler", persp="Fred",
         fred=True, builder="through_rod", params=dict(tpu="tpu_95a", rod_d=0.0012, rod_mat="cf_rod_pultruded"),
         variables=[SKEW, HR, V("flexure.length", 0.005, 0.012), V("flexure.thickness", 0.0015, 0.0035),
                    V("@rod_d", 0.0006, 0.0018), V("@rod_x0_frac", 0.25, 1.0),
                    C("@rod_mat", ["cf_rod_pultruded", "steel_302_spring"])]),
    dict(id="C05", name="Whole propeller one graded part (integral TPU root, CF rod into PA6-CF blade)",
         persp="Fred / materials", fred=True, builder="integral_graded", params=dict(tpu="tpu_95a", ins_d=0.0012),
         variables=[SKEW, HR, V("flexure.thickness", 0.0015, 0.0035), V("flexure.taper", 0.5, 1.0),
                    V("flexure.width", 0.007, 0.013), V("@ins_d", 0.0008, 0.002), V("@spread", 0.0, 0.3)]),
    dict(id="C06", name="Fully FDM rod plate (PA6-CF / PETG-CF printed 'rods', 0 pauses)", persp="Fred / print",
         fred=True, builder="rod_plate", params=dict(tpu="tpu_95a", ins_mat="pa6_cf", ins_d=0.0016, spread=0.2),
         variables=[SKEW, HR, V("flexure.length", 0.005, 0.012), V("flexure.thickness", 0.0015, 0.0035),
                    V("flexure.width", 0.007, 0.013), V("@ins_d", 0.0008, 0.0028), V("@spread", 0.0, 0.3),
                    C("@ins_mat", ["pa6_cf", "petg_cf"])]),
    dict(id="C07", name="Fully FDM torsion bar (PA6-CF bar + PA6-CF strap)", persp="Fred / print",
         fred=True, builder="flex_only", params=dict(tpu="tpu_95a", bar_mat="pa6_cf", strap_mat="pa6_cf"),
         fixed={"flexure.topology": "torsion_bar"},
         variables=[SKEW, HR, V("flexure.bar_diameter", 0.0012, 0.004), V("flexure.bar_length", 0.008, 0.02),
                    V("flexure.tension_rod_diameter", 0.0, 0.002), V("flexure.thickness", 0.001, 0.002)]),
    # ---- compliant-mechanism designer
    dict(id="C08", name="Symmetric butterfly-like crossed PA6-CF plates (Haringx sym)", persp="compliant mech",
         fred=False, builder="flex_only", params=dict(plate_mat="pa6_cf"),
         fixed={"flexure.topology": "skewed_cross_blade_sym", "flexure.cross_gap": 0.001},
         variables=[SKEW, HR, V("flexure.length", 0.006, 0.014), V("flexure.thickness", 0.0004, 0.0012),
                    V("flexure.width", 0.006, 0.014), V("flexure.cross_angle_deg", 60, 120),
                    V("flexure.tension_rod_diameter", 0.0, 0.001)]),
    dict(id="C09", name="Cross-axis / cartwheel pivot in TPU with embedded CF rods", persp="compliant mech",
         fred=False, builder="rod_plate", params=dict(ROD_ROOT, topology="skewed_cross_blade"),
         variables=[SKEW, HR, V("flexure.length", 0.006, 0.014), V("flexure.thickness", 0.0015, 0.003),
                    V("flexure.width", 0.005, 0.01), V("flexure.cross_angle_deg", 60, 120),
                    V("@ins_d", 0.0006, 0.0014), V("@spread", 0.0, 0.25)]),
    dict(id="C10", name="FACT wire set (5 wires through the hinge axis), CF rod or spring steel",
         persp="compliant mech", fred=False, builder="flex_only", params=dict(wire_mat="cf_rod_pultruded"),
         fixed={"flexure.topology": "wire_triplet"},
         variables=[SKEW, HR, V("flexure.length", 0.006, 0.016), V("flexure.thickness", 0.0003, 0.0012),
                    V("flexure.width", 0.004, 0.012), V("flexure.cross_angle_deg", 40, 120),
                    C("@wire_mat", ["cf_rod_pultruded", "steel_302_spring"])]),
    dict(id="C11", name="Isosceles-trapezoid / remote-centre pivot (LITF-like), virtual hinge outboard",
         persp="compliant mech", fred=False, builder="trapezoid", params=dict(half_angle_deg=30, remote=0.006,
                                                                               plate="pa6_cf"),
         variables=[SKEW, V("flexure.hinge_radius", 0.012, 0.022), V("flexure.length", 0.004, 0.01),
                    V("flexure.thickness", 0.0004, 0.0012), V("flexure.width", 0.006, 0.012),
                    V("@half_angle_deg", 10, 40), V("@remote", 0.002, 0.008), V("@strap_d", 0.0, 0.001),
                    C("@plate", ["pa6_cf", "tpu_rod"])]),
    dict(id="C12", name="Single TPU plate + radial CF strap through hinge point (split path)",
         persp="compliant mech", fred=False, builder="flex_only", params=dict(tpu="tpu_64d"),
         fixed={"flexure.topology": "skewed_single_blade"},
         variables=[SKEW, HR, V("flexure.length", 0.005, 0.012), V("flexure.thickness", 0.0015, 0.004),
                    V("flexure.width", 0.006, 0.013), V("flexure.tension_rod_diameter", 0.0005, 0.0016),
                    C("@tpu", ["tpu_95a", "tpu_64d"])]),
    # ---- helicopter rotor engineer
    dict(id="C13", name="Bearingless flex-beam (lag-soft) + inclined pitch link (geometric pitch-lag)",
         persp="rotor engineer", fred=False, builder="flexbeam_link",
         params=dict(link_z=0.003, link_dy=0.003, link_d=0.0008, beam_mat="pa6_cf"),
         fixed={"flexure.width": 0.006},
         variables=[V("flexure.hinge_radius", 0.009, 0.016), V("flexure.length", 0.006, 0.016),
                    V("flexure.thickness", 0.0004, 0.0012), V("flexure.width", 0.003, 0.008),
                    V("@link_z", -0.005, 0.005), V("@link_dy", -0.006, 0.006), V("@link_d", 0.0004, 0.0012)]),
    dict(id="C14", name="Elastomeric-style thick TPU shear pad + CF tension-torsion strap",
         persp="rotor engineer", fred=False, builder="flex_only", params=dict(tpu="tpu_95a"),
         fixed={"flexure.topology": "skewed_single_blade"},
         variables=[SKEW, HR, V("flexure.length", 0.002, 0.005), V("flexure.thickness", 0.003, 0.007),
                    V("flexure.width", 0.006, 0.014), V("flexure.tension_rod_diameter", 0.0005, 0.0014)]),
    dict(id="C15", name="Zero/low skew + chordwise attachment offset (coupling via offset, not skew)",
         persp="rotor engineer / critic", fred=False, builder="rod_plate", params=dict(ROD_ROOT),
         fixed=dict(ROD_FIX),
         variables=[V("flexure.skew_deg", 0.0, 20.0), V("blade.elastic_axis_chord_frac", 0.05, 0.6),
                    V("flexure.thickness", 0.0015, 0.0035), V("@ins_d", 0.0008, 0.002)]),
    # ---- materials / print
    dict(id="C16", name="Raster-steered PA6-CF thin plate (anisotropic compliance) + strap",
         persp="materials / print", fred=False, builder="raster", params=dict(psi_deg=0.0, plate_mat="pa6_cf"),
         variables=[SKEW, HR, V("flexure.length", 0.005, 0.014), V("flexure.thickness", 0.0004, 0.0012),
                    V("flexure.width", 0.006, 0.014), V("@psi_deg", 0, 75), V("@strap_d", 0.0, 0.001)]),
    dict(id="C17", name="Spring-steel shim leaf hinge (no TPU), skewed", persp="materials",
         fred=False, builder="flex_only", params=dict(plate_mat="steel_302_spring"),
         fixed={"flexure.topology": "skewed_single_blade"},
         variables=[SKEW, HR, V("flexure.length", 0.004, 0.014), V("flexure.thickness", 0.0001, 0.0006),
                    V("flexure.width", 0.004, 0.014)]),
    # ---- biomimicry
    dict(id="C18", name="Insect-vein X hinge: two crossed CF wires in a TPU pad (resilin analogue)",
         persp="biomimicry", fred=False, builder="xwire", params=dict(x_angle_deg=20, wire_d=0.0008),
         variables=[SKEW, HR, V("flexure.length", 0.005, 0.012), V("flexure.thickness", 0.0015, 0.0035),
                    V("flexure.width", 0.006, 0.014), V("@x_angle_deg", 5, 40), V("@wire_d", 0.0005, 0.0015),
                    C("@wire_mat", ["cf_rod_pultruded", "steel_302_spring"])]),
    dict(id="C19", name="Lag-inertia tuning: steel LE wire in outer blade (inertial 'tab' mass)",
         persp="critic / inertia", fred=False, builder="le_mass",
         params=dict(ROD_ROOT, mass_d=0.001, mass_pos=0.12, mass_r0=0.7), fixed=dict(ROD_FIX),
         variables=[SKEW, V("flexure.thickness", 0.0015, 0.0035), V("@mass_d", 0.0005, 0.0015),
                    V("@mass_pos", 0.08, 0.43), V("@mass_r0", 0.4, 0.9),
                    V("blade.elastic_axis_chord_frac", 0.25, 0.45)]),
    # ---- propeller (blade-shape) concepts on the rod root at fixed root dims
    dict(id="B01", name="Wide-chord low-tip-speed blade (5-7\")", persp="propeller", fred=False,
         builder="rod_plate", params=dict(ROD_ROOT), fixed=dict(ROD_FIX), blade=True,
         variables=[V("blade.diameter_in", 5.0, 7.0), V("@b_c_root", 0.18, 0.40), V("@b_taper", 0.4, 1.0),
                    V("flexure.thickness", 0.0015, 0.0035)]),
    dict(id="B02", name="Inverse taper / tip-heavy chord (more lag inertia per pulse)", persp="propeller",
         fred=False, builder="rod_plate", params=dict(ROD_ROOT), fixed=dict(ROD_FIX), blade=True,
         variables=[V("blade.diameter_in", 5.0, 7.0), V("@b_c_root", 0.12, 0.25), V("@b_taper", 1.0, 1.8),
                    V("flexure.thickness", 0.0015, 0.0035)]),
    dict(id="B03", name="Twist law (alpha, lambda) + collective", persp="propeller", fred=False,
         builder="rod_plate", params=dict(ROD_ROOT), fixed=dict(ROD_FIX), blade=True,
         variables=[V("@b_alpha", 3.0, 13.0), V("@b_lam", 0.05, 0.22), V("blade.diameter_in", 5.0, 7.0),
                    V("flexure.thickness", 0.0015, 0.0035)]),
    dict(id="B04", name="Airfoil camber / thickness (NACA m4tt, root thickening)", persp="propeller",
         fred=False, builder="rod_plate", params=dict(ROD_ROOT), fixed=dict(ROD_FIX), blade=True,
         variables=[V("@b_camber", 0, 6), V("@b_tc", 0.07, 0.15), V("@b_tc_root", 0.10, 0.22),
                    V("blade.diameter_in", 5.0, 7.0), V("flexure.thickness", 0.0015, 0.0035)]),
    dict(id="B05", name="Root cut-out / hinge offset trade", persp="propeller / rotor engineer",
         fred=False, builder="rod_plate", params=dict(ROD_ROOT), fixed=dict(ROD_FIX), blade=True,
         variables=[V("blade.root_cutout_frac", 0.16, 0.32), V("flexure.hinge_radius", 0.008, 0.016),
                    V("flexure.length", 0.005, 0.012), V("blade.diameter_in", 5.0, 7.0),
                    V("flexure.thickness", 0.0015, 0.0035)]),
    dict(id="B06", name="Joint blade + root LHS, 5-7\" (rod root)", persp="propeller", fred=False,
         builder="rod_plate", params=dict(ROD_ROOT), blade=True, n=60,
         variables=[V("blade.diameter_in", 5.0, 7.0), V("@b_c_root", 0.15, 0.38), V("@b_taper", 0.5, 1.4),
                    V("@b_alpha", 4.0, 12.0), V("@b_lam", 0.06, 0.2), V("@b_camber", 2, 6),
                    V("@b_tc", 0.08, 0.14), SKEW, V("flexure.thickness", 0.0015, 0.0035),
                    V("@ins_d", 0.0008, 0.002)]),
    dict(id="B07", name="Joint blade + torsion-bar root, 5-7\"", persp="propeller", fred=False,
         builder="flex_only", params=dict(tpu="tpu_95a"), blade=True, n=60,
         fixed={"flexure.topology": "torsion_bar", "flexure.bar_material": "cf_rod_pultruded",
                "flexure.thickness": 0.0013},
         variables=[V("blade.diameter_in", 5.0, 7.0), V("@b_c_root", 0.15, 0.38), V("@b_taper", 0.5, 1.4),
                    V("@b_alpha", 4.0, 12.0), V("@b_lam", 0.06, 0.2), V("@b_camber", 2, 6), SKEW,
                    V("flexure.bar_diameter", 0.001, 0.0025), V("flexure.bar_length", 0.008, 0.02),
                    V("flexure.tension_rod_diameter", 0.0, 0.0012)]),
]

# concepts the simulator cannot express (needs capability)
NEEDS: list[dict] = [
    dict(id="N01", name="Interlocking vertical teeth / dovetail TPU<->CF joint", persp="Fred / print", fred=True,
         mechanism="Mechanical interlock carries the centrifugal pull across the TPU-rigid interface by bearing "
                   "on teeth instead of adhesion; addresses the root-fitting failure (FEA util 4.8).",
         needs="FEA of joint detail: multi-part solids with interface/contact (or tied) + toothed interface "
               "geometry generator; beam model treats connectors as rigid"),
    dict(id="N02", name="Free-floating blade-to-blade through rod (hub not in the load path)", persp="Fred",
         fred=True, mechanism="One rod runs blade 1 -> hub -> blade 2, only located (not clamped) by the hub, so "
                              "the two centrifugal pulls cancel inside the rod; hub carries ~0 radial load.",
         needs="members connecting blade k to blade j (cross-blade members / non-hub endpoints) in rotor.py; "
               "sliding/pinned hub support"),
    dict(id="N03", name="Graded TPU hardness / functionally graded root (95A hinge -> 64D -> PA6-CF)",
         persp="materials / Fred", fred=True,
         mechanism="Modulus graded along the span so strain concentrates in the hinge zone and stress "
                   "steps at interfaces vanish.", needs="spanwise material grading per member element "
                                                        "(beam) and per-element material in FEA"),
    dict(id="N04", name="FACT seed -> free-form FEA topology optimisation", persp="Fred / method", fred=True,
         mechanism="Start from the best FACT layout and let density-based TO redistribute material under "
                   "centrifugal + 1/rev loads.", needs="FEA-in-the-loop harmonic objective (in progress in the "
                                                        "free-form optimiser agent)"),
    dict(id="N05", name="Butterfly pivot / two-stage (series) flexure", persp="compliant mech", fred=False,
         mechanism="Two flexure stages in series through an intermediate rigid body: one stage gives lag "
                   "compliance, the other carries pull / sets the skewed axis; large stroke, low axis drift.",
         needs="member chains with intermediate rigid bodies (node sharing between members) in rotor/fact"),
    dict(id="N06", name="Swept blade (pitch-lag via sweep/offset of the blade line)", persp="rotor engineer",
         fred=False, mechanism="Aft/forward sweep moves the blade CG/aero centre off the lag hinge line so lag "
                               "and centrifugal force produce pitch moment; no skewed hinge needed.",
         needs="sweep field per BladeStation (beam line offset in y) in schema/bladegeom/rotor/geometry"),
    dict(id="N07", name="Aeroelastic bend-twist coupled blade (off-axis fibres), no root pitch hinge",
         persp="critic", fred=False,
         mechanism="Lag bending of an anisotropic blade twists it (EI-GJ coupling from off-axis fibre/raster); "
                   "root only needs lag compliance.", needs="coupled section stiffness (bend-twist terms) in "
                                                             "section.py/beam.py"),
    dict(id="N08", name="Lattice / auxetic metamaterial hinge block", persp="materials", fred=False,
         mechanism="Printed lattice with tailored anisotropic homogenised stiffness: soft about the skewed axis, "
                   "stiff radially; auxetic cells resist necking.", needs="homogenised lattice materials + "
                                                                           "lattice geometry export"),
    dict(id="N09", name="Continuous-fibre steered loops (CF/aramid/GF filament)", persp="materials", fred=False,
         mechanism="Continuous fibre wrapped as a loop around hub pin and blade root carries pull in pure "
                   "tension; TPU/PA hinge filler.", needs="continuous-fibre, aramid and GF materials; curved "
                                                          "member paths"),
    dict(id="N10", name="Tuned inertial tab / pendulum on the blade root", persp="critic", fred=False,
         mechanism="A small mass on a compliant tab is driven by hub angular acceleration and pushes pitch "
                   "directly (amplifies response near its own resonance).",
         needs="lumped masses/inertias attachable to members or blade stations"),
    dict(id="N11", name="Asymmetric chordwise mass (speed-sensitive pitch, maple-seed-like)", persp="biomimicry",
         fred=False, mechanism="Blades with opposite chordwise CG offsets pitch in opposite senses with Omega "
                               "oscillation (propeller moment), giving differential 1/rev pitch without lag.",
         needs="per-blade (non-identical) blade definitions in rotor.py"),
    dict(id="N12", name="Corrugated / pleated resilin-like hinge", persp="biomimicry", fred=False,
         mechanism="Pleats lengthen the effective bending length (low k) while the pull runs straight through a "
                   "stiff vein.", needs="curved/corrugated member geometry + FEA"),
]


# --------------------------------------------------------------------------- build / evaluate


def set_path(obj, path, val):
    parts = path.split(".")
    for k in parts[:-1]:
        obj = obj[int(k)] if k.isdigit() else getattr(obj, k)
    setattr(obj, parts[-1], val)


def build(concept: dict, x: dict) -> Design:
    d = base_design()
    d.name = f"{concept['id']}"
    for k, v in (concept.get("fixed") or {}).items():
        set_path(d, k, v)
    params = dict(concept.get("params") or {})
    for k, v in x.items():
        if k.startswith("@"):
            params[k[1:]] = v
        else:
            set_path(d, k, v)
    apply_blade_law(d, params)
    BUILDERS[concept["builder"]](d, params)
    return d


def set_pulse(d: Design) -> Design:
    r0 = api.evaluate(d, ["trim"])
    cap = motor.capability(d.motor, r0.trim.rpm, r0.trim.torque)
    d.op.torque_mod_frac = float(cap.dQ_max_1p / r0.trim.torque)
    return d


def lhs(concept: dict, n: int, seed: int) -> list[dict]:
    from scipy.stats import qmc
    vs = concept["variables"]
    u = qmc.LatinHypercube(d=len(vs), seed=seed).random(n)
    out = []
    for row in u:
        x = {}
        for v, ui in zip(vs, row):
            if "choices" in v:
                x[v["path"]] = v["choices"][min(int(ui * len(v["choices"])), len(v["choices"]) - 1)]
            else:
                x[v["path"]] = float(v["lo"] + ui * (v["hi"] - v["lo"]))
        out.append(x)
    return out


def summarise(res) -> dict:
    h = res.harmonic
    by = {}
    for c in res.checks:
        by[c.name] = max(by.get(c.name, 0.0), c.utilisation)
    worst = max(res.checks, key=lambda c: c.utilisation) if res.checks else None
    s = res.summary
    return dict(
        control_ratio=h.control_ratio if h else None, tilt_deg=h.tilt_equiv_deg if h else None,
        pitch_amp_deg=s.get("pitch_amp_deg"), lag_amp_deg=s.get("lag_amp_deg"),
        lag_per_rev=s.get("lag_freq_per_rev"), flap_per_rev=s.get("flap_freq_per_rev"),
        pitch_per_rev=s.get("pitch_freq_per_rev"), pitch_lag_coupling=s.get("pitch_lag_coupling"),
        split_deg=s.get("trim_diff_pitch_deg"), rpm=s.get("rpm"), root_cf=s.get("root_cf"),
        fom=res.trim.figure_of_merit if res.trim else None, power=res.trim.power if res.trim else None,
        max_util=worst.utilisation if worst else None,
        max_struct_util=s.get("max_structural_utilisation"),
        governing=f"{worst.name}:{worst.member}" if worst else "",
        util_by_check=by,
        converged=not any("not converged" in w for w in res.warnings),
    )


def run_one(job):
    cid, i, x = job
    concept = next(c for c in CONCEPTS if c["id"] == cid)
    t0 = time.time()
    try:
        d = set_pulse(build(concept, x))
        res = api.evaluate(d, WHAT)
        row = summarise(res)
        row["ok"] = True
        row["pulse_frac"] = d.op.torque_mod_frac
    except Exception as e:  # noqa: BLE001
        row = dict(ok=False, error=f"{type(e).__name__}: {str(e)[:300]}")
    row.update(id=cid, i=i, x=x, t=time.time() - t0)
    return row


def feasible(r) -> bool:
    return bool(r.get("ok") and r.get("converged") and (r.get("max_util") or 9e9) <= 1.0
                and abs(r.get("split_deg") or 0.0) <= 2.0 and (r.get("control_ratio") or 0) > 0)


def cmd_screen(a):
    path = OUT / "screen_results.jsonl"
    done = set()
    if path.exists():
        for line in path.read_text().splitlines():
            r = json.loads(line)
            done.add((r["id"], r["i"]))
    ids = a.only.split(",") if a.only else [c["id"] for c in CONCEPTS]
    jobs = []
    for c in CONCEPTS:
        if c["id"] not in ids:
            continue
        for i, x in enumerate(lhs(c, c.get("n", a.n), seed=a.seed)):
            if (c["id"], i) not in done:
                jobs.append((c["id"], i, x))
    print(f"{len(jobs)} jobs", flush=True)
    t0 = time.time()
    with open(path, "a") as fh, ProcessPoolExecutor(a.workers) as ex:
        futs = [ex.submit(run_one, j) for j in jobs]
        for n, f in enumerate(as_completed(futs), 1):
            r = f.result()
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            if n % 50 == 0:
                print(f"{n}/{len(jobs)} {time.time() - t0:.0f}s", flush=True)
    print(f"done {time.time() - t0:.0f}s")


def _read_results():
    """screen_results.jsonl (live) or the committed screen_results.jsonl.gz."""
    import gzip
    p = OUT / "screen_results.jsonl"
    txt = p.read_text() if p.exists() else gzip.open(OUT / "screen_results.jsonl.gz", "rt").read()
    return txt.splitlines()


def load_rows():
    rows = [json.loads(l) for l in _read_results()]
    by = {}
    for r in rows:
        by.setdefault(r["id"], []).append(r)
    return by


def best_of(rows):
    feas = [r for r in rows if feasible(r)]
    if feas:
        return max(feas, key=lambda r: r["control_ratio"]), len(feas)
    ok = [r for r in rows if r.get("ok")]
    return (min(ok, key=lambda r: r.get("max_util") or 9e9) if ok else None), 0


def _fea_one(job):
    cid, x = job
    from flexblade import fea
    concept = next(c for c in CONCEPTS if c["id"] == cid)
    try:
        d = set_pulse(build(concept, x))
        r = fea.static_centrifugal(d, coarse=True, field=False)
        parts = {k: dict(material=v["material"], util_principal=v["util_principal_vs_Xt"],
                         util_interlayer=v["util_interlayer_vs_Zt"])
                 for k, v in r["parts"].items()}
        mx = max(max(v["util_principal"], v["util_interlayer"]) for v in parts.values())
        # pultruded rods / steel have no print layers: interlayer check is not meaningful there
        mx_print = max(max(v["util_principal"], 0 if v["material"] in ("cf_rod_pultruded", "steel_302_spring")
                           else v["util_interlayer"]) for v in parts.values())
        return dict(id=cid, ok=True, parts=parts, max_util=mx, max_util_printlayers=mx_print,
                    radial_stretch_tip=r.get("radial_stretch_tip"), warnings=r.get("warnings"))
    except Exception as e:  # noqa: BLE001
        return dict(id=cid, ok=False, error=f"{type(e).__name__}: {str(e)[:300]}",
                    tb=traceback.format_exc()[-800:])


def cmd_fea(a):
    by = load_rows()
    ranked = []
    for cid, rows in by.items():
        b, nf = best_of(rows)
        if nf:
            ranked.append((b["control_ratio"], cid, b["x"]))
    ranked.sort(reverse=True)
    jobs = [(cid, x) for _, cid, x in ranked[:a.top]]
    extra = [e for e in a.extra.split(",") if e]
    jobs += [(cid, x) for _, cid, x in ranked if cid in extra and cid not in {j[0] for j in jobs}]
    # one subprocess per FEA (gmsh/solver crashes must not kill the batch); modest parallelism
    import subprocess
    from concurrent.futures import ThreadPoolExecutor

    def run(job):
        p = subprocess.run([sys.executable, __file__, "fea1", "--job", json.dumps(job)],
                           capture_output=True, text=True, timeout=1800)
        line = [l for l in p.stdout.splitlines() if l.startswith("{")]
        return json.loads(line[-1]) if line else dict(id=job[0], ok=False,
                                                      error=f"exit {p.returncode}: {p.stderr[-300:]}")
    out = []
    with ThreadPoolExecutor(a.workers) as ex:
        for r in ex.map(run, jobs):
            print(r["id"], r.get("max_util"), r.get("max_util_printlayers"), r.get("error", "")[:200], flush=True)
            out.append(r)
    (OUT / "fea_results.json").write_text(json.dumps(out, indent=1))


def cmd_fea1(a):
    print(json.dumps(_fea_one(tuple(json.loads(a.job)))))


def cmd_export(a):
    by = load_rows()
    fea_res = {r["id"]: r for r in json.loads((OUT / "fea_results.json").read_text())} \
        if (OUT / "fea_results.json").exists() else {}
    verdicts = json.loads((OUT / "verdicts.json").read_text()) if (OUT / "verdicts.json").exists() else {}
    out = []
    for c in CONCEPTS:
        rows = by.get(c["id"], [])
        b, nf = best_of(rows)
        mid = {v["path"]: (v["choices"][0] if "choices" in v else 0.5 * (v["lo"] + v["hi"]))
               for v in c["variables"]}
        try:
            base = build(c, mid).model_dump(mode="json")
        except Exception as e:  # noqa: BLE001
            base = {"error": str(e)}
        best_design = set_pulse(build(c, b["x"])).model_dump(mode="json") if (b and nf) else None
        fe = sorted([r for r in rows if feasible(r)], key=lambda r: -r["control_ratio"])
        out.append(dict(
            id=c["id"], name=c["name"], perspective=c["persp"], fred_seed=c["fred"],
            expressible=True, template=dict(builder=c["builder"], params=c.get("params"), fixed=c.get("fixed"),
                                            base=base, variables=c["variables"],
                                            how="flexblade/study/ideation/screen.py build(concept, x)"),
            screening=dict(n=len(rows), n_ok=sum(1 for r in rows if r.get("ok")), n_feasible=nf,
                           best=({k: b.get(k) for k in ("control_ratio", "tilt_deg", "pitch_amp_deg",
                                                         "lag_amp_deg", "lag_per_rev", "flap_per_rev",
                                                         "pitch_per_rev", "pitch_lag_coupling", "split_deg",
                                                         "rpm", "root_cf", "fom", "power", "max_util",
                                                         "governing", "util_by_check", "x")} if b else None),
                           best_is_feasible=bool(nf),
                           top5_control=[round(r["control_ratio"], 4) for r in fe[:5]],
                           errors=sorted({r.get("error", "")[:120] for r in rows if not r.get("ok")})[:3]),
            best_design=best_design, fea=fea_res.get(c["id"]),
            verdict=verdicts.get(c["id"], {}).get("verdict"), why=verdicts.get(c["id"], {}).get("why"),
            cfd_candidate=bool(verdicts.get(c["id"], {}).get("cfd_candidate", False)),
        ))
    for n in NEEDS:
        out.append(dict(n, expressible=False, verdict="NEEDS-CAPABILITY", cfd_candidate=False))
    (OUT / "concepts.json").write_text(json.dumps(out, indent=1))
    print(f"wrote {len(out)} concepts")


def cmd_table(a):
    by = load_rows()
    for c in CONCEPTS:
        rows = by.get(c["id"], [])
        b, nf = best_of(rows)
        nok = sum(1 for r in rows if r.get("ok"))
        if b is None:
            errs = {r.get('error', '')[:100] for r in rows}
            print(f"{c['id']:4s} n={len(rows)} ok=0 ERR {errs}")
            continue
        print(f"{c['id']:4s} n={len(rows)} ok={nok} feas={nf:2d} ctrl={b['control_ratio'] or 0:.3f} "
              f"pitch={b['pitch_amp_deg'] or 0:5.2f} lag/rev={b['lag_per_rev'] or 0:.2f} "
              f"util={b['max_util'] or 0:.2f} {b['governing']} split={b['split_deg'] or 0:+.2f} "
              f"rpm={b['rpm'] or 0:.0f} fom={b['fom'] or 0:.2f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["screen", "fea", "fea1", "export", "table"])
    ap.add_argument("--n", type=int, default=48)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--workers", type=int, default=14)
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--only", default="")
    ap.add_argument("--job", default="")
    ap.add_argument("--extra", default="C01,C18,C03,C10,C09,C15,C17,C08",
                    help="fea: also run these concepts' best points (root-family diversity)")
    a = ap.parse_args()
    dict(screen=cmd_screen, fea=cmd_fea, fea1=cmd_fea1, export=cmd_export, table=cmd_table)[a.cmd](a)
