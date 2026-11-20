"""Vehicle sizing layer: battery, mass budget, hover power/endurance, motor headroom.

Every number below is a *typical / documented-assumption* value (±20-50 %), not a
measurement of Hakea hardware.  Sources are in the comments next to each table.

Chain (hover):  battery -> ESC(+wiring, R_eff) -> motor -> rotor.
    P_batt = sum_i [ w_i (Q_i + b w_i + T0) + R I_i^2 (1 + cyc) ] (1 + ESC_SW) + ESC idle + avionics
    I_i = (Q_i + b w_i + T0)/Kt ;  cyc = 0.5 ctrl_mod_typ^2 (extra copper of the 1/rev pulse,
    DeltaI = ctrl_mod_typ I0, mean of DeltaI^2/2 over a revolution).
Battery: V = S OCV(soc) - I R_pack, constant-power discharge integrated in soc.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .materials import get_material
from .motor import DUTY_MAX, capability, get_motor, kt
from .schema import (BatterySpec, Design, Layup, Motor, VehicleSpec)

G0 = 9.80665

# ------------------------------------------------------------------------------ cells
# wh_kg: cell-level specific energy. ir: DC internal resistance per cell (lipo: mOhm*Ah /1000,
# i.e. R = ir/Ah per pouch cell). imax: continuous A per cell (lipo: C rate). usable: fraction of
# capacity drawn before landing reserve. overhead: wiring/BMS/holders/shrink mass over cells.
# Li-ion: Molicel INR21700-P45B: 4.5 Ah typ, 69-70 g max, 45 A continuous, DC impedance ~15 mOhm
# at 50 % SoC (AC ~7 mOhm) [retailer sheets: imrbatteries / akkuteile / 18650batterystore;
# not the official datasheet].  P50B: 5.0 Ah typ, 60 A (retailer title, UNVERIFIED), mass and
# impedance assumed ~ P45B (69 g, 12 mOhm).  18650: P28A-class 2.8 Ah, 47 g, 35 A, 14 mOhm (ASSUMED
# from memory of Molicel P28A / Samsung 25R-30Q class).  LiPo: FPV pouch packs ~120-140 Wh/kg at
# pack level (1300 mAh 4S ~150 g) -> 150 Wh/kg cell + 10 % overhead; 100C-class cells ~6-8 mOhm*Ah
# per cell, 40C continuous used (derated from marketing rating).  LiHV: +~5 % energy.
CELLS: dict[str, dict] = {
    "lipo":  dict(Vnom=3.7, Vfull=4.20, Vcut=3.3, wh_kg=150.0, ir=0.008, imax=40.0, usable=0.85,
                  overhead=0.10, per_cell=False),
    "lihv":  dict(Vnom=3.8, Vfull=4.35, Vcut=3.3, wh_kg=158.0, ir=0.009, imax=40.0, usable=0.85,
                  overhead=0.10, per_cell=False),
    "liion_21700": dict(Vnom=3.6, Vfull=4.20, Vcut=3.0, Ah=4.5, kg=0.069, ir=0.013, imax=45.0,
                        usable=0.92, overhead=0.15, per_cell=True),
    "liion_21700_p50b": dict(Vnom=3.6, Vfull=4.20, Vcut=3.0, Ah=5.0, kg=0.069, ir=0.012, imax=60.0,
                             usable=0.92, overhead=0.15, per_cell=True),
    "liion_18650": dict(Vnom=3.6, Vfull=4.20, Vcut=3.0, Ah=2.8, kg=0.047, ir=0.014, imax=35.0,
                        usable=0.92, overhead=0.15, per_cell=True),
}
# OCV per cell vs soc (typical resting curves, ±0.05 V)
_SOC = np.array([0, .05, .1, .2, .4, .6, .8, .9, 1.0])
OCV = {
    "lipo": np.array([3.30, 3.55, 3.65, 3.72, 3.78, 3.85, 3.97, 4.07, 4.20]),
    "lihv": np.array([3.30, 3.55, 3.65, 3.72, 3.80, 3.90, 4.08, 4.20, 4.35]),
    "liion": np.array([3.00, 3.35, 3.45, 3.55, 3.65, 3.82, 4.00, 4.10, 4.20]),
}
PACK_FIXED_KG = 0.006     # leads + connector (XT30/XT60)
PACK_FIXED_R = 0.004      # ohm, leads + connector contact


def _ocv_table(chem: str) -> np.ndarray:
    return OCV["liion" if chem.startswith("liion") else chem]


@dataclass
class Pack:
    chemistry: str
    S: int
    P: int
    Ah: float
    mass: float
    R: float              # ohm, whole pack incl. leads
    I_cont: float         # A
    usable: float
    Vcut: float
    energy_Wh: float      # nominal

    def ocv(self, soc: float) -> float:
        return float(self.S * np.interp(soc, _SOC, _ocv_table(self.chemistry)))

    def current(self, P_w: float, soc: float) -> float:
        """Battery current for constant electrical power at `soc` (inf if unsupportable)."""
        v = self.ocv(soc)
        disc = v * v - 4 * self.R * P_w
        if disc < 0:
            return float("inf")
        return float((v - np.sqrt(disc)) / (2 * self.R))

    def v_loaded(self, P_w: float, soc: float) -> float:
        i = self.current(P_w, soc)
        return self.ocv(soc) - i * self.R if np.isfinite(i) else 0.0

    def specific_energy_Wh_kg(self) -> float:
        return self.energy_Wh / self.mass


def make_pack(b: BatterySpec) -> Pack:
    c = CELLS[b.chemistry]
    ovh = c["overhead"] if b.overhead_frac is None else b.overhead_frac
    if c["per_cell"]:
        Ah = b.P * c["Ah"]
        cells_kg = b.S * b.P * c["kg"]
        r_cell = c["ir"]
        R = b.S * r_cell / b.P
        I_cont = b.P * c["imax"]
        P = b.P
    else:
        Ah = b.capacity_Ah if b.capacity_Ah else 1.3
        cells_kg = Ah * c["Vnom"] * b.S / c["wh_kg"]
        R = b.S * c["ir"] / Ah
        I_cont = c["imax"] * Ah
        P = 1
    mass = cells_kg * (1 + ovh) + PACK_FIXED_KG
    return Pack(chemistry=b.chemistry, S=b.S, P=P, Ah=Ah, mass=mass, R=R + PACK_FIXED_R,
                I_cont=I_cont, usable=c["usable"] if b.usable_fraction is None else b.usable_fraction,
                Vcut=c["Vcut"], energy_Wh=Ah * c["Vnom"] * b.S)


def discharge(pack: Pack, P_w: float, steps: int = 60) -> dict:
    """Constant-power discharge from full to (1-usable) soc (or loaded Vcut). Returns endurance [s],
    energy [Wh], V_mid/V_end loaded, I_start/I_mid/I_end, limited_by."""
    s0, s1 = 1.0, 1.0 - pack.usable
    socs = np.linspace(s0, s1, steps + 1)
    t = 0.0
    lim = "usable"
    last = s0
    for a, b in zip(socs[:-1], socs[1:]):
        sm = 0.5 * (a + b)
        i = pack.current(P_w, sm)
        vcell = pack.v_loaded(P_w, sm) / pack.S
        if not np.isfinite(i) or vcell < pack.Vcut:
            lim = "voltage"
            break
        t += (a - b) * pack.Ah * 3600.0 / i
        last = b
    mid = 0.5 * (s0 + max(last, s1)) if last < s0 else s0
    return dict(endurance_s=float(t), energy_Wh=P_w * t / 3600.0, V_mid=pack.v_loaded(P_w, mid),
                V_end=pack.v_loaded(P_w, max(last, 1e-3)), V_start=pack.v_loaded(P_w, 1.0),
                I_start=pack.current(P_w, 1.0), I_mid=pack.current(P_w, mid),
                I_end=pack.current(P_w, max(last, 1e-3)), soc_end=last, limited_by=lim)


# ------------------------------------------------------------------------------ mass budget
# kg.  frame: body+mast mount+legs+fasteners ~60 g at 5" single (Hakea-class; ASSUMED), ∝ D^1.5
# (area/length mix); coax x1.5 (second motor mount, hollow/concentric mast).
# motor: T-Motor 2307 ~33 g, 2806.5 class ~55 g (catalog typical).  ESC: AM32 35-50 A single ~10-14 g.
# avionics: FC+RX+encoder+magnet+wiring ~28 g.  Hub/clamp 6 g (D/5)^2 per rotor.
FRAME_5IN = 0.060
FRAME_EXP = 1.5
COAX_FRAME_FACTOR = 1.5
MOTOR_MASS = {"2307": 0.033, "2806.5": 0.055}
ESC_MASS = {"2307": 0.010, "2806.5": 0.014}
AVIONICS = 0.028
HUB_5IN = 0.006
BLADE_OVERHEAD = 1.15     # root attachment, balance, finishing over bare print volume
P_2307_MAX = 150.0        # W shaft per motor above which the 2806.5-class is used


def blade_mass(d: Design) -> float:
    """One rotor's blades (n_blades) from station chord x NACA 4-digit area (0.685 t c^2)."""
    R = d.blade.radius
    st = d.blade.stations
    r = np.array([s.r_frac for s in st]) * R
    c = np.array([s.chord for s in st])
    tc = 0.12
    if d.blade.airfoil.kind == "naca4" and d.blade.airfoil.naca:
        tc = int(d.blade.airfoil.naca[2:]) / 100.0
    area = 0.685 * tc * c**2
    vol = float(np.trapezoid(area, r)) if hasattr(np, "trapezoid") else float(np.trapz(area, r))
    rho = get_material(d.blade.layup.material).rho
    return d.blade.n_blades * vol * rho * BLADE_OVERHEAD


def flexure_mass(d: Design) -> float:
    f = d.flexure
    lay = f.layup
    rho = get_material(lay.material).rho
    vol = f.width * f.thickness * f.length * (2 if f.topology == "skewed_cross_blade" else 1)
    return d.blade.n_blades * vol * rho


def rotor_mass(d: Design) -> dict:
    s = d.blade.diameter_in / 5.0
    return dict(blades=blade_mass(d), flexure=flexure_mass(d), hub=HUB_5IN * s**2)


def motor_class(shaft_W_per_motor: float) -> str:
    return "2307" if shaft_W_per_motor <= P_2307_MAX else "2806.5"


def build_motor(cls: str, kv: float, V: float, D_in: float) -> Motor:
    """2307-class or 2806.5-class motor re-wound to `kv`.  R_eff = R_fixed(ESC+wires) +
    R_winding(1300/kv)^2 (same stator, turns ∝ 1/kv => R ∝ 1/kv^2); 2307 anchored to the Hakea fit
    (0.158 ohm @1300 kv, ~0.06 of it ESC/wiring)."""
    if cls == "2307":
        base = get_motor("tmotor_2307_1300kv")
        R = 0.06 + 0.098 * (1300.0 / kv) ** 2
        imax = 40.0
    else:
        base = get_motor("2806.5_1100kv_6s")
        R = 0.05 + 0.05 * (1100.0 / kv) ** 2
        imax = 50.0
    from .motor import estimate_L
    dh = (23, 7) if cls == "2307" else (28, 6.5)
    return base.model_copy(update=dict(
        name=f"{cls}_{kv:.0f}kv", kv=float(kv), R=float(R), L=estimate_L(kv, *dh), max_current=imax,
        supply_V=float(V), J_hub=2.0e-6 * (D_in / 5.0) ** 3, duty_slew_max=2550.0))


# ------------------------------------------------------------------------------ rotor default


def default_rotor(D: float, layout: str = "single") -> Design:
    """TPU + CF-rod flexure, PA6-CF blade (study concept), elastic axis 0.43c, AM32 slew 2550."""
    from . import presets
    d = (presets.make_coax_preset(D, "tpu_rod") if layout == "coaxial"
         else presets.make_preset(D, "tpu_rod")).model_copy(deep=True)
    d.blade.layup = Layup(kind="solid", material="pa6_cf")
    d.blade.elastic_axis_chord_frac = 0.43
    d.motor.duty_slew_max = 2550.0
    return d


# ------------------------------------------------------------------------------ mission


@dataclass
class MissionResult:
    spec: VehicleSpec
    mass: dict
    auw: float
    payload_fraction: float
    pack: Pack
    motor: Motor
    motor_class: str
    rpm: list
    Q: list
    P_shaft: float
    P_elec: float
    P_breakdown: dict
    disc: dict
    endurance_min: float
    endurance_min_derated: float
    I_hover_mid: float
    I_peak_typ: float
    C_hover: float
    headroom: dict
    control_ratio: Optional[float] = None
    control_ratio_typ: Optional[float] = None
    control_ratio_end: Optional[float] = None
    max_util: Optional[float] = None
    checks_bad: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    flags: dict = field(default_factory=dict)
    feasible: bool = False
    design: Optional[Design] = None

    def summary(self) -> dict:
        return dict(
            name=self.spec.name, layout=self.spec.layout, D=self.spec.diameter_in,
            chem=self.pack.chemistry, S=self.pack.S, P=self.pack.P, Ah=round(self.pack.Ah, 3),
            payload_g=self.spec.payload_kg * 1000, auw_g=self.auw * 1000,
            pack_g=self.pack.mass * 1000, payload_frac=self.payload_fraction,
            P_elec_W=self.P_elec, P_shaft_W=self.P_shaft, endurance_min=self.endurance_min,
            endurance_min_derated=self.endurance_min_derated, I_hover_A=self.I_hover_mid,
            I_peak_typ_A=self.I_peak_typ, C_hover=self.C_hover, kv=self.motor.kv, motor=self.motor_class,
            V_mid=self.disc["V_mid"], V_end=self.disc["V_end"],
            dQ_mid=self.headroom["dQ_mid"], mod_mid=self.headroom["mod_mid"],
            mod_end=self.headroom["mod_end"], control_ratio=self.control_ratio,
            control_ratio_typ=self.control_ratio_typ, control_ratio_end=self.control_ratio_end,
            max_util=self.max_util, feasible=self.feasible, flags=self.flags)


def aero_loads(d: Design, auw: float, derate: float = 1.0) -> tuple[list, list, float]:
    """Aero-only hover trim -> ([rpm_i], [Q_i], P_shaft) per rotor at AUW (thrust_derate:
    required BEM thrust = W/derate)."""
    dd = d.model_copy(deep=True)
    dd.op = dd.op.model_copy(update=dict(vehicle_mass=auw / derate, rpm=None))
    if d.layout == "coaxial":
        from .coaxial import pair_trim
        pt = pair_trim(dd)
        return [pt.upper.rpm, pt.lower.rpm], [pt.upper.Q, pt.lower.Q], pt.upper.P + pt.lower.P
    from .aero import AeroModel
    t = AeroModel(dd).trim()
    return [t.rpm], [t.Q], t.P


def electrical_power(m: Motor, rpm: list, Q: list, ctrl_mod: float) -> tuple[float, dict]:
    """Battery-side power [W] for the rotor set; ESC switching loss 2 %, idle 0.3 W/ESC, avionics 1.5 W."""
    K = kt(m)
    shaft = cu = fr = 0.0
    for n, q in zip(rpm, Q):
        w = n * 2 * np.pi / 60
        qf = q + m.b * w + m.T0
        I = qf / K
        shaft += w * q
        fr += w * (qf - q)
        cu += m.R * I * I
    cyc = 0.5 * ctrl_mod**2 * cu
    mot = shaft + fr + cu + cyc
    esc = 0.02 * mot + 0.3 * len(rpm)
    av = 1.5
    return mot + esc + av, dict(shaft=shaft, friction=fr, copper=cu, cyclic_copper=cyc, esc=esc, avionics=av)


def mass_budget(spec: VehicleSpec, pack: Pack, cls: str, d: Design) -> dict:
    n = 2 if spec.layout == "coaxial" else 1
    s = spec.diameter_in / 5.0
    rm = rotor_mass(d)
    b = dict(
        frame=FRAME_5IN * s**FRAME_EXP * (COAX_FRAME_FACTOR if n == 2 else 1.0),
        motors=n * MOTOR_MASS[cls], escs=n * ESC_MASS[cls], avionics=AVIONICS,
        blades=n * rm["blades"], flexure=n * rm["flexure"], hub=n * rm["hub"],
        battery=pack.mass, payload=spec.payload_kg)
    b.update({k: v for k, v in spec.mass_overrides.items() if k in b})
    return b


def evaluate_mission(spec: VehicleSpec, full: bool = True, mapper=None) -> MissionResult:
    """Size motor kv to the supply, evaluate hover power/endurance, then (full=True) the full
    rotor/coaxial structural evaluation at the resulting supply voltage for control_ratio+checks."""
    d0 = spec.rotor or default_rotor(spec.diameter_in, spec.layout)
    d0 = d0.model_copy(deep=True)
    d0.layout = spec.layout
    pack = make_pack(spec.battery)
    D = d0.blade.diameter_in
    # mass (motor class depends on shaft power -> fixed point on class, 2 passes)
    cls = "2307"
    for _ in range(2):
        mb = mass_budget(spec, pack, cls, d0)
        auw = sum(mb.values())
        rpm, Q, Psh = aero_loads(d0, auw)
        cls_new = motor_class(Psh / len(rpm))
        if cls_new == cls:
            break
        cls = cls_new
    mb = mass_budget(spec, pack, cls, d0)
    auw = sum(mb.values())
    rpm, Q, Psh = aero_loads(d0, auw)
    rpm_d, Q_d, Psh_d = aero_loads(d0, auw, spec.thrust_derate if spec.thrust_derate < 1 else 0.77)
    # kv/V fixed point
    V = pack.S * 3.75
    for _ in range(4):
        kv = float(np.round(np.mean(rpm) / (spec.emf_frac * V), -1))
        m = build_motor(cls, kv, V, D)
        Pel, br = electrical_power(m, rpm, Q, spec.ctrl_mod_typ)
        disc = discharge(pack, Pel)
        V = max(disc["V_mid"], 0.5 * pack.S * 3.7)
    m = build_motor(cls, kv, V, D)
    Pel, br = electrical_power(m, rpm, Q, spec.ctrl_mod_typ)
    disc = discharge(pack, Pel)
    Pel_d, _ = electrical_power(m, rpm_d, Q_d, spec.ctrl_mod_typ)
    endu_d = discharge(pack, Pel_d)["endurance_s"] / 60.0

    # motor pulse headroom at mid / end-of-discharge supply
    def head(Vs):
        mm = m.model_copy(update=dict(supply_V=float(Vs)))
        dq, mod = [], []
        for n, q in zip(rpm, Q):
            c = capability(mm, n, q, DUTY_MAX)
            dq.append(c.dQ_max_1p)
            mod.append(min(c.dQ_max_1p / q, spec.max_mod_frac))
        return float(np.mean(dq)), float(np.mean(mod)), c.I_mean
    dq_mid, mod_mid, I0 = head(disc["V_mid"])
    dq_end, mod_end, _ = head(disc["V_end"])
    K = kt(m)
    I_peak_typ = disc["I_mid"] * (1 + spec.ctrl_mod_typ)
    res = MissionResult(
        spec=spec, mass=mb, auw=auw, payload_fraction=spec.payload_kg / auw, pack=pack, motor=m,
        motor_class=cls, rpm=rpm, Q=Q, P_shaft=Psh, P_elec=Pel, P_breakdown=br, disc=disc,
        endurance_min=disc["endurance_s"] / 60.0, endurance_min_derated=endu_d,
        I_hover_mid=disc["I_mid"], I_peak_typ=I_peak_typ, C_hover=disc["I_start"] / pack.Ah,
        headroom=dict(dQ_mid=dq_mid, dQ_end=dq_end, mod_mid=mod_mid, mod_end=mod_end,
                      emf_frac_mid=float(np.mean(rpm) / kv / disc["V_mid"]),
                      emf_frac_end=float(np.mean(rpm) / kv / disc["V_end"])))
    res.flags = dict(
        current_ok=disc["I_start"] <= pack.I_cont, peak_ok=I_peak_typ <= 1.5 * pack.I_cont,
        endurance_to_usable=disc["limited_by"] == "usable",
        motor_current_ok=max(q / K for q in Q) * (1 + spec.ctrl_mod_typ) <= m.max_current,
        headroom_end_ok=mod_end >= spec.ctrl_mod_typ, headroom_mid_ok=mod_mid >= spec.ctrl_mod_typ,
        backemf_ok=res.headroom["emf_frac_end"] <= 0.80)
    if full:
        d = d0.model_copy(deep=True)
        d.motor = m.model_copy(update=dict(supply_V=float(disc["V_mid"])))
        d.op = d.op.model_copy(update=dict(vehicle_mass=auw, rpm=None, torque_mod_frac=max(mod_mid, 1e-3)))
        if d.layout == "coaxial":
            d.coaxial = d.coaxial.model_copy(update=dict(lower_motor=d.motor.model_copy(deep=True)))
        res.design = d
        from . import api
        r = api.evaluate(d)
        if d.layout == "coaxial":
            res.control_ratio = r.coaxial.control_ratio
        else:
            res.control_ratio = r.harmonic.control_ratio if r.harmonic else None
        if res.control_ratio is not None and mod_mid > 0:
            res.control_ratio_typ = res.control_ratio * spec.ctrl_mod_typ / mod_mid
            res.control_ratio_end = res.control_ratio * mod_end / mod_mid
        utils = [c.utilisation for c in r.checks]
        res.max_util = max(utils) if utils else None
        res.checks_bad = [f"{c.name}:{c.member}={c.utilisation:.2f}" for c in r.checks if c.utilisation > 1]
        res.warnings = list(r.warnings)
        res.flags["structure_ok"] = not res.checks_bad
    res.feasible = all(res.flags.values())
    return res
