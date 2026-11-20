"""Motor / ESC torque model and 1/rev torque capability (OWNER: aero).

DC-equivalent BLDC model (same form as the Hakea motor-ID fit,
notes/sim/calib/params_v5all.json):

    V_term = R·I + Ke·ω,   Q_motor = Kt·I,   Kt = Ke = 60 / (2π·kv)   [N m/A]

with R the effective resistance (phase + ESC + wiring) and kv the effective rpm/V.
The ESC can apply at most V_max = duty_max·supply_V.  For a 1/rev torque pulse
Q(t) = Q0 + ΔQ cos(Ωt + φ) the current is I0 + ΔI cos(…), ΔI = ΔQ/Kt, and the
peak requires

    current:   I0 + ΔI ≤ max_current
    voltage:   R(I0 + ΔI) + Ke Ω + ω_e L ΔI ≤ V_max   (L term neglected, ≲1 V here)

The torque loop (encoder-commutated current/duty control) is modelled as a first
order lag with corner `torque_bandwidth_hz`: H(f) = 1/(1 + i f/f_bw).  The ESC can
pre-compensate the gain if headroom allows, so the limits above are on the
*delivered* amplitude; the lag is reported for phase bookkeeping.

Reference (Hakea, 5" T-Motor 2307 1300KV, 6S; fitted):
    R = 0.158 Ω, Kt(nominal 1300KV) = 0.00735 N m/A, effective kv ≈ 1340–1380,
    J_m = 4.17e-6 kg m² (motor bell), J_p = 2.53e-6 kg m² (5" prop + adapter),
    prop torque Q = kq ω², kq = 1.84e-8 N m s².
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .schema import Motor

DUTY_MAX = 0.95

HAKEA_REFERENCE = dict(
    source="/home/fred/vehicles/hakea/notes/sim/calib/params_v5all.json (motor-ID fit, logs 33/34)",
    R=0.15831754655637395, J_m=4.167271856614777e-06, J_p=2.5295148680153794e-06,
    kq=1.8425621926787373e-08, kv_nominal=1300.0, kv_effective=1360.0,
    Kt_nominal=0.00735, hover_torque_bench=0.056,
)


def kt(motor: Motor) -> float:
    """Torque constant Kt = Ke = 60/(2π kv) [N m/A] (SI back-EMF constant)."""
    return 60.0 / (2.0 * np.pi * motor.kv)


@dataclass
class MotorCapability:
    rpm: float
    Kt: float
    back_emf: float            # V at this rpm
    V_max: float               # duty_max · supply
    I_mean: float              # A for the requested mean torque
    Q_mean_max: float          # N m, max steady torque at this rpm
    dQ_max_1p: float           # N m, max deliverable 1/rev amplitude about Q_mean
    dQ_limit: str              # 'current' | 'voltage' | 'none (mean over limit)'
    dQ_current_limit: float
    dQ_voltage_limit: float
    dQ_thermal_limit: float    # continuous full-cyclic copper budget (inf if thermal=False)
    f_1p: float                # Hz
    bandwidth_gain: float      # |H(f_1p)|
    bandwidth_phase_deg: float  # arg H(f_1p) (lag negative)
    copper_W_mean: float
    copper_W_at_dQmax: float   # R(I0² + ΔI²/2)
    mod_frac_max: float        # dQ_max_1p / Q_mean


def capability(motor: Motor, rpm: float, mean_torque: float = 0.0,
               duty_max: float = DUTY_MAX, ambient_C: float = 25.0,
               thermal: bool = True) -> MotorCapability:
    """Steady and 1/rev torque capability at `rpm` around `mean_torque` [N m].

    Q_mean_max = Kt·min(I_max, (V_max − Ke ω)/R)
    dQ_max_1p  = Kt·min(I_max − I0, (V_max − Ke ω − R I0)/R), clipped at ≥ 0.
    """
    K = kt(motor)
    w = rpm * 2 * np.pi / 60.0
    emf = K * w
    Vmax = duty_max * motor.supply_V
    I0 = mean_torque / K
    i_v = (Vmax - emf) / motor.R
    Qmax = K * max(0.0, min(motor.max_current, i_v))
    dI_c = motor.max_current - I0
    dI_v = i_v - I0
    # thermal: steady winding T = Ta + Rth (R(I0² + dI²/2) + b w²) <= max_winding_C
    # (continuous full-amplitude cyclic = worst case; Rth is a guess, see schema)
    P_allow = (motor.max_winding_C - ambient_C) / motor.thermal_resistance_KW
    P_room = P_allow - motor.R * I0**2 - motor.b * w**2
    rf = max(motor.cyclic_rms_frac, 1e-3)
    dI_t = np.sqrt(2.0 * P_room / motor.R) / rf if (thermal and P_room > 0) else (0.0 if thermal else np.inf)
    dI = max(0.0, min(dI_c, dI_v, dI_t))
    lim = min((dI_c, "current"), (dI_v, "voltage"), (dI_t, "thermal"))[1]
    if dI <= 0:
        lim = "none (mean over limit)"
    f = rpm / 60.0
    H = 1.0 / (1.0 + 1j * f / motor.torque_bandwidth_hz)
    return MotorCapability(
        rpm=float(rpm), Kt=K, back_emf=emf, V_max=Vmax, I_mean=I0, Q_mean_max=Qmax,
        dQ_max_1p=K * dI, dQ_limit=lim, dQ_current_limit=K * max(dI_c, 0.0),
        dQ_voltage_limit=K * max(dI_v, 0.0), dQ_thermal_limit=K * max(float(dI_t), 0.0), f_1p=f, bandwidth_gain=float(abs(H)),
        bandwidth_phase_deg=float(np.degrees(np.angle(H))),
        copper_W_mean=motor.R * I0**2, copper_W_at_dQmax=motor.R * (I0**2 + 0.5 * dI**2),
        mod_frac_max=(K * dI / mean_torque) if mean_torque > 0 else float("inf"))


def thermal(motor: Motor, rpm: float, mean_torque: float, dI_amp: float = 0.0,
            ambient_C: float = 25.0) -> dict:
    """Steady winding temperature with a 1/rev current pulse of amplitude `dI_amp` [A].

    copper = I0^2 R + 1/2 (rms_frac dI)^2 R (I0 = Q/Kt; rms_frac = typical/max cyclic demand);  total = copper + b w^2 (iron/friction, lumped
    into the winding path).  T = ambient + total * Rth.  utilisation = rise / (max - ambient).
    """
    w = rpm * 2 * np.pi / 60.0
    I0 = mean_torque / kt(motor)
    cu = motor.R * (I0**2 + 0.5 * (motor.cyclic_rms_frac * dI_amp)**2)
    P = cu + motor.b * w**2
    T = ambient_C + P * motor.thermal_resistance_KW
    room = motor.max_winding_C - ambient_C
    return dict(copper_W=float(cu), total_W=float(P), T_winding_C=float(T),
                utilisation=float((T - ambient_C) / room) if room > 0 else float("inf"))


def max_rpm(motor: Motor, mean_torque: float, duty_max: float = DUTY_MAX) -> float:
    """Highest rpm at which `mean_torque` can still be produced (voltage limit)."""
    K = kt(motor)
    I0 = mean_torque / K
    w = (duty_max * motor.supply_V - motor.R * I0) / K
    return max(w, 0.0) * 60 / (2 * np.pi)


def hub_speed_amp(dQ: float, J_total: float, rpm: float) -> float:
    """Rigid-rotor 1/rev speed oscillation amplitude [rpm]: Δω = ΔQ/(J Ω)."""
    w = rpm * 2 * np.pi / 60.0
    return dQ / (J_total * w) * 60 / (2 * np.pi)


# ----------------------------------------------------------------------------- library
# Representative stock motors per rotor size. Effective R includes ESC + wiring
# (Hakea fit 0.158 Ω vs ~0.07 Ω datasheet for the bare 2307 winding), so
# datasheet values are roughly doubled; ±50 %.  J_rotor (bell + magnets + shaft)
# scaled from the fitted 2307 (4.17e-6) as J ∝ d_stator³·h_stator; ±40 %.
# max_current: datasheet peak/burst figures, conservative.  J_hub: hub +
# fasteners, 2e-6 at 5", scaled ∝ D³.  kv chosen so preset hover back-EMF is
# ~45-65 % of supply (leaves voltage headroom for the 1/rev pulse).

def estimate_L(kv: float, d_mm: float, h_mm: float) -> float:
    """Best-estimate phase-phase inductance (H) for an outrunner, scaled from Hakea's 2307.

    Same-construction scaling: flux linkage ∝ N·d·h → N ∝ 1/(kv·d·h); inductance ∝ N²·d·h
    → L ∝ 1/(kv²·d·h). Reference: 2307 1300KV (nominal) ≈ 36 µH ph-ph — consistent with
    community measurements of 2207 1750KV (~17 µH) and 2306 2450KV (~9 µH) scaled the
    same way (31-37 µH) and with the SITL calibration (L_phase 18 µH). Uncertainty ±30 %.
    """
    return L_REF * (KV_REF / kv) ** 2 * (23.0 * 7.0) / (d_mm * h_mm)


L_REF = 36e-6
KV_REF = 1300.0


def _J(d_mm, h_mm):
    return 4.17e-6 * (d_mm / 23.0) ** 3 * (h_mm / 7.0)


def _Rth(d_mm, h_mm):
    """Winding->ambient K/W, scaled from 1.5 K/W (2307, prop wash) by 1/stator area (d*h); +-50 %."""
    return 1.5 * (23.0 * 7.0) / (d_mm * h_mm)


MOTOR_LIBRARY: dict[str, tuple[Motor, str]] = {
    "1804_2450kv_4s": (Motor(name="1804_2450kv_4s", kv=2450, L=estimate_L(2450, 18, 4), R=0.20, J_rotor=_J(18, 4), thermal_resistance_KW=_Rth(18, 4),
                             J_hub=2.0e-6 * (4 / 5) ** 3, supply_V=16.0, max_current=22.0),
                       "typical 3-4\" 4S motor; datasheet Rm ~0.1 Ω"),
    "2004_1700kv_6s": (Motor(name="2004_1700kv_6s", kv=1700, L=estimate_L(1700, 20, 4), R=0.20, J_rotor=_J(20, 4), thermal_resistance_KW=_Rth(20, 4),
                             J_hub=2.0e-6 * (4 / 5) ** 3, supply_V=24.0, max_current=28.0),
                       "4\" 6S (e.g. 2004 1700KV class); datasheet Rm ~0.1 Ω"),
    "tmotor_2307_1300kv": (Motor(), "Hakea fitted motor (R, kv, J from motor-ID)"),
    "2806.5_1100kv_6s": (Motor(name="2806.5_1100kv_6s", kv=1100, L=estimate_L(1100, 28, 6.5), R=0.10, J_rotor=_J(28, 6.5), thermal_resistance_KW=_Rth(28, 6.5),
                               J_hub=2.0e-6 * (7 / 5) ** 3, supply_V=24.0, max_current=50.0),
                         "7\" long-range 6S class (1050-1300KV sold); datasheet Rm ~0.05 Ω"),
    "3115_640kv_6s": (Motor(name="3115_640kv_6s", kv=640, L=estimate_L(640, 31, 15), R=0.09, J_rotor=_J(31, 15), thermal_resistance_KW=_Rth(31, 15),
                            J_hub=2.0e-6 * (10 / 5) ** 3, supply_V=24.0, max_current=60.0),
                      "9-10\" 6S class (3115 640/900KV sold); datasheet Rm ~0.05 Ω"),
    "4010_475kv_6s": (Motor(name="4010_475kv_6s", kv=475, L=estimate_L(475, 40, 10), R=0.12, J_rotor=_J(40, 10), thermal_resistance_KW=_Rth(40, 10),
                            J_hub=2.0e-6 * (12 / 5) ** 3, supply_V=24.0, max_current=45.0),
                      "12-13\" 6S class (4010 ~370-580KV sold); datasheet Rm ~0.06 Ω"),
}

SIZE_TO_MOTOR = {4.0: "2004_1700kv_6s", 5.0: "tmotor_2307_1300kv", 7.0: "2806.5_1100kv_6s",
                 10.0: "3115_640kv_6s", 12.0: "4010_475kv_6s"}


def get_motor(name: str) -> Motor:
    return MOTOR_LIBRARY[name][0].model_copy(deep=True)


def motor_for_diameter(diameter_in: float) -> Motor:
    """Representative stock motor for a rotor size (nearest tabulated size)."""
    sizes = np.array(sorted(SIZE_TO_MOTOR))
    d = float(sizes[np.argmin(np.abs(sizes - diameter_in))])
    return get_motor(SIZE_TO_MOTOR[d])


def list_motors() -> list[dict]:
    return [dict(m.model_dump(), note=n) for m, n in MOTOR_LIBRARY.values()]


# --------------------------------------------------------------------------- dynamic electrical model
# Averaged BLDC model from scripts/motorid/motorid_model.py (Hakea fit), linearised:
#     L dI/dt = g·Vbus·d − Ke·ω − (R + Rb·(g d)²)·I,   Q = Kt·I
# Small-signal at frequency w (rad/s, rotating-frame = fixed-frame for the hub DOF):
#     ΔI = (g Vbus Δd − Ke Δω) / Z(w),  Z = R + Rb (g d0)² + i w L
# So the ESC duty modulation gives a hub torque  ΔQ_drive = Kt g Vbus Δd / Z
# and back-EMF adds a hub torque −(Kt Ke / Z) Δω = −(Kt Ke / Z) i w Δφ,
# i.e. a complex dynamic stiffness  k_e(w) = i w Kt Ke / Z(w)  on the hub rotation DOF.


def impedance(motor: Motor, w: float, d0: float = 0.6) -> complex:
    return motor.R + motor.Rb * (motor.voltage_gain * d0) ** 2 + 1j * w * motor.L


def duty_torque_gain(motor: Motor, w: float, d0: float = 0.6, vbus: float | None = None) -> complex:
    """ΔQ / Δd (N m per unit duty) at frequency w, speed held fixed."""
    vb = motor.supply_V if vbus is None else vbus
    return kt(motor) * motor.voltage_gain * vb / impedance(motor, w, d0)


def hub_electrical_stiffness(motor: Motor, w: float, d0: float = 0.6) -> complex:
    """Complex stiffness (N m/rad) to ADD on the hub rotation DOF: back-EMF damping + viscous."""
    k = kt(motor)
    return 1j * w * (k * k / impedance(motor, w, d0) + motor.b)


def steady_duty(motor: Motor, rpm: float, mean_torque: float, vbus: float | None = None) -> float:
    vb = motor.supply_V if vbus is None else vbus
    w = rpm * 2 * np.pi / 60
    k = kt(motor)
    I0 = (mean_torque + motor.T0 + motor.b * w) / k
    return float((k * w + motor.R * I0) / (motor.voltage_gain * vb))


def duty_drive(motor: Motor, rpm: float, mean_torque: float, torque_mod_frac: float,
               duty_mod_amp: float | None = None) -> dict:
    """Duty amplitude and the hub forcing for the 1/rev harmonic problem.

    Returns dict: d0, dd (duty amplitude), w (rad/s), Q_drive (complex N m, speed-locked
    torque at 1/rev), k_hub (complex stiffness to add on hub DOF), Q_dc (= dd·gain at DC),
    gain_ratio |G(w)/G(0)|, phase_deg of G(w), slew (max duty rate, fraction/s),
    slew_util (slew / motor.duty_slew_max), duty_peak.
    """
    w = rpm * 2 * np.pi / 60
    d0 = steady_duty(motor, rpm, mean_torque)
    G0 = duty_torque_gain(motor, 0.0, d0)
    Gw = duty_torque_gain(motor, w, d0)
    dd = duty_mod_amp if duty_mod_amp is not None else torque_mod_frac * mean_torque / abs(G0)
    slew = dd * w
    return dict(d0=d0, dd=dd, w=w, Q_drive=Gw * dd, Q_dc=abs(G0) * dd,
                k_hub=hub_electrical_stiffness(motor, w, d0),
                gain_ratio=abs(Gw) / abs(G0), phase_deg=float(np.degrees(np.angle(Gw))),
                slew=slew, slew_util=slew / motor.duty_slew_max, duty_peak=d0 + dd)
