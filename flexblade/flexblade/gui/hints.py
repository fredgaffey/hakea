"""Display hints for the parameter panel: display unit, SI->display scale, slider range.

Keys are Design dotted paths with list indices / nested layups replaced by '*' where useful.
Ranges are in *display* units. Values in Design JSON stay SI. Paths not listed fall back to
schema bounds or a range around the current value.
"""
from __future__ import annotations

MM = {"unit": "mm", "scale": 1e3}
DEG = {"unit": "deg", "scale": 1.0}
GCM2 = {"unit": "g·cm²", "scale": 1e7}

HINTS: dict[str, dict] = {
    "blade.diameter_in": {"unit": "in", "scale": 1, "min": 3, "max": 14, "step": 0.25},
    "blade.n_blades": {"unit": "", "scale": 1, "min": 1, "max": 6, "step": 1},
    "blade.root_cutout_frac": {"unit": "r/R", "scale": 1, "min": 0.05, "max": 0.5, "step": 0.01},
    "blade.elastic_axis_chord_frac": {"unit": "x/c", "scale": 1, "min": 0, "max": 1, "step": 0.01},
    "flexure.skew_deg": {**DEG, "min": 0, "max": 75, "step": 0.5},
    "flexure.hinge_radius": {**MM, "min": 2, "max": 40, "step": 0.1},
    "flexure.length": {**MM, "min": 0.5, "max": 30, "step": 0.1},
    "flexure.thickness": {**MM, "min": 0.1, "max": 5, "step": 0.02},
    "flexure.width": {**MM, "min": 0.5, "max": 30, "step": 0.1},
    "flexure.cross_angle_deg": {**DEG, "min": 20, "max": 160, "step": 1},
    "flexure.taper": {"unit": "", "scale": 1, "min": 0.2, "max": 2, "step": 0.01},
    "flexure.twist_deg": {**DEG, "min": -60, "max": 60, "step": 0.5},
    "flexure.hub_radius": {**MM, "min": 1, "max": 25, "step": 0.1},
    "*.skin_thickness": {**MM, "min": 0, "max": 2, "step": 0.02},
    "*.insert_diameter": {**MM, "min": 0, "max": 3, "step": 0.05},
    "motor.kv": {"unit": "rpm/V", "scale": 1, "min": 100, "max": 4000, "step": 10},
    "motor.R": {"unit": "Ω", "scale": 1, "min": 0.01, "max": 1, "step": 0.001},
    "motor.J_rotor": {**GCM2, "min": 0, "max": 500, "step": 0.5},
    "motor.J_hub": {**GCM2, "min": 0, "max": 500, "step": 0.5},
    "motor.supply_V": {"unit": "V", "scale": 1, "min": 3, "max": 60, "step": 0.1},
    "motor.max_current": {"unit": "A", "scale": 1, "min": 1, "max": 150, "step": 1},
    "motor.torque_bandwidth_hz": {"unit": "Hz", "scale": 1, "min": 50, "max": 10000, "step": 10},
    "motor.L": {"unit": "µH", "scale": 1e6, "min": 1, "max": 500, "step": 0.5},
    "motor.Rb": {"unit": "Ω", "scale": 1, "min": 0, "max": 0.5, "step": 0.001},
    "motor.b": {"unit": "µN·m·s", "scale": 1e6, "min": 0, "max": 100, "step": 0.1},
    "motor.T0": {"unit": "mN·m", "scale": 1e3, "min": 0, "max": 5, "step": 0.0001},
    "motor.voltage_gain": {"unit": "", "scale": 1, "min": 0.5, "max": 1.5, "step": 0.005},
    "motor.poles": {"unit": "", "scale": 1, "min": 2, "max": 42, "step": 2},
    "motor.duty_slew_max": {"unit": "1/s", "scale": 1, "min": 1, "max": 2000, "step": 1},
    "op.duty_mod_amp": {"unit": "duty", "scale": 1, "min": 0, "max": 0.5, "step": 0.005},
    "coaxial.spacing_frac": {"unit": "×R", "scale": 1, "min": 0.05, "max": 2, "step": 0.01},
    "coaxial.lower_collective_offset_deg": {**DEG, "min": -10, "max": 10, "step": 0.1},
    "coaxial.lower_rpm": {"unit": "rpm", "scale": 1, "min": 1000, "max": 40000, "step": 50},
    "coaxial.lower_torque_mod_frac": {"unit": "×Q̄", "scale": 1, "min": 0, "max": 1, "step": 0.01},
    "coaxial.pulse_phase_offset_deg": {**DEG, "min": -180, "max": 180, "step": 1},
    "op.vehicle_mass": {"unit": "kg", "scale": 1, "min": 0.05, "max": 5, "step": 0.01},
    "op.rpm": {"unit": "rpm", "scale": 1, "min": 1000, "max": 40000, "step": 50},
    "op.collective_deg": {**DEG, "min": -10, "max": 25, "step": 0.1},
    "op.torque_mod_frac": {"unit": "×Q̄", "scale": 1, "min": 0, "max": 1, "step": 0.01},
    "op.rho": {"unit": "kg/m³", "scale": 1, "min": 0.8, "max": 1.35, "step": 0.005},
    "op.temperature_C": {"unit": "°C", "scale": 1, "min": -20, "max": 70, "step": 1},
    "analysis.n_blade_elements": {"unit": "", "scale": 1, "min": 2, "max": 64, "step": 1},
    "analysis.n_flexure_elements": {"unit": "", "scale": 1, "min": 1, "max": 32, "step": 1},
    "analysis.n_modes": {"unit": "", "scale": 1, "min": 1, "max": 40, "step": 1},
    "analysis.fatigue_cycles": {"unit": "cycles", "scale": 1, "min": 1e4, "max": 1e10, "log": True},
    "*.chord": {**MM, "min": 1, "max": 80, "step": 0.1},
    "*.twist_deg": {**DEG, "min": -10, "max": 60, "step": 0.1},
    "*.r_frac": {"unit": "r/R", "scale": 1, "min": 0, "max": 1, "step": 0.01},
}

# fields whose string value is a material name -> dropdown
MATERIAL_FIELDS = ["material", "skin_material", "insert_material"]

# internal fields (set by coaxial trim etc.): shown only under an "advanced" fold
ADVANCED = ["op.thrust_share", "op.extra_inflow"]

GROUPS = [
    {"key": "coaxial", "label": "Coaxial (lower rotor)", "when": {"layout": "coaxial"}},
    {"key": "blade", "label": "Blade"},
    {"key": "flexure", "label": "Flexure"},
    {"key": "motor", "label": "Motor"},
    {"key": "op", "label": "Operating point"},
    {"key": "analysis", "label": "Analysis"},
]

# Results quantities that make sense as optimisation targets (dotted paths into Results)
TARGET_SUGGESTIONS = [
    "harmonic.control_ratio", "harmonic.moment_per_torque", "harmonic.hub_moment_mean",
    "harmonic.hub_moment_phase_deg", "harmonic.hub_moment_2p", "harmonic.thrust_1p",
    "harmonic.tilt_equiv_deg", "summary.lag_freq_per_rev", "summary.max_utilisation",
    "checks.max_utilisation", "trim.power", "trim.figure_of_merit", "trim.coning_deg",
    "trim.radial_stretch", "coaxial.control_ratio", "coaxial.control_moment_max",
    "coaxial.yaw_authority", "coaxial.power_total", "coaxial.moment_2p_combined",
    "coaxial.interference_power_factor",
]


def all_hints() -> dict:
    return {"fields": HINTS, "material_fields": MATERIAL_FIELDS, "groups": GROUPS, "advanced": ADVANCED,
            "target_suggestions": TARGET_SUGGESTIONS}
