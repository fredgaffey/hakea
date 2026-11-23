"""Hinge offset x tip mass sweep on the 5in lead (Fred 2026-11-23: inertial cyclic ideas - move the lag
'pendulum' toward 1/rev by hinge offset, add tip mass). Hover trim, simulator harmonic response.
usage: python study/inertial/sweep_hinge_tip.py OUT.json"""
import itertools
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from flexblade import api
from flexblade.schema import BalanceMass, Design

BASE = Path("study/designs/5in_v8s_2307_m245_301.json").read_text()
KEYS = ("rpm", "pitch_amp_deg", "lag_amp_deg", "control_ratio", "moment_per_torque", "hub_moment_phase_deg",
        "pitch_freq_per_rev", "flap_freq_per_rev", "lag_freq_per_rev", "near_1p_amp", "pitch_lag_coupling",
        "strip_strain_amp_1p", "pitch_k_eff", "pitch_zeta", "drive_phase_deg")


def run(args):
    hr, tm = args
    d = Design.model_validate_json(BASE)
    R = d.blade.diameter_in * 0.0254 / 2
    d.flexure.hinge_radius = hr
    d.blade.root_cutout_frac = max(d.blade.root_cutout_frac, (hr + 0.004) / R)
    d.blade.balance_masses = [BalanceMass(mass=tm, r_frac=0.95)] if tm else []   # tip mass on the pitch axis
    row = dict(hinge_mm=hr * 1e3, hinge_frac=hr / R, tip_g=tm * 1e3)
    try:
        r = api.evaluate(d)
        s = r.summary.model_dump() if hasattr(r.summary, "model_dump") else dict(r.summary)
        row.update({k: s.get(k) for k in KEYS})
        row["hub_speed_amp_rpm"] = r.harmonic.hub_speed_amp_rpm if r.harmonic else None
        row["thrust"] = s.get("thrust")
    except Exception as e:  # noqa: BLE001
        row["error"] = repr(e)[:300]
    return row


if __name__ == "__main__":
    grid = list(itertools.product((0.01075, 0.017, 0.019, 0.021), (0.0, 0.25e-3, 0.5e-3, 1.0e-3)))
    with ProcessPoolExecutor(16) as ex:
        rows = list(ex.map(run, grid))
    Path(sys.argv[1]).write_text(json.dumps(rows, indent=1, default=str))
    for r in rows:
        print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}))
