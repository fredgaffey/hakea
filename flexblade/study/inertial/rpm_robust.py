"""Robustness of hinge-offset / tip-mass candidates across throttle: vehicle mass x0.6/1.0/1.6 (=thrust per
rotor) -> rpm range; pitch amp, control ratio, drive phase. usage: python study/inertial/rpm_robust.py OUT.json"""
import itertools
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from flexblade import api
from flexblade.schema import BalanceMass, Design

BASE = Path("study/designs/5in_v8s_2307_m245_301.json").read_text()
CANDS = [(0.01075, 0.0), (0.017, 0.0), (0.019, 0.0), (0.019, 0.25e-3), (0.021, 0.25e-3), (0.017, 0.25e-3)]
SCALE = (0.6, 1.0, 1.6)


def run(a):
    (hr, tm), sc = a
    d = Design.model_validate_json(BASE)
    R = d.blade.diameter_in * 0.0254 / 2
    d.flexure.hinge_radius = hr
    d.blade.root_cutout_frac = max(d.blade.root_cutout_frac, (hr + 0.004) / R)
    d.blade.balance_masses = [BalanceMass(mass=tm, r_frac=0.95)] if tm else []
    d.op.vehicle_mass *= sc
    row = dict(hinge_mm=hr * 1e3, tip_g=tm * 1e3, scale=sc)
    try:
        s = api.evaluate(d).summary
        s = s if isinstance(s, dict) else s.model_dump()
        row.update({k: s.get(k) for k in ("rpm", "pitch_amp_deg", "control_ratio", "drive_phase_deg",
                                         "hub_moment_phase_deg", "pitch_freq_per_rev", "strip_strain_amp_1p")})
    except Exception as e:  # noqa: BLE001
        row["error"] = repr(e)[:200]
    return row


if __name__ == "__main__":
    with ProcessPoolExecutor(18) as ex:
        rows = list(ex.map(run, list(itertools.product(CANDS, SCALE))))
    Path(sys.argv[1]).write_text(json.dumps(rows, indent=1))
    for r in rows:
        print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}))
