"""Reference: ideal skewed pin hinge (Hakea-style) at the same hinge axis / skew as
each size's cf_flexure base design, same 1/rev pulse. Gives the control ratio a
perfect lag hinge would deliver — the ceiling the flexures are compared against.

    PYTHONPATH=<worktree>/flexblade ~/venv-flexblade/bin/python study/ideal_hinge.py
writes study/results/ideal_hinge.jsonl
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import run_study as R                      # noqa: E402
from flexblade import rotor                # noqa: E402


def run(D: float, k_hinge: float, ea: float, skew: float = 45.0) -> dict:
    base, cap = R.base_design(D, "cf_flexure")
    d = R.set_design(base, {"blade.elastic_axis_chord_frac": ea, "flexure.skew_deg": skew})
    m = rotor.build_model(d, ideal_hinge=dict(k_hinge=k_hinge, k_rot=1e5, k_trans=1e9))
    aero = rotor.make_aero(m)
    ss = rotor.static_trim(m, aero)
    hd = rotor.harmonic_1p(m, ss, aero)
    md = rotor.modes(m, ss, aero)
    i75 = int(np.argmin(np.abs(m.r_aero - 0.75 * m.R)))
    th = ss.blade_pitch_el
    f1 = ss.rpm / 60
    return dict(diameter_in=D, k_hinge=k_hinge, elastic_axis_chord_frac=ea, skew_deg=skew,
                rpm=ss.rpm, control_ratio=hd.result.control_ratio,
                tilt_equiv_deg=hd.result.tilt_equiv_deg,
                pitch_amp_deg=hd.result.blades[0].pitch_amp_deg,
                lag_amp_deg=hd.result.blades[0].lag_amp_deg,
                lag_per_rev=rotor._pick(md, "lag", ss.Omega),
                modes_per_rev=[float(f / f1) for f in md.freqs_hz],
                split_deg=float(np.degrees(th[0, i75] - th[1, i75])),
                torque_amp=float(abs(hd.dQ)), warnings=ss.warnings)


if __name__ == "__main__":
    out = HERE / "results" / "ideal_hinge.jsonl"
    out.parent.mkdir(exist_ok=True, parents=True)
    with open(out, "w") as fh:
        for D in (4.0, 5.0, 7.0, 10.0, 12.0):
            for ea in (0.25, 0.45):
                r = run(D, 1e-6, ea)
                fh.write(json.dumps(r) + "\n")
                fh.flush()
                print({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()
                       if k != "modes_per_rev"}, flush=True)
