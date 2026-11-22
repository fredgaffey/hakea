"""FITTING_COMPLIANCE check against the whole-rotor CalculiX case 1 (results/m05_summary.json).

Beam reference = beam_ref.py "as_cad, strips only" (rotor.flexure_only_model at the hinge point).
The model is re-run with the series glue-land + fitting-arm springs (flexblade/fitting_compliance.py)
at the CalculiX inputs: epoxy 3 GPa / 0.35 over the 8 mm lands, glue gap 0.325 mm per face (the
lamina-at-mid-plane model), isotropic fittings 210 / 7.5 / 2.0 GPa. No fitted parameters.
usage: PYTHONPATH=<flexblade pkg root> python fc_calib.py
"""
import json
import os
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
from flexblade import rotor as R, geometry as G, fitting_compliance as FC           # noqa: E402
from flexblade.cad.solidcraft import rotor_layout                                   # noqa: E402
from beam_ref import variant, terms, DJ                                             # noqa: E402

FEA = json.loads((HERE / "results" / "m05_summary.json").read_text())
CASES = (("STIFF", 210e9), ("PACF", 7.5e9), ("PETG", 2.0e9))


def k3(d):
    members, (hp, ax), _ = G.get_members(d)
    a = np.asarray(ax, float) / np.linalg.norm(ax)
    mm = [m for m in members if m.kind != "wire" and m.layup.material != "aramid_yarn"]
    d2 = d.model_copy(deep=True)
    d2.flexure.strap_strands = 0
    t = terms(R.flexure_only_model(d2, mm).stiffness_at(np.asarray(hp, float)), a)
    return {k: t[k] for k in ("k_hinge", "k_cross", "k_flap")}


def run(env):
    old = {k: os.environ.get(k) for k in env}
    os.environ.update({k: str(v) for k, v in env.items()})
    try:
        return k3(D)
    finally:
        for k, v in old.items():
            os.environ.pop(k) if v is None else os.environ.__setitem__(k, v)


lay = rotor_layout(DJ)
D = variant(lay["D"], True, strap=False)
if D.flexure.pre_lag_deg == "auto":
    D = R.resolve_pre_lag(D)
print("blade arm end (FEA BC) mm", lay["r_aero"] + 3.0)
print("layout r_aero", lay["r_aero"], "prm slot_t", lay["prm"].slot_t, "land", lay["prm"].land)
base = run({"FITTING_COMPLIANCE": "0"})
print("beam as-CAD strips only", {k: round(v, 5) for k, v in base.items()})
out = dict(beam=base, cases={})
on = {"FITTING_COMPLIANCE": "1", "FITTING_GLUE_T": "0.325e-3",
      "FITTING_BLADE_ARM_END": (lay["r_aero"] + 3.0) * 1e-3}     # CalculiX: blade rigid outboard of this
for tag, E in CASES:
    f = FEA[f"c1_{tag}"]
    row = {}
    for nm, extra in (("full", {"FITTING_E": E}), ("glue_only", {"FITTING_E": "inf"}),
                      ("arms_only", {"FITTING_E": E, "FITTING_GLUE_E": "inf"})):
        if nm == "glue_only" and tag != "STIFF":
            continue
        k = run({**on, **extra})
        row[nm] = {q: dict(model=k[q], ratio_model=k[q] / base[q], ratio_ccx=f[q] / base[q],
                           model_over_ccx=k[q] / f[q]) for q in k}
    out["cases"][tag] = row
    for nm, r in row.items():
        print(f"{tag:5s} {nm:9s} " + "  ".join(f"{q[2:]}: model/beam {v['ratio_model']:.3f} ccx/beam "
                                             f"{v['ratio_ccx']:.3f} model/ccx {v['model_over_ccx']:.3f}"
                                             for q, v in r.items()))
(HERE / "results" / "fc_calib.json").write_text(json.dumps(out, indent=1, default=float))
