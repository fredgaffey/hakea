"""Model truth for the 10in v5 cyclic gain: G = moment_per_torque * exp(j hub_moment_phase) vs rpm.
Generate (slow, runs rotor.harmonic_1p via api.evaluate) with:
  PYTHONPATH=flexblade python flexblade/study/calib/truth.py  -> truth_10in_v5.json
fit_cyclic/synth read the stored json, so tests never run the model."""
import json, sys
from pathlib import Path
HERE = Path(__file__).parent
OUT = HERE / "truth_10in_v5.json"
THR = [0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4]


def make(J=3.76e-6):
    sys.path.insert(0, str(HERE.parent))
    from flexblade import api, robust
    from flexblade.schema import Design
    d = Design.model_validate_json((HERE.parent / "designs" / "10in_tt_strap_steelx_v5.json").read_text())
    rows = []
    for f in THR:
        v = robust.variants(d, [f], None, [J])[0][1]
        r = api.evaluate(v, ["trim", "harmonic"]).model_dump(mode="json")
        h = r["harmonic"]
        rows.append(dict(throttle=f, rpm=r["trim"]["rpm"], gain=h["moment_per_torque"],
                         phase_deg=h["hub_moment_phase_deg"]))
    OUT.write_text(json.dumps(dict(design="10in_tt_strap_steelx_v5", J_hub=J, rows=rows), indent=1))
    return rows


def load():
    return json.loads(OUT.read_text())["rows"]


if __name__ == "__main__":
    for r in make():
        print(r)
