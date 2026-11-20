"""v5 of the 7in tt_strap_steelx (idea I23): bifilar tension-torsion strap.
Two strands at +-strap_radius (= b, along the rotor axis z) about the radial line through the hinge
point carry the centrifugal load T (∝ Ω²); a pitch rotation θ offsets their blade ends laterally by bθ,
so the pair adds a geometric pitch stiffness k = T b²/L (T = total tension, L = free length) that scales
with Ω² like the CF terms → its contribution to a mode's /rev is rpm-invariant (pendulum). The strands
sit on the z line so a rotation about the skewed hinge axis (in the x-z plane) does not stretch them;
it feels 0.75 × k through the skew (pitch component of the hinge axis ≈ sin(skew)).
Stages: scan | robust <key> | final <key>
    PYTHONPATH=flexblade python flexblade/study/verify_v5.py <stage>
Raw results -> study/results/verify_7in_v5.json. Reuses verify_v4 (make, run_sets, samples, summarize)."""
from __future__ import annotations
import json, sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import verify_v4 as V4                                           # noqa: E402
import verify_v3 as V3                                           # noqa: E402
from flexblade.schema import Design                              # noqa: E402

RES = HERE / "results" / "verify_7in_v5.json"
OUT = HERE / "designs" / "7in_tt_strap_steelx_v5.json"
V4D = Design.model_validate_json((HERE / "designs" / "7in_tt_strap_steelx_v4.json").read_text())
X4 = json.loads(V4D.notes.split("v4 (study/verify_v4.py): ")[1].split("}")[0] + "}")


def load():
    return json.loads(RES.read_text()) if RES.exists() else {}


def save(k, v):
    r = load(); r[k] = v; RES.write_text(json.dumps(r, indent=1, default=float))


def make(t: float, b: float, strand: float = 0.0011, L: float = 0.004) -> Design:
    """v4 knobs, strip thickness t, bifilar strap (2 strands of dia `strand` at +-b, free length L)."""
    x = dict(X4) | {"flexure.thickness": t, "flexure.strap_strand_diameter": strand}
    d = V4.make(x)
    f = d.flexure.model_copy(update=dict(strap_strands=2 if b > 0 else 1, strap_radius=b, strap_length=L))
    return d.model_copy(update=dict(flexure=f))


CANDS = {f"t{t*1e5:.0f}_b{b*1e3:.0f}": (t, b) for t in (1.0e-4, 1.3e-4) for b in (0.0, 0.003, 0.005, 0.007)}


def stage_scan():
    sets = {"v4": V4.make(X4)} | {k: make(*v) for k, v in CANDS.items()}
    res = V4.run_sets(sets)
    save("scan", res)
    for k, c in res.items():
        print(k, c["feasible"], c["viol"], {t: round(v, 3) for t, v in c["cr"].items()}, "fat", round(c["fatigue"], 2),
              "hp", round(c["hp_margin"], 3), c["hp_which"], "res", round(c["res_margin"] or 0, 3), c["res_which"])
        for t, ms in c["modes"].items():
            print("   ", t, [(l, round(f, 2)) for l, f, z in ms[:7]])


def stage_robust(key: str):
    nom = make(*CANDS[key])
    sets = {"nominal": nom} | {k: V3.perturbed(nom, s) for k, s in V4.samples(8, 11).items()}
    r = V4.run_sets(sets)
    out = load().get("robust", {})
    out[key] = dict(summary=V4.summarize(r), samples=r)
    save("robust", out)
    print(key, out[key]["summary"])


def stage_final(key: str):
    t, b = CANDS[key]
    d = make(t, b)
    d = d.model_copy(update=dict(name="7in_tt_strap_steelx_v5", notes=V4D.notes + (
        f"; v5 (study/verify_v5.py, I23 bifilar strap): strips t={t*1e3:.2f} mm, 2 aramid strands "
        f"{d.flexure.strap_strand_diameter*1e3:.1f} mm at +-{b*1e3:.1f} mm (rotor-axis direction) about the radial "
        f"line through the hinge point, free length {d.flexure.strap_length*1e3:.0f} mm: geometric pitch stiffness "
        f"T b^2/L ∝ Ω^2. I23 KILLED (scan in results/verify_7in_v5.json): the strap pitch stiffness reaches the skewed lag-pitch hinge (sin² skew = 0.75), stiffens the 1P cyclic hinge mode (0.57 -> 0.72/rev at b=3 mm, hub mode onto 1/rev) and the 2nd pitch strip mode still sweeps ~1.35x over 0.7-1.4 thrust; NOT a lead design")))
    OUT.write_text(d.model_dump_json(indent=1))
    from flexblade import drawing
    drawing.sheet(d, str(HERE / "results" / "steelx_7in_v5.png"))
    print(V4.envelope(d))


if __name__ == "__main__":
    st = sys.argv[1]
    {"scan": lambda: stage_scan(), "robust": lambda: stage_robust(sys.argv[2]),
     "final": lambda: stage_final(sys.argv[2])}[st]()
