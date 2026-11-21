"""v5 air-resonance with the damped isotropic TPU mount vs rigid arm (Floquet). Run from flexblade/."""
import sys, json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
sys.path.insert(0, str(Path(__file__).parent))
from flexblade import api, robust
from flexblade.schema import Airframe, Mount
import verify_v5 as v

MOUNT = dict(G_mpa=6.5, eta=0.15, r_in_mm=4, r_out_mm=9, length_mm=10, infill=0.4)


def job(a):
    tag, d, fa, mnt = a
    af = Airframe(f_hz=fa, f_y_hz=1.15 * fa, zeta=0.02, mount=Mount(**MOUNT) if mnt else None)
    d = d.model_copy(update=dict(analysis=d.analysis.model_copy(update=dict(stability="floquet")),
                                 op=d.op.model_copy(update=dict(airframe=af))))
    r = api.evaluate(d, ["trim", "modes", "harmonic"]).model_dump(mode="json")
    s = r["summary"]
    return tag, fa, mnt, s.get("air_resonance_zeta"), {k: s[k] for k in s if "hub" in k or "1p" in k.lower()}


if __name__ == "__main__":
    jobs = [(t, d, fa, m) for t, d in robust.variants(v.NOM) for fa in (190, 225, 260) for m in (0, 1)]
    with ProcessPoolExecutor(3) as ex:
        res = list(ex.map(job, jobs))
    for r in res:
        print(r)
    json.dump(res, open("study/results/mount_v5.json", "w"), indent=1, default=float)
