"""Lead 5in_v8s_2307_m245_301 with the series glue + fitting compliance (FITTING_COMPLIANCE):
nominal T0.7/1.0/1.25 at J 0.91e-6 (materials_5in.py screen basis) -> cr_pl, feasibility, modes
(/rev), stability checks, for: beam as before (OFF), ON with PA6-CF fittings (as designed), ON with
PETG fittings (flexure.fitting_material = prusament_petg). The sa.py score is run separately:
  SA_JHUB=0.34e-6,0.91e-6,2.0e-6 FITTING_COMPLIANCE=1 python i34tools/sa.py file <design> <out>
usage: fc_rescore_5in.py <out.json> [petg_design_out.json]
"""
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust                     # noqa: E402
from flexblade.materials import get_material          # noqa: E402
from flexblade.schema import Design                   # noqa: E402
import run_study as rs                                # noqa: E402

LEAD = HERE / "designs" / "5in_v8s_2307_m245_301.json"
TF = (0.7, 1.0, 1.25)
J = 0.91e-6
PETG = "prusament_petg"
STAB = ("stab", "resonance", "aeroelastic", "air_", "ground", "flutter", "damp")


def ev(arg):
    cfg, tag, dv = arg
    os.environ["FITTING_COMPLIANCE"] = "0" if cfg == "off" else "1"
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    m = rs.metrics(dv, r)
    s = r["summary"]
    rev = (s.get("rpm") or r.get("trim", {}).get("rpm") or 0) / 60.0
    modes = [(md["label"], round(md["freq_hz"] / rev, 3) if rev else None, round(md["damping_ratio"], 4))
             for md in (r.get("modes") or {}).get("modes", r.get("modes") if isinstance(r.get("modes"), list) else [])][:10]
    chk = [(c["name"], round(c["utilisation"], 3)) for c in r["checks"] if any(k in c["name"] for k in STAB)]
    return dict(cfg=cfg, tag=tag, cr_pl=m.get("cr_pl"), feas=m.get("feasible_pl"),
                hard=sorted((m.get("viol_hard") or {}).keys()), bind=m.get("pulse_binding"),
                per_rev={k: v for k, v in s.items() if "per_rev" in k}, modes=modes, stab=chk,
                rpm=s.get("rpm"))


def main():
    out = sys.argv[1]
    d = Design.model_validate_json(LEAD.read_text())
    dp = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(fitting_material=PETG))))
    if len(sys.argv) > 2:
        Path(sys.argv[2]).write_text(dp.model_dump_json(indent=1))
    print("PETG E1", get_material(PETG).E1, "nu", get_material(PETG).nu12, flush=True)
    jobs = [(cfg, tag, dv) for cfg, dd in (("off", d), ("on_pa6cf", d), ("on_petg", dp))
            for tag, dv in robust.variants(dd, TF, None, [J])]
    with ProcessPoolExecutor(int(os.environ.get("FC_WORKERS", 3))) as ex:
        res = list(ex.map(ev, jobs, chunksize=1))
    for r in res:
        print(json.dumps(r, default=float), flush=True)
    Path(out).write_text(json.dumps(res, indent=1, default=float))


if __name__ == "__main__":
    main()
