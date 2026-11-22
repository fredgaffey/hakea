"""CFD hover (OpenFOAM MRF sector, Mac) of knife-rotor CAD blades vs the ideal design blade, 2026-11-23.
Each case = NAME=STL (STL from study/cfd/cad_blade_stl.py, metres) or NAME=ideal (design blade, write_blade_stl).
Same rpm (design hover trim) and mesh settings for all. Rows appended to OUT/summary.json.
usage: python study/cfd/knife_cfd.py DESIGN.json OUTDIR NAME=STL|ideal ... [--np 6] [--cpr 12] [--lvl 4,5] [--iters 1500]
"""
import json
import sys
from pathlib import Path

from flexblade.cfd.validate import hover_case_run
from flexblade.schema import Design

a = sys.argv[1:]
opt = dict(np=6, cpr=12, lvl="4,5", iters=1500)
for k in list(opt):
    if f"--{k}" in a:
        i = a.index(f"--{k}"); opt[k] = type(opt[k])(a[i + 1]); del a[i:i + 2]
d = Design.model_validate_json(Path(a[0]).read_text())
out = Path(a[1]); out.mkdir(parents=True, exist_ok=True)
sf = out / "summary.json"
rows = json.loads(sf.read_text()) if sf.exists() else []
for kv in a[2:]:
    name, stl = kv.split("=", 1)
    try:
        r = hover_case_run(d, name, out, where="mac", np_=opt["np"], stl=None if stl == "ideal" else stl,
                           background_cells_per_R=opt["cpr"], surface_level=tuple(int(v) for v in opt["lvl"].split(",")),
                           iterations=opt["iters"])
    except Exception as e:  # noqa: BLE001
        r = dict(case=name, ok=False, error=repr(e))
    rows = [x for x in rows if x.get("case") != name] + [r]
    print(json.dumps({k: v for k, v in r.items() if k != "log_tail"}), flush=True)
    sf.write_text(json.dumps(rows, indent=1, default=str))
