"""E05: CFD validation queue on the Mac — fine hover + 1/rev pitching (kOmegaSSTLM) for the 5" design
and the ideation blade-shape candidates B01, B04, B02. Sequential, one heavy case at a time."""
import json, sys
from flexblade.cfd import validate
from flexblade.schema import Design
c = json.load(open("study/ideation/concepts.json"))
cs = c if isinstance(c, list) else c.get("concepts", c)
cs = cs if isinstance(cs, list) else list(cs.values())
designs = [Design.model_validate(json.load(open("study/designs/5in_tpu_rod_winner.json")))]
for x in cs:
    if x.get("cfd_candidate") and x.get("best_design"):
        d = Design.model_validate(x["best_design"]); d.name = f"ideation_{x['id']}"; designs.append(d)
print("designs:", [d.name for d in designs], flush=True)
rows = validate.blade_shapes(designs, where="mac", pitching=True, np_=10,
                             out_dir="study/campaign/experiments/e05_cfd",
                             hover_kw=dict(background_cells_per_R=12, surface_level=(4, 5), iterations=2000))
print(json.dumps(rows, default=str)[:4000], flush=True)
print("done", flush=True)
