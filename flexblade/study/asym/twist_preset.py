import sys, json
from flexblade import rotor
from flexblade.schema import Design
d = Design.model_validate_json(open(sys.argv[1]).read())
info = {}
d2 = rotor.resolve_strip_preset(d, iters=8, components=(0, 1, 2, 3, 4, 5), info=info)
d2 = d2.model_copy(update=dict(flexure=d2.flexure.model_copy(update=dict(pre_lag_deg="auto")),
                               name="10in_tt_strap_steelx_v9_twistpreset"))
print("resid", [round(x, 4) for x in info["resid_deg"]], "omega", info.get("omega"))
for kb, rows in enumerate(d2.flexure.strip_preset_deg):
    for j, r in enumerate(rows):
        print(f"b{kb+1}.flex{j}", [round(x, 3) for x in r])
open(sys.argv[2], "w").write(d2.model_dump_json(indent=1))
