"""E08b: beam k_hinge_free (hinge axis a, other DOF free) with member_anchorage none/hub/both,
unprestressed (ccx basis) and prestressed at trim (E01 basis). usage: designs..."""
import json, sys, warnings
import numpy as np
warnings.filterwarnings("ignore")
from flexblade import fact, rotor, api
from flexblade.schema import Design

for f in sys.argv[1:]:
    d0 = Design.model_validate(json.load(open(f)))
    a = fact.hinge_axis(d0)[1]
    out = []
    for anc in ("none", "hub", "both"):
        d = d0.model_copy(update=dict(analysis=d0.analysis.model_copy(update=dict(member_anchorage=anc))))
        K = rotor.flexure_stiffness(rotor.build_model(d))
        kh = 1.0 / (a @ np.linalg.inv(K)[3:, 3:] @ a)
        out.append(f"{anc}: k0 {kh:.3f}")
        if "--trim" in f or len(sys.argv) == 2:
            r = api.evaluate(d, ["trim"])
            Kp = np.asarray(r.flexure_stiffness)
            out[-1] += f" kpre {1.0 / (a @ np.linalg.inv(Kp)[3:, 3:] @ a):.3f}"
    print(d0.name, " | ".join(out), flush=True)
