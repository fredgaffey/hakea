"""E01: beam vs our FEA root 6x6 for the 5" rod winner, compared on the same basis as ccx
(K diagonal with others fixed, and hinge-axis stiffness with others free = 1/(aᵀCa))."""
import json, sys, time, warnings
import numpy as np
warnings.filterwarnings("ignore")
from flexblade import api, fea, fact
from flexblade.schema import Design
d = Design.model_validate(json.load(open("study/designs/5in_tpu_rod_winner.json")))
H0, a = fact.hinge_axis(d)[:2]
def report(tag, K):
    K = np.asarray(K, float); C = np.linalg.inv(K)
    kfree = 1.0 / (a @ C[3:, 3:] @ a)
    print(tag, "diag", np.array2string(np.diag(K), precision=3), "k_hinge_free", round(float(kfree), 4), flush=True)
r = api.evaluate(d, ["trim", "modes"])
report("beam(prestressed)", r.flexure_stiffness)
t = time.time(); rs = fea.root_stiffness(d)
report(f"fea.root_stiffness ({time.time()-t:.0f}s)", rs["K"])
print("fea k_hinge", rs.get("k_hinge"), "k_hinge_clamped", rs.get("k_hinge_clamped"), "point", rs.get("point"), flush=True)
