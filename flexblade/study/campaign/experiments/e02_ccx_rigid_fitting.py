"""E02: after making the hub fitting rigid PA6-CF, does CalculiX's root 6x6 agree with beam/FEA?"""
import json, warnings
import numpy as np
warnings.filterwarnings("ignore")
from flexblade import ccx, fact, api, fea
from flexblade.schema import Design
d = Design.model_validate(json.load(open("study/designs/5in_tpu_rod_winner.json")))
a = fact.hinge_axis(d)[1]
def kfree(K):
    C = np.linalg.inv(np.asarray(K, float)); return 1.0 / (a @ C[3:, 3:] @ a)
r = ccx.check(d, do_modal=False, do_centrifugal=False, workdir="study/campaign/experiments/e02_ccx")
K = r.get("K") or r.get("stiffness") or r.get("K6")
print("ccx keys", sorted(r)[:20])
if K is not None:
    print("ccx diag", np.array2string(np.diag(np.asarray(K)), precision=3), "k_hinge_free", round(kfree(K), 3))
rs = fea.root_stiffness(d); print("fea diag", np.array2string(np.diag(rs["K"]), precision=3), "k_hinge_free", round(kfree(rs["K"]), 3))
