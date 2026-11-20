"""E08: isolate hub-fitting / rod-anchorage compliance in the ccx root 6x6.
Variants (1-blade root_stiffness case, same mesh):
  real      : hub/spoke clamped, PA6-CF hub fitting deformable (as ccx.check)
  anch_only : hub fitting clamped except within one rod diameter of the rod end faces
  bulk_rigid: hub-fitting nodes further than 1.5 rod diameters from the member interface clamped
              (removes fitting bulk bending/shear, keeps the local rod-end anchorage zone)
  rigid     : whole hub fitting clamped (beam/fea basis)
usage: e08_root_compliance.py design.json workdir [mesh_scale]"""
import json, sys, time, warnings
import numpy as np
from scipy.spatial import cKDTree
warnings.filterwarnings("ignore")
from flexblade import ccx
from flexblade.schema import Design

d = Design.model_validate(json.load(open(sys.argv[1])))
root = sys.argv[2]
ms = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
m1 = ccx.mesh_design(d, blades=[1], h_blade=1.4 * ms, h_flex=0.7 * ms, h_hub=2.6 * ms)
own, names = m1.meta["elem_owner"], m1.meta["owner_name"]


def nodes_of(pred):
    ids = [i for i in names if pred(names[i])]
    return np.unique(m1.elems[np.nonzero(np.isin(own, ids))[0]].ravel())


hubfit = nodes_of(lambda n: n == "blade1_hub_fitting")
mem = nodes_of(lambda n: n.startswith("blade1_flex"))
iface = np.intersect1d(hubfit, mem)
dr = max(d.flexure.layup.insert_diameter * 1e3, d.flexure.thickness * 1e3)
X = m1.nodes
dist = cKDTree(X[iface]).query(X[hubfit])[0] if len(iface) else np.full(len(hubfit), 1e9)
a = np.asarray(m1.meta["hinge_axis"], float)
a /= np.linalg.norm(a)
m1.nsets["BLADESIDE"] = ccx.blade_side_nodes(m1)
base = ccx.hub_side_nodes(m1)
rods = nodes_of(lambda n: n.startswith("blade1_flex") and "rod" in n)
irod = np.intersect1d(hubfit, rods)
drod = cKDTree(X[irod]).query(X[hubfit])[0] if len(irod) else np.full(len(hubfit), 1e9)
print(sorted(set(names.values())), "rod-end iface nodes", len(irod))
print(d.name, "nodes", len(X), "hubfit", len(hubfit), "iface", len(iface), "zone mm", 1.5 * dr, flush=True)
for tag, extra in [("real", []), ("bulk_rigid", hubfit[dist > 1.5 * dr]), ("anch_only", hubfit[drod > d.flexure.layup.insert_diameter * 1e3]),
                   ("rigid", hubfit)]:
    m1.nsets["FIXHUB"] = np.union1d(base, extra).astype(int)
    wd = f"{root}/{d.name}_{tag}"
    ccx.write_inp_model(m1, wd, "root_stiffness", fixed=[("FIXHUB", 1, 3)],
                        rigid=dict(nset="BLADESIDE", ref=m1.meta["hinge"]))
    t = time.time()
    r = ccx.run(wd)
    K = ccx.stiffness_from_run(r)
    kh = 1.0 / (a @ np.linalg.inv(K)[3:, 3:] @ a)
    print(f"{tag:11s} k_hinge_free {kh:.3f} diag", np.array2string(np.diag(K), precision=2),
          f"{time.time() - t:.0f}s", flush=True)
