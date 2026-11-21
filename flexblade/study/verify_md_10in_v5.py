"""Print floq / ccx summary numbers from results/verify_10in_tt_strap_steelx_v5.json (for verify_10in_v5.md)."""
import json
from pathlib import Path
R = json.loads((Path(__file__).parent / "results/verify_10in_tt_strap_steelx_v5.json").read_text())
cx = R["ccx"]["ratio"]
print("ccx raw", {k: round(v, 3) for k, v in cx.items()}, "x8", {k: round(8 * v, 3) for k, v in cx.items()})
by = {}
for r in R["floq"]:
    zaf = r["zaf"]
    for fa, za, zs in r["sweep"]:
        k = (zaf, r["tag"])
        cur = by.get(k)
        if za is not None and (cur is None or za < cur[0]):
            by[k] = (za, fa, r["J"], zs)
for k in sorted(by, key=str):
    print(k, "min air zeta %.4f at f_x %s J %.3g (zeta_min struct %s)" % (by[k][0], by[k][1], by[k][2], by[k][3]))
for zaf in (0.02, 0.05, "mount"):
    v = [by[k][0] for k in by if k[0] == zaf]
    print(zaf, "overall min", min(v))
zs = [zs for r in R["floq"] for _, _, zs in r["sweep"] if zs is not None]
print("struct zeta_min", min(zs))
