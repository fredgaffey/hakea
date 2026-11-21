"""v6L: laminated strips on v6, equal EI, pulse-limited robust. Stages: sweep | tol <key> | save <key>."""
import json, random, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import pulse_limited as pl
import laminate_v5L as lv
from flexblade.optimize import get_path, set_design

NOM = pl.load_design("7in_tt_strap_steelx_v6")
CANDS = [(2, 1.3e-4), (3, 1.0e-4), (4, 1.0e-4)]
RES = pl.HERE / "results" / "v6L.json"

def lam(n, t, eta=0.0, base=NOM):
    return lv.lam_design(n, t, eta, base)

def sm(c):
    s = pl.summary(c)
    s["strain_1p"] = max(p.get("strip_strain_amp_1p", 0.0) for p in c.get("points", []))
    return s

def sweep():
    out = {}
    p1 = pl.robust_eval([NOM] + [lam(n, t) for n, t in CANDS])
    out["v6"] = sm(p1[0])
    eta_d = {}
    for (n, t), c in zip(CANDS, p1[1:]):
        k = f"n{n}_t{t*1e3:.2f}"
        out[k] = sm(c)
        eta, kap = lv.eta_for(lam(n, t), out[k]["strain_1p"])
        out[k].update(eta=eta, width=lam(n, t).flexure.width)
        eta_d[k] = lam(n, t, eta)
    p2 = pl.robust_eval(list(eta_d.values()))
    for (k, d), c in zip(eta_d.items(), p2):
        out[k + "+fric"] = sm(c); out[k + "+fric"].update(eta=d.flexure.strip_lam_eta, width=d.flexure.width)
    RES.write_text(json.dumps(out, indent=1, default=float))
    for k, v in out.items():
        print(k, "w=%.2f" % (v.get("width", NOM.flexure.width) * 1e3), "cr_pl=%.4f" % v["cr_pl"], v["feasible_pl"], v["viol_hard"], v["res_which"])
        print("   ", [(p["tag"], p["cr_pl"] and round(p["cr_pl"], 4), p["pulse_binding"], round(p["res_margin"] or 0, 3)) for p in v["points"]])

if __name__ == "__main__":
    if sys.argv[1] == "sweep":
        sweep()
