import sys, json, dataclasses
sys.path.insert(0, 'flexblade/study')
import steelx_10in_v5 as s
from flexblade import materials as M
from concurrent.futures import ProcessPoolExecutor
import pulse_limited as pl
b = M.LIBRARY["steel_1095_shim"]
M.LIBRARY["steel_1095_shim_177ph"] = dataclasses.replace(b, name="steel_1095_shim_177ph", E1=204e9, Xt=1650e6, Yt=1650e6, Zt=1650e6, S=950e6, fatigue_strain_amp_allow=0.0030, fatigue_stress_ratio=0.40)
M.LIBRARY["steel_1095_shim_peen"] = dataclasses.replace(b, name="steel_1095_shim_peen", fatigue_strain_amp_allow=0.0028 * 1.2, fatigue_stress_ratio=0.40 * 1.2)
L = {
 "base": {},
 "301fh": {"mat": "steel_301_fh"},
 "177ph_assumed": {"mat": "steel_1095_shim_177ph"},
 "peen+20%_assumed": {"mat": "steel_1095_shim_peen"},
 "t0.08_n8_w17.6": {"thickness": 8e-5, "width": 0.0176},
 "t0.08_n8_w12": {"thickness": 8e-5, "width": 0.012},
 "t0.13_n4_w8.2": {"thickness": 1.3e-4, "strip_laminae": 4, "width": 0.0082},
 "radiused_R20t": {"strip_clamp": "radiused", "strip_clamp_radius": 2e-3},
 "radiused_R10t": {"strip_clamp": "radiused", "strip_clamp_radius": 1e-3},
 "w12_t0.0912": {"width": 0.012, "thickness": 9.12e-5},
}
sel = sys.argv[1:] or list(L)
with ProcessPoolExecutor(3) as ex:
    for t in sel:
        x = dict(L[t])
        try:
            d = pl.load_design("10in_tt_strap_steelx_v5")
            fu = {k: v for k, v in x.items() if k != "mat"}
            fl = d.flexure.model_copy(update=fu)
            if "mat" in x:
                fl = fl.model_copy(update=dict(layup=fl.layup.model_copy(update=dict(material=x["mat"]))))
            d = d.model_copy(update=dict(flexure=fl))
            c = pl.robust_eval([d], js=s.JS, ex=ex)[0]
            r = pl.summary(c)
            print(t, round(r["cr_pl"] or 0, 4), r["feasible_pl"], {k: round(v, 3) for k, v in (r["viol_hard"] or {}).items()}, round(r["res_margin"], 3),
                  [(p["tag"][-14:], round(p["cr_pl"] or 0, 3), p["pulse_binding"]) for p in r["points"]], flush=True)
        except Exception as e:
            print(t, "ERR", repr(e)[:300], flush=True)
