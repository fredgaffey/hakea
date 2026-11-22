"""Beam-model reference numbers + CAD layout for the whole-rotor CAD FEA of 5in_v8s_2307_m245_301.

Writes fea_cad/layout.json (strip frames, hinge, strap wells in mm; blade-1 frame + blade maps; materials)
and fea_cad/beam_ref.json (hover rpm/thrust, flexure compliance terms at the hinge point, strip stresses
at hover, rotating modes in /rev) for two beam variants:
  design : the design as optimised (slot presets, axial release at the blade end, aramid strap)
  as_cad : what the SolidCraft CAD actually is (straight coaxial slots = presets 0, strips glued at both
           ends = no axial release, rigid clamps); strap kept (it is in the CAD) and also strips-only.
usage (from anywhere): PYTHONPATH=<flexblade pkg root> python beam_ref.py   (laptop: nice -n 19)
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
from flexblade import rotor as R, geometry as G, materials as M          # noqa: E402
from flexblade.schema import Design                                       # noqa: E402
from flexblade.cad.solidcraft import rotor_layout, _blade_map             # noqa: E402

PKG = Path(R.__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / "study"))
from ccx_preset_strip import beam_hover                                   # noqa: E402

DJ = json.loads((HERE / "cad" / "5in_v8s_2307_m245_301.json").read_text())


def terms(K, a):
    C = np.linalg.inv(K)
    Cr = C[3:, 3:]
    s = np.cross([0.0, 1.0, 0.0], a); s /= np.linalg.norm(s)
    y = np.array([0.0, 1.0, 0.0])
    return dict(k_hinge=1 / (a @ Cr @ a), k_cross=1 / (s @ Cr @ s), k_flap=1 / (y @ Cr @ y), Cr=Cr.tolist())


def variant(D, as_cad, strap=True):
    d = D.model_copy(deep=True)
    if as_cad:
        d.flexure.strip_preset_deg = [[[0.0] * 6 for _ in b] for b in D.flexure.strip_preset_deg]
        d.flexure.axial_release = False
        d.analysis.member_anchorage = "none"
    if not strap:
        d.flexure.strap_strands = 0
    return d


def flex_terms(d):
    members, (hp, ax), _ = G.get_members(d)
    a = np.asarray(ax, float) / np.linalg.norm(ax)
    out = {}
    for nm, mm in (("with_strap", members),
                   ("strips_only", [m for m in members if m.kind != "wire" and m.layup.material != "aramid_yarn"])):
        d2 = d.model_copy(deep=True)
        if nm == "strips_only":
            d2.flexure.strap_strands = 0
        K = R.flexure_only_model(d2, mm).stiffness_at(np.asarray(hp, float))
        out[nm] = terms(K, a)
    return out, np.asarray(hp) * 1e3, a


def main():
    lay = rotor_layout(DJ)
    D = lay["D"]
    prm = lay["prm"]
    L = dict(strips=[{k: np.asarray(v).tolist() if not isinstance(v, float) else v for k, v in f.items()}
                     for f in lay["strips"]],
             hinge=lay["hinge"].tolist(), hinge_axis=lay["hinge_axis"].tolist(),
             x_hw=lay["x_hw"], x_bw=lay["x_bw"], r_aero=lay["r_aero"], R=lay["R"], bore=lay["bore"],
             boss_h=lay["boss_h"], prm={k: float(v) for k, v in vars(prm).items() if np.isscalar(v)},
             blade_map=[_blade_map(k, lay["n"]).tolist() for k in range(1, lay["n"] + 1)])
    Ms = {n: M.get_material(n) for n in ("steel_301_fh", "pa6_cf", "aramid_yarn", D.blade.layup.material)}
    L["materials"] = {n: dict(E=m.E1, nu=getattr(m, "nu12", 0.3), rho=m.rho, Xt=m.Xt) for n, m in Ms.items()}
    L["blade_material"] = D.blade.layup.material
    L["n_lam"] = D.flexure.strip_laminae
    L["t_lam"] = D.flexure.thickness * 1e3
    (HERE / "layout.json").write_text(json.dumps(L, indent=1, default=float))
    print("layout", {k: L[k] for k in ("hinge", "hinge_axis", "x_hw", "x_bw", "r_aero", "blade_material")})

    out = {}
    for tag, as_cad in (("design", False), ("as_cad", True)):
        d = variant(D, as_cad)
        if d.flexure.pre_lag_deg == "auto":
            d = R.resolve_pre_lag(d)
        ft, hp, a = flex_terms(d)
        model = R.build_model(d)
        K6 = R.flexure_stiffness(model)
        K6b2 = R.flexure_stiffness(model, blade=2)
        kH = {f"b{b}": (1 / np.diag(np.linalg.inv(k6))[3:]).tolist() for b, k6 in ((1, K6), (2, K6b2))}
        aero = R.make_aero(model)
        ss = R.static_trim(model, aero)
        K6l = R.flexure_stiffness(model, ss)
        md = R.modes(model, ss, aero, n_modes=8)
        rev = ss.rpm / 60
        Fa = np.asarray(ss.F_aero) if ss.F_aero is not None else None
        Tz = float(Fa.reshape(-1, 6)[:, 2].sum()) if Fa is not None and Fa.size % 6 == 0 else None
        strips = beam_hover(d)
        kt = R.strip_kt(d.flexure)
        out[tag] = dict(pre_lag_deg=d.flexure.pre_lag_deg, rpm=ss.rpm, Omega=ss.Omega, thrust_N=Tz, trim_ok=ss.trim_ok,
                        hinge_mm=hp.tolist(), axis=a.tolist(), flex=ft,
                        k_frameH_compliance_unloaded=kH,
                        K6_frameH_unloaded_diag=np.diag(K6).tolist(), K6_frameH_hover_diag=np.diag(K6l).tolist(),
                        modes=[dict(f_hz=float(np.real(f)), per_rev=float(np.real(f)) / rev, label=lb)
                               for f, lb in zip(md.freqs_hz, md.labels)],
                        strips=[dict(tag=s["tag"], kt=kt, surf_hub=s["sm_hub"] / kt / 1e6, surf_blade=s["sm_blade"] / kt / 1e6,
                                     max_xkt=s["sm_all"] / 1e6, memb=[m / 1e6 for m in s["memb"]],
                                     oop=[[o / 1e6 for o in oo] for oo in s["oop"]], nlam=s["nlam"],
                                     F_blade_hub_frame_N=s["fb_hub"][:3].tolist(),
                                     M_blade_hub_frame_Nmm=(s["fb_hub"][3:] * 1e3).tolist()) for s in strips])
        print(tag, json.dumps({k: v for k, v in out[tag].items() if k not in ("strips", "flex")}, default=float))
        print("  flex", {k: {kk: round(vv, 5) for kk, vv in v.items() if kk != "Cr"} for k, v in ft.items()})
        for s in out[tag]["strips"]:
            print("  ", s["tag"], "surf hub/blade MPa", round(s["surf_hub"], 1), round(s["surf_blade"], 1), "memb", np.round(s["memb"], 1))
    (HERE / "beam_ref.json").write_text(json.dumps(out, indent=1, default=float))


if __name__ == "__main__":
    main()
