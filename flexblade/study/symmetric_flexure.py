"""Blade-2 flexure arrangement (flexure.blade2_flexure) vs 1P per-blade response and hub forces.
v8, T1.0, precone 0, pulse scale fixed at the v8 s* (0.907) for comparability.
PYTHONPATH=flexblade python flexblade/study/symmetric_flexure.py [case idx ...] -> results/symmetric_flexure.json"""
from __future__ import annotations
import json, sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import pulse_limited as pl
from two_per_rev import fixed
from flexblade import robust, rotor

S = 0.907
CASES = [("mirror_z", "skewed_cross_blade", True), ("mirror_z", "skewed_cross_blade", False),
         ("rot_x", "skewed_cross_blade", False), ("c2", "skewed_cross_blade", False),
         ("mirror_z", "skewed_cross_blade_sym", False), ("rot_x", "skewed_cross_blade_sym", False)]


def case(c):
    mode, topo, presets = c[:3]
    bo = dict(c[3]) if len(c) > 3 else {}
    tag = str(bo)
    d0 = pl.load_design("10in_tt_strap_steelx_v8")
    fu = dict(blade2_flexure=mode, topology=topo, pitch_link_stiffness=bo.pop("k_link", 0.0))
    if not presets:
        fu["strip_preset_deg"] = []
    d0 = d0.model_copy(update=dict(flexure=d0.flexure.model_copy(update=fu),
                                   blade=type(d0.blade).model_validate({**d0.blade.model_dump(), **bo})))
    d = robust.variants(d0, thrust_factors=(1.0,))[0][1]
    if d.flexure.pre_lag_deg == "auto":
        d = rotor.resolve_pre_lag(d)
    model = rotor.build_model(d)
    aero = rotor.make_aero(model)
    ss = rotor.static_trim(model, aero)
    lin = rotor.linearize(model, ss, d.analysis.coriolis)
    hd = rotor.harmonic_1p(model, ss, aero, lin=lin)
    U = S * hd.U
    hubd = 6 * model.hub + 5
    bl, th = [], []
    for bi in model.blades:
        u, rv = rotor.blade_local_motion_c(model, U, bi)
        i75 = np.argmin(np.abs(bi.chain_r - 0.75 * model.R))
        th.append(rv[i75, 0])
        c3 = lambda z: f"{abs(z):.2f}@{np.degrees(np.angle(z)):.0f}"
        bl.append(dict(flap=c3(np.rad2deg(1) * (rv[0, 1])), lag=c3(np.rad2deg(1) * (rv[0, 2] - U[hubd])),
                       pitch=c3(np.rad2deg(1) * (rv[i75, 0]))))
    return dict(mode=mode, topo=topo, presets=presets, blade_over=tag, rpm=ss.rpm, pre_lag=str(d.flexure.pre_lag_deg), blades=bl,
                cyclic_pitch_deg=float(np.rad2deg(abs(th[0] - th[1]) / 2)),
                collective_1p_pitch_deg=float(np.rad2deg(abs(th[0] + th[1]) / 2)),
                hubF_1p_fixed=fixed(S * hd.hub_force_H, 1), hubM_1p_fixed=fixed(S * hd.hub_moment_H, 1))


if __name__ == "__main__":
    BM = lambda m, z: [dict(mass=m, r_frac=0.2, chord_offset=0.0, normal_offset=z),
                       dict(mass=m, r_frac=0.2, chord_offset=0.0, normal_offset=-z)]
    CASES += [("mirror_z", "skewed_cross_blade", False, dict(balance_masses=BM(m, 0.01))) for m in (1e-6, 0.003)]
    CASES += [("mirror_z", "skewed_cross_blade", False, dict(elastic_axis_chord_frac=x)) for x in (0.25, 0.45)]
    BC = lambda m, y: [dict(mass=m, r_frac=0.2, chord_offset=y, normal_offset=0.0),
                       dict(mass=m, r_frac=0.2, chord_offset=-y, normal_offset=0.0)]
    CASES += [("mirror_z", "skewed_cross_blade", False, dict(balance_masses=BC(m, 0.01))) for m in (0.001, 0.002)]
    CASES += [(mo, "skewed_cross_blade", False, dict(balance_masses=BM(1e-6, 0.01))) for mo in ("rot_x", "c2")]
    CASES += [("mirror_z", "skewed_cross_blade", False, dict(balance_masses=BM(5e-4, 0.01))),
              ("mirror_z", "skewed_cross_blade", True, dict(balance_masses=BM(1e-6, 0.01))),
              ("mirror_z", "skewed_cross_blade_sym", False, dict(balance_masses=BM(1e-6, 0.01))),
              ("mirror_z", "skewed_cross_blade", False, dict(balance_masses=BM(1e-6, 0.01), precone_deg=1.3))]
    CASES += [("mirror_z", "skewed_cross_blade", True, dict(k_link=k)) for k in (0.3, 1.0, 3.0, 10.0)]
    CASES += [("mirror_z", "skewed_cross_blade", True, dict(k_link=k)) for k in (100.0, 1000.0)]
    CASES += [("rot_x", "skewed_cross_blade", False, dict(k_link=100.0)),
              ("mirror_z", "skewed_cross_blade", False, dict(k_link=100.0, balance_masses=BM(1e-6, 0.01)))]
    sel = [CASES[int(i)] for i in sys.argv[1:]] or CASES
    with ProcessPoolExecutor(2) as ex:
        res = list(ex.map(case, sel))
    out = HERE / "results" / "symmetric_flexure.json"
    old = json.loads(out.read_text()) if out.exists() else []
    key = lambda r: (r["mode"], r["topo"], r["presets"], r.get("blade_over", "{}"))
    keep = [r for r in old if key(r) not in {key(x) for x in res}]
    out.write_text(json.dumps(keep + res, indent=1, default=float))
    for r in res:
        print(r["mode"], r["topo"][-4:], r["presets"], r["blade_over"][:60], "prelag", r["pre_lag"][:6], r["blades"],
              "cyc %.2f coll %.2f" % (r["cyclic_pitch_deg"], r["collective_1p_pitch_deg"]),
              "F1", {k: [round(x, 2) for x in v] for k, v in r["hubF_1p_fixed"].items()},
              "M1", {k: [round(x, 3) for x in v] for k, v in r["hubM_1p_fixed"].items()})
