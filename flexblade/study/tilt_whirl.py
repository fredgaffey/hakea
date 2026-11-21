"""Hub-tilt whirl (soft mount) for 10in v5 (twin washers) and 7in v6 (single washer): Floquet min zeta
with / without the tilt DOFs over T{0.7,1,1.4} x J_hub{2.5,3.76,5.49}e-6, plus a k_tilt x eta sweep.
Run from flexblade/ with PYTHONPATH=$PWD.  usage: tilt_whirl.py [fa_hz]   -> study/results/tilt_whirl.json"""
import json, sys
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from flexblade import rotor, robust, stability as S
from flexblade.schema import Airframe, Design, Mount

HERE = Path(__file__).parent
FA = float(sys.argv[1]) if __name__ == "__main__" and len(sys.argv) > 1 else 225.0
# 10in v5b: twin TPU 85A washers ri 4 / ro 5.8 / t 12 (40 % gyroid) 30 mm apart == one ro 5.8 t 6 washer laterally
M10 = dict(G_mpa=6.5, eta=0.15, r_in_mm=4, r_out_mm=5.8, length_mm=6, infill=0.4, mass_motor_kg=0.124, k_tilt=19.4)
# 7in v6: single washer ri 4 / ro 9 / t 10 (53 kN/m), motor + carrier 45 g, k_tilt 4.4 from the washer formula
M7 = dict(G_mpa=6.5, eta=0.15, r_in_mm=4, r_out_mm=9, length_mm=10, infill=0.4, mass_motor_kg=0.045)
DES = {"10in_v5": ("10in_tt_strap_steelx_v5", M10), "7in_v6": ("7in_tt_strap_steelx_v6", M7)}
KT_X = (1.0, 2.0, 4.0, 8.0)
ETA = (0.15, 0.3)


def ar(model, ss, lin, aero, afs):
    tilt = S.tilt_on(afs)
    lam, B = S._basis(model, ss, lin, model.design.analysis.stability_basis, hub=True, tilt=tilt)
    ok = np.sqrt(np.abs(lam)) > 0.05 * ss.Omega
    fe = float(np.sqrt(abs(lam[ok][0])) / (2 * np.pi))
    rs = S.reduced_system(model, ss, lin, aero, B, fe, hub=True, tilt=tilt)
    I_rt = 0.5 * (rs.M[rs.it[0], rs.it[0]] + rs.M[rs.it[1], rs.it[1]]) if tilt else 0.0
    af = S.make_airframe(afs, model.design.op.vehicle_mass, rs.m_rotor, model.design.motor, I_rt)
    s, V = S.coupled_eigs(rs, af)
    z = S.frame_zeta(rs, af, s, V)
    Mc = S.coupled_matrices(rs, af, average=True)[0]
    n = len(Mc)
    keep = np.abs(s) > 0.05 * ss.Omega
    fr = []
    for k in range(len(s)):
        q = V[:n, k]
        tot = np.real(np.conj(q) @ Mc @ q)
        f = lambda ix: float(np.real(np.conj(q[ix]) @ Mc[np.ix_(ix, ix)] @ q[ix]) / tot)
        fr.append((f(list(rs.ih)), f(list(rs.it)) if tilt else 0.0))
    fr = np.array(fr)
    i = int(np.argmin(np.where(keep, z, np.inf)))
    out = dict(zmin=float(z[i]), f_rot_hz=float(abs(s[i].imag) / 2 / np.pi), hub_frac=float(fr[i, 0]),
               tilt_frac=float(fr[i, 1]), per_rev=float(abs(s[i].imag) / ss.Omega))
    if tilt:
        It = 0.5 * (Mc[rs.it[0], rs.it[0]] + Mc[rs.it[1], rs.it[1]])
        out.update(I_t_total=float(It), k_t=af.kt, f_tilt0_hz=float(np.sqrt(af.kt / It) / 2 / np.pi))
        wh = []
        for k in np.where((fr[:, 1] > 0.3) & (s.imag > 0) & keep)[0]:
            fam = np.abs(s[k].imag + ss.Omega * np.array([1, -1])) / 2 / np.pi
            wh.append((round(float(min(fam, key=lambda x: abs(x - out["f_tilt0_hz"]))), 1),
                       round(float(z[k]), 4), round(float(fr[k, 1]), 2)))
        out["whirl"] = sorted(wh)
    return out


def job(a):
    name, T, J = a
    fn, mk = DES[name]
    d0 = Design.model_validate_json((HERE / "designs" / f"{fn}.json").read_text())
    d = [dd for t, dd in robust.variants(d0, thrust_factors=(T,), j_hub=[J])][0]
    d = d.model_copy(update=dict(analysis=d.analysis.model_copy(update=dict(stability="floquet"))))
    if d.flexure.pre_lag_deg == "auto":
        d = rotor.resolve_pre_lag(d)
    model = rotor.build_model(d)
    aero = rotor.make_aero(model)
    ss = rotor.static_trim(model, aero)
    lin = rotor.linearize(model, ss, d.analysis.coriolis)
    kt0 = S.mount_tilt_stiffness(Mount(**mk))
    res = dict(design=name, T=T, J=J, rpm=float(ss.rpm), f1p=float(ss.Omega / 2 / np.pi))

    def afs(**kw):
        return Airframe(f_hz=FA, f_y_hz=1.15 * FA, zeta=0.02, mount=Mount(**{**mk, **kw}))
    res["no_tilt"] = ar(model, ss, lin, aero, afs(tilt=False))
    res["tilt"] = ar(model, ss, lin, aero, afs())
    res["sweep"] = {f"{x:g}x/{e:g}": ar(model, ss, lin, aero, afs(k_tilt=x * kt0, eta=e))["zmin"]
                    for x in KT_X for e in ETA if (x, e) != (1.0, 0.15)}
    return res


if __name__ == "__main__":
    jobs = [(n, T, J) for n in DES for T in (0.7, 1.0, 1.4) for J in (2.5e-6, 3.76e-6, 5.49e-6)]
    with ProcessPoolExecutor(4) as ex:
        res = []
        for r in ex.map(job, jobs):
            res.append(r)
            print(r["design"], r["T"], r["J"], "1P %.0f" % r["f1p"], "noTilt %.4f" % r["no_tilt"]["zmin"],
                  "tilt %.4f" % r["tilt"]["zmin"], {k: r["tilt"][k] for k in ("f_rot_hz", "hub_frac", "tilt_frac",
                                                                             "f_tilt0_hz", "whirl")},
                  r["sweep"], flush=True)
    (HERE / "results" / f"tilt_whirl_fa{FA:g}.json").write_text(json.dumps(res, indent=1, default=float))
