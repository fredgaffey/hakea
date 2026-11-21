"""k_tilt x eta scan (Floquet min zeta incl. hub tilt) for one design over T x J_hub; also the LTI
(isotropic support) cross-check and the motor I_t sensitivity.  Run from flexblade/ with PYTHONPATH=$PWD:$PWD/study.
usage: tilt_whirl_scan.py 7in_v6|10in_v5   -> study/results/tilt_whirl_scan_<name>.json"""
import json, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from flexblade import rotor, robust, stability as S
from flexblade.schema import Airframe, Design, Mount
import tilt_whirl as TW

NAME = sys.argv[1] if len(sys.argv) > 1 else "7in_v6"
KX = np.round(np.arange(1.0, 8.01, 0.25), 2)
ETAS = (0.15, 0.3, 0.5)


def job(a):
    name, T, J = a
    fn, mk = TW.DES[name]
    d0 = Design.model_validate_json((TW.HERE / "designs" / f"{fn}.json").read_text())
    d = [dd for t, dd in robust.variants(d0, thrust_factors=(T,), j_hub=[J])][0]
    if d.flexure.pre_lag_deg == "auto":
        d = rotor.resolve_pre_lag(d)
    model = rotor.build_model(d)
    aero = rotor.make_aero(model)
    ss = rotor.static_trim(model, aero)
    lin = rotor.linearize(model, ss, d.analysis.coriolis)
    m0 = Mount(**mk)
    kt0 = S.mount_tilt_stiffness(m0)
    It0 = 0.5 * (d.motor.J_rotor + d.motor.J_hub) + m0.mass_motor_kg * 0.015 ** 2

    def z(fy=1.15, **kw):
        return TW.ar(model, ss, lin, aero, Airframe(f_hz=TW.FA, f_y_hz=fy * TW.FA, zeta=0.02,
                                                    mount=Mount(**{**mk, **kw})))["zmin"]
    out = dict(T=T, J=J, kt0=kt0, It0=It0, f1p=float(ss.Omega / 2 / np.pi))
    out["lti_iso"] = z(fy=1.0)
    out["It_x"] = {f"{x:g}": z(I_t_kgm2=x * It0) for x in (0.5, 2.0)}
    out["scan"] = {f"{e:g}": [z(k_tilt=x * kt0, eta=e) for x in KX] for e in ETAS}
    return out


if __name__ == "__main__":
    jobs = [(NAME, T, J) for T in (0.7, 1.0, 1.4) for J in (2.5e-6, 3.76e-6, 5.49e-6)]
    with ProcessPoolExecutor(4) as ex:
        res = list(ex.map(job, jobs))
    for r in res:
        print(r["T"], r["J"], "kt0 %.2f It0 %.2e" % (r["kt0"], r["It0"]), "lti_iso %.4f" % r["lti_iso"],
              {k: round(v, 4) for k, v in r["It_x"].items()})
        for e, zs in r["scan"].items():
            bad = [float(k) for k, v in zip(KX, zs) if v < 0.005]
            print("   eta", e, "unstable/low k_x:", bad)
    (TW.HERE / "results" / f"tilt_whirl_scan_{NAME}.json").write_text(
        json.dumps(dict(KX=KX.tolist(), res=res), indent=1, default=float))
