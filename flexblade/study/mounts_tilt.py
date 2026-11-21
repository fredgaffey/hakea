"""Twin-washer mount tilt Floquet check for 7in_v6 (v6b mount) and 10in_v5 (v5c mount): min zeta over
T{0.7,1,1.4} x J_hub{2.5,3.76,5.49}e-6 x arm {190,225,260} Hz at 1x and 2x estimated motor I_t, + hub mode.
Run from flexblade/ with PYTHONPATH=$PWD:$PWD/study:$PWD/study/build.   -> study/results/mounts_tilt.json"""
import json, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from flexblade import rotor, robust, stability as S
from flexblade.schema import Airframe, Design, Mount
import tilt_whirl as TW
import mount_twin as MT

SIZES = {  # design key -> (washer dict, mass on mount kg, J_hub itemised)
    "7in_v6": (dict(ri=4, ro=8, t=22, d=76), 0.055),
    "10in_v5": (dict(ri=4, ro=5.8, t=12, d=84), 0.126),
}
ARMS = (190.0, 225.0, 260.0)


def mk(name):
    P, m = SIZES[name]
    return dict(G_mpa=6.5, eta=0.15, r_in_mm=P["ri"], r_out_mm=P["ro"], length_mm=P["t"] / 2, infill=0.4,
                mass_motor_kg=m, k_tilt=MT.props(**P)["k_t"])


def job(a):
    name, T, J = a
    fn, _ = TW.DES[name]
    mnt = mk(name)
    d0 = Design.model_validate_json((TW.HERE / "designs" / f"{fn}.json").read_text())
    d = [dd for t, dd in robust.variants(d0, thrust_factors=(T,), j_hub=[J])][0]
    d = d.model_copy(update=dict(analysis=d.analysis.model_copy(update=dict(stability="floquet"))))
    if d.flexure.pre_lag_deg == "auto":
        d = rotor.resolve_pre_lag(d)
    model = rotor.build_model(d)
    aero = rotor.make_aero(model)
    ss = rotor.static_trim(model, aero)
    lin = rotor.linearize(model, ss, d.analysis.coriolis)
    Jp = d.motor.J_rotor + d.motor.J_hub
    It0 = 0.5 * Jp + mnt["mass_motor_kg"] * 0.015 ** 2
    out = dict(design=name, T=T, J=J, f1p=float(ss.Omega / 2 / np.pi), It0=It0, Jp=Jp,
               rule1=MT.rule(ss.Omega, It0, Jp), rule2=MT.rule(ss.Omega, 2 * It0, Jp), kt=mnt["k_tilt"])
    for fa in ARMS:
        for ix in (1.0, 2.0, 3.0):
            af = Airframe(f_hz=fa, f_y_hz=1.15 * fa, zeta=0.02, mount=Mount(**{**mnt, "I_t_kgm2": ix * It0}))
            r = TW.ar(model, ss, lin, aero, af)
            out[f"z_{fa:g}_It{ix:g}"] = r["zmin"]
        h = S.mount_modes(mnt["mass_motor_kg"], 0.3, (mnt["mass_motor_kg"] + 0.3) * (2 * np.pi * fa) ** 2, 0.02,
                          S.mount_stiffness(Mount(**mnt)), 0.15)
        out[f"hub_{fa:g}"] = [float(h[0]), float(h[1])]
    return out


if __name__ == "__main__":
    names = sys.argv[1:] or list(SIZES)
    jobs = [(n, T, J) for n in names for T in (0.7, 1.0, 1.4) for J in (2.5e-6, 3.76e-6, 5.49e-6)]
    with ProcessPoolExecutor(4) as ex:
        res = list(ex.map(job, jobs))
    for r in res:
        print(r["design"], r["T"], r["J"], "1P %.0f" % r["f1p"], "kt %.1f rule1 %.1f rule2 %.1f" % (r["kt"], r["rule1"], r["rule2"]),
              {k[2:]: round(v, 4) for k, v in r.items() if k.startswith("z_")},
              {k: [round(x, 3) for x in v] for k, v in r.items() if k.startswith("hub_")}, flush=True)
    (TW.HERE / "results" / f"mounts_tilt_{'_'.join(names)}.json").write_text(json.dumps(res, indent=1, default=float))
