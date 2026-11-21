"""Integrated SYSTEM check, all pieces on at once: flexure + twin-washer tilt-stiff mount + arm f_x/f_y +
ESC drive with rev-averaged governor + J_hub + motor I_t.  Run from the repo root (worktree) with
PYTHONPATH=$PWD/flexblade:$PWD/flexblade/study:$PWD/flexblade/study/build.
Usage: system_check_10in_v5.py [10in_v5|7in_v6] [one]   (system_check_7in_v6.py is the same script, 7in default)
-> study/results/system_check_<name>.json.   Pulse amplitudes: the 1P cyclic moment/force is constant in the fixed
frame, so hub tilt = M/k_t and lateral = F/k_series (quasi-static, mount in series with the arm; rotor mass
neglected on the arm).  Imbalance: 2 g mm at 1P on the 2-DOF hub/arm with the complex mount stiffness."""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from flexblade import rotor, robust, stability as S
from flexblade.schema import Airframe, Design, Drive, Mount
import tilt_whirl as TW
import mounts_tilt as MTL
import pulse_limited as pl

CFG = {  # name -> arms (spec first), governor bandwidth
    "10in_v5": dict(arms=(120.0, 150.0, 200.0), bw=20.0),
    "7in_v6": dict(arms=(180.0, 200.0, 225.0, 260.0), bw=27.0),
}
TAUS = (0.5e-3, 2e-3)
JS = (2.5e-6, 3.76e-6, 5.49e-6)
IMB = 2e-6   # kg m (2 g mm)


def with_esc(d, esc):
    return d.model_copy(update=dict(motor=d.motor.model_copy(update=dict(esc=esc))))


def imbalance_um(mnt, m_vehicle, f_arm, Om):
    km = S.mount_stiffness(Mount(**mnt))
    mh = mnt["mass_motor_kg"]
    ma = 0.25 * m_vehicle - mh
    ka = 0.25 * m_vehicle * (2 * np.pi * f_arm) ** 2
    e = mnt["eta"]
    K = np.array([[km * (1 + 1j * e), -km * (1 + 1j * e)], [-km * (1 + 1j * e), km * (1 + 1j * e) + ka * (1 + 0.04j)]])
    A = K - Om ** 2 * np.diag([mh, ma])
    x = np.linalg.solve(A, np.array([IMB * Om ** 2, 0]))
    return float(abs(x[0]) * 1e6)


def job(a):
    name, T, J = a
    c = CFG[name]
    fn, _ = TW.DES[name]
    mnt = MTL.mk(name)
    d0 = Design.model_validate_json((TW.HERE / "designs" / f"{fn}.json").read_text())
    d = [dd for t, dd in robust.variants(d0, thrust_factors=(T,), j_hub=[J])][0]
    ps = pl._eval_point(d)   # pulse_scale s*: cr_pl = cr x s* (blade stress / fatigue / jitter limits)
    d = d.model_copy(update=dict(analysis=d.analysis.model_copy(update=dict(stability="floquet"))))
    if d.flexure.pre_lag_deg == "auto":
        d = rotor.resolve_pre_lag(d)
    model = rotor.build_model(d)
    aero = rotor.make_aero(model)
    ss = rotor.static_trim(model, aero)
    lin = rotor.linearize(model, ss, d.analysis.coriolis)
    Jp = d.motor.J_rotor + d.motor.J_hub
    It0 = 0.5 * Jp + mnt["mass_motor_kg"] * 0.015 ** 2
    W = d.op.vehicle_mass
    out = dict(pulse_scale=float(ps['pulse_scale']), cr_pl_nom=float(ps['cr_pl']), T=T, J=J, rpm=ss.rpm, f1p=float(ss.Omega / 2 / np.pi), It0=It0, kt=mnt["k_tilt"], rows=[])
    cfgs = [("free", None)] + [(f"rev_avg_{t * 1e3:g}ms", Drive(bandwidth_hz=c["bw"], latency_s=t, rpm_filter="rev_avg"))
                               for t in TAUS]
    ref = None
    for tag, esc in cfgs:
        model.design = with_esc(d, esc)
        hd = rotor.harmonic_1p(model, ss, aero, lin=lin)
        hr = hd.result
        Mc = hr.hub_moment_mean * np.exp(1j * np.radians(hr.hub_moment_phase_deg))
        ref = Mc if ref is None else ref
        Fm, _ = rotor._fixed_frame(hd.hub_force_H)
        sc = float(ps['pulse_scale'])        # the pulse actually commanded at cr_pl
        Fmag = float(np.hypot(*Fm)) * sc
        base = dict(tag=tag, gain=float(abs(Mc / ref)), dphase=float(np.degrees(np.angle(Mc / ref))),
                    cr_pl=float(hr.control_ratio) * sc, M=float(hr.hub_moment_mean) * sc, F=Fmag,
                    tilt_deg=float(np.degrees(hr.hub_moment_mean * sc / mnt["k_tilt"])))
        for fa in c["arms"]:
            ka = 0.25 * W * (2 * np.pi * fa) ** 2
            kser = 1 / (1 / S.mount_stiffness(Mount(**mnt)) + 1 / ka)
            for ix in (1.0, 2.0):
                af = Airframe(f_hz=fa, f_y_hz=1.15 * fa, zeta=0.02,
                              mount=Mount(**{**mnt, "I_t_kgm2": ix * It0}))
                zmin, smin, s, _ = S.air_resonance(model, ss, lin, aero, af, d.analysis.stability_basis)
                s = s[np.abs(s) > 1e-6 * ss.Omega]
                real = s[np.abs(s.imag) <= np.abs(s.real)]
                ndiv = int((real.real > 1e-3 * ss.Omega).sum())
                out["rows"].append(dict(base, arm=fa, It=ix, zmin=float(zmin), fmin=float(abs(smin.imag) / 2 / np.pi),
                                        max_re=float(s.real.max()), ndiv=ndiv, lat_um=float(Fmag / kser * 1e6),
                                        imb_um=imbalance_um(mnt, W, fa, ss.Omega)))
    return out


def main(name):
    jobs = [(name, T, J) for T in (0.7, 1.0, 1.4) for J in JS]
    if len(sys.argv) > 1 and sys.argv[-1] == "one":
        jobs = jobs[:1]
    with ProcessPoolExecutor(4) as ex:
        res = list(ex.map(job, jobs))
    (TW.HERE / "results" / f"system_check_{name}.json").write_text(json.dumps(res, indent=1, default=float))
    for r in res:
        for x in r["rows"]:
            if x["It"] == 1.0 or x["zmin"] < 0:
                print(name, f"T{r['T']:g} J{r['J']:.3g}", x["tag"], x["arm"], f"It{x['It']:g}", f"z {x['zmin']:+.4f} nd {x['ndiv']}",
                      f"G {x['gain']:.3f} {x['dphase']:+.1f} cr {x['cr_pl']:.3f} tilt {x['tilt_deg']:.2f} lat {x['lat_um']:.0f} imb {x['imb_um']:.0f}", flush=True)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in CFG else "10in_v5")
