"""ESC/motor drive impedance on the rotor spin DOF (schema.Drive = motor.esc): 1P authority change vs
free spin, and the single-washer residual real root, for 10in v5 and 7in v6 at T{0.7,1,1.4}.
Run from flexblade/ with PYTHONPATH=$PWD.  -> study/results/drive_impedance.json"""
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from flexblade import rotor, robust, stability as S
from flexblade.schema import Airframe, Design, Drive, Mount

HERE = Path(__file__).parent
M10 = dict(G_mpa=6.5, eta=0.15, r_in_mm=4, r_out_mm=5.8, length_mm=6, infill=0.4, mass_motor_kg=0.124)
M7 = dict(G_mpa=6.5, eta=0.15, r_in_mm=4, r_out_mm=9, length_mm=10, infill=0.4, mass_motor_kg=0.045)
DES = {"10in_v5": ("10in_tt_strap_steelx_v5", M10), "7in_v6": ("7in_tt_strap_steelx_v6", M7)}
CFG = {"free": None, "emf": Drive(),
       "v30_0.5ms": Drive(bandwidth_hz=30, latency_s=5e-4), "v30_2ms": Drive(bandwidth_hz=30, latency_s=2e-3),
       "v80_0.5ms": Drive(bandwidth_hz=80, latency_s=5e-4), "v80_2ms": Drive(bandwidth_hz=80, latency_s=2e-3),
       "cc_free": Drive(current_control=True), "cc50_1ms": Drive(current_control=True, bandwidth_hz=50, latency_s=1e-3)}


def with_esc(d, esc):
    return d.model_copy(update=dict(motor=d.motor.model_copy(update=dict(esc=esc))))


def job(a):
    name, T = a
    fn, mk = DES[name]
    d0 = Design.model_validate_json((HERE / "designs" / f"{fn}.json").read_text())
    d = dict(robust.variants(d0, thrust_factors=(T,)))[f"T{T:g}"]
    if d.flexure.pre_lag_deg == "auto":
        d = rotor.resolve_pre_lag(d)
    model = rotor.build_model(d)
    aero = rotor.make_aero(model)
    ss = rotor.static_trim(model, aero)
    lin = rotor.linearize(model, ss, d.analysis.coriolis)
    J = S.spin_inertia(model, ss, lin)
    afs = Airframe(f_hz=225.0, mount=Mount(**mk, tilt=False))
    out = dict(rpm=ss.rpm, J_spin=J, cfg={})
    ref = None
    for tag, esc in CFG.items():
        model.design = with_esc(d, esc)
        hd = rotor.harmonic_1p(model, ss, aero, lin=lin)
        hr = hd.result
        Mc = hr.hub_moment_mean * np.exp(1j * np.radians(hr.hub_moment_phase_deg))
        ref = Mc if ref is None else ref
        _, _, s, _ = S.air_resonance(model, ss, lin, aero, afs, d.analysis.stability_basis)
        s = s[np.abs(s) > 1e-6 * ss.Omega]
        real = s[np.abs(s.imag) <= np.abs(s.real)]
        r = dict(gain=float(abs(Mc / ref)), dphase_deg=float(np.degrees(np.angle(Mc / ref))),
                 cr=float(hr.control_ratio), hub_rpm=float(hr.hub_speed_amp_rpm),
                 max_real_root=float(real.real.max()) if len(real) else None,
                 max_re=float(s.real.max()))
        dp = S.drive_params(model.design.motor, J) if esc is not None else None
        if dp is not None:
            Zd = S.drive_impedance(dp, 1j * ss.Omega, emf=True) / (1j * ss.Omega * J)
            r.update(kp=dp["kp"], ki=dp["ki"], c=dp["c"], Zrel_mag=float(abs(Zd)),
                     Zrel_phase_deg=float(np.degrees(np.angle(Zd))))
        out["cfg"][tag] = r
    return name, T, out


if __name__ == "__main__":
    jobs = [(n, T) for n in DES for T in (0.7, 1.0, 1.4)]
    with ProcessPoolExecutor(6) as ex:
        res = list(ex.map(job, jobs))
    js = {f"{n}:T{T:g}": o for n, T, o in res}
    (HERE / "results" / "drive_impedance.json").write_text(json.dumps(js, indent=1))
    for k, o in js.items():
        print(k, f"rpm {o['rpm']:.0f} J {o['J_spin']:.3g}")
        for t, r in o["cfg"].items():
            print(f"  {t:10s} |G| {r['gain']:.3f} dphi {r['dphase_deg']:+6.1f} cr {r['cr']:.3f} "
                  f"realroot {r['max_real_root']} maxRe {r['max_re']:+.2f}"
                  + (f" kp {r['kp']:.2e} ki {r['ki']:.2e} c {r['c']:.2e} Z/JW {r['Zrel_mag']:.3f}@{r['Zrel_phase_deg']:+.0f}"
                     if 'kp' in r else ""))
