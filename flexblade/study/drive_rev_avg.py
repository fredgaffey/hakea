"""Rev-synchronous averaged rpm feedback (Drive.rpm_filter = "rev_avg") vs raw eRPM and a 1st-order
lowpass (corner 2 × bw), for 10in v5 / 7in v6 at T{0.7,1,1.4}, bw {30,80} Hz, τ {0.5,2} ms:
1P cyclic gain/phase vs free spin (harmonic_1p, exact filter) and min ζ / max Re s (airframe Floquet with
the drive state model).  Plus the speed loop's own limit: rev_avg bandwidth sweep (Floquet) and the
rigid-spin loop phase margin  L = (kp + ki/s) M(s) e^{−sτ}/(1 + s tl) / (J s + c).
Run from flexblade/ with PYTHONPATH=$PWD.  -> study/results/drive_rev_avg.json"""
import json
import sys
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from flexblade import rotor, robust, stability as S
from flexblade.schema import Airframe, Design, Drive, Mount

sys.path.insert(0, str(Path(__file__).parent))
from drive_impedance import DES, HERE, with_esc  # noqa: E402

FILTS = ("none", "lowpass", "rev_avg")
CFG = {"free": None}
for bw in (30, 80):
    for tau in (0.5e-3, 2e-3):
        for f in FILTS:
            CFG[f"{f}_bw{bw}_{tau * 1e3:g}ms"] = Drive(bandwidth_hz=bw, latency_s=tau, rpm_filter=f)
SWEEP_BW = (20, 40, 50, 60, 120)
for bw in SWEEP_BW:
    for tau in (0.5e-3, 2e-3):
        CFG[f"rev_avg_bw{bw}_{tau * 1e3:g}ms"] = Drive(bandwidth_hz=bw, latency_s=tau, rpm_filter="rev_avg")


def loop_margin(dp, Omega):
    """Rigid-spin speed loop: crossover (Hz) and phase margin (deg) of L(jw)."""
    w = np.logspace(0, np.log10(20 * Omega), 6000)
    L = np.array([(S.drive_impedance(dp, 1j * x, emf=False)) / (1j * x * dp["J"] + dp["c"]) for x in w])
    i = np.where(np.abs(L) >= 1)[0]
    if not len(i):
        return None, None
    k = i[-1]
    ph = np.unwrap(np.angle(L))
    return float(w[k] / (2 * np.pi)), float(180 + np.degrees(ph[k]))


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
    out = dict(rpm=ss.rpm, f1p_hz=ss.Omega / (2 * np.pi), J_spin=J, cfg={})
    ref = None
    for tag, esc in CFG.items():
        model.design = with_esc(d, esc)
        sweep = tag.startswith("rev_avg") and any(f"_bw{b}_" in tag for b in SWEEP_BW)
        r = {}
        if not sweep:
            hr = rotor.harmonic_1p(model, ss, aero, lin=lin).result
            Mc = hr.hub_moment_mean * np.exp(1j * np.radians(hr.hub_moment_phase_deg))
            ref = Mc if ref is None else ref
            r.update(gain=float(abs(Mc / ref)), dphase_deg=float(np.degrees(np.angle(Mc / ref))))
        zmin, _, s, _ = S.air_resonance(model, ss, lin, aero, afs, d.analysis.stability_basis)
        s = s[np.abs(s) > 1e-6 * ss.Omega]
        r.update(zeta_min=float(zmin), max_re=float(s.real.max()))
        if esc is not None:
            dp = S.drive_params(model.design.motor, J, Omega=ss.Omega)
            fc, pm = loop_margin(dp, ss.Omega)
            r.update(loop_fc_hz=fc, loop_pm_deg=pm)
        out["cfg"][tag] = r
    # rigid-spin loop phase margin vs bandwidth for rev_avg (fine grid): max bw with PM >= 0 / 30 / 45 deg
    out["pm_curve"] = {}
    for tau in (0.5e-3, 2e-3):
        rows = []
        for bw in np.arange(5, 101, 2.5):
            dp = S.drive_params(with_esc(d, Drive(bandwidth_hz=float(bw), latency_s=tau,
                                                  rpm_filter="rev_avg")).motor, J, Omega=ss.Omega)
            rows.append((float(bw),) + loop_margin(dp, ss.Omega))
        out["pm_curve"][f"{tau * 1e3:g}ms"] = rows
    return name, T, out


if __name__ == "__main__":
    jobs = [(n, T) for n in DES for T in (0.7, 1.0, 1.4)]
    with ProcessPoolExecutor(6) as ex:
        res = list(ex.map(job, jobs))
    js = {f"{n}:T{T:g}": o for n, T, o in res}
    (HERE / "results" / "drive_rev_avg.json").write_text(json.dumps(js, indent=1))
    for k, o in js.items():
        print(k, f"rpm {o['rpm']:.0f} 1P {o['f1p_hz']:.0f} Hz")
        for t, r in o["cfg"].items():
            print(f"  {t:22s} |G| {r.get('gain', float('nan')):.3f} dphi {r.get('dphase_deg', float('nan')):+7.1f} "
                  f"zmin {r['zeta_min']:+.4f} maxRe {r['max_re']:+8.2f} "
                  f"fc {r.get('loop_fc_hz')} PM {r.get('loop_pm_deg')}")
        for tau, rows in o["pm_curve"].items():
            lim = {p: max([b for b, fc, pm in rows if pm is not None and pm >= p], default=None) for p in (0, 30, 45)}
            print(f"  rigid loop rev_avg τ {tau}: max bw for PM>=0/30/45: {lim}")
