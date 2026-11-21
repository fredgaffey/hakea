"""CF-invariant mechanism mode: mode/rev drift over T0.7..1.4 and its elastic vs CF/geometric split.

For each variant of the v8 lead and throttle T in (0.7, 1.0, 1.4): trim, harmonic 1P, mechanism mode
(largest share of the 1P response, as run_study.mechanism_mode), then at the FROZEN trimmed state the
same mode with the geometric/centrifugal stiffness lin.Kgeo scaled (s=0: elastic only) and with
Omega scaled (Kgeo*k^2, G*k) to get the frozen-state drift. Single process.
usage: cf_invariant_mode.py [variant ...]   -> prints a table, writes results/cf_invariant_mode.json"""
import sys, os, json, dataclasses
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_study import modal_participation  # noqa: E402
from flexblade import rotor  # noqa: E402
from flexblade.schema import Design, BalanceMass  # noqa: E402

BASE = Design.model_validate_json(open(f"{HERE}/designs/10in_tt_strap_steelx_v8.json").read())
TF = (0.7, 1.0, 1.4)


def upd(d, blade=None, flexure=None):
    u = {}
    if blade:
        u["blade"] = d.blade.model_copy(update=blade)
    if flexure:
        u["flexure"] = d.flexure.model_copy(update=flexure)
    return d.model_copy(update=u)


def bm(m, r, c=0.0, z=0.0):
    return [BalanceMass(mass=m, r_frac=r, chord_offset=c, normal_offset=z),
            BalanceMass(mass=m, r_frac=r, chord_offset=-c, normal_offset=-z)]


VARIANTS = {
    "v8": BASE,
    "no_bal": upd(BASE, blade=dict(balance_masses=[])),
    "chord_bal": upd(BASE, blade=dict(balance_masses=bm(0.001, 0.2, c=0.010))),
    "chord_bal_r35": upd(BASE, blade=dict(balance_masses=bm(0.001, 0.35, c=0.010))),
    "thin80": upd(BASE, flexure=dict(thickness=0.8 * BASE.flexure.thickness)),
    "strap2x": upd(BASE, flexure=dict(strap_length=2 * BASE.flexure.strap_length)),
    "thin80_chord_bal": upd(BASE, blade=dict(balance_masses=bm(0.001, 0.2, c=0.010)),
                            flexure=dict(thickness=0.8 * BASE.flexure.thickness)),
}


def mode_at(model, ss, lin, ref, s_geo=1.0, k=1.0):
    """frequency [Hz] of the mode best matching shape ref (MAC), with Kgeo*s_geo*k^2, G*k, Omega*k."""
    l2 = dataclasses.replace(lin, Kgeo=lin.Kgeo * s_geo * k * k, G=lin.G * k)
    s2 = dataclasses.replace(ss, Omega=ss.Omega * k)
    md = rotor.modes(model, s2, None, n_modes=12, lin=l2)
    best, fb = -1, np.nan
    for f, x in zip(md.freqs_hz, md.shapes):
        if f < 1e-3 * s2.Omega:
            continue
        mac = abs(np.vdot(ref, x)) ** 2 / (np.vdot(ref, ref).real * np.vdot(x, x).real)
        if mac > best:
            best, fb = mac, float(f)
    return fb, best


def run_point(d0, f):
    d = d0
    if d.flexure.pre_lag_deg == "auto":
        d = rotor.resolve_pre_lag(d)          # built at the nominal mass, as robust.variants
    d = d.model_copy(update=dict(op=d.op.model_copy(update=dict(
        vehicle_mass=d0.op.vehicle_mass * f, rpm=None, pulse_capability_margin=0.95))))
    model = rotor.build_model(d)
    aero = rotor.make_aero(model)
    ss = rotor.static_trim(model, aero)
    lin = rotor.linearize(model, ss, d.analysis.coriolis)
    md = rotor.modes(model, ss, aero, lin=lin)
    hd = rotor.harmonic_1p(model, ss, aero, lin=lin)
    w = np.r_[np.full(3, 1.0 / model.R), np.ones(3)]
    U = hd.U.reshape(-1, 6) * w
    Phi = np.stack([(np.real(x).reshape(-1, 6) * w).ravel() for x in md.shapes], 1)
    c = modal_participation(U.ravel(), Phi)
    c = np.where(md.freqs_hz >= 0.05 * ss.rpm / 60, c, -1)
    im = int(np.argmax(c))
    ref = md.shapes[im]
    f1 = ss.rpm / 60.0
    out = dict(T=f, rpm=float(ss.rpm), label=md.labels[im], mpr=float(md.freqs_hz[im] / f1),
               ctrl_M=float(hd.result.hub_moment_mean), cr=float(hd.result.control_ratio),
               phase=float(hd.result.hub_moment_phase_deg))
    fe0, mac0 = mode_at(model, ss, lin, ref, s_geo=0.0)
    ff, _ = mode_at(model, ss, lin, ref)
    out["mpr_freeze"] = ff / f1
    out["f_cf"] = 1.0 - (fe0 / ff) ** 2          # CF/geometric share of omega^2 (rotating frame)
    out["mac_el"] = mac0
    fp, _ = mode_at(model, ss, lin, ref, k=1.03)
    fm, _ = mode_at(model, ss, lin, ref, k=1 / 1.03)
    out["drift_freeze"] = (np.log(fp / (1.03 * f1)) - np.log(fm / (f1 / 1.03))) / (2 * np.log(1.03))
    return out


def main():
    names = sys.argv[1:] or list(VARIANTS)
    path = f"{HERE}/results/cf_invariant_mode.json"
    res = json.load(open(path)) if os.path.exists(path) else {}
    for n in names:
        rows = [run_point(VARIANTS[n], f) for f in TF]
        a, b = rows[0], rows[-1]
        dr = (np.log(b["mpr"]) - np.log(a["mpr"])) / (np.log(b["rpm"]) - np.log(a["rpm"]))
        res[n] = dict(rows=rows, dlnmpr_dlnrpm=float(dr), d_mpr_dlnrpm=float(dr * rows[1]["mpr"]))
        for r in rows:
            print(f"{n:18s} T{r['T']:.1f} rpm {r['rpm']:6.0f} {r['label'][:28]:28s} mpr {r['mpr']:.3f} "
                  f"f_cf {r['f_cf']:.2f} (mac {r['mac_el']:.2f}) drift_frz {r['drift_freeze']:+.3f} "
                  f"M {r['ctrl_M']:.3f} cr {r['cr']:.3f} ph {r['phase']:.0f}", flush=True)
        print(f"{n:18s} d ln(mpr)/d ln(rpm) T0.7-1.4 = {dr:+.3f}  d(mpr)/d ln rpm = {dr * rows[1]['mpr']:+.3f}",
              flush=True)
        json.dump(res, open(path, "w"), indent=1)


if __name__ == "__main__":
    main()
