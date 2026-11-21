"""10" tt_strap_steelx v5: place the 'pitch antisym+hub' (cyclic) mode away from 1P.

2-DOF view: w^2 = (k_struct + k_cf Omega^2)/I  ->  /rev = sqrt(k_struct/(I Omega^2) + k_cf/I).
`decomp` Rayleigh-splits the model mode (real shape x): k_struct = x'Re(K_el(f))x, k_geo = x'Kgeo x
(CF + stress stiffening), I = x'Mx, at T0.7/1.0/1.4 (J 3.76e-6), plus a least-squares fit of
w^2 = a + b Omega^2 over the throttle points; `knobs` finite-differences /rev per knob;
`pl` / `tol` / `save` use pulse_limited (throttle x J_hub {3.76, 2.5, 5.49}e-6).
    PYTHONPATH=flexblade python flexblade/study/steelx_10in_v5.py decomp '<json x>'
    PYTHONPATH=flexblade python flexblade/study/steelx_10in_v5.py knobs
    PYTHONPATH=flexblade python flexblade/study/steelx_10in_v5.py pl '<json {tag: x}>'
    PYTHONPATH=flexblade python flexblade/study/steelx_10in_v5.py tol '<json x>'
    PYTHONPATH=flexblade python flexblade/study/steelx_10in_v5.py save '<json x>'
-> study/results/steelx_10in_v5.json
"""
from __future__ import annotations
import json, math, random, sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from flexblade import robust, rotor                               # noqa: E402
from flexblade.optimize import get_path, set_design               # noqa: E402
from flexblade.schema import BalanceMass, Design                  # noqa: E402
import pulse_limited as pl                                        # noqa: E402

RES = HERE / "results" / "steelx_10in_v5.json"
V4 = pl.load_design("10in_tt_strap_steelx_v4")
OUT = HERE / "designs" / "10in_tt_strap_steelx_v5.json"
JS = (3.76e-6, 2.5e-6, 5.49e-6)
JNOM = 3.76e-6


def load():
    return json.loads(RES.read_text()) if RES.exists() else {}


def save(k, v):
    r = load(); r[k] = v; RES.write_text(json.dumps(r, indent=1, default=float))


def build(x: dict | None = None) -> Design:
    x = dict(x or {})
    bal = {k: x.pop(k) for k in ("bal_m", "bal_r", "bal_n", "bal_c") if k in x}
    raw = {k: x.pop(k) for k in list(x) if isinstance(x[k], (str, bool))}
    d = set_design(V4, x) if x else V4
    if raw:
        d = d.model_copy(update=dict(flexure=d.flexure.model_copy(
            update={k.split(".", 1)[1]: v for k, v in raw.items()})))
    if bal:
        b0 = d.blade.balance_masses[0]
        m = bal.get("bal_m", b0.mass); r = bal.get("bal_r", b0.r_frac)
        n = bal.get("bal_n", abs(b0.normal_offset)); c = bal.get("bal_c", b0.chord_offset)
        bm = [BalanceMass(mass=m, r_frac=r, chord_offset=c, normal_offset=n),
              BalanceMass(mass=m, r_frac=r, chord_offset=c, normal_offset=-n)] if m > 0 else []
        d = d.model_copy(update=dict(blade=d.blade.model_copy(update=dict(balance_masses=bm))))
    d = Design.model_validate_json(d.model_dump_json())
    return d.model_copy(update=dict(name="10in_tt_strap_steelx_v5"))


def _decomp(dv):
    """Rayleigh split of the hub-participating mode nearest 1/rev."""
    try:
        with rotor.part_temperature(dv.op.ambient_C):
            d = rotor.resolve_pre_lag(dv) if dv.flexure.pre_lag_deg == "auto" else dv
            model = rotor.build_model(d)
            aero = rotor.make_aero(model)
            ss = rotor.static_trim(model, aero)
            lin = rotor.linearize(model, ss, d.analysis.coriolis)
            md = rotor.modes(model, ss, aero, lin=lin)
            Om = ss.Omega
            el = [i for i, f in enumerate(md.freqs_hz) if f * 2 * np.pi > 0.3 * Om]
            hub = [i for i in el if md.parts[i].get("hub", 0) > 0.15] or el
            i = min(hub, key=lambda j: abs(md.freqs_hz[j] * 2 * np.pi / Om - 1))
            x = np.real(md.shapes[i])
            x = x / x[6 * model.hub + 5]          # referred to hub swing psi = 1 rad: I in kg m^2, k in N m/rad
            f = md.freqs_hz[i]
            scales = np.array([e.complex_scale(f) for e in model.fe.elements]).real
            Kel = rotor.assemble_Kmat(model.fe, lin.sa, scales)
            ks = float(x @ Kel @ x); kg = float(x @ lin.Kgeo @ x); I = float(x @ lin.M @ x)
            return dict(rpm=ss.rpm, Omega=Om, label=md.labels[i], per_rev=f * 2 * np.pi / Om,
                        zeta=float(md.damping[i]), hub=md.parts[i].get("hub"), k_struct=ks, k_cf=kg / Om**2,
                        I=I, rayleigh_per_rev=math.sqrt(max(ks + kg, 0) / I) / Om, struct_share=ks / (ks + kg),
                        others=[(md.labels[j], round(md.freqs_hz[j] * 2 * np.pi / Om, 3)) for j in el[:6]])
    except Exception as e:  # noqa: BLE001
        return dict(error=repr(e))


def decomp_points(designs, ex):
    jobs = [dv for d in designs for _, dv in robust.variants(d, j_hub=[JNOM])]
    out = list(ex.map(_decomp, jobs))
    return [out[3 * k:3 * k + 3] for k in range(len(designs))]


def fit(pts):
    """least squares w^2 = a + b Omega^2 over the throttle points (a = k_struct/I, b = k_cf/I)."""
    O = np.array([p["Omega"] for p in pts]); w = np.array([p["per_rev"] * p["Omega"] for p in pts])
    A = np.c_[np.ones_like(O), O**2]
    (a, b), *_ = np.linalg.lstsq(A, w**2, rcond=None)
    return dict(a_kstruct_over_I=float(a), b_kcf_over_I=float(b),
                rel_rms=float(np.sqrt(np.mean((A @ [a, b] - w**2) ** 2)) / np.mean(w**2)),
                per_rev_fit=[float(math.sqrt(a / o**2 + b)) for o in O])


KNOBS = {"flexure.hinge_radius": 0.05, "flexure.skew_deg": 0.05, "blade.elastic_axis_chord_frac": 0.10,
         "bal_r": 0.25, "bal_n": 0.25, "bal_m": 0.5, "flexure.length": 0.05, "flexure.strap_length": 0.15,
         "flexure.strap_strand_diameter": 0.10, "flexure.cross_angle_deg": 0.05, "flexure.width": 0.10}


def x0(k):
    b = V4.blade.balance_masses[0]
    return {"bal_r": b.r_frac, "bal_n": abs(b.normal_offset), "bal_m": b.mass}.get(k) or float(get_path(V4, k))


def main():
    cmd = sys.argv[1]
    arg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    with ProcessPoolExecutor(4) as ex:
        if cmd == "decomp":
            pts = decomp_points([build(arg)], ex)[0]
            out = dict(x=arg, points=pts, fit=fit(pts) if all("Omega" in p for p in pts) else None)
            save("decomp" if not arg else "decomp_" + json.dumps(arg), out)
            print(json.dumps(out, indent=1))
        elif cmd == "knobs":
            xs = {k: {k: x0(k) * (1 + h)} for k, h in KNOBS.items()} if not arg else arg
            xs = {"base": {}, **xs}
            res = decomp_points([build(v) for v in xs.values()], ex)
            base = res[0]
            out = {}
            for (k, v), pts in zip(xs.items(), res):
                pr = [p.get("per_rev", float("nan")) for p in pts]
                dx = KNOBS.get(k, 0.1)
                out[k] = dict(x=v, per_rev=pr, d_per_rev_per_10pct=[(a - b0.get("per_rev", np.nan)) / dx * 0.1
                                                                     for a, b0 in zip(pr, base)],
                              share=[p.get("struct_share") for p in pts], label=[p.get("label") for p in pts],
                              err=[p.get("error") for p in pts if "error" in p])
                print(k, [round(a, 3) for a in pr], [round(a, 4) for a in out[k]["d_per_rev_per_10pct"]],
                      [round(p.get("struct_share") or 0, 3) for p in pts], out[k]["err"] or "", flush=True)
            r = load().get("knobs", {}); r.update(out); save("knobs", r)
        elif cmd == "pl":
            tags = list(arg)
            cs = pl.robust_eval([build(arg[t]) for t in tags], js=JS, ex=ex)
            r = load().get("pl", {})
            for t, c in zip(tags, cs):
                s = pl.summary(c); s["x"] = arg[t]
                r[t] = s
                pts = [(p["tag"], round(p["res_margin"], 3), round(p["cr_pl"] or 0, 3), p["pulse_binding"])
                       for p in s["points"]]
                print(t, round(s["cr_pl"] or 0, 4), s["feasible_pl"],
                      {k: round(v, 3) for k, v in (s["viol_hard"] or {}).items()},
                      s["res_which"], round(s["res_margin"], 3), pts, flush=True)
            save("pl", r)
        elif cmd == "tol":
            import verify_v5 as vv
            d0 = build(arg)
            rnd = random.Random(11)
            T, W, L, s = "flexure.thickness", "flexure.width", "flexure.length", "flexure.strap_strand_diameter"
            cases = {"nominal": []}
            for i in range(8):
                u = lambda a: 1 + rnd.uniform(-a, a)  # noqa: E731
                cases[f"random{i:02d}"] = [("mul", T, u(.05)), ("mul", W, u(.05)), ("mul", L, u(.05)),
                                           ("mul", "flexure.hinge_radius", u(.05)), ("rho", "", round(u(.05), 4)),
                                           ("mul", s, math.sqrt(u(.05))), ("add", "flexure.skew_deg", rnd.uniform(-1, 1)),
                                           ("add", "blade.elastic_axis_chord_frac", rnd.uniform(-.01, .01))]
            cs = pl.robust_eval([vv.perturbed(d0, sp) for sp in cases.values()], js=(JNOM,), ex=ex)
            out = {n: pl.summary(c) for n, c in zip(cases, cs)}
            npass = sum(bool(o["feasible_pl"]) for o in out.values())
            save("tol", dict(x=arg, J_hub=JNOM, n_pass=npass, n=len(out), cases=out))
            for n, o in out.items():
                print(n, round(o["cr_pl"] or 0, 4), o["feasible_pl"], o["viol_hard"], round(o["res_margin"], 3), flush=True)
            print(f"tolerance {npass}/{len(out)}")
        elif cmd == "save":
            d = build(arg)
            d = d.model_copy(update=dict(notes=(V4.notes or "") + "; v5 (cyclic-mode placement, steelx_10in_v5.py): "
                                         + json.dumps(arg)))
            OUT.write_text(d.model_dump_json(indent=1))
            print("saved", OUT)


if __name__ == "__main__":
    main()
