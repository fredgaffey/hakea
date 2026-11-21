"""Blade-level inertia/aero levers on the CF-dominated mech (pitch-lag-flap) mode of v8/v10m.
Each variant: presets re-solved (resolve_candidate 6 it), then tol.py-style nominal eval at
T0.7/1.0/1.4, J_hub 3.76e-6, criterion "margin" (robust.combine). See results/blade_inertia_lever.md.
usage: blade_inertia_lever.py out.json '[["name", {"base":"v8", ...}], ...]'
kw: base v8|v10m; rho (blade density x); E (blade E x); taper (tip-chord x, linear in r, root fixed);
    chord (all chords x); tc (t/c override); bal [[g, r_frac, chord_off, normal_off], ...] (replaces;
    entries with normal_off != 0 are mirrored at +-normal_off); add [...] (appended, same format);
    hinge (hinge_radius x); cutout (root_cutout_frac abs)."""
import sys, os, json, time
os.environ.setdefault("FLEXBLADE_GROUND_RES", "1")
here = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, here)
import run_study as rs
from flexblade import robust, api
from flexblade.schema import Design, BalanceMass


def load(v):
    return Design.model_validate_json(open(f"{here}/designs/10in_tt_strap_steelx_{v}.json").read())


def bm(entries):
    out = []
    for g, r, c, z in entries:
        for s in ((1, -1) if z else (1,)):
            out.append(BalanceMass(mass=g * 1e-3, r_frac=r, chord_offset=c, normal_offset=s * z))
    return out


def build(base="v8", rho=None, E=None, taper=None, chord=None, tc=None, bal=None, add=None, hinge=None,
          cutout=None):
    d = load(base); b = d.blade; bu = {}
    if rho or E:
        m = b.layup.material + "@" + ",".join(f"{k}={v}" for k, v in (("rho", rho), ("E", E)) if v)
        bu["layup"] = b.layup.model_copy(update=dict(material=m))
    st = b.stations; r0 = st[0].r_frac
    if taper or chord or tc:
        st = [s.model_copy(update=dict(
            chord=s.chord * (chord or 1) * (1 + ((taper or 1) - 1) * (s.r_frac - r0) / (1 - r0)),
            thickness_frac=tc if tc else s.thickness_frac)) for s in st]
        bu["stations"] = st
    if bal is not None:
        bu["balance_masses"] = bm(bal)
    if add:
        bu["balance_masses"] = list(bu.get("balance_masses", b.balance_masses)) + bm(add)
    if cutout:
        bu["root_cutout_frac"] = cutout
    fu = {"hinge_radius": d.flexure.hinge_radius * hinge} if hinge else {}
    return d.model_copy(update=dict(blade=b.model_copy(update=bu), flexure=d.flexure.model_copy(update=fu)))


def run(args):
    name, kw = args; t0 = time.time()
    d = build(**kw)
    try:
        d = rs.resolve_candidate(d, 6)
    except Exception as e:  # noqa: BLE001
        return dict(name=name, err=str(e))
    ms = []
    for T, dv in robust.variants(d, robust.THRUST_FACTORS, None, [3.76e-6], None):
        r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
        m = rs.metrics(dv, r, criterion="margin")
        for _, sub in rs._sub_results(r):
            im = rs.mechanism_mode(sub)
            if im is not None:
                mm = sub["modes"][im]; rpm = (sub.get("summary") or {}).get("rpm") or 1.0
                m.update(mech_zeta=float(mm.get("damping_ratio", float("nan"))), rpm=rpm,
                         mech_pr=float(mm["freq_hz"] * 60 / rpm))
        ms.append((T, m))
    c = robust.combine([m for _, m in ms], [t for t, _ in ms])
    gd = [(m.get("util") or {}).get("fatigue_stress") for _, m in ms]
    rd = lambda xs: [None if x is None else round(x, 4) for x in xs]  # noqa: E731
    return dict(name=name, cr=c.get("cr_pl"), res=c.get("res_margin"), which=c.get("res_which"),
                mech=rd([m.get("mech_pr") for _, m in ms]), zeta=rd([m.get("mech_zeta") for _, m in ms]),
                rpm=rd([m.get("rpm") for _, m in ms]), good=max((x for x in gd if x is not None), default=None),
                viol={k: round(v, 3) for k, v in (c.get("viol_hard") or {}).items()},
                jit=rd([(m.get("util") or {}).get("harmonic_jitter") for _, m in ms]), kw=kw,
                s=round(time.time() - t0))


if __name__ == "__main__":
    P = json.loads(sys.argv[2])
    from multiprocessing import Pool
    with Pool(min(3, len(P))) as p:
        R = []
        for r in p.imap(run, P, chunksize=1):
            print(json.dumps(r, default=str), flush=True); R.append(r)
    json.dump(R, open(sys.argv[1], "w"), default=str)
