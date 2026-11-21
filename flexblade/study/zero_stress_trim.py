"""I34 zero-stress trim: strip_preset_deg (pre-angled clamp slots) resolved at hover, scored with
Goodman + pulse_limited robust T x J; residual trim end rotations at T0.7/T1.4; slot tolerance.
PYTHONPATH=flexblade python flexblade/study/zero_stress_trim.py <base> <preset.json> [--tol 0.5]"""
import sys
sys.path.insert(0, __import__("pathlib").Path(__file__).parent.as_posix())
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import pulse_limited as pl
from flexblade import api, robust, rotor
from flexblade.schema import Design


def goodman(d):
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(strip_fatigue_mean="goodman"))))


def shift(d, dp=None, rng=None, tol=0.5):
    """Perturb the bending presets (θ2, θ3 per end) by dp*tol or uniform ±tol (rng)."""
    sp = [[list(r) for r in b] for b in d.flexure.strip_preset_deg]
    for b in sp:
        for r in b:
            if any(r):
                for c in (1, 2, 4, 5):
                    r[c] += tol * (dp if rng is None else rng.uniform(-1, 1))
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(strip_preset_deg=sp))))


def point(dv):
    """worst strip fatigue check, mode freqs/labels, max |residual bending end rotation| (deg)."""
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"])
    rj = r.model_dump(mode="json")
    w = max((c for c in rj["checks"] if c["name"] == "fatigue_stress"), key=lambda c: c["utilisation"])
    dd = rotor.resolve_pre_lag(dv)
    m = rotor.build_model(dd)
    ss = rotor.static_trim(m, rotor.make_aero(m))
    res = max(abs(row[c]) for b in rotor.strip_end_rotations(m, ss) for row in b for c in (1, 2, 4, 5))
    return (w["utilisation"], w["note"][-150:], [round(x.freq_hz, 1) for x in r.modes[:6]],
            [x.label for x in r.modes[:6]], res)


if __name__ == "__main__":
    a = sys.argv[1:]
    tol = float(a[a.index("--tol") + 1]) if "--tol" in a else 0.5
    base = goodman(pl.load_design(a[0]))
    z = goodman(Design.model_validate_json(open(a[1]).read()))
    cases = [("base", base), ("z", z), (f"z+{tol}", shift(z, 1.0, tol=tol)), (f"z-{tol}", shift(z, -1.0, tol=tol)),
             (f"z_rand{tol}", shift(z, rng=np.random.default_rng(1), tol=tol))]
    with ProcessPoolExecutor(3) as ex:
        rs = pl.robust_eval([d for _, d in cases], js=pl.JS, ex=ex)
        for (tag, d), r in zip(cases, rs):
            s = pl.summary(r)
            print(tag, "cr_pl", round(s["cr_pl"] or 0, 4), "feas", s["feasible_pl"],
                  "bind", sorted({p["pulse_binding"] for p in s["points"]}), flush=True)
        for tag, d in cases[:2]:
            vs = robust.variants(d, robust.THRUST_FACTORS, None, None)
            for (vt, _), out in zip(vs, ex.map(point, [dv for _, dv in vs])):
                print(tag, vt, "fat_s_util %.3f" % out[0], "resid_bend_deg %.3f" % out[4], out[2], out[3],
                      out[1], flush=True)
