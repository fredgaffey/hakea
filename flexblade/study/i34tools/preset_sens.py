"""Finite-difference sensitivity of the near-1P flap pair (/rev) to each strip-end preset angle.
Nominal geometry, one thrust factor x one J_hub; modes tracked by MAC against the nominal shapes.
hi = lowest mode in (1.0, 1.4)/rev, hi2 = next mode above it (its blade-localised partner).
env: SENS_J=1245 (preset components: 1 hub.th2, 2 hub.th3, 4 bld.th2, 5 bld.th3), SENS_TF=1.25,
     SENS_ALPHA=a (re-solve presets via run_study.resolve_candidate at alpha=a first; SENS_SAVE=path saves it)
usage: preset_sens.py <design.json> [delta_deg=0.25] [J_hub=2e-6]"""
import sys, os, json
import numpy as np
os.environ.setdefault("FLEXBLADE_GROUND_RES", "1")
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import run_study as rs
from flexblade import robust, api
from flexblade.schema import Design

NAMES = {1: "hub.th2", 2: "hub.th3", 4: "bld.th2", 5: "bld.th3"}


def modes(d, TF, J):
    (t, dv), = robust.variants(d, [TF], None, [J], None)
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    m = rs.metrics(dv, r, criterion="margin")
    rpm = r["summary"]["rpm"]
    md = [(x["label"], x["freq_hz"] * 60 / rpm, x["participation"], np.asarray(x["shape"], dtype=float).ravel())
          for x in r["modes"] if x["freq_hz"] * 60 / rpm > 0.05]
    return m, md


def track(md, ref):
    """mode with max MAC to the reference shape"""
    def mac(x):
        return abs(x[3] @ ref[3]) ** 2 / ((x[3] @ x[3]) * (ref[3] @ ref[3]) + 1e-300)
    return max(((x, mac(x)) for x in md), key=lambda p: p[1])


def main():
    d0 = Design.model_validate_json(open(sys.argv[1]).read())
    DL = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
    J = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0e-6
    TF = float(os.environ.get("SENS_TF", 1.25))
    if os.environ.get("SENS_ALPHA"):
        info = {}
        d0 = rs.resolve_candidate(d0, 8, alpha=float(os.environ["SENS_ALPHA"]), info=info)
        print("resolved alpha", os.environ["SENS_ALPHA"], "resid_deg", [round(x, 3) for x in info.get("resid_deg", [])])
        if os.environ.get("SENS_SAVE"):
            open(os.environ["SENS_SAVE"], "w").write(d0.model_dump_json())
    print("preset", json.dumps([[[round(x, 3) for x in e] for e in m[:2]] for m in d0.flexure.strip_preset_deg]))
    m0, md0 = modes(d0, TF, J)
    hi = min((x for x in md0 if 1.0 < x[1] < 1.4), key=lambda x: x[1])
    hi2 = min((x for x in md0 if hi[1] + 0.01 < x[1] < 2.0), key=lambda x: x[1])
    print(f"nom res {m0['res_margin']:.3f} ({m0['res_which']}); hi {hi[0]} {hi[1]:.3f}; hi2 {hi2[0]} {hi2[1]:.3f}")
    sp0 = d0.flexure.strip_preset_deg
    for b in range(len(sp0)):
        for s in range(min(2, len(sp0[b]))):
            for j in [int(c) for c in os.environ.get("SENS_J", "1245")]:
                fr = {}
                for sg in (1, -1):
                    sp = json.loads(json.dumps(sp0))
                    sp[b][s][j] += sg * DL
                    m, md = modes(d0.model_copy(update=dict(flexure=d0.flexure.model_copy(
                        update=dict(strip_preset_deg=sp)))), TF, J)
                    (th, mh), (th2, _) = track(md, hi), track(md, hi2)
                    fr[sg] = (th[1], th2[1], m["res_margin"], mh)
                dh = (fr[1][0] - fr[-1][0]) / (2 * DL)
                dh2 = (fr[1][1] - fr[-1][1]) / (2 * DL)
                curv = (fr[1][0] + fr[-1][0] - 2 * hi[1]) / DL ** 2
                print(f"b{b} s{s} {NAMES[j]}  d(hi) {dh:+.4f}/rev/deg  d2(hi) {curv:+.4f}  d(hi2) {dh2:+.4f}  "
                      f"res+ {fr[1][2]:.3f} res- {fr[-1][2]:.3f} MAC {min(fr[1][3], fr[-1][3]):.2f}", flush=True)


if __name__ == "__main__":
    main()
