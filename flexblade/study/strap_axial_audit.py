"""Strap cord axial vs bending strain audit (lead 5in_v8s_2307_m245_301, J_hub 0.91e-6).

Per thrust factor: strap (cable) axial strain eps0 mean, 1P amplitude, min over the rev, and the
solid-section bending part of the max fibre strain; then cr_pl / binding with the default cable
strain and with the opt-in CABLE_STRAIN=axial, for strap_anchor pin_wrap and knot.
    FLEXBLADE_CACHE=off PYTHONPATH=flexblade python flexblade/study/strap_axial_audit.py
"""
from __future__ import annotations
import json, math, os, sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust, rotor                         # noqa: E402
from flexblade.schema import Design                              # noqa: E402
import run_study as rs                                           # noqa: E402

LEAD = HERE / "designs" / "5in_v8s_2307_m245_301.json"
J = 0.91e-6
STRAP = "b2.flex2"
REC: dict = {}
_cur = [None]
_fc, _fa = rotor._fibre_checks, rotor._fibre_amp


def _bend(sec, eps):
    return max(abs(fb.z * eps[4] - fb.y * eps[5]) for fb in sec.fibres)


def fc(e, sec, sf, t_hat, cable=False):
    _cur[0] = id(e)
    if cable:
        eps = np.linalg.solve(sec.stiffness6(), sf)
        r = REC.setdefault(id(e), dict(tag=e.tag, st=[], amp=[]))
        r["st"].append((float(eps[0]), float(_bend(sec, eps)), float(math.hypot(eps[4], eps[5])),
                        float(sf[0])))
    return _fc(e, sec, sf, t_hat, cable)


def fa(sec, s_amp, t_hat, freq_hz, cable=False, *a, **k):
    if cable:
        eps = np.linalg.solve(sec.stiffness6().astype(complex), s_amp)
        REC[_cur[0]]["amp"].append((float(freq_hz), float(abs(eps[0])), float(_bend(sec, eps)),
                                    float(math.hypot(abs(eps[4]), abs(eps[5])))))
    return _fa(sec, s_amp, t_hat, freq_hz, cable, *a, **k)


rotor._fibre_checks, rotor._fibre_amp = fc, fa


def job(args):
    T, anchor, mode = args
    os.environ["CABLE_STRAIN"] = mode
    REC.clear()
    d = Design.model_validate_json(LEAD.read_text())
    d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(strap_anchor=anchor))))
    (tag, dv), = robust.variants(d, [T], None, [J])
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    m = rs.metrics(dv, r)
    ch = {(c["name"], c["member"]): c["utilisation"] for c in r["checks"]}
    puls = sorted(((round(c["utilisation"], 4), c["name"], c["member"]) for c in r["checks"]
                   if c["name"] in rs.PULSE_EXP), reverse=True)[:3]
    rows = []
    for rec in REC.values():
        if rec["tag"] != STRAP:
            continue
        # per element: last 2 static calls (2 end sections), first 2 amp calls = 1P (then nP crossings)
        for (e0, b, k, N), (f, A0, Ab, Ak) in zip(rec["st"][-2:], rec["amp"][:2]):
            rows.append(dict(eps0=e0, bend=b, N=N, f=f, amp0=A0, ampb=Ab, ampk=Ak, emin=e0 - A0))
    agg = {}
    if rows:
        agg = dict(eps0_mean=max(x["eps0"] for x in rows), N_mean=max(x["N"] for x in rows),
                   amp0=max(x["amp0"] for x in rows), emin=min(x["emin"] for x in rows),
                   bend_static=max(x["bend"] for x in rows), bend_amp=max(x["ampb"] for x in rows),
                   kap_amp=max(x["ampk"] for x in rows), f1p=rows[0]["f"], n_sec=len(rows),
                   tags=sorted({v["tag"] for v in REC.values()}))
    return dict(T=T, anchor=anchor, mode=mode or "default", tag=tag, cr=m.get("control_ratio"),
                cr_pl=m.get("cr_pl"), s=m.get("pulse_scale"), bind=m.get("pulse_binding"),
                feas_pl=m.get("feasible_pl"), hard=list(m.get("viol_hard", {})),
                strap_fat=ch.get(("fatigue_strain", "flex2")), strap_fats=ch.get(("fatigue_stress", "flex2")),
                strap_creep=ch.get(("creep_strain", "flex2")), anchor_u=ch.get(("strap_anchor", "strap")),
                top_pulse=puls, strap=agg)


if __name__ == "__main__":
    anchors = sys.argv[1].split(",") if len(sys.argv) > 1 else ["pin_wrap", "knot"]
    combos = [(T, a, md) for a in anchors for md in ("", "axial") for T in (0.7, 1.0, 1.25)]
    with ProcessPoolExecutor(2) as ex:
        for res in ex.map(job, combos):
            print(json.dumps(res), flush=True)
