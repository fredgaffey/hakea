"""2307 5-6in authority budget: cr_pl = (dQ_avail/Q) x (M/dQ) x (Q/(W R)) per lever, T0.7/1.0/1.25, one J."""
import sys, json, math, traceback
FB = __import__("os").path.dirname(__import__("os").path.dirname(__import__("os").path.abspath(__file__)))
OUT = FB + "/study/results/lever_2307.json"
sys.path.insert(0, FB + "/study")
from concurrent.futures import ProcessPoolExecutor
from flexblade import api, robust, presets, motor as mot
from flexblade.schema import Design
from flexblade.optimize import set_design
import run_study as rs

TF = (0.7, 1.0, 1.25)
J6 = 1.57e-6


def load(n):
    d = Design.model_validate_json(open(f"{FB}/study/designs/{n}.json").read())
    return d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(strip_fatigue_mean="goodman"))))


def op(d, **k):
    return d.model_copy(update=dict(op=d.op.model_copy(update=k)))


def motor_(d, **k):
    return d.model_copy(update=dict(motor=d.motor.model_copy(update=k)))


def blade_(d, **k):
    return d.model_copy(update=dict(blade=d.blade.model_copy(update=k)))


def chord(d, f):
    st = [s.model_copy(update=dict(chord=s.chord * f)) for s in d.blade.stations]
    return blade_(d, stations=st)


def cases():
    b = load("6in_tt_strap_steelx_2307")
    c = {"base6": (b, J6)}
    for J in (0.58e-6, 3.46e-6):
        c[f"J{J:.3g}"] = (b, J)
    for m in (0.5, 0.6, 0.7):
        c[f"m{m}"] = (op(b, vehicle_mass=m), J6)
    for I in (30.0, 50.0):
        c[f"I{I:g}"] = (motor_(b, max_current=I), J6)
    c["B3"] = (blade_(b, n_blades=3), J6)
    c["coll+3"] = (op(b, collective_deg=3.0), J6)
    c["chord1.3"] = (chord(b, 1.3), J6)
    s5 = presets.scale_design(b, 5.0)
    s5 = op(s5.model_copy(update=dict(motor=b.motor)), vehicle_mass=b.op.vehicle_mass)
    c["5in"] = (s5, J6 * (5 / 6) ** 3)
    c["v8_10in"] = (load("10in_tt_strap_steelx_v8"), 3.76e-6)
    return c


def ev(item):
    name, tag, dv = item
    try:
        r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
        m = rs.metrics(dv, r)
        tr, h = r["trim"], r["harmonic"] or {}
        Q, W, R = tr["torque"], dv.op.vehicle_mass * 9.80665, dv.blade.diameter_in * 0.0254 / 2
        s = m.get("pulse_scale", float("nan"))
        cap = mot.capability(dv.motor, tr["rpm"], Q, ambient_C=dv.op.ambient_C)
        ck = {c["name"]: (round(c["utilisation"], 3), c.get("note")) for c in r["checks"] if c["member"] in ("motor", "esc")}
        return dict(case=name, tag=tag, rpm=tr["rpm"], Q=Q, W=W, R=R, QWR=Q / (W * R), FM=tr.get("figure_of_merit"),
                    dQ=h.get("torque_amp"), s=s, dQav_Q=s * h.get("torque_amp", 0) / Q, mpt=h.get("moment_per_torque"),
                    cr=m.get("control_ratio"), cr_pl=m.get("cr_pl"), bind=m.get("pulse_binding"),
                    feas=m.get("feasible_pl"), hard=list((m.get("viol_hard") or {}).keys()),
                    capI=cap.dQ_current_limit / Q, capV=cap.dQ_voltage_limit / Q, capT=cap.dQ_thermal_limit / Q,
                    I0=cap.I_mean, checks=ck, pitch_pr=r["summary"].get("pitch_freq_per_rev"))
    except Exception as e:
        return dict(case=name, tag=tag, err=repr(e), tb=traceback.format_exc()[-600:])


def main():
    sel = sys.argv[1:] or None
    items = []
    for n, (d, J) in cases().items():
        if sel and n not in sel:
            continue
        for t, dv in robust.variants(d, TF, None, [J]):
            items.append((n, t, dv))
    with ProcessPoolExecutor(3) as ex:
        res = list(ex.map(ev, items, chunksize=1))
    old = json.load(open(OUT)) if (sel and __import__("os").path.exists(OUT)) else []
    old = [x for x in old if x["case"] not in {r["case"] for r in res}] + res
    json.dump(old, open(OUT, "w"), indent=1, default=float)
    for x in res:
        if "err" in x:
            print(x["case"], x["tag"], "ERR", x["err"]); continue
        print(f"{x['case']:9s} {x['tag']:14s} rpm {x['rpm']:6.0f} Q {x['Q']:.4f} QWR {x['QWR']:.3f} dQav/Q {x['dQav_Q']:.2f} "
              f"(capI {x['capI']:.2f} V {x['capV']:.2f} T {x['capT']:.2f}) mpt {x['mpt']:.4f} cr_pl {x['cr_pl']:.4f} "
              f"s {x['s']:.2f} {x['bind']} feas {x['feas']} {x['hard']} p/rev {x['pitch_pr']:.2f}")


if __name__ == "__main__":
    main()
