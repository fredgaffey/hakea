"""Hakea vehicle spec v0: mass budgets (est.) -> per-rotor mass -> lever_2307 evaluation (T0.7/1.0/1.25)
+ hover power (vehicle.electrical_power, ctrl_mod 0.3). Usage: vehicle_v0.py [budget|run]."""
import sys, os, json
FB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, FB + "/study")
OUT = FB + "/study/results/vehicle_spec_v0.json"
from flexblade import vehicle as V, robust, presets
from flexblade.schema import BatterySpec
import lever_2307 as L

D6 = L.load("6in_tt_strap_steelx_2307")
RM = V.rotor_mass(D6)
ROTOR = 1e3 * (RM["blades"] + RM["flexure"] + RM["hub"]) + 2.0   # + prop nut/clamp 2 g est.
PACK = {a: 1e3 * V.make_pack(BatterySpec(chemistry="lipo", S=6, capacity_Ah=a / 1e3)).mass
        for a in (450, 650, 850, 1100)}
CAP = {"single_min": 450, "single_cam": 850}  # mAh, default 650
# grams; all est. except motor (~35 g, notes/sim/am32-ramp-study.md:437)
BASE1 = dict(motor=35, rotor=ROTOR, fc=7, rx=2, encoder=3, wiring=8, body=15, hardware=5, vanes=15, servo=5)
BUILDS = {
    "single_min": dict(BASE1, esc=7, legs=6, battery=PACK[450]),
    "single_norm": dict(BASE1, esc=15, legs=8, battery=PACK[650]),
    "single_cam": dict(BASE1, esc=15, legs=8, camera=14, battery=PACK[850]),
    "coax_norm": dict(motor=70, rotor=2 * ROTOR, fc=7, rx=2, encoder=6, esc=15, wiring=12, body=30, legs=10,
                      hardware=6, battery=PACK[650]),
    "tandem_norm": dict(motor=70, rotor=2 * ROTOR, fc=7, rx=2, encoder=6, esc=15, wiring=14, body=28, legs=8,
                        hardware=6, battery=PACK[650]),
}
NROT = {"coax_norm": 2, "tandem_norm": 2}
KAPPA = {"coax_norm": 1.15}  # coax interference power factor, est.


def auw(b):
    return sum(BUILDS[b].values()) / 1e3


def cases():
    c = {b: (L.op(D6, vehicle_mass=auw(b) / NROT.get(b, 1)), L.J6) for b in BUILDS}
    c["single_norm_J3.46"] = (c["single_norm"][0], 3.46e-6)
    s5 = presets.scale_design(D6, 5.0)
    s5 = L.op(s5.model_copy(update=dict(motor=D6.motor)), vehicle_mass=auw("single_norm"))
    c["single_norm_5in"] = (s5, L.J6 * (5 / 6) ** 3)
    return c


def main():
    print("rotor g", {k: round(v * 1e3, 2) for k, v in RM.items()}, "pack g", {k: round(v, 1) for k, v in PACK.items()})
    for b, m in BUILDS.items():
        print(b, round(1e3 * auw(b), 1), {k: round(v, 1) for k, v in m.items()})
    if sys.argv[1:2] != ["run"]:
        return
    from concurrent.futures import ProcessPoolExecutor
    sel = sys.argv[2:]
    items = [(n, t, dv) for n, (d, J) in cases().items() if not sel or n in sel
             for t, dv in robust.variants(d, L.TF, None, [J])]
    with ProcessPoolExecutor(3) as ex:
        res = list(ex.map(L.ev, items, chunksize=1))
    for x in res:
        if "err" in x or not x["tag"].startswith("T1:"):
            continue
        b = x["case"].split("_J")[0].replace("_5in", "")
        n = NROT.get(b, 1)
        P, _ = V.electrical_power(D6.motor, [x["rpm"]] * n, [x["Q"]] * n, 0.3)
        P = (P - 1.5) * KAPPA.get(b, 1.0) + 1.5
        Wh = 6 * 3.7 * CAP.get(b, 650) / 1e3
        x.update(auw=auw(b), P_hover=P, Wh=Wh, hover_min=Wh * 0.8 / P * 60)
    old = json.load(open(OUT)) if (sel and os.path.exists(OUT)) else []
    old = [x for x in old if x["case"] not in {r["case"] for r in res}] + res
    json.dump(old, open(OUT, "w"), indent=1, default=float)
    for x in res:
        if "err" in x:
            print(x["case"], x["tag"], "ERR", x["err"]); continue
        print(f"{x['case']:18s} {x['tag'][:5]} rpm {x['rpm']:6.0f} Q {x['Q']:.4f} I0 {x['I0']:.1f} "
              f"cr_pl {x['cr_pl']:.4f} cr {x['cr']:.4f} {x['bind']} feas {x['feas']} {x['hard']} "
              f"p/rev {x['pitch_pr']:.2f} capI {x['capI']:.2f} T {x['capT']:.2f} "
              + (f"P {x['P_hover']:.1f} W {x['hover_min']:.1f} min" if 'P_hover' in x else ""))


if __name__ == "__main__":
    main()
