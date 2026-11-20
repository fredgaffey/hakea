"""Vehicle sizing sweep: layout x rotor D x S x battery x payload.

    python sweep.py run [--workers 8]      -> results.jsonl (resumable)
    python sweep.py report                 -> VEHICLE.md

Rotor: TPU+CF-rod flexure (presets "tpu_rod"), PA6-CF blade, elastic axis 0.43c, AM32 slew 2550/s.
Motor kv rewound per (S, rpm) so hover back-EMF = 57 % of mid-SoC loaded voltage.
"""
from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

HERE = Path(__file__).parent
OUT = HERE / "results.jsonl"
LAYOUTS = ("single", "coaxial")
SIZES = (5, 6, 7, 8)
SERIES = (3, 4, 6)
PAYLOADS = (0.0, 0.05, 0.10, 0.20)
BATTS = ([("lipo", 1, c) for c in (1.0, 1.5, 2.2, 3.3)] + [("lihv", 1, c) for c in (1.5, 2.2)] +
         [("liion_21700", 1, None), ("liion_21700", 2, None), ("liion_18650", 3, None)])


def configs():
    for lay in LAYOUTS:
        for D in SIZES:
            for S in SERIES:
                for chem, P, cap in BATTS:
                    for pl in PAYLOADS:
                        yield dict(layout=lay, D=D, S=S, chem=chem, P=P, cap=cap, payload=pl)


def key(c):
    return f"{c['layout']}|{c['D']}|{c['S']}|{c['chem']}|{c['P']}|{c['cap']}|{c['payload']}"


def work(c):
    from flexblade.schema import BatterySpec, VehicleSpec
    from flexblade.vehicle import evaluate_mission
    try:
        spec = VehicleSpec(layout=c["layout"], diameter_in=float(c["D"]), payload_kg=c["payload"],
                           battery=BatterySpec(chemistry=c["chem"], S=c["S"], P=c["P"],
                                               capacity_Ah=c["cap"]))
        r = evaluate_mission(spec, full=True)
        s = r.summary()
        s.update(key=key(c), bad=r.checks_bad, warn=len(r.warnings))
        return s
    except Exception as ex:   # noqa: BLE001
        return dict(key=key(c), error=f"{type(ex).__name__}: {ex}")


def run(workers=8):
    done = set()
    if OUT.exists():
        done = {json.loads(l)["key"] for l in OUT.open()}
    todo = [c for c in configs() if key(c) not in done]
    print(f"{len(todo)} to run ({len(done)} done)", flush=True)
    with Pool(min(workers, 8)) as p, OUT.open("a") as f:
        for i, s in enumerate(p.imap_unordered(work, todo, chunksize=2)):
            f.write(json.dumps(s) + "\n")
            f.flush()
            if i % 50 == 0:
                print(i, flush=True)


def load():
    return [r for r in (json.loads(l) for l in OUT.open()) if "error" not in r]


def pareto(rows, xk, yk):
    """Max-max Pareto front on (xk, yk)."""
    rows = [r for r in rows if r.get(xk) is not None and r.get(yk) is not None]
    rows.sort(key=lambda r: (-r[xk], -r[yk]))
    out, best = [], -1e18
    for r in rows:
        if r[yk] > best:
            out.append(r)
            best = r[yk]
    return out


def cfg(r):
    b = f"{r['chem']} {r['S']}S" + (f"{r['P']}P" if r["chem"].startswith("liion") else f" {r['Ah']:.1f}Ah")
    return b


CAVEATS = """
## Model and caveats (read before trusting any number)

- **Aero**: BEM hover thrust is probably ~23 % optimistic (first coarse CFD, see
  `study/cfd_validation.md`).  The "BEM -23 %" endurance column re-trims with the required thrust
  inflated to W/0.77 (rpm up, power ~+40 %); the first column is the raw BEM.  Treat reality as
  nearer the derated column.  Coaxial interference is the actuator-disk model in `coaxial.py`.
- **Battery data are typical, not measured**: Molicel P45B 4.5 Ah/69 g/45 A/13 mOhm (retailer sheets,
  not the official datasheet), P50B 60 A unverified, 18650 and LiPo/LiHV values assumed from typical
  FPV packs (150 Wh/kg cell, 10 % wiring overhead; Li-ion 15 % for holders/strips).  Constant-power
  discharge with OCV table and series R; no Peukert/temperature/ageing.  Usable fractions 0.85
  (LiPo) / 0.92 (Li-ion) include no landing reserve beyond that.
- **Mass budget** (frame 60 g (D/5)^1.5, x1.5 coax; motors 33/55 g; ESC 10-14 g; avionics 28 g;
  hub 6 g (D/5)^2; blades/flexure from geometry x PA6-CF/TPU density) is +-30 %; frame is the
  largest guess.  AUW is a straight sum -- the sweep does not constrain AUW or size the rotor to
  the pack, so "best endurance" tends to pick the largest pack; the optimum for a real build is a
  max-AUW or max-power-limit question, not shown here.
- **Motor**: kv rewound per supply (R_eff = R_fixed + R_wind (kv_ref/kv)^2, max current 40 A /
  50 A) so hover back-EMF = 57 % of mid-SoC loaded voltage; the library 2307 R, L, b, T0 are the
  Hakea fit; L is a +-30 % estimate.  Thermal limits are not modelled.
- **control_ratio is NOT an optimised flexure result.**  The flexure is the scaled `tpu_rod`
  placeholder at each size/mass; control authority depends strongly on where the flap/lag/pitch
  resonances land relative to the hover rpm (note the jumps between 5-6 in and 8 in).  Re-tune the
  flexure per vehicle with `optimize` before comparing layouts on control_ratio.  Pulse amplitude is
  capped at 1.0 x mean torque; at this cap the pulse slew/hub-speed oscillation may already exceed
  what the ESC/hub tolerates (slew util not a gate here).  "typ" = 0.3 x mean torque (linear
  scaling).
- Extra copper from cyclic pulsing at the typical 0.3 amplitude is only ~4.5 % of motor copper
  (a few W) -- negligible against hover power; ESC switching 2 % + idle, avionics 1.5 W.
- Feasibility flags: hover battery C <= rated; peak (1.3 x hover) <= 1.5 x rated; motor
  current; end-of-discharge pulse headroom >= 0.3; back-EMF <= 80 % of loaded end voltage;
  structural checks (static, creep, fatigue, interlayer) <= 1.
"""


def report():
    rows = load()
    errs = sum(1 for l in OUT.open() if "error" in json.loads(l))
    feas = [r for r in rows if r["feasible"]]
    L = ["# Vehicle sizing study (flexblade.vehicle)", "",
         f"{len(rows)} configs evaluated ({errs} errors), {len(feas)} feasible "
         "(battery current <= rated, motor current, end-of-discharge pulse headroom >= 0.3 of mean "
         "torque, back-EMF <= 80 % of loaded supply, all structural checks <= 1).", "",
         "Rotor: TPU + CF-rod flexure (`tpu_rod`), PA6-CF blade, elastic axis 0.43c, AM32 slew 2550/s, "
         "5-8 in, 2 blades. Motor kv rewound so hover back-EMF = 57 % of mid-SoC loaded voltage "
         "(2307-class, 2806.5-class above 150 W shaft/motor). Pulse amplitude capped at 1.0 x mean "
         "torque. `ctrl_ratio` = hub moment/(W R) at the capped amplitude and mid-SoC voltage; "
         "`ctrl@end` is scaled to the end-of-discharge voltage; `typ` is at 0.3 x mean torque.", ""]
    L += ["## Best battery per (layout, size, payload) -- max endurance, feasible", "",
          "| layout | D in | payload g | battery | AUW g | pack g | kv | hover W | endur min | endur min (BEM -23 %) | "
          "I hover A | C | ctrl@end | ctrl typ |", "|" + "---|" * 14]
    for lay in LAYOUTS:
        for D in SIZES:
            for pl in PAYLOADS:
                c = [r for r in feas if r["layout"] == lay and r["D"] == D and r["payload_g"] == pl * 1000]
                if not c:
                    L.append(f"| {lay} | {D} | {pl*1000:.0f} | none feasible | | | | | | | | | | |")
                    continue
                r = max(c, key=lambda r: r["endurance_min"])
                L.append(f"| {lay} | {D} | {pl*1000:.0f} | {cfg(r)} | {r['auw_g']:.0f} | {r['pack_g']:.0f} | "
                         f"{r['kv']:.0f} | {r['P_elec_W']:.0f} | {r['endurance_min']:.1f} | "
                         f"{r['endurance_min_derated']:.1f} | {r['I_hover_A']:.1f} | {r['C_hover']:.1f} | "
                         f"{r['control_ratio_end']:.4f} | {r['control_ratio_typ']:.4f} |")
    L += ["", "## Pareto: endurance vs payload (feasible, any size/battery, per layout)", "",
          "| layout | payload g | endur min | D in | battery | AUW g | ctrl@end |", "|" + "---|" * 7]
    for lay in LAYOUTS:
        for pl in PAYLOADS:
            c = [r for r in feas if r["layout"] == lay and r["payload_g"] == pl * 1000]
            if c:
                r = max(c, key=lambda r: r["endurance_min"])
                L.append(f"| {lay} | {pl*1000:.0f} | {r['endurance_min']:.1f} | {r['D']:.0f} | {cfg(r)} | "
                         f"{r['auw_g']:.0f} | {r['control_ratio_end']:.4f} |")
    for lay in LAYOUTS:
        for pl in (0.0, 0.10):
            c = [r for r in feas if r["layout"] == lay and r["payload_g"] == pl * 1000]
            f = pareto(c, "endurance_min", "control_ratio_end")
            L += ["", f"## Pareto: control_ratio@end vs endurance -- {lay}, payload {pl*1000:.0f} g", "",
                  "| endur min | ctrl@end | ctrl typ | D in | battery | AUW g | kv | hover W |", "|" + "---|" * 8]
            for r in f:
                L.append(f"| {r['endurance_min']:.1f} | {r['control_ratio_end']:.4f} | {r['control_ratio_typ']:.4f} | "
                         f"{r['D']:.0f} | {cfg(r)} | {r['auw_g']:.0f} | {r['kv']:.0f} | {r['P_elec_W']:.0f} |")
    # infeasibility reasons
    L += ["", "## Why configs fail (count of failed flags among all evaluated)", ""]
    fl = {}
    for r in rows:
        for k, v in r["flags"].items():
            if not v:
                fl[k] = fl.get(k, 0) + 1
    L += [f"- {k}: {v}" for k, v in sorted(fl.items(), key=lambda kv: -kv[1])]
    # coax vs single at equal payload / size
    L += ["", "## Coaxial vs single at the best battery (same D, payload)", "",
          "| D | payload g | single min | coax min | coax/single | single ctrl@end | coax ctrl@end |", "|" + "---|" * 7]
    for D in SIZES:
        for pl in PAYLOADS:
            def best(lay):
                c = [r for r in feas if r["layout"] == lay and r["D"] == D and r["payload_g"] == pl * 1000]
                return max(c, key=lambda r: r["endurance_min"]) if c else None
            a, b = best("single"), best("coaxial")
            if a and b:
                L.append(f"| {D} | {pl*1000:.0f} | {a['endurance_min']:.1f} | {b['endurance_min']:.1f} | "
                         f"{b['endurance_min']/a['endurance_min']:.2f} | {a['control_ratio_end']:.4f} | "
                         f"{b['control_ratio_end']:.4f} |")
    L += CAVEATS
    (HERE / "VEHICLE.md").write_text("\n".join(L) + "\n")
    print("wrote VEHICLE.md")


if __name__ == "__main__":
    if sys.argv[1] == "run":
        w = int(sys.argv[sys.argv.index("--workers") + 1]) if "--workers" in sys.argv else 8
        run(w)
    else:
        report()
