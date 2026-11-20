"""Grid study: which reinforcement method stops TPU-flexure creep while keeping cyclic authority.

Methods (flexure only; blade layup from the size preset):
  mono    integral_taper monolithic TPU (control)
  rod     skewed_single_blade TPU plate + embedded rods (CF pultruded / spring steel),
          centred (on the bending neutral line) or spread across the width
  strap   TPU plate + radial tension strap through the hinge point (line meets hinge axis)
  skin    TPU core + PA6-CF skin
  tbar    torsion bar on the hinge axis (+TPU plate) + optional radial strap
All at the motor's 1/rev torque capability. Run:
  PYTHONPATH=flexblade python flexblade/study/reinforce_grid.py --hosts local:20,mac:9 --out DIR
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import time
from pathlib import Path

from flexblade import api, fact, motor, presets
from flexblade.schema import FlexureMember, Layup, PrintOrientation

SIZES = (5, 7, 10)
TPUS = ("tpu_95a", "tpu_64d")
SKEWS = (35.0, 45.0, 55.0)
THICK = (0.0015, 0.0025, 0.0035)


def base_for(D):
    d = presets.all_presets()[f"{D}in_tpu_rod"].model_copy(deep=True)
    # stiff PA6-CF blade (TPU blade creeps on its own centrifugal load) + chordwise mass
    # balance (attachment line at 0.43 c removes the static blade-to-blade pitch split)
    d.blade.layup = Layup(kind="solid", material="pa6_cf")
    d.blade.elastic_axis_chord_frac = 0.43
    d.motor.duty_slew_max = 2550.0   # AM32 max_ramp 255 (stock 16 -> 160/s clips the 1/rev pulse)
    r0 = api.evaluate(d, ["trim"])
    cap = motor.capability(d.motor, r0.trim.rpm, r0.trim.torque)
    d.op.torque_mod_frac = float(cap.dQ_max_1p / r0.trim.torque)
    return d


INSERT_MATS = ("cf_rod_pultruded", "steel_302_spring")
STRAP_MAT = "cf_rod_pultruded"


def strap(d, dia, mat=None):
    mat = mat or STRAP_MAT
    H0 = fact.hinge_axis(d)[0]
    f = d.flexure
    x0, x1 = f.hub_radius, 2 * H0[0] - f.hub_radius
    return FlexureMember(kind="wire", p0=(x0, 0, float(H0[2])), p1=(x1, 0, float(H0[2])),
                         width=dia, thickness=dia, width_dir=(0, 0, 1), shape="circle",
                         layup=Layup(kind="solid", material=mat),
                         orientation=PrintOrientation(build_dir=(0, 0, 1), raster_dir=(1, 0, 0)))


def variants(base, D):
    out = []

    def mk(tag, **kw):
        d = base.model_copy(deep=True)
        f = d.flexure
        for k, v in kw.items():
            if k != "tension":
                setattr(f, k, v)
        if kw.get("tension"):
            f.tension_member = strap(d, kw["tension"])
        else:
            f.tension_member = None
        d.name = f"{D}in_{tag}"
        out.append((dict(size=D, method=tag.split("|")[0], tag=tag), d))

    for tpu, sk, t in itertools.product(TPUS, SKEWS, THICK):
        mk(f"mono|{tpu}|sk{sk:.0f}|t{t*1e3:.1f}", topology="integral_taper", skew_deg=sk, thickness=t,
           layup=Layup(kind="solid", material=tpu))
        for mat, dia, pos in itertools.product(INSERT_MATS,
                                               (0.0008, 0.0012, 0.0016, 0.002), ((0.5,), (0.3, 0.7))):
            if dia >= t:
                continue
            mk(f"rod|{tpu}|{mat[:5]}|d{dia*1e3:.1f}|{'c' if len(pos)==1 else 's'}|sk{sk:.0f}|t{t*1e3:.1f}",
               topology="skewed_single_blade", skew_deg=sk, thickness=t,
               layup=Layup(kind="rod_insert", material=tpu, insert_material=mat,
                           insert_diameter=dia, insert_positions=list(pos)))
        for sd in (0.0006, 0.0009, 0.0012, 0.0016):
            mk(f"strap|{tpu}|d{sd*1e3:.1f}|sk{sk:.0f}|t{t*1e3:.1f}", topology="skewed_single_blade",
               skew_deg=sk, thickness=t, layup=Layup(kind="solid", material=tpu), tension=sd)
        for st in (0.0002, 0.0004):
            mk(f"skin|{tpu}|s{st*1e3:.1f}|sk{sk:.0f}|t{t*1e3:.1f}", topology="skewed_single_blade",
               skew_deg=sk, thickness=t, layup=Layup(kind="core_skin", material=tpu,
                                                     skin_material="pa6_cf", skin_thickness=st))
    for tpu, sk, bd, bl, sd in itertools.product(TPUS, SKEWS, (0.0015, 0.002, 0.0025),
                                                 (0.012, 0.016, 0.02), (0.0, 0.0006, 0.001)):
        mk(f"tbar|{tpu}|b{bd*1e3:.1f}|L{bl*1e3:.0f}|s{sd*1e3:.1f}|sk{sk:.0f}", topology="torsion_bar",
           skew_deg=sk, bar_diameter=bd, bar_length=bl, thickness=0.0015, bar_material=STRAP_MAT,
           layup=Layup(kind="solid", material=tpu), tension=sd or None)
    return out


def summarise(meta, rec):
    row = dict(meta)
    if "error" in rec:
        row.update(ok=False, error=str(rec["error"])[:200])
        return row
    r = rec["results"]
    h = r.get("harmonic") or {}
    checks = r.get("checks") or []
    worst = max(checks, key=lambda c: c["utilisation"]) if checks else None
    by = {}
    for c in checks:
        by[c["name"]] = max(by.get(c["name"], 0.0), c["utilisation"])
    warns = r.get("warnings") or []
    row.update(
        ok=True, host=rec.get("host"),
        control_ratio=h.get("control_ratio"),
        pitch_1p_deg=(h.get("blades") or [{}])[0].get("pitch_amp_deg"),
        tilt_deg=h.get("tilt_equiv_deg"),
        max_util=worst["utilisation"] if worst else None,
        governing=f'{worst["name"]}:{worst["member"]}' if worst else "",
        creep=by.get("creep_strain"), fatigue=max(by.get("fatigue_strain", 0), by.get("fatigue_stress", 0)),
        static=by.get("static_stress"), interlayer=by.get("interlayer"),
        slew=by.get("esc_duty_slew"),
        split_deg=(r.get("summary") or {}).get("trim_diff_pitch_deg"),
        lag_per_rev=(r.get("summary") or {}).get("lag_freq_per_rev"),
        converged=not any("not converged" in w for w in warns),
    )
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hosts", default="local:20,mac:9")
    ap.add_argument("--out", default="flexblade/study/reinforce")
    ap.add_argument("--sizes", default=",".join(map(str, SIZES)))
    ap.add_argument("--printed", action="store_true",
                    help="fully FDM: rods/strap/bar printed in PA6-CF / PETG-CF instead of inserts")
    a = ap.parse_args()
    global INSERT_MATS, STRAP_MAT
    if a.printed:
        INSERT_MATS, STRAP_MAT = ("pa6_cf", "petg_cf"), "pa6_cf"
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    jobs = []
    for D in map(int, a.sizes.split(",")):
        jobs += variants(base_for(D), D)
    print(f"{len(jobs)} designs", flush=True)
    from flexblade.remote import HostPool
    rows, t0 = [], time.time()
    with open(out / "results.jsonl", "w") as fj, HostPool(a.hosts, what=["trim", "modes", "harmonic", "checks"]) as pool:
        for n, (i, rec) in enumerate(pool.imap([d for _, d in jobs]), 1):
            row = summarise(jobs[i][0], rec)
            rows.append(row)
            fj.write(json.dumps(row) + "\n")
            fj.flush()
            if n % 100 == 0:
                print(f"{n}/{len(jobs)} {time.time()-t0:.0f}s", flush=True)
    keys = sorted({k for r in rows for k in r})
    with open(out / "results.csv", "w", newline="") as fc:
        w = csv.DictWriter(fc, keys)
        w.writeheader()
        w.writerows(rows)
    # headline: per size × method, best feasible (max_util ≤ 1, converged) by control_ratio,
    # else the least-overloaded
    lines = ["| size | method | feasible/total | best design | ctrl | pitch1p° | max util | governing |",
             "|---|---|---|---|---|---|---|---|"]
    for D in sorted({r["size"] for r in rows}):
        for m in ("mono", "rod", "strap", "skin", "tbar"):
            grp = [r for r in rows if r["size"] == D and r["method"] == m and r.get("ok")]
            if not grp:
                continue
            feas = [r for r in grp if r["converged"] and (r["max_util"] or 9e9) <= 1.0
                    and abs(r.get("split_deg") or 0) <= 2.0]
            best = (max(feas, key=lambda r: r["control_ratio"] or 0) if feas
                    else min(grp, key=lambda r: r["max_util"] or 9e9))
            lines.append(f"| {D}\" | {m} | {len(feas)}/{len(grp)} | {best['tag']} | "
                         f"{(best['control_ratio'] or 0):.3f} | {(best['pitch_1p_deg'] or 0):.2f} | "
                         f"{(best['max_util'] or 0):.2f} | {best['governing']} |")
    (out / "SUMMARY.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"done {time.time()-t0:.0f}s, errors {sum(not r.get('ok') for r in rows)}")


if __name__ == "__main__":
    main()
