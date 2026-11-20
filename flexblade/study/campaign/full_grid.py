"""Large resumable grid for the 2-day campaign.

Concept families × sizes 5–8" with the Hakea 2307 motor at 0.6 kg, AM32 ramp 255, PA6-CF
blade with mass balance (base from study/reinforce_grid.base_for). Results stream to
full_grid/results.jsonl; reruns skip tags already done. SUMMARY.md is rewritten every
2000 results.

  PYTHONPATH=flexblade nohup python flexblade/study/campaign/full_grid.py --hosts local:12,mac:5 &
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import reinforce_grid as rg  # noqa: E402
from flexblade.schema import Layup  # noqa: E402

rg.MOTOR, rg.MASS = "tmotor_2307_1300kv", 0.6
SIZES = (5, 6, 7, 8)
TPUS = ("tpu_95a", "tpu_64d")
SKEWS = (30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0)
THICK = (0.001, 0.0015, 0.002, 0.0025, 0.003, 0.0035, 0.004)
HR = (0.8, 1.0, 1.2)            # × preset hinge_radius
LEN = (0.6, 0.8, 1.0, 1.25)     # × preset flexure length
WID = (0.8, 1.0, 1.25)          # × preset width


def variants(base, D):
    f0 = base.flexure
    out = []

    def mk(tag, **kw):
        d = base.model_copy(deep=True)
        f = d.flexure
        tension = kw.pop("tension", None)
        for k, v in kw.items():
            setattr(f, k, v)
        f.tension_member = rg.strap(d, tension) if tension else None
        d.name = f"{D}in_{tag}"
        out.append((dict(size=D, method=tag.split("|")[0], tag=f"{D}|{tag}"), d))

    for tpu, sk, t, hr, ln, wd in itertools.product(TPUS, SKEWS, THICK, HR, LEN, WID):
        geo = dict(skew_deg=sk, thickness=t, hinge_radius=f0.hinge_radius * hr,
                   length=f0.length * ln, width=f0.width * wd)
        g = f"sk{sk:.0f}|t{t*1e3:.1f}|hr{hr}|L{ln}|w{wd}"
        for dia, pos in itertools.product((0.001, 0.0012, 0.0014, 0.0016, 0.0018, 0.002),
                                          ((0.5,), (0.3, 0.7), (0.2, 0.5, 0.8))):
            if dia >= t:
                continue
            mk(f"rod|{tpu}|d{dia*1e3:.1f}|n{len(pos)}|{g}", topology="skewed_single_blade",
               layup=Layup(kind="rod_insert", material=tpu, insert_material="cf_rod_pultruded",
                           insert_diameter=dia, insert_positions=list(pos)), **geo)
        for sd in (0.0006, 0.0009, 0.0012, 0.0016):
            mk(f"strap|{tpu}|d{sd*1e3:.1f}|{g}", topology="skewed_single_blade",
               layup=Layup(kind="solid", material=tpu), tension=sd, **geo)
    for tpu, sk, bd, bl, sd, pt in itertools.product(
            TPUS, SKEWS, (0.0012, 0.0015, 0.002, 0.0025, 0.003), (0.008, 0.012, 0.016, 0.02, 0.025),
            (0.0, 0.0006, 0.001, 0.0014), (0.001, 0.0015, 0.0025)):
        mk(f"tbar|{tpu}|b{bd*1e3:.1f}|L{bl*1e3:.0f}|s{sd*1e3:.1f}|p{pt*1e3:.1f}|sk{sk:.0f}",
           topology="torsion_bar", skew_deg=sk, bar_diameter=bd, bar_length=bl, thickness=pt,
           layup=Layup(kind="solid", material=tpu), tension=sd or None)
    for sk, t, ln, wd, gap in itertools.product(SKEWS, (0.0006, 0.0008, 0.001, 0.0012, 0.0015),
                                                LEN, WID, (0.0008, 0.0012)):
        mk(f"cfsym|sk{sk:.0f}|t{t*1e3:.1f}|L{ln}|w{wd}|g{gap*1e3:.1f}",
           topology="skewed_cross_blade_sym", skew_deg=sk, thickness=t, cross_gap=gap,
           length=f0.length * ln, width=f0.width * wd, layup=Layup(kind="solid", material="pa6_cf"))
    return out


def write_summary(rows, out):
    lines = ["| size | method | feasible/done | best | ctrl | pitch1p° | max util | governing |",
             "|---|---|---|---|---|---|---|---|"]
    for D in sorted({r["size"] for r in rows}):
        for m in sorted({r["method"] for r in rows}):
            grp = [r for r in rows if r["size"] == D and r["method"] == m and r.get("ok")]
            if not grp:
                continue
            feas = [r for r in grp if r["converged"] and (r["max_util"] or 9e9) <= 1.0
                    and abs(r.get("split_deg") or 0) <= 2.0]
            best = max(feas, key=lambda r: r["control_ratio"] or 0) if feas else \
                min(grp, key=lambda r: r["max_util"] or 9e9)
            lines.append(f"| {D}\" | {m} | {len(feas)}/{len(grp)} | {best['tag']} | "
                         f"{(best['control_ratio'] or 0):.3f} | {(best['pitch_1p_deg'] or 0):.2f} | "
                         f"{(best['max_util'] or 0):.2f} | {best['governing']} |")
    feas = sorted([r for r in rows if r.get("ok") and r["converged"] and (r["max_util"] or 9e9) <= 1
                   and abs(r.get("split_deg") or 0) <= 2], key=lambda r: -(r["control_ratio"] or 0))
    lines += ["", "Top 25 feasible overall:", ""] + [
        f"- {r['tag']}: ctrl {r['control_ratio']:.3f}, pitch {r['pitch_1p_deg']:.2f}°, "
        f"util {r['max_util']:.2f} ({r['governing']})" for r in feas[:25]]
    (out / "SUMMARY.md").write_text("\n".join(lines) + f"\n\n{len(rows)} results, {time.ctime()}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hosts", default="local:12,mac:5")
    ap.add_argument("--out", default=str(HERE / "full_grid"))
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    res = out / "results.jsonl"
    rows = [json.loads(l) for l in open(res)] if res.exists() else []
    done = {r["tag"] for r in rows}
    jobs = []
    for D in SIZES:
        b = rg.base_for(D)
        b.blade.elastic_axis_chord_frac = 0.43
        jobs += [j for j in variants(b, D) if j[0]["tag"] not in done]
    print(f"{len(jobs)} to do ({len(done)} done)", flush=True)
    from flexblade.remote import HostPool
    import subprocess
    hosts = []
    for part in a.hosts.split(","):
        name = part.split(":")[0]
        if name != "local":
            ok = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", name, "true"],
                                capture_output=True).returncode == 0
            print(f"host {name}: {'up' if ok else 'unreachable, skipped this pass'}", flush=True)
            if not ok:
                continue
        hosts.append(part)
    a.hosts = ",".join(hosts)
    t0 = time.time()
    with open(res, "a") as fj, HostPool(a.hosts, what=["trim", "modes", "harmonic", "checks"]) as pool:
        for n, (i, rec) in enumerate(pool.imap([d for _, d in jobs]), 1):
            if "died" in str(rec.get("error", "")):
                continue                       # host dropped out (Mac asleep/away): retry next pass
            row = rg.summarise(jobs[i][0], rec)
            rows.append(row)
            fj.write(json.dumps(row) + "\n")
            fj.flush()
            if n % 2000 == 0:
                write_summary(rows, out)
                print(f"{n}/{len(jobs)} {time.time()-t0:.0f}s", flush=True)
    write_summary(rows, out)
    print("done", flush=True)


if __name__ == "__main__":
    main()
