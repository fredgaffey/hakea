import glob, json, os
SP = os.path.dirname(os.path.abspath(__file__))
DROP = ("cqc_mech_margin", "cqc_mech_side")
seen = {}
for f in sorted(glob.glob(f"{SP}/**/*.log", recursive=True) + glob.glob(f"{SP}/**/*.txt", recursive=True)):
    for ln in open(f, errors="ignore"):
        if ln.startswith("SUM "):
            _, name, js = ln.split(" ", 2)
            s = json.loads(js.replace("Infinity", "1e999"))
            seen[name] = (os.path.relpath(f, SP), s)
for name, (f, s) in sorted(seen.items(), key=lambda a: -a[1][1]["cr_pl"]):
    v = s["viol"]
    vm = {k: x for k, x in v.items() if k not in DROP}
    print(f"| {name} | {f} | {s['cr_pl']:.4f} | {s['feas']} | {not vm} | {s['mech']:.4f}{' FLIP' if s['flip'] else ''} "
          f"| {s['ph']:.1f}/{100*s['gain']:.0f} | {sorted(vm) or '-'} |")
