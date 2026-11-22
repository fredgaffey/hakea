"""First G10/FR4 crossed-strip score (results/2307_5_6in.md section 13): G10 variants of the
5" 301 lead, presets re-solved as the pipeline does (resolve_candidate(d, 6)), then a nominal
check screen at T1.0 / J 0.91e-6. The robust score is sa.py on the written json with the
T0.7/1.0/1.25 patch:  g10_v0.py screen <outdir>  |  g10_v0.py sa <design.json> <out.json>"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_study as rs
from flexblade import robust, api
from flexblade.schema import Design

ROOT = os.path.dirname(os.path.abspath(__file__))
LEAD = f"{ROOT}/designs/5in_v8s_2307_m245_301.json"
# name: (t m, n, w m, L factor); width scaled for pitch k ~ n E w t^3 / L (301 9 x 4 x 0.05, E 193)
VARIANTS = {"a": (0.2e-3, 1, 5.4e-3, 1.0), "b": (0.2e-3, 1, 8.1e-3, 1.5)}


def make(t, n, w, lf):
    d = Design.model_validate_json(open(LEAD).read())
    lay = d.flexure.layup.model_copy(update=dict(material="fr4_g10"))
    f = d.flexure.model_copy(update=dict(layup=lay, thickness=t, strip_laminae=n, width=w,
                                         length=d.flexure.length * lf, strip_lam_eta=0.0))
    return rs.resolve_candidate(d.model_copy(update=dict(flexure=f)), 6)


def screen(outdir):
    os.makedirs(outdir, exist_ok=True)
    for k, v in VARIANTS.items():
        d = make(*v)
        p = f"{outdir}/g10_{k}.json"
        open(p, "w").write(d.model_dump_json(indent=1))
        _, dv = robust.variants(d, (1.0,), None, [0.91e-6], None)[0]
        r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
        cs = sorted(r["checks"], key=lambda c: -c["utilisation"])
        print("VAR", k, v, "preset", json.dumps(d.flexure.strip_preset_deg)[:200], flush=True)
        for c in cs:
            if c["utilisation"] > 0.3 or c["name"] in ("interlayer", "creep_strain", "fatigue_strain",
                                                         "fatigue_stress", "static_stress"):
                print("  %-22s %-12s %.3f  %s" % (c["name"], c["member"], c["utilisation"],
                                                  (c.get("note") or "")[:110]), flush=True)


def sa(inp, outp):
    sys.path.insert(0, f"{ROOT}/i34tools")
    import sa as sam
    robust.THRUST_FACTORS = (0.7, 1.0, 1.25)
    os.environ.setdefault("SA_JHUB", "0.34e-6,0.91e-6,2.0e-6")
    sys.argv = ["sa.py", "file", inp, outp]
    sam.main()


if __name__ == "__main__":
    screen(sys.argv[2]) if sys.argv[1] == "screen" else sa(*sys.argv[2:4])
