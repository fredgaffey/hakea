"""sa.py-basis screen (materials_5in.run): robust.variants(d, (0.7,1,1.25), None, [0.91e-6]) -> rs.metrics.
usage: score.py <variant> [resolve]"""
import json, sys, time
from pathlib import Path
STUDY = Path("study").resolve()
sys.path.insert(0, str(STUDY))
from flexblade import api, robust          # noqa: E402
from flexblade.schema import Design         # noqa: E402
import run_study as rs                      # noqa: E402

name = sys.argv[1]
d = Design.model_validate_json((STUDY / "designs/5in_v8s_2307_m245_301.json").read_text())
f = d.flexure
U = {"lead": {},
     "c_pair_3p3": dict(strap_strands=2, strap_radius=3.3e-3, strap_strand_diameter=1.3e-3 / 2 ** 0.5),
     "c_pair_1p0": dict(strap_strands=2, strap_radius=1.0e-3, strap_strand_diameter=1.3e-3 / 2 ** 0.5),
     "d_gap_p3p5": dict(cross_gap=f.cross_gap + 3.5e-3),
     "c33_L13": dict(strap_strands=2, strap_radius=3.3e-3, strap_strand_diameter=1.3e-3 / 2 ** 0.5, strap_length=13.6e-3),
     "c33_d13": dict(strap_strands=2, strap_radius=3.3e-3, strap_strand_diameter=1.3e-3),
     "c25_L13": dict(strap_strands=2, strap_radius=2.5e-3, strap_strand_diameter=1.3e-3 / 2 ** 0.5, strap_length=13.6e-3),
     "c32_L0": dict(strap_strands=2, strap_radius=3.2e-3, strap_strand_diameter=1.3e-3 / 2 ** 0.5, strap_length=0.0),
     "c32": dict(strap_strands=2, strap_radius=3.2e-3, strap_strand_diameter=1.3e-3 / 2 ** 0.5),
     "a045": dict(strap_strand_diameter=0.45e-3),
     "a045_g1p5": dict(strap_strand_diameter=0.45e-3, cross_gap=f.cross_gap + 1.5e-3),
     "a045_g2p0": dict(strap_strand_diameter=0.45e-3, cross_gap=f.cross_gap + 2.0e-3),
     "a060_g2p5": dict(strap_strand_diameter=0.60e-3, cross_gap=f.cross_gap + 2.5e-3)}[name]
d = d.model_copy(update=dict(flexure=f.model_copy(update=U)))
if len(sys.argv) > 2:
    d = rs.resolve_candidate(d, 6)
t0 = time.time()
pts = []
for tag, dv in robust.variants(d, (0.7, 1.0, 1.25), None, [0.91e-6]):
    r = api.evaluate(dv, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    m = rs.metrics(dv, r)
    ck = {c["name"]: c for c in r["checks"]}
    st = ck.get("strap_tension", {})
    top = sorted(((c["name"] + ":" + str(c.get("member")), round(c["utilisation"], 3)) for c in r["checks"]), key=lambda x: -x[1])[:4]
    S = r["summary"]
    pts.append(dict(tag=tag, cr_pl=m.get("cr_pl"), feas=m.get("feasible_pl"), bind=m.get("pulse_binding"),
                    hard=sorted((m.get("viol_hard") or {}).keys()), top=top, strapN=st.get("value"),
                    strap_u=st.get("utilisation"), pitch_pr=S.get("pitch_freq_per_rev"), rpm=S.get("rpm"), anc_u=ck.get("strap_anchor", {}).get("utilisation"),
                    k={q: S.get(q) for q in ("hinge_k_free", "pitch_k_struct", "pitch_k_eff", "pitch_k_prop", "pitch_k_aero", "root_cf")},
                    skeys=sorted(S.keys()) if not pts else None))
print(json.dumps(dict(name=name + ("_res" if len(sys.argv) > 2 else ""), secs=round(time.time() - t0), pts=pts)))
