import sys, numpy as np
from flexblade import rotor, api, robust
from flexblade.schema import Design
d0 = Design.model_validate_json(open(sys.argv[1]).read())
mode = sys.argv[2]; Tf = float(sys.argv[3])
u = dict(strip_fatigue_mean="goodman")
if mode == "nopre":
    u.update(strip_preset_deg=[[[0.0]*6]*3]*2)
if mode == "nopl":
    u.update(strip_preset_deg=[[[0.0]*6]*3]*2, pre_lag_deg=0.0)
d0 = d0.model_copy(update=dict(flexure=d0.flexure.model_copy(update=u)))
d = [dv for t, dv in robust.variants(d0, (Tf,)) if t == f"T{Tf:g}"][0]
orig = rotor.checks


def wrap(model, ss, hd=None, forced=None):
    fe, st = model.fe, ss.state
    for e in fe.elements:
        if e.kind == "flexure":
            b, n = e.tag.split(".", 1); e.tag = f"{b}.{b}{n}"
    out = orig(model, ss, hd, forced)
    for e in fe.elements:
        if e.kind == "flexure":
            b, n = e.tag.split(".", 1); e.tag = f"{b}.{n[len(b):]}"
    ch = {}
    for e in fe.elements:
        if e.kind == "flexure":
            ch.setdefault(e.tag, []).append(e)
    for tag, els in sorted(ch.items()):
        n0, n1 = els[0].n0, els[-1].n1
        e1 = rotor.unit(st.x[n1] - st.x[n0])
        rv = rotor._logm(st.R[n1] @ st.R[n0].T)
        print(f"  {tag} rel-rot about e1 {np.degrees(rv@e1):+.2f} perp {np.degrees(rv-(rv@e1)*e1).round(2)} deg; "
              f"hub-end {np.degrees(rotor._logm(st.R[n0])).round(2)} blade-end {np.degrees(rotor._logm(st.R[n1])).round(2)}")
    for bi in model.blades:
        print(f"  hinge lag {np.degrees(rotor._logm(st.R[bi.master])@bi.hinge_dir):+.3f} deg, master rot "
              f"{np.degrees(rotor._logm(st.R[bi.master])).round(3)}")
    print("  preset_pitch", np.degrees(rotor.preset_pitch(model)).round(3), "pre_lag",
          np.degrees(rotor.pre_lag_rad(model.design)).round(3), "Omega", ss.Omega)
    return out


rotor.checks = wrap
r = api.evaluate(d, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
for c in r["checks"]:
    if "flex" in c["member"] and c["name"] in ("fatigue_stress", "strip_ltb", "static_stress"):
        print(f"{mode} T{Tf} {c['name']:15s} {c['member']:9s} u={c['utilisation']:.3f} {(c.get('note') or '')[:190]}")
