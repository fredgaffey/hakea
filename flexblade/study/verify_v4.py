"""v4 of the 7in tt_strap_steelx: constrained-layer damped (CLD) strips, robust re-tune, clamp clearance.
Stages: rku | damp | search | robust | tol | floq | write | draw
    PYTHONPATH=flexblade python flexblade/study/verify_v4.py <stage>
Raw results -> study/results/verify_7in_v4.json (each stage merges its key). Reuses verify_v3 helpers
(perturbed, _eval_point, _floq) and run_study.metrics / robust.combine."""
from __future__ import annotations
import json, math, sys, random
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from flexblade import api, robust, fact                          # noqa: E402
from flexblade.optimize import set_design                        # noqa: E402
from flexblade.schema import Design                              # noqa: E402
import run_study as rs                                           # noqa: E402
import verify_v3 as V3                                           # noqa: E402

RES = HERE / "results" / "verify_7in_v4.json"
OUT = HERE / "designs" / "7in_tt_strap_steelx_v4.json"
V3D = V3.NOM
WORKERS = 3


def load():
    return json.loads(RES.read_text()) if RES.exists() else {}


def save(k, v):
    r = load(); r[k] = v; RES.write_text(json.dumps(r, indent=1, default=float))


# ------------------------------------------------------------------ 1. RKU constrained layer
# layer 1 = steel strip (base), 2 = viscoelastic, 3 = constraining layer.  Ross-Kerwin-Ungar
# 3-layer flexural stiffness with complex shear parameter g* = G2*/(E3 H3 H2 k^2); composite
# eta = Im EI*/Re EI*.  k: bending wavenumber of the strip's S-shaped cross-spring deflection
# (inflection at mid length, clamped ends) ~ pi/L.
VISCO = {  # name: (G' Pa at 100-300 Hz / 25 C, eta2, rho)   (datasheet / literature ranges)
    "3M ISD-112 0.05mm": (0.35e6, 1.0, 980, 5e-5),
    "3M VHB 9469 0.13mm": (0.6e6, 0.7, 960, 1.3e-4),
    "3M VHB 4910 0.5mm": (0.6e6, 0.7, 960, 5e-4),
    "TPU 95A 0.2mm print": (9e6, 0.12, 1210, 2e-4),
}
CONSTRAIN = {  # name: (E, H3, rho, fatigue strain allowable)
    "1095 shim 0.05mm": (200e9, 5e-5, 7850, 0.0028),
    "1095 shim 0.025mm": (200e9, 2.5e-5, 7850, 0.0028),
    "PET film 0.125mm": (4.5e9, 1.25e-4, 1390, 0.006),
    "PET film 0.25mm": (4.5e9, 2.5e-4, 1390, 0.006),
}


def rku(E1, H1, G2, eta2, H2, E3, H3, k):
    E2 = 3 * G2 * (1 + 1j * eta2)
    g = G2 * (1 + 1j * eta2) / (E3 * H3 * H2 * k * k)
    H21, H31 = (H1 + H2) / 2, (H1 + H3) / 2 + H2
    D = (E2 * H2 * (H21 - H31 / 2) + g * (E2 * H2 * H21 + E3 * H3 * H31)) / \
        (E1 * H1 + E2 * H2 / 2 + g * (E1 * H1 + E2 * H2 + E3 * H3))
    EI = (E1 * H1**3 / 12 + E2 * H2**3 / 12 + E3 * H3**3 / 12 - E2 * H2**2 / 12 * (H31 - D) / (1 + g)
          + E1 * H1 * D**2 + E2 * H2 * (H21 - D)**2 + E3 * H3 * (H31 - D)**2
          - (E2 * H2 / 2 * (H21 - D) + E3 * H3 * (H31 - D)) * (H31 - D) / (1 + g))
    EI0 = E1 * H1**3 / 12
    # constraining-layer peak strain / base-strip surface strain (membrane part scaled by g/(1+g))
    e3 = (abs(H31 - D) * abs(g / (1 + g)) + H3 / 2) / (H1 / 2)
    # base strip: neutral axis shift D raises its peak strain to (|D| + H1/2)/(H1/2)
    e1 = (abs(D) + H1 / 2) / (H1 / 2)
    return dict(eta=float(EI.imag / EI.real), EI_ratio=float(EI.real / EI0), g=abs(g),
                e3_ratio=float(e3), e1_ratio=float(e1))


def rku_table(t=1.5e-4, L=0.024):
    out = []
    for vn, (G2, eta2, rho2, H2) in VISCO.items():
        for cn, (E3, H3, rho3, ea3) in CONSTRAIN.items():
            for kf in (1.0, 2.0):
                r = rku(200e9, t, G2, eta2, H2, E3, H3, kf * math.pi / L)
                r.update(visco=vn, constrain=cn, k_over_piL=kf,
                         mass_ratio=(7850 * t + rho2 * H2 + rho3 * H3) / (7850 * t), e3_allow=ea3)
                out.append(r)
    return out


def stage_rku():
    save("rku", rku_table())
    for r in load()["rku"]:
        print(f"{r['visco']:22s} {r['constrain']:18s} k{r['k_over_piL']:.0f} eta {r['eta']:.3f} EI x{r['EI_ratio']:.2f} "
              f"g {r['g']:.2f} e3/e1 {r['e3_ratio']:.2f} e1x {r['e1_ratio']:.2f} m x{r['mass_ratio']:.2f}")


def cld_material(t, L, visco, constrain, derate=0.7):
    """steel strip variant carrying the CLD laminate: E x EI ratio (S x same so the stress check
    stays a base-strip strain check), rho x mass ratio, eta = derate * RKU composite (the ~3 mm
    clamp lands where the layer cannot shear, and edge effects of a 8-12 mm wide strip)."""
    G2, eta2, rho2, H2 = VISCO[visco]
    E3, H3, rho3, _ = CONSTRAIN[constrain]
    k = (1.5 * math.pi / L)          # between the two wavenumber bounds
    r = rku(200e9, t, G2, eta2, H2, E3, H3, k)
    m = (7850 * t + rho2 * H2 + rho3 * H3) / (7850 * t)
    eta = derate * r["eta"]
    return f"steel_1095_shim@E={r['EI_ratio']:.4f},S={r['EI_ratio']:.4f},rho={m:.4f},eta={eta:.4f}", r, eta


# ------------------------------------------------------------------ candidates
KNOBS = ["flexure.thickness", "flexure.width", "flexure.length", "flexure.hinge_radius", "flexure.skew_deg",
         "flexure.cross_angle_deg", "blade.elastic_axis_chord_frac", "flexure.strap_strand_diameter"]
CLD = dict(visco="3M VHB 9469 0.13mm", constrain="1095 shim 0.05mm")


def make(x: dict, cld: dict | None = CLD) -> Design:
    """v3 + knobs x; clamp fixes: crossing gap = strand + 0.4 mm (>= strand + 0.3 mm), pin dia 4x strand."""
    x = dict(x)
    s = x.get("flexure.strap_strand_diameter", V3D.flexure.strap_strand_diameter)
    x["flexure.cross_gap"] = round(s + 0.0004, 5)
    x["flexure.strap_pin_diameter"] = round(4 * s, 5)
    d = set_design(V3D, x)
    if cld:
        mat, _, _ = cld_material(d.flexure.thickness, d.flexure.length, cld["visco"], cld["constrain"])
        lay = d.flexure.layup.model_copy(update=dict(material=mat))
        d = d.model_copy(update=dict(flexure=d.flexure.model_copy(update=dict(layup=lay))))
    return d


def envelope(d: Design) -> dict:
    """flexure member extents (blade 1 frame, mm): z half-height incl. plate width, min x."""
    zs, xs = [], []
    for m in fact.generate_members(d):
        wd = np.asarray(m.width_dir, float); wd = wd / max(np.linalg.norm(wd), 1e-12)
        for p in (m.p0, m.p1):
            for sg in (1, -1):
                q = np.asarray(p) + sg * wd * m.width / 2
                zs.append(abs(q[2])); xs.append(q[0])
    return dict(z_max_mm=1e3 * max(zs), x_min_mm=1e3 * min(xs), hub_mm=1e3 * d.flexure.hub_radius)


def samples(n, seed=11):
    """random +-5 % combined (as verify_v3.cases), relative specs for perturbed()"""
    T, W, L, s = "flexure.thickness", "flexure.width", "flexure.length", "flexure.strap_strand_diameter"
    rnd = random.Random(seed); out = {}
    for i in range(n):
        u = lambda a: 1 + rnd.uniform(-a, a)
        out[f"random{i:02d}"] = [("mul", T, u(.05)), ("mul", W, u(.05)), ("mul", L, u(.05)),
                                 ("mul", "flexure.hinge_radius", u(.05)), ("rho", "", round(u(.05), 4)),
                                 ("mul", s, math.sqrt(u(.05))), ("add", "flexure.skew_deg", rnd.uniform(-1, 1)),
                                 ("add", "blade.elastic_axis_chord_frac", rnd.uniform(-.01, .01))]
    return out


def _eval(args):
    tag, d = args
    r = api.evaluate(d, ["trim", "modes", "harmonic", "checks"]).model_dump(mode="json")
    m = rs.metrics(d, r)
    f1 = r["summary"]["rpm"] / 60
    # 2nd-and-higher pitch modes vs 4P/5P/6P (distance in fraction of kP); flap modes ignored
    hm, hw = 9.0, ""
    for md in r["modes"]:
        pr = md["freq_hz"] / f1
        if "pitch" not in md["label"] or pr < 2.0:
            continue
        for k in (4, 5, 6):
            g = abs(pr / k - 1)
            if g < hm:
                hm, hw = g, f"{md['label']} {pr:.2f}/rev vs {k}P"
    m["hp_margin"], m["hp_which"] = hm, hw
    m["modes"] = [(md["label"], md["freq_hz"] / f1, md["damping_ratio"]) for md in r["modes"] if md["freq_hz"] > 0.05 * f1]
    return tag, m


def run_sets(sets: dict[str, Design]) -> dict:
    """sets: name -> design; each at 3 throttles. -> name -> combined summary"""
    jobs, owner = [], []
    for name, d in sets.items():
        for tag, dv in robust.variants(d):
            jobs.append((tag, dv)); owner.append(name)
    with ProcessPoolExecutor(WORKERS) as ex:
        outs = list(ex.map(_eval, jobs))
    res = {}
    for name in sets:
        ms = [(t, m) for (t, m), o in zip(outs, owner) if o == name]
        c = robust.combine([m for _, m in ms], [t for t, _ in ms])
        u = c.get("util", {})
        res[name] = dict(cr_min=c["control_ratio"], feasible=c["feasible"], viol=c["viol"],
                         res_margin=c.get("res_margin"), res_which=c.get("res_which"),
                         fatigue=max(u.get("fatigue_strain", 0), u.get("fatigue_stress", 0)),
                         util={k: round(v, 3) for k, v in u.items() if not k.startswith("member:")},
                         hp_margin=min(m["hp_margin"] for _, m in ms),
                         hp_which=min((m["hp_margin"], m["hp_which"]) for _, m in ms)[1],
                         cr={t: m["control_ratio"] for t, m in ms},
                         modes={t: m["modes"][:8] for t, m in ms})
    return res


def ok_targets(c):
    return c["feasible"] and (c["res_margin"] or 0) >= 0.22 and c["fatigue"] <= 0.7 and c["hp_margin"] >= 0.08


# ------------------------------------------------------------------ 2. damping study (v3 geometry)
def stage_damp():
    """eig zeta of v3 with bare vs CLD strips (same geometry), all modes"""
    out = run_sets({"v3 bare": make({}, None), "v3 + CLD": make({})})
    save("damp", out)
    for k, v in out.items():
        print(k, v["res_margin"], v["res_which"], v["hp_margin"], v["hp_which"], v["fatigue"])
        for t, ms in v["modes"].items():
            print("  ", t, [(l, round(f, 2), round(z, 4)) for l, f, z in ms[:5]])


# ------------------------------------------------------------------ 3. search
def stage_search(n=40, seed=3):
    rnd = random.Random(seed)
    grid = {"flexure.thickness": [1.3e-4, 1.5e-4], "flexure.width": [0.008, 0.010, 0.012],
            "flexure.length": [0.020, 0.024, 0.028], "flexure.hinge_radius": [0.016, 0.0175, 0.019],
            "flexure.skew_deg": [60.0, 65.0, 70.0], "flexure.cross_angle_deg": [110.0, 130.0, 150.0],
            "blade.elastic_axis_chord_frac": [0.36, 0.38, 0.40],
            "flexure.strap_strand_diameter": [0.0012, 0.0015, 0.0019]}
    cands = {"c00": {}}
    while len(cands) < n:
        x = {k: rnd.choice(v) for k, v in grid.items()}
        cands[f"c{len(cands):02d}"] = x
    res = run_sets({k: make(x) for k, x in cands.items()})
    for k in res:
        res[k]["x"] = cands[k]
    save("search", res)
    rank = sorted(res, key=lambda k: (-ok_targets(res[k]), -(res[k]["feasible"]), -min(res[k]["res_margin"] or 0, 0.25)
                                      - min(res[k]["hp_margin"], 0.1) + max(res[k]["fatigue"] - 0.7, 0), -res[k]["cr_min"]))
    for k in rank[:15]:
        c = res[k]
        print(k, c["feasible"], round(c["cr_min"], 3), "res", round(c["res_margin"] or 0, 3), c["res_which"],
              "hp", round(c["hp_margin"], 3), c["hp_which"], "fat", round(c["fatigue"], 2), c["x"])


def robust_eval(x: dict, n=8, seed=11) -> dict:
    nom = make(x)
    sets = {"nominal": nom} | {k: V3.perturbed(nom, spec) for k, spec in samples(n, seed).items()}
    return run_sets(sets)


def summarize(res):
    crs = sorted(v["cr_min"] for v in res.values())
    p20 = float(np.percentile(crs, 20))
    allf = all(v["feasible"] for v in res.values())
    allt = all(ok_targets(v) for v in res.values())
    per_t = {t: float(np.percentile([v["cr"][t] for v in res.values()], 20)) for t in res["nominal"]["cr"]}
    return dict(p20=p20, all_feasible=allf, all_targets=allt, n_feasible=sum(v["feasible"] for v in res.values()),
                n_targets=sum(ok_targets(v) for v in res.values()), n=len(res), p20_per_throttle=per_t,
                min_res=min(v["res_margin"] or 0 for v in res.values()), max_fat=max(v["fatigue"] for v in res.values()),
                min_hp=min(v["hp_margin"] for v in res.values()))


def stage_robust(keys: list[str]):
    s = load()["search"]
    out = load().get("robust", {})
    for k in keys:
        x = s[k]["x"] if k in s else json.loads(k)
        r = robust_eval(x)
        out[k] = dict(x=x, summary=summarize(r), samples=r)
        print(k, out[k]["summary"])
        save("robust", out)


# ------------------------------------------------------------------ 4. Floquet + airframe
def _floq(args):
    tag, d, fa, zaf = args
    from flexblade.schema import Airframe
    d = d.model_copy(update=dict(
        analysis=d.analysis.model_copy(update=dict(stability="floquet")),
        op=d.op.model_copy(update=dict(airframe=Airframe(f_hz=fa, f_y_hz=1.15 * fa, zeta=zaf)))))
    r = api.evaluate(d, ["trim", "modes"]).model_dump(mode="json")
    s = r["summary"]
    return tag, fa, zaf, s.get("air_resonance_zeta"), s.get("rpm")


def floq_sweep(d: Design, zafs=(0.02, 0.05)):
    """coarse 50..400 Hz + fine 0.78..1.22 x 0.92 f1 (step 0.02) per throttle"""
    jobs = []
    f1 = {"T0.7": 185.6, "T1": 220.4, "T1.4": 261.6}
    for t, dv in robust.variants(d):
        fs = sorted(set([50, 80, 120, 160, 200, 250, 300, 400] +
                        [round(0.92 * f1[t] * (0.78 + 0.02 * i), 1) for i in range(23)]))
        jobs += [(t, dv, fa, z) for fa in fs for z in zafs]
    with ProcessPoolExecutor(WORKERS) as ex:
        res = list(ex.map(_floq, jobs))
    return [dict(tag=t, f_hz=fa, zeta_af=z, air_zeta=az) for t, fa, z, az, _ in res]


def stage_floq(which):
    out = load().get("floq", {})
    d = {"v3_bare": make({}, None), "v4": Design.model_validate_json(OUT.read_text())}[which]
    out[which] = floq_sweep(d)
    save("floq", out)
    for z in (0.02, 0.05):
        for t in ("T0.7", "T1", "T1.4"):
            pts = [p for p in out[which] if p["tag"] == t and p["zeta_af"] == z and p["air_zeta"] is not None]
            m = min(pts, key=lambda p: p["air_zeta"])
            print(which, z, t, round(m["air_zeta"], 4), m["f_hz"])


# ------------------------------------------------------------------ 5. final tolerance table
def stage_tol(n=20):
    d = Design.model_validate_json(OUT.read_text())
    cs = {"nominal": []} | {k: v for k, v in V3.cases().items() if not k.startswith("random")} | samples(n, seed=7)
    cs["t=0.13mm"] = [("set", "flexure.thickness", 0.00013)]
    cs.pop("t=0.20mm", None)
    t0 = d.flexure.thickness
    for tt in (1.3e-4, 1.5e-4, 2.0e-4):
        if abs(tt - t0) > 1e-6:
            cs[f"t={tt*1e3:.2f}mm"] = [("set", "flexure.thickness", tt)]
    cs.pop("t=0.13mm", None) if abs(t0 - 1.3e-4) < 1e-6 else None
    res = run_sets({k: V3.perturbed(d, v) for k, v in cs.items()})
    save("tol", res)
    print(summarize({k: v for k, v in res.items() if k == "nominal" or k.startswith("random")}))


if __name__ == "__main__":
    st = sys.argv[1]
    if st == "floq":
        stage_floq(sys.argv[2])
    elif st == "tol":
        stage_tol()


if __name__ == "__main__" and sys.argv[1] in ("rku", "damp", "search", "robust"):
    st = sys.argv[1]
    if st == "rku":
        stage_rku()
    elif st == "damp":
        stage_damp()
    elif st == "search":
        stage_search(int(sys.argv[2]) if len(sys.argv) > 2 else 40)
    elif st == "robust":
        stage_robust(sys.argv[2:])
