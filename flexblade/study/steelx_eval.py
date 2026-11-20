"""tt_strap_steelx hand designs: tension-torsion strap + crossed spring-steel shim strips.

Two shim strips (feeler stock, AISI 1095 / 301 FH) form a Haringx cross-spring pivot whose
crossing line is the skewed hinge axis; strip ends are clamped (embedded) in printed hub and
blade fittings; the blade fitting rides a radial slider along the strap line (axial_release),
so one aramid strand through the crossing gap carries the centrifugal (CF) load. The blade
CG is put on the strap line (blade.elastic_axis_chord_frac balanced in a spin-only static
solve) so CF makes no couple on the strips.

Per size: stiffness ratios vs the ideal card, strip load fraction / k_hinge in the loaded
trim state vs unloaded, strip bending-strain fatigue (ε = t/(2ρ), ρ = L/θ for a cross-spring
crossing at mid-length) → max stock thickness for the needed rotation, Euler buckling of a
strip under the worst in-plane transient, control_ratio nominal/robust, binding constraint.

    python study/steelx_eval.py [--cards DIR] [--sizes 5,7,10] [--robust] [--draw]
writes study/designs/<D>in_tt_strap_steelx_hand.json (+ study/results/steelx_<D>in.png).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import run_study as R                                   # noqa: E402
import strap_eval as SE                                 # noqa: E402
from flexblade import fact, materials, motor as mot, rotor   # noqa: E402
from flexblade.beam import assemble_internal, element_end_forces   # noqa: E402
from flexblade.schema import Design                     # noqa: E402

CARDS = Path("/home/fred/vehicles/hakea/flexblade/study/results")
# hand picks: shim stock thickness, strip width / free length, strap strand, crossing angle.
# skew / hinge radius are taken from the ideal card when available.
HAND = {
    5.0: {"flexure.thickness": 1.3e-4, "flexure.width": 0.008, "flexure.length": 0.008,
          "flexure.strap_strand_diameter": 0.0007, "flexure.strap_length": 0.004},
    7.0: {"flexure.thickness": 1.0e-4, "flexure.width": 0.008, "flexure.length": 0.010,
          "flexure.strap_strand_diameter": 0.0009, "flexure.strap_length": 0.004},
    10.0: {"flexure.thickness": 8.0e-5, "flexure.width": 0.008, "flexure.length": 0.010,
           "flexure.strap_strand_diameter": 0.0012, "flexure.strap_length": 0.004},
}
COMMON = {"flexure.cross_angle_deg": 90.0, "flexure.cross_gap": 0.0015, "flexure.taper": 1.0,
          "flexure.strap_strands": 1, "flexure.strap_radius": 0.0,
          "flexure.strap_material": "aramid_yarn", "flexure.strap_anchor": "pin_wrap",
          "flexure.strap_pin_diameter": 0.003, "flexure.axial_release": True,
          "flexure.cruciform_rod_diameter": 0.0}
BUCKLE_SF = 3.0          # required Euler margin under the worst transient


def card_for(D: float, cards: Path = CARDS) -> dict | None:
    p = cards / f"ideal_spec_{int(D)}in.json"
    return json.loads(p.read_text()) if p.exists() else None


def design(D: float, card: dict | None = None, extra: dict | None = None,
           balance: bool = True) -> Design:
    base, _ = R.base_design(D, "tt_strap_steelx")
    vals = {**COMMON, **HAND[D], **(extra or {})}
    if card is not None:
        vals.setdefault("flexure.skew_deg", float(card["x"]["flexure.skew_deg"]))
        vals.setdefault("flexure.hinge_radius", float(card["x"]["flexure.hinge_radius"]))
    vals["flexure.strap_pin_diameter"] = max(0.003, 4 * vals["flexure.strap_strand_diameter"])
    d = SE.apply_strap(base, vals, f"{int(D)}in_tt_strap_steelx_hand", topology="skewed_cross_blade")
    if balance and "blade.elastic_axis_chord_frac" not in (extra or {}):
        d = cg_balance(d)
    return d


# --------------------------------------------------------------------------- load path


def _members(model, k: int = 1):
    bi = model.blades[k - 1]
    strips = {f"flex{j}" for j, m in enumerate(bi.members) if not m.cable and m.kind == "blade"}
    strap = {f"flex{j}" for j, m in enumerate(bi.members) if m.cable}
    return strips, strap


def member_forces(model, st) -> dict:
    """Blade-1 strip / strap end forces (element frame) at state st: strap tension, the sum
    over strips of the end force resultant |(N, V2, V3)| and max |M| (static strip moment)."""
    fe = model.fe
    strips, strap = _members(model)
    sa = assemble_internal(fe, st, False)
    T, Fs, Ms, Nmin = 0.0, {}, 0.0, 0.0
    for e, c in zip(fe.elements, sa.cr):
        if e.kind != "flexure" or e.blade != 1:
            continue
        tag = e.tag.split(".", 1)[-1]
        fa, fb = element_end_forces(e, c)
        if tag in strap:
            T = max(T, abs(fa[0]), abs(fb[0]))
        elif tag in strips:
            for f in (fa, fb):
                Fs[tag] = max(Fs.get(tag, 0.0), float(np.linalg.norm(f[:3])))
                Ms = max(Ms, float(np.linalg.norm(f[4:6])))
                Nmin = min(Nmin, float(f[0]) if f is fb else float(-fa[0]))
    return dict(strap_T=float(T), strip_F=float(sum(Fs.values())), strip_M=Ms,
                strip_frac=float(sum(Fs.values()) / max(T, 1e-9)), strip_Nmin=Nmin)


def _spin_static(d: Design, rpm: float):
    m = rotor.build_model(d)
    ss = rotor.static_trim(m, None, rpm=rpm)
    return m, ss


def cg_balance(d: Design, lo: float = 0.25, hi: float = 0.55, n: int = 14) -> Design:
    """Set blade.elastic_axis_chord_frac so the blade CG lies on the strap line: minimise the
    strip end-force resultant in a spin-only (no aero) static solve at the trim rpm."""
    r = SE.api.evaluate(d, ["trim"])
    rpm = r.trim.rpm

    def f(x):
        dd = d.model_copy(update=dict(blade=d.blade.model_copy(update=dict(elastic_axis_chord_frac=x))))
        m, ss = _spin_static(dd, rpm)
        return member_forces(m, ss.state)["strip_F"]

    g = (math.sqrt(5) - 1) / 2
    a, b = lo, hi
    c1, c2 = b - g * (b - a), a + g * (b - a)
    f1, f2 = f(c1), f(c2)
    for _ in range(n):
        if f1 < f2:
            b, c2, f2 = c2, c1, f1
            c1 = b - g * (b - a)
            f1 = f(c1)
        else:
            a, c1, f1 = c1, c2, f2
            c2 = a + g * (b - a)
            f2 = f(c2)
    x = 0.5 * (a + b)
    return d.model_copy(update=dict(blade=d.blade.model_copy(update=dict(elastic_axis_chord_frac=round(x, 4)))))


def hinge_k(K6, a) -> float:
    C = np.linalg.inv(np.asarray(K6, float))[3:, 3:]
    return float(1.0 / abs(a @ C @ a))


# --------------------------------------------------------------------------- strip checks


def strip_checks(d: Design, r: dict, model, ss) -> dict:
    """Bending-strain fatigue at the 1/rev hinge rotation amplitude and Euler buckling under
    the worst compressive in-plane transient (hover torque + motor 1/rev capability)."""
    f = d.flexure
    mat = materials.get_material(f.layup.material)
    s = r["summary"]
    # hinge rotation amplitude from the pitch / lag amplitudes (pitch = θ_h sinδ, lag = θ_h cosδ)
    th_amp = math.radians(math.hypot(s.get("pitch_amp_deg", 0.0), s.get("lag_amp_deg", 0.0)))
    # mean hinge rotation at trim (blade-1 master rotation about the hinge axis)
    bi = model.blades[0]
    _, rv = rotor.blade_local_motion(model, ss.state, bi, [bi.master])
    _, a = fact.hinge_axis(d)
    th_mean = abs(float(rv[0] @ a))
    L, t, w = f.length, f.thickness, f.width
    eps_amp = t * th_amp / (2 * L)            # cross-spring, crossing at mid-length: ρ = L/θ
    eps_mean = t * th_mean / (2 * L)
    t_max = 2 * L * mat.fatigue_strain_amp_allow / max(th_amp, 1e-9)
    # Goodman on stress: σa/σf + σm/Xt ≤ 1
    good = (mat.E1 * eps_amp) / (mat.fatigue_strain_amp_allow * mat.E1) + mat.E1 * eps_mean / mat.Xt
    stock = [x for x in materials.SHIM_THICKNESSES.get(f.layup.material.split("@")[0], ())
             if x <= t_max]
    # buckling: in-plane (drag-direction) blade-root force goes into the strips along their
    # length ±γ/2 from x'; the radial slider frees x, so a strip sees ≈ F_y / (2 sin γ/2).
    cap = mot.capability(d.motor, r["trim"]["rpm"], r["trim"]["torque"])
    Q = r["trim"]["torque"] + cap.dQ_max_1p
    r75 = 0.75 * d.blade.radius
    Fy = Q / d.blade.n_blades / r75
    g = math.radians(f.cross_angle_deg) / 2
    P = Fy / (2 * math.sin(g))
    EI = mat.E1 * w * t ** 3 / 12
    Pcr = 4 * math.pi ** 2 * EI / L ** 2      # clamped-clamped strip
    Nmin = member_forces(model, ss.state)["strip_Nmin"]     # FE trim compression (< 0)
    P = max(P, -Nmin)
    return dict(theta_amp_deg=math.degrees(th_amp), theta_mean_deg=math.degrees(th_mean),
                eps_amp=eps_amp, eps_mean=eps_mean, fatigue_util=eps_amp / mat.fatigue_strain_amp_allow,
                goodman=good, t_max_mm=t_max * 1e3, stock_ok_mm=[x * 1e3 for x in stock],
                buckle_P=P, buckle_Pcr=Pcr, buckle_util=BUCKLE_SF * P / Pcr, fe_Nmin=Nmin)


def loaded_state(d: Design) -> dict:
    """Strip load fraction and k_hinge: loaded trim vs unloaded (initial state)."""
    model = rotor.build_model(d)
    aero = rotor.make_aero(model)
    ss = rotor.static_trim(model, aero)
    _, a = fact.hinge_axis(d)
    k_ld = hinge_k(rotor.flexure_stiffness(model, ss), a)
    k_un = hinge_k(rotor.flexure_stiffness(model, None), a)
    mf = member_forces(model, ss.state)
    # CF-only share: spin-only (no aero) solve at the trim rpm
    sp = member_forces(model, rotor.static_trim(model, None, rpm=ss.rpm).state)
    return dict(model=model, ss=ss, k_hinge_loaded=k_ld, k_hinge_unloaded=k_un,
                K6_unloaded=rotor.flexure_stiffness(model, None), cf_strip_frac=sp["strip_frac"], **mf)


def evaluate(D: float, cards: Path = CARDS, robust: bool = False, extra: dict | None = None) -> tuple[Design, dict]:
    card = card_for(D, cards)
    d = design(D, card, extra)
    out = SE.evaluate(d, card, robust)
    r = SE.api.evaluate(d).model_dump(mode="json")
    ls = loaded_state(d)
    out["ea_frac"] = d.blade.elastic_axis_chord_frac
    out["trim_split_deg"] = r["summary"].get("trim_diff_pitch_deg")
    out["load"] = {k: v for k, v in ls.items() if k not in ("model", "ss", "K6_unloaded")}
    if card is not None:
        # unloaded (material-only) ratios alongside the loaded ones (out["ratios"] from the
        # trim-state flexure_stiffness). The loaded K6 used to carry a spurious T·d pendulum
        # (d = master − hinge point, ≈ 5.5 mm here) from a preload-free transport; fixed in
        # rotor.condense_to_point, so loaded is now like-for-like with the ideal K6.
        du = SE.IS.distance(ls["K6_unloaded"], card)
        t = card["targets"]
        out["ratios_unloaded"] = {k: 10 ** v * t[k] for k, v in du["terms"].items()
                                  if k in ("k_hinge", "k_cross_over_k_hinge", "k_flap_over_k_hinge")}
    out["strip"] = strip_checks(d, r, ls["model"], ls["ss"])
    out["util"] = {f'{c["member"]}:{c["name"]}': round(c["utilisation"], 3) for c in r["checks"]}
    return d, out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--cards", default=str(CARDS))
    ap.add_argument("--sizes", default="5,7,10")
    ap.add_argument("--robust", action="store_true")
    ap.add_argument("--draw", action="store_true")
    ap.add_argument("--set", default="", help="json dict of extra design values")
    a = ap.parse_args(argv)
    extra = json.loads(a.set) if a.set else None
    for D in (float(x) for x in a.sizes.split(",")):
        d, res = evaluate(D, Path(a.cards), a.robust, extra)
        p = HERE / "designs" / f"{d.name}.json"
        p.write_text(d.model_dump_json(indent=1))
        if a.draw:
            from flexblade import drawing
            drawing.sheet(d, str(HERE / "results" / f"steelx_{int(D)}in.png"))
        print(json.dumps({"D": D, **res}, default=float, indent=1))


if __name__ == "__main__":
    main()
