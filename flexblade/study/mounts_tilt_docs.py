"""Writes the mount section + I_t bench section into the two BUILD.md files and study/results/mounts_tilt.md.
Run from flexblade/ with PYTHONPATH=$PWD:$PWD/study:$PWD/study/build after mounts_tilt.py."""
import json, re
from pathlib import Path
import numpy as np
import mount_twin as MT
import mounts_tilt as MTL

R = Path("study/results")
CFG = {  # key: (build dir, f1p max Hz, J_p itemised, label, 1P min Hz, motor mass)
    "7in_v6": ("study/build/7in_v6", 260.4, 6.986e-6 + 0.65e-6, "7in v6b", 182.6),
    "10in_v5": ("study/build/10in_v5", 151.5, 2.188e-5 + 3.59e-6, "10in v5c", 107.6),
}
res = {k: json.loads((R / f"mounts_tilt_{k}.json").read_text()) for k in CFG}
md = ["# Twin-washer motor mounts sized for hub-tilt stability (7in v6b, 10in v5c)", "",
      "Floquet with hub tilt + gyroscopics (stability.Mount.tilt), T {0.7,1,1.4} x J_hub {2.5,3.76,5.49}e-6 x arm {190,225,260} Hz "
      "(f_y = 1.15 f_x), airframe zeta 0.02. Script: study/mounts_tilt.py (raw: results/mounts_tilt_*.json), docs: study/mounts_tilt_docs.py. "
      "Rule: k_t >= margin x Omega_max^2 (I_t - J_p), Omega_max = 1P at T1.4; design margin 2.5x evaluated at 2x the estimated motor "
      "I_t (= 0.5 J_p + m (15 mm)^2).", ""]
for k, (bd, f1max, Jp, lab, f1min) in CFG.items():
    P, m = MTL.SIZES[k]
    pr = MT.props(**P)
    r = res[k]
    Om = 2 * np.pi * f1max
    It0 = 0.5 * Jp + m * 0.015 ** 2
    need = 2.5 * MT.rule(Om, 2 * It0, Jp)
    z = {f: min(x[f] for x in r) for f in r[0] if f.startswith("z_")}
    z1 = min(v for f, v in z.items() if f.endswith("It1"))
    z2 = min(v for f, v in z.items() if f.endswith("It2"))
    z3 = min(v for f, v in z.items() if f.endswith("It3"))
    hubs = [h for x in r for f, h in x.items() if f.startswith("hub_")]
    f_lo, f_hi = min(h[0] for h in hubs), max(h[0] for h in hubs)
    zh = (min(h[1] for h in hubs), max(h[1] for h in hubs))
    rows = ["| measured motor+hub I_t (kg m^2) | min k_t at 2x margin (N m/rad) | at 2.5x |", "|---|---|---|"]
    for ix in (0.5, 1, 2, 3, 4):
        rr = MT.rule(Om, ix * It0, Jp)
        rows.append(f"| {ix * It0:.2e} ({ix:g}x est.) | {2 * rr:.0f} | {2.5 * rr:.0f} |")
    sec = f"""## Damped isotropic motor mount, twin washers, tilt-stiff (air-resonance and tilt-whirl margin)
**Rule: never tilt-soft-mount a 2-blade rotor.** A single soft washer gives rotating stiffness about the blade axis k_t + Omega^2 (J_p - I_t) that goes negative when the motor+hub transverse inertia I_t exceeds the polar J_p: synchronous 1/rev tilt divergence (the 7in v6 single washer was unstable, zeta -0.09...-0.14). Decouple: translation soft (hub mode <= 0.7 x lowest 1P), tilt stiff (k_t >= 2x Omega_max^2 (I_t - J_p), designed 2.5x at 2x the estimated I_t).

Parts (make_mount.py -> ../mount_twin.py, watertight STLs in this folder, parameters in mount_parts.json): `mount_tube_clamp.stl` (split clamp for a 10 mm tube, bore 10.1, M3 pinch bolt, M4 tap), 2x `mount_tpu_washer.stl` (TPU 85A, ri {P['ri']} / ro {P['ro']} / t {P['t']} mm, 40 % gyroid), `mount_carrier.stl` (plate + skirt holding both washers, motor holes M3 on 16x16 and 19x19), `mount_post.stl` (d 7.8 rod with two d {2 * P['ro']:g} x 2 flanges on the washer inner faces, M4 at the clamp end), `mount_cap.stl` (retainer ring, M2 x3). Washer centres {P['d']} mm apart. Preload the M4 so BOTH washers carry thrust (then the pair is a stiff tilt joint: tilt k = 2 Ec pi (ro^2-ri^2)/t (d/2)^2 + local term).

Sized (study/mounts_tilt.py): k_lat {pr['k_lat'] / 1e3:.1f} kN/m, k_t {pr['k_t']:.0f} N m/rad (rule x2.5 at 2x I_t needs {need:.0f}), axial k {pr['k_ax'] / 1e3:.0f} kN/m (sag {1e3 * 10 / pr['k_ax']:.2f} mm per 10 N). Mass on the mount assumed {m * 1e3:.0f} g. Lateral hub mode {f_lo:.0f}-{f_hi:.0f} Hz ({f_lo / f1min:.2f}-{f_hi / f1min:.2f} x lowest 1P {f1min:.0f} Hz at T0.7), zeta_af {zh[0]:.3f}-{zh[1]:.3f}. Floquet min zeta over the T x J x arm scan: {z1:.4f} (estimated I_t), {z2:.4f} (2x I_t), {z3:.4f} (3x I_t); all positive. Details: study/results/mounts_tilt.md. TPU properties (G, eta) must be verified on a printed sample (tap test: the motor bell should ring at the hub-mode frequency, x = y within 5 %).

## Bench: transverse inertia I_t of motor + carrier + hub (bifilar about a diameter)
Measure next to J_p. Hang the motor + hub + carrier (no blades) bifilar from two threads (length l ~ 0.5 m, separation 2a ~ 60 mm) with the rotor axis HORIZONTAL (a bar through the carrier holds it) so the oscillation is a twist about a diameter through the centre of mass (use the cup axis point where the washers sit as the tilt centre; add m h^2 for the offset h of the centre of mass from the tilt centre by the parallel-axis theorem): I = m g a^2 T^2 / (4 pi^2 l). Compare with the estimate I_t_est = {It0:.2e} kg m^2 (0.5 J_p + m (15 mm)^2, J_p = {Jp:.2e}). Then look up the minimum k_t (rule k_t >= margin x Omega_max^2 (I_t - J_p), Omega_max = {Om:.0f} rad/s = 1P at T1.4, J_p = {Jp:.2e}); the built mount has k_t = {pr['k_t']:.0f} N m/rad:

""" + "\n".join(rows) + "\n\nIf the measured I_t needs more than the built k_t at 2x margin, lengthen the washer spacing (k_t ~ d^2) before flying. Measure J_p the same way about the axis (existing bifilar section).\n"
    p = Path(bd) / "BUILD.md"
    t = p.read_text()
    i = t.index("## Damped isotropic motor mount")
    j = t.index("\n## ", i + 5)
    p.write_text(t[:i] + sec + t[j:])
    md += [f"## {lab}", f"- washers ri {P['ri']} / ro {P['ro']} / t {P['t']} mm x2, centres {P['d']} mm; mass on mount {m * 1e3:.0f} g",
           f"- k_lat {pr['k_lat'] / 1e3:.1f} kN/m, k_t {pr['k_t']:.0f} N m/rad, axial {pr['k_ax'] / 1e3:.0f} kN/m",
           f"- rule: Omega_max {Om:.0f} rad/s, J_p {Jp:.2e}, est. I_t {It0:.2e}; need 2.5 x rule(2 I_t) = {need:.0f}; margin built/need = {pr['k_t'] / need:.2f}; "
           f"margin vs rule at estimated I_t = {pr['k_t'] / MT.rule(Om, It0, Jp):.1f}x, at 2x I_t = {pr['k_t'] / MT.rule(Om, 2 * It0, Jp):.1f}x",
           f"- hub mode {f_lo:.1f}-{f_hi:.1f} Hz = {f_lo / f1min:.2f}-{f_hi / f1min:.2f} x lowest 1P ({f1min:.0f} Hz), zeta {zh[0]:.3f}-{zh[1]:.3f}",
           f"- Floquet min zeta (9 T x J x 3 arms): I_t x1 {z1:.4f}, x2 {z2:.4f}, x3 {z3:.4f}", "", "Min zeta per arm and I_t:", "",
           "| arm Hz | x1 | x2 | x3 |", "|---|---|---|---|"]
    for fa in (190, 225, 260):
        md.append(f"| {fa} | " + " | ".join(f"{z[f'z_{fa}_It{ix}']:.4f}" for ix in (1, 2, 3)) + " |")
    md += ["", "Min k_t vs measured motor I_t:", ""] + rows + [""]
(R / "mounts_tilt.md").write_text("\n".join(md) + "\n")
print("\n".join(md))
