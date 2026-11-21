"""Build the printable parts for 7in_tt_strap_steelx_v5 (run from flexblade/ with PYTHONPATH=$PWD).

Parts come from flexblade.fittings (parametric clamp fittings built by manifold3d CSG from the
flexure parameters: clamp lands + slots, radiused jaw exits, free span = modelled strip length,
strand channel + knot well, >= 1.2 mm walls). Outputs watertight STL (mm, hub frame, z up =
build direction = rotor axis) plus drawing.png and parts.json into study/build/7in_v5/.
"""
import json
from pathlib import Path

import numpy as np
import trimesh

from flexblade.schema import Design
from flexblade import fittings as FT
from flexblade.materials import get_material
from flexblade.geometry import HUB_MATERIAL

OUT = Path("study/build/7in_v5"); OUT.mkdir(parents=True, exist_ok=True)
D = Design.model_validate(json.load(open("study/designs/7in_tt_strap_steelx_v5.json")))
parts = FT.strip_print_parts(D)
fits = parts.pop("_fits")
prm = fits.params
info = {}

for name, fn in (("hub", "hub_fitting_assembly"), ("blade1", "blade1_with_root_fitting"),
                 ("blade2", "blade2_with_root_fitting")):
    m, kept, dropped = FT.to_trimesh(parts[name])
    m.export(OUT / f"{fn}.stl")
    m2 = trimesh.load(OUT / f"{fn}.stl")
    info[fn] = dict(watertight=bool(m2.is_watertight), components=kept, fragments_dropped=dropped,
                    volume_mm3=round(float(m2.volume), 1), bbox_mm=np.round(m2.extents, 1).tolist())
    print(fn, info[fn])
for p in OUT.glob("blade_with_root_fitting.stl"):
    p.unlink()                                   # old single-blade file (blade 2 is mirrored)

rho_hub = get_material(HUB_MATERIAL).rho
rho_bl = get_material(D.blade.layup.material).rho
# printed fitting masses (fitting solid only: jaws + web + post, cuts applied)
m_hub = fits.hub.volume() * rho_hub * 1e-6
m_root = fits.blade.volume() * rho_bl * 1e-6
# model: blade-side hardware inboard of the aero root is the shank beam element(s)
from flexblade import rotor
M = rotor.build_model(D)
m_model = sum(e.sections[0].mass_per_len * e.L0 for e in M.fe.elements
              if e.blade == 1 and e.kind == "shank") * 1e3
lag, pit = FT.motion_range(D)
clr = FT.fitting_clearance(D, lag, pit, fits=fits)
rest = float(fits.blade.min_gap(fits.hub, 20.0))
info.update(params=vars(prm), notes=fits.notes, clearance=clr, rest_gap_mm=rest,
            mass_g=dict(hub_fitting_per_blade=round(m_hub, 2), root_fitting=round(m_root, 2),
                        model_shank=round(m_model, 2), hub_material=HUB_MATERIAL,
                        blade_material=D.blade.layup.material))
print(json.dumps({k: info[k] for k in ("clearance", "rest_gap_mm", "mass_g")}, indent=1))

# ---- drawing: three projections of the blade-1 fittings + strips (rest pose)
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from flexblade.fittings import strip_members, _frame
strips, wires, _ = strip_members(D)
fig, axs = plt.subplots(1, 3, figsize=(16, 6))
views = (("top (x-y)", 0, 1), ("front (y-z)", 1, 2), ("side (x-z)", 0, 2))
for ax, (title, i, j) in zip(axs, views):
    for mm, col in ((fits.hub, "tab:blue"), (fits.blade, "tab:orange")):
        tm, _, _ = FT.to_trimesh(mm)
        ax.triplot(tm.vertices[:, i], tm.vertices[:, j], tm.faces, color=col, lw=0.15, alpha=0.5)
    for m in strips:
        p0, p1, L, t, w, n = _frame(m)
        c = np.array([p0 - w * prm.w / 2, p1 - w * prm.w / 2, p1 + w * prm.w / 2, p0 + w * prm.w / 2, p0 - w * prm.w / 2])
        ax.plot(c[:, i], c[:, j], "k-", lw=1.2)
    for m in wires:
        a0, a1 = np.array(m.p0) * 1e3, np.array(m.p1) * 1e3
        ax.plot([a0[i], a1[i]], [a0[j], a1[j]], "g-", lw=2.5)
    ax.set_title(title); ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("xyz"[i] + " mm"); ax.set_ylabel("xyz"[j] + " mm")
fig.suptitle(f"7in_tt_strap_steelx_v5 blade-1 fittings: hub (blue), root (orange), strips (black, "
             f"free {prm.L:.2f} mm), strand (green). Land {prm.land:.1f}, slot {prm.slot_t:.2f} x "
             f"{prm.slot_w:.1f}, jaw R {prm.jaw_r:.2f}; min gap {clr['min_gap_mm']:.2f} mm over lag "
             f"+-{lag:g} / pitch +-{pit:g} deg", fontsize=10)
fig.tight_layout(); fig.savefig(OUT / "drawing.png", dpi=110)


# ---- fitting mass / inertia: fittings.py model vs trimesh of the exported geometry
fmp = FT.fitting_mass_props(D)
chk = {}
meshes = {"hub": FT.to_trimesh(FT._sub(fits.hub, [fits.keepout]))[0],
          "root": FT.to_trimesh(FT._sub(fits.blade, [fits.keepout]))[0]}
for key, mat in (("hub", HUB_MATERIAL), ("root", D.blade.layup.material)):
    mm = meshes[key]; mm.density = get_material(mat).rho * 1e-9      # kg/mm^3
    mp = mm.mass_properties
    I_t = np.asarray(mp.inertia) * 1e-6                               # kg m^2 about centroid
    m_f, c_f, I_f = fmp[key]
    chk[key] = dict(mass_g_fittings=round(m_f * 1e3, 3), mass_g_trimesh=round(float(mp.mass) * 1e3, 3),
                    Ixx_Iyy_Izz_fittings=np.diag(I_f).tolist(), Ixx_Iyy_Izz_trimesh=np.diag(I_t).tolist(),
                    com_mm_fittings=(c_f * 1e3).tolist(), com_mm_trimesh=np.asarray(mp.center_mass).tolist())
info["mass_check"] = chk
print(json.dumps(chk, indent=1, default=float))

# ---- predicted modes (hover rpm, nominal T1, and at rest 0 rpm with aero off)
from flexblade import robust
def mode_tables():
    res = {}
    for tag, d in robust.variants(D):
        if tag not in ("T0.7", "T1", "T1.4"):
            continue
        with rotor.part_temperature(d.op.ambient_C):
            inf = {}
            d = rotor.resolve_pre_lag(d, info=inf)
            m = rotor.build_model(d); a = rotor.make_aero(m); ss = rotor.static_trim(m, a)
            md = rotor.modes(m, ss, a, lin=rotor.linearize(m, ss, d.analysis.coriolis))
            hov = [(float(f), l) for f, l in zip(md.freqs_hz, md.labels) if f > 5]
            d0 = d.model_copy(update=dict(analysis=d.analysis.model_copy(update=dict(aero="none"))))
            m0 = rotor.build_model(d0); s0 = rotor.static_trim(m0, None, rpm=0.01)
            md0 = rotor.modes(m0, s0, None, lin=rotor.linearize(m0, s0, d.analysis.coriolis))
            rest = [(float(f), l) for f, l in zip(md0.freqs_hz, md0.labels) if f > 5]
            res[tag] = dict(rpm=float(ss.rpm), pre_lag=d.flexure.pre_lag_deg, hover=hov, rest=rest)
    return res
MT = mode_tables()
info["modes"] = MT
(OUT / "parts.json").write_text(json.dumps(info, indent=1, default=float))

hubstl = trimesh.load(OUT / "hub_fitting_assembly.stl")
info["hub_stl_total_g"] = round(float(hubstl.volume) * rho_hub * 1e-6, 2)
print("hub assembly STL total g", info["hub_stl_total_g"])


# ---- BUILD.md (numbers from the design / fittings / model)
f = D.flexure
Lfree, land, wst = prm.L, prm.land, prm.w
cut_len = Lfree + 2 * land
pre = MT["T1"]["pre_lag"]
def mrow(tag, key):
    return MT[tag][key]
def table(key):
    rows = ["| Mode | " + " | ".join(f"{t} ({MT[t]['rpm']:.0f} rpm)" if key == "hover" else t for t in ("T0.7", "T1", "T1.4")) + " |",
            "|---|---|---|---|"]
    names = sorted({(l) for t in MT for _, l in MT[t][key]})
    for lab in ("pitch sym", "pitch antisym", "pitch antisym+hub", "flap sym", "flap antisym"):
        cols = []
        for t in ("T0.7", "T1", "T1.4"):
            fr = [f"{fq:.0f}" for fq, l in MT[t][key] if l == lab]
            cols.append(", ".join(fr) or "-")
        rows.append(f"| {lab} | " + " | ".join(cols) + " |")
    return "\n".join(rows)
md = f"""# BUILD SHEET: 7in_tt_strap_steelx_v5 (2-blade, crossed 1095 strips, aramid strand)

Files (this folder): `hub_fitting_assembly.stl` (print 1x), `blade1_with_root_fitting.stl` and `blade2_with_root_fitting.stl` (print 1x each; blade 2's flexure and root fitting are MIRRORED in z, its airfoil is not, so it is a different part), `drawing.png`, `parts.json`; generator `../build_7in_v5.py` (parts from `flexblade/fittings.py`). All STLs watertight, one shell each (parts.json).
Coordinates mm, hub frame, z up = build direction = rotor axis. Strips crossed at **{f.cross_angle_deg:.2f} deg**, skew {f.skew_deg:.2f} deg.

## Cut list (bought parts)
| Item | Spec | Qty |
|---|---|---|
| Spring-steel shim strip, 1095, 0.15 mm | **{wst:.2f} wide x {cut_len:.1f} mm long** = {Lfree:.2f} free + 2 x {land:.1f} clamp lands; cut 6 (4 + 2 spare) | 4 |
| Constrained-layer damper (the design assumes it, eta 0.0298): 3M VHB 9469 0.13 mm + 0.05 mm shim on each strip free length, ending 1 mm short of the jaw exits | {wst:.2f} x {Lfree - 2:.1f} mm | 4 |
| Aramid strand 1.5 mm | **40 mm** per blade: 4 free + 2 x 8 channel + 2 x 8 knot/tail; one continuous strand hub side to blade side | 2 |
| Glue | Strips: structural epoxy, thin film. CA only to tack. Knot wells: thin CA then epoxy seal | |
| Balance | Pin-balance the blade pair; tip mass difference < 0.05 g | |

Strip prep: tin snips, strip LONG axis parallel to the rolling (grain) direction. Cut ~0.3 mm wide, file to {wst:.2f} (the strip is narrower than v4: measure with calipers, tolerance +-0.05 mm; width is the most sensitive dimension of the flexure stiffness, k_hinge ~ w). Deburr all four edges (400 grit along the length, break ~0.1 mm). Degrease with IPA, scuff only the glued lands.

## Slots and radii (in the STLs)
- Free span {Lfree:.2f} mm between hub and root jaw exits; clamp land {land:.1f} mm per strip end; slot {prm.slot_t:.2f} mm x {prm.slot_w:.1f} mm; jaw exits radiused R = {prm.jaw_r:.2f} mm.
- Cross gap {f.cross_gap*1e3:.1f} mm between the two strip planes: hold this with a {f.cross_gap*1e3:.1f} mm shim while the epoxy cures.
- Clearance (rigid sweep, lag +-{lag:g} deg x pitch +-{pit:g} deg): rest gap {rest:.2f} mm, minimum {clr['min_gap_mm']:.2f} mm.
- Printed fitting mass (PA6-CF, per blade, in the rotor model): hub fitting {m_hub:.2f} g, root fitting {m_root:.2f} g (fitting_slim webs). Hub assembly STL total {info['hub_stl_total_g']} g.

## Print settings
PA6-CF dried 6+ h at 80 C; z up as exported; 0.4 nozzle, 0.12 layers, 6 perimeters, 100 % infill, no supports. The slot is ~1.7 layers: check with a spare strip, scrape with a fresh blade, do not drill.

## Assembly
1. Dry test-fit deburred strips in both slots.
2. Per blade, push each strip from behind through the hub jaw, across the free span, into the blind root slot, strips crossing at {f.cross_angle_deg:.2f} deg (drawing.png).
3. Thin epoxy on the lands, hold the {f.cross_gap*1e3:.1f} mm shim between the strips until cured, remove it.
4. Fit damper layers 1 mm short of the lands.
5. Thread the strand, stopper knots, strand just taut at rest, CA then epoxy seal.
6. Repeat for blade 2; balance the blade pair, then the assembled rotor.

## Pitch preset (static, per blade)
`pre_lag auto` on the pitch axis resolves (resolve_pre_lag, T1 hover) to **{pre:+.3f} deg per blade**, i.e. essentially zero: no deliberate preset is needed in v5 (v4 needed +2.86 deg). Build both blades at 0 +- 0.3 deg: phone inclinometer on the chord at 0.5R vs the hub plane, blades matched to +-0.3 deg. If one blade is off by more than that, re-seat the root jaw before gluing, do not bend the strips.

## Predicted modes for the pluck / tap test (Hz; model, nominal strips, damper, real fitting mass)
At rest (0 rpm, aero off; hub treated as the model hub, tap with the arm clamped and the hub as stiff as you can make it):

{table('rest')}

At hover rpm (aero on, trim rpm per thrust level T = 0.7 / 1.0 / 1.4 x weight, centrifugal stiffening included; only observable by an accelerometer or a mic during a spin-up, or by the throttle sweep below):

{table('hover')}

Notes for the bench test: the 'pitch' modes are the flexure hinge / torsion modes of the crossed strips (the skewed hinge couples pitch and lag, so there is no separate low lag mode: the hinge mode IS the 'pitch sym' line). 'pitch antisym+hub' is the differential mode that also moves the hub in-plane, so it depends on the arm mount stiffness. Flap modes at rest (~250 Hz) are the strip bending on the free span plus blade root. Tolerance: model values assume 0.15 mm / {wst:.2f} mm strips; more than 15 % off in the pitch-sym line (~35 Hz) means a loose clamp or wrong width. Frequency scales as sqrt(k): width +5 % = +2.4 % frequency. Pluck the blade tip chordwise (excites pitch) and flapwise (excites flap), record 5 s with a phone spectrum app, read the lowest peak.

## Airframe isotropy requirement
Make the motor-arm mount in-plane stiffness ISOTROPIC (x ~ y within 5 %) or soft-damped. A 2-blade rotor on an anisotropic support has a hub-dominated 2/rev parametric air-resonance window that strip damping cannot cure. Tap-test the arm in x and y (tones within ~5 %); fly with a rubber/TPU damped mount if not. The v5 Floquet check (verify_7in_v5.md) is with f_y = 1.15 f_x as a worst realistic anisotropy; the airframe damping assumed is 0.02 minimum.

## What to measure to calibrate the nP aero forcing (blade-root accelerometer, slow throttle sweep)
The forced-response criterion (forced.py) assumes torque harmonics 3 % at 2P, 1 % at 3P, aero nP loads ~5-10 % of mean lift moment at 2P falling ~1/n^2. These are assumptions; measure:
1. Mount a 3-axis MEMS accelerometer (>= 4 kHz sample rate, e.g. ADXL372 or a logging IMU in the blade-root fitting, or on the arm near the hub as a fall-back) on one blade root; log with a telemetry-free logger on the rotor or a slip-ring.
2. Slow throttle sweep: ramp 40 % to 100 % of hover throttle over >= 60 s with the vehicle restrained (thrust stand or tethered), so every nP line crosses every mode slowly (rpm range ~ 11000-15700). Log rpm from ESC telemetry (or the 1P line itself) and motor current.
3. Spectrogram vs rpm: read the 1P, 2P, 3P, 4P amplitudes, and the amplification peaks where nP meets a mode of the table above (predicted: e.g. 2P at {2*MT['T1']['rpm']/60:.0f} Hz at T1 rpm against the pitch antisym+hub line at {[fq for fq, l in MT['T1']['hover'] if l == 'pitch antisym+hub'][0]:.0f} Hz).
4. Per crossing record amplitude A_pk (g) and the half-power width of the resonance peak, from which zeta = df/(2 f) (compare with the model zeta). Ratio of the nP acceleration far from any crossing to the 1P acceleration gives the forcing fraction to compare with the 3 % / 1 % assumptions.
5. Repeat with the motor current harmonic spectrum logged (shunt + scope/ADC) to separate the electrical (torque) nP source from aero nP: if the root acceleration at nP follows the current nP amplitude it is torque; if it follows thrust / tilt it is aero. Then test the aero part by tilting the rotor disc (a few degrees) or running near a wall and see the 2P scale.
6. Pitch jitter: if possible a hall/optical pitch pickup on one blade; the criterion limit is pitch jitter in degrees, accelerometer double-integration is not reliable.

## Approximations and limits
- Strap anchor `pin_wrap` replaced by a stopper-knot well; knot strength not modelled. Clearance is a rigid-body sweep; strip S-bending not included.
- ccx cross-check of the strip pair (study/verify_v5.py ccx): all terms within 6 % of the beam model.
"""
(OUT / "BUILD.md").write_text(md)
print("BUILD.md written", len(md))
