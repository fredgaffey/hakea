"""Post-process study/build/10in_v5/BUILD.md written by build_10in_v5.py: fix the lamina cut list and the
pitch-preset text, append the 10in sections (mount, hub J, bifilar, pluck, pulse schedule).
Run from flexblade/ with PYTHONPATH=$PWD after build_10in_v5.py and verify_10in_v5.py pl."""
import json
from pathlib import Path

B = Path("study/build/10in_v5")
md = (B / "BUILD.md").read_text()
R = json.loads(Path("study/results/verify_10in_tt_strap_steelx_v5.json").read_text())
info = json.loads((B / "parts.json").read_text())
pre = info["modes"]["T1"]["pre_lag"]

# --- cut list: 8 x 0.10 mm laminae per strip
md = md.replace(
    "| Spring-steel shim strip, 1095, 0.15 mm | **9.00 wide x 49.1 mm long** = 25.12 free + 2 x 12.0 clamp lands; cut 6 (4 + 2 spare) | 4 |",
    "| Spring-steel shim, 1095, **0.10 mm** (0.004\"), laminae of one strip | **9.00 wide x 49.1 mm long** = 25.12 free + 2 x 12.0 clamp lands. 4 strips x **8 laminae = 32 laminae**; cut 40 (8 spare) | 32 |")
md = md.replace("Strip prep: tin snips,", "Strip prep (per lamina): tin snips,")
md = md.replace("(the strip is narrower than v4: measure", "(v5 strip: 9 mm wide, 25.12 mm free; measure")
md = md.replace("0.15 mm / 9.00 mm strips", "8 x 0.10 mm / 9.00 mm laminae")
md = md.replace("the model strips", "the model strips")
md += """
## Lamina stack (8 x 0.10 mm per strip, UNBONDED in the free span)
The model carries 8 independent laminae (stiffness 8 x one lamina, ccx per-lamina x 8 within 6 %), stacked 0.80 mm in a 0.85 mm slot (0.05 mm allowance: tight, test-fit a spare stack and a spare slot first). Stack the 8 laminae with a thin film of light oil (3-in-1 / sewing-machine oil) between them over the FREE span only; epoxy ONLY on the 12 mm clamp lands (wet the whole stack in the land, clamp flat, wipe oil off the land first). Keep oil off the lands. Do not bond the free span: a bonded stack is 64x stiffer in bending than 8 sliding laminae and will fail the fatigue/pitch design. Damper layer (VHB 0.13 mm + 0.05 mm shim) goes over the stack, 1 mm short of the jaw exits. Cut all laminae from the same sheet, same rolling direction, matched widths +-0.05 mm (a width spread of 5 % is inside the 8/9 tolerance run; width is the most sensitive dimension).
"""
# --- pitch preset: it is not zero at 10"
i = md.index("## Pitch preset")
j = md.index("## Predicted modes")
md = md[:i] + f"""## Pitch preset (static, per blade)
`pre_lag auto` on the pitch axis resolves (resolve_pre_lag, T1 hover) to **{pre:+.3f} deg per blade** (v4 needed +2.86 deg). Build both blades at this value +- 0.3 deg: phone inclinometer on the chord at 0.5R vs the hub plane, blades matched to +-0.3 deg. If one blade is off by more than that, re-seat the root jaw before gluing, do not bend the strips.

""" + md[j:]
md = md.replace("(~35 Hz)", "(~19 Hz at rest)").replace("Flap modes at rest (~250 Hz)", "Flap modes at rest (~190 Hz)")
md = md.replace("more than 15 % off in the pitch-sym line (~19 Hz at rest)", "more than 15 % off in the pitch-sym line (~19 Hz at rest, 62-88 Hz at hover)")
md = md.replace("(rpm range ~ 11000-15700)", "(rpm range ~ 6500-9100)")
md = md.replace("The v5 Floquet check (verify_10in_v4.md)", "The v5 Floquet check (verify_10in_v5.md)")
md = md.replace("- ccx cross-check of the strip pair (study/verify_v5.py ccx): all terms within 6 % of the beam model.",
                "- ccx cross-check of the strip pair (verify_10in_v5.py ccx): per-lamina ccx x 8 within 6 % of the 8-lamina beam model; lamina slip/contact not modelled in ccx.")

# --- mount, J, bifilar, pluck, pulse schedule
Jt = R["J"]
rows = ["| throttle (thrust x weight) | J_hub | pulse, x 0.95 capability | pulse, x mod_frac_max | control ratio at that pulse | cr_pl | binding | 1P resonance margin (need 0.15) |", "|---|---|---|---|---|---|---|---|"]
for p in R["pl"]["summary"]["points"]:
    t, J = p["tag"].split(":J")
    rows.append(f"| {t[1:]} | {float(J):.2e} | {p['pulse_scale']:.3f} | {p['pulse_frac_cap']:.3f} | {p['control_ratio']:.4f} | {p['cr_pl']:.4f} | {p['pulse_binding']} | {p['res_margin']:.3f} |")
md += f"""
## Damped isotropic motor mount (air-resonance margin)
Parts (make_mount.py, STLs in this folder, all watertight): `mount_tube_clamp.stl` (split clamp for a 10 mm tube, bore 10.1, M3 pinch bolt, 22x24x20 block, post d 7.8 with 18 mm flange, M4 tap), `mount_tpu_washer.stl` (TPU 85A, ri 4 / ro 9 / t 10 mm, 40 % gyroid so the in-plane shear stiffness is 53 kN/m; print a trial and trim), `mount_carrier.stl` (30x30x4 plate, motor holes M3 on 16x16 and 19x19, central d 9 hole, collar lip d 23; the 3115 motor pattern is 19x19). Stack: clamp flange / washer / carrier; M4 screw with large washer 1 mm above the carrier is the bump stop, not preloaded. Washer axis = motor axis so k_x = k_y by construction. The 10 mm tube is the 7in clamp: check the 1.7 kg vehicle's arm tube; for a bigger tube scale the clamp block only, not the washer.
10in sizing (study/mount_calc_10in.py): rotating mass on the mount 124 g (motor ~90 + carrier 12 + hub 17.7 + screws), arm modal mass 0.25 x 1.697 kg = 0.424 kg, k_m = 53 kN/m, eta 0.15 (TPU tan-delta 0.1-0.2, VERIFY on a printed sample). Hub mode **99-102 Hz at zeta_af 0.068-0.072** (arm mode 238-317 Hz at 0.023-0.027 for f_arm 190-260). Tap test: motor bell on the carrier rings ~100 Hz, x and y equal within 5 %.
Caveat: the 10in 1P is 108 Hz (T0.7, 6456 rpm), 128 Hz (T1), 152 Hz (T1.4), so the mount hub mode sits just under 1P at low throttle. Hub 1P amplitude for 2 g mm unbalance: 106 um at 108 Hz, 45 um at 128 Hz, 30 um at 152 Hz: BALANCE THE ROTOR to < 0.5 g mm (pin balance + hub spin balance) or the low-throttle 1P hub motion is large. A softer washer (k x 0.4: ro 6.5 mm, same ri/t, or t 25 mm) puts the hub mode at ~66 Hz and cuts the 108 Hz amplitude to 25 um, but that variant is not Floquet-checked.
Costs: tilt stiffness ~4.4 N m/rad: the 0.0975 N m cyclic moment tilts the disc ~1.3 deg relative to the arm (steady); axial sag ~92 um at 16.65 N hover thrust.

## Minimum-inertia hub build (J_hub itemised)
Direct-to-bell clamp: the printed hub assembly is screwed straight onto the motor bell (no nut, no adapter). Itemised polar inertia about the rotor axis (bell stays in J_rotor):
| item | mass | radius | J (kg m^2) |
|---|---|---|---|
| printed hub assembly (hub_fitting_assembly.stl, PA6-CF, Izz of the STL) | {Jt['hub_g']:.2f} g | - | {Jt['J_hub_stl']*1e6:.2f}e-6 |
| 4 hub-side strip lands, 8 laminae of 0.10 x 9 x 12 mm steel each, 2 per blade side | 4 x {Jt['strip_land_g_each']:.2f} g | 12.6-22.7 mm (land centre to axis) | {Jt['J_lands']*1e6:.2f}e-6 |
| 4x M3x6 socket screws into the bell | 4 x 0.5 g | 8 mm | {Jt['J_screws']*1e6:.2f}e-6 |
| thread-locker / knots / tape allowance | ~0.2 g | 8 mm | {Jt['J_misc']*1e6:.2f}e-6 |
| total J_hub | | | **{Jt['J_total']*1e6:.2f}e-6** |

The schedule below is computed at J_hub 2.5, 3.76 and 5.49e-6 (the 3.76e-6 column is the v4 itemisation; the v5 itemised value {Jt['J_total']*1e6:.2f}e-6 lies between the 2.5 and 3.76 columns; use the smaller pulse of the bracketing rows). Not verified below 2.5e-6 or above 5.49e-6.

## Bifilar measurement of J_hub
Hang the motor bell + printed hub (no blades, screws fitted) from two parallel threads of length l, separation 2a (a ~ 30 mm, l ~ 0.5 m); twist about the vertical axis and time 20+ oscillations (period T): J = m g a^2 T^2 / (4 pi^2 l), m = bell + hub mass. Repeat with the bell alone (3115 bell, expected J_rotor ~2.2e-5 per the design motor): J_hub = J(bell+hub) - J(bell). Photogate or slow-mo video. Accept J_hub within the itemised value +- 0.5e-6; the design is feasible 2.5 to 5.49e-6.

## Pluck-test mode table
See "Predicted modes" above (rest and hover tables from the v5 model). Pass criteria: lowest pitch-sym line (~19 Hz at rest) within 15 %, antisym+hub line (31 Hz at rest, mount-dependent: clamp the arm stiffly) within 20 %, flap sym/antisym (~190 Hz) within 10 %; frequency scales as sqrt(k), strip width +5 % = +2.4 %. Pluck the tip chordwise (pitch) and flapwise (flap), 5 s phone spectrum. Hover line to avoid: pitch antisym+hub 100 / 119 / 141 Hz against 1P 108 / 128 / 152 Hz (ratio 0.93, 0.93, 0.93; the resonance check requires |1 - ratio| margin >= 0.15 from the effective frequency including damping; see verify_10in_v5.md).

## FC pulse schedule (cyclic pulse amplitude vs throttle and measured J_hub)
The stored design op pulse is the original; the FC applies this schedule (Lua in-flight calibrated 1P phase; boot reference on the ESC). Pulse scale is relative to 0.95 x the motor's capability pulse at that rpm; values above 1 x mod_frac_max are fine because the motor_torque_1p check on the delivered torque is the real cap. Interpolate linearly in throttle and J_hub between rows; between columns use the smaller pulse of the two bracketing rows. Fatigue binds at low and mid throttle, the motor 1P torque at T1.4. Robust cr_pl (min over all) = **{R['pl']['summary']['cr_pl']:.4f}**; tolerance (nominal + 8 random +-5 % on t, w, L, hinge r, rho, strap, skew +-1 deg, EA +-0.01) **8/9 pass** (study/results/steelx_10in_v5.md; random07 fails the resonance check, margin 0.144).

""" + "\n".join(rows) + "\n\nProcedure: measure J_hub (bifilar), pick the matching J column, program the pulse for each throttle.\n"
md = md.replace("Files (this folder):", "Files (this folder): `mount_tube_clamp.stl`, `mount_tpu_washer.stl`, `mount_carrier.stl` (make_mount.py), ", 1)
(B / "BUILD.md").write_text(md)
print(len(md))
