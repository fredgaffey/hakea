"""E06: print orientation / install / forming comparison, torsion-bar family on 5" and 6"
(2307, 0.6 kg; base = reinforce_grid.base_for with full_grid motor/mass). Writes e06_orientation.md."""
import sys, warnings, itertools
from pathlib import Path
warnings.filterwarnings("ignore")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent.parent))
import full_grid as fg, reinforce_grid as rg            # noqa: E402  (sets motor/mass)
from flexblade import api                                # noqa: E402
from flexblade.schema import PrintSetup, Layup           # noqa: E402

PAUSE = {"bar": "pause"}
VARIANTS = [
    ("legacy (no print_setup)", None),
    ("flat, bar paused", PrintSetup(preset="flat", install=PAUSE)),
    ("on_edge, bar paused", PrintSetup(preset="on_edge", install=PAUSE)),
    ("hinge_in_plane, bar paused", PrintSetup(preset="hinge_in_plane", install=PAUSE)),
    ("tilted 45, bar paused", PrintSetup(preset="tilted", tilt_deg=45.0, install=PAUSE)),
    ("drilled_torsion_bar (flat)", PrintSetup(preset="drilled_torsion_bar")),
    ("formed_torsion_bar (rod mandrel)", PrintSetup(preset="formed_torsion_bar")),
]


READING = """## Reading

- flat (build = rotor axis z) with a laid-in bar: the bar lies 45 deg out of the layer plane (insert_in_layer util 9.0) -> infeasible; the strength model agrees (bar fibres cross-layer: ctrl 0.001-0.01, interlayer util 5.4 at 5in, 1.5 at 6in).
- on_edge (build = y): bar in-plane, interlayer unchanged (0.25 / 0.34), but the plate lies horizontal in mid-air: feasible only as a bridge (plate length <= 10 mm, counted 0.9 x the 50 deg limit); a longer plate needs supports.
- hinge_in_plane == tilted 45 (build = (cos skew, 0, sin skew), tilt 45 deg for skew 45): bar in-plane, 0 deg overhang, no supports -- cleanest print, but the blade span crosses the layers at 45 deg: interlayer util 0.81 (5in) / 0.92 (6in) vs 0.25 / 0.34, and ctrl ~3 % lower.
- drilled_torsion_bar (flat print, bar drilled after): no layer constraint, fibres along the rod regardless of build -> ctrl/axis error as on_edge; interlayer 0.80-0.90 (plate printed edge-on). Wall (2.5 mm plate, 1.5 mm hole -> 0.5 mm) is exactly at the limit (util 1.00); drill length passes (12 d = 18 mm).
- formed_torsion_bar (flat, rod first as forming mandrel, twist 90 deg about the rod, then bend 45 deg about y at 8.75 mm radius): insert in-plane as printed, twist axis == rod axis, TPU 95A forming 100 C < rod Tg 110 C (assumed), bend radius >= 3 t. Interlayer util 0.25/0.34 (best), ctrl as legacy. PA6-CF (160 C) would need the rod drilled after forming.

Caveats: forming = rigid rotation of region orientation (residual stress not modelled); blade k>1 strength uses blade-1 orientations (printability checks use true per-blade axes); bridging (<= 10 mm), bed seating (1 mm), forming temperatures and rod Tg are assumptions; hinge-axis error (~8-11 deg) is not changed by print setup.
"""


def design(D):
    d = rg.base_for(D).model_copy(deep=True)
    f = d.flexure
    f.topology, f.bar_diameter, f.bar_length, f.thickness = "torsion_bar", 0.0015, 0.012 * D / 5, 0.0025
    f.layup = Layup(kind="solid", material="tpu_95a")
    f.tension_member = None
    return d


def main():
    lines = ["# E06 print orientation, install and forming (torsion bar, 2307, 0.6 kg)", "",
             "Bar 1.5 mm CF rod, TPU 95A plate 2.5 mm, skew 45 deg, bar length 12 mm x D/5. "
             "Fails = print:* checks with utilisation > 1 (first 3 listed). il = worst interlayer util "
             "(blade+flexure). Bed 270 mm cube.", "",
             "| D | variant | ctrl | axis err deg | il | insert err deg | overhang deg | print max util | fails |",
             "|---|---|---|---|---|---|---|---|---|"]
    for D in (5, 6):
        base = design(D)
        for name, ps in VARIANTS:
            try:
                r = api.evaluate(base.model_copy(update=dict(print_setup=ps)))
            except Exception as e:                       # noqa: BLE001
                lines.append(f"| {D} | {name} | error {str(e)[:60]} |"); continue
            s = r.summary
            il = max([c.utilisation for c in r.checks if c.name == "interlayer"] or [0])
            fails = sorted({f"{c.name}({c.utilisation:.1f})" for c in r.checks
                            if c.member.startswith("print:") and c.utilisation > 1})
            lines.append(f"| {D} | {name} | {s['control_ratio']:.3f} | {s.get('hinge_axis_err_deg', 0):.1f} | "
                         f"{il:.2f} | {s.get('print_insert_err_deg', float('nan')):.1f} | "
                         f"{s.get('print_overhang_max_deg', float('nan')):.0f} | "
                         f"{s.get('print_max_util', float('nan')):.2f} | {', '.join(fails[:3]) or 'none'} |")
    lines += ["", READING]
    (HERE / "e06_orientation.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
