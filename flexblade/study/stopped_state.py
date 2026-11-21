"""Stopped/assembled rotor with pre-angled strip slots (strip_preset_deg): zero-load self-stressed
equilibrium (no CF, no aero), preset load-stepped 0->100 % (fact.stopped_state). Reports per-strip axial
force vs Euler, max clamp-edge stress, buckling multiplier, stopped tangent k_cross/k_flap.
Usage: python study/stopped_state.py [design.json ...]"""
import sys
from pathlib import Path

from flexblade import rotor
from flexblade.fact import stopped_state
from flexblade.materials import get_material
from flexblade.schema import Design

HERE = Path(__file__).resolve().parent
DEF = [HERE / "designs" / f"10in_tt_strap_steelx_{v}.json" for v in ("v8", "v10m")]

if __name__ == "__main__":
    steps = [int(x) for x in (sys.argv[1].split(",") if len(sys.argv) > 1 and "," in sys.argv[1] else ["4", "20"])]
    for p in [a for a in sys.argv[1:] if a.endswith(".json")] or DEF:
        d = rotor.resolve_pre_lag(Design.model_validate_json(Path(p).read_text()))
        mat = get_material(d.flexure.layup.material)
        print(f"== {Path(p).stem}  Xt {mat.Xt / 1e6:.0f} MPa, fatigue amp "
              f"{mat.fatigue_strain_amp_allow * mat.E1 / 1e6:.0f} MPa, Kt {rotor.strip_kt(d.flexure):.2f}")
        for nst in steps:
            r = stopped_state(d, n_steps=nst)
            print(f"steps {nst}: converged {r['converged']} iters {r['iters']}  buckling multiplier "
                  f"{r['buckling_multiplier']:.2f}  max sigma {r['sigma_max'] / 1e6:.0f} MPa")
            for s in r["strips"]:
                print(f"  {s['tag']:12s} N {s['N']:+8.2f} N  Pcr {s['Pcr']:7.1f}  N/Pcr {s['N'] / s['Pcr']:+.3f}  "
                      f"sigma {s['sigma'] / 1e6:6.0f} MPa")
            for k, v in r["k"].items():
                print(f"  blade {k}: k_cross {v['k_cross']:.1f} (no preset {v['k_cross0']:.1f})  k_flap "
                      f"{v['k_flap']:.1f} ({v['k_flap0']:.1f})  k_hinge {v['k_hinge']:.4f} ({v['k_hinge0']:.4f})\n"
                      f"     principal: k_soft {v['k_soft']:.4f}  k_2nd {v['k_2nd']:.1f} (no preset {v['k_2nd0']:.1f})  "
                      f"soft-axis tilt {v['axis_tilt_deg']:.2f} deg ({v['axis_tilt_deg0']:.3f})  fitting rot "
                      f"{v['fitting_rot_deg']:.2f} deg ({v['fitting_rot_hinge_deg']:+.2f} about hinge)")
            print("  path k_cross b1:", " ".join(f"{x:.0f}" for x in r["path_kcross"]))
