"""Build the printable parts (one-piece breakaway jig print) for 10in_easy_steelx (run from flexblade/).

Parts come from flexblade.fittings (parametric clamp fittings built by manifold3d CSG from the
flexure parameters: clamp lands + slots, radiused jaw exits, free span = modelled strip length,
strand channel + knot well, >= 1.2 mm walls). Outputs watertight STL (mm, hub frame, z up =
build direction = rotor axis) plus drawing.png and parts.json into study/build/10in_easy/. BUILD.md is hand-written and never overwritten.
"""
import json
from pathlib import Path

import numpy as np
import trimesh

from flexblade.schema import Design
from flexblade import fittings as FT
from flexblade.materials import get_material
from flexblade.geometry import HUB_MATERIAL

OUT = Path("study/build/10in_easy"); OUT.mkdir(parents=True, exist_ok=True)
D = Design.model_validate(json.load(open("study/designs/10in_easy_steelx.json")))
parts = FT.strip_print_parts(D, breakaway_jig=True)
fits = parts.pop("_fits")
bridge_info = parts.pop("_bridge_info")
prm = fits.params
info = {}

for name, fn in (("rotor", "rotor_jig"),):
    m, kept, dropped = FT.to_trimesh(parts[name])
    m.export(OUT / f"{fn}.stl")
    m2 = trimesh.load(OUT / f"{fn}.stl", process=True)
    info[fn] = dict(watertight=bool(m2.is_watertight), components=kept, fragments_dropped=dropped,
                    volume_mm3=round(float(m2.volume), 1), bbox_mm=np.round(m2.extents, 1).tolist(),
                    bridge_volume_mm3=round(parts["bridges"].volume(), 2),
                    bridges=[[dict(a=a_, b=b_, length_mm=round(g, 2)) for a_, b_, g in bi] for bi in bridge_info])
    print(fn, info[fn])
for old in ("blade_with_root_fitting", "hub_fitting_assembly", "blade1_with_root_fitting", "blade2_with_root_fitting"):
    for p in OUT.glob(old + ".stl"):
        p.unlink()                               # superseded by the one-piece jig print

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
fig.suptitle(f"10in_tt_strap_steelx_easy blade-1 fittings: hub (blue), root (orange), strips (black, "
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

(OUT / "parts.json").write_text(json.dumps(info, indent=1, default=float))
print("jig free length mm (print sets jaw exits)", prm.L)
