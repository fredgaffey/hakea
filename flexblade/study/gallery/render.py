"""Fast static renders (matplotlib) of a design: material colours, and see-through (inserts solid).
Usage: render.py <design.json> <out_prefix>"""
import json, sys, warnings
warnings.filterwarnings("ignore")
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from flexblade import geometry
from flexblade.schema import Design

def draw(ax, parts, see, zoom=None):
    allv = []
    for p in parts:
        V = np.asarray(p["vertices"]) * 1000.0; F = np.asarray(p["faces"])
        insert = p.get("role") in ("rod", "wire", "bar", "strip")
        a = 1.0 if (insert or not see) else 0.12
        col = p.get("color") or "#888888"
        pc = Poly3DCollection(V[F], facecolors=col, edgecolors=col, linewidths=0, alpha=a, shade=True,
                              lightsource=matplotlib.colors.LightSource(azdeg=225, altdeg=45))
        ax.add_collection3d(pc); allv.append(V)
    V = np.vstack(allv); c = V.mean(0); r = np.ptp(V, 0).max() / 2
    if zoom is not None:
        c, r = np.asarray(zoom[0]), zoom[1]
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r*0.5, c[2]+r*0.5)
    ax.set_box_aspect((1, 1, 0.5)); ax.view_init(28, -60); ax.set_axis_off()

d = Design.model_validate(json.load(open(sys.argv[1])))
s = geometry.build_surface(d)
parts = [p for p in s["parts"] if p.get("blade") in (1, 0, None) or p.get("role") == "hub"]
R = d.blade.radius * 1000
root = ([0.17 * R, 0, 0], 0.2 * R)
for name, see, zoom in (("material", False, None), ("root", False, root), ("seethrough", True, root)):
    fig = plt.figure(figsize=(5.2, 3.6), dpi=110); ax = fig.add_subplot(111, projection="3d")
    draw(ax, parts, see, zoom); fig.subplots_adjust(-0.15, -0.15, 1.15, 1.15)
    fig.savefig(f"{sys.argv[2]}_{name}.png", facecolor="white"); plt.close(fig)
mats = sorted({(p["material"], p.get("color") or "#888") for p in parts})
json.dump(mats, open(f"{sys.argv[2]}_legend.json", "w"))
