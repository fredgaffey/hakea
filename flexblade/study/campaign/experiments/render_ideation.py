"""Render every ideation concept's best design (material, see-through, utilisation, FEA) via the GUI
screenshot tool, into study/ideation/renders/<id>_*.png. Uses its own GUI server on port 8791."""
import json, shutil, subprocess, sys, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]                    # flexblade/
PY = str(Path.home() / "venv-flexblade/bin/python")
out = ROOT / "study/ideation/renders"; out.mkdir(parents=True, exist_ok=True)
tmp = ROOT / "study/campaign/experiments/_render_tmp"; tmp.mkdir(exist_ok=True)
srv = subprocess.Popen([str(Path.home() / "venv-flexblade/bin/flexblade"), "gui", "--port", "8791"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(8)
c = json.load(open(ROOT / "study/ideation/concepts.json"))
cs = c if isinstance(c, list) else c.get("concepts", c)
cs = cs if isinstance(cs, list) else list(cs.values())
try:
    for x in cs:
        if not x.get("best_design"):
            continue
        cid = x["id"]; dj = tmp / f"{cid}.json"; dj.write_text(json.dumps(x["best_design"]))
        for extra, names in (([], ("material", "seethrough", "util")), (["--fea"], ("fea",))):
            sd = tmp / f"{cid}_shots"; shutil.rmtree(sd, ignore_errors=True); sd.mkdir()
            subprocess.run([PY, str(ROOT / "examples/shoot.py"), *extra, "--design", str(dj),
                            "http://127.0.0.1:8791", str(sd)], timeout=600, capture_output=True)
            for f in sd.glob("gui_*.png"):
                shutil.copy(f, out / f"{cid}_{f.stem.replace('gui_', '')}.png")
        print(cid, "rendered", flush=True)
finally:
    srv.terminate()
print("done")
