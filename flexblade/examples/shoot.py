"""Headless-Chrome smoke test + screenshots of the flexblade GUI (dev tool).

    FLEXBLADE_FAKE=1 flexblade gui --port 8799 &
    python examples/shoot.py http://127.0.0.1:8799 examples/   [--tab campbell ...]

Fails (exit 1) on any console error / uncaught exception. Needs google-chrome or chromium
and the `websockets` package. Uses the DevTools protocol directly (no playwright).
"""
from __future__ import annotations

import asyncio
import base64
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

import websockets


def find_chrome() -> str:
    for c in (os.environ.get("CHROME"), "google-chrome", "chromium", "chromium-browser", "google-chrome-stable"):
        if c and shutil.which(c):
            return shutil.which(c)
    raise SystemExit("no chrome/chromium found")


async def main(base: str, outdir: str, shots: list[tuple[str, str]]):
    port = 9333
    prof = tempfile.mkdtemp(prefix="fb-chrome-")
    proc = subprocess.Popen([find_chrome(), "--headless=new", f"--remote-debugging-port={port}",
                             f"--user-data-dir={prof}", "--window-size=1600,1000", "--hide-scrollbars",
                             "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--no-first-run",
                             "about:blank"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    errors: list[str] = []
    try:
        for _ in range(50):
            try:
                tabs = json.load(urllib.request.urlopen(f"http://127.0.0.1:{port}/json"))
                ws_url = next(t["webSocketDebuggerUrl"] for t in tabs if t["type"] == "page")
                break
            except Exception:  # noqa: BLE001
                time.sleep(0.2)
        async with websockets.connect(ws_url, max_size=50_000_000) as ws:
            n = 0
            pending: dict[int, asyncio.Future] = {}

            async def reader():
                async for msg in ws:
                    m = json.loads(msg)
                    if "id" in m and m["id"] in pending:
                        pending.pop(m["id"]).set_result(m)
                    elif m.get("method") == "Runtime.exceptionThrown":
                        d = m["params"]["exceptionDetails"]
                        errors.append(f"exception: {d.get('exception', {}).get('description') or d.get('text')}")
                    elif m.get("method") == "Runtime.consoleAPICalled" and m["params"]["type"] in ("error", "assert"):
                        errors.append("console.error: " + " ".join(str(a.get("value", a.get("description")))
                                                                   for a in m["params"]["args"]))
                    elif m.get("method") == "Log.entryAdded" and m["params"]["entry"]["level"] == "error":
                        e = m["params"]["entry"]
                        errors.append(f"log: {e['text']} {e.get('url', '')}")

            task = asyncio.create_task(reader())

            async def cmd(method, **params):
                nonlocal n
                n += 1
                fut = asyncio.get_event_loop().create_future()
                pending[n] = fut
                await ws.send(json.dumps({"id": n, "method": method, "params": params}))
                return (await asyncio.wait_for(fut, 60)).get("result", {})

            await cmd("Runtime.enable")
            await cmd("Log.enable")
            await cmd("Page.enable")
            await cmd("Emulation.setDeviceMetricsOverride", width=1600, height=1000, deviceScaleFactor=1, mobile=False)
            for name, query, *action in shots:
                await cmd("Page.navigate", url=f"{base}/?{query}")
                for _ in range(150):
                    r = await cmd("Runtime.evaluate", expression="document.body.dataset.ready === '1' && "
                                  "!(window.flexblade && window.flexblade.evaluating)", returnByValue=True)
                    if r.get("result", {}).get("value"):
                        break
                    await asyncio.sleep(0.2)
                if action:
                    await cmd("Runtime.evaluate", expression=action[0])
                    await asyncio.sleep(float(action[1]) if len(action) > 1 else 3.0)
                await asyncio.sleep(2.0)  # let geometry + animation settle
                st = await cmd("Runtime.evaluate", expression="document.getElementById('status').textContent + ' | ' + "
                               "document.getElementById('vp-msg').textContent", returnByValue=True)
                print(f"{name}: status = {st['result'].get('value')}")
                shot = await cmd("Page.captureScreenshot", format="png")
                path = os.path.join(outdir, f"gui_{name}.png")
                with open(path, "wb") as f:
                    f.write(base64.b64decode(shot["data"]))
                print("wrote", path)
            task.cancel()
    finally:
        proc.terminate()
        shutil.rmtree(prof, ignore_errors=True)
    if errors:
        print("ERRORS:\n  " + "\n  ".join(errors))
        return 1
    print("no console errors")
    return 0


if __name__ == "__main__":
    base = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8765"
    out = sys.argv[2] if len(sys.argv) > 2 else "."
    shots = [("summary_dark", "eval=1&theme=dark"), ("campbell_light", "eval=1&theme=light&tab=campbell"),
             ("harmonic_dark", "eval=1&theme=dark&tab=harmonic"), ("checks_dark", "eval=1&theme=dark&tab=checks"),
             ("optimise_dark", "eval=1&theme=dark&tab=optimise",
              "document.querySelector('#tab-optimise button.primary').click()", 5),
             ("coaxial_dark", "eval=1&theme=dark&set=layout=coaxial", "document.getElementById('show-nodes').click();", 1),
             ("coaxial_harmonic_light", "eval=1&theme=light&tab=harmonic&set=layout=coaxial"),
             ("util_colour_light", "eval=1&theme=light&tab=checks",
              "document.getElementById('colour-by').value='util';"
              "document.getElementById('colour-by').dispatchEvent(new Event('change'));"
              "document.getElementById('show-nodes').click();", 1)]
    sys.exit(asyncio.run(main(base, out, shots)))
