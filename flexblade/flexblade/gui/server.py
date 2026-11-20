"""FastAPI app for the flexblade GUI and REST API (OWNER: ui). Built only on flexblade.api.

REST (documented at /docs):
  GET  /api/info                 versions, which backend modules import, fake flag
  GET  /api/schema               JSON schemas (Design, Results, OptimizeRequest)
  GET  /api/presets              {presets: {name: Design}, error?}
  GET  /api/presets/{name}       Design
  GET  /api/materials            material library
  GET  /api/hints                parameter-panel hints (units, ranges, groups)
  POST /api/validate             {design} -> {ok, design_hash, design} | {ok:false, errors}
  POST /api/set                  {design, values:{path:value}} -> Design
  POST /api/evaluate             {design, what?} -> Results (full, incl. nodal harmonic field)
  POST /api/digest               {results} -> {digest}
  POST /api/geometry             {design} -> {units, frame, parts:[{name, role, blade, material, color, vertices, faces}]}
  GET  /api/geometry             same for ?preset=NAME, else the session design
  POST /api/export?fmt=step      {design} -> file download (glb|stl|step|json)
  GET  /api/export?fmt=step      same for ?preset=NAME, else the session design (Keel bridge)
  POST /api/optimize             OptimizeRequest -> {job_id}
  POST /api/sweep                {design, variables:{path:[values] | "lo:hi:n"}, what?} -> {job_id}
  GET  /api/jobs/{id}            {status, progress, history, result|error}
  GET/PUT /api/session/design    the "current" design shared between GUI tabs and the Keel bridge

`design` everywhere may be a Design JSON, a preset name, or {"preset": NAME, "set": {path: value}}.
"""
from __future__ import annotations

import importlib
import ipaddress
import os
import tempfile
import threading
import time
import uuid
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

from fastapi import Body, FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from .. import api, uicommon as uc
from ..schema import MODEL_VERSION, Design, OptimizeRequest
from .hints import all_hints

STATIC = Path(__file__).parent / "static"
KEEL = Path(__file__).resolve().parents[1] / "keel"

DEFAULT_CORS = ["http://127.0.0.1:8787", "http://localhost:8787"]
MEDIA = {"glb": "model/gltf-binary", "stl": "model/stl", "step": "model/step", "json": "application/json"}


def _cors_origins(extra: list[str] | None) -> list[str]:
    env = [o.strip() for o in os.environ.get("FLEXBLADE_CORS_ORIGINS", "").split(",") if o.strip()]
    return sorted(set(DEFAULT_CORS + env + (extra or [])))


def _err(e: Exception, status: int = 400) -> HTTPException:
    detail: dict[str, Any] = {"error": str(e).splitlines()[0] if str(e) else type(e).__name__,
                              "type": type(e).__name__}
    if hasattr(e, "errors") and callable(e.errors):
        try:
            detail["errors"] = [{"loc": ".".join(map(str, x["loc"])), "msg": x["msg"]} for x in e.errors()]
            status = 422
        except Exception:  # noqa: BLE001
            pass
    if isinstance(e, (ModuleNotFoundError, ImportError, NotImplementedError)):
        detail["hint"] = "physics module not available yet"
        status = 501
    return HTTPException(status_code=status, detail=detail)


class Jobs:
    """Background jobs (optimise / sweep) in a small thread pool, polled by the client."""

    def __init__(self, workers: int = 2):
        self.pool = ThreadPoolExecutor(max_workers=workers, thread_name_prefix="flexblade-job")
        self.jobs: "OrderedDict[str, dict]" = OrderedDict()
        self.lock = threading.Lock()

    def submit(self, kind: str, fn, *args) -> str:
        jid = uuid.uuid4().hex[:12]
        job = {"id": jid, "kind": kind, "status": "queued", "created": time.time(), "started": None,
               "finished": None, "progress": {"i": 0, "n": None, "best": None}, "history": [],
               "result": None, "error": None}
        with self.lock:
            self.jobs[jid] = job
            while len(self.jobs) > 50:
                self.jobs.popitem(last=False)

        def run():
            job["status"] = "running"
            job["started"] = time.time()
            try:
                job["result"] = fn(job, *args)
                job["status"] = "done"
            except Exception as e:  # noqa: BLE001
                job["error"] = f"{type(e).__name__}: {e}"
                job["status"] = "error"
            job["finished"] = time.time()

        self.pool.submit(run)
        return jid

    def get(self, jid: str) -> dict:
        if jid not in self.jobs:
            raise HTTPException(404, {"error": f"no job {jid}"})
        j = dict(self.jobs[jid])
        t1 = j["finished"] or time.time()
        j["elapsed_s"] = (t1 - j["started"]) if j["started"] else 0.0
        return j


def _optimize_job(job: dict, req: OptimizeRequest) -> dict:
    def progress(i, n, obj=None, best=None):
        job["progress"] = {"i": i, "n": n, "best": best}
        job["history"].append({"eval": i, "objective": obj, "best": best})

    res = uc.call_optimize(req, progress)
    rj = res.model_dump(mode="json")
    if not job["history"]:  # backend gave no progress: show its history at the end
        best = None
        for h in rj.get("history", []):
            o = h.get("objective")
            best = o if best is None or (o is not None and o < best) else best
            job["history"].append({"eval": h.get("eval"), "objective": o, "best": best})
    rj["digest"] = uc.digest(res.results)
    return rj


def _sweep_job(job: dict, design: Design, variables, what) -> dict:
    rows = []
    n = 1
    for _, v in variables:
        n *= len(v)
    for rec in uc.run_sweep(design, variables, what, workers=1):
        rows.append(rec)
        job["progress"] = {"i": len(rows), "n": n, "best": None}
    return {"variables": {p: v for p, v in variables}, "rows": rows}


def create_app(cors_origins: list[str] | None = None) -> FastAPI:
    app = FastAPI(title="flexblade", version=MODEL_VERSION,
                  description="Flexure/compliant-blade designer and simulator for the Hakea "
                              "swashplateless rotor. SI units except *_deg (degrees) and "
                              "diameter_in (inches). See DESIGN.md for frames.")
    origins = _cors_origins(cors_origins)
    app.add_middleware(CORSMiddleware, allow_origins=origins, allow_methods=["*"], allow_headers=["*"],
                       expose_headers=["Content-Disposition", "X-Design-Hash"])

    @app.middleware("http")
    async def private_network(request: Request, call_next):
        # Chrome Private Network Access: an https Keel page calling http://127.0.0.1 sends a
        # preflight with Access-Control-Request-Private-Network; answer it for allowed origins.
        resp = await call_next(request)
        if request.headers.get("access-control-request-private-network") and \
                request.headers.get("origin") in origins:
            resp.headers["Access-Control-Allow-Private-Network"] = "true"
        return resp

    jobs = Jobs()
    eval_cache: "OrderedDict[tuple, dict]" = OrderedDict()
    session: dict[str, Any] = {"design": None}
    app.state.jobs = jobs
    app.state.session = session
    app.state.cors_origins = origins

    def design_of(body: Any) -> Design:
        try:
            return uc.resolve_design(body)
        except Exception as e:  # noqa: BLE001
            raise _err(e, 422) from e

    def session_design() -> Design:
        if session["design"] is None:
            pres, _ = uc.presets_safe()
            session["design"] = next(iter(pres.values())) if pres else Design()
        return session["design"]

    # ------------------------------------------------------------------ meta

    @app.get("/api/info")
    def info():
        mods = {}
        for m in ("rotor", "aero", "optimize", "geometry", "presets", "fea"):
            try:
                importlib.import_module(f"flexblade.{m}")
                mods[m] = True
            except Exception:  # noqa: BLE001
                mods[m] = False
        fake = getattr(api.evaluate, "__module__", "").endswith("fake_backend")
        return {"model_version": MODEL_VERSION, "modules": mods, "fake_backend": fake,
                "cors_origins": origins}

    @app.get("/api/schema")
    def schema():
        return api.schema()

    @app.get("/api/presets")
    def presets():
        pres, err = uc.presets_safe()
        out: dict[str, Any] = {"presets": {k: v.model_dump(mode="json") for k, v in pres.items()}}
        if err:
            out["error"] = err
        return out

    @app.get("/api/presets/{name}")
    def preset(name: str):
        return design_of({"preset": name}).model_dump(mode="json")

    @app.get("/api/materials")
    def materials():
        return {"materials": api.materials()}

    @app.get("/api/hints")
    def hints():
        return all_hints()

    # ------------------------------------------------------------------ design

    @app.post("/api/validate")
    def validate(body: dict = Body(...)):
        try:
            d = uc.resolve_design(body.get("design", body))
        except Exception as e:  # noqa: BLE001
            he = _err(e, 422)
            return {"ok": False, **he.detail}
        return {"ok": True, "design_hash": d.hash(), "design": d.model_dump(mode="json")}

    @app.post("/api/set")
    def set_values(body: dict = Body(...)):
        d = design_of(body.get("design"))
        try:
            return uc.set_values(d, body.get("values", {})).model_dump(mode="json")
        except Exception as e:  # noqa: BLE001
            raise _err(e, 422) from e

    @app.get("/api/session/design")
    def get_session():
        return session_design().model_dump(mode="json")

    @app.put("/api/session/design")
    def put_session(body: dict = Body(...)):
        session["design"] = design_of(body.get("design", body))
        return {"ok": True, "design_hash": session["design"].hash()}

    # ------------------------------------------------------------------ analysis

    @app.post("/api/evaluate")
    def evaluate(body: dict = Body(..., examples=[{"design": "hakea_5in_hinge_equiv",
                                                   "what": ["trim", "modes", "harmonic", "checks"]}])):
        d = design_of(body.get("design"))
        what = body.get("what")
        key = (d.hash(), tuple(what or ()), id(api.evaluate))
        if key in eval_cache:
            eval_cache.move_to_end(key)
            return eval_cache[key]
        try:
            r = api.evaluate(d, what).model_dump(mode="json")
        except Exception as e:  # noqa: BLE001
            raise _err(e, 500) from e
        eval_cache[key] = r
        while len(eval_cache) > 32:
            eval_cache.popitem(last=False)
        return r

    @app.post("/api/digest")
    def digest(body: dict = Body(...)):
        return {"digest": uc.digest(body.get("results", body))}

    @app.post("/api/geometry")
    def geometry(body: dict = Body(...)):
        d = design_of(body.get("design"))
        try:
            return uc.load_geometry(d)
        except Exception as e:  # noqa: BLE001
            raise _err(e, 500) from e

    @app.get("/api/geometry")
    def geometry_get(preset: str | None = None):
        d = design_of({"preset": preset}) if preset else session_design()
        try:
            return uc.load_geometry(d)
        except Exception as e:  # noqa: BLE001
            raise _err(e, 500) from e

    def _export(d: Design, fmt: str) -> Response:
        if fmt not in MEDIA:
            raise HTTPException(400, {"error": f"fmt must be one of {sorted(MEDIA)}"})
        td = tempfile.mkdtemp(prefix="flexblade-export-")
        fname = f"{d.name or 'flexblade'}.{fmt}"
        try:
            p = api.export_geometry(d, fmt, str(Path(td) / fname))
        except Exception as e:  # noqa: BLE001
            raise _err(e, 500) from e
        return FileResponse(p or str(Path(td) / fname), media_type=MEDIA[fmt], filename=fname,
                            headers={"X-Design-Hash": d.hash()})

    @app.post("/api/export")
    def export(fmt: str = "step", body: dict = Body(...)):
        return _export(design_of(body.get("design")), fmt)

    @app.get("/api/export")
    def export_get(fmt: str = "step", preset: str | None = None):
        return _export(design_of({"preset": preset}) if preset else session_design(), fmt)

    # ------------------------------------------------------------------ jobs

    @app.post("/api/optimize")
    def optimize(body: dict = Body(...)):
        if isinstance(body.get("design"), (str, dict)):
            body = dict(body, design=design_of(body["design"]).model_dump(mode="json"))
        try:
            req = OptimizeRequest.model_validate(body)
        except Exception as e:  # noqa: BLE001
            raise _err(e, 422) from e
        return {"job_id": jobs.submit("optimize", _optimize_job, req)}

    @app.post("/api/sweep")
    def sweep(body: dict = Body(...)):
        d = design_of(body.get("design"))
        try:
            variables = [uc.parse_sweep_var(f"{p}={v}") if isinstance(v, str) else (p, list(v))
                         for p, v in body.get("variables", {}).items()]
            n = 1
            for p, v in variables:
                uc.set_values(d, {p: v[0]})
                n *= len(v)
        except Exception as e:  # noqa: BLE001
            raise _err(e, 422) from e
        if n > int(body.get("max_points", 400)):
            raise HTTPException(422, {"error": f"grid has {n} points > max_points"})
        return {"job_id": jobs.submit("sweep", _sweep_job, d, variables, body.get("what"))}

    @app.get("/api/jobs/{jid}")
    def job(jid: str):
        return jobs.get(jid)

    @app.get("/api/jobs")
    def job_list():
        return {"jobs": [{k: j[k] for k in ("id", "kind", "status", "progress")} for j in jobs.jobs.values()]}

    # ------------------------------------------------------------------ static

    @app.exception_handler(HTTPException)
    async def http_exc(request: Request, exc: HTTPException):
        detail = exc.detail if isinstance(exc.detail, dict) else {"error": str(exc.detail)}
        return JSONResponse(detail, status_code=exc.status_code)

    if KEEL.exists():
        app.mount("/keel", StaticFiles(directory=KEEL), name="keel")
    app.mount("/", StaticFiles(directory=STATIC, html=True), name="static")
    return app


def _is_loopback(host: str) -> bool:
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def serve(host: str = "127.0.0.1", port: int = 8765, cors_origins: list[str] | None = None,
          open_browser: bool = False) -> None:
    import sys

    import uvicorn
    if not _is_loopback(host) and os.environ.get("FLEXBLADE_ALLOW_REMOTE") != "1":
        raise SystemExit(f"refusing to bind {host}: the GUI has no auth; use 127.0.0.1 "
                         "(or set FLEXBLADE_ALLOW_REMOTE=1 if you really mean it)")
    app = create_app(cors_origins)
    url = f"http://{host}:{port}/"
    print(f"[flexblade] GUI on {url}  (API docs {url}docs; CORS: {', '.join(app.state.cors_origins)})",
          file=sys.stderr)
    if open_browser:
        import webbrowser
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    uvicorn.run(app, host=host, port=port, log_level="warning")
