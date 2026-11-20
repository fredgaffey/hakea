"""flexblade MCP server (stdio) for LLM agents (OWNER: ui). Built only on flexblade.api.

Run: `flexblade mcp`  (Claude Code: `claude mcp add flexblade -- ~/venv-flexblade/bin/flexblade mcp`)

The stdio transport diverts fd 1 to stderr while serving, so prints from physics code never
corrupt the protocol stream. Long calls run in worker threads.
"""
from __future__ import annotations

import functools
import inspect
import json
import tempfile
from pathlib import Path
from typing import Any, Literal

import anyio
import anyio.from_thread
import anyio.to_thread
from mcp.server.mcpserver import Context, MCPServer
from mcp.server.mcpserver.exceptions import ToolError

from . import api, uicommon as uc
from .schema import MODEL_VERSION, OptimizeRequest

DESIGN_MD = Path(__file__).resolve().parents[1] / "DESIGN.md"

INSTRUCTIONS = f"""\
flexblade {MODEL_VERSION}: designer/simulator for flexure (compliant) lag hinges and tuned
blades on a swashplateless torque-pulsed rotor (Hakea). Cyclic control comes from a 1/rev motor
torque pulse: blades lead/lag, a skewed hinge (skew_deg, blade 2 mirrored) turns lag into
opposite pitch on the two blades -> hub moment.

Units: SI everywhere (m, kg, s, N, Pa, rad) except fields ending _deg (degrees) and
blade.diameter_in (inches). Hub frame H rotates +Omega about +z (thrust up); blade 1 along +x,
travelling +y. Pitch + = nose up about blade +x; lag + = tip moves toward -y; flap + = tip up.
Harmonic convention x(t)=Re{{X e^(i w t)}}.

Workflow: list_presets -> get_preset -> set_design_values (dotted paths, e.g.
flexure.thickness=0.0012) -> evaluate_design (read the digest) -> explain_results / sweep /
optimize_design -> export_geometry. Every tool accepting `design` also takes a preset name or
{{"preset": NAME, "set": {{path: value}}}}. Read resource flexblade://design for the full contract.
Utilisation > 1 in checks means failure. Results are deterministic and carry design_hash.
"""

mcp = MCPServer(name="flexblade", version=MODEL_VERSION, instructions=INSTRUCTIONS)



def _err_text(e: Exception) -> str:
    msg = f"{type(e).__name__}: {str(e).splitlines()[0] if str(e) else ''}"
    if hasattr(e, "errors") and callable(e.errors):
        try:
            msg += " | " + "; ".join(f"{'.'.join(map(str, x['loc']))}: {x['msg']}" for x in e.errors())
        except Exception:  # noqa: BLE001
            pass
    if isinstance(e, (ModuleNotFoundError, NotImplementedError)):
        msg += " (physics module not available yet)"
    return msg


def tool(**kw):
    """mcp.tool that turns any exception into a ToolError the model can read."""
    def deco(fn):
        if inspect.iscoroutinefunction(fn):
            @functools.wraps(fn)
            async def wrapped(*a, **k):
                try:
                    return await fn(*a, **k)
                except ToolError:
                    raise
                except Exception as e:  # noqa: BLE001
                    raise ToolError(_err_text(e)) from e
        else:
            @functools.wraps(fn)
            def wrapped(*a, **k):
                try:
                    return fn(*a, **k)
                except ToolError:
                    raise
                except Exception as e:  # noqa: BLE001
                    raise ToolError(_err_text(e)) from e
        mcp.tool(**kw)(wrapped)
        return fn
    return deco


DesignArg = dict[str, Any] | str
DESIGN_DOC = ("Design JSON object (schema: get_schema('Design')), or a preset name, or "
              "{\"preset\": NAME, \"set\": {\"dotted.path\": value}}.")


def _design(d: DesignArg):
    return uc.resolve_design(d)


def _compact_results(r) -> dict:
    return uc.strip_heavy(r.model_dump(mode="json"))


# --------------------------------------------------------------------------- read-only tools


@tool(description="JSON schema of 'Design' (input), 'Results' (evaluate output) or "
                      "'OptimizeRequest'. Field descriptions carry units; SI unless *_deg/*_in.")
def get_schema(which: Literal["Design", "Results", "OptimizeRequest", "all"] = "Design") -> dict:
    s = api.schema()
    return s if which == "all" else s[which]


@tool(description="List preset designs (4\"-12\" rotors x material concepts): name, diameter_in, "
                      "flexure topology and materials. Use get_preset(name) for the full Design.")
def list_presets() -> dict:
    pres, err = uc.presets_safe()
    out = {"presets": [{"name": k, "diameter_in": d.blade.diameter_in, "topology": d.flexure.topology,
                        "flexure_material": d.flexure.layup.material,
                        "blade_material": d.blade.layup.material, "notes": d.notes}
                       for k, d in pres.items()]}
    if err and not pres:
        out["error"] = f"presets unavailable: {err}"
    return out


@tool(description="Full Design JSON of a preset (all SI; *_deg degrees; diameter_in inches).")
def get_preset(name: str) -> dict:
    return uc.resolve_design({"preset": name}).model_dump(mode="json")


@tool(description="Material library: generic entries plus real filament / insert products. Per "
                      "entry: name, density rho (kg/m^3), orthotropic moduli E1 (raster), E2 (in-layer "
                      "transverse), E3 (interlayer/build) in Pa, Xt/Zt strengths, loss factor, elastomer "
                      "flag, source; for real products also brand, product, cls (class code), group, url "
                      "(buy), tds_url, price/currency/usd_per_kg, confidence, shore, hardened_nozzle, "
                      "ams, moisture. Filter with `selector` (all | filaments | rigid | inserts | "
                      "class:tpu | brand:Bambu Lab | group:NAME). Use names in layup.material / "
                      "skin_material / insert_material.")
def list_materials(selector: str = "all") -> dict:
    from .materials import select_materials
    names = set(select_materials(selector))
    return {"materials": [m for m in api.materials() if m["name"] in names]}


@tool(description="Material sweep: evaluate `design` once per material in a layup slot "
                      "(body = blade+flexure base material, blade, flexure, skin, insert). `materials` "
                      "is a selector (all | filaments | rigid | inserts | class:tpu | class:pa_cf+pet_cf "
                      "| brand:NAME | comma list). resize=true rescales geometry so each element keeps "
                      "the original soft bending stiffness (fair comparison); false = same geometry. "
                      "torque: 'design' | 'capability' (motor 1/rev capability) | number. Returns rows "
                      "sorted by fitness: control_ratio, pitch_1p_deg, max_util + governing check, "
                      "lag/pitch per-rev and margins to 1P/2P, pitch split, rotor mass, filament cost, "
                      "fitness, feasible. " + DESIGN_DOC)
async def material_sweep(design: DesignArg, slot: str, materials: str = "all", resize: bool = False,
                         torque: str = "design", workers: int = 8, hosts: str | None = None,
                         out_path: str | None = None) -> dict:
    d = _design(design)
    try:
        tq: Any = float(torque)
    except ValueError:
        tq = torque

    def run():
        return api.material_sweep(d, slot, materials, resize=resize, torque=tq, hosts=hosts,
                                  workers=workers)
    recs = await anyio.to_thread.run_sync(run)
    recs.sort(key=lambda r: -(r.get("fitness") or -1))
    if out_path:
        from .matsweep import to_csv, to_jsonl
        Path(out_path).write_text(to_csv(recs) if out_path.endswith(".csv") else to_jsonl(recs))
    keep = ("material", "brand", "product", "cls", "control_ratio", "pitch_1p_deg", "max_util",
            "governing", "lag_per_rev", "pitch_per_rev", "lag_margin", "pitch_margin",
            "pitch_split_deg", "rotor_mass_g", "filament_cost_usd", "fitness", "feasible", "url",
            "error")
    rows = [{k: r.get(k) for k in keep if r.get(k) is not None} for r in recs]
    for r, full in zip(rows, recs):
        if full.get("resize"):
            r["resize_feasible"] = full["resize"]["feasible"]
    return {"n": len(rows), "slot": slot, "resized": resize, "rows": rows,
            **({"out_path": str(Path(out_path).resolve())} if out_path else {})}


@tool(description="Material-agnostic resize: switch layup `slot` (body|blade|flexure|skin|insert) to "
                      "`material` and rescale thickness (then width if below t_min) / blade t/c / skin "
                      "thickness / rod diameter so every element keeps its original soft bending "
                      "stiffness. Returns the resized Design and a report (per element old/new, achieved "
                      "ratio, feasibility, EA/GJ/EI_stiff/mass ratios). Re-run evaluate_design on it. "
                      + DESIGN_DOC)
def resize_for_material(design: DesignArg, slot: str, material: str, t_min: float = 0.4e-3) -> dict:
    d2, rep = api.resize_for_material(_design(design), slot, material, t_min=t_min)
    return {"design": d2.model_dump(mode="json"), "report": rep}


@tool(description="Validate a design. Returns {ok, design_hash, design} or {ok:false, errors:[{loc,msg}]}. "
                      + DESIGN_DOC)
def validate_design(design: DesignArg) -> dict:
    try:
        d = _design(design)
    except Exception as e:  # noqa: BLE001
        errs = []
        if hasattr(e, "errors"):
            errs = [{"loc": ".".join(map(str, x["loc"])), "msg": x["msg"]} for x in e.errors()]
        return {"ok": False, "error": str(e).splitlines()[0], "errors": errs}
    return {"ok": True, "design_hash": d.hash(), "design": d.model_dump(mode="json")}


@tool(description="Set values by dotted path and return the new validated Design. Paths address the "
                      "Design JSON, list indices as numbers: e.g. {\"flexure.thickness\": 0.0012 (m), "
                      "\"flexure.skew_deg\": 40, \"blade.diameter_in\": 7, \"flexure.layup.material\": "
                      "\"pa6_cf\", \"blade.stations.3.chord\": 0.012, \"op.rpm\": null}. " + DESIGN_DOC)
def set_design_values(design: DesignArg, values: dict[str, Any]) -> dict:
    return uc.set_values(_design(design), values).model_dump(mode="json")


@tool(description="Compact text digest of a Results JSON (as returned by evaluate_design 'results' or "
                      "an OptimizeResult's 'results'): trim, 1/rev harmonic (per-blade pitch/lag/flap amp "
                      "and phase, hub moment, control ratio), modes with per-rev ratios, summary, checks "
                      "sorted by utilisation (FAIL if >1), warnings.")
def explain_results(results: dict[str, Any]) -> str:
    return uc.digest(results)


# --------------------------------------------------------------------------- compute tools


@tool(description="Evaluate a design: static trim (BEM + centrifugal), modes/Campbell, 1/rev harmonic "
                      "response to the torque pulse, and stress/creep/fatigue checks. `what` subset of "
                      "[trim, modes, harmonic, checks] (default all). detail: 'digest' (text only), "
                      "'compact' (digest + Results without nodal arrays/mode shapes, default), 'full'. "
                      + DESIGN_DOC)
async def evaluate_design(design: DesignArg, what: list[str] | None = None,
                          detail: Literal["digest", "compact", "full"] = "compact") -> dict:
    d = _design(design)
    r = await anyio.to_thread.run_sync(lambda: api.evaluate(d, what))
    out: dict[str, Any] = {"design_hash": r.design_hash, "digest": uc.digest(r)}
    if detail == "compact":
        out["results"] = _compact_results(r)
    elif detail == "full":
        out["results"] = r.model_dump(mode="json")
    return out


@tool(description="Optimise design variables against targets. request = OptimizeRequest JSON: "
                      "{design, variables:[{path, lo, hi, log}], targets:[{quantity (dotted path into "
                      "Results, e.g. harmonic.control_ratio, summary.lag_freq_per_rev, "
                      "checks.max_utilisation), kind: min|max|eq|le|ge, value, weight}], method: "
                      "nelder_mead|differential_evolution|slsqp, max_evals, seed}. Can take minutes; "
                      "progress is reported. Returns best design, objective, digest, compact results, "
                      "history (last 50).")
async def optimize_design(request: dict[str, Any], ctx: Context) -> dict:
    if isinstance(request.get("design"), (str,)) or (
            isinstance(request.get("design"), dict) and "preset" in request["design"]):
        request = dict(request, design=_design(request["design"]).model_dump(mode="json"))
    req = OptimizeRequest.model_validate(request)

    def progress(i, n, obj=None, best=None):
        try:
            anyio.from_thread.run(ctx.report_progress, float(i), float(n),
                                  f"eval {i}/{n} best={best}")
        except Exception:  # noqa: BLE001 - progress is best-effort
            pass

    res = await anyio.to_thread.run_sync(lambda: uc.call_optimize(req, progress))
    rj = res.model_dump(mode="json")
    return {"objective": rj["objective"], "n_evals": rj["n_evals"], "converged": rj["converged"],
            "message": rj["message"], "design": rj["design"], "digest": uc.digest(res.results),
            "results": uc.strip_heavy(rj["results"]), "history": rj["history"][-50:]}


@tool(description="Grid sweep: evaluate every combination of `variables` = {dotted.path: [values]} or "
                      "{dotted.path: \"lo:hi:n\"} (linear) / \"lo:hi:n:log\". Returns per point the values, "
                      "summary, key harmonic numbers, max utilisation (or error). Optionally writes full "
                      "JSON lines to `out_path`. max_points caps the grid size.")
async def sweep(design: DesignArg, variables: dict[str, Any], what: list[str] | None = None,
                max_points: int = 400, workers: int = 1, out_path: str | None = None,
                ctx: Context = None) -> dict:
    d = _design(design)
    vars_ = []
    for p, v in variables.items():
        vars_.append(uc.parse_sweep_var(f"{p}={v}") if isinstance(v, str) else (p, list(v)))
    n = 1
    for _, v in vars_:
        n *= len(v)
    if n > max_points:
        raise ValueError(f"grid has {n} points > max_points={max_points}")

    def run():
        rows, fh = [], (open(out_path, "w") if out_path else None)
        try:
            for rec in uc.run_sweep(d, vars_, what, workers=workers):
                if fh:
                    fh.write(json.dumps(uc.to_jsonable(rec)) + "\n")
                if "error" in rec:
                    rows.append({"values": rec["values"], "error": rec["error"]})
                    continue
                r = rec["results"]
                h = r.get("harmonic") or {}
                row = {"values": rec["values"], "summary": r.get("summary", {}),
                       "max_utilisation": max((c["utilisation"] for c in r.get("checks", [])), default=None)}
                for k in ("control_ratio", "hub_moment_mean", "hub_moment_phase_deg", "moment_per_torque"):
                    if k in h:
                        row[k] = h[k]
                if r.get("trim"):
                    row["rpm"] = r["trim"]["rpm"]
                rows.append(row)
                if ctx is not None:
                    try:
                        anyio.from_thread.run(ctx.report_progress, float(len(rows)), float(n), None)
                    except Exception:  # noqa: BLE001
                        pass
        finally:
            if fh:
                fh.close()
        return rows

    rows = await anyio.to_thread.run_sync(run)
    return {"n": n, "variables": {p: v for p, v in vars_}, "rows": rows,
            **({"out_path": str(Path(out_path).resolve())} if out_path else {})}


@tool(description="Export geometry of a design: fmt glb|stl|step|json (json = surface mesh parts with "
                      "vertices in metres, frame H). Writes to `path` (default: a temp file) and returns "
                      "the absolute path. " + DESIGN_DOC)
async def export_geometry(design: DesignArg, fmt: Literal["glb", "stl", "step", "json", "stl_parts", "3mf"] = "step",
                          path: str | None = None) -> dict:
    d = _design(design)
    if not path:
        path = str(Path(tempfile.mkdtemp(prefix="flexblade-")) / f"{d.name}.{fmt}")
    p = await anyio.to_thread.run_sync(lambda: api.export_geometry(d, fmt, str(Path(path).resolve())))
    return {"path": str(p or path), "fmt": fmt, "design_hash": d.hash()}


# --------------------------------------------------------------------------- resources


@mcp.resource("flexblade://design", name="DESIGN.md", mime_type="text/markdown",
              description="flexblade design contract: goals, mechanism, units/frames, module interfaces.")
def design_doc() -> str:
    return DESIGN_MD.read_text() if DESIGN_MD.exists() else "DESIGN.md not found"


@mcp.resource("flexblade://schema/design", name="Design schema", mime_type="application/json",
              description="JSON schema of Design")
def design_schema() -> str:
    return json.dumps(api.schema()["Design"])


def main() -> None:
    mcp.run("stdio")


if __name__ == "__main__":
    main()
