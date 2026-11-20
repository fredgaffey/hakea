"""Helpers shared by the CLI, MCP server, GUI and Keel bridge (OWNER: ui).

Only uses the public facade (`flexblade.api`) and the schema. Nothing here does physics.
"""
from __future__ import annotations

import copy
import itertools
import json
import math
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Iterable

from . import api
from .schema import Design, Results

# --------------------------------------------------------------------------- JSON io


def read_json(src: str | None) -> Any:
    """Read JSON from a path, '-' / None (stdin) or an inline JSON string starting with '{' or '['."""
    if src is None or src == "-":
        text = sys.stdin.read()
    elif src.lstrip().startswith(("{", "[")):
        text = src
    else:
        text = Path(src).read_text()
    return json.loads(text)


def to_jsonable(obj: Any) -> Any:
    if hasattr(obj, "model_dump"):
        return obj.model_dump(mode="json")
    if isinstance(obj, dict):
        return {k: to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_jsonable(v) for v in obj]
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    try:  # numpy scalars / arrays
        import numpy as np
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.generic):
            return obj.item()
    except ImportError:  # pragma: no cover
        pass
    return obj


# --------------------------------------------------------------------------- presets / designs


def presets_safe() -> tuple[dict[str, Design], str | None]:
    """(presets, error). Presets may not exist yet while physics modules land."""
    try:
        return dict(api.presets()), None
    except Exception as e:  # noqa: BLE001
        return {}, f"{type(e).__name__}: {e}"


def resolve_design(obj: Any) -> Design:
    """Accept a Design, a Design dict, a preset name, or {"preset": name, "set": {path: value}}."""
    if isinstance(obj, Design):
        return obj
    if isinstance(obj, str):
        obj = {"preset": obj}
    if isinstance(obj, dict) and "preset" in obj and set(obj) <= {"preset", "set"}:
        pres, err = presets_safe()
        name = obj["preset"]
        if name not in pres:
            raise KeyError(f"unknown preset {name!r}; known: {sorted(pres)}" + (f" ({err})" if err else ""))
        d = pres[name]
        if obj.get("set"):
            d = set_values(d, obj["set"])
        return d
    return Design.model_validate(obj)


# --------------------------------------------------------------------------- dotted paths


def _split(path: str) -> list[str | int]:
    out: list[str | int] = []
    for p in path.split("."):
        if p == "":
            raise ValueError(f"bad path {path!r}")
        out.append(int(p) if p.lstrip("-").isdigit() else p)
    return out


def get_path(obj: Any, path: str) -> Any:
    cur = obj.model_dump(mode="json") if hasattr(obj, "model_dump") else obj
    for k in _split(path):
        if isinstance(cur, list):
            cur = cur[int(k)]
        elif isinstance(cur, dict):
            if k not in cur:
                raise KeyError(f"{path!r}: no key {k!r} (have {sorted(cur)})")
            cur = cur[k]
        else:
            raise KeyError(f"{path!r}: cannot index into {type(cur).__name__} at {k!r}")
    return cur


def parse_value(text: Any) -> Any:
    """CLI value: JSON literal if it parses (numbers, true, null, lists, objects), else string."""
    if not isinstance(text, str):
        return text
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return text


def set_values(design: Design | dict, values: dict[str, Any]) -> Design:
    """Return a new validated Design with dotted-path values set.

    Paths address the Design JSON, e.g. "flexure.thickness", "blade.stations.3.chord",
    "blade.layup.material". Unknown keys are rejected by validation (extra="forbid").
    """
    d = copy.deepcopy(design.model_dump(mode="json") if isinstance(design, Design) else design)
    for path, val in values.items():
        keys = _split(path)
        cur = d
        for k in keys[:-1]:
            if isinstance(cur, list):
                cur = cur[int(k)]
            else:
                if cur.get(k) is None:
                    cur[k] = {}
                cur = cur[k]
        last = keys[-1]
        if isinstance(cur, list):
            cur[int(last)] = val
        else:
            cur[last] = val
    return Design.model_validate(d)


def parse_assignments(items: Iterable[str]) -> dict[str, Any]:
    out = {}
    for it in items:
        if "=" not in it:
            raise ValueError(f"expected path=value, got {it!r}")
        k, v = it.split("=", 1)
        out[k.strip()] = parse_value(v.strip())
    return out


# --------------------------------------------------------------------------- sweeps


def parse_sweep_var(spec: str) -> tuple[str, list[Any]]:
    """'path=lo:hi:n' (linear), 'path=lo:hi:n:log' (log) or 'path=v1,v2,...'."""
    if "=" not in spec:
        raise ValueError(f"sweep var must be path=lo:hi:n or path=v1,v2,..; got {spec!r}")
    path, rng = spec.split("=", 1)
    path = path.strip()
    if ":" in rng:
        parts = rng.split(":")
        if len(parts) not in (3, 4):
            raise ValueError(f"bad range {rng!r}")
        lo, hi, n = float(parts[0]), float(parts[1]), int(parts[2])
        log = len(parts) == 4 and parts[3] == "log"
        if n < 1:
            raise ValueError("n must be >= 1")
        if n == 1:
            return path, [lo]
        if log:
            return path, [lo * (hi / lo) ** (i / (n - 1)) for i in range(n)]
        return path, [lo + (hi - lo) * i / (n - 1) for i in range(n)]
    return path, [parse_value(v.strip()) for v in rng.split(",")]


def sweep_grid(variables: list[tuple[str, list[Any]]]) -> list[dict[str, Any]]:
    paths = [p for p, _ in variables]
    return [dict(zip(paths, combo)) for combo in itertools.product(*[v for _, v in variables])]


def _sweep_one(args):
    design_json, values, what, full = args
    try:
        d = set_values(design_json, values)
        r = api.evaluate(d, what).model_dump(mode="json")
        return {"values": values, "design_hash": d.hash(),
                "results": r if full else strip_heavy(r)}
    except Exception as e:  # noqa: BLE001
        return {"values": values, "error": f"{type(e).__name__}: {e}"}


def run_sweep(design: Design, variables: list[tuple[str, list[Any]]], what: list[str] | None = None,
              workers: int = 1, full: bool = False, progress=None):
    """Yield one record per grid point (in grid order)."""
    grid = sweep_grid(variables)
    dj = design.model_dump(mode="json")
    jobs = [(dj, v, what, full) for v in grid]
    if workers <= 1 or len(jobs) <= 1:
        for i, j in enumerate(jobs):
            rec = _sweep_one(j)
            rec["index"] = i
            if progress:
                progress(i + 1, len(jobs))
            yield rec
        return
    import multiprocessing as mp
    os.environ["FLEXBLADE_WORKER"] = "1"
    with mp.get_context("spawn" if os.environ.get("FLEXBLADE_SPAWN") else None).Pool(workers) as pool:
        for i, rec in enumerate(pool.imap(_sweep_one, jobs)):
            rec["index"] = i
            if progress:
                progress(i + 1, len(jobs))
            yield rec


# --------------------------------------------------------------------------- results digest

HEAVY = ("nodes", "displacement_re", "displacement_im")


def strip_heavy(r: dict) -> dict:
    """Drop nodal arrays and mode shapes (large; only useful for animation)."""
    r = dict(r)
    if r.get("harmonic"):
        r["harmonic"] = {k: v for k, v in r["harmonic"].items() if k not in HEAVY}
    if r.get("modes"):
        r["modes"] = [{k: v for k, v in m.items() if k != "shape"} for m in r["modes"]]
    if r.get("coaxial"):
        c = dict(r["coaxial"])
        for k in ("upper", "lower"):
            if c.get(k):
                c[k] = strip_heavy(c[k])
        r["coaxial"] = c
    return r


def _f(x: Any, nd: int = 4) -> str:
    if x is None:
        return "-"
    if isinstance(x, (int,)) and not isinstance(x, bool):
        return str(x)
    try:
        x = float(x)
    except (TypeError, ValueError):
        return str(x)
    if x == 0:
        return "0"
    if abs(x) >= 1e5 or abs(x) < 1e-3:
        return f"{x:.{nd - 1}e}"
    return f"{x:.{nd}g}"


def digest(results: Results | dict, max_checks: int = 12) -> str:
    """Compact, LLM-friendly text summary of a Results object (coaxial: totals + each rotor)."""
    r = results.model_dump(mode="json") if hasattr(results, "model_dump") else results
    c = r.get("coaxial")
    if c:
        L = [f"design_hash={r.get('design_hash')} model_version={r.get('model_version')} layout=coaxial",
             "COAXIAL TOTALS: thrust={} N power={} W FM={} rpm upper/lower={}/{} thrust_share_upper={} "
             "yaw_torque_net={} N·m yaw_authority={} N·m control_moment_max={} N·m control_ratio={} "
             "2P_combined={} N·m interference_power_factor={}".format(
                 _f(c["thrust_total"]), _f(c["power_total"]), _f(c["figure_of_merit"]), _f(c["rpm_upper"]),
                 _f(c["rpm_lower"]), _f(c["thrust_share_upper"]), _f(c["yaw_torque_net"]),
                 _f(c["yaw_authority"]), _f(c["control_moment_max"]), _f(c["control_ratio"]),
                 _f(c["moment_2p_combined"]), _f(c["interference_power_factor"]))]
        if r.get("summary"):
            L.append("SUMMARY: " + ", ".join(f"{k}={_f(v)}" for k, v in r["summary"].items()))
        for k in ("upper", "lower"):
            if c.get(k):
                sub = _digest_single(c[k], max_checks=6).splitlines()[1:]
                L.append(f"--- {k.upper()} ROTOR ---")
                L += [ln for ln in sub if not ln.startswith(("timings_s", "SUMMARY"))]
        seen = set(L)
        L += [f"WARNING: {w}" for w in r.get("warnings") or [] if f"WARNING: {w}" not in seen]
        return "\n".join(L)
    return _digest_single(r, max_checks)


def _digest_single(r: dict, max_checks: int = 12) -> str:
    L: list[str] = [f"design_hash={r.get('design_hash')} model_version={r.get('model_version')}"]
    t = r.get("trim")
    if t:
        L.append("TRIM: rpm={} thrust={} N torque={} N·m power={} W FM={} collective={}° "
                 "CF_root={} N coning={}° root_pitch_change={}° stretch={} m".format(
                     _f(t["rpm"]), _f(t["thrust"]), _f(t["torque"]), _f(t["power"]),
                     _f(t["figure_of_merit"]), _f(t["collective_deg"]),
                     _f(t["centrifugal_force_root"]), _f(t.get("coning_deg")),
                     _f(t.get("root_pitch_change_deg")), _f(t.get("radial_stretch"))))
    h = r.get("harmonic")
    if h:
        L.append("HARMONIC 1/rev: torque_amp={} N·m hub_speed_amp={} rpm hub_moment_mean={} N·m "
                 "phase={}° 2P_ripple={} N·m thrust_1P={} N moment/torque={} control_ratio={} "
                 "tilt_equiv={}°".format(
                     _f(h["torque_amp"]), _f(h["hub_speed_amp_rpm"]), _f(h["hub_moment_mean"]),
                     _f(h["hub_moment_phase_deg"]), _f(h["hub_moment_2p"]), _f(h["thrust_1p"]),
                     _f(h["moment_per_torque"]), _f(h["control_ratio"]), _f(h["tilt_equiv_deg"])))
        for b in h.get("blades", []):
            L.append("  blade {}: pitch {}°∠{}° lag {}°∠{}° flap {}°∠{}°".format(
                b["blade"], _f(b["pitch_amp_deg"], 3), _f(b["pitch_phase_deg"], 3),
                _f(b["lag_amp_deg"], 3), _f(b["lag_phase_deg"], 3),
                _f(b["flap_amp_deg"], 3), _f(b["flap_phase_deg"], 3)))
    modes = r.get("modes") or []
    if modes:
        rpm = (t or {}).get("rpm")
        per_rev = (lambda f: f" ({_f(f / (rpm / 60), 3)}/rev)") if rpm else (lambda f: "")
        L.append("MODES: " + "; ".join(
            f"{m['label']} {_f(m['freq_hz'])} Hz{per_rev(m['freq_hz'])} ζ={_f(m['damping_ratio'], 2)}"
            for m in modes[:10]))
    if r.get("summary"):
        L.append("SUMMARY: " + ", ".join(f"{k}={_f(v)}" for k, v in r["summary"].items()))
    checks = sorted(r.get("checks") or [], key=lambda c: -c["utilisation"])
    if checks:
        fails = [c for c in checks if c["utilisation"] > 1]
        L.append(f"CHECKS: {len(checks)} total, {len(fails)} FAIL (utilisation>1), "
                 f"max utilisation {_f(checks[0]['utilisation'], 3)} ({checks[0]['name']} @ {checks[0]['member']})")
        for c in checks[:max_checks]:
            flag = "FAIL" if c["utilisation"] > 1 else ("near" if c["utilisation"] > 0.8 else "ok")
            L.append(f"  [{flag}] {c['name']} @ {c['member']}: {_f(c['value'])} / {_f(c['allowable'])}"
                     f" = {_f(c['utilisation'], 3)}" + (f" ({c['note']})" if c.get("note") else ""))
    for w in r.get("warnings") or []:
        L.append(f"WARNING: {w}")
    if r.get("timings_s"):
        L.append("timings_s: " + ", ".join(f"{k}={_f(v, 3)}" for k, v in r["timings_s"].items()))
    return "\n".join(L)


# --------------------------------------------------------------------------- geometry

MATERIAL_COLOURS = {
    "tpu_95a": "#e9b44c", "tpu_64d": "#d98f2b", "pa6_cf": "#3a3f47", "petg_cf": "#4a515b",
    "pla": "#9aa6b2", "cf_rod_pultruded": "#1c1f24", "steel_302_spring": "#b8c2cc",
}


def load_geometry(design: Design) -> dict:
    """Surface mesh as {"units","frame","parts":[{name, role, blade, material, color, vertices, faces}]}.

    Uses export_geometry(fmt="json"); if that file has no parts with vertices/faces, falls back
    to GLB via trimesh (one part per scene geometry).
    """
    with tempfile.TemporaryDirectory(prefix="flexblade-geo-") as td:
        p = Path(td) / "geom.json"
        err = None
        try:
            out = api.export_geometry(design, "json", str(p))
            g = json.loads(Path(out or p).read_text())
            if _geometry_ok(g):
                return _normalise_geometry(g)
        except Exception as e:  # noqa: BLE001
            err = e
        try:
            import trimesh
            gp = Path(td) / "geom.glb"
            out = api.export_geometry(design, "glb", str(gp))
            scene = trimesh.load(out or gp, force="scene")
            parts = []
            for node in scene.graph.nodes_geometry:
                T, gname = scene.graph[node]
                m = scene.geometry[gname].copy()
                m.apply_transform(T)
                parts.append(dict(name=str(node), vertices=m.vertices.tolist(), faces=m.faces.tolist()))
            return _normalise_geometry({"units": "m", "frame": "H", "parts": parts})
        except Exception as e2:  # noqa: BLE001
            raise RuntimeError(f"geometry unavailable: json export: {err!r}; glb export: {e2!r}") from e2


def _geometry_ok(g: Any) -> bool:
    return isinstance(g, dict) and isinstance(g.get("parts"), list) and all(
        "vertices" in p and "faces" in p for p in g["parts"]) and len(g["parts"]) > 0


def _normalise_geometry(g: dict) -> dict:
    g = dict(g)
    g.setdefault("units", "m")
    g.setdefault("frame", "H")
    parts = []
    for i, p in enumerate(g["parts"]):
        p = dict(p)
        p.setdefault("name", f"part{i}")
        name = p["name"].lower()
        if "role" not in p:
            p["role"] = ("hub" if "hub" in name else "flexure" if ("flex" in name or "member" in name)
                         else "blade" if "blade" in name else "other")
        if "blade" not in p or p["blade"] is None:
            digits = "".join(ch for ch in name.split("blade")[-1][:2] if ch.isdigit()) if "blade" in name else ""
            p["blade"] = int(digits) if digits else None
        if "rotor" not in p:
            p["rotor"] = "upper" if name.startswith("upper") else "lower" if name.startswith("lower") else None
        if not p.get("color"):
            p["color"] = MATERIAL_COLOURS.get(p.get("material", ""), "#8892a0")
        parts.append(p)
    g["parts"] = parts
    return g


# --------------------------------------------------------------------------- misc


def numeric_leaves(d: Any, prefix: str = "") -> dict[str, float]:
    """Flatten numeric (non-bool) leaves of a JSON tree into dotted paths."""
    out: dict[str, float] = {}
    if isinstance(d, dict):
        for k, v in d.items():
            out.update(numeric_leaves(v, f"{prefix}{k}."))
    elif isinstance(d, list):
        for i, v in enumerate(d):
            out.update(numeric_leaves(v, f"{prefix}{i}."))
    elif isinstance(d, (int, float)) and not isinstance(d, bool):
        out[prefix[:-1]] = d
    return out


def diff(a: Any, b: Any, prefix: str = "") -> dict[str, tuple[Any, Any]]:
    """Leaf-level differences between two JSON trees: {path: (a, b)}."""
    out: dict[str, tuple[Any, Any]] = {}
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            out.update(diff(a.get(k), b.get(k), f"{prefix}{k}."))
    elif isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        for i, (x, y) in enumerate(zip(a, b)):
            out.update(diff(x, y, f"{prefix}{i}."))
    elif a != b:
        out[prefix[:-1]] = (a, b)
    return out


def call_optimize(request, progress=None, hosts=None):
    """api.optimize, passing a progress(i, n, objective, best) callback if the backend accepts one."""
    if hosts:
        return api.optimize(request, progress=progress, hosts=hosts)
    import inspect
    fn = api.optimize
    try:
        params = inspect.signature(fn).parameters
    except (TypeError, ValueError):  # pragma: no cover
        params = {}
    if progress is not None and ("progress" in params or any(
            p.kind == p.VAR_KEYWORD for p in params.values())):
        return fn(request, progress=progress)
    return fn(request)
