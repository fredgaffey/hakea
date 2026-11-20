"""`flexblade` command line (OWNER: ui). JSON in, JSON out; built only on flexblade.api.

Every subcommand prints exactly one JSON document to stdout (except `sweep` without --out,
which prints JSON lines, and `gui`/`mcp`, which are servers). Errors print {"error": ...}
and exit nonzero. Anything the physics modules print is diverted to stderr.

Design arguments accept: a file path, '-' (stdin), inline JSON, or 'preset:NAME'.
"""
from __future__ import annotations

import argparse
import contextlib
import json
import os
import sys
from pathlib import Path
from typing import Any

from . import api, uicommon as uc
from .schema import Design, OptimizeRequest

EXIT_ERROR = 1
EXIT_USAGE = 2


class CliError(Exception):
    pass


# --------------------------------------------------------------------------- output plumbing


class _Out:
    """Writes JSON to the real stdout while fd 1 is diverted to stderr."""

    def __init__(self, pretty: bool):
        self.pretty = pretty
        self._fd = None
        self._file = None

    def __enter__(self):
        sys.stdout.flush()
        try:
            self._fd = os.dup(1)
            os.dup2(2, 1)
            self._file = os.fdopen(self._fd, "w", encoding="utf-8", closefd=True)
        except OSError:  # no real fd (e.g. captured in tests): write to sys.stdout
            self._fd = None
            self._file = None
        self._saved_sys = sys.stdout
        if self._file is not None:
            sys.stdout = sys.stderr
        return self

    def _target(self):
        return self._file if self._file is not None else self._saved_sys

    def emit(self, obj: Any) -> None:
        f = self._target()
        f.write(json.dumps(uc.to_jsonable(obj), indent=2 if self.pretty else None,
                           allow_nan=False, default=str))
        f.write("\n")
        f.flush()

    def line(self, obj: Any) -> None:
        f = self._target()
        f.write(json.dumps(uc.to_jsonable(obj), allow_nan=False, default=str) + "\n")
        f.flush()

    def __exit__(self, *exc):
        if self._file is not None:
            sys.stdout.flush()
            os.dup2(self._fd, 1)
            sys.stdout = self._saved_sys
            self._file.close()
        return False


# --------------------------------------------------------------------------- helpers


def _design_arg(src: str | None) -> Design:
    if src and src.startswith("preset:"):
        return uc.resolve_design({"preset": src.split(":", 1)[1]})
    return uc.resolve_design(uc.read_json(src))


def _maybe_fake() -> None:
    """FLEXBLADE_FAKE=1 (all) or =missing: dev/demo hook using tests/fake_backend.py."""
    mode = os.environ.get("FLEXBLADE_FAKE")
    if not mode or mode == "0":
        return
    import importlib.util
    p = Path(__file__).resolve().parents[1] / "tests" / "fake_backend.py"
    if not p.exists():
        raise CliError(f"FLEXBLADE_FAKE set but {p} not found")
    spec = importlib.util.spec_from_file_location("flexblade_fake_backend", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    patched = mod.install(only_missing=(mode == "missing"))
    print(f"[flexblade] FAKE backend active for: {', '.join(patched) or 'nothing'}", file=sys.stderr)


def _what(s: str | None) -> list[str] | None:
    return [w.strip() for w in s.split(",") if w.strip()] if s else None


# --------------------------------------------------------------------------- commands


def cmd_schema(a, out):
    s = api.schema()
    out.emit(s[a.which] if a.which else s)


def cmd_presets(a, out):
    pres, err = uc.presets_safe()
    if err and not pres:
        raise CliError(f"presets unavailable: {err}")
    if a.name:
        if a.name not in pres:
            raise CliError(f"unknown preset {a.name!r}; known: {sorted(pres)}")
        out.emit(pres[a.name])
        return
    out.emit({"presets": [
        {"name": k, "diameter_in": d.blade.diameter_in, "topology": d.flexure.topology,
         "flexure_material": d.flexure.layup.material, "blade_material": d.blade.layup.material,
         "notes": d.notes} for k, d in pres.items()]})


def cmd_materials(a, out):
    out.emit({"materials": api.materials()})


def cmd_validate(a, out):
    d = _design_arg(a.design)
    out.emit({"ok": True, "design_hash": d.hash(), "name": d.name})


def cmd_set(a, out):
    d = _design_arg(a.design)
    d = uc.set_values(d, uc.parse_assignments(a.assign))
    out.emit(d)


def cmd_get(a, out):
    d = _design_arg(a.design)
    out.emit({p: uc.get_path(d, p) for p in a.paths})


def cmd_diff(a, out):
    x, y = _design_arg(a.a), _design_arg(a.b)
    out.emit({k: {"a": v[0], "b": v[1]} for k, v in
              uc.diff(x.model_dump(mode="json"), y.model_dump(mode="json")).items()})


def cmd_evaluate(a, out):
    d = _design_arg(a.design)
    if a.set:
        d = uc.set_values(d, uc.parse_assignments(a.set))
    r = api.evaluate(d, _what(a.what))
    if a.digest:
        out.emit({"design_hash": r.design_hash, "digest": uc.digest(r)})
        return
    rj = r.model_dump(mode="json")
    out.emit(uc.strip_heavy(rj) if a.compact else rj)


def cmd_optimize(a, out):
    req = OptimizeRequest.model_validate(uc.read_json(a.request))

    def progress(i, n, obj=None, best=None):
        if a.progress:
            print(json.dumps({"progress": i, "of": n, "objective": obj, "best": best}), file=sys.stderr)

    res = uc.call_optimize(req, progress, hosts=a.hosts)
    rj = res.model_dump(mode="json")
    if a.compact:
        rj["results"] = uc.strip_heavy(rj["results"])
    out.emit(rj)


def cmd_export(a, out):
    d = _design_arg(a.design)
    path = a.output or f"{d.name}.{a.fmt}"
    p = api.export_geometry(d, a.fmt, str(Path(path).resolve()))
    out.emit({"path": str(p or path), "fmt": a.fmt, "design_hash": d.hash()})


def cmd_sweep(a, out):
    d = _design_arg(a.design)
    if not a.var:
        raise CliError("sweep needs at least one --var path=lo:hi:n")
    variables = [uc.parse_sweep_var(v) for v in a.var]
    n_total = 1
    for _, vals in variables:
        n_total *= len(vals)
    if n_total > a.max_points:
        raise CliError(f"grid has {n_total} points > --max-points {a.max_points}")
    for p, vals in variables:  # validate paths early (fails fast on typos)
        uc.set_values(d, {p: vals[0]})
    n_err = 0
    fh = open(a.out, "w") if a.out else None
    try:
        if a.hosts:
            from . import remote
            grid = uc.sweep_grid(variables)
            designs = [uc.set_values(d, v) for v in grid]

            def _remote_records():
                with remote.HostPool(a.hosts, what=_what(a.what), full=a.full) as pool:
                    for i, r in pool.imap(designs):
                        rec = {"values": grid[i], "design_hash": designs[i].hash(), "index": i,
                               "host": r.get("host")}
                        rec.update({"results": r["results"]} if "results" in r
                                   else {"error": r.get("error")})
                        yield rec
            records = _remote_records()
        else:
            records = uc.run_sweep(d, variables, _what(a.what), workers=a.workers, full=a.full)
        for rec in records:
            n_err += "error" in rec
            if fh:
                fh.write(json.dumps(uc.to_jsonable(rec), default=str) + "\n")
                fh.flush()
            else:
                out.line(rec)
            if a.progress:
                print(json.dumps({"progress": rec["index"] + 1, "of": n_total}), file=sys.stderr)
    finally:
        if fh:
            fh.close()
    if fh:
        out.emit({"out": str(Path(a.out).resolve()), "n": n_total, "errors": n_err,
                  "variables": {p: v for p, v in variables}})


def cmd_digest(a, out):
    r = uc.read_json(a.results)
    out.emit({"digest": uc.digest(r)})


def cmd_gui(a, out):
    from .gui.server import serve
    serve(host=a.host, port=a.port, cors_origins=a.cors_origin, open_browser=a.open)


def cmd_mcp(a, out):
    from .mcp_server import main as mcp_main
    mcp_main()


# --------------------------------------------------------------------------- parser


class _Parser(argparse.ArgumentParser):
    def error(self, message):  # JSON usage errors on stdout, exit 2
        sys.stdout.write(json.dumps({"error": message, "type": "usage", "usage": self.format_usage().strip()}) + "\n")
        sys.exit(EXIT_USAGE)


def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--pretty", action="store_true", help="indent JSON output")
    common.add_argument("--json", action="store_true", help="machine-readable JSON (default)")

    p = _Parser(prog="flexblade", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
                parents=[common])
    sub = p.add_subparsers(dest="cmd", required=True, parser_class=_Parser)
    D = "design JSON: path, '-' for stdin, inline JSON, or preset:NAME"

    s = sub.add_parser("schema", parents=[common], help="JSON schemas of Design/Results/OptimizeRequest")
    s.add_argument("--which", choices=["Design", "Results", "OptimizeRequest"])
    s.set_defaults(fn=cmd_schema)

    s = sub.add_parser("presets", parents=[common], help="list presets, or print one as Design JSON")
    s.add_argument("name", nargs="?")
    s.set_defaults(fn=cmd_presets)

    s = sub.add_parser("materials", parents=[common], help="material library")
    s.set_defaults(fn=cmd_materials)

    s = sub.add_parser("validate", parents=[common], help="validate a design")
    s.add_argument("design", nargs="?", default="-", help=D)
    s.set_defaults(fn=cmd_validate)

    s = sub.add_parser("set", parents=[common], help="edit a design via dotted paths; prints new Design")
    s.add_argument("design", help=D)
    s.add_argument("assign", nargs="+", help="path=value (value parsed as JSON if possible)")
    s.set_defaults(fn=cmd_set)

    s = sub.add_parser("get", parents=[common], help="read dotted paths from a design")
    s.add_argument("design", help=D)
    s.add_argument("paths", nargs="+")
    s.set_defaults(fn=cmd_get)

    s = sub.add_parser("diff", parents=[common], help="leaf differences between two designs")
    s.add_argument("a", help=D)
    s.add_argument("b", help=D)
    s.set_defaults(fn=cmd_diff)

    s = sub.add_parser("evaluate", parents=[common], help="run analysis; prints Results JSON")
    s.add_argument("design", nargs="?", default="-", help=D)
    s.add_argument("--what", help="comma list: trim,modes,harmonic,checks (default all)")
    s.add_argument("--set", action="append", default=[], metavar="PATH=VALUE", help="override (repeatable)")
    s.add_argument("--compact", action="store_true", help="drop nodal arrays and mode shapes")
    s.add_argument("--digest", action="store_true", help="print a short text digest instead")
    s.set_defaults(fn=cmd_evaluate)

    s = sub.add_parser("optimize", parents=[common], help="run OptimizeRequest; prints OptimizeResult")
    s.add_argument("request", nargs="?", default="-")
    s.add_argument("--compact", action="store_true")
    s.add_argument("--progress", action="store_true", help="JSON progress lines on stderr")
    s.add_argument("--hosts", help="evaluate DE populations over machines, e.g. local:28,mac:10 "
                                   "(ssh alias:slots; remote code rsynced to /tmp/flexblade-work)")
    s.set_defaults(fn=cmd_optimize)

    s = sub.add_parser("export", parents=[common], help="export geometry")
    s.add_argument("design", nargs="?", default="-", help=D)
    s.add_argument("--fmt", choices=["glb", "stl", "step", "json", "stl_parts", "3mf"], default="step")
    s.add_argument("-o", "--output", help="output path (default <name>.<fmt>)")
    s.set_defaults(fn=cmd_export)

    s = sub.add_parser("sweep", parents=[common], help="grid sweep; JSON lines per point")
    s.add_argument("design", help=D)
    s.add_argument("--var", action="append", default=[],
                   help="path=lo:hi:n | path=lo:hi:n:log | path=v1,v2,... (repeatable; full grid)")
    s.add_argument("--out", help="results .jsonl (default: JSON lines to stdout)")
    s.add_argument("--what", help="comma list passed to evaluate")
    s.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    s.add_argument("--full", action="store_true", help="keep nodal arrays / mode shapes")
    s.add_argument("--max-points", type=int, default=5000)
    s.add_argument("--hosts", help="spread over machines, e.g. local:31,mac:10 (ssh alias:slots); "
                                   "remote code synced by rsync into /tmp/flexblade-work")
    s.add_argument("--progress", action="store_true")
    s.set_defaults(fn=cmd_sweep)

    s = sub.add_parser("digest", parents=[common], help="text digest of a Results JSON")
    s.add_argument("results", nargs="?", default="-")
    s.set_defaults(fn=cmd_digest)

    s = sub.add_parser("gui", parents=[common], help="local web GUI (127.0.0.1)")
    s.add_argument("--port", type=int, default=8765)
    s.add_argument("--host", default="127.0.0.1", help="bind address (keep 127.0.0.1)")
    s.add_argument("--cors-origin", action="append", default=None,
                   help="extra allowed origin for the Keel bridge (repeatable)")
    s.add_argument("--open", action="store_true", help="open a browser")
    s.set_defaults(fn=cmd_gui)

    s = sub.add_parser("mcp", parents=[common], help="MCP server on stdio")
    s.set_defaults(fn=cmd_mcp)
    return p


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    a = parser.parse_args(argv)
    if a.cmd in ("gui", "mcp"):
        _maybe_fake()
        a.fn(a, None)
        return 0
    with _Out(a.pretty) as out:
        try:
            _maybe_fake()
            a.fn(a, out)
            return 0
        except Exception as e:  # noqa: BLE001
            err = {"error": str(e) or type(e).__name__, "type": type(e).__name__}
            if hasattr(e, "errors") and callable(e.errors):  # pydantic ValidationError
                try:
                    err["details"] = [{"loc": ".".join(map(str, x["loc"])), "msg": x["msg"]} for x in e.errors()]
                except Exception:  # noqa: BLE001
                    pass
            if isinstance(e, (ModuleNotFoundError, NotImplementedError)):
                err["hint"] = "physics module not available yet (set FLEXBLADE_FAKE=1 for a demo backend)"
            out.emit(err)
            return EXIT_ERROR


if __name__ == "__main__":
    sys.exit(main())
