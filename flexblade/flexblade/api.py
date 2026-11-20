"""Public facade. CLI, MCP server, GUI and Keel bridge use only these functions.

Stubs raise NotImplementedError until modules land; see DESIGN.md.
"""
from __future__ import annotations

import time
from typing import Any

from .schema import Design, OptimizeRequest, OptimizeResult, Results


def _as_design(d: Design | dict) -> Design:
    return d if isinstance(d, Design) else Design.model_validate(d)


def evaluate(design: Design | dict, what: list[str] | None = None) -> Results:
    design = _as_design(design)
    what = what or ["trim", "modes", "harmonic", "checks"]
    t0 = time.perf_counter()
    if design.layout == "coaxial":
        from . import coaxial
        res = coaxial.evaluate(design, what)
    else:
        from . import rotor  # noqa: WPS433
        res = rotor.evaluate(design, what)
    res.timings_s.setdefault("total", time.perf_counter() - t0)
    return res


def optimize(request: OptimizeRequest | dict) -> OptimizeResult:
    req = request if isinstance(request, OptimizeRequest) else OptimizeRequest.model_validate(request)
    from . import optimize as opt
    return opt.run(req)


def export_geometry(design: Design | dict, fmt: str, path: str) -> str:
    from . import geometry
    return geometry.export(_as_design(design), fmt, path)


def presets() -> dict[str, Design]:
    from . import presets as p
    return p.all_presets()


def schema() -> dict[str, Any]:
    return {"Design": Design.model_json_schema(), "Results": Results.model_json_schema(),
            "OptimizeRequest": OptimizeRequest.model_json_schema()}


def materials() -> list[dict]:
    from .materials import list_materials
    return list_materials()
