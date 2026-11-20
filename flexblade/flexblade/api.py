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


def material_sweep(design: Design | dict, slot: str, materials: str | list[str] = "all",
                   resize: bool = False, torque: str | float = "design", hosts: str | None = None,
                   workers: int | None = None, progress=None) -> list[dict]:
    """Evaluate `design` once per material in layup `slot` (body|blade|flexure|skin|insert).
    materials: 'all' | 'filaments' | 'rigid' | 'inserts' | 'class:tpu' | 'brand:Bambu Lab'
    | comma list. resize=True rescales geometry to the original section stiffnesses
    (resize.py). torque: 'design' | 'capability' | number (op.torque_mod_frac).
    Returns one flat metrics record per material (see matsweep.metrics)."""
    from .matsweep import material_sweep as ms
    return ms(_as_design(design), slot, materials, resize=resize, torque=torque, hosts=hosts,
              workers=workers, progress=progress)


def resize_for_material(design: Design | dict, slot: str, material: str,
                        t_min: float = 0.4e-3) -> tuple[Design, dict[str, Any]]:
    """Switch `slot` to `material` and resize thickness / width / t/c / skin / rod
    diameter so each element keeps its original soft bending stiffness. Returns
    (design, report)."""
    from .resize import resize_for_material as rf
    return rf(_as_design(design), slot, material, t_min=t_min)


def section_targets(design: Design | dict) -> dict[str, Any]:
    """Section stiffnesses (EA, EI_soft, EI_stiff, GJ, mass/len) of flexure members and
    blade stations — the stiffness targets of the material-agnostic mode."""
    from .resize import section_targets as st
    return st(_as_design(design))


def material_slots(design: Design | dict) -> dict[str, list[str]]:
    """{slot: [materials]} for the layup slots this design uses."""
    from .matsweep import used_slots
    return used_slots(_as_design(design))
