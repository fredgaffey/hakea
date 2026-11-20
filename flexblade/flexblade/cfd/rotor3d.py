"""3D single-blade periodic-sector hover case (SRFSimpleFoam, steady RANS).

Domain: annular sector of angle 2π/n centred on blade 1 (+x), rotationally cyclic sides,
r ∈ [r_axis, farfield_R·R], z ∈ [−below·R, above·R]. Background blockMesh (polar blocks,
≤ 45° each, arc edges) + snappyHexMesh onto the blade STL (blade-1 aerodynamic surface from
geometry; flexure/hub omitted). Whole domain in the single rotating frame (SRF, rpm about
+z), absolute far field at rest (SRFFreestreamVelocity UInf 0). Forces on the blade patch
via `forces` → hover thrust/torque ×n (post.hover_forces) for comparison with aero BEMT.
"""
from __future__ import annotations

import json
import math
import shutil
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from ..schema import Design
from . import foam
from .foam import RawEntry


@dataclass
class RotorCase:
    case_dir: str
    rpm: float
    n_blades: int
    R: float
    sector_deg: float
    nu: float
    bemt_thrust: Optional[float] = None
    bemt_torque: Optional[float] = None
    n_background_cells: int = 0
    surface_level: tuple = (3, 4)

    def save(self):
        Path(self.case_dir, "flexblade_case.json").write_text(json.dumps(asdict(self), indent=1))


def write_blade_stl(design: Design, path: Path) -> np.ndarray:
    """ASCII STL (m) of the blade-1 aerodynamic surface (outer skin), solid name 'blade'."""
    from ..geometry import build_surface
    surf = build_surface(design.model_copy(update=dict(layout="single")))
    parts = [p for p in surf["parts"] if p["blade"] == 1 and p["role"] in ("blade", "skin")]
    if not parts:
        raise ValueError("no blade surface")
    V, F = parts[0]["vertices"], parts[0]["faces"]
    lines = ["solid blade"]
    for f in F:
        a, b, c = V[f]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        n = n / nn if nn > 0 else n
        lines.append(f" facet normal {n[0]:.6e} {n[1]:.6e} {n[2]:.6e}\n  outer loop")
        for p in (a, b, c):
            lines.append(f"   vertex {p[0]:.9e} {p[1]:.9e} {p[2]:.9e}")
        lines.append("  endloop\n endfacet")
    lines.append("endsolid blade")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")
    return V


def _sector_blockmesh(case: Path, n_blades: int, r_in: float, r_out: float, z0: float,
                      z1: float, cell: float) -> int:
    sector = 2 * math.pi / n_blades
    nb = max(1, math.ceil(math.degrees(sector) / 45.0))
    th = np.linspace(-sector / 2, sector / 2, nb + 1)
    nr = max(4, int(round((r_out - r_in) / cell)))
    nz = max(4, int(round((z1 - z0) / cell)))
    verts, blocks, edges = [], [], []

    def vid(i, j, k):           # i: theta index, j: 0 inner/1 outer, k: 0 bottom/1 top
        return (i * 2 + j) * 2 + k
    for t in th:
        for r in (r_in, r_out):
            for z in (z0, z1):
                verts.append((r * math.cos(t), r * math.sin(t), z))
    cells = 0
    for i in range(nb):
        ra = 0.5 * (r_in + r_out)
        nt = max(4, int(round(ra * (th[i + 1] - th[i]) / cell)))
        # hex vertex order: (x1=r, x2=theta, x3=z) right-handed
        v = [vid(i, 0, 0), vid(i, 1, 0), vid(i + 1, 1, 0), vid(i + 1, 0, 0),
             vid(i, 0, 1), vid(i, 1, 1), vid(i + 1, 1, 1), vid(i + 1, 0, 1)]
        blocks.append(f"    hex ({' '.join(map(str, v))}) ({nr} {nt} {nz}) simpleGrading (1 1 1)")
        cells += nr * nt * nz
        tm = 0.5 * (th[i] + th[i + 1])
        for r, j in ((r_in, 0), (r_out, 1)):
            for z, k in ((z0, 0), (z1, 1)):
                edges.append(f"    arc {vid(i, j, k)} {vid(i + 1, j, k)} "
                             f"({r * math.cos(tm):.9g} {r * math.sin(tm):.9g} {z:.9g})")

    def faces(sel):
        return "\n".join(f"            ({' '.join(map(str, f))})" for f in sel)
    top = [(vid(i, 0, 1), vid(i, 1, 1), vid(i + 1, 1, 1), vid(i + 1, 0, 1)) for i in range(nb)]
    bot = [(vid(i, 0, 0), vid(i + 1, 0, 0), vid(i + 1, 1, 0), vid(i, 1, 0)) for i in range(nb)]
    outer = [(vid(i, 1, 0), vid(i + 1, 1, 0), vid(i + 1, 1, 1), vid(i, 1, 1)) for i in range(nb)]
    inner = [(vid(i, 0, 0), vid(i, 0, 1), vid(i + 1, 0, 1), vid(i + 1, 0, 0)) for i in range(nb)]
    lo = [(vid(0, 0, 0), vid(0, 1, 0), vid(0, 1, 1), vid(0, 0, 1))]
    hi = [(vid(nb, 0, 0), vid(nb, 0, 1), vid(nb, 1, 1), vid(nb, 1, 0))]
    ang = math.degrees(sector)
    body = "convertToMeters 1;\n\nvertices\n(\n" + "\n".join(
        f"    ({x:.9g} {y:.9g} {z:.9g})" for x, y, z in verts) + "\n);\n\nblocks\n(\n" + \
        "\n".join(blocks) + "\n);\n\nedges\n(\n" + "\n".join(edges) + "\n);\n\nboundary\n(\n" + f"""    inlet  {{ type patch; faces ( {faces(top)} ); }}
    outlet {{ type patch; faces ( {faces(bot)} ); }}
    farfield {{ type patch; faces ( {faces(outer)} ); }}
    axis {{ type patch; faces ( {faces(inner)} ); }}
    cyc_lo
    {{
        type cyclic; neighbourPatch cyc_hi; transform rotational;
        rotationAxis (0 0 1); rotationCentre (0 0 0); rotationAngle {ang:.9g};
        faces ( {faces(lo)} );
    }}
    cyc_hi
    {{
        type cyclic; neighbourPatch cyc_lo; transform rotational;
        rotationAxis (0 0 1); rotationCentre (0 0 0); rotationAngle {-ang:.9g};
        faces ( {faces(hi)} );
    }}
);
"""
    foam.write_dict(case, "system/blockMeshDict", body)
    return cells


def rotor_sector_case(design: Design, case_dir: str | Path, rpm: Optional[float] = None,
                      farfield_R: float = 2.5, above_R: float = 2.0, below_R: float = 4.0,
                      background_cells_per_R: int = 12, surface_level: tuple = (4, 5),
                      n_layers: int = 0, np_: int = 8, iterations: int = 1500,
                      overwrite: bool = True) -> RotorCase:
    """Write the hover sector case. rpm default = aero trim rpm (hover thrust)."""
    case = Path(case_dir)
    if case.exists() and overwrite:
        shutil.rmtree(case)
    case.mkdir(parents=True, exist_ok=True)
    d = design.model_copy(update=dict(layout="single"))
    R = d.blade.radius
    n = d.blade.n_blades
    nu = 1.5e-5
    Tb = Qb = None
    try:
        from ..aero import AeroModel, air_viscosity
        tr = AeroModel(d).trim(rpm=rpm) if rpm else AeroModel(d).trim()
        rpm = tr.rpm
        Tb, Qb = tr.T, tr.Q
        nu = air_viscosity(d.op.temperature_C) / d.op.rho
    except Exception as e:  # noqa: BLE001
        if rpm is None:
            raise ValueError(f"rpm required (aero trim failed: {e})") from e
    V = write_blade_stl(d, case / "constant/triSurface/blade.stl")
    r_in = 0.5 * float(np.linalg.norm(V[:, :2], axis=1).min())
    cell = R / background_cells_per_R
    ncell = _sector_blockmesh(case, n, r_in, farfield_R * R, -below_R * R, above_R * R, cell)
    bb_lo, bb_hi = V.min(0), V.max(0)
    pad = 0.15 * R
    foam.write_dict(case, "system/snappyHexMeshDict", f"""castellatedMesh true;
snap            true;
addLayers       {'true' if n_layers else 'false'};
geometry
{{
    blade.stl {{ type triSurfaceMesh; name blade; }}
    nearBlade
    {{
        type searchableBox;
        min ({bb_lo[0] - pad:.6g} {bb_lo[1] - pad:.6g} {bb_lo[2] - pad:.6g});
        max ({bb_hi[0] + pad:.6g} {bb_hi[1] + pad:.6g} {bb_hi[2] + pad:.6g});
    }}
    wake
    {{
        type searchableCylinder; point1 (0 0 {0.1 * R:.6g}); point2 (0 0 {-1.5 * R:.6g}); radius {1.1 * R:.6g};
    }}
}}
castellatedMeshControls
{{
    maxLocalCells 4000000; maxGlobalCells 8000000; minRefinementCells 10;
    maxLoadUnbalance 0.10; nCellsBetweenLevels 3;
    features ();
    refinementSurfaces {{ blade {{ level ({surface_level[0]} {surface_level[1]}); patchInfo {{ type wall; }} }} }}
    resolveFeatureAngle 30;
    refinementRegions
    {{
        nearBlade {{ mode inside; levels ((1e15 {max(surface_level[0] - 1, 1)})); }}
        wake {{ mode inside; levels ((1e15 2)); }}
    }}
    locationInMesh ({0.5 * R:.6g} {0.0:.6g} {0.9 * above_R * R:.6g});
    allowFreeStandingZoneFaces true;
}}
snapControls {{ nSmoothPatch 3; tolerance 2.0; nSolveIter 50; nRelaxIter 5; nFeatureSnapIter 10;
                implicitFeatureSnap true; explicitFeatureSnap false; multiRegionFeatureSnap false; }}
addLayersControls
{{
    relativeSizes true; layers {{ blade {{ nSurfaceLayers {n_layers}; }} }}
    expansionRatio 1.2; finalLayerThickness 0.4; minThickness 0.05; nGrow 0; featureAngle 120;
    nRelaxIter 3; nSmoothSurfaceNormals 1; nSmoothNormals 3; nSmoothThickness 10;
    maxFaceThicknessRatio 0.5; maxThicknessToMedialRatio 0.3; minMedialAxisAngle 90;
    nBufferCellsNoExtrude 0; nLayerIter 50;
}}
meshQualityControls {{ #includeEtc "caseDicts/meshQualityDict"
    nSmoothScale 4; errorReduction 0.75; relaxed {{ maxNonOrtho 75; }} }}
mergeTolerance 1e-6;
""")
    foam.write_dict(case, "system/surfaceFeatureExtractDict", "")
    foam.write_dict(case, "constant/SRFProperties", {
        "SRFModel": "rpm", "origin": (0.0, 0.0, 0.0), "axis": (0.0, 0.0, 1.0),
        "rpmCoeffs": {"rpm": float(rpm)}})
    foam.write_dict(case, "constant/transportProperties",
                    {"transportModel": "Newtonian", "nu": RawEntry(f"[0 2 -1 0 0 0 0] {nu:.6g}")})
    foam.write_dict(case, "constant/turbulenceProperties",
                    {"simulationType": "RAS", "RAS": {"RASModel": "kOmegaSST", "turbulence": True,
                                                      "printCoeffs": True}})
    Vt = rpm * 2 * math.pi / 60 * R
    kf, wf = foam.turbulence_fields_kw("kOmegaSST", Vt, R, intensity=0.01, visc_ratio=1.0, nu=nu)
    far = ("inlet", "outlet", "farfield")
    cyc = {"cyc_lo": {"type": "cyclic"}, "cyc_hi": {"type": "cyclic"}}
    Uf = {p: {"type": "SRFFreestreamVelocity", "UInf": (0.0, 0.0, 0.0),
              "value": RawEntry("uniform (0 0 0)")} for p in far}
    Uf.update({"axis": {"type": "slip"}, "blade": {"type": "noSlip"}}, **cyc)
    foam.field_file(case, "Urel", "volVectorField", "[0 1 -1 0 0 0 0]", "uniform (0 0 0)", Uf)
    pf = {p: {"type": "freestreamPressure", "freestreamValue": RawEntry("uniform 0")} for p in far}
    pf.update({"axis": {"type": "zeroGradient"}, "blade": {"type": "zeroGradient"}}, **cyc)
    foam.field_file(case, "p", "volScalarField", "[0 2 -2 0 0 0 0]", "uniform 0", pf)
    for name, val, wall, dims in (
            ("k", kf, {"type": "kqRWallFunction", "value": RawEntry(f"uniform {kf:.6g}")}, "[0 2 -2 0 0 0 0]"),
            ("omega", wf, {"type": "omegaWallFunction", "value": RawEntry(f"uniform {wf:.6g}")}, "[0 0 -1 0 0 0 0]")):
        ff = {p: {"type": "inletOutlet", "inletValue": RawEntry(f"uniform {val:.6g}"),
                  "value": RawEntry(f"uniform {val:.6g}")} for p in far}
        ff.update({"axis": {"type": "zeroGradient"}, "blade": wall}, **cyc)
        foam.field_file(case, name, "volScalarField", dims, f"uniform {val:.6g}", ff)
    nf = {p: {"type": "calculated", "value": RawEntry("uniform 0")} for p in far}
    nf.update({"axis": {"type": "calculated", "value": RawEntry("uniform 0")},
               "blade": {"type": "nutUSpaldingWallFunction", "value": RawEntry("uniform 0")}}, **cyc)
    foam.field_file(case, "nut", "volScalarField", "[0 2 -1 0 0 0 0]", "uniform 0", nf)
    foam.write_dict(case, "system/controlDict", f"""application     SRFSimpleFoam;
startFrom       latestTime;
startTime       0;
stopAt          endTime;
endTime         {iterations};
deltaT          1;
writeControl    timeStep;
writeInterval   {iterations};
purgeWrite      1;
writeFormat     binary;
runTimeModifiable false;
functions
{{
    forces
    {{
        type            forces;
        libs            (forces);
        writeControl    timeStep;
        writeInterval   1;
        patches         (blade);
        rho             rhoInf;
        rhoInf          {d.op.rho:.6g};
        CofR            (0 0 0);
    }}
}}
""")
    foam.write_dict(case, "system/fvSchemes", """ddtSchemes      { default steadyState; }
gradSchemes     { default Gauss linear; grad(Urel) cellLimited Gauss linear 1; }
divSchemes
{
    default         none;
    div(phi,Urel)   bounded Gauss linearUpwindV grad(Urel);
    div(phi,k)      bounded Gauss upwind;
    div(phi,omega)  bounded Gauss upwind;
    div((nuEff*dev2(T(grad(Urel))))) Gauss linear;
}
laplacianSchemes { default Gauss linear limited 0.5; }
interpolationSchemes { default linear; }
snGradSchemes   { default limited 0.5; }
wallDist        { method meshWave; }
""")
    foam.write_dict(case, "system/fvSolution", """solvers
{
    p     { solver GAMG; smoother GaussSeidel; tolerance 1e-7; relTol 0.05; }
    "(Urel|k|omega)" { solver smoothSolver; smoother symGaussSeidel; tolerance 1e-8; relTol 0.1; }
}
SIMPLE { nNonOrthogonalCorrectors 1; consistent yes; residualControl { p 1e-5; Urel 1e-6; } }
relaxationFactors { fields { p 0.3; } equations { Urel 0.7; k 0.5; omega 0.5; } }
""")
    foam.decompose_dict(case, np_)
    foam.write_allrun(case, [
        "run blockMesh",
        "run snappyHexMesh -overwrite",
        "run checkMesh",
        "if [ \"$NP\" -gt 1 ]; then run decomposePar -force; fi",
        "runp SRFSimpleFoam",
        "echo done",
    ])
    info = RotorCase(str(case), float(rpm), n, R, 360.0 / n, nu, Tb, Qb, ncell, tuple(surface_level))
    info.save()
    return info
