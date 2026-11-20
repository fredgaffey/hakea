"""2D pitching-airfoil URANS case (pimpleFoam + solid-body oscillatingRotatingMotion).

Validates aero.py's unsteady (Theodorsen / linearised) lift amplitude and phase at the
1/rev reduced frequency k = ω b / V = c / (2 r) for a blade station.

Mesh: gmsh O-domain (radius `farfield_chords`·c) with a quad boundary layer on the
airfoil, one cell extruded in z, written as MSH 2.2 -> gmshToFoam on the run side.
The whole mesh rotates rigidly about the pivot (quarter chord by default):
α(t) = α0 + A sin(ωt). Freestream along +x; nose-up α positive; lift +y.
"""
from __future__ import annotations

import json
import math
import shutil
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from .. import bladegeom
from ..schema import Design
from . import foam
from .foam import RawEntry


@dataclass
class PitchCase:
    case_dir: str
    r: float
    chord: float
    V: float
    Re: float
    mach: float
    k: float
    omega: float
    alpha_mean_deg: float
    alpha_amp_deg: float
    pivot_chord_frac: float
    period: float
    end_time: float
    dt: float
    nu: float
    turbulence: str
    n_cells: int = 0
    source: str = ""

    def save(self):
        Path(self.case_dir, "flexblade_case.json").write_text(json.dumps(asdict(self), indent=1))


def station_conditions(design: Design, r_frac: float, rpm: Optional[float] = None,
                       alpha_mean_deg: Optional[float] = None) -> dict:
    """Section r, chord, thickness, V, Re, Mach, mean α from aero trim when available."""
    blade = design.blade
    R = blade.radius
    r = r_frac * R
    st = bladegeom.resolve_stations(blade)
    rs = np.array([s.r_frac * R for s in st])
    chord = float(np.interp(r, rs, [s.chord for s in st]))
    tf = [s.thickness_frac for s in st]
    tfrac = None
    if all(t is not None for t in tf):
        tfrac = float(np.interp(r, rs, tf))
    nu = 1.5e-5
    a = 343.0
    src = "given"
    alpha = alpha_mean_deg
    V = None
    try:
        from ..aero import AeroModel, air_viscosity, speed_of_sound
        am = AeroModel(design)
        tr = am.trim(rpm=rpm) if rpm else am.trim()
        rpm = tr.rpm
        V = float(np.interp(r, tr.r, tr.W))
        if alpha is None:
            alpha = float(np.degrees(np.interp(r, tr.r, tr.alpha)))
        nu = air_viscosity(design.op.temperature_C) / design.op.rho
        a = speed_of_sound(design.op.temperature_C)
        src = "aero.trim"
    except Exception as e:  # noqa: BLE001
        if rpm is None:
            raise ValueError(f"rpm required (aero trim failed: {e})") from e
        src = f"rpm given, no aero ({e})"
    if V is None:
        V = rpm * 2 * math.pi / 60 * r
    return dict(r=r, chord=chord, thickness_frac=tfrac, V=V, rpm=rpm, nu=nu, a=a,
                alpha_mean_deg=alpha if alpha is not None else 4.0, source=src)


def airfoil_mesh(xy: np.ndarray, chord: float, alpha_deg: float, pivot: float,
                 path: Path, farfield_chords: float = 20.0, n_surf: int = 160,
                 first_cell: float = 2e-5, bl_thickness: float = 0.04, bl_ratio: float = 1.15,
                 far_size: float = 1.5, span: float = 0.01,
                 wake_size: float = 0.06, wake_chords: float = 8.0) -> int:
    """Write MSH2.2 O-mesh of airfoil (chord-scaled, rotated by α about pivot) to path.
    Lengths in chords for the BL parameters. Returns number of 3D cells."""
    import gmsh
    from ..geometry import _Gmsh
    xy = np.asarray(xy, float)
    if np.linalg.norm(xy[0] - xy[-1]) < 1e-9:
        xy = xy[:-1]
    # resample to n_surf points (cosine-ish already) keep as is if close
    P = (xy - [pivot, 0.0]) * chord
    a = -math.radians(alpha_deg)      # nose-up = clockwise for flow +x, LE at left
    Rm = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    P = P @ Rm.T
    c = chord
    with _Gmsh() as g:
        geo = g.model.geo
        h_surf = c * 2.0 / n_surf
        # sharp trailing edge (open-TE gap closed by averaging the two TE points), split at
        # the LE into upper / lower splines so the TE is a true corner for the BL fan
        ile = int(np.argmin(P[:, 0] * math.cos(a) - P[:, 1] * math.sin(a)))
        te = 0.5 * (P[0] + P[-1])
        Q = P.copy()
        Q[0] = te
        Q = Q[:-1] if np.linalg.norm(P[-1] - P[0]) > 0 else Q
        pts = [geo.addPoint(x, y, 0, h_surf) for x, y in Q]
        up = geo.addSpline(pts[:ile + 1])
        lo = geo.addSpline(pts[ile:] + [pts[0]])
        curves = [up, lo]
        loop_af = geo.addCurveLoop(curves)
        te_pt = pts[0]
        Rf = farfield_chords * c
        cp = geo.addPoint(0, 0, 0)
        fp = [geo.addPoint(Rf * math.cos(t), Rf * math.sin(t), 0, far_size * c)
              for t in (0, math.pi / 2, math.pi, 3 * math.pi / 2)]
        arcs = [geo.addCircleArc(fp[i], cp, fp[(i + 1) % 4]) for i in range(4)]
        loop_ff = geo.addCurveLoop(arcs)
        surf = geo.addPlaneSurface([loop_ff, loop_af])
        geo.synchronize()
        f = g.model.mesh.field.add("BoundaryLayer")
        g.model.mesh.field.setNumbers(f, "CurvesList", curves)
        g.model.mesh.field.setNumber(f, "Size", first_cell * c)
        g.model.mesh.field.setNumber(f, "Ratio", bl_ratio)
        g.model.mesh.field.setNumber(f, "Thickness", bl_thickness * c)
        g.model.mesh.field.setNumber(f, "Quads", 1)
        g.model.mesh.field.setNumbers(f, "FanPointsList", [te_pt])
        g.model.mesh.field.setAsBoundaryLayer(f)
        # wake / near-field refinement
        fd = g.model.mesh.field.add("Distance")
        g.model.mesh.field.setNumbers(fd, "CurvesList", curves)
        ft = g.model.mesh.field.add("Threshold")
        g.model.mesh.field.setNumber(ft, "InField", fd)
        g.model.mesh.field.setNumber(ft, "SizeMin", h_surf * 1.5)
        g.model.mesh.field.setNumber(ft, "SizeMax", far_size * c)
        g.model.mesh.field.setNumber(ft, "DistMin", 0.05 * c)
        g.model.mesh.field.setNumber(ft, "DistMax", 8 * c)
        fb = g.model.mesh.field.add("Box")      # wake band (pitching wake sheds ±α_amp)
        g.model.mesh.field.setNumber(fb, "VIn", wake_size * c)
        g.model.mesh.field.setNumber(fb, "VOut", far_size * c)
        g.model.mesh.field.setNumber(fb, "XMin", -0.5 * c)
        g.model.mesh.field.setNumber(fb, "XMax", wake_chords * c)
        g.model.mesh.field.setNumber(fb, "YMin", -0.6 * c)
        g.model.mesh.field.setNumber(fb, "YMax", 0.6 * c)
        g.model.mesh.field.setNumber(fb, "Thickness", 2 * c)
        fm = g.model.mesh.field.add("Min")
        g.model.mesh.field.setNumbers(fm, "FieldsList", [ft, fb])
        g.model.mesh.field.setAsBackgroundMesh(fm)
        g.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
        g.option.setNumber("Mesh.Algorithm", 6)
        g.option.setNumber("Mesh.RecombineAll", 0)
        ext = geo.extrude([(2, surf)], 0, 0, span, numElements=[1], recombine=True)
        geo.synchronize()
        vol = [t for d, t in ext if d == 3][0]
        top = [t for d, t in ext if d == 2][0]
        sides = [t for d, t in ext if d == 2][1:]
        # sides follow the order of the surface boundary curves: farfield arcs then airfoil
        n_ff = 4
        ff_s, af_s = sides[:n_ff], sides[n_ff:]
        g.model.addPhysicalGroup(3, [vol], name="fluid")
        g.model.addPhysicalGroup(2, [surf, top], name="frontAndBack")
        g.model.addPhysicalGroup(2, ff_s, name="farfield")
        g.model.addPhysicalGroup(2, af_s, name="airfoil")
        g.model.mesh.generate(2)
        g.model.mesh.generate(3)
        g.option.setNumber("Mesh.MshFileVersion", 2.2)
        g.option.setNumber("Mesh.SaveAll", 0)
        g.write(str(path))
        types, tags, _ = g.model.mesh.getElements(3)
        return int(sum(len(t) for t in tags))


def pitching_airfoil_case(design: Design, case_dir: str | Path, r_frac: float = 0.75,
                          rpm: Optional[float] = None, alpha_mean_deg: Optional[float] = None,
                          alpha_amp_deg: float = 2.0, pivot_chord_frac: float = 0.25,
                          k: Optional[float] = None, n_periods: float = 5.0,
                          steps_per_period: int = 1440, turbulence: str = "kOmegaSST",
                          np_: int = 4, first_cell_chords: Optional[float] = None,
                          farfield_chords: float = 20.0, overwrite: bool = True) -> PitchCase:
    """Write an OpenFOAM case for a pitching section of `design` at r = r_frac·R.

    Reduced frequency k defaults to 1/rev: ω = Ω, k = Ω b / V. turbulence: 'kOmegaSST'
    (fully turbulent) or 'laminar' (Re ≲ 5e4 sections are transitional; run both).
    """
    case = Path(case_dir)
    if case.exists() and overwrite:
        shutil.rmtree(case)
    case.mkdir(parents=True, exist_ok=True)
    sc = station_conditions(design, r_frac, rpm, alpha_mean_deg)
    c, V, nu = sc["chord"], sc["V"], sc["nu"]
    b = c / 2
    Omega = sc["rpm"] * 2 * math.pi / 60
    if k is None:
        omega = Omega
        k = omega * b / V
    else:
        omega = k * V / b
    Re = V * c / nu
    T = 2 * math.pi / omega
    dt = T / steps_per_period
    end = n_periods * T
    # y+ ~ 1 first cell (flat plate Cf estimate)
    if first_cell_chords is None:
        cf = 0.027 / Re ** (1 / 7)
        utau = V * math.sqrt(cf / 2)
        first_cell_chords = min(1.0 * nu / utau / c, 2e-3) if turbulence != "laminar" else \
            min(0.2 / math.sqrt(Re), 2e-3)
    xy = bladegeom.airfoil_coords(design.blade.airfoil, sc["thickness_frac"], n=101)
    span = 0.05 * c
    n_cells = airfoil_mesh(xy, c, sc["alpha_mean_deg"], pivot_chord_frac, case / "mesh.msh",
                           farfield_chords=farfield_chords, first_cell=first_cell_chords,
                           span=span)
    info = PitchCase(str(case), sc["r"], c, V, Re, V / sc["a"], k, omega, sc["alpha_mean_deg"],
                     alpha_amp_deg, pivot_chord_frac, T, end, dt, nu, turbulence, n_cells,
                     sc["source"])
    _write_pitch_dicts(case, info, span, np_)
    info.save()
    return info


def _write_pitch_dicts(case: Path, ci: PitchCase, span: float, np_: int):
    V, c = ci.V, ci.chord
    lam = ci.turbulence == "laminar"
    foam.write_dict(case, "constant/transportProperties",
                    {"transportModel": "Newtonian", "nu": RawEntry(f"[0 2 -1 0 0 0 0] {ci.nu:.6g}")})
    foam.write_dict(case, "constant/turbulenceProperties",
                    {"simulationType": "laminar"} if lam else
                    {"simulationType": "RAS", "RAS": {"RASModel": ci.turbulence, "turbulence": True,
                                                      "printCoeffs": True}})
    foam.write_dict(case, "constant/dynamicMeshDict", {
        "dynamicFvMesh": "dynamicMotionSolverFvMesh",
        "motionSolverLibs": RawEntry("(fvMotionSolvers)"),
        "motionSolver": "solidBody",
        "solidBodyMotionFunction": "oscillatingRotatingMotion",
        "oscillatingRotatingMotionCoeffs": {
            "origin": (0.0, 0.0, 0.0),
            # rotation about +z is nose-DOWN for flow +x; amplitude negative → α = α0 + A sin ωt
            "amplitude": (0.0, 0.0, -ci.alpha_amp_deg),
            "omega": ci.omega}})
    kf, wf = foam.turbulence_fields_kw(ci.turbulence, V, c, nu=ci.nu)
    foam.field_file(case, "U", "volVectorField", "[0 1 -1 0 0 0 0]", f"uniform ({V:.6g} 0 0)", {
        "farfield": {"type": "freestreamVelocity", "freestreamValue": RawEntry(f"uniform ({V:.6g} 0 0)")},
        "airfoil": {"type": "movingWallVelocity", "value": RawEntry("uniform (0 0 0)")},
        "frontAndBack": {"type": "empty"}})
    foam.field_file(case, "p", "volScalarField", "[0 2 -2 0 0 0 0]", "uniform 0", {
        "farfield": {"type": "freestreamPressure", "freestreamValue": RawEntry("uniform 0")},
        "airfoil": {"type": "zeroGradient"},
        "frontAndBack": {"type": "empty"}})
    if not lam:
        for name, val, wall in (("k", kf, {"type": "kqRWallFunction", "value": RawEntry(f"uniform {kf:.6g}")}),
                                ("omega", wf, {"type": "omegaWallFunction", "value": RawEntry(f"uniform {wf:.6g}")})):
            dims = "[0 2 -2 0 0 0 0]" if name == "k" else "[0 0 -1 0 0 0 0]"
            foam.field_file(case, name, "volScalarField", dims, f"uniform {val:.6g}", {
                "farfield": {"type": "inletOutlet", "inletValue": RawEntry(f"uniform {val:.6g}"),
                             "value": RawEntry(f"uniform {val:.6g}")},
                "airfoil": wall, "frontAndBack": {"type": "empty"}})
        foam.field_file(case, "nut", "volScalarField", "[0 2 -1 0 0 0 0]", "uniform 0", {
            "farfield": {"type": "calculated", "value": RawEntry("uniform 0")},
            "airfoil": {"type": "nutLowReWallFunction", "value": RawEntry("uniform 0")},
            "frontAndBack": {"type": "empty"}})
    write_every = max(1, int(round(ci.period / ci.dt)))
    foam.write_dict(case, "system/controlDict", f"""application     pimpleFoam;
startFrom       latestTime;
startTime       0;
stopAt          endTime;
endTime         {ci.end_time:.8g};
deltaT          {ci.dt:.8g};
writeControl    timeStep;
writeInterval   {write_every * 100};
purgeWrite      1;
writeFormat     binary;
writePrecision  8;
timeFormat      general;
timePrecision   8;
runTimeModifiable false;
adjustTimeStep  no;

functions
{{
    forceCoeffs
    {{
        type            forceCoeffs;
        libs            (forces);
        writeControl    timeStep;
        writeInterval   1;
        patches         (airfoil);
        rho             rhoInf;
        rhoInf          1;
        CofR            (0 0 0);
        liftDir         (0 1 0);
        dragDir         (1 0 0);
        pitchAxis       (0 0 -1);
        magUInf         {V:.8g};
        lRef            {c:.8g};
        Aref            {c * span:.8g};
    }}
}}
""")
    foam.write_dict(case, "system/fvSchemes", """ddtSchemes      { default backward; }
gradSchemes     { default Gauss linear; grad(U) cellLimited Gauss linear 1; }
divSchemes
{
    default         none;
    div(phi,U)      Gauss linearUpwindV grad(U);
    div(phi,k)      bounded Gauss upwind;
    div(phi,omega)  bounded Gauss upwind;
    div((nuEff*dev2(T(grad(U))))) Gauss linear;
}
laplacianSchemes { default Gauss linear limited 0.5; }
interpolationSchemes { default linear; }
snGradSchemes   { default limited 0.5; }
wallDist        { method meshWave; }
""")
    foam.write_dict(case, "system/fvSolution", """solvers
{
    "(p|pcorr)"     { solver GAMG; smoother GaussSeidel; tolerance 1e-7; relTol 0.01; }
    "(p|pcorr)Final" { $p; relTol 0; }
    "(U|k|omega)"   { solver smoothSolver; smoother symGaussSeidel; tolerance 1e-8; relTol 0.05; maxIter 200; }
    "(U|k|omega)Final" { $U; relTol 0; }
}
PIMPLE
{
    momentumPredictor yes;
    nOuterCorrectors 3;
    nCorrectors     2;
    nNonOrthogonalCorrectors 1;
    correctPhi      no;
    moveMeshOuterCorrectors no;
    turbOnFinalIterOnly false;
}
relaxationFactors
{
    fields      { p 0.7; pFinal 1; }
    equations   { "(U|k|omega)" 0.8; "(U|k|omega)Final" 1; }
}
""")
    foam.decompose_dict(case, np_)
    foam.write_allrun(case, [
        "run gmshToFoam mesh.msh",
        "foamDictionary constant/polyMesh/boundary -entry entry0/frontAndBack/type -set empty > /dev/null",
        "foamDictionary constant/polyMesh/boundary -entry entry0/airfoil/type -set wall > /dev/null",
        "foamDictionary constant/polyMesh/boundary -entry entry0/farfield/type -set patch > /dev/null",
        "rm -f mesh.msh",
        "run checkMesh",
        "if [ \"$NP\" -gt 1 ]; then run decomposePar -force; fi",
        "runp pimpleFoam",
        "echo done",
    ])
