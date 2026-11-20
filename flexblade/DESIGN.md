# flexblade — design contract

Shared contract for every module. Read fully before writing code. If you need to
change something here, say so in your report instead of silently diverging.

## Goal

Rough-but-good simulator to decide whether a swashplateless (torque-pulsed) rotor
can use a **flexure** (or a fully compliant tuned blade) instead of a skewed lag
hinge, for rotor diameters **4"–12"**, and with which materials:

1. monolithic tuned TPU blade (preferred if it works)
2. TPU + embedded rod (carbon pultrusion / stainless wire, print-pause insert)
3. TPU core + CF-filament skin (multi-material print)
4. all CF-filament flexure (avoid creep)

Method: Freedom-and-Constraint Topology (FACT) defines manufacturable flexure
layouts; parameters (thickness, width, length, twist, skew, …) are solved by
optimisation against targets. Full topology optimisation is a later fallback.
Core analysis is a **reduced model**: 3D rotating beam FE (blade + flexure members
+ hub) + strip aero → linear harmonic response at 1/rev. FEA (3D solid) and CFD
are spot checks.

## Mechanism (Paulos/Yim; Joshua Bird)

Motor torque has a 1/rev component `ΔQ·cos(Ωt + φ)`. Hub speed oscillates; blade
inertia makes each blade lead/lag relative to the hub. A **skewed** lag hinge
(axis tilted by δ from the rotor axis toward the span) couples lag ζ to pitch:
θ ≈ ζ·tan δ. Blade 1 and blade 2 have **opposite skew**, so one pitches up while
the other pitches down → differential thrust → mean hub moment (+2/rev ripple).
Same-sign skew would give 1/rev collective thrust only. Only `n_blades = 2` is
supported for the cyclic concept (others allowed for trim/aero only).

Important physics the model must include: centrifugal tension (hundreds of N),
centrifugal stiffening, spin softening, propeller-moment (tennis-racket) pitch
stiffness, Coriolis, hub torsional DOF with motor+hub inertia, frequency-dependent
complex modulus of TPU (viscoelastic loss), print anisotropy, quasi-steady or
Theodorsen unsteady aero, Pitt–Peters dynamic inflow for the 1/rev hub moment.

## Units and frames

SI everywhere (m, kg, s, N, Pa, rad). Degrees only in user-facing fields
whose name ends in `_deg`. Diameters for humans in inches only in `diameter_in`.

Rotating hub frame H (rotates with mean speed Ω about +z):
- +z: rotor axis, thrust direction (up).
- Rotor spins **+Ω about +z** (CCW seen from above).
- Blade 1 spans along **+x**; blade k is rotated by 2π(k-1)/n about z.
- For blade 1 the direction of travel is **+y**; leading edge faces +y.
- Pitch θ: + rotation about the blade +x axis (leading edge up, nose-up). 
- Lag ζ: + = blade lags (tip moves toward −y) = rotation about −z.
- Flap β: + = tip up = rotation about −y.
- Beam node DOFs: [ux, uy, uz, rx, ry, rz] in frame H.
- Hub DOF: φ = hub rotation about +z (perturbation from Ωt).

Fixed frame F: hub moment output Mx (roll), My (pitch); azimuth ψ = Ωt of blade 1,
ψ=0 when blade 1 is along fixed +x. Cyclic phase is reported as angle between
the torque-pulse peak azimuth and the hub-moment vector direction.

Harmonic convention: x(t) = Re{X e^{iωt}}.

## Package layout and ownership

```
flexblade/
  schema.py      pydantic models: Design, Results (OWNER: lead) — the contract
  materials.py   material library + anisotropy helpers (OWNER: lead)
  api.py         facade: evaluate / optimize / export (OWNER: lead, stubs until wired)
  section.py     2D composite cross-section properties (OWNER: structures)
  beam.py        3D rotating beam FE: K, M, G, Kσ, spin softening (OWNER: structures)
  fact.py        FACT freedom/constraint spaces → flexure element layouts (OWNER: structures)
  rotor.py       assembly: trim prestress, modal/Campbell, 1/rev harmonic response,
                 stress/creep/fatigue checks (OWNER: structures)
  optimize.py    parametric solve of flexure params against targets (OWNER: structures)
  aero.py        polars (NeuralFoil, cached), BEM trim, linearised unsteady strip aero,
                 Pitt–Peters (OWNER: aero)
  motor.py       motor/ESC torque model, 1/rev torque capability (OWNER: aero)
  presets.py     4"–12" reference designs (OWNER: aero, structures fills flexure)
  geometry.py    surface mesh (GLB/STL/OBJ) + gmsh OCC solid + STEP export (OWNER: fea)
  fea.py         scikit-fem 3D anisotropic elasticity checks (OWNER: fea)
  cfd/           OpenFOAM case generation + local/remote (ssh mac) runner (OWNER: fea)
  cli.py, mcp_server.py, gui/ (OWNER: ui)
  keel/          SolidCraft/Keel plugin bridge (OWNER: ui)
tests/           pytest, one file per module
```

Python env: `~/venv-flexblade` (numpy, scipy, numba, pydantic 2, scikit-fem, gmsh,
meshio, neuralfoil, aerosandbox, fastapi, uvicorn, trimesh, mcp, pytest).
Package installed editable. Run tests with `~/venv-flexblade/bin/pytest`.

Do NOT touch anything outside `flexblade/` (the parent hakea repo has submodules).
Do NOT git commit. Do NOT install system packages without asking the lead.

## Key interfaces

### materials.py (exists)
`get_material(name) -> Material`; `Material.stiffness_matrix(orientation)` (6×6
Voigt in frame H); `Material.axial_modulus(orientation, t_hat)`,
`Material.shear_modulus(orientation, t_hat, n_hat)`; `Material.complex_scale(freq_hz)`
(multiplier for storage modulus incl. loss: E* = E·complex_scale).

### section.py
`section_properties(shape, layup, orientation, t_hat) -> SectionProps`
SectionProps fields: EA, EIy, EIz, EIyz (or principal angle), GJ, GAy, GAz,
mass_per_len (rho·A), Ip_mass (polar mass moment per len), Iyy_mass, Izz_mass,
centroid offset (y,z), shear centre offset (y,z), elastic and mass offsets
relative to the reference axis (beam line). shape: airfoil polygon (from
geometry/aero airfoil coordinates scaled by chord, twisted) or rectangle/circle.

### beam.py / rotor.py
Rotor model = list of nodes (frame H) + list of beam elements, each with its
SectionProps + material (for complex modulus) + orientation. Flexure FACT members
are beam elements too (wire = slender beam, blade flexure = wide thin beam, split
into several parallel beams across width if needed). Blade = chain of beam elements
along the span with properties from sections at stations.
Rotor must expose:
- `build_model(design) -> RotorModel`
- `static_trim(model, aero) -> StaticState` (nonlinear centrifugal + mean aero,
  iterate to convergence; gives axial preload for Kσ, mean deflection, collective)
- `modes(model, state, rpm) -> list[Mode]`; `campbell(model, rpm_grid)`
- `harmonic_1p(model, state, aero) -> HarmonicResult` (unit and design-level
  torque modulation; per-blade pitch/lag/flap amplitudes+phases; hub moment in
  fixed frame; nodal complex displacements for animation)
- `checks(model, state, harmonic) -> list[Check]` (stress, interlayer stress,
  creep strain under centrifugal load, fatigue amplitude)

### aero.py
```
class AeroModel:
    def __init__(self, design): ...            # builds/caches polars per station
    def trim(self, rpm=None, thrust=None, inflow=None, blade_shape=None) -> AeroTrim
        # BEM hover trim: if thrust given solve rpm (or collective); returns per-station
        # r, chord, phi, alpha, Re, cl, cd, cm, inflow v_i, dFz/dr, dFy/dr (in-plane,
        # +y = opposing travel? NO: report force on blade in frame H), dMx/dr, T, Q, P.
    def linearized(self, trim, omega) -> list[np.ndarray]
        # per station i: complex 3x4 matrix A_i(omega) such that
        # [dFz, dFy, dMx]_per_unit_span = A_i @ [uz, uy, theta, dlambda]
        # uz, uy: section translations at aerodynamic centre (quarter chord) in frame H,
        # theta: section pitch about +x, dlambda: local inflow perturbation (m/s, +down).
        # Includes rate terms via i*omega (quasi-steady) or Theodorsen C(k) if enabled.
    def pitt_peters_hover(self, trim, omega) -> complex 2x2 (fixed frame) relating
        # hub aerodynamic moment coefficients to cyclic inflow coefficients.
```
Sign: Fz + up (thrust), Fy in frame H (drag on blade 1 acts toward −y), Mx about +x.

### api.py (facade every consumer uses — CLI, MCP, GUI, Keel)
```
evaluate(design: Design | dict, what: list[str] = ["trim","modes","harmonic","checks"]) -> Results
optimize(request: OptimizeRequest | dict) -> OptimizeResult
export_geometry(design, fmt in {"glb","stl","step","json"}, path) -> path
presets() -> dict[name, Design];  schema() -> JSON schema of Design/Results
```
Everything JSON-serialisable (pydantic `.model_dump(mode="json")`). Deterministic.
Each Results carries `design_hash` (sha256 of canonical Design JSON) and
`model_version`.

### bladegeom.py (exists, lead)
`resolve_stations(blade)`, `station_arrays(blade)`, `airfoil_coords(airfoil, t/c)`,
`section_polygon_H(xy, chord, twist, r)` — single source of truth for blade shape.

### fact.py → shared
`generate_members(design) -> list[FlexureMember]` for blade 1 in frame H (blade k
by rotation 2π(k-1)/n about z and skew sign mirror for k even). geometry.py and
rotor.py both call this. Also `hinge_axis(design) -> (point, unit_dir)`.

### presets.py
`all_presets() -> dict[str, Design]`: at least hakea_5in_hinge_equiv, 4in, 7in,
10in, 12in for each material concept (tpu_monolithic, tpu_rod, tpu_cf_skin,
cf_flexure). `scale_design(design, diameter_in)` with motor/mass scaling.

## Conventions settled after aero landed (binding)
- Aero loads (`AeroTrim` Fz/Fy/Mx and `linearized`) are at the **quarter chord**.
  Structures transfers them to the beam reference line when
  `elastic_axis_chord_frac != 0.25`.
- `linearized` already contains aerodynamic **apparent mass** (+πρb²ω²); structures
  must not add it again.
- `pitt_peters_hover(trim, omega)` takes the **fixed-frame** frequency; steady
  control moment → omega = 0. Hub moment convention: Mx = Σ∫ y dFz, My = −Σ∫ x dFz
  (fixed frame), inflow λ positive down, v(r,ψ) = ΩR (r/R)(λc cosψ + λs sinψ).
  Use `aero.cyclic_inflow(...).blade_1p(...)` for the rotating-frame dλ fed to column 4
  of `linearized`, and `aero.mean_hub_moment` for the fixed-frame moment.
- Default blade law (bladegeom.resolve_stations) = presets hover law:
  c = R(0.30 − 0.12x), θ = 8° + atan(0.12/x).

## Coaxial layout (added)
`Design.layout = "coaxial"` → `api.evaluate` dispatches to `coaxial.evaluate(design, what)`
(OWNER: aero). Upper rotor = primary definition, spins +Ω. Lower spins −Ω, analysed in
its own mirrored frame by the same single-rotor code, results mirrored back. Coupling via
`op.extra_inflow` (InflowProfile, m/s down, ADDED to own induced velocity inside
aero.trim — aero must honour it) and `op.thrust_share`. Structures/rotor code needs no
coaxial awareness beyond passing design.op through to AeroModel.

## BLAS threads (binding)
BLAS is pinned to 1 thread at import (multithreaded BLAS on our ~300-DOF matrices was
~500× slower under load). For dense work with n ≳ 500 wrap it in
`with parallel.blas_threads_for(n): ...` — it stays at 1 inside parallel workers
(FLEXBLADE_WORKER=1). Sparse solvers (splu/eigsh) are unaffected.
