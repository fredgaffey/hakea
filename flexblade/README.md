# flexblade

Designs and simulates flexure (compliant) lag hinges and tuned blades for the Hakea
swashplateless rotor, for single or coaxial layouts. See [DESIGN.md](DESIGN.md) for the
physics, units and frames.

There are three ways in, and all of them sit on the same facade (`flexblade.api`):

| For | Use |
|---|---|
| People | `flexblade gui`, a local web app with a 3D view |
| Scripts and LLM agents | `flexblade <cmd>`, JSON in and JSON out |
| LLM agents (MCP) | `flexblade mcp`, a stdio MCP server |
| CAD (Keel / SolidCraft) | the Flexblade panel, see [flexblade/keel/KEEL.md](flexblade/keel/KEEL.md) |

All values are SI (m, kg, s, N, Pa, rad). The exceptions are fields ending in `_deg`
(degrees) and `blade.diameter_in` (inches). The GUI shows lengths in mm, but the JSON
stays in SI.

## Install

```bash
python3 -m venv ~/venv-flexblade
~/venv-flexblade/bin/pip install -e '.[gui,mcp,aero,fea]'
```

## GUI

```bash
flexblade gui                 # http://127.0.0.1:8765  (REST docs at /docs)
flexblade gui --port 9000 --open --cors-origin https://keel.example.com
```

- **Left panel.** Every parameter, generated from the `Design` schema and grouped into
  blade, flexure, motor, operating point, analysis, and coaxial (shown when `layout` is
  `coaxial`). Each one has a slider and display units. Material fields are dropdowns, and a
  dot marks values that differ from the preset. Internal fields
  (`op.thrust_share`, `op.extra_inflow`) are folded under *advanced*.
- **Top bar.** Pick a preset, load or save the design JSON, and press **Evaluate**
  (Ctrl+Enter). Tick **live** to re-evaluate automatically after each change. Export as
  STEP, STL, GLB or JSON.
- **3D view.** The rotor spins in the fixed frame, or you can untick that to view it in the
  hub frame. The blades show their 1/rev deformation, exaggerated; the
  *exaggeration* slider multiplies an automatic scale, and the readout shows the factor
  actually applied. A coaxial rotor shows both rotors counter-rotating. You can show beam
  nodes and colour by material or by check utilisation.
- **Tabs.**
  - Summary: KPIs, plus coaxial totals with an upper/lower table.
  - Checks: utilisation bars; anything above 1 shows red.
  - Campbell: mode frequencies against rpm, with 1P–4P lines and the operating rpm.
  - Harmonic: per-blade pitch/lag/flap amplitude and phase, phasors, and traces over one
    revolution.
  - Optimise: choose variables and targets, run it as a background job with a live history
    plot, then **Apply best**.
  - Diff: changes against the preset.
  - JSON: the raw results.
- **URL parameters.** For example:
  `?preset=5in_tpu_monolithic&set=layout=coaxial&set=flexure.thickness=0.001&eval=1&tab=harmonic&theme=light`.

The server binds to 127.0.0.1 only and has no authentication. three.js loads from
jsdelivr (`three@0.160.0`), so the 3D view needs network access on first load. Everything
else works offline.

## CLI (JSON in, JSON out)

A design argument can be a file path, `-` for stdin, inline JSON, or `preset:NAME`.
Each command prints exactly one JSON document on stdout. Anything the physics prints goes
to stderr. Errors come back as `{"error": ..., "type": ..., "details"?: [...]}` with exit
code 1, or 2 for usage errors.

```bash
flexblade schema --which Design            # JSON schema (with field descriptions/units)
flexblade presets                          # list; `flexblade presets NAME` -> Design JSON
flexblade materials
flexblade set preset:5in_tpu_monolithic flexure.thickness=0.0012 flexure.skew_deg=40 > d.json
flexblade get d.json flexure.thickness op.rpm
flexblade validate d.json
flexblade diff preset:5in_tpu_monolithic d.json
flexblade evaluate d.json                  # full Results JSON
flexblade evaluate d.json --compact        # without nodal arrays / mode shapes
flexblade evaluate d.json --digest         # short text digest (best for LLMs)
flexblade evaluate d.json --what trim,modes --set op.rpm=20000
flexblade digest results.json
flexblade export d.json --fmt step -o blade.step     # glb | stl | step | json
flexblade sweep d.json --var flexure.thickness=0.6e-3:2e-3:8 --var flexure.skew_deg=30,40,50 \
                       --out sweep.jsonl --workers 8      # lo:hi:n[:log] or v1,v2,...
flexblade optimize request.json --progress  # OptimizeRequest JSON -> OptimizeResult JSON
```

In a value, `path=value` is parsed as JSON where possible, so `op.rpm=null`,
`op.rpm_grid=[1e4,2e4]` and `layout=coaxial` all work. Paths use dots and list indices, as in
`blade.stations.3.chord`.

An example `OptimizeRequest`:

```json
{"design": {"preset": "5in_tpu_monolithic"},
 "variables": [{"path": "flexure.thickness", "lo": 4e-4, "hi": 2e-3},
               {"path": "flexure.skew_deg", "lo": 20, "hi": 60}],
 "targets": [{"quantity": "harmonic.control_ratio", "kind": "max"},
             {"quantity": "checks.max_utilisation", "kind": "le", "value": 0.8, "weight": 10}],
 "method": "differential_evolution", "max_evals": 300}
```

The GUI, REST and MCP also accept `{"preset": NAME, "set": {path: value}}` for the design.
`flexblade optimize` only accepts `{"preset": NAME}` with no `set`: that path validates the
request directly, so to change values first, run `flexblade set` and pass the full design.

### Topology optimisation (GPU)

When the parametric FACT flexure cannot meet its targets, `flexblade topopt` runs a 3D
two-material (soft TPU / stiff PA6-CF, both print-anisotropic) density topology
optimisation of the hub-to-blade-root region on a voxel grid (PyTorch, CUDA if present),
seeded from the voxelised FACT layout. The hinge stiffness about the skewed FACT axis is
set from the rotor model (`--lag-target`, default lag mode at 0.7/rev at hover); the other
five directions are maximised (`min_ratio` adds a constrained/hinge ratio floor). Loads come
from the rotor model with the flexure as a 6×6: the steady hinge-point wrench at trim (blade
root CF pull + steady moments) plus the flexure's own body force ρΩ²r is the sustained (creep)
case, the complex 1/rev wrench the fatigue case. Constraints (p-norms with adaptive
max-correction): equivalent-strain creep and 1/rev fatigue against each material's
`creep_strain_allow` / `fatigue_strain_amp_allow`, interlayer (σ_n/Zt, τ/S on the layer
plane) and Tsai-Hill strength, axis alignment, radial stretch, volume:

```
flexblade topopt preset:5in_cf_flexure --out out/5in --n-elem 100000 --soft-material tpu_95a --evaluate
```

Writes `render.png` / `render_util.png`, Taubin-smoothed `<name>_soft.stl` / `_stiff.stl` and
a faceted `<name>_topopt.step` (mm), `result.json` (6×6 at the hinge point, history, loads,
validation against a P2-tet re-analysis) and `design_topopt.json` (`flexure.topology =
"topopt"`, the stored 6×6 + mass properties + STL parts, so `flexblade evaluate` runs the
1/rev analysis on it and the GUI draws the optimised flexure on every blade). Creep study vs
the FACT plate (5" winner, 6" 2307 variant): `examples/topopt/creep_study.py`, results in
`examples/topopt/comparison.md`.

## MCP server (for LLM agents)

```bash
claude mcp add flexblade -- ~/venv-flexblade/bin/flexblade mcp
```

Or add it to any MCP client config:

```json
{"mcpServers": {"flexblade": {"command": "/home/fred/venv-flexblade/bin/flexblade", "args": ["mcp"]}}}
```

| Tool | What it does |
|---|---|
| `get_schema` | Returns the JSON schema of Design, Results or OptimizeRequest. |
| `list_presets` / `get_preset` | List the preset designs, or fetch one. |
| `list_materials` | Lists the materials: orthotropic moduli, density, loss factor. |
| `validate_design` | Checks a design. Returns `{ok, design_hash, design}` or `{ok: false, errors: [{loc, msg}]}`. |
| `set_design_values` | Sets values by dotted path, e.g. `{"flexure.thickness": 0.0012}`, and returns the new Design. |
| `evaluate_design` | Returns a digest plus compact Results. `detail` can be `digest`, `compact` or `full`. |
| `explain_results` | Writes a text digest of a Results JSON, with coaxial totals and each rotor. |
| `optimize_design` | Runs an optimisation and reports progress. |
| `sweep` | Runs a grid sweep, e.g. `{"flexure.thickness": "5e-4:2e-3:5"}`, and returns compact rows. |
| `export_geometry` | Writes glb, stl, step or json and returns the path. |
| Resources | `flexblade://design` (DESIGN.md) and `flexblade://schema/design`. |

A typical agent loop is: `list_presets`, then `get_preset`, then `set_design_values`, then
`evaluate_design` (read the digest), then `sweep` or `optimize_design`, then
`export_geometry`. Every `design` argument also takes a preset name or
`{"preset": NAME, "set": {...}}`. A tool that fails returns an error message the model can
read, including pydantic field locations.

## REST

Docs are served at `http://127.0.0.1:8765/docs`.

| Endpoint | Purpose |
|---|---|
| `/api/info` | Server info. |
| `/api/schema` | JSON schemas. |
| `/api/presets[/name]` | Presets. |
| `/api/materials` | Material library. |
| `/api/hints` | Parameter-panel hints. |
| `POST /api/validate` | Validate a design. |
| `POST /api/set` | Set values by dotted path. |
| `POST /api/evaluate` | Run an evaluation. |
| `POST /api/digest` | Text digest of a Results JSON. |
| `POST /api/geometry` | Mesh parts for the viewer. |
| `GET` or `POST /api/export?fmt=step` | Export geometry. |
| `POST /api/optimize` | Start an optimisation job. |
| `POST /api/sweep` | Start a sweep job. |
| `GET /api/jobs/{id}` | Job status, progress and history. |
| `GET`/`PUT /api/session/design` | The design currently open in the GUI. The Keel bridge reads it. |

## Development

```bash
~/venv-flexblade/bin/pytest tests/test_cli.py tests/test_mcp.py tests/test_gui.py
FLEXBLADE_FAKE=1 flexblade gui         # demo with placeholder physics (tests/fake_backend.py)
FLEXBLADE_FAKE=missing flexblade gui   # fake only the modules that don't import yet
FLEXBLADE_BROWSER_TEST=1 pytest tests/test_gui.py -k headless    # headless Chrome render
python examples/shoot.py http://127.0.0.1:8765 examples/        # screenshots + console-error check
```

The UI code is in these files:

| File | Contents |
|---|---|
| `flexblade/cli.py` | The CLI. |
| `flexblade/mcp_server.py` | The MCP server. |
| `flexblade/uicommon.py` | Dotted paths, sweeps, digest, geometry loading. |
| `flexblade/gui/server.py` | The FastAPI server. |
| `flexblade/gui/hints.py` | Display units and slider ranges. |
| `flexblade/gui/static/` | The frontend: vanilla ES modules, no build step. |
| `flexblade/keel/` | The CAD bridge. |
