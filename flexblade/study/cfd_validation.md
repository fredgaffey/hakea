# CFD validation of blade shapes (status)

Mac went offline mid-task, so this is partial. Everything below marked PENDING still needs a run.

## Tooling (done)
- `flexblade.cfd.validate.blade_shapes(designs, where="mac", pitching=False, np_=10)` runs, one design at a
  time, a 3D hover sector (`hover_case_run`) and optionally a 1/rev 0.75R pitching case, writing
  `summary.json` + `table.md` (T, Q, FoM vs `AeroModel.trim` BEM; |H|, phase vs Theodorsen).
- `pitching_airfoil_case(..., turbulence="kOmegaSSTLM")` now writes ReThetat/gammaInt fields (Tu 0.5 %,
  Langtry-Menter Re_theta_t correlation) and schemes/solvers. Case generation tested locally; NOT yet run
  (kOmegaSSTLM exists in v2606 RAS list; field names/BCs unverified in a solver run).
- `run_case(where="mac")` fails fast with a clear "Mac unreachable" error (ConnectTimeout 10 s, keepalives,
  retries on transport failure only).

## 3D hover, 5in_tpu_rod_winner, 18521 rpm (BEM: T 5.884 N, Q 0.0583 N m, FoM 0.716)
| mesh | cells | iters | wall time | T cfd/BEM | Q cfd/BEM | FoM cfd |
|---|---|---|---|---|---|---|
| coarse (8 cells/R bg, surf lvl 3-4) | 262k | 800 | 248 s (np=10, incl. meshing) | 0.766 | 1.002 | 0.479 |
| fine (12 cells/R, lvl 4-5, ~1.3M) | PENDING | 2000 | PENDING | | | |

Command: `python hover.py fine 12 4 5 2000` equivalent = `hover_case_run(d, "fine", out, background_cells_per_R=12, surface_level=(4,5), iterations=2000)`.

Caveats: coarse checkMesh failed 5 checks (1 negative-volume cell, non-orth max 110 deg, 40 faces > 70 deg),
likely at the thin TPU trailing edge/tip; run did not diverge and forces were steady (std 0.15 % T), but
treat as unconverged-in-mesh. Single-blade MRF steady RANS, fully-turbulent SST with wall functions at
Re ~ 5e4-1e5 is questionable, and BEM uses 2D polars; the thrust deficit (-23 %) at equal torque is the
expected sign for a low-Re blade where BEM polars are optimistic, but mesh independence is not shown.
Flexure/hub omitted, blade rigid.

## 2D pitching 0.75R 1/rev (5in_tpu_monolithic, Re 8.3e4)
| model | |H| | phase |
|---|---|---|
| Theodorsen (aero.linearized) | 5.18 | -0.9 deg |
| old CFD kOmegaSST | 3.79 | +27.9 deg |
| kOmegaSSTLM | PENDING | PENDING |
