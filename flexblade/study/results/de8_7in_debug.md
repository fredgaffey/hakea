# DE8 7in all-fail debug
- Local single evals of the 7in seed (robust 0.7/1.0/1.4, J-hub set) pass in all 4 flag combos (resolve-presets 0/6 x one-sided 0/1): J about -6.5..-6.95, no hard viol. Neither flag breaks it.
- Also checked: seed via encode_design/set_design with bal.* (strap_length clipped 0.004->0.006, bal.mass=0): still fine.
- Root cause: evals.jsonl.gz shows 413/416 (7in) and 719/729 (10in) DE8 evals fail with `ModuleNotFoundError: manifold3d`. rotor.py (fitting_mass="auto", clamped strips) imports fittings -> manifold3d.
- The Mac worker venv (/tmp/flexblade-work/venv) lacks manifold3d (remote.DEPS omitted it; bootstrap probe only imported flexblade). Failing evals return instantly, so work stealing sends nearly every job to the Mac. Only ~3 local evals succeed: best J +7.31 is a real (infeasible) eval.
- Fix: remote.DEPS += manifold3d; bootstrap probe imports manifold3d (forces rebuild). Test: tests/test_remote_deps.py.
- Needed: restart DE8 after the Mac venv is rebuilt, or run local-only hosts. 10in is affected too.
