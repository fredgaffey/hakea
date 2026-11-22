# Trim convergence audit
- static_trim (rotor.py ~1011-1169): rpm bracket centred on rigid BEMT rpm; geometric walk x1.06 (26 pts), fallback walk from 0.4x rpm (x1.15, 18 pts), then Illinois regula-falsi.
- Root outside bracket: no exception, no clamp. Best point returned with trim_ok=False and warning "trim failed: ..."; non-converged inner Newton adds "did not converge".
- Consumers: run_study.metrics sets viol["trim_failed"] and feasible=False on summary.trim_ok False; warnings containing "failed"/"not converge" add viol["convergence"]; control_ratio is NaN -> 0 on failure. So no normal-looking score escapes.
- Measured (static_trim, all conv=True, trim_ok=True, |T-W| <= 0.005 N):
  - 5in lead, flex t x1.00/0.95/1.05, T0.7/1.0/1.25: rpm 12249/14197/15666, 12315/14290/15769, 12195/14111/15562.
  - 10in_tt_strap_steelx_v8, T0.7/1.0/1.4: rpm 6490/7730/9145 (T-W -0.001/-0.004/-0.005 N).
- 5in lead is NOT affected; the 10in v8 steelx design is NOT affected at these points. The old "rpm at bracket ends" 10in cases were not reproduced here (likely other designs / pre-fix code).
- No behaviour change made. Added tests/test_trim_unconverged.py (40x mass forces out-of-bracket; asserts trim_ok False + warning).
