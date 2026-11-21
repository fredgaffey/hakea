# 10in trim non-convergence (viol convergence=2.00)
- Case 10in_tt_strap_steelx has NO rows in evals.jsonl.gz (no tt_strap cases at all). Used 10in_cf_flexure instead: 161 of 3768 rows have convergence>=1 (10in_tpu_monolithic 1214/2296, tpu_rod 126, tpu_split 22).
- Reproduced 3 cf_flexure x vectors (base_design(10,'cf_flexure') + set_design); all fail static_trim.
- Fixed-rpm scan (design 1): thrust is monotone in rpm (3.6 N @3950, 11.8 @6715, 15.3 @7900, 19.9 @8690, W = 16.6 N). Not stall/flap non-monotonicity.
- The bracket is NOT the problem: the inner nonlinear-static Newton does not converge at the centre rpm (cold start), nor on warm-started walks from low rpm (converges to ~2400 rpm, fails above, residual 0.03-0.4). Thrust at the failed iterate still crosses W (~8.6k rpm).
- Design 2 converges at 3950/5530/8690/9875 but not 6715/7900: failure islands, i.e. static-solve robustness (soft flexure torsion/pitch collapse), not outer bracketing.
- Tried: warm-start only from converged points + low-rpm continuation skipping failed points. No gain (all 3 still fail), +30 solves. Reverted; no code change, no regression test.
- Next: improve solve_static (load stepping on the aero load, not only lambda; line search/arc-length), or keep treating as infeasible (viol 2.0).
