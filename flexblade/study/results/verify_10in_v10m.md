# Verification: 10in_tt_strap_steelx_v10m (v8, cross_angle 120, re-solved presets, cr_pl 0.124)
Tools: verify_10in_v5.py ccx|fit, ccx_preset_strip.py strip, two_per_rev.py (DN env var selects the design), tol.py N=0, robust.variants T0.7/1/1.4.
## 1. ccx vs beam
- Unloaded (x8 laminae): k_flap 1.13, kx 1.11, ky 1.03, kz 1.05 PASS; k_cross 1.80 FAIL (ccx 72.9x8 vs beam 324). k_hinge ~0 (axis noise, beam axis 0.59 deg off design; v8 0.45).
- Loaded preset strip b2.flex0 (beam vs ccx peak clamp-edge surface stress), hub/blade: T0.7 171 vs 104 (ccx/beam 0.64) / 88 vs 79 (0.95); T1.0 254 vs 109 (0.45) / 122 vs 83 (0.71).
  Beam is conservative; T0.7 blade passes within 20 %, hub and all of T1.0 fail the 20 % band (same pattern as v8: ccx stress barely moves T0.7->T1).
## 2. Two-per-rev (T1.0, pulse s*; v8: 11 N / 0.45 N)
- precone 0: 1P vertical hub F 10.3 N (v8 11.0); 2P in-plane (1P/3P lateral) 0.23 / 0.41 N (v8 0.20/0.20); 2P vertical 2.2 N; 1P lateral-fixed 3.2 N (2P lateral).
- precone 1.3: 1P vertical 10.8 N (v8 1.55, worse), 2P in-plane 0.29 / 2.71 N, 2P vertical 2.8 N.
- Per-blade 1P flap/lag mismatch (precone 0): 0.64/3.04 vs 2.70/3.65 deg (flap mismatch 2.1 deg; v8 2.57 vs 0.36). precone 1.3: 2.20/3.44 vs 1.63/5.05.
## 3. Fatigue / LTB / manufacturability
- Worst fatigue_stress (Goodman util, nominal): T0.7 1.064 (FAIL), T1.0 0.852, T1.4 0.618. Worst strip_ltb: 0.326 / 0.395 / 0.478 (all <1).
- tol.py nominal: cr_pl 0.1240, resonance margin 0.080 (need 0.15) -> hard resonance violation 0.465 (0.94/rev vs 1P).
- 120 deg crossing: strip w 9.0 mm, t 0.8 mm stack, cross_gap 2.71 mm (v8 had 0.0015 m class gap: edge gap 2.71 mm < 2 walls), so fittings open the slot on the crossing edge (U jaws, strip overhangs jaw 0.5 mm). slot 9.2 x 0.85 mm, wall 1.2 mm. Min fitting gap 4.67 mm at lag 8/pitch 12 (3.07 at 12/16; v8-like), no interference. Printable but with open U jaws.
- Verdict: marginal. T0.7 fatigue >1, resonance margin 0.08, k_cross ccx 1.8x beam; hub-force at precone 1.3 poor.
