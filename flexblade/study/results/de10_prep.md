# DE10 prep: light wide hollow CF blade, consistent mass/EI/GJ

Mechanism: `Layup.core_fraction` (core_skin only; core E, G, rho x fraction, skins full), so a hollow
PA6-CF shell with optional infill gets mass, EI, GJ from the layup. New variable `blade.chord_scale`
(set_design rescales stations by new/old; `Blade.chord_scale` records the cumulative factor).
`--blade-vars` adds blade.chord_scale [0.9,1.6], blade.layup.skin_thickness [0.4,1.2] mm (log),
blade.layup.core_fraction [0,1] to tt_strap_steelx. Init designs MUST have a core_skin blade layup
(material pa6_cf, skin_material pa6_cf, skin_thickness 0.0008, core_fraction ~0.18). v8/v11 are solid
(v11 uses pa6_cf@rho=0.59), so convert them first (blade.layup only), else the new paths are absent.

## Sanity: v10m, chord x1.3, skin 0.8 mm, core_fraction 0.18 (section at 0.65R, t = 4.4 mm)
- mass 0.60 x solid(x1.3) = 1.01 x solid(x1.0); f=0 (pure shell) gives 0.51.
- EIy 0.83, EIz 0.73, GJ 0.83 of solid x1.3 (= 2.4x, 2.1x, 2.4x of solid x1.0).
- nominal eval (presets re-solved, 6 it, T0.7/1.0/1.4, J 3.76e-6): cr_pl 0.120, res margin 0.156
  (pitch antisym+hub 0.91/rev vs 1P), hard viol harmonic_jitter 1.105. Skin 0.8 mm (>= 2 perimeters): OK.

## Command (cwd flexblade/; not in jobs.txt; convert init designs first, then point --init-designs/--base-design at the converted files)
steelx_de10 PYTHONPATH=. ~/venv-flexblade/bin/python study/run_study.py --hosts "$H" --sizes 10 --concepts tt_strap_steelx --maxiter 30 --popmult 8 --seed 101 --redo --resolve-presets 6 --robust 0.7,1.0,1.4 --objective pulse_limited --ground-resonance --criterion mech_band --mech-margin 0.07 --goodman --one-sided --blade-vars --j-hub "10:2.5e-6,3.76e-6,5.49e-6" --init-designs study/designs/10in_tt_strap_steelx_v8.json,study/designs/10in_tt_strap_steelx_v11.json --init-frac 0.3 --base-design study/designs/10in_tt_strap_steelx_v8.json,study/designs/10in_tt_strap_steelx_v11.json >> study/campaign/steelx_de10.log 2>&1
