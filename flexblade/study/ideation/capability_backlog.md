# Capability backlog (ranked by expected value)

Ranking = (how much it could change the answer) × (how many concepts it unlocks) ÷ effort.
Concept ids refer to `concepts.json` / `CONCEPTS.md`.

| # | Capability | Unlocks | Why it ranks here | Effort |
|---|---|---|---|---|
| 1 | **Root-joint / connector modelling**: rod continuity into rigid parts (rod end embedded N mm into blade and hub solids), tied/contact interfaces, and the toothed/dovetail interface generator (finish unmerged branch `worktree-agent-a4d870c2…`) | N01 (Fred teeth), C01, C05, C19, B01–B06 | The beam model treats connectors as rigid. Every rod-root leader passes the beam checks but fails coarse FEA in the PA6-CF root fitting (utilisation 2.1–4.3). The FEA root fitting is now the binding constraint, and this is the only item that can change the top of the ranking. | M |
| 2 | **Blade sweep per station** (beam-line y-offset, also in geometry/aero) | N06, also bend-twist via sweep and CG/AC offset for pitch-lag | The lead asked for it. Sweep moves the blade CG and aerodynamic centre relative to the hinge, which is the classic helicopter alternative to a skewed hinge axis. | M |
| 3 | **Robustness metric in screening**: d(control)/d(lag stiffness) or the control ratio at ±15 % flexure stiffness / ±10 % TPU modulus | all | In every root family, the best control ratio comes from parking the lag mode at 0.75–0.85/rev. Below 0.7/rev the rod roots drop from 0.17–0.22 to about 0.04. Without a robustness term the optimiser will always pick near-resonant points that real TPU modulus scatter can wipe out. | S |
| 4 | **Member chains / intermediate rigid bodies and non-hub endpoints** | N05 (butterfly, two-stage), N02 (free-floating blade-to-blade rod) | These open new topology families that the current hub→member→blade star cannot represent. | M |
| 5 | **Material additions**: GF and aramid (Kevlar) laminate, continuous-fibre CF filament, TPU 85A/98A grades, PA12-CF | C02 variants, N09, N03 | A cheap data change. The laminate concept (C02) was only tested with CF and steel. | S |
| 6 | **Bend-twist coupled section stiffness** (off-axis fibre/raster → EI–GJ coupling) | N07, C16 done properly | Allows a pitch hinge built into the blade itself. C16 (raster-steered plate) shows that the beam model ignores the coupling. | M–L |
| 7 | **Spanwise graded material** (per-element material along a member; FEA per-element material) | N03 (Fred graded prop), C05 refinement | Removes stress steps at the TPU-to-rigid transition, which is where FEA fails. | M |
| 8 | **Lumped masses / inertias** attached to members or stations | N10 (tuned inertial tab) | C19 (steel LE wire) shows that inertia tuning is a strong lever, so a dedicated tuned mass could amplify pitch response. | S |
| 9 | **Per-blade (non-identical) blade definitions** | N11 (asymmetric chordwise mass) | A speculative mechanism. | S–M |
| 10 | FEA-in-the-loop harmonic objective / topology optimisation | N04 (Fred: FACT → free-form) | Already being done by the free-form optimiser agent. | (in progress) |
| 11 | Geometry robustness: HXT mesh fails for custom wire members crossing a plate (C10, C18) | FEA of C10/C18 | Without it, 2 of the top 12 concepts could not be FEA-checked. | S |
| 12 | Lattice homogenisation and corrugated member geometry | N08, N12 | Low expected value at this scale (cells ≈ nozzle width). | L |
