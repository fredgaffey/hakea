# Ideation: diverge + screen (2026-11-20)

38 concepts: 26 can be expressed in the simulator and were screened, and 12 need a new
simulator capability. Results: 15 KEEP, 11 KILL, 12 NEEDS-CAPABILITY.

**Screening setup:** 5" Hakea base (2307 1300KV at 0.6 kg, PA6-CF blade, elastic axis at 0.43c,
AM32 duty_slew_max 2550). Each sample is pulsed at the motor's 1/rev capability
(`motor.capability(...).dQ_max_1p`). Each concept got a 48-sample Latin hypercube (60 for B06/B07),
1272 beam evaluations in total. Blade-shape concepts span 5–7".

**Feasible** means all checks ≤ 1, trim converged, and static pitch split ≤ 2°. The best feasible
point of 16 concepts then got a coarse FEA (`fea.static_centrifugal(coarse=True)`). Templates
(builder + base Design JSON + variables) and full results are in `concepts.json`. To reproduce,
run `screen.py screen | fea | export | table`.

## Headline findings (honest)

1. **The lag mode near resonance sets the ranking.** Every high-control point puts the lag mode
   at 0.72–0.85/rev. If the lag mode is restricted to ≤ 0.7/rev, the root families fall from
   0.17–0.22 to about 0.04. The blade-shape levers that lower rpm (wider chord, larger
   diameter, root cut-out) keep 0.18–0.25 at 0.58–0.62/rev, so they are the more robust lever.
   Screening needs a robustness term (backlog #3).
2. **The root fitting is the real constraint, not the flexure.** Every rod-in-TPU leader (C01,
   C05, C19, B01–B05) passes the beam checks but fails coarse FEA. The worst part is the
   PA6-CF root fitting or the rod ends, at 2.1–4.3 (C01 4.3, matching the earlier 4.8). The
   beam model's rigid connectors hide this.
3. **The torsion bar (C03) is the only root that passes both:** beam ctrl 0.076, FEA max 0.66.
   Its control is about one third of the rod family's.
4. **Fully FDM with 0 pauses:** only the symmetric PA6-CF cross (C08, ctrl 0.032) is
   beam-feasible, and its FEA plate utilisation is 3.2. Printed rods or bars (C06, C07) never
   pass creep. **A 0-pause design is not available yet. 1 pause (CF rod or bar) is needed.**
5. Fred's seeds:
   - **Laminate (C02):** killed with CF and steel. Retry with GF or aramid.
   - **Through-hub rod (C04):** killed as modelled (clamped). The free-floating version is N02.
   - **Graded whole prop (C05):** kept, ctrl 0.212.
   - **Torsion bar (C03):** kept, FEA-clean.
   - **Teeth (N01) and FACT→free-form (N04):** need capability.

## Screen table (best feasible point per concept)

| id | concept | persp. | feas/n | best ctrl | lag/rev | governing (beam) | FEA max util (part) | verdict |
|---|---|---|---|---|---|---|---|---|
| B01 | wide-chord blade 5.0", c/R 0.32 | propeller | 36/48 | **0.296** | 0.75 | fatigue flex | 2.7 root fitting | KEEP, CFD |
| B04 | camber/thickness (≈4414, 5.6") | propeller | 35/48 | **0.269** | 0.74 | motor | 2.7 rod end / fitting | KEEP, CFD |
| B06 | joint blade+root LHS | propeller | 36/60 | 0.247 | 0.78 | motor | crashed (rerun) | KEEP |
| B05 | root cut-out 0.29, hinge 8.7 mm, 5.9" | propeller | 44/48 | 0.237 | 0.72 | motor | 2.1 | KEEP |
| C19 | steel LE wire, outer blade (lag inertia) | critic | 26/48 | 0.220 | 0.78 | fatigue shank | 2.9 rod end / 1.9 fitting | KEEP |
| C05 | Fred: integral graded prop, rod into blade | Fred | 34/48 | 0.212 | 0.84 | motor | 3.4 rod end / 3.3 fitting | KEEP |
| B03 | twist law (α 4.9°, λ 0.14) | propeller | 40/48 | 0.206 | 0.78 | motor | 2.5 fitting | KEEP |
| B02 | inverse taper, 6.75", 10.6k rpm | propeller | 24/48 | 0.179 | **0.58** | motor | invalid (disconnected) | KEEP, CFD |
| C01 | TPU plate + 2 CF rods (reference) | baseline | 38/48 | 0.171 | 0.80 | motor | **4.3 root fitting** | KEEP |
| C18 | insect-vein X wires in TPU | biomimicry | 17/48 | 0.130 | 0.78 | motor | mesh failed | KEEP |
| C03 | Fred: torsion bar + TPU + strap | Fred | 17/48 | 0.076 | 0.75 | creep flex1 | **0.66 (passes)** | KEEP |
| C10 | FACT 5-wire set, CF | compliant | 14/48 | 0.067 | 0.74 | motor | mesh failed | KEEP |
| B07 | torsion-bar root + blade shape | propeller | 12/60 | 0.059 | 0.56 | motor | – | KEEP |
| C09 | cartwheel cross, TPU+rods | compliant | 27/48 | 0.041 | 0.90 | motor | not converged | KILL |
| C15 | chordwise offset instead of skew | rotor eng. | 18/48 | 0.041 | 0.97 | motor | 1.25 | KILL |
| C17 | spring-steel shim leaf | materials | 32/48 | 0.036 | 0.65 | ESC peak | 2.1 fitting | KILL |
| C08 | symmetric PA6-CF cross (0-pause) | compliant | 3/48 | 0.032 | 0.70 | motor | 3.2 plate | KEEP (backup) |
| C11 | trapezoid / remote-centre (LITF-like) | compliant | 7/48 | 0.019 | 1.05 | motor | – | KILL |
| C16 | raster-steered PA6-CF plate | print | 6/48 | 0.015 | 0.66 | creep | – | KILL |
| C02 | Fred: neutral-axis laminate (CF/steel) | Fred | 7/48 | 0.013 | 0.92 | motor | – | KILL |
| C13 | flex-beam + inclined pitch link | rotor eng. | 1/48 | 0.010 | 0.73 | ESC peak | – | KILL |
| C12 | TPU plate + radial strap | compliant | 13/48 | 0.004 | 0.67 | creep | – | KILL |
| C14 | thick TPU shear pad + strap (elastomeric) | rotor eng. | 18/48 | 0.003 | 1.03 | motor | – | KILL |
| C04 | Fred: rod hub-centre→blade, clamped | Fred | 14/48 | 0.001 | 0.95 | ESC peak | – | KILL (as modelled) |
| C06 | fully FDM printed rods | print | 0/48 | – | – | creep 2.6 | – | KILL |
| C07 | fully FDM printed torsion bar | print | 0/48 | – | – | creep 1.9 | – | KILL |

The FEA "rod end" utilisation is the interlayer check applied to a pultruded rod, which is
partly an artefact. The PA6-CF root-fitting values are real.

Pitch amplitudes for the KEEP leaders are 8–14° at 1/rev, with the lag mode close to 1/rev.

## Short write-ups

- **B01 / B04 / B05 / B02 (propeller shape).** These make the blade's lag inertia larger
  relative to its centrifugal stiffness, and lower the rpm at the same thrust. Hub-speed
  oscillation and lag response go up, and rod centrifugal stress stays about the same.
  - B02 (6.75", tip-heavy inverse taper, 10.6k rpm) gives 0.18 with a large resonance margin,
    at an FoM cost (0.69). Its FEA result was invalid (the solid model was disconnected, with
    57 mm displacement) and must be rerun.
  - Risks: BEM only, so CFD is needed. The flagged CFD candidates are B01, B04 and B02.
- **C19 (inertial LE mass).** A steel wire at 0.10c from 0.43R outward raises lag inertia and
  moves the chordwise CG forward. It reaches 0.22, but shank fatigue governs.
- **C05 (Fred, graded whole prop).** One part with the TPU root integral to the blade and a CF
  rod running from the root into the blade. It matches C01 in the beam model. In FEA it still
  fails, because the solid builder does not yet embed the rod into the blade (backlog #1).
- **C03 (Fred, torsion bar).** It is the only FEA-clean root. The bar on the skewed axis
  carries bending while the strap carries the pull. Creep in the TPU plate (flex1) limits how
  soft it can be.
- **C18 (insect vein).** Two CF wires crossing in the plate plane, each line meeting the hinge
  axis, so it is FACT-valid. It reaches 0.13 and is a credible distinct family. FEA meshing
  fails.
- **C10 (FACT wire set).** Five CF wires through the axis, ctrl 0.067. It is clean, but FEA
  meshing fails.
- **Kills.** C02 (laminate thickness trade), C04 (clamped radial rod = lag cantilever),
  C06/C07 (printed load paths creep), C11–C14 (the strap, shear pad and remote-centre families
  are too stiff in lag or have poor coupling), C15 (offset alone gives no coupling), C16
  (model-limited), C17 (FEA fails).

## Needs-capability concepts (see capability_backlog.md)

| id | concept | needs |
|---|---|---|
| N01 | Fred: interlocking teeth / dovetail TPU↔CF joint | joint FEA (tied/contact, tooth generator); WIP branch `worktree-agent-a4d870c2…` |
| N02 | Fred: free-floating blade-to-blade rod through the hub | cross-blade members, sliding hub support |
| N03 | Fred: graded TPU hardness / graded prop | spanwise graded material in beam + FEA |
| N04 | Fred: FACT seed → free-form FEA optimisation | FEA-in-loop objective (other agent) |
| N05 | butterfly / two-stage series flexure | member chains with intermediate bodies |
| N06 | swept blade (pitch-lag by sweep) | BladeStation sweep |
| N07 | aeroelastic bend-twist blade, no root pitch hinge | coupled section stiffness |
| N08 | lattice / auxetic hinge | homogenised lattice |
| N09 | continuous-fibre loops (CF/aramid/GF) | materials + curved members |
| N10 | tuned inertial tab / pendulum | lumped masses |
| N11 | asymmetric chordwise mass (speed-sensitive pitch) | per-blade definitions |
| N12 | corrugated resilin-like hinge | corrugated geometry |
