# Flexure manufacturing review: crossed 1095 shim hinge (10" v5, 7" v6, future 2–3")

Scope: how to make, finish, clamp and terminate the crossed-strip hinge in `build/10in_v5/BUILD.md` (8 × 0.10 × 9 mm 1095 laminae, 25.12 mm free, R1.0 jaw exits, epoxy-bonded 12 mm lands in PA6-CF, 1.5 mm aramid strand with stopper knots). Load case: σa 300–450 MPa, σm ~200 MPa at 100–150 Hz. Life arithmetic: at 125 Hz, 10⁷ cycles ≈ 22 h of spinning, 10⁸ ≈ 220 h, 10⁹ ≈ 2 200 h. A realistic hobby target is 10⁸; 10⁹ is "airframe life". Tags: **[S]** = from a cited source, **[H]** = handbook/standard engineering practice that I did not verify for thin strip; treat [H] numbers as ±30 %.

## 1. Cutting the strips
| Method | Edge state | Width control | Access (one-off/small batch) | Verdict |
|---|---|---|---|---|
| Tin snips / guillotine | Rollover + burr + micro-tears, local work hardening, camber | ±0.1–0.3 mm; needs filing | Free | OK only as a rough blank before finishing |
| Fibre laser (ns) | Recast/HAZ ~10–50 µm, dross; tempers 1095 locally [H]; HAZ 9–13 µm even in µ-cut NiTi [S6] | ±0.02–0.05 | PCBWay et al. laser-cut stainless at ±0.1 mm, min. part 10×10 mm [S9] | Must remove the HAZ afterwards; poor for 1095 |
| Photochemical etch | Burr-free, stress-free, slight cusp/undercut ~0.1–0.2 t [S1–S3] | ±0.025 mm (PEI) [S2] | Fotofab, PEI, Precision Micro (stainless 0.010–2.5 mm) [S1–S3], TMP (quick-turn prototypes) [S4]; PCBWay lists an etching service [S9] | Best for stainless (301/17-7) and for whole frets of many strips; 1095 not commonly offered |
| Wire EDM | Smooth to the eye, pitted 2–10 µm recast "white layer" [S7]; thin-hinge EDM fatigue matched literature data [S5] | ±0.005 mm, **whole stack in one pass** | Any job shop; clamp the 40 blanks between two 1–2 mm steel plates | Best route for matched 1095 widths; stone the recast off |
| Stamping/fine blanking | Shear zone + burr; work-hardened edge linked to early fatigue at 3–8 mil [S8] | excellent | Tooling cost; only for >1 000 parts | No |

**Edge finishing** (do it whatever the cutting method; the clamp exit corners carry the peak stress):
- Stoning: stack the laminae edge-up in a vice between hardened plates and stone **lengthwise only** (400 → 1000 → 2000 grit, or a hard Arkansas stone) until a 10× loupe shows no cross-scratches or nicks; break corners to r ≈ 0.02–0.03 mm (t/4). BUILD.md's "400 grit, break ~0.1 mm" is too coarse a finish and too large a chamfer for 0.10 mm stock (it removes 1/5 of the width-edge material at the corner).
- Vibratory tumbling (fine ceramic/porcelain media) removes HAZ and adds slight compressive stress [S10, practitioner advice], but 0.1 mm strips bend and tangle: tape them to a carrier or tumble them as an etched fret.
- Electropolishing: removes 5–20 µm and rounds edges; standard for stainless and NiTi [H]. On 1095 at HRC ~50 avoid acid pickling or plating (hydrogen embrittlement); if any acid step is used, bake 190 °C × 4 h [H].
- Shot peening does not work on shim this thin; it deforms rather than compresses the surface [S11].
- **Edge knock-down**: polishing over punched edges raised fatigue strength 10–20 % at 10⁵–10⁷ in sheet steel [S12]; SSAB says mechanically cut edges only reach good fatigue values if crack-like defects are removed [S13]. Marin surface factors at Su ≈ 1 800 MPa [H, Shigley]: ground ≈ 0.85, machined ≈ 0.62, hot-rolled/as-sheared ≈ 0.27–0.45. **Plan on 0.5–0.6 × polished for a snipped edge and 0.85–0.95 for a stoned/EDM-and-stoned edge.** With Goodman at σm 200 MPa, that difference decides whether σa 450 MPa passes at all.

## 2. Material condition
| Material | Hardness / Su | Fatigue (data) | Corrosion | Thin-gauge availability |
|---|---|---|---|---|
| 1095 blue-tempered | HRC 48–51 (McMaster "extra hard") [S14]; Su ≈ 1 700–1 900 MPa [H] | Polished bending ≈ 0.4–0.5 Su ≈ 650–850 MPa at 10⁷ [H]; decarburised skin lowers this | Rusts: pits become cracks. Keep oiled | 0.001"–0.010" rolls (Precision Brand, McMaster) [S14,S15]; Precision Brand lists Rb 90–100 for C-4: **check temper with a file before use** |
| 301 spring / full hard | ≥ 965 MPa yield FH; spring ≈ HRC 40+ [S16] | ≈ 0.3–0.35 Su ≈ 450–550 MPa [H] | Good | McMaster 0.0005"/0.001" foil in FH/spring tempers, 0.002" spring at HRC 40 [S16]; Goodfellow foil <0.01 mm is ±25 % thickness [S16] |
| 17-7PH CH900 | Su ≥ 1 655 MPa [S17] | **567 MPa at 10⁷** (datasheet) [S18]; RH950 534 MPa endurance, 690 MPa vapour-blasted [S18] | Good; better than hardenable Cr steels [S18] | Strip only in condition C ≤ 1 mm; CH900 needs a 482 °C age after cutting (easy in a toaster-oven-class kiln) [S18] |

Reading: on the strain figure of merit σ_f/E, 1095 (≈3.5e-3) is slightly ahead of 17-7PH CH900 (≈2.8e-3) and 301 (≈2.5e-3). The stainless grades buy corrosion immunity and etchability at ~15–25 % less fatigue strength. **Gigacycle caveat [H]**: steels above ~1 400 MPa have no true endurance limit; inclusion "fish-eye" failures cut the 10⁹ strength 10–20 % below 10⁷. Use 0.8 × the 10⁷ value for a 10⁹ design.

## 3. Clamping thin laminae in printed parts
- **The clamp exit is the failure site** (fretting + stress concentration); flexure pivots in industry most often fail in vibration testing [S19]. Keep the jaw exit **radiused (R ≥ 10 t; our R1.0 = 10 t is fine)** and make sure the bond, not plastic preload, holds the lamina, so the lip does not micro-slip.
- **Creep**: bolted preload into PA6-CF/PETG relaxes (PA6 also loses ~half its modulus when it absorbs water) [H]. Rely on (a) epoxy lands + (b) geometric capture, not preload. If bolts are used: heat-set brass inserts + a steel clamp plate spanning the land + a spring/Belleville washer, so creep loses only a little preload.
- **Print-pause embedding**: at the pause, drop a pre-bonded strip end ("cartridge": 8 laminae epoxied together over the land between two 0.2–0.3 mm steel doublers whose exit edges are radiused and stepped back 0.5 mm so the adhesive forms a tapered toe) into a pocket and print over it. Positive capture, no creep path, and the stress raiser at the exit is a steel/epoxy edge rather than plastic. Costs: one pause per part, and the cartridge jig must set the 105° cross and the 25.12 mm free length.
- **Adhesives**: toughened epoxies (3M DP460/DP420, Araldite 2015, Loctite EA 9466) bond degreased, scuffed steel well [H]. 3M DP8005 is an **acrylic** made for low-energy plastics, not an epoxy [S20]. Nylon 6 bonds more easily than 6/6; solvent wipe + abrasion matter [S21]; the Loctite HY4090 CA/epoxy hybrid was best on printed PA [S22]. PETG abrades and bonds acceptably with epoxy [H]. Do lap-shear coupons of your own print (ASTM D1002).
- **Compliant clamp edge**: a 0.3–0.5 mm adhesive toe fillet or a short soft lip (VHB/TPU) at the jaw spreads the curvature change; avoid a hard knife edge at the exit [H].
- **Fretting between laminae**: interleaf friction gives damping that is hard to control [S23], and separating plates improves linearity [S24]. Options: (1) the BUILD.md light-oil film (fine; renew it, and watch for red-brown fretting powder on 1095); (2) dry PTFE/MoS₂ spray film (no migration onto bonded lands); (3) 12.5 µm PTFE or Kapton interleaves (μ ≈ 0.1 / 0.3) add 0.1 mm per 8-stack and will not fit the 0.85 mm slot as drawn.

## 4. Alternative flexure materials (vs steel shim for 100 Hz, 10⁸–10⁹ cycles)
- **FDM PETG / nylon / TPU / PEEK**: PETG folding-endurance specimens failed at ~860 cycles [S25]. FDM tough PA hinges are fatigue-limited [S26]. Polymers at 100 Hz also self-heat (hysteresis) and creep under CF. **No.**
- **Moulded polypropylene living hinge**: Paulos & Yim's cyclic rotor used a PP flexure moulded into the blade [S27]. That works at very low strain and small scale, but there is no 10⁸ data. **No at 10".**
- **CF/GF laminate leaf**: UD-GF fatigue strain is ~0.4–0.6 % at 10⁷, and it is what full-size flexbeams use [S28] [H]. Hand-laid at 0.1–0.3 mm the voids and thickness scatter are large. **Maybe** at 10", only with coupon tests.
- **Markforged continuous CF**: fibre in an Onyx (nylon) matrix; little fatigue data exists, especially for short-fibre printed composites [S29]. Matrix creep and moisture uptake. **No.**
- **Titanium (β or 6-4 sheet)**: σ_f/E ≈ 4e-3 (better than steel) [H], but notch- and fretting-sensitive, thin sheet is hard to buy and gauls in the clamp. **Marginal; not worth it.**
- **NiTi superelastic**: ~0.4–0.6 % strain runout at 10⁷–10⁸ at zero mean, but only ~0.16 % at 4×10⁸ with mean strain ≥ 3 % [S30]. Stiffness is nonlinear and temperature-dependent (breaks the pitch-mode tuning). **No.**
- **Beryllium copper C17200**: 442 MPa rotating bending at 10⁷ (rod) [S31], E = 131 GPa → σ_f/E ≈ 3.4e-3, about equal to 1095. Non-magnetic, corrosion-proof, etches well, but Be dust is toxic (no grinding/stoning at home). **Equal, not better.**

## 5. Strap / strand termination
- **Fibre choice**: Vectran has essentially no creep and is the "upgrade from Kevlar" for sustained load (NASA) [S32]. Aramid creeps a little; HMPE (Dyneema) creeps the most: SK78 ≈ 0.033 %/month in mooring service, rupture at 70 °C within days in tests [S33]. Next to a hot motor bell under constant CF, **Dyneema is the wrong choice**. Vectran ≥ aramid.
- **Flex fatigue**: the strand bends through the hinge angle every cycle. Aramid is weak in axial compression / kink bands; Vectran and HMPE are much better in flex [H]. Keep the free strand where it bends near the hinge axis.
- **Terminations** [S34][H]: eye splice 90–100 % of line strength (easy in HMPE/Vectran, harder in aramid [S34]); wrap around a pin of D ≥ 4 d + epoxy-potted tail ≈ 70–90 %; potted end in a conical socket ≈ 60–80 %; **stopper knot ≈ 40–60 % (aramid knot strength is especially poor)**, and knots also bed in (creep) for hours. The model already notes that knot strength is not modelled. Pre-tension the strand at ~50 % of the CF load for a day before setting it "just taut".

## 6. Miniaturisation (2–3", t = 0.0125–0.025 mm)
- Foil: 301/304 spring/FH at 0.0005"/0.001" is buyable [S16]; 1095 below 0.001" is rare. Thickness tolerance is ±10 % (McMaster ±0.0004" at 0.004") to ±25 % (Goodfellow <0.01 mm) [S14,S16]. Stiffness ∝ t³, so **sort and measure each lamina** (micrometer, or weigh a strip of known area).
- Handling: cut on glass with a new scalpel and a steel rule; tape-carrier the foil (Kapton tape) for cutting/stoning; tweezers with plastic tips. Snipping is hopeless at 12.5 µm.
- **Etched monolithic sheet**: one sheet cannot hold a crossed-strip pivot (the strips lie in different planes). Practical alternatives: (a) etch a fret of strips with tabs and fold/insert, as model-railway photo-etch frets do; (b) a planar remote-centre pivot (isosceles-trapezoid/"butterfly" type, Henein [S19]) cut in one plane; (c) pop-up laminate (CF/Kapton/CF, Harvard RoboBee-style) where 7.5–25 µm polyimide flexures run at ~120 Hz [H]. Their lives are hours (~10⁶–10⁷), not 10⁸.
- At this scale etching (±0.025 mm on a 2–3 mm strip ≈ ±1 % width) beats any hand method.

## 7. Precedents
- Instrument/aerospace cross-spring pivots (Bendix/Riverhawk Free-Flex, C-Flex) use stainless crossed leaves brazed/welded into metal sleeves and are rated for unlimited life at small angles [H]. CSEM's space pivot was designed for 6×10⁶ cycles at ±7.5° and tested >1.4×10⁶ [S19]. Wire-EDM monolithic hinges perform per literature fatigue data [S5].
- Watches: monolithic silicon/steel flexure oscillators (e.g. Zenith Defy Lab, CSEM/EPFL [S35]) run at 10⁹-class cycle counts because they are monolithic: no clamp and no fretting.
- Rotors: full-size bearingless tail rotors use composite flexbeams/straps (GF/CF/Kevlar), with bolt holes moved to thickened roots because holes weaken the beam [S28]. I found no documented RC spring-steel flex hub. Paulos/Yim used moulded flexures [S27].
- Common lesson: **monolithic or bonded/brazed metal-to-metal roots win**; clamped thin leaves fail at the clamp.

## (a) Recommended build process, 10" hinge
1. Buy one roll of 0.004" blue-tempered 1095; file-test the hardness; mark the rolling direction; cut 40 blanks at 11–12 mm with snips.
2. Stack the 40 blanks between two 2 mm steel plates; wire-EDM (or surface-grind) the stack to 9.00 mm. All widths match within ±0.01 mm.
3. Stone the stack edges lengthwise at 600/1000/2000 grit; break corners to ~0.02 mm; inspect at 10×; reject nicked laminae.
4. Degrease with IPA; scuff the lands only (240 grit, lengthwise); oil or dry-PTFE the free span only.
5. Bond each 8-stack land between 0.25 mm steel doublers (exit edge radiused, adhesive toe) in a jig that fixes the 25.12 mm span, the 105° cross and the 2.7 mm gap. Use DP460/Araldite 2015 and a 24 h RT + 2 h 50 °C cure.
6. Print PA6-CF with land pockets; at the single pause, drop in the cartridges and print over them (or bond them into the existing slots).
7. Strand: Vectran or aramid; pin-wrap D ≥ 6 mm with an epoxy-potted tail instead of stopper knots; bed in for 24 h at ~50 % CF load, then set taut.
8. Store dry, keep oiled; re-check the pluck frequency every 10 h (k drift > 2 % → replace the strip set).
9. Before flight: run one sacrificial strip pair at the design σa on a shaker or crank at 100 Hz to 10⁷ (~28 h), and look at the clamp exit.

## (b) Top 3 manufacturing risks
1. **Edge defects at the clamp exit** (snipped/filed edge ≈ 0.5–0.6 × polished fatigue, which fails the Goodman margin). → EDM/grind the stack, stone lengthwise, inspect at 10×, test one fatigue coupon.
2. **Clamp-exit fretting + plastic creep/moisture** (micro-slip at the lip, PA6 softening when wet). → bonded steel doublers/cartridges, no preload into plastic, dry the PA6-CF, look for red fretting powder.
3. **Strand termination** (aramid stopper knot ≈ 40–60 % strength, bedding creep changes the strap/strip load share). → pin-wrap + potting or splice in Vectran, pre-tension, inspect knot/slot wear.

## (c) Is there a better construction?
Not a different concept: crossed spring-steel leaves are the established high-cycle pivot. The improvement is in the **roots**. Make the steel stack and its end doublers a bonded metal cartridge (Free-Flex practice) and capture it in the print, rather than gluing bare laminae into plastic slots. Second, choose 17-7PH CH900 (567 MPa at 10⁷, etchable, no rust) if a vendor etches it cheaply, accepting about 15 % less fatigue strength than well-finished 1095. Third, at 2–3" consider an etched planar remote-centre pivot or an etched fret, because hand-made crossed foils are impractical.

## Sources
[S1] fotofab.com/products/shims/ · [S2] photofabrication.com/library/photochemically-etched-shims/ · [S3] precisionmicro.com/?p=66 · [S4] TMP brochure (slideshare MichelleLingle1/tmp-brochure-2016)
[S5] Henein et al., SPIE 3834 (1999), "Fatigue failure of thin wire-EDM flexure hinges" · [S6] Wamai et al., HAZ of laser vs µEDM NiTi (ResearchGate 317429422)
[S7] fab.cba.mit.edu/classes/865.21/people/zach/fatigueexploration.html · [S8] US 9121507 "Springs and methods of forming same" · [S9] pcbway.com/rapid-prototyping/sheet-metal/… and …/chem-treatment/etching/
[S10] eng-tips thread 147722 (laser blanking .005" 301) · [S11] eng-tips thread 125138 (high-fatigue spring material) · [S12] SAE 2020-01-1072 and J. Soc. Mat. Sci. Japan 62(12) 764 (punched vs polished electrical steel)
[S13] ssab.com trailer design guide, edge quality · [S14] mcmaster.com blue-tempered spring-steel shim / 1095 HRC48–51 · [S15] penntoolco.com Precision Brand blue-tempered shim
[S16] mcmaster.com spring-tempered stainless / stainless foil; goodfellow.com AISI 301 foil; ulbrich.com 301 · [S17] ulbrich.com 17-7PH · [S18] Cleveland-Cliffs 17-7PH product data (Table 3)
[S19] Henein et al., ESMATS 2003, "Flexure pivot for aerospace mechanisms" (infoscience.epfl.ch 306417) · [S20] 3M DP8005 TDS · [S21] ITW Plexus "Structural bonding of nylon" (Curbell) · [S22] MDPI Polymers 17(22) 3020
[S23] US 4456232 (leaf friction control) · [S24] US 4792005 (scale flexure) · [S25] Balderrama-Armendariz, SFF 2016 paper 188 · [S26] EPFL infoscience 276486 (AM of hinges, Materials & Design 2020)
[S27] Paulos & Yim, IROS 2013 / JGCD 2018 (modlabupenn.org) · [S28] US 5690474, 4306837, 6805948 (flexbeam/flexstrap patents) · [S29] Polimi ECCM 2022 Onyx fatigue · [S30] Cao et al. 2020 Nitinol HCF (lbl.gov/ritchie)
[S31] Materials 16(2) 815 (2023), C17200 RBF · [S32] NASA NTRS 20050040773 (Vectran vs Kevlar) · [S33] samsonrope.com "Understanding creep"; riggingdoctor.com "Not all Dyneema is created equal"
[S34] yalecordage.com engineering data / fiber selection; animatedknots.com rope materials · [S35] Thalmann et al. 2018, flexure-pivot oscillator (infoscience.epfl.ch 264022).
