#!/usr/bin/env python3
"""Build fatigue_creep.json. Every value below was read from the cited source text
(PDF text extraction or Europe PMC full-text XML). Unit conversions only (psi->MPa)."""
import json

PSI = 0.006895
OUT = "/tmp/claude-1000/-home-fred-vehicles-hakea/d3cb2918-65fe-44b5-aae3-7340f9b6d1ce/scratchpad/research/fatigue_creep.json"

# ---- sources
EZEH19 = ("Ezeh O.H., Susmel L. (2019) Fatigue strength of additively manufactured polylactide (PLA): "
          "effect of raster angle and non-zero mean stresses. International Journal of Fatigue 126:319-326. "
          "doi:10.1016/j.ijfatigue.2019.05.014 (accepted manuscript, White Rose eprint 146276)")
EZEH19_URL = "https://eprints.whiterose.ac.uk/id/eprint/146276/1/3D-printed%20PLA_plain%20Fatigue_IJFatigue.pdf"
EZEH_SPEC = ("printed FDM; Ultimaker 2 Extended+, flat on bed, 100% infill, 0.1 mm layer, 240 C nozzle, "
             "+/-45 deg raster; dog-bone 6 mm net width, 3 and 5 mm thick; axial, 10 Hz, run-out 2e6")
CESNIK = ("Cesnik M., Slavic J. (2024) Frequency-Dependent Fatigue Properties of Additively Manufactured PLA. "
          "Polymers 16(15):2147. doi:10.3390/polym16152147")
CESNIK_URL = "https://www.ebi.ac.uk/europepmc/webservices/rest/PMC11313685/fullTextXML"
FTOUTOU = ("Ftoutou E., Allegue L., Marouani H., Hassine T., Fouad Y. (2024) Modeling of Effect of Infill Density "
           "Percentage on Rotating Bending Fatigue Behavior of Additive-Manufactured PLA Polymers. "
           "Materials 17(2):471. doi:10.3390/ma17020471")
FTOUTOU_URL = "https://www.ebi.ac.uk/europepmc/webservices/rest/PMC10820292/fullTextXML"
RENDAS = ("Rendas P., Imperadeiro A., Martins R.F., Soares B.A.R. (2024) High-Cycle Fatigue Behaviour of "
          "Polyetheretherketone (PEEK) Produced by Additive Manufacturing. Polymers 16(1):18. doi:10.3390/polym16010018")
RENDAS_URL = "https://www.ebi.ac.uk/europepmc/webservices/rest/PMC10781079/fullTextXML"
RODR = ("Rodrigues A., Branco R., Franco M., Silva R., Malca C., Martins R.F. (2026) Mechanical, Fatigue, and Thermal "
        "Characterization of ASA, Nylon 12, PC, and PC-ABS Manufactured by Fused Filament Fabrication (FFF). "
        "Polymers 18(2):302. doi:10.3390/polym18020302")
RODR_URL = "https://www.ebi.ac.uk/europepmc/webservices/rest/PMC12845617/fullTextXML"
RODR_SPEC = ("printed FDM (Stratasys filaments), XY flat, 100% infill, +/-45 deg raster, 0.254 mm layer; "
             "axial tension-tension, 10 Hz, staircase method, run-out 1e6")
YANKIN = ("Yankin A., Serik G., Danenova S., Alipov Y., Temirgali A., Talamona D., Perveen A. (2023) Optimization of "
          "Fatigue Performance of FDM ABS and Nylon Printed Parts. Micromachines 14(2):304. doi:10.3390/mi14020304")
YANKIN_URL = "https://www.ebi.ac.uk/europepmc/webservices/rest/PMC9960376/fullTextXML"
TEREKH = ("Terekhina S., Tarasova T., Egorov S., Skornyakov I., Guillaumat L., Hattali M.L. (2020) The effect of build "
          "orientation on both flexural quasi-static and fatigue behaviours of filament deposited PA6 polymer. "
          "International Journal of Fatigue 140:105825. doi:10.1016/j.ijfatigue.2020.105825")
TEREKH_URL = "https://www.ebi.ac.uk/europepmc/webservices/rest/PMC7357525/fullTextXML"
AHMAD = ("Ahmadifar M., Benfriha K., Shirinbayan M. (2023) Thermal, Tensile and Fatigue Behaviors of the PA6, Short "
         "Carbon Fiber-Reinforced PA6, and Continuous Glass Fiber-Reinforced PA6 Materials in Fused Filament "
         "Fabrication (FFF). Polymers 15(3):507. doi:10.3390/polym15030507")
AHMAD_URL = "https://www.ebi.ac.uk/europepmc/webservices/rest/PMC9919798/fullTextXML"
CIGANAS = ("Ciganas J., Kalinauskis T., Cigane U. (2026) Thermo-Mechanical and Fatigue Behavior of 3D-Printed PA12 CF15 "
           "for Engineering Application. Polymers 18(5):563. doi:10.3390/polym18050563")
CIGANAS_URL = "https://www.ebi.ac.uk/europepmc/webservices/rest/PMC12986956/fullTextXML"
WANG = ("Wang S., Tang S., He C., Wang Q. (2023) Cyclic Deformation and Fatigue Failure Mechanisms of Thermoplastic "
        "Polyurethane in High Cycle Fatigue. Polymers 15(4):899. doi:10.3390/polym15040899")
WANG_URL = "https://www.ebi.ac.uk/europepmc/webservices/rest/PMC9958809/fullTextXML"
SAFAI = ("Safai L., Cuellar J.S., Smit G., Zadpoor A.A. (2019) A review of the fatigue behavior of 3D printed polymers. "
         "Additive Manufacturing 28:87-97. doi:10.1016/j.addma.2019.03.023")
SAFAI_URL = "https://repository.tudelft.nl/file/File_435ba53d-a2c0-4562-ad80-8a3385ab8a65"
BAYER_PMD = ("Bayer Corporation (2000) Engineering Polymers Part and Mold Design - Thermoplastics: A Design Guide. "
             "Pub. KU-GE028 (info current April 2000). Table 3-3 / Table 4-1 and Chapter 3 Long-Term Mechanical Properties")
BAYER_PMD_URL = "https://kompozit.org.tr/wp-content/uploads/2021/11/A_Design_Guide_Part_and_Mold_Design_Engi.pdf"
BAYER_SNAP = ("Bayer MaterialScience, Snap-Fit Joints for Plastics - A Design Guide, Table 2 'General guide data for "
              "the allowable short-term strain for snap joints'")
BAYER_SNAP_URL = "https://cdn.sparkfun.com/assets/home_page_posts/1/4/1/0/Plastic_Snap_fit_design.pdf"
DELRIN = ("DuPont, Delrin acetal resin Design Guide - Module III (reorder no. H-57472), Table 2 'Typical Properties of "
          "Delrin Acetal Resins' and Fatigue Resistance section (Figs 20-21)")
DELRIN_URL = "https://www.delrin.com/wp-content/uploads/2023/10/Delrin-Design-Guide-Mod-3.pdf"
VOLGERS = ("Volgers P., Shojaei A.K. (DuPont Transportation and Advanced Polymers), Fatigue Behavior of Engineering "
           "Polymers and Its Numerical Implementation, SIMULIA Science in the Age of Experience conference paper (c.2018)")
VOLGERS_URL = ("https://www.3ds.com/assets/invest/2024-01/"
               "2218-saoe-fatigue_behavior_of_engineering_polymers_and_its_numerical_implementation.pdf")


def F(quantity, value, *, cycles=None, duration_h=None, R=None, T=None, specimen, source, url, notes,
      measure=None, freq=None, fit=None, load_mode=None):
    d = {"quantity": quantity, "value": value, "cycles": cycles, "duration_h": duration_h,
         "stress_ratio_R": R, "temperature_C": T, "specimen": specimen, "source": source, "url": url,
         "notes": notes}
    if measure is not None:
        d["stress_measure"] = measure
    if load_mode is not None:
        d["load_mode"] = load_mode
    if freq is not None:
        d["frequency_Hz"] = freq
    if fit is not None:
        d["fit"] = fit
    return d


out = {}
out["_meta"] = {
    "compiled": "2026-11-20",
    "conventions": ("Strains are fractions (0.04 = 4%). stress_measure says whether a stress value is an "
                    "amplitude, range (max-min), or max stress. Extra quantities beyond the brief: "
                    "'snap_fit_strain_allow' (permissible short-term strain, single snap), "
                    "'relaxation_modulus_MPa', 'fatigue_strain_max' (max strain in a strain-controlled test), "
                    "'sn_curve_fit' (value null, parameters in 'fit'). No values were derived except unit "
                    "conversions (psi->MPa, % -> fraction)."),
    "warnings": ("Almost all FDM fatigue tests run at 1-100 Hz; self-heating at ~300 Hz is a real risk "
                 "(see POM Volgers note: still vs circulating air changed allowable peak stress 44 -> 55 MPa). "
                 "Cesnik & Slavic report the opposite trend (longer life at higher frequency) when temperature "
                 "is held constant (+3 C rise) in a 200-800 Hz vibration test. Run-out in most FDM studies is "
                 "only 1e6-2e6 cycles, not 1e7+."),
}

# ---------------- PLA
pla = []
for raster, uts, rows in [(0, 42.6, [(-1, 7.7, 10.4), (-0.5, 8.9, 10.1), (0, 7.4, 6.1), (0.3, 7.0, 5.1)]),
                          (30, 40.9, [(-1, 4.3, 4.5), (0, 6.9, 4.7)]),
                          (45, 42.5, [(-1, 6.5, 6.3), (0, 7.5, 4.7)])]:
    for R, k, sa in rows:
        pla.append(F("fatigue_strength_MPa", sa, cycles=2e6, R=R, T=None, measure="amplitude", freq=10,
                     load_mode="axial", specimen=EZEH_SPEC + f"; manufacturing angle {raster} deg",
                     source=EZEH19, url=EZEH19_URL,
                     notes=(f"Table 1: sigma_A,50% (median endurance limit amplitude extrapolated to 2e6), "
                            f"inverse slope k={k}, UTS={uts} MPa. Room temperature. Note: Table 1 lists angles "
                            f"0/30/45 deg while the methods text says 0/45/90 deg.")))
pla.append(F("fatigue_ratio", 0.10, cycles=2e6, R=None, measure="max", specimen="printed FDM (re-analysis of own + literature data)",
             source=EZEH19, url=EZEH19_URL,
             notes=("Recommended conservative design curve for AM PLA: negative inverse slope 5.5, endurance limit "
                    "sigma_MAX = 0.1*UTS at 2e6 cycles (>=95% probability of survival); mean stress handled by "
                    "using max stress in cycle (eq. 6 and conclusions).")))
for fr, b, sf in [("250-700", -0.148, 48.60), ("450-700", -0.091, 22.38), ("300-400", -0.167, 56.93), ("250-350", -0.363, 571.3)]:
    pla.append(F("sn_curve_fit", None, R=-1, measure="amplitude", load_mode="random vibration bending (resonant)",
                 specimen=("printed FDM, Prusa i3 MK3S+, PLA (Plastika Trcek), 100% rectilinear, 45 deg raster, 0.2 mm "
                           "layer, 220 C; printed in y-direction so layer normal is PARALLEL to the stress (interlayer loading)"),
                 source=CESNIK, url=CESNIK_URL, fit={"form": "sigma_a = Sf * N^b", "Sf_MPa": sf, "b": b,
                                                     "natural_freq_range_Hz": fr},
                 notes=("Table 3 frequency-dependent Basquin parameters from narrow-band random vibration tests on "
                        "an electrodynamic shaker. Measured fatigue-zone temperature rise 3 C. Authors: higher "
                        "loading frequency prolonged life at constant temperature; low-frequency (4-30 Hz) data "
                        "are conservative at high frequency. R not stated; random zero-mean excitation assumed "
                        "(recorded R=-1 as zero-mean vibration).")))
pla.append(F("sn_curve_fit", None, R=-1, freq=100, measure="amplitude", load_mode="rotating bending",
             specimen="printed FDM PLA, 100% infill (also 25/50/75% in paper), 200 C nozzle, 60 C bed",
             source=FTOUTOU, url=FTOUTOU_URL,
             fit={"form": "log10(Nf) = CB1 - CB2*log10(sigma)", "CB1": 9.974, "CB2": 3.3825, "R2": 0.999,
                  "infill_pct": 100},
             notes=("Table 3. Other infill: 75% CB1 9.740 CB2 3.2922; 50% 8.420/2.5953; 25% 6.206/1.4678. "
                    "Test stresses 14, 35, 49, 70 MPa peak bending; 100 Hz loading frequency.")))
pla.append(F("fatigue_strength_MPa", 15.5, R=-1, measure="amplitude", specimen="printed FDM PLA (Kiani et al., cited)",
             source=RENDAS + " - Table 5, citing Kiani et al. [61]", url=RENDAS_URL,
             notes="Secondary citation. sigma_u 61.3 MPa, sigma_m 0, sigma_f 15.5 MPa; cycle count of run-out not given in table. PLA/PCL 80/20: 13.6 MPa (sigma_u 33.5)."))
out["pla"] = pla

# ---------------- ABS
out["abs"] = [
    F("snap_fit_strain_allow", 0.018, T=23, specimen="injection moulded (Bayer Lustran ABS)", source=BAYER_PMD,
      url=BAYER_PMD_URL, notes="Permissible short-term strain, single snap; 'for frequent separation and rejoining, use about 60% of these values'."),
    F("sn_curve_fit", None, R=-1, measure="amplitude", load_mode="rotating bending",
      specimen="printed FDM ABS, Ultimaker S3/S5, horizontal, 0.15 mm layer, 220 C",
      source=YANKIN, url=YANKIN_URL, fit={"form": "sigma_a = A * N^b", "log10A_range": [2.12, 2.22], "b_range": [-0.202, -0.176]},
      notes=("Table 5 Basquin parameters over 9 Taguchi runs (nozzle dia, infill pattern, speed). Data are mostly "
             "low-cycle (~1e4 cycles); text: ABS withstood about 26.5 MPa at 9000 cycles. Not valid for 1e7 extrapolation without care.")),
]

# ---------------- ASA
out["asa"] = [
    F("fatigue_strength_MPa", 8.0, cycles=1e6, R=0.05, freq=10, measure="range", load_mode="axial",
      specimen=RODR_SPEC + "; Stratasys ASA", source=RODR, url=RODR_URL,
      notes="Table 11: run-out (infinite life) stress range at 25% of UTS (UTS 31.7 MPa, E 1423 MPa). Porosity ~8%."),
    F("fatigue_ratio", 0.25, cycles=1e6, R=0.05, measure="range", specimen=RODR_SPEC + "; Stratasys ASA",
      source=RODR, url=RODR_URL, notes="Stress range / UTS at run-out (text: infinite life below 25% x sigma_R)."),
    F("snap_fit_strain_allow", 0.019, T=23, specimen="injection moulded (Bayer Centrex ASA)", source=BAYER_PMD,
      url=BAYER_PMD_URL, notes="Single snap; use ~60% for repeated."),
]

# ---------------- PC
pc = [
    F("fatigue_strength_MPa", 3.6, cycles=1e6, R=0.05, freq=10, measure="range", load_mode="axial",
      specimen=RODR_SPEC + "; Stratasys PC", source=RODR, url=RODR_URL,
      notes="Table 11: run-out at 7% of UTS (UTS 57.1 MPa, E 1549 MPa). High porosity 11.2% blamed for poor fatigue."),
    F("fatigue_ratio", 0.07, cycles=1e6, R=0.05, measure="range", specimen=RODR_SPEC + "; Stratasys PC",
      source=RODR, url=RODR_URL, notes="Infinite life only below 7% x sigma_R."),
    F("fatigue_strength_MPa", 1.6, R=-1, measure="amplitude", specimen="printed FDM PC (Puigoriol-Forcada et al., cited)",
      source=RENDAS + " - Table 5, citing Puigoriol-Forcada et al. [56]", url=RENDAS_URL,
      notes="Secondary; sigma_u 48.7, sigma_m 4.4 and sigma_f 1.5 'estimated from the S-N curve fit', Goodman-corrected to R=-1: 1.6 MPa."),
    F("snap_fit_strain_allow", 0.04, T=23, specimen="injection moulded (Bayer Makrolon PC; Apec high-heat PC also 4%)",
      source=BAYER_SNAP, url=BAYER_SNAP_URL, notes="Single snap; use ~60% for frequent separation. Makrolon 10% GF 2.2%, 20% GF 2.0%. Same values in Bayer Part and Mold Design Table 3-3."),
    F("creep_modulus_MPa", round(220000 * PSI), duration_h=1000, T=23, specimen="injection moulded (Makrolon PC)",
      source=BAYER_PMD, url=BAYER_PMD_URL,
      notes="Worked example from Fig. 3-10 isochronous curves: 2,800 psi (19.3 MPa) for 1000 h -> 1.2% strain, apparent modulus 220,000 psi (converted)."),
    F("creep_modulus_MPa", round(166000 * PSI), duration_h=1000, T=40, specimen="injection moulded (Makrolon PC)",
      source=BAYER_PMD, url=BAYER_PMD_URL,
      notes="Fig. 3-27 example: 2,900 psi (20 MPa) for 1000 h at 40 C -> apparent modulus 166,000 psi vs instantaneous 350,000 psi."),
    F("creep_modulus_MPa", round(98000 * PSI), duration_h=10000, T=40, specimen="injection moulded (Makrolon PC)",
      source=BAYER_PMD, url=BAYER_PMD_URL,
      notes="Example 3-6: 4,900 psi on 10,000 h 40 C curve -> 'roughly 5%' strain, apparent modulus 98,000 psi."),
    F("relaxation_modulus_MPa", round(233333 * PSI), duration_h=1000, T=23, specimen="injection moulded (Makrolon PC)",
      source=BAYER_PMD, url=BAYER_PMD_URL,
      notes="Table 3-7, held strain 1.2%: stress 3,750 psi at 0.1 h (Er 312,500 psi), 2,800 psi at 1e3 h, 2,500 psi at 1e4 h (208,333 psi), 2,200 psi at 6e4 h (183,333 psi)."),
    F("creep_strain_allow", None, T=23, specimen="injection moulded (Bayer resins, general)", source=BAYER_PMD, url=BAYER_PMD_URL,
      notes=("No numeric long-term permissible strain table in this guide. Text: 'Designs that see multiple applications of "
             "an applied load should stay below 60% of these values' (short-term strain limits); general stress limits "
             "'such as 25% of the published tensile yield stress' carry large safety factors; use safety factor >= 2.0. "
             "Fig. 3-13 gives dynamic (1e7 cycle) design limit lines for Bayblend T85 PC/ABS only graphically.")),
]
out["pc"] = pc

# ---------------- PC/ABS
out["pc_abs"] = [
    F("fatigue_strength_MPa", 4.7, cycles=1e6, R=0.05, freq=10, measure="range (as stated 'fatigue limit stress')",
      load_mode="axial", specimen=RODR_SPEC + "; Stratasys PC-ABS", source=RODR, url=RODR_URL,
      notes="Infinite life below 15% x sigma_R (UTS 31.43 MPa). Text compares to 45.2 MPa reported for (non-FFF) PC-ABS in ref [20]."),
    F("snap_fit_strain_allow", 0.025, T=23, specimen="injection moulded (Bayer Bayblend PC/ABS)", source=BAYER_SNAP,
      url=BAYER_SNAP_URL, notes="Worked example: frequent separation -> 0.6 x 2.5% = 1.5%."),
]

# ---------------- PA (unfilled)
out["pa"] = [
    F("fatigue_strength_MPa", 7.4, cycles=1e6, R=0.05, freq=10, measure="range (as stated 'fatigue limit stress')",
      load_mode="axial", specimen=RODR_SPEC + "; Stratasys Nylon 12", source=RODR, url=RODR_URL,
      notes=("Text: 'experimentally determined fatigue limit stress of Nylon 12 was 7.4 MPa'; infinite life below 17% x "
             "sigma_R (UTS 37 MPa, E 916 MPa, porosity 4.2%). Table 11 lists run-out at 15% = 6.6 MPa stress range. "
             "HCF Basquin intercept 296.8 MPa, slope -0.274 (stress range).")),
    F("fatigue_ratio", 0.17, cycles=1e6, R=0.05, measure="range", specimen=RODR_SPEC + "; Stratasys Nylon 12",
      source=RODR, url=RODR_URL, notes="Infinite life below 17% x sigma_R."),
    F("snap_fit_strain_allow", 0.04, T=23, specimen="injection moulded (Bayer Durethan PA, dry)", source=BAYER_PMD,
      url=BAYER_PMD_URL, notes="Conditioned (moist): 6.0%. Single snap; ~60% for repeated."),
    F("sn_curve_fit", None, R=-1, measure="amplitude", load_mode="rotating bending",
      specimen="printed FDM 'Nylon' (paper calls it PA6), Ultimaker S3/S5, horizontal, 0.15 mm layer, 210 C",
      source=YANKIN, url=YANKIN_URL, fit={"form": "sigma_a = A * N^b", "log10A_range": [2.48, 2.53], "b_range": [-0.223, -0.211]},
      notes="Table 5; mostly low-cycle data; text: Nylon withstood about 45 MPa at 9000 cycles."),
    F("sn_curve_fit", None, freq=5, load_mode="3-point flexure, imposed displacement",
      specimen="printed FDM PA6, XY and XZ build orientations", source=TEREKH, url=TEREKH_URL,
      fit={"form": "r = a - c*log10(N10)", "XY": {"a": 1.2278, "c": 0.086}, "XZ": {"a": 1.1323, "c": 0.078},
           "end_of_life": "N10 = 10% drop of max load"},
      notes=("Eqs (6)/(7); r = loading ratio (normalised imposed load level, highest tested 68%). Specimens never broke; "
             "damage = interlayer micro-delamination. Temperature rise < 10 C at 5 Hz. |1/B| 11.6 (XY), 12.8 (XZ) vs "
             "13.3 for injection-moulded PA6.6 (Table 7).")),
    F("fatigue_strength_MPa", 15, R=-1, measure="amplitude", load_mode="rotating bending",
      specimen="SLS PA12 (NOT FDM), hourglass", source=SAFAI + " - Table 2, citing Munguia and Dalgarno (2014)",
      url=SAFAI_URL, notes="Secondary: 'Specimens had an endurance limit of 15 MPa'. Cycle count not given in review table."),
]

# ---------------- PA-CF
out["pa_cf"] = [
    F("fatigue_strain_max", 0.045, cycles=2e5, R=0.1, freq=10, load_mode="3-point bending, strain controlled",
      specimen="printed FFF Markforged Onyx (PA6 + 6.5 wt% chopped CF), solid fill", source=AHMAD, url=AHMAD_URL,
      notes="'CF-PA6 specimens had a fatigue life of about 200,000 cycles in the applied strain equal to 4.5%'; strain ratio 0.1, mean strain 0.55 emax. Self-heating observed at 10 Hz."),
    F("fatigue_ratio", 0.6, cycles=1e6, R=0.1, freq=2, measure="max", load_mode="axial",
      specimen="printed FFF Fiberlogy PA12 CF15, Creality K1 Max, flat, 100% linear infill +/-45, 0.2 mm layer, dried 80 C 12 h",
      source=CIGANAS, url=CIGANAS_URL,
      notes=("Conclusions: 'at moderate loads (<=60-70% of maximum tensile strength) and temperatures above 80 C, no failure "
             "... within 1e6 cycles'; failure 'regardless of temperature' if load amplitude > 80% of max tensile force. "
             "Temperature-specific UTS: 49.07 MPa at 23 C, 26.60 at 80 C, 18.68 at 120 C. Low resolution; only 3 tests per condition.")),
]

# ---------------- PA-GF
out["pa_gf"] = [
    F("snap_fit_strain_allow", 0.015, T=23, specimen="injection moulded (Bayer Durethan PA 30% glass, dry)", source=BAYER_PMD,
      url=BAYER_PMD_URL, notes="Conditioned: 2.0%. For GF grades without yield point permissible strain is about half elongation at break."),
    F("fatigue_strain_max", 0.045, cycles=8000, R=0.1, freq=10, load_mode="3-point bending, strain controlled",
      specimen="printed Markforged Onyx + CONTINUOUS glass fibre (not short-GF)", source=AHMAD, url=AHMAD_URL,
      notes="'about 8000 cycles' at 4.5% applied strain vs ~200,000 for Onyx alone."),
]

# ---------------- POM
out["pom"] = [
    F("fatigue_strength_MPa", 31, cycles=1e6, T=23, load_mode="flexural fatigue ASTM D671",
      specimen="injection moulded Delrin 500 (POM-H)", source=DELRIN, url=DELRIN_URL,
      notes="Flexural Fatigue Endurance Limit, 50% RH, 23 C, 1e6 cycles: Delrin 100 32 MPa (4.7 kpsi), 500 31 MPa (4.5 kpsi), 900 32 MPa (4.6 kpsi). Fig. 21 S-N curve extends to 1e7 but no tabulated 1e7 value. Guide: Delrin 100 ~40% higher fatigue endurance than 500 in gear tests."),
    F("fatigue_strength_MPa", 32, cycles=1e6, T=23, load_mode="flexural fatigue ASTM D671",
      specimen="injection moulded Delrin 100 (POM-H)", source=DELRIN, url=DELRIN_URL, notes="Same table as above."),
    F("fatigue_strength_MPa", 44, cycles=3e4, R=0.1, freq=10, T=23, measure="max", load_mode="axial",
      specimen="injection moulded Delrin (POM-H), test chamber door open (still air)", source=VOLGERS, url=VOLGERS_URL,
      notes=("Self-heating illustration: to reach ~30,000 cycles the still-air sample is loaded to 44 MPa peak, the "
             "circulating-air sample to 55 MPa. Around 50 MPa peak, failure at ~2500 cycles. Shows cooling dominates "
             "polymer fatigue at higher frequency - relevant to ~300 Hz flexures.")),
]

# ---------------- PEEK
out["peek"] = [
    F("fatigue_strength_MPa", 65, cycles=1e6, R=0.2, measure="max", load_mode="axial (ASTM D7791)",
      specimen="printed FDM Apium PEEK 450 Natural", source=RENDAS, url=RENDAS_URL,
      notes="Run-out at sigma_max = 75% of printed UTS 86.7 MPa. Basquin A=64.026, b=-0.061 (r=0.543). Goodman-corrected R=-1 equivalent 47.3 MPa (Table 5)."),
    F("fatigue_ratio", 0.75, cycles=1e6, R=0.2, measure="max", specimen="printed FDM Apium PEEK 450",
      source=RENDAS, url=RENDAS_URL, notes="sigma_max/UTS at run-out."),
]

# ---------------- TPU
out["tpu"] = [
    F("fatigue_strength_MPa", 10.25, cycles=1e7, R=0.1, freq=5, T=None,
      measure="as stated ('fatigue strength'; figure captions call these 'stress')", load_mode="axial tension-tension",
      specimen="TPU plate (medical 80A), cut specimen with notched ends - NOT printed", source=WANG, url=WANG_URL,
      notes=("'The fatigue strength decreased from 23.04 MPa at 1e3 cycles to 10.25 MPa at 1e7 cycles.' 13.08 MPa -> 1e6, "
             "15.72 -> 3e5, 17.08 -> 1e5. Basquin as printed: sigma_a = 1.697 (Nf)^-0.0935 (coefficient appears to be log10). "
             "Room temperature. Large cyclic creep (ratcheting) noted.")),
]

# ---------------- empty classes (documented gaps)
for c in ("petg", "pp", "pp_gf", "pet_cf", "petg_cf", "cf_rod", "steel_wire"):
    out[c] = []

with open(OUT, "w") as f:
    json.dump(out, f, indent=1, ensure_ascii=False)
print("classes:", {k: len(v) for k, v in out.items() if k != "_meta"})
