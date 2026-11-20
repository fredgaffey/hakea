import json

OUT = "/tmp/claude-1000/-home-fred-vehicles-hakea/d3cb2918-65fe-44b5-aae3-7340f9b6d1ce/scratchpad/research/flexure_products.json"

TDS_KEYS = ["density_g_cm3",
            "tensile_modulus_xy_MPa", "tensile_strength_xy_MPa", "elongation_xy_pct",
            "tensile_modulus_z_MPa", "tensile_strength_z_MPa", "elongation_z_pct",
            "flexural_modulus_xy_MPa", "flexural_strength_xy_MPa",
            "flexural_modulus_z_MPa", "flexural_strength_z_MPa",
            "shore_hardness", "hdt_C", "hdt_load_MPa",
            "impact_xy_kJ_m2", "impact_z_kJ_m2", "impact_method",
            "tear_strength_kN_m", "test_standard", "specimen", "specimen_notes",
            "conditioning", "conditioned"]
PRINT_KEYS = ["nozzle_temp_C", "bed_temp_C", "hardened_nozzle_required", "ams_compatible",
              "drying", "moisture_sensitivity", "annealing"]


def rec(name, brand, product, cls, url, tds_url, price, tds, prt, conf, notes):
    t = {k: None for k in TDS_KEYS}
    t.update(tds)
    p = {k: None for k in PRINT_KEYS}
    p.update(prt)
    return {"name": name, "brand": brand, "product": product, "class": cls, "url": url,
            "tds_url": tds_url, "price": price, "tds": t, "print": p,
            "confidence": conf, "notes": notes}


R = []

# ---------------- POM ----------------
R.append(rec(
    "grupaazoty_tarfuse_pom", "Grupa Azoty (Tarfuse)", "Tarfuse POM", "pom",
    "https://filament2print.com/de/erweitert/2241-tarfuse-pom-nat-filament.html",
    "https://filament2print.com/de/index.php?controller=attachment&id_attachment=2060",
    {"value": 44.95, "currency": "EUR", "per_kg": True, "spool_kg": 1.0, "date": "2026-11",
     "store": "filament2print.com", "note": "incl. VAT, natural 1.75 mm 1 kg; distributor (manufacturer has no web store found)"},
    {"density_g_cm3": 1.42,
     "tensile_modulus_xy_MPa": 1870, "tensile_strength_xy_MPa": 50, "elongation_xy_pct": 11,
     "test_standard": "ISO 527-1,-2 (50 mm/min strength, 1 mm/min modulus); ISO 1183 density",
     "specimen": "printed",
     "specimen_notes": "XY = flat. XZ (on edge) and ZX (upright) listed 'To be tested'. Print params for specimens: nozzle 230 C, chamber 70 C (table on p2).",
     "conditioning": "unknown"},
    {"nozzle_temp_C": "210-240", "bed_temp_C": "100-130", "hardened_nozzle_required": None,
     "drying": "140 C 4-6 h (hot air dryer)", "moisture_sensitivity": None},
    "high",
    "PRELIMINARY TDS v1.1 (04.2021), Grupa Azoty S.A. Polyacetal copolymer (POM-C). Build chamber 70-140 C recommended; nozzle >=0.4 mm; print speed 10-30 mm/s; 'parts up to 100 mm'. Humidity absorption 0.2% (23C/50%RH), water absorption 0.8% (ISO 62). Tg -50 C, Tm 165-170 C. Flexural, impact, HDT all blank ('-') in TDS."))

# ---------------- PA12 ----------------
R.append(rec(
    "fiberlogy_nylon_pa12", "Fiberlogy", "Nylon PA12", "pa",
    "https://fiberlogy.com/en/Nylon-PA12-Filament-1_75mm-2_5kg",
    "https://fiberlogy.com/app/uploads/2026/05/FIBERLOGY_NYLONPA12_TDS.pdf",
    {"value": 54.70, "currency": "EUR", "per_kg": True, "spool_kg": 1.0, "date": "2026-11",
     "store": "fiberlogy.com", "note": "most colours 54.70 EUR per 1 kg variant (range 50.70-60.50); 2.5 kg variant 125.20-135.00 EUR. TDS still states 0.75 kg net weight (older spool)."},
    {"density_g_cm3": 1.01,
     "tensile_modulus_xy_MPa": 1400, "tensile_strength_xy_MPa": 45, "elongation_xy_pct": None,
     "shore_hardness": None, "hdt_C": 135, "hdt_load_MPa": 0.45,
     "impact_xy_kJ_m2": 12, "impact_method": "Charpy notched ISO 179, 23 C",
     "test_standard": "ISO 527, ISO 179, ISO 75, ISO 306, ISO 11357-3, ISO 1183",
     "specimen": "unknown",
     "specimen_notes": "TDS does not state specimen type or orientation; disclaimer says data 'gathered from standard reference materials and/or supplier test data'. Tensile strength is @ yield.",
     "conditioning": "unknown"},
    {"nozzle_temp_C": "255-270", "bed_temp_C": "100", "ams_compatible": True,
     "drying": None, "moisture_sensitivity": None},
    "medium",
    "TDS last update May 20, 2025 (PDF is image-only; values read via OCR - re-check if critical). Elongation @ yield 5%, elongation @ break '>50%' (lower bound, not recorded as number). HDT @ 1.8 MPa = 55 C. Vicat 145 C, Tm 178 C. Product page: 'Compatible with BambuLab AMS' (new spool system). Distributor filament2print quotes drying 70 C 4 h."))

R.append(rec(
    "formfutura_styx_12", "Formfutura", "STYX-12", "pa",
    "https://www.3djake.com/formfutura/styx-12-nylon-white",
    "https://cdn.shopify.com/s/files/1/0069/3049/7593/files/TDS_-_STYX-12.pdf",
    None,
    {"density_g_cm3": 1.02,
     "tensile_modulus_xy_MPa": 1400, "tensile_strength_xy_MPa": 60.0, "elongation_xy_pct": None,
     "shore_hardness": "81D",
     "impact_xy_kJ_m2": 14, "impact_method": "Charpy notched ISO 179 @23 C",
     "test_standard": "ISO 527-1/-2, ISO 179, ISO 868, ISO 1183",
     "specimen": "unknown",
     "specimen_notes": "TDS gives single 'typical value' column, no orientation or specimen type. Tensile strength @ yield.",
     "conditioning": "unknown"},
    {"nozzle_temp_C": "240-270", "bed_temp_C": None, "drying": None},
    "medium",
    "TDS v1, 15 May 2017 (old). Elongation at break '>150%' (lower bound, not recorded). Tm 250 +/-10 C as written (implausible for PA12; likely TDS error). Vicat ~105 C. Product appears DISCONTINUED: 3DJake says it no longer sells it; formfutura.com product URL 404. Price null. TDS PDF hosted on a third-party Shopify CDN."))

R.append(rec(
    "gehr_fil_a_gehr_pa12", "GEHR", "FIL-A-GEHR PA12", "pa",
    "https://directindustry.com/prod/gehr-kunststoffwerk/product-17481-2367092.html",
    "https://cdn.shopify.com/s/files/1/0672/7306/4714/files/FIL-A-GEHR_PA12_Data.pdf",
    None,
    {"density_g_cm3": 1.01,
     "tensile_modulus_xy_MPa": 1440, "tensile_strength_xy_MPa": 43,
     "impact_xy_kJ_m2": 11, "impact_method": "Charpy notched ISO 179 +23 C",
     "test_standard": "ISO 527, ISO 179, ISO 1183",
     "specimen": "unknown",
     "specimen_notes": "TDS states 'Data based on raw material' (i.e. not printed; likely moulded). Tensile strength is stress at yield.",
     "conditioning": "unknown"},
    {"nozzle_temp_C": "245-260", "bed_temp_C": "110", "drying": "80 C 4-6 h"},
    "medium",
    "One-page TDS dated 2017-08-11 (PDF metadata). Density given as 1010 kg/m3. MVR 8 cm3/10 min (235 C/5 kg). Melt point 180 C. Bed: Pertinax plates. url is a catalogue listing (no web store price found)."))

R.append(rec(
    "esun_epa12", "eSUN", "ePA12", "pa",
    "https://www.bastelgarage.ch/epa12-nylon-white-filament-1-75mm-1kg-esun",
    "https://download.bastelgarage.ch/Produkte/eSUN_ePA12-Filament_TDS_V4.0.pdf",
    {"value": 65.90, "currency": "CHF", "per_kg": True, "spool_kg": 1.0, "date": "2026-11",
     "store": "bastelgarage.ch", "note": "incl. VAT; listed out of stock. esun3d.com blocks automated fetch."},
    {"density_g_cm3": 1.012,
     "tensile_strength_xy_MPa": 50.81, "elongation_xy_pct": 165.41,
     "flexural_modulus_xy_MPa": 659.49, "flexural_strength_xy_MPa": 47.27,
     "hdt_C": 100, "hdt_load_MPa": 0.45,
     "impact_xy_kJ_m2": 10.14, "impact_method": "IZOD GB/T 1843",
     "test_standard": "GB/T 1040, GB/T 9341, GB/T 1843, GB/T 1634, GB/T 1033",
     "specimen": "injection moulded",
     "specimen_notes": "TDS: properties 'obtained based on the injection molding spline test'.",
     "conditioning": "unknown"},
    {"nozzle_temp_C": "260-290", "bed_temp_C": "70-90", "drying": "70 C >12 h",
     "moisture_sensitivity": None},
    "high",
    "TDS Version 4.0, Nov 2021 (distributor-hosted copy of eSUN TDS). No tensile modulus given. Flexural modulus 659 MPa is unusually low for PA12 - recorded as stated. MFI 55.64 g/10min (270 C/2.16 kg). Search snippet of esun3d.com product page shows a newer 'Tensile Strength (XY) 47.74 MPa' (not fetched; site blocks bots)."))

# ---------------- PP ----------------
R.append(rec(
    "ultimaker_pp", "UltiMaker", "PP", "pp",
    "https://ultimaker.com/materials/s-series-materials/",
    "https://www.alloys.com.au/Documents/Ultimaker-PP-TDS-v3.00.pdf",
    None,
    {"density_g_cm3": 0.89,
     "tensile_modulus_xy_MPa": 253, "tensile_strength_xy_MPa": 10.2, "elongation_xy_pct": None,
     "tensile_modulus_z_MPa": 234, "tensile_strength_z_MPa": 8.6, "elongation_z_pct": 22.8,
     "flexural_modulus_xy_MPa": 293, "flexural_strength_xy_MPa": 15.3,
     "flexural_modulus_z_MPa": 212, "flexural_strength_z_MPa": 9.4,
     "shore_hardness": "42D", "hdt_C": 64.1, "hdt_load_MPa": 0.455,
     "impact_xy_kJ_m2": 49.1, "impact_method": "Charpy notched ISO 179-1/1eB (printed notch), 23 C",
     "test_standard": "ASTM D3039 (tensile), ISO 178 (flex), ISO 179-1, ISO 75-2/B, ISO 306, ISO 1183",
     "specimen": "printed",
     "specimen_notes": "100% infill, 0.15 mm layer, AA0.4, UltiMaker S5. XY = flat (mostly infill), YZ = side (walls), Z = up. Tensile strength = stress at yield. YZ: E 273 MPa, yield 12.1 MPa, flex mod 250 MPa, flex str 12.9 MPa. Conditioned >=24 h at room temperature.",
     "conditioning": "unknown"},
    {"nozzle_temp_C": None, "bed_temp_C": None, "drying": None},
    "high",
    "TDS v3.00 (distributor-hosted copy at alloys.com.au). 2.85 mm only, 500 g spool. XY/YZ elongation at break >800% (no break); Z elongation at break 22.8%, Z yield elong 18.7%; XY yield elongation 34.8%. Flex strength given at ~8.4% strain (no break). Tm 130.6 C (as stated). TDS explicitly lists 'exceptional fatigue resistance' and living hinges as applications. No price found."))

R.append(rec(
    "formfutura_centaur_pp", "Formfutura", "Centaur PP", "pp",
    "https://dhm-online.com/centaur-pp-formfutura/4261-centaur-pp-black-formfutura.html",
    "https://cdn.shopify.com/s/files/1/1339/4265/files/TDS_-_Centaur_PP.pdf",
    None,
    {"density_g_cm3": 0.9,
     "tensile_strength_xy_MPa": 12, "elongation_xy_pct": None,
     "flexural_modulus_xy_MPa": 402,
     "shore_hardness": "50D",
     "test_standard": "ASTM D638 (2.0 in/min), ASTM D790 (1.27 mm/min), ASTM D2240, ASTM D1505",
     "specimen": "unknown", "specimen_notes": "Single typical-value column; no orientation/specimen stated. Tensile strength @ yield.",
     "conditioning": "unknown"},
    {"nozzle_temp_C": "220-240", "bed_temp_C": None, "drying": None},
    "medium",
    "Formfutura TDS (undated). Tensile modulus '-' (not given). Elongation at break '>600%%' (lower bound, not recorded). MFR 8 g/10min (260 C/5 kg). Vicat ~103 C. Distributor product page blocked fetch (403); price null. Values identical to Spectrum PP TDS -> likely same base resin."))

R.append(rec(
    "spectrum_pp", "Spectrum Filaments", "PP", "pp",
    "https://spectrumfilaments.com/en/filament/pp/",
    "https://spectrumfilaments.com/wp-content/uploads/2025/11/en_tds_spectrum_pp.pdf",
    None,
    {"density_g_cm3": 0.9,
     "tensile_strength_xy_MPa": 16, "elongation_xy_pct": 637,
     "flexural_modulus_xy_MPa": 400,
     "shore_hardness": "50D", "tear_strength_kN_m": 127,
     "specimen": "unknown", "specimen_notes": "No test standards, orientation or specimen type stated.",
     "conditioning": "unknown"},
    {"nozzle_temp_C": "245-280", "bed_temp_C": "80-100", "hardened_nozzle_required": False,
     "drying": None, "moisture_sensitivity": "low"},
    "medium",
    "TDS prepared 27-10-2025. Lists 'Tensile Stress 12 MPa', 'Tensile Strength at Yield 16 MPa', 'Tensile Strength at Break 26 MPa', 'Tensile Stress at 100% 11 MPa', elongation at yield 20%, at break 637%. Tear strength (Die C) 127 kN/m. Vicat 103 C. No tensile modulus. 'Dry box recommended: No'; 'very low moisture absorption'. Price not shown on product page."))

R.append(rec(
    "xtellar_braskem_pp_fl105pp", "Xtellar (Braskem)", "Polypropylene FL105PP", "pp",
    "https://www.3dfuel.com/products/braskem-polypropylene-filament-1-75mm-by-xtellar",
    "https://cdn.shopify.com/s/files/1/0380/5149/files/eTDS_xtellar_Polypropylene_PP__FL105PP_1.1.pdf",
    None,
    {"density_g_cm3": 0.91,
     "tensile_modulus_xy_MPa": 1241, "tensile_strength_xy_MPa": 16.9, "elongation_xy_pct": 519.2,
     "flexural_modulus_xy_MPa": 1012, "flexural_strength_xy_MPa": 10.1,
     "shore_hardness": "63D",
     "test_standard": "ASTM D638, ASTM D790, ASTM D792, ASTM D2240, ASTM D648",
     "specimen": "printed",
     "specimen_notes": "Printed XY and XZ (ASTM orientation codes; XZ not mapped to *_z per brief). XZ: E 526 MPa, yield 6.8 MPa, elong 1.8%, flex mod 430 MPa. Flexural values are @1% strain (no break). Tensile strength = @ yield (break 10.2 MPa XY).",
     "conditioning": "unknown"},
    {"nozzle_temp_C": "230", "bed_temp_C": "85", "hardened_nozzle_required": False},
    "medium",
    "eTDS v1.1 rev. April 29 2023 (hosted on a reseller Shopify CDN). Bulk (injection moulded) density 0.92. HDT @1.82 MPa: XY 47.7 C, XZ 42.0 C (0.45 MPa blank). Izod impacts given in J/m (not converted): notched 23 C XY 201.5 J/m, XZ 92.0 J/m. Text says 'for use in pellet-based 3D printers' though SKU FL = filament. XY modulus 1241 MPa is ~5x other PP filaments - outlier, use with care. Price: search snippet showed 35.00 USD (was 45.00) at 3dfuel.com, spool size not confirmed -> price null."))

# ---------------- PP-GF ----------------
R.append(rec(
    "xtellar_braskem_pp_gf_fl500", "Xtellar (Braskem)", "Glass Fiber PP FL500PP-GF", "pp_gf",
    "https://shop3duniverse.com/products/xtellar-pro-glass-fiber-pp-2-85mm-black",
    "https://cdn.shopify.com/s/files/1/0380/5149/files/eTDS_-_GF_PP.pdf",
    None,
    {"density_g_cm3": 1.06,
     "tensile_modulus_xy_MPa": 5035, "tensile_strength_xy_MPa": 43.6, "elongation_xy_pct": 2.5,
     "flexural_modulus_xy_MPa": 3518, "flexural_strength_xy_MPa": 35.2,
     "shore_hardness": "58D", "hdt_C": 144.7, "hdt_load_MPa": 0.45,
     "test_standard": "ASTM D638, ASTM D790, ASTM D648, ASTM D792, ASTM D2240",
     "specimen": "printed",
     "specimen_notes": "Printed XY and XZ (XZ not mapped). XZ: E 1100 MPa, yield 8.9 MPa, elong 1.7%, flex mod 560 MPa, HDT(0.45) 79.3 C. Flexural @1% strain (no break). Tensile strength @ yield (break 43.1).",
     "conditioning": "unknown"},
    {"nozzle_temp_C": "240-260", "bed_temp_C": "80", "hardened_nozzle_required": True,
     "drying": "no drying needed (per product data sheet)", "moisture_sensitivity": "low"},
    "medium",
    "eTDS v1.1 rev. April 29 2023 (reseller-hosted). Glass fibre content not stated. Separate one-page Product Data Sheet FL500PP-GF (https://cdn.shopify.com/s/files/1/0843/3791/8249/files/Product_Data_Sheet_-_FL500PP-GF.pdf) gives printed XY: density 1.06, UTS 46 MPa, elong 2.2%, Young's 5.2 GPa, flex chord modulus 4.1 GPa, Izod 189 J/m, HDT(0.455) 154 C, Vicat 157 C, Shore 63D; nozzle >=0.6 mm hardened; 'estimated values'. Izod in J/m not converted. Price not fetched (rate-limited)."))

# ---------------- CF / GF strip ----------------
R.append(rec(
    "easycomposites_cf_strip_0p5x3", "Easy Composites", "Pultruded Carbon Fibre Strip 3 mm x 0.5 mm", "cf_rod",
    "https://www.easycomposites.co.uk/3mm-x-05mm-carbon-fibre-strip",
    "https://media.easycomposites.eu/datasheets/EC-TDS-Carbon-Fibre-Pultrusions.pdf",
    {"value": 2.15, "currency": "GBP", "per_kg": False, "spool_kg": None, "date": "2026-11",
     "store": "easycomposites.co.uk", "note": "per metre, ex VAT; sold in 1 m or 2 m lengths"},
    {"density_g_cm3": 1.30,
     "tensile_modulus_xy_MPa": 28000, "tensile_strength_xy_MPa": 400,
     "tensile_modulus_z_MPa": 8000, "tensile_strength_z_MPa": 18,
     "flexural_modulus_xy_MPa": 20000, "flexural_strength_xy_MPa": 250,
     "fibre_modulus_GPa": None, "rod_modulus_GPa": 28, "rod_modulus_range_GPa": [28, 40],
     "rod_modulus_type": "tensile (lengthways)",
     "tensile_strength_MPa": 400, "tensile_strength_range_MPa": [400, 500],
     "flexural_modulus_range_GPa": [20, 30], "flexural_strength_range_MPa": [250, 400],
     "density_range_g_cm3": [1.30, 1.50],
     "diameters_mm": ["0.5x3", "0.8x3", "1x3", "1x4", "1x6", "0.5x10", "2x12"],
     "test_standard": None, "specimen": "unknown",
     "specimen_notes": "TDS gives RANGES; numeric fields hold the LOWER bound of each stated range (conservative). *_z fields = 'widthways' (transverse): tensile modulus 8-12 GPa, tensile strength 18-30 MPa.",
     "conditioning": "unknown"},
    {},
    "high",
    "EC TDS 'Carbon Fibre Pultrusions' (CFP) v1.0 rev 20/09/2024: 100% UD carbon, max service 80 C, 'data representative of a typical pultrusion'. Product page: vinylester matrix, standard-modulus fibre, 2.42 g/m (implies ~1.61 g/cm3 for 1.5 mm2 nominal section - above TDS density range; not recorded). Compressive modulus lengthways 10-20 GPa; short beam shear 30-40 MPa. Note low stated modulus (28-40 GPa) vs typical UD carbon pultrusion."))

R.append(rec(
    "easycomposites_cf_strip_1x3", "Easy Composites", "Pultruded Carbon Fibre Strip 3 mm x 1 mm", "cf_rod",
    "https://www.easycomposites.co.uk/3mm-x-1mm-carbon-fibre-strip",
    "https://media.easycomposites.eu/datasheets/EC-TDS-Carbon-Fibre-Pultrusions.pdf",
    {"value": 2.60, "currency": "GBP", "per_kg": False, "spool_kg": None, "date": "2026-11",
     "store": "easycomposites.co.uk", "note": "per metre, ex VAT"},
    {"density_g_cm3": 1.30,
     "tensile_modulus_xy_MPa": 28000, "tensile_strength_xy_MPa": 400,
     "fibre_modulus_GPa": None, "rod_modulus_GPa": 28, "rod_modulus_range_GPa": [28, 40],
     "rod_modulus_type": "tensile (lengthways)",
     "tensile_strength_MPa": 400, "tensile_strength_range_MPa": [400, 500],
     "density_range_g_cm3": [1.30, 1.50],
     "diameters_mm": ["0.5x3", "0.8x3", "1x3", "1x4", "1x6", "0.5x10", "2x12"],
     "specimen": "unknown",
     "specimen_notes": "Same generic TDS as 0.5x3 record; lower bounds of ranges recorded.",
     "conditioning": "unknown"},
    {},
    "medium",
    "Product page states EPOXY matrix for this size (0.5x3 page states vinylester), standard-modulus fibre, 4.5 g/m. Product page tensile modulus 28-40 GPa, strength 400-500 MPa (same as TDS)."))

R.append(rec(
    "dragonplate_cf_strip_032x310", "DragonPlate (Allred & Associates)", "Carbon Strip (Rectangle) .032\" x .310\" x 48\"", "cf_rod",
    "https://dragonplate.com/dragonplate-carbon-strip-rectangle-032-x-310-x-48",
    None,
    {"value": 8.24, "currency": "USD", "per_kg": False, "spool_kg": None, "date": "2026-11",
     "store": "dragonplate.com", "note": "8.24 USD per 48 in (1.219 m) piece = 6.76 USD/m; qty breaks 10+ 7.75, 100+ 6.76 per piece"},
    {"density_g_cm3": 1.55,
     "fibre_modulus_GPa": 227.5, "rod_modulus_GPa": None,
     "rod_modulus_type": "not stated (only fibre modulus 'Standard (33 Msi)')",
     "tensile_strength_MPa": None,
     "diameters_mm": ["0.81x7.87 (.032x.310 in)", "0.89x2.69 (.035x.106 in)", "1.78x11.10 (.070x.437 in)", "3.18x8.26 (.125x.325 in)"],
     "specimen": "unknown",
     "specimen_notes": "Solid CF, all unidirectional (UD 0), tooled finish, max temp 200 F.",
     "conditioning": "unknown"},
    {},
    "medium",
    "Product page lists fibre modulus 'Standard (33 Msi)' -> 227.5 GPa (unit conversion only; this is FIBRE modulus, not strip modulus). Density 0.056 lb/in3 -> 1.55 g/cm3. No strip tensile/flexural modulus or strength published (Weights & Specs page gives only fibre-level table). Other thin strips: .035x.106x48 from 4.02 USD; .070x.437x48 from 12.32 USD; .125x.325x48 from 14.35 USD."))

R.append(rec(
    "carbonfibreprofiles_cf_pultrusion", "Carbon Fibre Profiles (UK)", "Pultruded carbon rods/tubes up to 15 mm (vinylester, STS carbon)", "cf_rod",
    "https://carbonfibreprofiles.com/pages/technical-information",
    "https://cdn.shopify.com/s/files/1/0553/9584/4132/files/Typical_Properties_Carbon_Rods_Tubes_CFP.pdf?v=1659688999",
    None,
    {"density_g_cm3": 1.6,
     "tensile_modulus_xy_MPa": 134000, "tensile_strength_xy_MPa": 1750,
     "fibre_modulus_GPa": None, "rod_modulus_GPa": 134, "rod_modulus_type": "tensile",
     "tensile_strength_MPa": 1750, "diameters_mm": None,
     "specimen": "unknown", "specimen_notes": "Datasheet title: 'Typical Mechanical Properties of Carbon Fibre, Rods & Tubes (Up to 15.0 mm)'. Compressive strength 1450 MPa; HDT (Tg) 135 C.",
     "conditioning": "unknown"},
    {},
    "low",
    "Datasheet covers rods & tubes; flat-strip product availability/price NOT confirmed (shop pages rate-limited during research). Included because this is a sourced pultruded UD-carbon/vinylester property set ~4x stiffer than the Easy Composites figure."))

R.append(rec(
    "carbonfibreprofiles_grp_flat_bar", "Carbon Fibre Profiles (UK)", "Pultruded GRP flat bar / rods / tubes (UD E-glass, up to 31.8 mm)", "gf_rod",
    "https://carbonfibreprofiles.com/pages/technical-information",
    "https://cdn.shopify.com/s/files/1/0553/9584/4132/files/Typical_Properties_of_rods_tubes_GRP_CFP.pdf?v=1659688999",
    None,
    {"density_g_cm3": 1.8,
     "tensile_modulus_xy_MPa": 27000, "tensile_strength_xy_MPa": 500,
     "flexural_modulus_xy_MPa": 30000, "flexural_strength_xy_MPa": 650,
     "fibre_modulus_GPa": None, "rod_modulus_GPa": 27, "rod_modulus_type": "tensile (flexural 30 GPa also stated)",
     "tensile_strength_MPa": 500, "diameters_mm": None,
     "specimen": "unknown",
     "specimen_notes": "Datasheet: 'Typical Mechanical Properties of GRP Flat Bar, Rods & Tubes (Up to 31.8 mm)', unidirectional E-glass rovings; polyester or vinyl ester resin.",
     "conditioning": "unknown"},
    {},
    "medium",
    "Units in datasheet MN/m2 and GN/m2 (= MPa, GPa). Compressive strength 190 MPa; water absorption 0.5-2%; CTE 7.2e-6/K; Barcol 40-70; HDT 125-145 C. Flat-bar sizes and price not obtained (shop pages rate-limited)."))

json.dump(R, open(OUT, "w"), indent=1, ensure_ascii=False)
print(len(R), "records")
