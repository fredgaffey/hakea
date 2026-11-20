import json

OUT = "/tmp/claude-1000/-home-fred-vehicles-hakea/d3cb2918-65fe-44b5-aae3-7340f9b6d1ce/scratchpad/research/engineering.json"

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


def tds(**kw):
    d = {k: None for k in TDS_KEYS}
    d.update(kw)
    return d


def prt(**kw):
    d = {k: None for k in PRINT_KEYS}
    d.update(kw)
    return d


def price(value, cur, spool, store, note="", per_kg=True):
    v = round(value / spool, 2) if per_kg else value
    return {"value": v, "currency": cur, "per_kg": True, "spool_kg": spool,
            "date": "2026-11", "store": store, "note": note}


R = []

# ---------------- Bambu Lab ----------------
BAMBU_SPEC = "Printed, 100% infill, nozzle 260/270 C; all specimens annealed and dried at 80 C for 12 h before testing (per TDS)."
R.append(dict(
    name="bambu_abs", brand="Bambu Lab", product="ABS", **{"class": "abs"},
    url="https://au.store.bambulab.com/products/abs-filament",
    tds_url="https://www.additive-x.com/shop/mpattachments/file/viewonline/id/566/product_id/2315",
    price=price(23.99, "AUD", 1.0, "au.store.bambulab.com", "Refill 1 kg; 28.99 AUD with spool"),
    tds=tds(density_g_cm3=1.05,
            tensile_modulus_xy_MPa=3800, tensile_strength_xy_MPa=41, elongation_xy_pct=10.5,
            tensile_modulus_z_MPa=2960, tensile_strength_z_MPa=32, elongation_z_pct=4.7,
            flexural_modulus_xy_MPa=1880, flexural_strength_xy_MPa=68,
            flexural_modulus_z_MPa=1590, flexural_strength_z_MPa=45,
            hdt_C=84, hdt_load_MPa=1.8,
            impact_xy_kJ_m2=39.3, impact_z_kJ_m2=7.4,
            impact_method="ISO 179 Charpy; XY value is unnotched (XY notched = 21.5 kJ/m2); Z notch state not stated",
            test_standard="ISO 527, ISO 178, ISO 179 (GB/T equivalents)", specimen="printed",
            specimen_notes=BAMBU_SPEC + " Nozzle 260 C, bed 80 C, 200 mm/s.",
            conditioning="annealed"),
    print=prt(nozzle_temp_C="240-270", bed_temp_C="80-100", drying="80 C, 8 h",
              annealing=None),
    confidence="medium",
    notes="Bambu TDS V2.0 via Additive-X distributor copy (bambulab.com blocked). HDT 87 C at 0.45 MPa. "
          "Stated tensile (Young's) modulus XY 3800 MPa is ~2x the stated bending modulus 1880 MPa; recorded as written. "
          "Chamber 45-60 C recommended. Water absorption 0.65% (25 C, 55% RH)."))

R.append(dict(
    name="bambu_asa", brand="Bambu Lab", product="ASA", **{"class": "asa"},
    url="https://au.store.bambulab.com/products/asa-filament",
    tds_url="https://www.additive-x.com/shop/mpattachments/file/viewonline/id/612/product_id/2399",
    price=price(27.59, "AUD", 1.0, "au.store.bambulab.com", "With spool, on sale (list 45.99 AUD)"),
    tds=tds(density_g_cm3=1.05,
            tensile_modulus_xy_MPa=2450, tensile_strength_xy_MPa=52, elongation_xy_pct=9.2,
            tensile_modulus_z_MPa=2120, tensile_strength_z_MPa=35, elongation_z_pct=4.6,
            flexural_modulus_xy_MPa=1920, flexural_strength_xy_MPa=74,
            flexural_modulus_z_MPa=1650, flexural_strength_z_MPa=42,
            hdt_C=92, hdt_load_MPa=1.8,
            impact_xy_kJ_m2=41.0, impact_z_kJ_m2=4.9,
            impact_method="ISO 179 Charpy; XY value is unnotched (XY notched = 19.6 kJ/m2); Z notch state not stated",
            test_standard="ISO 527, ISO 178, ISO 179 (GB/T equivalents)", specimen="printed",
            specimen_notes=BAMBU_SPEC, conditioning="annealed"),
    print=prt(nozzle_temp_C="240-270", bed_temp_C="80-100", drying="80 C, 8 h"),
    confidence="medium",
    notes="Bambu TDS V2.0 via Additive-X distributor copy. HDT 100 C at 0.45 MPa. Chamber 45-60 C. "
          "Water absorption 0.45%. A V3.0 copy (polyalkemi.no) reportedly gives the same Z modulus 2120 MPa."))

R.append(dict(
    name="bambu_pc", brand="Bambu Lab", product="PC", **{"class": "pc"},
    url="https://au.store.bambulab.com/products/pc-filament",
    tds_url="https://www.additive-x.com/shop/mpattachments/file/viewonline/id/591/product_id/2340",
    price=price(37.19, "AUD", 1.0, "au.store.bambulab.com", "With spool, on sale (list 61.99 AUD)"),
    tds=tds(density_g_cm3=1.20,
            tensile_modulus_xy_MPa=2110, tensile_strength_xy_MPa=62, elongation_xy_pct=3.8,
            tensile_modulus_z_MPa=1450, tensile_strength_z_MPa=56, elongation_z_pct=2.1,
            flexural_modulus_xy_MPa=2310, flexural_strength_xy_MPa=108,
            flexural_modulus_z_MPa=1620, flexural_strength_z_MPa=55,
            hdt_C=117, hdt_load_MPa=1.8,
            impact_xy_kJ_m2=34.8, impact_z_kJ_m2=9.0,
            impact_method="ISO 179 Charpy; XY value is unnotched (second XY value 7.5 kJ/m2, presumably notched); Z notch state not stated",
            test_standard="ISO 527, ISO 178, ISO 179 (GB/T equivalents)", specimen="printed",
            specimen_notes=BAMBU_SPEC + " Nozzle 270 C, bed 100 C.", conditioning="annealed"),
    print=prt(nozzle_temp_C="260-280", bed_temp_C="90-110", drying="80 C, 8 h"),
    confidence="medium",
    notes="Bambu PC TDS V2.0 via Additive-X distributor copy. TDS lists HDT 117 C at 1.8 MPa and 112 C at 0.45 MPa "
          "(inverted order, recorded as written)."))

# ---------------- Polymaker ----------------
PM_SPEC = "Printed: nozzle 260 C, bed {bed} C, environment {env} C, 2 shells, 3 top/bottom, 100% infill, fan off; X-Y flat, Z upright."
R.append(dict(
    name="polymaker_polylite_abs", brand="Polymaker", product="PolyLite ABS", **{"class": "abs"},
    url="https://wiki.polymaker.com/polymaker-products/more-about-our-products/documents/technical-data-sheets/abs-asa/polylite-tm-abs",
    tds_url="https://cdn.shopify.com/s/files/1/0548/7299/7945/files/PolyLite_ABS_TDS_EN_V5.4.pdf?v=1731955200",
    price=None,
    tds=tds(density_g_cm3=1.04,
            tensile_modulus_xy_MPa=2246.6, tensile_strength_xy_MPa=33.4, elongation_xy_pct=17.9,
            tensile_modulus_z_MPa=2080.9, tensile_strength_z_MPa=29.7, elongation_z_pct=3.1,
            flexural_modulus_xy_MPa=2127.2, flexural_strength_xy_MPa=56.2,
            hdt_C=98, hdt_load_MPa=1.8, impact_xy_kJ_m2=18.0,
            impact_method="Notched Charpy ISO 179",
            test_standard="ISO 527, ISO 178, ISO 179 (GB/T equivalents)", specimen="printed",
            specimen_notes=PM_SPEC.format(bed=90, env=90), conditioning="unknown"),
    print=prt(nozzle_temp_C="245-265", bed_temp_C="90-100", drying="70 C for 6 h", annealing="none stated"),
    confidence="high",
    notes="Document content is TDS V5.5 (file named V5.4 on Polymaker's Shopify CDN, linked from Polymaker wiki). "
          "HDT 100 C at 0.45 MPa. Bending Z N/A. url is Polymaker wiki product doc page (shop.polymaker.com rate-limited)."))

R.append(dict(
    name="polymaker_polylite_asa", brand="Polymaker", product="PolyLite ASA (now 'Polymaker ASA')", **{"class": "asa"},
    url="https://wiki.polymaker.com/polymaker-products/more-about-our-products/documents/technical-data-sheets/abs-asa/polylite-tm-asa",
    tds_url="https://cdn.shopify.com/s/files/1/0548/7299/7945/files/PolyLite_ASA_TDS_EN_V5.4.pdf?v=1731955208",
    price=None,
    tds=tds(density_g_cm3=1.13,
            tensile_modulus_xy_MPa=2379, tensile_strength_xy_MPa=43.8, elongation_xy_pct=6.7,
            tensile_modulus_z_MPa=1965, tensile_strength_z_MPa=32, elongation_z_pct=1.65,
            flexural_modulus_xy_MPa=3206, flexural_strength_xy_MPa=73.4,
            hdt_C=100, hdt_load_MPa=1.8, impact_xy_kJ_m2=10.3, impact_z_kJ_m2=6.7,
            impact_method="Notched Charpy ISO 179",
            test_standard="ISO 527, ISO 178, ISO 179 (GB/T equivalents)", specimen="printed",
            specimen_notes=PM_SPEC.format(bed=90, env=90) + " Tested with PolyLite ASA SKU PF01046.",
            conditioning="unknown"),
    print=prt(nozzle_temp_C="230-260", bed_temp_C="75-95", drying="70 C / 7 h"),
    confidence="high",
    notes="Document content is TDS V6.0 titled 'Polymaker ASA' (PolyLite ASA renamed), file named V5.4. "
          "HDT 103 C at 0.45 MPa. Tg 98 C. Enclosure needed. Equilibrium water absorption 0.40% (70% RH)."))

R.append(dict(
    name="polymaker_polylite_pc", brand="Polymaker", product="PolyLite PC", **{"class": "pc"},
    url="https://wiki.polymaker.com/polymaker-products/more-about-our-products/documents/technical-data-sheets/polycarbonate/polylite-tm-pc",
    tds_url="https://cdn.shopify.com/s/files/1/0548/7299/7945/files/PolyLite_PC_TDS_EN_V5.4.pdf?v=1731958076",
    price=None,
    tds=tds(density_g_cm3=1.19,
            tensile_modulus_xy_MPa=2497, tensile_strength_xy_MPa=69.1, elongation_xy_pct=4.8,
            tensile_modulus_z_MPa=2371, tensile_strength_z_MPa=52.8, elongation_z_pct=2.7,
            flexural_modulus_xy_MPa=2640, flexural_strength_xy_MPa=106.1,
            hdt_C=107, hdt_load_MPa=1.8, impact_xy_kJ_m2=4.1,
            impact_method="Notched Charpy ISO 179",
            test_standard="ISO 527, ISO 178, ISO 179 (GB/T equivalents)", specimen="printed",
            specimen_notes=PM_SPEC.format(bed=100, env=90), conditioning="unknown"),
    print=prt(nozzle_temp_C="250-270", bed_temp_C="90-105", drying="75 C for 6 h", annealing="90 C for 2 h"),
    confidence="high",
    notes="TDS V5.4. HDT 111 C at 0.45 MPa; Tg 113 C. Enclosure / heated chamber (70-100 C) needed. "
          "Specimen annealing state not stated."))

R.append(dict(
    name="polymaker_polymax_pc", brand="Polymaker", product="PolyMax PC", **{"class": "pc"},
    url="https://wiki.polymaker.com/polymaker-products/more-about-our-products/documents/technical-data-sheets/polycarbonate/polymax-tm-pc",
    tds_url="https://polymaker.com/wp-content/uploads/TDS_Polymaker_PolyMax-PC_V5.5_2026-01-05_EN.pdf",
    price=None,
    tds=tds(density_g_cm3=1.19,
            tensile_modulus_xy_MPa=2435, tensile_strength_xy_MPa=53.44, elongation_xy_pct=4.53,
            tensile_modulus_z_MPa=2149, tensile_strength_z_MPa=41.43, elongation_z_pct=2.79,
            flexural_modulus_xy_MPa=2050, flexural_strength_xy_MPa=81.29,
            hdt_C=99, hdt_load_MPa=1.8, impact_xy_kJ_m2=21.28,
            impact_method="Notched Charpy ISO 179",
            test_standard="ISO 527, ISO 178, ISO 179 (GB/T equivalents)", specimen="printed",
            specimen_notes=PM_SPEC.format(bed=100, env=90) + " Tested with SKU PC02003.",
            conditioning="unknown"),
    print=prt(nozzle_temp_C="250-270", bed_temp_C="90-105", drying="75 C / 6 h", annealing="90 C / 2 h"),
    confidence="high",
    notes="TDS V5.5 (2026-01-05). HDT 114 C at 0.45 MPa; Tg 113 C; Vicat 117 C. Toughened PC."))

R.append(dict(
    name="polymaker_pc_abs", brand="Polymaker", product="PC-ABS", **{"class": "pc_abs"},
    url="https://wiki.polymaker.com/polymaker-products/more-about-our-products/documents/technical-data-sheets/polycarbonate/polymaker-pc-abs",
    tds_url="https://polymaker.com/wp-content/uploads/TDS_Polymaker_PC-ABS_V5.5_2025-12-10_EN.pdf",
    price=None,
    tds=tds(density_g_cm3=1.1,
            tensile_modulus_xy_MPa=1835, tensile_strength_xy_MPa=39.9, elongation_xy_pct=4.2,
            tensile_modulus_z_MPa=1677, tensile_strength_z_MPa=22.9, elongation_z_pct=1.5,
            flexural_modulus_xy_MPa=2081, flexural_strength_xy_MPa=66.3,
            hdt_C=106, hdt_load_MPa=1.8, impact_xy_kJ_m2=25.8,
            impact_method="Notched Charpy ISO 179",
            test_standard="ISO 527, ISO 178, ISO 179 (GB/T equivalents)", specimen="printed",
            specimen_notes=PM_SPEC.format(bed=100, env=90) + " Tested with SKU PC04001 (black).",
            conditioning="unknown"),
    print=prt(nozzle_temp_C="250-270", bed_temp_C="90-105", drying="75 C / 6 h", annealing="90 C for 2 h"),
    confidence="high",
    notes="TDS V5.5 (2025-12-10). HDT 112 C at 0.45 MPa; Tg 109 C. Low-temp (-30 C) notched Charpy XY 13, Z 1.5 kJ/m2. "
          "Chamber 90-100 C needed. Equilibrium water absorption ~0.35% (70% RH)."))

# ---------------- Prusament ----------------
PR_NOTE = ("Prusa 'Horizontal' mapped to XY. Prusa's second column is 'Vertical xz' (on edge, layers still along the bar) - NOT a "
           "true Z/upright test, so *_z fields are left null and the xz values are in tds.vertical_xz. The TDS's "
           "'Interlayer Adhesion' (Prusa Polymers method) is the closest Z-strength figure and is in tds.interlayer_adhesion_MPa. "
           "Tensile values are yield strength and elongation at yield.")
R.append(dict(
    name="prusament_asa", brand="Prusament", product="ASA", **{"class": "asa"},
    url="https://www.prusa3d.com/product/prusament-asa-signal-white-850g/",
    tds_url="https://storage.googleapis.com/prusa3d-content-prod-14e8-wordpress-prusament-prod/2024/05/bc8551ef-tds_prusament-asa_2024_en.pdf",
    price=price(38.99, "AUD", 0.8, "prusa3d.com (AU)", "800 g spool 38.99 AUD"),
    tds=dict(tds(density_g_cm3=1.07,
                 tensile_modulus_xy_MPa=1600, tensile_strength_xy_MPa=42, elongation_xy_pct=3.4,
                 flexural_modulus_xy_MPa=2000, flexural_strength_xy_MPa=64,
                 shore_hardness="78D", hdt_C=86, hdt_load_MPa=1.8, impact_xy_kJ_m2=25,
                 impact_method="Charpy unnotched ISO 179-1, edgewise (notched: 12)",
                 test_standard="ISO 527-1, ISO 178, ISO 179-1", specimen="printed",
                 specimen_notes="MK3S, 0.20 mm layers, 2 perimeters, 100% rectilinear infill, 265 C nozzle, 110 C bed.",
                 conditioning="unknown"),
             vertical_xz={"tensile_strength_MPa": 45, "tensile_modulus_MPa": 1700, "elongation_at_yield_pct": 3.8,
                          "flexural_strength_MPa": 69, "flexural_modulus_MPa": 1900,
                          "charpy_unnotched_kJ_m2": 38, "charpy_notched_kJ_m2": 15},
             interlayer_adhesion_MPa=11, notched_charpy_xy_kJ_m2=12),
    print=prt(nozzle_temp_C="260 +/- 10", bed_temp_C="110 +/- 5"),
    confidence="high",
    notes="TDS v1.1 (2024-05-20). HDT 93 C at 0.45 MPa. Filament tensile yield 40 MPa. " + PR_NOTE))

R.append(dict(
    name="prusament_pc_blend", brand="Prusament", product="PC Blend", **{"class": "pc"},
    url="https://www.prusa3d.com/product/prusament-pc-blend-jet-black-970g/",
    tds_url="https://prusament.com/wp-content/uploads/2022/10/PCBlend_Prusament_TDS_2022_16_EN.pdf",
    price=price(47.99, "AUD", 0.9, "prusa3d.com (AU)", "900 g spool 47.99 AUD"),
    tds=dict(tds(density_g_cm3=1.22,
                 tensile_modulus_xy_MPa=1900, tensile_strength_xy_MPa=63, elongation_xy_pct=5.8,
                 flexural_modulus_xy_MPa=2100, flexural_strength_xy_MPa=88,
                 shore_hardness="79D", hdt_C=93, hdt_load_MPa=1.8, impact_xy_kJ_m2=None,
                 impact_method="Charpy ISO 179-1 edgewise: unnotched 'no break'; notched 12 kJ/m2",
                 test_standard="ISO 527-1, ISO 178, ISO 179-1", specimen="printed",
                 specimen_notes="MK3S, PrusaSlicer 2.1.1, 0.20 mm layers, 275 C nozzle, 115 C bed, cooling off.",
                 conditioning="unknown"),
             vertical_xz={"tensile_strength_MPa": 63, "tensile_modulus_MPa": 2000, "elongation_at_yield_pct": 5.8,
                          "flexural_strength_MPa": 94, "flexural_modulus_MPa": 2200,
                          "charpy_unnotched_kJ_m2": "no break", "charpy_notched_kJ_m2": 12},
             interlayer_adhesion_MPa=21, notched_charpy_xy_kJ_m2=12),
    print=prt(nozzle_temp_C="275 +/- 10", bed_temp_C="110 +/- 10"),
    confidence="high",
    notes="TDS v1.1 (2022-02-16). HDT 113 C at 0.45 MPa. Filament tensile yield 58 MPa. Unnotched Charpy 'no break' so "
          "impact_xy left null. " + PR_NOTE))

R.append(dict(
    name="prusament_pp_gf", brand="Prusament", product="PP Glass Fiber", **{"class": "pp_gf"},
    url="https://www.prusa3d.com/product/prusament-pp-glass-fiber-850g/",
    tds_url="https://storage.googleapis.com/prusa3d-content-prod-14e8-wordpress-prusament-prod/2025/07/512923fd-tds_pp_glass_fiber_en.pdf",
    price=None,
    tds=dict(tds(density_g_cm3=1.12,
                 tensile_modulus_xy_MPa=2100, tensile_strength_xy_MPa=40.3, elongation_xy_pct=4.5,
                 flexural_modulus_xy_MPa=2100, flexural_strength_xy_MPa=50.2,
                 shore_hardness="68.5D", hdt_C=112.6, hdt_load_MPa=1.8, impact_xy_kJ_m2=17.6,
                 impact_method="Charpy unnotched ISO 179-1, edgewise (notched: 7.9)",
                 test_standard="ISO 527-1, ISO 178, ISO 179-1", specimen="printed",
                 specimen_notes="MK4S, PrusaSlicer 2.8.0, 0.20 mm layers, 2 perimeters, 100% rectilinear, 245 C nozzle, 95 C bed, cooling off.",
                 conditioning="unknown"),
             vertical_xz={"tensile_strength_MPa": 48.8, "tensile_modulus_MPa": 2500, "elongation_at_yield_pct": 4.1,
                          "flexural_strength_MPa": 70.4, "flexural_modulus_MPa": 2700,
                          "charpy_unnotched_kJ_m2": 26.9, "charpy_notched_kJ_m2": 9.9},
             interlayer_adhesion_MPa=18, notched_charpy_xy_kJ_m2=7.9),
    print=prt(nozzle_temp_C="245 +/- 10", bed_temp_C="95 +/- 10", hardened_nozzle_required=True),
    confidence="high",
    notes="TDS v1.0 (2024-11-15). HDT 138.3 C at 0.45 MPa. Filament tensile yield 46.6 MPa. Moisture absorption 0.11% (7 d). "
          "Requires PP print sheet. Glass fibre fraction not stated. Price not found on prusa3d.com page. " + PR_NOTE))

R.append(dict(
    name="prusament_pp_cf", brand="Prusament", product="PP Carbon Fiber", **{"class": "pp_cf"},
    url="https://www.prusa3d.com/product/prusament-pp-carbon-fiber-black-650g/",
    tds_url="https://storage.googleapis.com/prusa3d-content-prod-14e8-wordpress-prusament-prod/2025/01/b4ef2bf6-technical-data-sheet-1.pdf",
    price=price(104.99, "AUD", 0.65, "prusa3d.com (AU)", "650 g spool 104.99 AUD"),
    tds=dict(tds(density_g_cm3=0.91,
                 tensile_modulus_xy_MPa=2100, tensile_strength_xy_MPa=27.3, elongation_xy_pct=3.4,
                 flexural_modulus_xy_MPa=1800, flexural_strength_xy_MPa=27.5,
                 shore_hardness="62D", hdt_C=115, hdt_load_MPa=1.8, impact_xy_kJ_m2=19,
                 impact_method="Charpy unnotched ISO 179-1, edgewise (notched: 14.5)",
                 test_standard="ISO 527-1, ISO 178, ISO 179-1", specimen="printed",
                 specimen_notes="MK4, PrusaSlicer 2.7.4, 0.20 mm layers, 2 perimeters, 100% rectilinear, 270 C nozzle, 85 C bed, cooling off.",
                 conditioning="unknown"),
             vertical_xz={"tensile_strength_MPa": 30.7, "tensile_modulus_MPa": 2500, "elongation_at_yield_pct": 3.2,
                          "flexural_strength_MPa": 41.7, "flexural_modulus_MPa": 3400,
                          "charpy_unnotched_kJ_m2": 17.2, "charpy_notched_kJ_m2": 10.5},
             interlayer_adhesion_MPa=13, notched_charpy_xy_kJ_m2=14.5),
    print=prt(nozzle_temp_C="270 +/- 10", bed_temp_C="85 +/- 10", hardened_nozzle_required=True),
    confidence="high",
    notes="Bonus record (outside unfilled/PP-GF scope; found because prusament.com/materials/prusament-pp/ is the PP CF page - "
          "there is no unfilled Prusament PP). TDS v1.0 (2024-11-15). HDT 158 C at 0.45 MPa. Density 0.91 as stated "
          "(low for a CF-filled grade; recorded as written). Print speed up to 40 mm/s. " + PR_NOTE))

# ---------------- eSun ----------------
R.append(dict(
    name="esun_abs_plus", brand="eSun", product="ABS+", **{"class": "abs"},
    url="https://filament2print.com/en/abs/2911-esun-abs-plus.html",
    tds_url="https://filament2print.com/en/index.php?controller=attachment&id_attachment=2867",
    price=price(16.45, "EUR", 1.0, "filament2print.com", "1 kg, 1.75 mm white, incl. tax"),
    tds=tds(density_g_cm3=1.06, tensile_strength_xy_MPa=40, elongation_xy_pct=30,
            flexural_modulus_xy_MPa=1203, flexural_strength_xy_MPa=68,
            hdt_C=73, hdt_load_MPa=None, impact_xy_kJ_m2=42,
            impact_method="Izod GB/T 1843 (notch state not stated)",
            test_standard="GB/T 1033, GB/T 1040, GB/T 9341, GB/T 1843, GB/T 1634",
            specimen="injection moulded",
            specimen_notes="TDS: properties 'obtained based on the injection molding spline test'.",
            conditioning="unknown"),
    print=prt(nozzle_temp_C="230-270", bed_temp_C="95-110", drying="N/A (not given)"),
    confidence="medium",
    notes="eSun TDS Version 4.0 (Nov 2021), distributor copy via Filament2Print (esun3d.com PDF returns 404). "
          "No tensile modulus given. HDT load not stated. Closed chamber recommended. eSun reformulated ABS+ in 2025; "
          "no newer TDS found."))

R.append(dict(
    name="esun_easa", brand="eSun", product="eASA", **{"class": "asa"},
    url="https://www.esun3d.com/",
    tds_url="https://cdn.shopify.com/s/files/1/1745/6181/files/eSUN_eASA-Filament_TDS_V4.01.pdf",
    price=None,
    tds=tds(density_g_cm3=1.0, tensile_strength_xy_MPa=50, elongation_xy_pct=30,
            flexural_modulus_xy_MPa=4300, flexural_strength_xy_MPa=35,
            hdt_C=88, hdt_load_MPa=0.45, impact_xy_kJ_m2=19,
            impact_method="Izod GB/T 1843 (notch state not stated)",
            test_standard="GB/T 1033, GB/T 1040, GB/T 9341, GB/T 1843, GB/T 1634",
            specimen="injection moulded",
            specimen_notes="TDS: properties obtained from injection-moulded specimens.",
            conditioning="unknown"),
    print=prt(nozzle_temp_C="240-270", bed_temp_C="90-110", drying="N/A (not given)"),
    confidence="low",
    notes="eSun TDS Version 4.0 (Nov 2021; file named V4.01) on eSun's Shopify CDN. Values look internally inconsistent "
          "(density printed as '1'; flexural strength 35 MPa < tensile 50 MPa; flexural modulus 4300 MPa is high for "
          "unfilled ASA) - recorded exactly as written, treat with caution. No tensile modulus given. url is eSun home "
          "(product page not fetched)."))

# ---------------- Fiberlogy ----------------
FL_SPEC_UNK = "Specimen type not stated (generic 'typical value' table, likely resin/supplier data)."
R.append(dict(
    name="fiberlogy_abs", brand="Fiberlogy", product="ABS", **{"class": "abs"},
    url="https://fiberlogy.com/filamenty/abs/",
    tds_url="https://fiberlogy.com/app/uploads/2026/07/FIBERLOGY_ABS_TDS.pdf",
    price=None,
    tds=dict(tds(density_g_cm3=1.05,
                 tensile_modulus_xy_MPa=2305, tensile_strength_xy_MPa=27.8, elongation_xy_pct=6.6,
                 flexural_modulus_xy_MPa=1914, flexural_strength_xy_MPa=52.6,
                 hdt_C=100, hdt_load_MPa=1.8, impact_xy_kJ_m2=17.6,
                 impact_method="Izod notched ISO 180 (printed)",
                 test_standard="ISO 527, ISO 178, ISO 180", specimen="printed",
                 specimen_notes="'3D printed' table: Bambu Lab X1, 0.4 mm nozzle, 265 C, bed 100 C, 0.2 mm layers, 2 perimeters, "
                                "100% rectilinear. Orientation not stated (assumed flat). Tensile strength is at yield "
                                "(break 21.8 MPa); elongation is at break (yield 2.1%).",
                 conditioning="unknown"),
             material_values={"density_g_cm3": 1.05, "tensile_strength_yield_MPa": 45, "tensile_modulus_MPa": 2300,
                              "elongation_yield_pct": 2.5, "flexural_strength_MPa": 68, "flexural_modulus_MPa": 2300,
                              "charpy_notched_kJ_m2": 18, "hdt_1p8_C": "100*", "vicat_C": 97}),
    print=prt(nozzle_temp_C="250-265", bed_temp_C="90-110"),
    confidence="high",
    notes="Fiberlogy TDS last updated 2026-10-05. Main fields = printed-specimen table; generic material table in "
          "tds.material_values. HDT (100 C, asterisked in TDS) and density from the material table."))

R.append(dict(
    name="fiberlogy_asa", brand="Fiberlogy", product="ASA", **{"class": "asa"},
    url="https://fiberlogy.com/filamenty/asa/",
    tds_url="https://fiberlogy.com/app/uploads/2026/05/FIBERLOGY_ASA_TDS.pdf",
    price=None,
    tds=tds(density_g_cm3=1.07, tensile_modulus_xy_MPa=2500, tensile_strength_xy_MPa=43, elongation_xy_pct=25,
            hdt_C=87, hdt_load_MPa=1.8, impact_xy_kJ_m2=10, impact_method="Izod notched ISO 180",
            test_standard="ASTM D792, ISO 527, ISO 180, ASTM D648, ASTM D1525", specimen="unknown",
            specimen_notes=FL_SPEC_UNK + " Tensile strength at yield; elongation at break (yield 3%).",
            conditioning="unknown"),
    print=prt(nozzle_temp_C="255-270", bed_temp_C="90-110"),
    confidence="medium",
    notes="Fiberlogy TDS last updated 2025-05-20 (image PDF, read from rendered page). HDT 96 C at 0.45 MPa; Vicat 96 C. "
          "Net weight 0.75 kg. No flexural data."))

R.append(dict(
    name="fiberlogy_pc_abs", brand="Fiberlogy", product="PC/ABS", **{"class": "pc_abs"},
    url="https://fiberlogy.com/filamenty/abs/pc-abs/",
    tds_url="https://fiberlogy.com/app/uploads/2026/05/FIBERLOGY_PCABS_TDS.pdf",
    price=None,
    tds=tds(density_g_cm3=1.07, tensile_modulus_xy_MPa=2000, tensile_strength_xy_MPa=42, elongation_xy_pct=7,
            flexural_modulus_xy_MPa=2000, flexural_strength_xy_MPa=68,
            impact_xy_kJ_m2=53, impact_method="Charpy notched ISO 179 (Izod notched ISO 180: 55 kJ/m2)",
            test_standard="ISO 1183, ISO 527, ISO 178, ISO 179, ISO 180, ISO 306", specimen="unknown",
            specimen_notes=FL_SPEC_UNK + " Tensile strength at yield.", conditioning="unknown"),
    print=prt(nozzle_temp_C="260-280", bed_temp_C="80-100"),
    confidence="medium",
    notes="Fiberlogy TDS last updated 2025-05-20 (image PDF, read from rendered page). No HDT; Vicat B50 113 C, B120 115 C. "
          "Net weight 0.75 kg."))

R.append(dict(
    name="fiberlogy_pp", brand="Fiberlogy", product="PP", **{"class": "pp"},
    url="https://fiberlogy.com/filamenty/pp/",
    tds_url="https://fiberlogy.com/app/uploads/2026/08/FIBERLOGY_PP_TDS.pdf",
    price=None,
    tds=tds(density_g_cm3=1.05, tensile_modulus_xy_MPa=700, tensile_strength_xy_MPa=14, elongation_xy_pct=None,
            hdt_C=51, hdt_load_MPa=1.8, impact_xy_kJ_m2=30, impact_method="Charpy notched ISO 179, 23 C",
            test_standard="ISO 1183, ISO 527, ISO 179, ISO 75, ISO 306, ISO 11357", specimen="unknown",
            specimen_notes=FL_SPEC_UNK, conditioning="unknown"),
    print=prt(nozzle_temp_C="220-250", bed_temp_C="not required when using packing tape"),
    confidence="medium",
    notes="Fiberlogy TDS last updated 2026-01-12. Elongation at break '>100%' (no number). Vicat A120 110 C, B120 49 C; "
          "Tg -31 C. Density stated 1.05 g/cm3 - unusually high for unfilled PP (~0.9), recorded as written. Net weight 0.75 kg."))

# ---------------- Verbatim ----------------
R.append(dict(
    name="verbatim_pp", brand="Verbatim", product="PP filament", **{"class": "pp"},
    url="https://www.verbatim-europe.com/en/3d/products/verbatim-pp-filament-175mm-500g-natural-55952",
    tds_url="https://www.materialpro3d.cz/user/related_files/pp_datasheet_2018_en.pdf",
    price=None,
    tds=tds(density_g_cm3=0.89, tensile_strength_xy_MPa=14, elongation_xy_pct=None,
            flexural_modulus_xy_MPa=350, flexural_strength_xy_MPa=14, shore_hardness="D55",
            impact_xy_kJ_m2=10, impact_method="Charpy ISO 179, 23 C (notch state not stated)",
            test_standard="ISO 1183, ISO 1133, ISO 306, ISO 178, ISO 179, ISO 527, ISO 868", specimen="unknown",
            specimen_notes="Specimen type not stated.", conditioning="unknown"),
    print=prt(nozzle_temp_C="240 +/- 10 (product page); 200-240 (2018 sheet)", bed_temp_C="80"),
    confidence="medium",
    notes="Values identical on Verbatim's current product page (55952) and a 2018 Verbatim datasheet hosted by distributor "
          "MaterialPro3D. Tensile elongation '>200%'. No tensile modulus or HDT; Vicat 115 C. 0.5 kg spools. "
          "PP film tape recommended for bed."))

# ---------------- 3DXTech ----------------
X3_SPEC = "Printed: open-source FFF, 0.4 mm nozzle, 0.25 mm layers, 100% infill +/-45 deg, specimen orientation XY flat."
R.append(dict(
    name="3dxtech_3dxmax_pc", brand="3DXTech", product="3DXMax PC", **{"class": "pc"},
    url="https://www.3dxtech.com/product/3dxmax-pc-polycarbonate/",
    tds_url="https://filament2print.com/en/index.php?controller=attachment&id_attachment=3720",
    price=price(46.00, "EUR", 0.5, "filament2print.com", "0.5 kg spool 46.00 EUR incl. tax"),
    tds=tds(density_g_cm3=1.2, tensile_modulus_xy_MPa=2410, tensile_strength_xy_MPa=62, elongation_xy_pct=7,
            flexural_modulus_xy_MPa=2200, flexural_strength_xy_MPa=78, hdt_C=135, hdt_load_MPa=0.45,
            test_standard="ISO 1183, ISO 527, ISO 178, ISO 75", specimen="printed",
            specimen_notes=X3_SPEC + " Extrusion 275-295 C, bed 110-120 C.", conditioning="unknown"),
    print=prt(nozzle_temp_C="280-310", bed_temp_C="110-120", drying="65 C 4 h (120 C >4 h for demanding use)"),
    confidence="medium",
    notes="3DXTech TDS Rev 3.0 via Filament2Print distributor copy (3dxtech.com rate-limited). Tg 147 C (Lexan PC). "
          "Print/drying recs from Filament2Print product page. No Z or impact data."))

R.append(dict(
    name="3dxtech_thermax_peek", brand="3DXTech", product="ThermaX PEEK", **{"class": "peek"},
    url="https://www.matterhackers.com/store/l/3dxtech-thermax-peek-05kg-175mm/sk/MWVDJMFR",
    tds_url="https://www.matterhackers.com/r/nZtN6G",
    price=None,
    tds=tds(density_g_cm3=1.30, tensile_modulus_xy_MPa=3720, tensile_strength_xy_MPa=100, elongation_xy_pct=28,
            flexural_modulus_xy_MPa=2700, flexural_strength_xy_MPa=130, hdt_C=140, hdt_load_MPa=0.45,
            test_standard="ISO 1183, ISO 527, ISO 178, ISO 75", specimen="printed",
            specimen_notes=X3_SPEC + " Extrusion 380-400 C, bed 130-140 C.", conditioning="unknown"),
    print=prt(nozzle_temp_C="375-410", bed_temp_C="130-145", drying="120 C for at least 4 h"),
    confidence="medium",
    notes="3DXTech TDS Rev 3.0 via MatterHackers download link. Tg 143 C; UL94 V-0 @1.5 mm (base resin). Enclosure 70-140 C "
          "(hotter better). Crystallinity/annealing state of specimens not stated. Price blank on MatterHackers page; "
          "0.5 kg spools."))

# ---------------- PEEK / PEKK reference ----------------
R.append(dict(
    name="intamsys_peek", brand="INTAMSYS", product="PEEK", **{"class": "peek"},
    url="https://ultimate3dprintingstore.com/collections/3d-printer-filaments-resins/products/intamsys-peek-3d-printer-filament",
    tds_url="https://cdn.shopify.com/s/files/1/0022/2671/6784/files/INTAMSYS_PEEK_TDS_V3.pdf",
    price=None,
    tds=tds(density_g_cm3=1.30, tensile_modulus_xy_MPa=3738, tensile_strength_xy_MPa=99.9, elongation_xy_pct=9.1,
            flexural_modulus_xy_MPa=3612, flexural_strength_xy_MPa=147.0, shore_hardness="85D",
            hdt_C=152, hdt_load_MPa=1.8, impact_xy_kJ_m2=7.1, impact_method="Charpy notched ISO 179",
            test_standard="ISO 527, ISO 178, ISO 179, ISO 75-f", specimen="printed",
            specimen_notes="FUNMAT HT, 400 C, 45 mm/s, 2 shells, 100% infill. Orientation not stated (assumed flat/XY). "
                           "All specimens annealed at 200 C for 2 h.",
            conditioning="annealed"),
    print=prt(nozzle_temp_C="400 (specimen print temp)", annealing="200 C for 2 h (specimens)"),
    confidence="medium",
    notes="INTAMSYS PEEK TDS V3 (2021) from a reseller's Shopify CDN. Density is 'crystalline'. Tg 143 C, Tm 343 C, "
          "water absorption 0.07% (24 h) / 0.4% equilibrium. Recommended print temperatures/drying not on TDS."))

R.append(dict(
    name="kimya_pekk_a", brand="Kimya (Armor)", product="PEKK-A", **{"class": "pekk"},
    url="https://shop3duniverse.com/products/kimya-pekk-a-1-75mm",
    tds_url="https://shop3duniverse.com/cdn/shop/t/137/assets/kimya-pekk-a-3d-filament_en.pdf",
    price=None,
    tds=tds(density_g_cm3=1.261, tensile_modulus_xy_MPa=2510, tensile_strength_xy_MPa=65, elongation_xy_pct=5,
            flexural_modulus_xy_MPa=1660, flexural_strength_xy_MPa=63.2,
            hdt_C=172, hdt_load_MPa=1.8, impact_xy_kJ_m2=2.5, impact_method="Charpy ISO 179-1/1eA (notched)",
            test_standard="ISO 527-2/5A, ISO 178, ISO 179-1, ISO 75f", specimen="printed",
            specimen_notes="Printed XY, 100% rectilinear +/-45 deg, 20-40 mm/s, nozzle 370-380 C, bed 110-125 C, chamber 60-80 C. "
                           "Tensile stress at break 48 MPa; strain at break >5%. Flexural value is stress at conventional "
                           "deflection (3.5% strain), test ended at 5% without break.",
            conditioning="unknown"),
    print=prt(nozzle_temp_C="370-380 (2019 TDS); 395-400 XY (2025 Airtech TDS)",
              bed_temp_C="110-125 (2019 TDS); 150-160 (2025 Airtech TDS)"),
    confidence="medium",
    notes="Kimya TDS created 2018-09-11, revised 2019-11-25 (reseller copy). Amorphous PEKK from Arkema Kepstan. "
          "Tg 159 C, Tm 308 C, max use temp 150 C, UL94 V0. HDT 172 C listed under filament properties. A newer Airtech "
          "Europe TDS (2025-12-22, makershop.fr) has no mechanical data but revises print settings (chamber 150-160 C)."))

R.append(dict(
    name="ultrafuse_pp_gf30", brand="BASF Forward AM (Ultrafuse)", product="Ultrafuse PP GF30", **{"class": "pp_gf"},
    url="https://forward-am.com/material-portfolio/ultrafuse-filaments-for-fused-filaments-fabrication-fff/reinforced-filaments/ultrafuse-pp-gf30/",
    tds_url="https://www.farnell.com/datasheets/2913585.pdf",
    price=None,
    tds=dict(tds(density_g_cm3=1.066,
                 tensile_modulus_xy_MPa=2628, tensile_strength_xy_MPa=41.7, elongation_xy_pct=4.4,
                 tensile_modulus_z_MPa=2242, tensile_strength_z_MPa=15.9, elongation_z_pct=0.8,
                 flexural_modulus_xy_MPa=3507, flexural_strength_xy_MPa=76.8,
                 flexural_modulus_z_MPa=1671, flexural_strength_z_MPa=19.3,
                 hdt_C=73, hdt_load_MPa=1.8, impact_xy_kJ_m2=23.1, impact_z_kJ_m2=2.5,
                 impact_method="Charpy unnotched ISO 179-2 (notched: XY 5.3, Z 1.2)",
                 test_standard="ISO 527, ISO 178, ISO 179-2, ISO 180, ISO 75-2", specimen="printed",
                 specimen_notes="XY = flat, XZ = on edge, ZX = upright (Z). Density is printed-part density.",
                 conditioning="unknown"),
             edge_xz={"flexural_strength_MPa": 95.3, "flexural_modulus_MPa": 4026,
                      "charpy_unnotched_kJ_m2": 25.8, "charpy_notched_kJ_m2": 5.2}),
    print=prt(nozzle_temp_C="240-260", bed_temp_C="20-40 (PP tape) / 70-90 (PPGF adhesive)",
              drying="60 C, 4-16 h (hot air or vacuum)"),
    confidence="medium",
    notes="BASF Ultrafuse PP GF30 TDS v2.3 (2019-11-14) via Farnell distributor copy; product now sold by Forward AM. "
          "30% glass fibre. HDT 127 C at 0.45 MPa. Nozzle >= 0.6 mm recommended; hardened nozzle not stated."))

names = [r["name"] for r in R]
assert len(names) == len(set(names)), "duplicate names"
with open(OUT, "w", encoding="utf-8") as f:
    json.dump(R, f, indent=2, ensure_ascii=False)
print(len(R), "records written")
