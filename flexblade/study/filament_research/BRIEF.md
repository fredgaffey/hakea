# Filament datasheet research brief (shared by all research agents)

Purpose: a database of REAL 3D-printing filament / insert material data for a rotor-blade
flexure simulator. Values are used to compute stiffness, mass, strength, creep and fatigue
of printed parts. Accuracy and traceability matter more than coverage. Today is 2026-11-20.

## Rules
- NEVER invent or "typical-value" a number. Every number you record must appear on the
  manufacturer's product page or TDS (or a distributor page that reproduces the TDS — say so
  in notes). If a field is not given, put null. Do not convert hardness to modulus, etc.
- Record exactly what the source says, in the units below (convert units only: GPa->MPa,
  psi->MPa (1 psi = 0.006895 MPa), g/cm3). If the TDS gives both dry and conditioned values,
  record dry in the main fields and put conditioned values in `tds.conditioned` (same keys).
- If a TDS gives values for several print orientations, map them: XY / "horizontal" /
  "flat" / "X-Y" / along raster -> *_xy; Z / "vertical" / "upright" -> *_z. If the TDS gives
  only injection-moulded or filament (not printed) values, record them in the *_xy fields
  and set `tds.specimen` = "injection moulded" / "filament" accordingly.
- `price`: approximate current list price for 1 kg (normalise per kg; note spool size) in the
  store's native currency (USD/EUR/GBP/AUD...), with date "2026-11" and the store.
  Prefer the manufacturer's own store. null if not findable.
- `url` = product (purchase) page. `tds_url` = direct TDS PDF/page link (null if none).
- Use WebSearch + WebFetch. PDFs can be fetched with WebFetch. If a site blocks fetching, try
  the manufacturer's other regional store, a distributor (e.g. Matterhackers, Printed Solid,
  3DJake, Filament2Print), or the PDF on a CDN. Spend effort proportionate: ~3-6 fetches per
  product max; record what you got with confidence "low" if partial.
- confidence: "high" = full TDS fetched and values read directly; "medium" = product page
  table or distributor copy of TDS; "low" = partial / ambiguous / old version.

## Output
Write a single JSON file (UTF-8, a JSON array of records) to the path given in your task.
Validate it parses (python3 -c "import json;json.load(open(PATH))"). Record schema:

```json
{
  "name": "bambu_paht_cf",              // snake_case brand_product, unique, [a-z0-9_]
  "brand": "Bambu Lab",
  "product": "PAHT-CF",
  "class": "pa_cf",                     // one of the class codes below
  "url": "https://...",                 // product / purchase page
  "tds_url": "https://...pdf",          // or null
  "price": {"value": 59.99, "currency": "USD", "per_kg": true, "spool_kg": 1.0,
            "date": "2026-11", "store": "bambulab.com", "note": ""},   // or null
  "tds": {
    "density_g_cm3": 1.06,
    "tensile_modulus_xy_MPa": 4230, "tensile_strength_xy_MPa": 94, "elongation_xy_pct": 3.2,
    "tensile_modulus_z_MPa": null,  "tensile_strength_z_MPa": 41,  "elongation_z_pct": null,
    "flexural_modulus_xy_MPa": 4460, "flexural_strength_xy_MPa": 160,
    "flexural_modulus_z_MPa": null,  "flexural_strength_z_MPa": null,
    "shore_hardness": null,            // string as written, e.g. "95A", "64D"
    "hdt_C": 194, "hdt_load_MPa": 0.45, // HDT and the load it was measured at (0.45 or 1.8)
    "impact_xy_kJ_m2": 12.6, "impact_z_kJ_m2": null, "impact_method": "Charpy unnotched ISO 179",
    "tear_strength_kN_m": null,          // TPU only, if given
    "test_standard": "ISO 527, ISO 178", // as stated
    "specimen": "printed",               // "printed" | "injection moulded" | "filament" | "unknown"
    "specimen_notes": "100% infill, XY = flat, Z = upright, 0.4 nozzle",
    "conditioning": "dry",               // "dry" | "conditioned" | "annealed" | "unknown"
    "conditioned": null                  // or dict with the same keys for conditioned/annealed values
  },
  "print": {
    "nozzle_temp_C": "260-290", "bed_temp_C": "80-100",
    "hardened_nozzle_required": true,     // true / false / null if not stated
    "ams_compatible": false,              // Bambu AMS / multi-material feeder compatibility if stated, else null
    "drying": "80 C 12 h",               // as stated, or null
    "moisture_sensitivity": "high",       // "low" | "medium" | "high" | null (from TDS/vendor statement)
    "annealing": null
  },
  "confidence": "high",
  "notes": "TDS v2.1 (2024). Z values from 'Z-direction' row. Price on sale."
}
```

For INSERT materials (rods, wire, tubes) use the same record with `class` "cf_rod",
"gf_rod", "steel_wire", "cf_continuous", and put these extra keys in `tds`:
`fibre_modulus_GPa` (if stated), `rod_modulus_GPa` (the composite / wire tensile or
flexural modulus actually stated), `diameters_mm` (list of sizes sold), `tensile_strength_MPa`
(use tensile_strength_xy_MPa), and note which modulus (tensile/flexural) the seller states.

## Class codes
tpu, tpe, pla, pla_plus, pla_tough, pla_ht, pla_cf, petg, petg_cf, pet_cf, abs, asa, abs_cf,
asa_cf, pc, pc_abs, pc_cf, pa (unfilled PA6/PA12/PA11/copolymer), pa_cf, pa_gf, ppa_cf,
pp, pp_gf, pp_cf, peek, pekk, peek_cf, onyx (Markforged micro-CF nylon), cf_continuous,
cf_rod, gf_rod, steel_wire.
