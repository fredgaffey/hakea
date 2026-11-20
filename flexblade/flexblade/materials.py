"""Material library with print anisotropy and frequency-dependent complex modulus.

Values are representative datasheet / literature numbers for FDM parts and are
*rough*: they exist so trends and feasibility come out right. Every material has
`source` notes; refine with coupon tests (DMA at ~1/rev frequency for TPU).

Orthotropic convention: axis 1 = raster/fibre direction, 2 = in-layer transverse,
3 = build (layer stacking) direction. Interlayer properties = 3-direction.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field

import numpy as np

from .schema import PrintOrientation


T_REF_C = 25.0          # reference temperature of all tabulated properties
DERATE_FLOOR = 0.3      # knock-down at/after HDT (and beyond)


def temp_knockdown(T_C: float, hdt_C: float) -> float:
    """Simple documented derating law, applied to moduli, strengths, creep and fatigue
    allowables alike: 1 at T <= 25 C, falling linearly to DERATE_FLOOR (0.3) at the
    heat-deflection temperature, constant beyond (the hdt check fails there instead).
    hdt_C <= 25 means n/a -> 1. ROUGH: a coupon test at temperature supersedes it."""
    if hdt_C <= T_REF_C or T_C <= T_REF_C:
        return 1.0
    return max(DERATE_FLOOR, 1.0 - (1.0 - DERATE_FLOOR) * (T_C - T_REF_C) / (hdt_C - T_REF_C))


@dataclass(frozen=True)
class Material:
    name: str
    rho: float                    # kg/m^3
    E1: float                     # Pa, along raster
    E2: float                     # Pa, in-layer transverse
    E3: float                     # Pa, interlayer (build dir)
    G12: float
    G13: float
    G23: float
    nu12: float
    nu13: float
    nu23: float
    loss_factor: float = 0.01     # η: E* = E (1 + iη) at reference frequency
    # storage-modulus multiplier vs frequency (Hz), log-linear interpolation.
    # 1.0 at the quasi-static frequency where E1.. were measured.
    freq_scale: tuple[tuple[float, float], ...] = ((0.01, 1.0),)
    # loss factor vs frequency (optional); empty -> constant loss_factor
    freq_loss: tuple[tuple[float, float], ...] = ()
    # strengths, Pa
    Xt: float = 50e6              # tensile along raster
    Yt: float = 40e6              # tensile in-layer transverse
    Zt: float = 25e6              # tensile interlayer
    S: float = 25e6               # shear
    # sustained (creep) strain allowable over design life, and fatigue strain/stress
    creep_strain_allow: float = 0.01
    fatigue_strain_amp_allow: float = 0.005   # at 1e7 cycles
    fatigue_stress_ratio: float = 0.3         # fatigue strength ≈ ratio × static, 1e7
    is_elastomer: bool = False
    source: str = ""
    # post-print heat forming: working temperature of a part made of this material (°C), and
    # glass-transition / softening Tg for materials that can be inserted pre-forming (0 = n/a).
    # ROUGH, assumed values (see FORMING_TEMP_C / TG_C below) -- verify with a coupon.
    forming_temp_C: float = 0.0
    tg_C: float = 0.0
    # heat-deflection temperature (C, ~0.45 MPa); 0 = n/a (rods, wire: no derating)
    hdt_C: float = 0.0
    # provenance of creep_strain_allow / fatigue_*: "class default" unless brand-sourced
    allow_source: str = "class default"

    # ------------------------------------------------------------------ temperature
    def derated(self, T_C: float) -> "Material":
        """Copy with moduli and allowables knocked down for part temperature T_C
        (see temp_knockdown). Returns self at/below 25 C or when hdt_C is n/a."""
        k = temp_knockdown(T_C, self.hdt_C)
        if k >= 1.0:
            return self
        return dataclasses.replace(
            self, E1=self.E1 * k, E2=self.E2 * k, E3=self.E3 * k, G12=self.G12 * k,
            G13=self.G13 * k, G23=self.G23 * k, Xt=self.Xt * k, Yt=self.Yt * k,
            Zt=self.Zt * k, S=self.S * k, creep_strain_allow=self.creep_strain_allow * k,
            fatigue_strain_amp_allow=self.fatigue_strain_amp_allow * k,
            source=self.source + f" [derated x{k:.3f} at {T_C:.0f} C]")

    # ------------------------------------------------------------------ frequency
    def storage_scale(self, freq_hz: float) -> float:
        f = np.array([p[0] for p in self.freq_scale])
        s = np.array([p[1] for p in self.freq_scale])
        return float(np.interp(np.log10(max(freq_hz, 1e-6)), np.log10(f), s))

    def loss(self, freq_hz: float) -> float:
        if not self.freq_loss:
            return self.loss_factor
        f = np.array([p[0] for p in self.freq_loss])
        e = np.array([p[1] for p in self.freq_loss])
        return float(np.interp(np.log10(max(freq_hz, 1e-6)), np.log10(f), e))

    def complex_scale(self, freq_hz: float) -> complex:
        """Multiply quasi-static moduli by this to get complex modulus at freq."""
        return self.storage_scale(freq_hz) * (1.0 + 1j * self.loss(freq_hz))

    # ------------------------------------------------------------------ elasticity
    def compliance_local(self) -> np.ndarray:
        """6x6 Voigt compliance in material axes (11,22,33,23,13,12)."""
        S = np.zeros((6, 6))
        S[0, 0] = 1 / self.E1
        S[1, 1] = 1 / self.E2
        S[2, 2] = 1 / self.E3
        S[0, 1] = S[1, 0] = -self.nu12 / self.E1
        S[0, 2] = S[2, 0] = -self.nu13 / self.E1
        S[1, 2] = S[2, 1] = -self.nu23 / self.E2
        S[3, 3] = 1 / self.G23
        S[4, 4] = 1 / self.G13
        S[5, 5] = 1 / self.G12
        return S

    def stiffness_local(self) -> np.ndarray:
        return np.linalg.inv(self.compliance_local())

    def stiffness_matrix(self, orientation: PrintOrientation | None = None) -> np.ndarray:
        """6x6 Voigt stiffness in hub frame H (engineering shear strains)."""
        R = rotation_matrix(orientation)
        T = _voigt_stress_transform(R)  # σ_H = T σ_local
        C = self.stiffness_local()
        # with engineering strain: ε_H = T^{-T} ε_local → C_H = T C T^T
        return T @ C @ T.T

    def plate_modulus(self, orientation: PrintOrientation | None, t_hat, w_hat, n_hat) -> float:
        """Cylindrical-bending modulus Q11 of a wide plate (length t_hat, width w_hat, normal
        n_hat): plane stress (sigma_n = 0) with eps_w = gamma_tw = 0 (width fully restrained).
        Equals E/(1-nu^2) for isotropic material; larger than axial_modulus when the length
        direction is off the material axes (shear coupling)."""
        R = np.array([_local_dir(orientation, v) for v in (t_hat, w_hat, n_hat)])
        pairs = [(0, 0), (1, 1), (2, 2), (1, 2), (0, 2), (0, 1)]
        K = np.zeros((6, 6))
        for i, (a, b) in enumerate(pairs):
            for j, (c, d) in enumerate(pairs):
                K[i, j] = R[a, c] * R[b, d] if c == d else R[a, c] * R[b, d] + R[a, d] * R[b, c]
        SN = np.linalg.inv(K @ self.stiffness_local() @ K.T)   # compliance, plate axes (L,W,N)
        idx = [0, 1, 5]
        return float(np.linalg.inv(SN[np.ix_(idx, idx)])[0, 0])

    def axial_modulus(self, orientation: PrintOrientation | None, t_hat) -> float:
        """Young's modulus along unit direction t_hat (frame H)."""
        a = _local_dir(orientation, t_hat)
        S = self.compliance_local()
        # uniaxial stress along a: σ_local = a_i a_j
        s = np.array([a[0]**2, a[1]**2, a[2]**2, a[1]*a[2], a[0]*a[2], a[0]*a[1]])
        eps = S @ s
        strain_a = eps[0]*a[0]**2 + eps[1]*a[1]**2 + eps[2]*a[2]**2 \
            + eps[3]*a[1]*a[2] + eps[4]*a[0]*a[2] + eps[5]*a[0]*a[1]
        return float(1.0 / strain_a)

    def shear_modulus(self, orientation: PrintOrientation | None, t_hat, n_hat) -> float:
        """Shear modulus for shear in plane spanned by t_hat, n_hat (frame H)."""
        a = _local_dir(orientation, t_hat)
        b = _local_dir(orientation, n_hat)
        b = b - a * (a @ b)
        b /= np.linalg.norm(b)
        # pure shear stress τ (a⊗b + b⊗a)
        sig = np.outer(a, b) + np.outer(b, a)
        s = np.array([sig[0, 0], sig[1, 1], sig[2, 2], sig[1, 2], sig[0, 2], sig[0, 1]])
        eps = self.compliance_local() @ s
        epsT = np.array([[eps[0], eps[5]/2, eps[4]/2],
                         [eps[5]/2, eps[1], eps[3]/2],
                         [eps[4]/2, eps[3]/2, eps[2]]])
        gamma = 2 * a @ epsT @ b
        return float(1.0 / gamma)

    def strength_along(self, orientation: PrintOrientation | None, t_hat) -> float:
        """Conservative tensile strength for uniaxial stress along t_hat (Hill-type)."""
        a = _local_dir(orientation, t_hat)
        inv = (a[0]**2 / self.Xt)**2 + (a[1]**2 / self.Yt)**2 + (a[2]**2 / self.Zt)**2 \
            + ((a[0]*a[1])**2 + (a[0]*a[2])**2 + (a[1]*a[2])**2) / self.S**2
        return float(1.0 / np.sqrt(inv))


def _normalize(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def rotation_matrix(orientation: PrintOrientation | None) -> np.ndarray:
    """Columns = material axes 1,2,3 expressed in frame H."""
    if orientation is None:
        orientation = PrintOrientation()
    e3 = _normalize(orientation.build_dir)
    e1 = np.asarray(orientation.raster_dir, float)
    e1 = e1 - e3 * (e1 @ e3)
    if np.linalg.norm(e1) < 1e-9:
        e1 = np.array([1.0, 0, 0]) if abs(e3[0]) < 0.9 else np.array([0, 1.0, 0])
        e1 = e1 - e3 * (e1 @ e3)
    e1 = _normalize(e1)
    e2 = np.cross(e3, e1)
    return np.column_stack([e1, e2, e3])


def _local_dir(orientation, v) -> np.ndarray:
    return rotation_matrix(orientation).T @ _normalize(v)


def _voigt_stress_transform(R: np.ndarray) -> np.ndarray:
    """T such that σ_H(voigt) = T σ_local(voigt), order 11,22,33,23,13,12."""
    idx = [(0, 0), (1, 1), (2, 2), (1, 2), (0, 2), (0, 1)]
    T = np.zeros((6, 6))
    for I, (i, j) in enumerate(idx):
        for J, (k, l) in enumerate(idx):
            if k == l:
                T[I, J] = R[i, k] * R[j, l]
            else:
                T[I, J] = R[i, k] * R[j, l] + R[i, l] * R[j, k]
    return T


def _iso(name, rho, E, nu, **kw) -> Material:
    G = E / (2 * (1 + nu))
    return Material(name, rho, E, E, E, G, G, G, nu, nu, nu, **kw)


def _fdm(name, rho, E1, E2, E3, nu, **kw) -> Material:
    G12 = E2 / (2 * (1 + nu)) * 1.0
    G13 = E3 / (2 * (1 + nu))
    G23 = E3 / (2 * (1 + nu))
    return Material(name, rho, E1, E2, E3, G12, G13, G23, nu, nu, nu, **kw)


# TPU: strong frequency dependence. Storage modulus at ~300-500 Hz roughly 1.5-2.5x
# the quasi-static tensile modulus for 95A-64D TPU at 25 C (DMA literature, TTS);
# loss factor 0.08-0.2. Interlayer stiffness of well-fused TPU ≈ 0.85-0.95 bulk,
# interlayer strength ≈ 50-70 % bulk. Creep: TPU creeps noticeably above ~1-2 %
# sustained strain; fatigue strain amplitude ~ few % for elastomers but print
# defects dominate — kept conservative.
_TPU_F = ((0.01, 1.0), (1.0, 1.15), (10.0, 1.3), (100.0, 1.55), (1000.0, 1.9), (1e4, 2.3))
_TPU_LOSS = ((0.01, 0.08), (10.0, 0.10), (100.0, 0.12), (1000.0, 0.15), (1e4, 0.18))

LIBRARY: dict[str, Material] = {
    "tpu_95a": _fdm("tpu_95a", 1210, 26e6, 24e6, 22e6, 0.45, loss_factor=0.12,
                    freq_scale=_TPU_F, freq_loss=_TPU_LOSS,
                    Xt=35e6, Yt=30e6, Zt=18e6, S=15e6,
                    creep_strain_allow=0.02, fatigue_strain_amp_allow=0.03,
                    fatigue_stress_ratio=0.25, is_elastomer=True,
                    source="generic 95A TPU datasheets; DMA trends; rough"),
    "tpu_64d": _fdm("tpu_64d", 1220, 90e6, 85e6, 75e6, 0.42, loss_factor=0.10,
                    freq_scale=_TPU_F, freq_loss=_TPU_LOSS,
                    Xt=40e6, Yt=35e6, Zt=20e6, S=18e6,
                    creep_strain_allow=0.015, fatigue_strain_amp_allow=0.02,
                    fatigue_stress_ratio=0.25, is_elastomer=True,
                    source="stiff TPU (e.g. 64D/72D grades) datasheets; rough"),
    "pa6_cf": _fdm("pa6_cf", 1200, 7.5e9, 3.2e9, 2.4e9, 0.35, loss_factor=0.02,
                   freq_scale=((0.01, 1.0), (1000.0, 1.08)),
                   Xt=110e6, Yt=55e6, Zt=35e6, S=35e6,
                   creep_strain_allow=0.004, fatigue_strain_amp_allow=0.003,
                   fatigue_stress_ratio=0.3,
                   source="PA6-CF / PAHT-CF dry, XY along raster; humidity drops ~30 %"),
    "petg_cf": _fdm("petg_cf", 1300, 4.0e9, 2.4e9, 1.9e9, 0.36, loss_factor=0.02,
                    Xt=55e6, Yt=35e6, Zt=22e6, S=22e6,
                    creep_strain_allow=0.003, fatigue_strain_amp_allow=0.0025,
                    fatigue_stress_ratio=0.3, source="PETG-CF datasheets; rough"),
    "pla": _fdm("pla", 1240, 3.4e9, 3.1e9, 2.8e9, 0.36, loss_factor=0.015,
                Xt=55e6, Yt=45e6, Zt=28e6, S=25e6,
                creep_strain_allow=0.003, fatigue_strain_amp_allow=0.002,
                fatigue_stress_ratio=0.25, source="reference only"),
    "cf_rod_pultruded": Material(
        "cf_rod_pultruded", 1550, 125e9, 8e9, 8e9, 4.5e9, 4.5e9, 3e9, 0.3, 0.3, 0.4,
        loss_factor=0.005, Xt=1500e6, Yt=50e6, Zt=50e6, S=60e6,
        creep_strain_allow=0.008, fatigue_strain_amp_allow=0.005, fatigue_stress_ratio=0.5,
        source="pultruded CF rod, fibre along raster_dir (set raster_dir = rod axis)"),
    "steel_302_spring": _iso("steel_302_spring", 7900, 193e9, 0.29, loss_factor=0.002,
                             Xt=1300e6, Yt=1300e6, Zt=1300e6, S=700e6,
                             creep_strain_allow=0.005, fatigue_strain_amp_allow=0.002,
                             fatigue_stress_ratio=0.35, source="spring-temper stainless wire"),
    # ---- sheet stock clamped in printed fittings (not printed; isotropic) ----------------
    # Spring-steel shim / feeler-gauge stock, AISI 1095 blue-tempered (HRC 48-52) or 301
    # stainless full-hard: E 193-207 GPa (200 used), ν 0.29, ρ 7850; yield 1.2-1.6 GPa
    # (1095 blue-tempered UTS ~1.7 GPa, 301 FH UTS ≥ 1.28 GPa, yield ≥ 0.96 GPa; Xt = 1.4 GPa
    # used). Fully reversed bending fatigue of polished spring strip at 1e8+ cycles ~550-700
    # MPa → strain amplitude 0.25-0.35 % (0.28 % used; edges deburred/stoned — sheared edges
    # cut it by ~30 %). Stock thicknesses: SHIM_THICKNESSES. Creep: none at rotor temperatures.
    "steel_1095_shim": _iso("steel_1095_shim", 7850, 200e9, 0.29, loss_factor=0.001,
                            Xt=1400e6, Yt=1400e6, Zt=1400e6, S=800e6,
                            creep_strain_allow=0.005, fatigue_strain_amp_allow=0.0028,
                            fatigue_stress_ratio=0.40,
                            source="AISI 1095 / 301 FH spring shim (feeler stock): E 200 GPa, "
                                   "Xt 1.4 GPa, fatigue ε_a 0.28 % at 1e8"),
    # FR4 / G10 glass-epoxy sheet (NEMA LI-1; e.g. Isola/Rogers G10 datasheets): in-plane
    # E 17-24 GPa (20 used), through-thickness ~10 GPa, ρ 1850, flexural strength ~400 MPa,
    # tensile ~300 MPa; flexural fatigue at 1e7-1e8 ~ 25-30 % static → ε_a ~0.4 %. Sheets
    # 0.2/0.4/0.8 mm. Moisture and edge delamination: seal cut edges.
    "fr4_g10": Material("fr4_g10", 1850, 20e9, 20e9, 10e9, 8e9, 5e9, 5e9, 0.13, 0.3, 0.3,
                        loss_factor=0.01, Xt=300e6, Yt=300e6, Zt=40e6, S=60e6,
                        creep_strain_allow=0.006, fatigue_strain_amp_allow=0.004,
                        fatigue_stress_ratio=0.27,
                        source="FR4/G10 sheet: E 20 GPa in-plane, Xt 300 MPa, ε_a 0.4 % (1e8)"),
    # ---- tension-torsion strap strands (unbonded yarn/tow/braid; fibre along raster_dir =
    # strand axis). E1/Xt are for the *fibre-equivalent* solid diameter (strand diameter given
    # as the solid fibre area equivalent, not the bulky yarn envelope). Transverse values are
    # nominal (dry strands have ~no shear/transverse strength; rotor.py scales strand EI/GJ by
    # the strap bend factor, so they barely enter). creep_strain_allow = sustained-stress
    # fraction × Xt / E1 (the creep check compares elastic strain at trim with it).
    # Kevlar 49 yarn (DuPont Kevlar technical guide): ρ 1440, E 112 GPa, yarn UTS ~3.0 GPa,
    # 2.4 % elongation; Xt knocked to 2.4 GPa for strand translation efficiency. Creep is low
    # (~0.03 % over 1e4 h at 50 % UTS) and creep-rupture life is long at ≤ 40-50 % UTS → allow
    # 0.4 × 2.4/112 ≈ 0.0086. Poor in transverse compression/abrasion (pin bends!).
    "aramid_yarn": Material(
        "aramid_yarn", 1440, 112e9, 2.5e9, 2.5e9, 1.5e9, 1.5e9, 1.0e9, 0.36, 0.36, 0.4,
        loss_factor=0.02, Xt=2.4e9, Yt=20e6, Zt=20e6, S=30e6,
        creep_strain_allow=0.0086, fatigue_strain_amp_allow=0.004, fatigue_stress_ratio=0.4,
        source="Kevlar 49 yarn (DuPont guide): E 112 GPa, UTS 3.0 GPa (2.4 used, strand eff.); "
               "creep allow at 40 % Xt"),
    # Dyneema SK75 fibre (DSM datasheet): ρ 975, E 109-132 GPa, UTS 3.4-3.6 GPa; as a 12-strand
    # braid the helix angle drops it to ~E 70 GPa, UTS ~2.0 GPa (braid efficiency ~0.6).
    # FLAG: UHMWPE CREEPS — SK75 creeps several % over years at 20 % UTS at 20 C and creep
    # rate rises ~10x per +20 C (motor heat). Sustained load must stay ≲ 10 % UTS (SK78/SK99
    # ~3-5x better): allow 0.10 × 2.0/70 ≈ 0.0029. Melts 144-152 C; bonds poorly to epoxy.
    "uhmwpe_braid": Material(
        "uhmwpe_braid", 975, 70e9, 1.5e9, 1.5e9, 0.8e9, 0.8e9, 0.6e9, 0.4, 0.4, 0.45,
        loss_factor=0.03, Xt=2.0e9, Yt=10e6, Zt=10e6, S=15e6,
        creep_strain_allow=0.0029, fatigue_strain_amp_allow=0.006, fatigue_stress_ratio=0.35,
        source="Dyneema SK75 (DSM): E 109-132 GPa, UTS 3.5 GPa; braid E 70 GPa, UTS 2.0 GPa; "
               "CREEP-LIMITED: sustained ≤ 10 % UTS"),
    # T700S 12K carbon tow (Toray datasheet): ρ 1800, E 230 GPa, UTS 4.9 GPa. Dry unbonded tow:
    # filament misalignment and abrasion → E ~200 GPa, strength efficiency ~0.5 → 2.5 GPa.
    # Negligible creep (allow 0.65 × 2.5/200 ≈ 0.008). Brittle over small bend radii (pin
    # wrap needs D/d ≫ 10) and filaments fret at the anchor under 1/rev motion.
    "cf_tow": Material(
        "cf_tow", 1800, 200e9, 3.0e9, 3.0e9, 1.5e9, 1.5e9, 1.0e9, 0.3, 0.3, 0.4,
        loss_factor=0.005, Xt=2.5e9, Yt=20e6, Zt=20e6, S=30e6,
        creep_strain_allow=0.008, fatigue_strain_amp_allow=0.003, fatigue_stress_ratio=0.5,
        source="Toray T700S 12K: E 230 GPa, UTS 4.9 GPa; dry tow E 200 GPa, Xt 2.5 GPa"),
}
STRAP_MATERIALS = ("aramid_yarn", "uhmwpe_braid", "cf_tow")
# sheet stock cut into strips and clamped (embedded) in the printed fittings, not printed
SHEET_MATERIALS = ("steel_1095_shim", "fr4_g10")
SHIM_THICKNESSES = {"steel_1095_shim": (5e-5, 8e-5, 1.0e-4, 1.3e-4, 1.5e-4, 2.0e-4),
                    "fr4_g10": (2e-4, 4e-4)}


# assumed: TPU 95A/64D soften ~100-130 C; PETG-CF Tg ~85 C (form ~90-100); PLA Tg ~60 C;
# PA6-CF needs ~150-170 C; pultruded CF rod epoxy/vinyl-ester matrix Tg ~110 C (ASSUMPTION);
# steel spring wire does not care below its tempering temperature.
# ROUGH heat-deflection temperatures of the generic entries (0 = n/a, no derating)
HDT_C = {"uhmwpe_braid": 100.0, "tpu_95a": 90.0, "tpu_64d": 90.0, "petg_cf": 75.0, "pla": 55.0, "pa6_cf": 150.0}
# class-default HDT (C) for brand entries whose TDS gives none; inserts -> 0 (n/a)
CLASS_HDT_C = {"pla": 55, "pla_plus": 55, "pla_tough": 55, "pla_ht": 100, "pla_cf": 60, "petg": 70,
               "petg_cf": 75, "pet_cf": 80, "abs": 95, "abs_cf": 100, "abs_gf": 100, "asa": 95,
               "asa_cf": 100, "pc": 130, "pc_abs": 105, "pc_cf": 135, "pa": 70, "pa_cf": 150,
               "pa_gf": 160, "ppa_cf": 200, "onyx": 145, "pp": 60, "pp_cf": 70, "pp_gf": 90,
               "peek": 150, "peek_cf": 200, "pekk": 160, "pps_cf": 250, "tpu": 90, "tpe": 80}
FORMING_TEMP_C = {"tpu_95a": 100.0, "tpu_64d": 105.0, "petg_cf": 90.0, "pla": 65.0, "pa6_cf": 160.0}
TG_C = {"cf_rod_pultruded": 110.0, "steel_302_spring": 400.0, "steel_1095_shim": 400.0}
# FR4 Tg ~130-140 C: HDT-type derating applies
HDT_C_SHEET = {"fr4_g10": 130.0}
for _n, _m in list(LIBRARY.items()):
    LIBRARY[_n] = dataclasses.replace(_m, forming_temp_C=FORMING_TEMP_C.get(_n, 0.0), tg_C=TG_C.get(_n, 0.0),
                                      hdt_C=HDT_C.get(_n, HDT_C_SHEET.get(_n, 0.0)))
GENERIC_NAMES = tuple(LIBRARY)

# Metadata of the real-product entries (brand, product, url, price, confidence, …) keyed by
# material name; generic entries above have none. Filled by _register_filaments().
META: dict[str, dict] = {}


def _register_filaments() -> None:
    """Register every entry of data/filaments.json as a Material (see filament_db.py)."""
    from .filament_db import CLASSES, load_entries
    for e in load_entries():
        s = e["sim"]
        if e["name"] in LIBRARY and e["name"] not in META:
            raise ValueError(f"filament entry {e['name']!r} clashes with a generic material")
        hdt = (e.get("tds") or {}).get("hdt_C") or CLASS_HDT_C.get(e["class"], 0.0)
        dv = e["derived"]
        sourced = all(not str(dv.get(k, "")).startswith(("class table", "elastomer hardness"))
                      for k in ("creep_strain_allow", "fatigue_strain_amp_allow"))
        LIBRARY[e["name"]] = Material(
            e["name"], s["rho"], s["E1"], s["E2"], s["E3"], s["G12"], s["G13"], s["G23"],
            s["nu12"], s["nu13"], s["nu23"], loss_factor=s["loss_factor"],
            freq_scale=tuple(tuple(p) for p in s["freq_scale"]),
            freq_loss=tuple(tuple(p) for p in s.get("freq_loss") or ()),
            Xt=s["Xt"], Yt=s["Yt"], Zt=s["Zt"], S=s["S"],
            creep_strain_allow=s["creep_strain_allow"],
            fatigue_strain_amp_allow=s["fatigue_strain_amp_allow"],
            fatigue_stress_ratio=s["fatigue_stress_ratio"], is_elastomer=s["is_elastomer"],
            hdt_C=float(hdt), allow_source="brand" if sourced else "class default",
            source=f"{e['brand']} {e['product']} — {e.get('tds_url') or e['url']} "
                   f"(confidence {e['confidence']}; derived fields: "
                   f"{sum(not v.startswith('tds:') for v in e['derived'].values())})")
        price = e.get("price") or {}
        META[e["name"]] = dict(
            brand=e["brand"], product=e["product"], cls=e["class"],
            group=CLASSES[e["class"]]["group"], url=e["url"], tds_url=e.get("tds_url"),
            price=price.get("value"), currency=price.get("currency"),
            per_kg=price.get("per_kg", True), usd_per_kg=price.get("usd_per_kg"),
            price_date=price.get("date"), confidence=e["confidence"],
            variant_of=e.get("variant_of"), shore=(e.get("tds") or {}).get("shore_hardness"),
            hardened_nozzle=(e.get("print") or {}).get("hardened_nozzle_required"),
            ams=(e.get("print") or {}).get("ams_compatible"),
            moisture=(e.get("print") or {}).get("moisture_sensitivity"))


_register_filaments()

# generic entries: class/group for grouping in the GUI and class filters
_GENERIC_CLASS = {"tpu_95a": "tpu", "tpu_64d": "tpu", "pa6_cf": "pa_cf", "petg_cf": "petg_cf",
                  "pla": "pla", "cf_rod_pultruded": "cf_rod", "steel_302_spring": "steel_wire",
                  "steel_1095_shim": "steel_shim", "fr4_g10": "fr4"}


def material_class(name: str) -> str | None:
    base = name.split("@", 1)[0]
    if base in META:
        return META[base]["cls"]
    return _GENERIC_CLASS.get(base)


def _variant(name: str) -> Material:
    """'<base>@E=0.8,S=1.2,rho=1.0,eta=0.05' → base material with moduli ×E, strengths ×S,
    density ×rho and (optional) loss factor set to eta (datasheet-uncertainty sensitivity, or a
    laminate's composite loss factor, e.g. a constrained-layer damped shim; works across worker
    processes)."""
    from dataclasses import replace
    base, _, spec = name.partition("@")
    m = LIBRARY[base]
    f = {"E": 1.0, "S": 1.0, "rho": 1.0, "eta": None}
    for part in spec.split(","):
        k, _, v = part.partition("=")
        if k.strip() not in f:
            raise KeyError(f"bad material variant key {k!r} in {name!r} (use E, S, rho, eta)")
        f[k.strip()] = float(v)
    e, s = f["E"], f["S"]
    v = replace(m, name=name, rho=m.rho * f["rho"], E1=m.E1 * e, E2=m.E2 * e, E3=m.E3 * e,
                G12=m.G12 * e, G13=m.G13 * e, G23=m.G23 * e, Xt=m.Xt * s, Yt=m.Yt * s,
                Zt=m.Zt * s, S=m.S * s, source=m.source + f" [variant {spec}]")
    if f["eta"] is not None:
        v = replace(v, loss_factor=f["eta"], freq_loss=())
    LIBRARY[name] = v
    return v


_PART_T = [T_REF_C]     # part temperature (C) applied by get_material; see part_temperature
_DERATED: dict = {}


class part_temperature:
    """Context manager: get_material() returns temperature-derated materials inside."""
    def __init__(self, T_C: float):
        self.T = float(T_C)

    def __enter__(self):
        self.prev = _PART_T[0]
        _PART_T[0] = self.T

    def __exit__(self, *a):
        _PART_T[0] = self.prev


def _get_raw(name: str) -> Material:
    try:
        return LIBRARY[name]
    except KeyError:
        if "@" in name:
            try:
                return _variant(name)
            except KeyError:
                pass
        raise KeyError(f"unknown material {name!r}; known: {sorted(LIBRARY)}") from None


def get_material(name: str) -> Material:
    m = _get_raw(name)
    T = _PART_T[0]
    if T <= T_REF_C or m.hdt_C <= T_REF_C:
        return m
    key = (id(m), round(T, 3))
    d = _DERATED.get(key)
    if d is None:
        d = _DERATED[key] = (m, m.derated(T))
    return d[1]


def flexure_indices(m: Material) -> dict:
    """Material indices for a strain-limited flexure (no simulation):
    fatigue_strain = σ_f/E (allowable 1/rev strain amplitude at 1e7 cycles),
    creep_strain   = sustained-strain allowable,
    fatigue_strain_per_rho = σ_f/E per (g/cm³) (strain capability per unit mass),
    specific_stiffness = E1/ρ (blade, MPa·cm³/g), specific_strength = Xt/ρ (tension member)."""
    eps_f = m.fatigue_strain_amp_allow
    return dict(fatigue_strain=eps_f, creep_strain=m.creep_strain_allow,
                fatigue_strain_per_rho=eps_f / (m.rho / 1000.0),
                specific_stiffness=m.E1 / 1e6 / (m.rho / 1000.0),
                specific_strength=m.Xt / 1e6 / (m.rho / 1000.0))


def list_materials() -> list[dict]:
    """All registered materials. Real products carry brand/product/url/price/confidence;
    `group` and `cls` are set for every entry (GUI grouping, class filters)."""
    from .filament_db import CLASSES
    out = []
    for m in LIBRARY.values():
        if "@" in m.name:
            continue
        d = dict(name=m.name, rho=m.rho, E1=m.E1, E2=m.E2, E3=m.E3, Xt=m.Xt, Zt=m.Zt,
                 loss_factor=m.loss_factor, elastomer=m.is_elastomer, source=m.source,
                 **flexure_indices(m))
        meta = META.get(m.name)
        if meta:
            d.update(meta)
        else:
            cls = _GENERIC_CLASS.get(m.name)
            d.update(brand="generic", product=m.name, cls=cls,
                     group=CLASSES[cls]["group"] if cls in CLASSES else "Generic",
                     url=None, tds_url=None, price=None, currency=None, usd_per_kg=None,
                     confidence="low")
        out.append(d)
    return out


def select_materials(spec: str | list[str]) -> list[str]:
    """Material selector: 'all', 'filaments' (all real products), 'generic',
    'class:tpu' / 'class:pa_cf+pa_gf', 'group:Nylon (PA)', 'brand:Bambu Lab',
    'rigid' (all non-elastomer printable), 'inserts', or a comma list of names.
    Conditioned (`_cond`) variants are included only when named or with ':cond' suffix
    on the selector (e.g. 'class:pa_cf:cond')."""
    from .filament_db import INSERT_CLASSES
    if isinstance(spec, list):
        return [get_material(n).name for n in spec]
    spec = spec.strip()
    with_cond = spec.endswith(":cond")
    if with_cond:
        spec = spec[: -len(":cond")]
    allm = [m for m in list_materials()
            if with_cond or not (m.get("variant_of"))]
    if spec == "all":
        sel = allm
    elif spec == "filaments":
        sel = [m for m in allm if m["name"] in META]
    elif spec == "generic":
        sel = [m for m in allm if m["name"] not in META]
    elif spec == "rigid":
        sel = [m for m in allm if not m["elastomer"] and m.get("cls") not in INSERT_CLASSES]
    elif spec == "inserts":
        sel = [m for m in allm if m.get("cls") in INSERT_CLASSES]
    elif spec.startswith("class:"):
        cl = set(spec[6:].split("+"))
        sel = [m for m in allm if m.get("cls") in cl]
    elif spec.startswith("group:"):
        sel = [m for m in allm if m.get("group") == spec[6:]]
    elif spec.startswith("brand:"):
        sel = [m for m in allm if (m.get("brand") or "").lower() == spec[6:].lower()]
    else:
        return [get_material(n.strip()).name for n in spec.split(",") if n.strip()]
    return [m["name"] for m in sel]
