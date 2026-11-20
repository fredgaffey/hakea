"""Material library with print anisotropy and frequency-dependent complex modulus.

Values are representative datasheet / literature numbers for FDM parts and are
*rough*: they exist so trends and feasibility come out right. Every material has
`source` notes; refine with coupon tests (DMA at ~1/rev frequency for TPU).

Orthotropic convention: axis 1 = raster/fibre direction, 2 = in-layer transverse,
3 = build (layer stacking) direction. Interlayer properties = 3-direction.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .schema import PrintOrientation


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
}


def get_material(name: str) -> Material:
    try:
        return LIBRARY[name]
    except KeyError:
        raise KeyError(f"unknown material {name!r}; known: {sorted(LIBRARY)}") from None


def list_materials() -> list[dict]:
    out = []
    for m in LIBRARY.values():
        out.append(dict(name=m.name, rho=m.rho, E1=m.E1, E2=m.E2, E3=m.E3,
                        loss_factor=m.loss_factor, elastomer=m.is_elastomer, source=m.source))
    return out
