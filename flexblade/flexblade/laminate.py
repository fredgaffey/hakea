"""Laminated (unbonded) shim-stack strips and their interlaminar friction damping (idea I28).

A strip made of n unbonded laminae of thickness t, clamped together at both ends and free to
slide between the clamps, bends as n independent plates:

    EI_thin  = n * E w t^3 / 12            (bonded: E w (n t)^3 / 12 = n^2 x more)
    EA, EI_inplane, GA, GJ, mass  ~ n * (single lamina)   (GJ: n * beta w t^3 G, unbonded)
    surface strain per lamina  = t * kappa / 2           (bonded stack: n t kappa / 2)

so for the same pivot stiffness the lamina strain is lower (t_lam = t_single / n^(1/3) at equal
w) while CF / in-plane / buckling capacity scales with n t.

Friction damping (conservative: clamp-end slip only, no mid-span contact pressure): near each
jaw exit an edge zone of length a carries a normal force N (the decaying tail of the clamp
pressure). Adjacent laminae slip there by s(x) = t * theta(x) ~ t * kappa0 * x, mean
s = t kappa0 a / 2. Full-slip Coulomb loop energy per interface per end per cycle is 4 mu N s
(valid while mu N << the shear needed to lock the laminae, ~E kappa t^2 w, i.e. always here).
Harmonic-balance equivalent viscous ratio: zeta_eq = E_d / (2 pi * 2 U_max) with U_max the peak
strip strain energy, taken conservatively as a uniform curvature kappa0 over the free length
(true S-bend energy is <= this). zeta_eq ~ 1/kappa0 (Coulomb), so it is evaluated at the
operating 1P amplitude; eta = 2 zeta_eq is added to the strip elements' complex stiffness.

Sources: Henein, Spanoudakis et al., "Flexure pivot for aerospace mechanisms" / Cosandier,
Henein et al. "The art of flexure mechanism design" (EPFL Press 2017) -- laminated (multi-blade)
leaf springs to cut strain at equal stiffness; Smith, "Flexures: Elements of Elastic
Mechanisms" (2000) leaf stacks; Ai, Wang et al. / Fancher & Ervin "Leaf-spring hysteresis"
(UMTRI, SAE 1980s) and Cebon (1986) interleaf friction models of multi-leaf springs;
Den Hartog (1931) / Caughey: equivalent viscous damping of Coulomb friction, zeta = E_d/(4 pi U).
"""
from __future__ import annotations

import copy
import math

MU_DEFAULT = 0.15          # steel/steel lubricated-ish (oily shim stock), conservative low
EDGE_FORCE_DEFAULT = 2.0   # N, normal force in the jaw-exit edge zone per end (decaying tail)
EDGE_LEN_DEFAULT = 0.5e-3  # m, edge-zone length (~ jaw radius scale)


def laminate_section(sp, n: int):
    """SectionProps of n unbonded laminae from the single-lamina section `sp`: every stiffness
    and mass term x n; the fibres stay at the lamina surface (strain = t kappa / 2)."""
    if n <= 1:
        return sp
    o = copy.copy(sp)
    for a in ("EA", "EIy", "EIz", "EIyz", "GJ", "GAy", "GAz",
              "mass_per_len", "Izz_mass", "Iyz_mass", "Iyy_mass"):
        if hasattr(o, a):
            setattr(o, a, getattr(sp, a) * n)
    return o


def bending_ratio_unbonded(n: int) -> float:
    """EI_unbonded / EI_bonded for n equal laminae = 1 / n^2."""
    return 1.0 / (n * n)


def lamina_strain(kappa: float, t_lam: float) -> float:
    return 0.5 * t_lam * kappa


def friction_energy(n: int, t: float, kappa0: float, mu: float = MU_DEFAULT,
                    N_edge: float = EDGE_FORCE_DEFAULT, a_edge: float = EDGE_LEN_DEFAULT,
                    ends: int = 2) -> float:
    """Coulomb energy per cycle [J] of one stack: (n-1) interfaces x ends x 4 mu N s_mean."""
    s = 0.5 * t * kappa0 * a_edge
    return (n - 1) * ends * 4.0 * mu * N_edge * s


def strain_energy(n: int, E: float, w: float, t: float, L: float, kappa0: float) -> float:
    """Peak bending strain energy [J] of one stack, uniform curvature kappa0 (upper bound)."""
    return 0.5 * n * E * w * t**3 / 12.0 * kappa0**2 * L


def zeta_eq(E_d: float, U_max: float) -> float:
    """Harmonic-balance equivalent viscous damping ratio E_d / (2 pi * 2 U_max)."""
    return E_d / (2.0 * math.pi * 2.0 * U_max) if U_max > 0 else 0.0


def friction_eta(n: int, E: float, w: float, t: float, L: float, kappa0: float,
                 mu: float = MU_DEFAULT, N_edge: float = EDGE_FORCE_DEFAULT,
                 a_edge: float = EDGE_LEN_DEFAULT) -> float:
    """Equivalent hysteretic loss factor eta = 2 zeta_eq at the 1P clamp curvature kappa0."""
    if n <= 1 or kappa0 <= 0:
        return 0.0
    return 2.0 * zeta_eq(friction_energy(n, t, kappa0, mu, N_edge, a_edge),
                         strain_energy(n, E, w, t, L, kappa0))
