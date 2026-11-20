"""Member-end anchorage compliance (flexure member butting onto a printed fitting).

geometry.py ends every flexure member (plate and its CF rod inserts) flush on the face of a
compact clamp block: the PA6-CF hub fitting at the hub end, the blade-material root fitting at
the blade end. The beam model used to join member ends with stiff links, i.e. it treated the
fitting face as rigid. CalculiX (study/campaign/experiments/e08_root_compliance.py) shows the
local bearing of the stiff rod ends on the much softer fitting is the dominant missing
compliance (≈ 70 % of the hub-side gap for 5in_tpu_rod_winner).

Model: each load-carrying region of the member end face (each rod; the matrix plate/bar) is a
rigid flat punch on an elastic half-space (the fitting), in parallel, transferred to the
member reference axis with the parallel-axis rule. Classical punch stiffnesses for a circle of
radius a (Johnson, Contact Mechanics §3.8; Mindlin 1949):
    axial  k_n = 2 a E*                      E* = 1 / ((1-ν1²)/E1 + (1-ν2²)/E2)
    tilt   k_θ = 4/3 a³ E*
    shear  k_t = 8 a G* / (2 - ν)            G* = 1 / (1/G1 + 1/G2)
    twist  k_φ = 16/3 a³ G*
Non-circular regions (rectangular plate) use the equal-area radius for k_n, k_t and scale the
tilt/twist terms with the region's own second moments (k_θ = k_n · 8/3 · I/A, exact for a
circle). Moduli: each body's modulus along the member axis from its print orientation.
No calibration constants; the elastic half-space ignores the finite fitting depth and the bulk
bending of the fitting (the remaining ≈ 30 % of the ccx gap), so it is a lower bound on the
real end compliance.
"""
from __future__ import annotations

import numpy as np

from .materials import get_material
from .schema import FlexureMember, PrintOrientation


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], float)


def _region_K(a_eq: float, Iy: float, Iz: float, A: float, Es: float, Gs: float, nu: float,
              y: float, z: float) -> np.ndarray:
    """6×6 [u1 u2 u3 θ1 θ2 θ3] punch stiffness of one region centred at (0, y, z) in the
    member frame (e1 axis), transferred to the member reference point (0, 0, 0)."""
    kn = 2.0 * a_eq * Es
    kt = 8.0 * a_eq * Gs / (2.0 - nu)
    J = Iy + Iz
    Kp = np.diag([kn, kt, kt,
                  16.0 / 3.0 * a_eq ** 3 * Gs * J / (0.5 * np.pi * a_eq ** 4),
                  kn * 8.0 / 3.0 * Iy / A, kn * 8.0 / 3.0 * Iz / A])
    r = np.array([0.0, y, z])
    T = np.eye(6)
    T[:3, 3:] = -_skew(r)                 # u_point = u + θ × r
    return T.T @ Kp @ T


def _star(m1, o1, m2, o2, t):
    E1, E2 = m1.axial_modulus(o1, t), m2.axial_modulus(o2, t)
    nu1, nu2 = m1.nu12, m2.nu12
    Es = 1.0 / ((1 - nu1 ** 2) / E1 + (1 - nu2 ** 2) / E2)
    Gs = 1.0 / (2 * (1 + nu1) / E1 + 2 * (1 + nu2) / E2)
    return Es, Gs, nu2


def end_spring(m: FlexureMember, member_orient: PrintOrientation | None, e1, e2, e3,
               fitting_material: str, fitting_orient: PrintOrientation | None) -> np.ndarray | None:
    """6×6 anchorage stiffness of member `m` on a fitting, in the member frame (e1, e2, e3),
    about the member end-face centre. None for rigid members."""
    if m.kind == "rigid":
        return None
    from .section import rod_orientation
    fit = get_material(fitting_material)
    lay = m.layup
    mat = get_material(lay.material)
    t = m.thickness
    K = np.zeros((6, 6))
    if m.shape == "circle":
        a = 0.5 * t
        A, I = np.pi * a * a, np.pi * a ** 4 / 4
        Es, Gs, nu = _star(mat, member_orient, fit, fitting_orient, e1)
        return _region_K(a, I, I, A, Es, Gs, nu, 0.0, 0.0)
    w = m.width
    A, Iy, Iz = w * t, w * t ** 3 / 12, t * w ** 3 / 12      # Iy: about e2 (thickness bending)
    Es, Gs, nu = _star(mat, member_orient, fit, fitting_orient, e1)
    K += _region_K(np.sqrt(A / np.pi), Iy, Iz, A, Es, Gs, nu, 0.0, 0.0)
    if lay.kind == "rod_insert" and lay.insert_diameter > 0:
        rod = get_material(lay.insert_material or "cf_rod_pultruded")
        a = 0.5 * lay.insert_diameter
        Ar, Ir = np.pi * a * a, np.pi * a ** 4 / 4
        Es, Gs, nu = _star(rod, rod_orientation(e1), fit, fitting_orient, e1)
        for pos in (lay.insert_positions or [0.5]):
            K += _region_K(a, Ir, Ir, Ar, Es, Gs, nu, (pos - 0.5) * w, 0.0)
    return 0.5 * (K + K.T)
