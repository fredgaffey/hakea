"""3D rotating-frame beam finite elements (OWNER: structures).

Formulation
-----------
* 2-node, 12-DOF beam, node DOFs [ux, uy, uz, rx, ry, rz] in hub frame H.
* Linear element stiffness from the *exact flexibility* of a prismatic/tapered beam
  with a full 6×6 section stiffness (section.SectionProps.stiffness6): shear
  deformation (Timoshenko), tension-centre and shear-centre offsets from the node line,
  EIyz coupling. Arbitrary orientation via the element frame E0 = [e1 axis, e2, e3].
* Geometric nonlinearity: co-rotational (CR). Local deformations
  d7 = [ū, θ̄a, θ̄b] from current node positions/rotations; ū includes the second-order
  bowing stretch ½∫(v'²+w'²) of the Hermite local deflection and the Wagner/trapeze
  term ½k²∫θ1'² (k² = (EIy+EIz)/EA), so the initial-stress stiffness is the
  consistent one (cubic-Hermite Kσ + trapeze torsion) and *derived from a potential*.
* Rotating frame (constant Ω about +z): the centrifugal potential
  V = −½Ω² Σ m |P r|² (P = diag(1,1,0)) is summed over mass points (4 Gauss points
  along the element × 4 section points reproducing m, mass centre and second mass
  moments). Point positions r(q) use the same CR kinematics (chord + Hermite bowing +
  rotated section offset), so the centrifugal load, spin softening and the propeller
  (offset-mass) moment are mutually consistent and a rigid rotation about the spin
  axis has exactly zero stiffness at equilibrium.
* All derivatives are central finite differences w.r.t. node perturbations
  δ = [Δx_a, ω_a, Δx_b, ω_b] with rotations applied as R ← exp(ω)R (numba):
      B = ∂d7/∂δ,  f_int = Bᵀ K7 d7,  K_mat = Bᵀ K7 B,
      K_geo = ∂²(f7·d7)/∂δ² + ∂²V/∂δ²   (initial stress + centrifugal),
      J_p = ∂r_p/∂δ → M = Σ m JᵀJ, G = Σ 2Ω m Jᵀ[ẑ×]J (Coriolis), F_cf = −∂V/∂δ.
  K_mat is kept per element so it can be scaled by material.complex_scale(f).

Equation of motion (rotating frame, perturbation about a static state):
    M ü + G u̇ + (K_mat* + K_geo) u = F
Zero-length springs (ideal hinges) use the same machinery with
d = [0, Fᵀ(x_b − x_a), log(Fᵀ R_b E0)], F = R_a E0.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np
from numba import njit
from scipy.linalg import lu_factor as sl_lu_factor, lu_solve as sl_lu_solve

from .section import SectionProps

# --------------------------------------------------------------------------- small math


def skew(v) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


@njit(cache=True)
def _expm(w):
    th2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2]
    if th2 < 1e-6:
        a = 1.0 - th2 / 6.0 + th2 * th2 / 120.0
        b = 0.5 - th2 / 24.0 + th2 * th2 / 720.0
    else:
        th = np.sqrt(th2)
        a = np.sin(th) / th
        b = (1.0 - np.cos(th)) / th2
    K = np.zeros((3, 3))
    K[0, 1] = -w[2]
    K[0, 2] = w[1]
    K[1, 0] = w[2]
    K[1, 2] = -w[0]
    K[2, 0] = -w[1]
    K[2, 1] = w[0]
    return np.eye(3) + a * K + b * (K @ K)


@njit(cache=True)
def _logm(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    n = np.sqrt(x * x + y * y + z * z)
    if n < 1e-7:
        f = 2.0 / w * (1.0 - n * n / (3.0 * w * w))
    else:
        f = 2.0 * np.arctan2(n, w) / n
    out = np.empty(3)
    out[0] = f * x
    out[1] = f * y
    out[2] = f * z
    return out


@njit(cache=True)
def _cross(a, b):
    out = np.empty(3)
    out[0] = a[1] * b[2] - a[2] * b[1]
    out[1] = a[2] * b[0] - a[0] * b[2]
    out[2] = a[0] * b[1] - a[1] * b[0]
    return out


@njit(cache=True)
def _cr_frame(xa, xb, Ra, Rb, E0):
    d = xb - xa
    l = np.sqrt(d @ d)
    r1 = d / l
    e2 = E0[:, 1].copy()
    q = 0.5 * (Ra @ e2 + Rb @ e2)
    r3 = _cross(r1, q)
    r3 = r3 / np.sqrt(r3 @ r3)
    r2 = _cross(r3, r1)
    Rr = np.empty((3, 3))
    Rr[:, 0] = r1
    Rr[:, 1] = r2
    Rr[:, 2] = r3
    return l, Rr


@njit(cache=True)
def _eval(xa, xb, Ra, Rb, E0, L0, k2, mpts, d7, pos):
    """Fill d7 (7) and mass-point positions pos (np,3) for one configuration."""
    if L0 <= 0.0:
        Fa = Ra @ E0
        d7[0] = 0.0
        d7[1:4] = Fa.T @ (xb - xa)
        d7[4:7] = _logm(Fa.T @ Rb @ E0)
        return
    l, Rr = _cr_frame(xa, xb, Ra, Rb, E0)
    ta = _logm(Rr.T @ Ra @ E0)
    tb = _logm(Rr.T @ Rb @ E0)
    bow = l * (2 * ta[2] ** 2 - ta[2] * tb[2] + 2 * tb[2] ** 2
               + 2 * ta[1] ** 2 - ta[1] * tb[1] + 2 * tb[1] ** 2) / 30.0
    d7[0] = l - L0 + bow + 0.5 * k2 * (tb[0] - ta[0]) ** 2 / l
    d7[1:4] = ta
    d7[4:7] = tb
    sl_ = np.zeros(3)
    loc = np.empty(3)
    ph = np.empty(3)
    for p in range(mpts.shape[0]):
        xi = mpts[p, 0]
        H2 = xi - 2 * xi**2 + xi**3
        H4 = -xi**2 + xi**3
        d2 = 1 - 4 * xi + 3 * xi**2
        d4 = -2 * xi + 3 * xi**2
        ph[0] = (1 - xi) * ta[0] + xi * tb[0]
        ph[1] = d2 * ta[1] + d4 * tb[1]
        ph[2] = d2 * ta[2] + d4 * tb[2]
        sl_[1] = mpts[p, 1]
        sl_[2] = mpts[p, 2]
        off = _expm(ph) @ sl_
        loc[0] = xi * l + off[0]
        loc[1] = l * (H2 * ta[2] + H4 * tb[2]) + off[1]
        loc[2] = -l * (H2 * ta[1] + H4 * tb[1]) + off[2]
        pos[p] = xa + Rr @ loc


@njit(cache=True)
def _vcf(pos, pos0, mpts, W2):
    """Centrifugal potential relative to the base configuration (cancellation-safe)."""
    v = 0.0
    for p in range(pos.shape[0]):
        a = (pos[p, 0] - pos0[p, 0]) * (pos[p, 0] + pos0[p, 0]) \
            + (pos[p, 1] - pos0[p, 1]) * (pos[p, 1] + pos0[p, 1])
        v += -0.5 * W2 * mpts[p, 3] * a
    return v


@njit(cache=True)
def _pert(xa, xb, Ra, Rb, dl):
    return (xa + dl[0:3], xb + dl[6:9], _expm(dl[3:6]) @ Ra, _expm(dl[9:12]) @ Rb)


@njit(cache=True)
def _element_derivs(xa, xb, Ra, Rb, E0, L0, k2, K7, mpts, W2, want_h, fw):
    """Returns d7, f7, B (7×12), gradV (12), J (np,3,12), pos0 (np,3), Kgeo (12×12)."""
    npt = mpts.shape[0]
    h = np.empty(12)
    ht = 1e-4 * L0 if L0 > 0.0 else 1e-7
    for i in range(12):
        h[i] = ht if (i % 6) < 3 else 1e-4
    d0 = np.empty(7)
    pos0 = np.empty((npt, 3))
    _eval(xa, xb, Ra, Rb, E0, L0, k2, mpts, d0, pos0)
    f7 = K7 @ d0
    fH = f7 if fw.shape[0] != 7 else fw     # Hessian weights (initial-stress predictor)
    dp = np.empty((12, 7))
    dm = np.empty((12, 7))
    Vp = np.empty(12)
    Vm = np.empty(12)
    J = np.zeros((npt, 3, 12))
    B = np.empty((7, 12))
    gV = np.empty(12)
    tmpd = np.empty(7)
    tmpp = np.empty((npt, 3))
    tmpp2 = np.empty((npt, 3))
    for i in range(12):
        dl = np.zeros(12)
        dl[i] = h[i]
        a, b, c, d = _pert(xa, xb, Ra, Rb, dl)
        _eval(a, b, c, d, E0, L0, k2, mpts, tmpd, tmpp)
        dp[i] = tmpd
        Vp[i] = _vcf(tmpp, pos0, mpts, W2)
        dl[i] = -h[i]
        a, b, c, d = _pert(xa, xb, Ra, Rb, dl)
        _eval(a, b, c, d, E0, L0, k2, mpts, tmpd, tmpp2)
        dm[i] = tmpd
        Vm[i] = _vcf(tmpp2, pos0, mpts, W2)
        B[:, i] = (dp[i] - dm[i]) / (2.0 * h[i])
        gV[i] = (Vp[i] - Vm[i]) / (2.0 * h[i])
        for p in range(npt):
            for k in range(3):
                J[p, k, i] = (tmpp[p, k] - tmpp2[p, k]) / (2.0 * h[i])
    Kr = np.zeros((12, 12))
    if want_h:
        phi0 = fH @ d0
        for i in range(12):
            Kr[i, i] = ((fH @ dp[i] - 2.0 * phi0 + fH @ dm[i]) + (Vp[i] + Vm[i])) / (h[i] * h[i])
            for j in range(i + 1, 12):
                dl = np.zeros(12)
                dl[i] = h[i]
                dl[j] = h[j]
                a, b, c, d = _pert(xa, xb, Ra, Rb, dl)
                _eval(a, b, c, d, E0, L0, k2, mpts, tmpd, tmpp)
                s = fH @ tmpd + _vcf(tmpp, pos0, mpts, W2)
                dl[j] = -h[j]
                a, b, c, d = _pert(xa, xb, Ra, Rb, dl)
                _eval(a, b, c, d, E0, L0, k2, mpts, tmpd, tmpp)
                s -= fH @ tmpd + _vcf(tmpp, pos0, mpts, W2)
                dl[i] = -h[i]
                a, b, c, d = _pert(xa, xb, Ra, Rb, dl)
                _eval(a, b, c, d, E0, L0, k2, mpts, tmpd, tmpp)
                s += fH @ tmpd + _vcf(tmpp, pos0, mpts, W2)
                dl[j] = h[j]
                a, b, c, d = _pert(xa, xb, Ra, Rb, dl)
                _eval(a, b, c, d, E0, L0, k2, mpts, tmpd, tmpp)
                s -= fH @ tmpd + _vcf(tmpp, pos0, mpts, W2)
                v = s / (4.0 * h[i] * h[j])
                Kr[i, j] = v
                Kr[j, i] = v
    return d0, f7, B, gV, J, pos0, Kr


# --------------------------------------------------------------------------- shape functions / quadrature


def hermite_maps(xi: float, l: float) -> tuple[np.ndarray, np.ndarray]:
    """Local (element-frame) translation map Nt (3×12) and rotation map Nr (3×12)."""
    H1, H2, H3, H4 = 1 - 3 * xi**2 + 2 * xi**3, xi - 2 * xi**2 + xi**3, 3 * xi**2 - 2 * xi**3, -xi**2 + xi**3
    d1, d2, d3, d4 = -6 * xi + 6 * xi**2, 1 - 4 * xi + 3 * xi**2, 6 * xi - 6 * xi**2, -2 * xi + 3 * xi**2
    Nt = np.zeros((3, 12))
    Nt[0, 0], Nt[0, 6] = 1 - xi, xi
    Nt[1, 1], Nt[1, 5], Nt[1, 7], Nt[1, 11] = H1, H2 * l, H3, H4 * l
    Nt[2, 2], Nt[2, 4], Nt[2, 8], Nt[2, 10] = H1, -H2 * l, H3, -H4 * l
    Nr = np.zeros((3, 12))
    Nr[0, 3], Nr[0, 9] = 1 - xi, xi
    Nr[1, 2], Nr[1, 4], Nr[1, 8], Nr[1, 10] = -d1 / l, d2, -d3 / l, d4
    Nr[2, 1], Nr[2, 5], Nr[2, 7], Nr[2, 11] = d1 / l, d2, d3 / l, d4
    return Nt, Nr


GAUSS3 = (np.array([0.5 - np.sqrt(0.15), 0.5, 0.5 + np.sqrt(0.15)]), np.array([5, 8, 5]) / 18.0)
_g4x = np.array([-0.8611363115940526, -0.3399810435848563, 0.3399810435848563, 0.8611363115940526])
_g4w = np.array([0.3478548451374538, 0.6521451548625461, 0.6521451548625461, 0.3478548451374538])
GAUSS4 = (0.5 * (_g4x + 1), 0.5 * _g4w)
IDX7 = np.array([6, 3, 4, 5, 9, 10, 11])


def local_stiffness(S_list: list[np.ndarray], L: float) -> np.ndarray:
    """12×12 local stiffness from section stiffness at the 3 Gauss points (exact flexibility)."""
    xs, ws = GAUSS3
    F = np.zeros((6, 6))
    for xi, w, S in zip(xs, ws, S_list):
        x = xi * L
        P = np.eye(6)
        P[4, 2] = -(L - x)
        P[5, 1] = (L - x)
        F += w * L * P.T @ np.linalg.inv(S) @ P
    Kbb = np.linalg.inv(F)
    Kbb = 0.5 * (Kbb + Kbb.T)
    A = np.eye(6)
    A[3:6, 0:3] = L * skew([1.0, 0.0, 0.0])
    Bm = np.hstack([-A.T, np.eye(6)])
    return Bm.T @ Kbb @ Bm


# --------------------------------------------------------------------------- element


@dataclass
class Element:
    n0: int
    n1: int
    e2: np.ndarray                     # reference e2 direction (H); orthogonalised to the axis
    sections: list[SectionProps]       # 3 (stiffness Gauss pts) — or 1 (uniform)
    mass_sections: Optional[list[SectionProps]] = None   # 4 Gauss pts; None → use sections
    material_fracs: dict = field(default_factory=dict)  # {material: EA fraction} for complex modulus
    tag: str = ""
    kind: str = "beam"                 # blade | shank | flexure | rigid | spring
    blade: int = 0                     # 1-based blade index (0 = hub)
    massless: bool = False
    spring_K: Optional[np.ndarray] = None    # 6×6 [translations; rotations] in frame E0 → zero-length spring
    spring_frame: Optional[np.ndarray] = None  # 3×3 columns (axis a, e2, ·) in H
    # derived
    L0: float = 0.0
    E0: np.ndarray = None
    K12: np.ndarray = None
    K7: np.ndarray = None
    k2: float = 0.0
    mpts: np.ndarray = None            # (n,4): xi, y, z, mass (local section coords)

    def prepare(self, X0: np.ndarray) -> None:
        if self.spring_K is not None:
            self.L0 = 0.0
            F = np.asarray(self.spring_frame, float)
            e1 = unit(F[:, 0])
            e2 = unit(F[:, 1] - e1 * (e1 @ F[:, 1]))
            self.E0 = np.ascontiguousarray(np.column_stack([e1, e2, np.cross(e1, e2)]))
            self.K7 = np.zeros((7, 7))
            self.K7[1:, 1:] = self.spring_K
            self.K12 = np.zeros((12, 12))
            self.mpts = np.zeros((0, 4))
            self.massless = True
            return
        xa, xb = X0[self.n0], X0[self.n1]
        d = xb - xa
        self.L0 = float(np.linalg.norm(d))
        e1 = d / self.L0
        e2 = np.asarray(self.e2, float) - e1 * (e1 @ self.e2)
        if np.linalg.norm(e2) < 1e-9:
            ref = np.array([0.0, 0, 1]) if abs(e1[2]) < 0.9 else np.array([0.0, 1, 0])
            e2 = np.cross(ref, e1)
        e2 = unit(e2)
        e3 = np.cross(e1, e2)
        self.E0 = np.ascontiguousarray(np.column_stack([e1, e2, e3]))
        secs = self.sections if len(self.sections) == 3 else [self.sections[0]] * 3
        self.K12 = local_stiffness([s.stiffness6() for s in secs], self.L0)
        self.K7 = np.ascontiguousarray(self.K12[np.ix_(IDX7, IDX7)])
        s0 = secs[1]
        self.k2 = float((s0.EIy + s0.EIz) / s0.EA) if s0.EA > 0 else 0.0
        pts = []
        if not self.massless:
            ms = self.mass_sections or ([self.sections[0]] * 4 if len(self.sections) == 1 else
                                        [self.sections[0], self.sections[1], self.sections[1], self.sections[2]])
            for xi, w, s in zip(GAUSS4[0], GAUSS4[1], ms):
                m, s1, s2 = s.mass_moments()
                if m <= 0:
                    continue
                c = s1 / m
                cov = s2 / m - np.outer(c, c)
                lam, V = np.linalg.eigh(0.5 * (cov + cov.T))
                lam = np.maximum(lam, 0.0)
                for k in range(2):
                    a = np.sqrt(2 * lam[k]) * V[:, k]
                    for sg in (1, -1):
                        pts.append([xi, c[0] + sg * a[0], c[1] + sg * a[1], m * w * self.L0 / 4])
        self.mpts = np.ascontiguousarray(np.array(pts)) if pts else np.zeros((0, 4))

    def complex_scale(self, freq_hz: float) -> complex:
        if not self.material_fracs:
            return 1.0 + 0j
        from .materials import get_material
        return sum(fr * get_material(m).complex_scale(freq_hz) for m, fr in self.material_fracs.items())


# --------------------------------------------------------------------------- model / state


@dataclass
class State:
    x: np.ndarray        # (nn,3) current positions
    R: np.ndarray        # (nn,3,3) nodal rotations

    def copy(self) -> "State":
        return State(self.x.copy(), self.R.copy())


class FEModel:
    def __init__(self) -> None:
        self.X0: list[np.ndarray] = []
        self.elements: list[Element] = []
        self.fixed: set[int] = set()
        self.lumped: list[tuple[int, np.ndarray]] = []   # (node, 6×6 lumped mass; no rotating terms)
        self.node_tags: list[str] = []
        self._prepared = False

    def add_node(self, x, tag: str = "") -> int:
        self.X0.append(np.asarray(x, float))
        self.node_tags.append(tag)
        self._prepared = False
        return len(self.X0) - 1

    def add_element(self, el: Element) -> int:
        self.elements.append(el)
        self._prepared = False
        return len(self.elements) - 1

    def fix(self, node: int, dofs=range(6)) -> None:
        for d in dofs:
            self.fixed.add(6 * node + d)

    def add_lumped_inertia(self, node: int, Mn: np.ndarray) -> None:
        self.lumped.append((node, np.asarray(Mn, float)))

    @property
    def nn(self) -> int:
        return len(self.X0)

    @property
    def ndof(self) -> int:
        return 6 * self.nn

    def prepare(self) -> None:
        X = np.array(self.X0)
        for e in self.elements:
            e.prepare(X)
        self._X = X
        self._prepared = True

    @property
    def Xref(self) -> np.ndarray:
        if not self._prepared:
            self.prepare()
        return self._X

    def free_dofs(self) -> np.ndarray:
        return np.array([i for i in range(self.ndof) if i not in self.fixed], int)

    def initial_state(self) -> State:
        X = self.Xref
        return State(X.copy(), np.repeat(np.eye(3)[None], self.nn, axis=0))


def _edofs(e: Element) -> np.ndarray:
    return np.r_[6 * e.n0:6 * e.n0 + 6, 6 * e.n1:6 * e.n1 + 6]


# --------------------------------------------------------------------------- assembly


@dataclass
class ElementCR:
    d7: np.ndarray
    B: np.ndarray
    f7: np.ndarray
    J: np.ndarray       # (np,3,12) mass-point Jacobians
    pos: np.ndarray     # (np,3)
    gV: np.ndarray      # gradient of centrifugal potential (12)
    Kr: np.ndarray      # initial-stress + centrifugal Hessian (12×12)


_NOW = np.zeros(0)


def element_cr(e: Element, st: State, Omega: float = 0.0, want_h: bool = True, fw=None) -> ElementCR:
    d7, f7, B, gV, J, pos, Kr = _element_derivs(st.x[e.n0], st.x[e.n1], st.R[e.n0], st.R[e.n1], e.E0,
                                                e.L0, e.k2, e.K7, e.mpts, float(Omega) ** 2, want_h,
                                                _NOW if fw is None else np.asarray(fw, float))
    return ElementCR(d7, B, f7, J, pos, gV, Kr)


_ZX = skew([0.0, 0.0, 1.0])


@dataclass
class Assembly:
    fint: np.ndarray                # Bᵀ f7 assembled
    Fcf: np.ndarray                 # centrifugal generalised load (−∂V/∂q)
    Kmat_e: list                    # per-element 12×12 material tangent (unit scale)
    Kgeo: Optional[np.ndarray]      # initial stress + centrifugal (spin softening, propeller)
    M: Optional[np.ndarray]
    G: Optional[np.ndarray]
    cr: list


def assemble(model: FEModel, st: State, Omega: float = 0.0, want_tangent: bool = True,
             want_mass: bool = False, du_pred: np.ndarray | None = None) -> Assembly:
    """Assemble at state st. du_pred: if given, the initial-stress Hessian is weighted by
    the predicted element forces f7 + K7·B·du_pred (prestress predictor for unstressed
    starts, where spin softening is not yet balanced by tension)."""
    n = model.ndof
    fint = np.zeros(n)
    Fcf = np.zeros(n)
    Kg = np.zeros((n, n)) if want_tangent else None
    M = np.zeros((n, n)) if want_mass else None
    G = np.zeros((n, n)) if want_mass else None
    Kmat_e, crs = [], []
    for e in model.elements:
        dofs = _edofs(e)
        if du_pred is not None and want_tangent:
            c0 = element_cr(e, st, Omega, False)
            c = element_cr(e, st, Omega, True, c0.f7 + e.K7 @ (c0.B @ du_pred[dofs]))
        else:
            c = element_cr(e, st, Omega, want_tangent)
        fint[dofs] += c.B.T @ c.f7
        Fcf[dofs] -= c.gV
        if want_tangent:
            ix = np.ix_(dofs, dofs)
            Kg[ix] += 0.5 * (c.Kr + c.Kr.T)
            Km = c.B.T @ e.K7 @ c.B
            Kmat_e.append(0.5 * (Km + Km.T))
        if want_mass and len(e.mpts):
            m = e.mpts[:, 3]
            Me = np.einsum("p,pki,pkj->ij", m, c.J, c.J)
            Ge = 2.0 * Omega * np.einsum("p,pki,kl,plj->ij", m, c.J, _ZX, c.J)
            ix = np.ix_(dofs, dofs)
            M[ix] += 0.5 * (Me + Me.T)
            G[ix] += 0.5 * (Ge - Ge.T)
        crs.append(c)
    if want_mass:
        for node, Mn in model.lumped:
            d = np.r_[6 * node:6 * node + 6]
            M[np.ix_(d, d)] += Mn
    return Assembly(fint, Fcf, Kmat_e, Kg, M, G, crs)


def assemble_internal(model: FEModel, st: State, want_tangent: bool = True, Omega: float = 0.0) -> Assembly:
    """Elastic part only (Ω = 0 by default: no centrifugal terms)."""
    return assemble(model, st, Omega, want_tangent, False)


def assemble_Kmat(model: FEModel, sa: Assembly, scales=None) -> np.ndarray:
    n = model.ndof
    dtype = complex if scales is not None and np.iscomplexobj(np.asarray(scales)) else float
    K = np.zeros((n, n), dtype)
    for i, (e, Ke) in enumerate(zip(model.elements, sa.Kmat_e)):
        dofs = _edofs(e)
        K[np.ix_(dofs, dofs)] += Ke * (1.0 if scales is None else scales[i])
    return K


def element_point_maps(e: Element, st: State, xi: float, s_local_yz):
    """Position r (3,), offset s_H, translation Jacobian L (3×12) and rotation map Nr (3×12)
    of a material point (xi, local y, z) — same CR kinematics as the mass points."""
    pts = np.array([[float(xi), float(s_local_yz[0]), float(s_local_yz[1]), 0.0]])
    _, _, _, _, J, pos, _ = _element_derivs(st.x[e.n0], st.x[e.n1], st.R[e.n0], st.R[e.n1], e.E0,
                                            e.L0, e.k2, e.K7, pts, 0.0, False, _NOW)
    xa, xb = st.x[e.n0], st.x[e.n1]
    l, Rr = _cr_frame(xa, xb, st.R[e.n0], st.R[e.n1], e.E0)
    Nt, Nr = hermite_maps(float(xi), l)
    T = np.zeros((12, 12))
    for k in range(4):
        T[3 * k:3 * k + 3, 3 * k:3 * k + 3] = Rr.T
    Nrg = Rr @ Nr @ T
    r = pos[0]
    sH = r - ((1 - xi) * xa + xi * xb)
    return r, sH, J[0], Nrg


def euler_load(model: FEModel, st: State, Omega_dot: float) -> np.ndarray:
    """Generalised load from hub angular acceleration Ω̇ in the frame rotating with the
    hub: −m Ω̇ ẑ × r per mass point (+ lumped hub inertia)."""
    F = np.zeros(model.ndof)
    for e in model.elements:
        if e.massless or len(e.mpts) == 0:
            continue
        c = element_cr(e, st, 0.0, False)
        dofs = _edofs(e)
        for p in range(len(e.mpts)):
            F[dofs] += -Omega_dot * e.mpts[p, 3] * c.J[p].T @ np.cross([0.0, 0.0, 1.0], c.pos[p])
    for node, Mn in model.lumped:
        F[6 * node + 5] += -Omega_dot * Mn[5, 5]
    return F


# --------------------------------------------------------------------------- static solve


def update_state(st: State, du: np.ndarray) -> State:
    out = st.copy()
    d = du.reshape(-1, 6)
    out.x += d[:, :3]
    for i in range(len(out.R)):
        if np.any(d[i, 3:]):
            out.R[i] = _expm(d[i, 3:].copy()) @ out.R[i]
    return out


@dataclass
class NewtonInfo:
    converged: bool
    iterations: int
    residual: float
    message: str = ""


def solve_static(model: FEModel, Omega: float, fext: Callable[[State], np.ndarray] | np.ndarray | None = None,
                 state0: State | None = None, n_steps: int = 4, tol: float = 1e-7, max_iter: int = 30,
                 verbose: bool = False, extra_fixed=(), lam0: float | None = None) -> tuple[State, NewtonInfo]:
    """Nonlinear static equilibrium in the rotating frame: f_int(q) = F_cf(q, Ω) + λF_ext.

    Adaptive continuation in λ = (Ω/Ω₀)² (centrifugal and external loads ramp together;
    first step 1/n_steps from an unloaded state, whole step if state0 is given), Newton
    with the exact FD tangent of the potential, residual backtracking, 0.2 rad cap on
    rotation increments, Levenberg–Marquardt damping after rejected steps. Residual =
    max|r|/max(|F|,|f_int|) on free DOFs; FD derivatives limit it to ~1e-8, so a
    stagnating residual below 1e-5 is accepted (noted in the message)."""
    st = state0.copy() if state0 is not None else model.initial_state()
    xf = set(extra_fixed)
    free = np.array([d for d in model.free_dofs() if d not in xf], int)
    total_it = 0
    msg = ""
    ok_all = True

    def residual(s, Om, lam):
        sa = assemble(model, s, Om, False, False)
        F = sa.Fcf.copy()
        if fext is not None:
            F += lam * (fext(s) if callable(fext) else fext)
        r = (sa.fint - F)[free]
        scale = max(np.abs(F[free]).max(), np.abs(sa.fint[free]).max(), 1e-12)
        return r, np.abs(r).max() / scale

    def newton_at(st, lam, max_it):
        nonlocal total_it
        Om = Omega * np.sqrt(lam)
        r, res = residual(st, Om, lam)
        K = None
        hist = []
        mu = 0.0
        for it in range(max_it):
            if res < tol:
                return st, True, res, ""
            if len(hist) >= 4 and res < 1e-5 and res > 0.5 * hist[-4]:
                return st, True, res, f"stagnated at residual {res:.1e} (FD precision)"
            if K is None or len(hist) < 2 or res > 0.1 * hist[-1]:
                sa = assemble(model, st, Om, True, False)
                Km = assemble_Kmat(model, sa)
                if res > 0.2:
                    # unbalanced state: predict prestress from a linear solve, re-weight Kσ
                    # regularise near-mechanisms (soft hinges) with ~Ω²·diag(M) so the
                    # predicted forces are not polluted by huge rigid-body motions
                    sm = assemble(model, st, Om, False, True)
                    Kp = Km + (Om * Om + 1e-12) * np.diag(np.diag(sm.M))
                    du_l = np.zeros(model.ndof)
                    try:
                        du_l[free] = np.linalg.solve(Kp[np.ix_(free, free)], -r)
                        sa = assemble(model, st, Om, True, False, du_pred=du_l)
                    except np.linalg.LinAlgError:
                        pass
                K = (Km + sa.Kgeo)[np.ix_(free, free)]
                if mu > 0:
                    K = K + mu * np.diag(np.abs(np.diag(Km)[free]) + 1e-12)
                lu = sl_lu_factor(K, check_finite=False)
            dq = sl_lu_solve(lu, -r)
            du = np.zeros(model.ndof)
            du[free] = dq
            rot = np.abs(du.reshape(-1, 6)[:, 3:]).max()
            if not np.isfinite(rot):
                return st, False, res, "non-finite Newton step"
            if rot > 0.2:
                du *= 0.2 / rot
            hist.append(res)
            for _ in range(4):
                st2 = update_state(st, du)
                r2, res2 = residual(st2, Om, lam)
                if res2 < 1.5 * res:
                    break
                du *= 0.5
                K = None
            if res2 >= res:
                mu = max(10 * mu, 1e-4)
                K = None
            elif mu > 0:
                mu = mu * 0.1 if mu > 1e-7 else 0.0
                K = None
            if verbose:
                print(f"  lam {lam:.3f} it {it} res {res:.3e} -> {res2:.3e} |du| {np.abs(du).max():.3e} mu {mu:.0e}")
            st, r, res = st2, r2, res2
            total_it += 1
            if mu > 1e3:
                break
        return st, res < tol, res, ""

    lam = 0.0
    dlam = 1.0 if state0 is not None else (lam0 or 1.0 / max(n_steps, 1))
    n_fail = 0
    res = np.inf
    while lam < 1.0 - 1e-12:
        lt = min(1.0, lam + dlam)
        st_try, ok, res, note = newton_at(st, lt, 15 if lt < 1.0 else max_iter)
        if ok:
            st, lam = st_try, lt
            msg = note or msg
            dlam = min(1.0, dlam * 2.0)
        else:
            n_fail += 1
            dlam *= 0.35
            if dlam < 2e-3 or n_fail > 10:
                st = st_try
                ok_all = False
                msg = f"Newton did not converge at λ=(Ω/Ω₀)²={lt:.3f} (residual {res:.1e})"
                break
    return st, NewtonInfo(ok_all, total_it, float(res), msg)


# --------------------------------------------------------------------------- element recovery


def element_end_forces(e: Element, c: ElementCR) -> tuple[np.ndarray, np.ndarray]:
    """Section forces [N, V2, V3, T, M2, M3] at end a and end b (element frame, CR)."""
    d12 = np.zeros(12)
    d12[IDX7] = c.d7
    f12 = e.K12 @ d12
    return -f12[:6], f12[6:]


def element_end_forces_lin(e: Element, c: ElementCR, du_e: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Linearised (possibly complex) section force increments for element DOF increment du_e."""
    dd7 = c.B @ du_e
    d12 = np.zeros(12, dtype=dd7.dtype)
    d12[IDX7] = dd7
    f12 = e.K12 @ d12
    return -f12[:6], f12[6:]
