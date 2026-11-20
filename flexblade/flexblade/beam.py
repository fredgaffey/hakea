"""3D rotating-frame beam finite elements (OWNER: structures).

Formulation
-----------
* 2-node, 12-DOF beam, node DOFs [ux, uy, uz, rx, ry, rz] in hub frame H.
* Linear element stiffness from the *exact flexibility* of a prismatic/tapered beam
  with a full 6×6 section stiffness (section.SectionProps.stiffness6): includes
  shear deformation (Timoshenko), tension-centre and shear-centre offsets from the
  node line, and bending–bending (EIyz) coupling. Arbitrary orientation via the
  element frame E0 = [e1 axis, e2, e3].
* Geometric nonlinearity: co-rotational (CR) element. Local deformations
  d7 = [ū, θ̄a, θ̄b] are computed from the current node positions and rotation
  matrices; internal force f = Bᵀ K7 d7 and tangent K = Bᵀ K7 B + Σ f7_k H_k, with
  B and H (7×12, 7×12×12) obtained by central finite differences of d7 with respect
  to node perturbations δ = [Δx_a, ω_a, Δx_b, ω_b] (rotations applied as
  R ← exp(ω) R). The geometric part Σ f7_k H_k is the *exact* (discretised) initial
  stress stiffness incl. axial force, torque and end moments — it is what makes a
  rigid rotation about the spin axis stiffness-free in the rotating frame.
* Rotating frame (constant Ω about +z): all inertial terms are obtained by numerical
  quadrature of the element mass distribution. Along the element 4 Gauss points;
  in each cross-section 4 mass points reproducing m, first and second mass moments
  (incl. mass-centre offset). For each mass point with linear kinematic map
  L (3×12) and rotation map Nr (3×12):
      M   += m Lᵀ L
      G   += m Lᵀ (2Ω[ẑ×]) L                      (Coriolis / gyroscopic, skew)
      Kcf += −Ω² m (Lᵀ P L + Nrᵀ Q Nr)           (spin softening + offset-mass
                                                  centrifugal moment, P = diag(1,1,0),
                                                  Q = sym(s cᵀ) − (s·c) I, c = P r)
      Fcf += Ω² m Lᵀ P r                         (centrifugal load)
      Feul += −Ω̇ m Lᵀ (ẑ × r)                    (Euler load from hub acceleration)
  Kcf's Q-term is the propeller ("tennis racket") moment of chordwise/thickness
  mass offsets; together with the CR geometric stiffness this reproduces e.g.
  the classical flap ν² = 1 + 3e/2(1-e) and pitch propeller stiffness.

Equation of motion (rotating frame, perturbation about a static state):
    M ü + (G + C) u̇ + (K_mat* + K_geo + K_cf) u = F
K_mat may be multiplied per element by material.complex_scale(f) (viscoelastic).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np
from numba import njit

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
    R = np.eye(3) + a * K + b * (K @ K)
    return R


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
    q = 0.5 * (Ra @ E0[:, 1] + Rb @ E0[:, 1])
    r3 = _cross(r1, q)
    r3 = r3 / np.sqrt(r3 @ r3)
    r2 = _cross(r3, r1)
    Rr = np.empty((3, 3))
    Rr[:, 0] = r1
    Rr[:, 1] = r2
    Rr[:, 2] = r3
    return l, Rr


@njit(cache=True)
def _d7(xa, xb, Ra, Rb, E0, L0):
    l, Rr = _cr_frame(xa, xb, Ra, Rb, E0)
    out = np.empty(7)
    out[0] = l - L0
    out[1:4] = _logm(Rr.T @ Ra @ E0)
    out[4:7] = _logm(Rr.T @ Rb @ E0)
    return out


@njit(cache=True)
def _d7_pert(xa, xb, Ra, Rb, E0, L0, dl):
    xa2 = xa + dl[0:3]
    xb2 = xb + dl[6:9]
    Ra2 = _expm(dl[3:6]) @ Ra
    Rb2 = _expm(dl[9:12]) @ Rb
    return _d7(xa2, xb2, Ra2, Rb2, E0, L0)


@njit(cache=True)
def _cr_derivs(xa, xb, Ra, Rb, E0, L0, want_h):
    """d7, B = ∂d7/∂δ (7×12), H = ∂²d7/∂δ² (7×12×12) by central differences."""
    h = np.empty(12)
    ht = 1e-4 * L0
    hr = 1e-4
    for i in range(12):
        h[i] = ht if (i % 6) < 3 else hr
    z = np.zeros(12)
    f0 = _d7_pert(xa, xb, Ra, Rb, E0, L0, z)
    fp = np.empty((12, 7))
    fm = np.empty((12, 7))
    for i in range(12):
        dl = np.zeros(12)
        dl[i] = h[i]
        fp[i] = _d7_pert(xa, xb, Ra, Rb, E0, L0, dl)
        dl[i] = -h[i]
        fm[i] = _d7_pert(xa, xb, Ra, Rb, E0, L0, dl)
    B = np.empty((7, 12))
    for i in range(12):
        B[:, i] = (fp[i] - fm[i]) / (2.0 * h[i])
    H = np.zeros((7, 12, 12))
    if want_h:
        for i in range(12):
            H[:, i, i] = (fp[i] - 2.0 * f0 + fm[i]) / (h[i] * h[i])
            for j in range(i + 1, 12):
                dl = np.zeros(12)
                dl[i] = h[i]
                dl[j] = h[j]
                fpp = _d7_pert(xa, xb, Ra, Rb, E0, L0, dl)
                dl[j] = -h[j]
                fpm = _d7_pert(xa, xb, Ra, Rb, E0, L0, dl)
                dl[i] = -h[i]
                fmm = _d7_pert(xa, xb, Ra, Rb, E0, L0, dl)
                dl[j] = h[j]
                fmp = _d7_pert(xa, xb, Ra, Rb, E0, L0, dl)
                v = (fpp - fpm - fmp + fmm) / (4.0 * h[i] * h[j])
                H[:, i, j] = v
                H[:, j, i] = v
    return f0, B, H


# --------------------------------------------------------------------------- shape functions


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


def _blockT(Rr: np.ndarray) -> np.ndarray:
    T = np.zeros((12, 12))
    for k in range(4):
        T[3 * k:3 * k + 3, 3 * k:3 * k + 3] = Rr.T
    return T


GAUSS3 = (np.array([0.5 - np.sqrt(0.15), 0.5, 0.5 + np.sqrt(0.15)]), np.array([5, 8, 5]) / 18.0)
_g4x = np.array([-0.8611363115940526, -0.3399810435848563, 0.3399810435848563, 0.8611363115940526])
_g4w = np.array([0.3478548451374538, 0.6521451548625461, 0.6521451548625461, 0.3478548451374538])
GAUSS4 = (0.5 * (_g4x + 1), 0.5 * _g4w)
IDX7 = np.array([6, 3, 4, 5, 9, 10, 11])


# --------------------------------------------------------------------------- element


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


@dataclass
class Element:
    n0: int
    n1: int
    e2: np.ndarray                     # reference e2 direction (H); orthogonalised to the axis
    sections: list[SectionProps]       # 3 (stiffness Gauss pts) — or 1 (uniform)
    mass_sections: Optional[list[SectionProps]] = None   # 4 Gauss pts; None → use sections
    material_fracs: dict = field(default_factory=dict)  # {material: EA fraction} for complex modulus
    tag: str = ""
    kind: str = "beam"                 # blade | flexure | rigid | tension
    blade: int = 0                     # 1-based blade index (0 = hub)
    massless: bool = False
    # derived
    L0: float = 0.0
    E0: np.ndarray = None
    K12: np.ndarray = None
    K7: np.ndarray = None
    mpts: np.ndarray = None            # (n,4): xi, y, z, m·w·L  (local section coords)

    def prepare(self, X0: np.ndarray) -> None:
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
        self.E0 = np.column_stack([e1, e2, e3])
        secs = self.sections if len(self.sections) == 3 else [self.sections[0]] * 3
        self.K12 = local_stiffness([s.stiffness6() for s in secs], self.L0)
        self.K7 = self.K12[np.ix_(IDX7, IDX7)].copy()
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
        self.mpts = np.array(pts) if pts else np.zeros((0, 4))

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

    def displacement(self, X0: np.ndarray) -> np.ndarray:
        """Full DOF vector-like (nn,6): translations, rotation vectors (log R)."""
        out = np.zeros((len(self.x), 6))
        out[:, :3] = self.x - X0
        for i, R in enumerate(self.R):
            out[i, 3:] = _logm(R)
        return out


class FEModel:
    def __init__(self) -> None:
        self.X0: list[np.ndarray] = []
        self.elements: list[Element] = []
        self.fixed: set[int] = set()
        self.lumped: list[tuple[int, np.ndarray]] = []   # (node, 6×6 lumped mass at node, rotating terms ignored)
        self.node_tags: list[str] = []
        self._prepared = False

    # building
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
    H: np.ndarray
    f7: np.ndarray


def element_cr(e: Element, st: State, want_h: bool = True) -> ElementCR:
    d7, B, H = _cr_derivs(st.x[e.n0], st.x[e.n1], st.R[e.n0], st.R[e.n1], e.E0, e.L0, want_h)
    return ElementCR(d7, B, H, e.K7 @ d7)


def local_geo_correction(e: Element, st: State, N: float) -> np.ndarray:
    """Initial-stress terms the rigid-chord CR kinematics miss (global 12×12):
    consistent-minus-string transverse bending terms N/(30l)[6,3l,−6,3l; …] and the
    trapeze (Wagner) torsion term N k²/l with k² = (EIy+EIz)/EA about the tension centre.
    Both are orthogonal to rigid-body motion, so rotating-frame consistency is kept."""
    if N == 0.0:
        return np.zeros((12, 12))
    l, Rr = _cr_frame(st.x[e.n0], st.x[e.n1], st.R[e.n0], st.R[e.n1], e.E0)
    k = np.array([[6, 3 * l, -6, 3 * l], [3 * l, 4 * l * l, -3 * l, -l * l],
                  [-6, -3 * l, 6, -3 * l], [3 * l, -l * l, -3 * l, 4 * l * l]]) * N / (30 * l)
    K = np.zeros((12, 12))
    # v (dir 2) with θ3 = v', w (dir 3) with θ2 = −w'
    i2 = [1, 5, 7, 11]
    K[np.ix_(i2, i2)] += k
    i3 = [2, 4, 8, 10]
    sg = np.diag([1.0, -1.0, 1.0, -1.0])
    K[np.ix_(i3, i3)] += sg @ k @ sg
    s0 = e.sections[len(e.sections) // 2]
    k2 = (s0.EIy + s0.EIz) / max(s0.EA, 1e-30)
    kt = N * k2 / l
    K[np.ix_([3, 9], [3, 9])] += kt * np.array([[1.0, -1.0], [-1.0, 1.0]])
    T = _blockT(Rr)
    return T.T @ K @ T


@dataclass
class StaticAssembly:
    fint: np.ndarray
    Kmat_e: list[np.ndarray]        # per element 12×12 (global coords), unit material scale
    Kgeo: np.ndarray
    cr: list[ElementCR]


def assemble_internal(model: FEModel, st: State, want_tangent: bool = True) -> StaticAssembly:
    n = model.ndof
    fint = np.zeros(n)
    Kgeo = np.zeros((n, n))
    Kmat_e, crs = [], []
    for e in model.elements:
        c = element_cr(e, st, want_tangent)
        dofs = _edofs(e)
        fint[dofs] += c.B.T @ c.f7
        if want_tangent:
            Km = c.B.T @ e.K7 @ c.B
            Kg = np.einsum("k,kij->ij", c.f7, c.H) + local_geo_correction(e, st, c.f7[0])
            Kgeo[np.ix_(dofs, dofs)] += 0.5 * (Kg + Kg.T)
            Kmat_e.append(0.5 * (Km + Km.T))
        crs.append(c)
    return StaticAssembly(fint, Kmat_e, Kgeo, crs)


def assemble_Kmat(model: FEModel, sa: StaticAssembly, scales=None) -> np.ndarray:
    n = model.ndof
    dtype = complex if scales is not None and np.iscomplexobj(np.asarray(scales)) else float
    K = np.zeros((n, n), dtype)
    for i, (e, Ke) in enumerate(zip(model.elements, sa.Kmat_e)):
        dofs = _edofs(e)
        K[np.ix_(dofs, dofs)] += Ke * (1.0 if scales is None else scales[i])
    return K


def element_point_maps(e: Element, st: State, xi: float, s_local_yz) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """For a material point at (xi, local y, z) of element e in the current state:
    returns (r (3,), s_H (3,) offset from node line, L (3×12) translation map, Nr (3×12) rotation map)."""
    xa, xb = st.x[e.n0], st.x[e.n1]
    l, Rr = _cr_frame(xa, xb, st.R[e.n0], st.R[e.n1], e.E0)
    th_a = _logm(Rr.T @ st.R[e.n0] @ e.E0)
    th_b = _logm(Rr.T @ st.R[e.n1] @ e.E0)
    Rloc = _expm((1 - xi) * th_a + xi * th_b)
    s_H = Rr @ Rloc @ np.array([0.0, s_local_yz[0], s_local_yz[1]])
    r = (1 - xi) * xa + xi * xb + s_H
    Nt, Nr = hermite_maps(xi, l)
    T = _blockT(Rr)
    Ntg = Rr @ Nt @ T
    Nrg = Rr @ Nr @ T
    L = Ntg - skew(s_H) @ Nrg
    return r, s_H, L, Nrg


@dataclass
class InertiaAssembly:
    M: np.ndarray
    G: np.ndarray
    Kcf: np.ndarray
    Fcf: np.ndarray


_P = np.diag([1.0, 1.0, 0.0])
_ZX = skew([0.0, 0.0, 1.0])


def assemble_inertia(model: FEModel, st: State, Omega: float, need_matrices: bool = True) -> InertiaAssembly:
    n = model.ndof
    M = np.zeros((n, n)) if need_matrices else None
    G = np.zeros((n, n)) if need_matrices else None
    Kcf = np.zeros((n, n)) if need_matrices else None
    Fcf = np.zeros(n)
    W2 = Omega * Omega
    for e in model.elements:
        if e.massless or len(e.mpts) == 0:
            continue
        dofs = _edofs(e)
        Me = np.zeros((12, 12))
        Ge = np.zeros((12, 12))
        Ke = np.zeros((12, 12))
        fe = np.zeros(12)
        for xi, y, z, m in e.mpts:
            r, s, L, Nr = element_point_maps(e, st, xi, (y, z))
            c = _P @ r
            fe += W2 * m * L.T @ c
            if need_matrices:
                Me += m * L.T @ L
                Ge += 2.0 * Omega * m * L.T @ _ZX @ L
                Q = 0.5 * (np.outer(s, c) + np.outer(c, s)) - (s @ c) * np.eye(3)
                Ke -= W2 * m * (L.T @ _P @ L + Nr.T @ Q @ Nr)
        Fcf[dofs] += fe
        if need_matrices:
            ix = np.ix_(dofs, dofs)
            M[ix] += Me
            G[ix] += Ge
            Kcf[ix] += 0.5 * (Ke + Ke.T)
    if need_matrices:
        for node, Mn in model.lumped:
            d = np.r_[6 * node:6 * node + 6]
            M[np.ix_(d, d)] += Mn
    return InertiaAssembly(M, G, Kcf, Fcf)


def euler_load(model: FEModel, st: State, Omega_dot: float) -> np.ndarray:
    """Generalised load from hub angular acceleration Ω̇ (frame rotating with the hub):
    −m Ω̇ ẑ × r per mass point."""
    F = np.zeros(model.ndof)
    for e in model.elements:
        if e.massless:
            continue
        dofs = _edofs(e)
        for xi, y, z, m in e.mpts:
            r, s, L, Nr = element_point_maps(e, st, xi, (y, z))
            F[dofs] += -Omega_dot * m * L.T @ np.cross([0.0, 0.0, 1.0], r)
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
                 state0: State | None = None, n_steps: int = 4, tol: float = 1e-8, max_iter: int = 30,
                 ) -> tuple[State, NewtonInfo]:
    """Nonlinear static equilibrium in the rotating frame: f_int(q) = F_cf(q, Ω) + F_ext.

    Load stepping over Ω² (n_steps) with full Newton per step. fext: constant vector or
    callable(state) (dead loads, may depend on the deformed shape)."""
    st = state0.copy() if state0 is not None else model.initial_state()
    free = model.free_dofs()
    info = NewtonInfo(True, 0, 0.0)
    total_it = 0
    for k in range(1, n_steps + 1):
        lam = k / n_steps
        Om = Omega * np.sqrt(lam)
        ref = None
        for it in range(max_iter):
            sa = assemble_internal(model, st, True)
            ia = assemble_inertia(model, st, Om, True)
            F = ia.Fcf.copy()
            if fext is not None:
                F += lam * (fext(st) if callable(fext) else fext)
            r = (sa.fint - F)[free]
            K = (assemble_Kmat(model, sa) + sa.Kgeo + ia.Kcf)[np.ix_(free, free)]
            scale = max(np.abs(F[free]).max(), np.abs(sa.fint[free]).max(), 1e-12)
            res = np.abs(r).max() / scale
            if ref is None:
                ref = res
            try:
                dq = np.linalg.solve(K, -r)
            except np.linalg.LinAlgError:
                dq = np.linalg.lstsq(K, -r, rcond=None)[0]
            # limit rotation increments to 0.2 rad per iteration
            du = np.zeros(model.ndof)
            du[free] = dq
            rot = np.abs(du.reshape(-1, 6)[:, 3:]).max()
            if rot > 0.2:
                du *= 0.2 / rot
            st = update_state(st, du)
            total_it += 1
            if res < tol or np.abs(dq).max() < 1e-13:
                break
        else:
            info = NewtonInfo(False, total_it, res, f"Newton did not converge at Ω step {k}")
    if info.converged:
        info = NewtonInfo(True, total_it, res)
    return st, info


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
