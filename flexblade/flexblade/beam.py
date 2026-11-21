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
from numba import njit, prange
from scipy.linalg import lu_factor as sl_lu_factor, lu_solve as sl_lu_solve

from .section import SectionProps

# FLEXBLADE_FAST=0 selects the reference kernels below (tests compare the two paths).
_FAST = __import__("os").environ.get("FLEXBLADE_FAST", "1") != "0"

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
        # second-order mixed differences reusing the ±h_i evaluations:
        # H_ij ≈ [f(+i+j) − f(+i) − f(+j) + 2f0 − f(−i) − f(−j) + f(−i−j)] / (2 h_i h_j)
        phi0 = fH @ d0
        fp = np.empty(12)
        fm = np.empty(12)
        for i in range(12):
            fp[i] = fH @ dp[i] + Vp[i]
            fm[i] = fH @ dm[i] + Vm[i]
            Kr[i, i] = (fp[i] - 2.0 * phi0 + fm[i]) / (h[i] * h[i])
        npos = npt if W2 != 0.0 else 0
        mp = mpts[:npos]
        tq = np.empty((npos, 3))
        p0 = pos0[:npos]
        for i in range(12):
            for j in range(i + 1, 12):
                dl = np.zeros(12)
                dl[i] = h[i]
                dl[j] = h[j]
                a, b, c, d = _pert(xa, xb, Ra, Rb, dl)
                _eval(a, b, c, d, E0, L0, k2, mp, tmpd, tq)
                spp = fH @ tmpd + _vcf(tq, p0, mp, W2)
                dl[i] = -h[i]
                dl[j] = -h[j]
                a, b, c, d = _pert(xa, xb, Ra, Rb, dl)
                _eval(a, b, c, d, E0, L0, k2, mp, tmpd, tq)
                smm = fH @ tmpd + _vcf(tq, p0, mp, W2)
                v = (spp - fp[i] - fp[j] + 2.0 * phi0 - fm[i] - fm[j] + smm) / (2.0 * h[i] * h[j])
                Kr[i, j] = v
                Kr[j, i] = v
    return d0, f7, B, gV, J, pos0, Kr



@njit(cache=True, parallel=True)
def _all_derivs(XA, XB, RA, RB, E0s, L0s, k2s, K7s, MP, NP, W2, want_h, FW, use_fw):
    ne = XA.shape[0]
    npmax = MP.shape[1]
    D7 = np.zeros((ne, 7))
    F7 = np.zeros((ne, 7))
    Bs = np.zeros((ne, 7, 12))
    GV = np.zeros((ne, 12))
    Js = np.zeros((ne, npmax, 3, 12))
    POS = np.zeros((ne, npmax, 3))
    KR = np.zeros((ne, 12, 12))
    for k in prange(ne):
        n = NP[k]
        fw = FW[k] if use_fw[k] else np.zeros(0)
        d0, f7, B, gV, J, pos, Kr = _element_derivs(XA[k], XB[k], RA[k], RB[k], E0s[k], L0s[k], k2s[k],
                                                    K7s[k], MP[k, :n], W2, want_h, fw)
        D7[k] = d0
        F7[k] = f7
        Bs[k] = B
        GV[k] = gV
        Js[k, :n] = J
        POS[k, :n] = pos
        KR[k] = Kr
    return D7, F7, Bs, GV, Js, POS, KR


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
    loss_eta: float = 0.0                    # hysteretic loss factor (no material_fracs)
    extra_mpts: Optional[np.ndarray] = None  # (n,4) extra point masses xi, y, z, mass (local)
    d7_0: Optional[np.ndarray] = None        # (7,) initial strain: stress-free CR deformation
                                             # (built-in end rotations); forces = K7·(d7 − d7_0)
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
        if self.extra_mpts is not None and len(self.extra_mpts):
            self.mpts = np.ascontiguousarray(np.vstack([self.mpts, np.asarray(self.extra_mpts, float)]))

    def complex_scale(self, freq_hz: float) -> complex:
        if not self.material_fracs:
            return 1.0 + 1j * self.loss_eta
        from .materials import get_material
        return (sum(fr * get_material(m).complex_scale(freq_hz) for m, fr in self.material_fracs.items())
                + 1j * self.loss_eta)   # extra structural loss (e.g. laminated-strip friction)


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
        self._pack = None
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


# Strip preset (I34) model: "slot" = the end-frame (clamp slot) is rotated by d7_0[1:4] / d7_0[4:7]
# (straight stress-free member between pre-angled slots, exact CR kinematics); "strain" = legacy
# initial-strain K7·(d7 − d7_0), which treats the end element as PRE-CURVED (the bow/arc-length term
# then couples end rotation to axial strain -> spurious arch stiffening; kept for comparison only).
PRESET_MODE = "slot"


def _rotv(w) -> np.ndarray:
    return _expm(np.asarray(w, float))


def slot_frames(e: "Element"):
    """(Qa, Qb) global-reference 3×3 right-factors: R_eff = R_node @ Q so R_eff E0 = R E0 expm(−p)."""
    if e.d7_0 is None or not np.any(e.d7_0) or PRESET_MODE != "slot":
        return None
    p = np.asarray(e.d7_0, float)
    return (e.E0 @ _rotv(-p[1:4]) @ e.E0.T, e.E0 @ _rotv(-p[4:7]) @ e.E0.T)


def _endR(e: "Element", st) -> tuple:
    Ra, Rb = st.R[e.n0], st.R[e.n1]
    q = slot_frames(e)
    return (Ra, Rb) if q is None else (Ra @ q[0], Rb @ q[1])


def element_cr(e: Element, st: State, Omega: float = 0.0, want_h: bool = True, fw=None) -> ElementCR:
    Ra, Rb = _endR(e, st)
    if PRESET_MODE == "strain" and e.d7_0 is not None and np.any(e.d7_0):
        p = np.asarray(e.d7_0, float)
        fp = e.K7 @ p
        if fw is None and want_h:
            c0 = element_cr(e, st, Omega, False, None)    # elastic f7 at this state (already shifted)
            fw = c0.f7
        d7, f7, B, gV, J, pos, Kr = _element_derivs(st.x[e.n0], st.x[e.n1], Ra, Rb, e.E0,
                                                    e.L0, e.k2, e.K7, e.mpts, float(Omega) ** 2, want_h,
                                                    _NOW if fw is None else np.asarray(fw, float))
        return ElementCR(d7 - p, B, f7 - fp, J, pos, gV, Kr)
    d7, f7, B, gV, J, pos, Kr = _element_derivs(st.x[e.n0], st.x[e.n1], Ra, Rb, e.E0,
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


def _pack(model: FEModel):
    """Static per-element arrays for the batched kernel (cached on the model)."""
    pk = getattr(model, "_pack", None)
    if pk is not None and pk[0] == len(model.elements) and pk[2] == PRESET_MODE:
        return pk[1]
    els = model.elements
    ne = len(els)
    npmax = max([len(e.mpts) for e in els] + [1])
    MP = np.zeros((ne, npmax, 4))
    NP = np.zeros(ne, np.int64)
    for k, e in enumerate(els):
        NP[k] = len(e.mpts)
        if NP[k]:
            MP[k, :NP[k]] = e.mpts
    d = dict(n0=np.array([e.n0 for e in els]), n1=np.array([e.n1 for e in els]),
             E0=np.ascontiguousarray(np.array([e.E0 for e in els])), L0=np.array([e.L0 for e in els]),
             k2=np.array([e.k2 for e in els]), K7=np.ascontiguousarray(np.array([e.K7 for e in els])),
             MP=MP, NP=NP, dofs=np.array([_edofs(e) for e in els]))
    P = np.zeros((ne, 7))
    for k, e in enumerate(els):
        if e.d7_0 is not None:
            P[k] = e.d7_0
    QA = np.repeat(np.eye(3)[None], ne, axis=0)
    QB = QA.copy()
    d["hasQ"] = False
    if PRESET_MODE == "slot":
        for k, e in enumerate(els):
            q = slot_frames(e)
            if q is not None:
                QA[k], QB[k] = q
                d["hasQ"] = True
        P = np.zeros((ne, 7))
    d["QA"], d["QB"] = QA, QB
    d["D7P"] = P
    d["hasP"] = bool(np.any(P))
    d["F7P"] = np.einsum("eij,ej->ei", d["K7"], P)
    model._pack = (ne, d, PRESET_MODE)
    return d


def slot_R(pk, RA, RB, ei=None):
    """Node rotations gathered per element -> effective end frames (pre-angled slots, I34)."""
    if not pk.get("hasQ"):
        return np.ascontiguousarray(RA), np.ascontiguousarray(RB)
    QA, QB = (pk["QA"], pk["QB"]) if ei is None else (pk["QA"][ei], pk["QB"][ei])
    return (np.ascontiguousarray(np.einsum("eij,ejk->eik", RA, QA)),
            np.ascontiguousarray(np.einsum("eij,ejk->eik", RB, QB)))


def assemble(model: FEModel, st: State, Omega: float = 0.0, want_tangent: bool = True,
             want_mass: bool = False, du_pred: np.ndarray | None = None) -> Assembly:
    """Assemble at state st (all elements in one parallel numba call).
    du_pred: if given, the initial-stress Hessian is weighted by the predicted element
    forces f7 + K7·B·du_pred (prestress predictor for unstressed starts)."""
    if _FAST:
        from . import beam_fast
        return beam_fast.assemble(model, st, Omega, want_tangent, want_mass, du_pred)
    n = model.ndof
    pk = _pack(model)
    ne = len(model.elements)
    n0, n1 = pk["n0"], pk["n1"]
    XA, XB = np.ascontiguousarray(st.x[n0]), np.ascontiguousarray(st.x[n1])
    RA, RB = slot_R(pk, st.R[n0], st.R[n1])
    W2 = float(Omega) ** 2
    FW = np.zeros((ne, 7))
    use_fw = np.zeros(ne, np.bool_)
    if want_tangent and (du_pred is not None or pk["hasP"]):
        D7, F7, Bs, *_ = _all_derivs(XA, XB, RA, RB, pk["E0"], pk["L0"], pk["k2"], pk["K7"], pk["MP"],
                                     pk["NP"], W2, False, FW, use_fw)
        FW = F7 - pk["F7P"]
        if du_pred is not None:
            FW = FW + np.einsum("eij,ejk,ek->ei", pk["K7"], Bs, du_pred[pk["dofs"]])
        use_fw[:] = True
    D7, F7, Bs, GV, Js, POS, KR = _all_derivs(XA, XB, RA, RB, pk["E0"], pk["L0"], pk["k2"], pk["K7"],
                                              pk["MP"], pk["NP"], W2, want_tangent, FW, use_fw)
    if pk["hasP"]:
        D7, F7 = D7 - pk["D7P"], F7 - pk["F7P"]
    dofs = pk["dofs"]
    fint = np.zeros(n)
    Fcf = np.zeros(n)
    np.add.at(fint, dofs, np.einsum("eij,ei->ej", Bs, F7))
    np.add.at(Fcf, dofs, -GV)
    Kg = M = G = None
    Kmat_e = []
    if want_tangent:
        Kg = np.zeros((n, n))
        KRs = 0.5 * (KR + KR.transpose(0, 2, 1))
        Kme = np.einsum("eki,ekl,elj->eij", Bs, pk["K7"], Bs)
        Kme = 0.5 * (Kme + Kme.transpose(0, 2, 1))
        rows = np.repeat(dofs, 12, axis=1)
        cols = np.tile(dofs, (1, 12))
        np.add.at(Kg, (rows.ravel(), cols.ravel()), KRs.reshape(ne, -1).ravel())
        Kmat_e = list(Kme)
    if want_mass:
        M = np.zeros((n, n))
        G = np.zeros((n, n))
        m = pk["MP"][:, :, 3]
        Me = np.einsum("ep,epki,epkj->eij", m, Js, Js)
        Ge = 2.0 * Omega * np.einsum("ep,epki,kl,eplj->eij", m, Js, _ZX, Js)
        Ge = 0.5 * (Ge - Ge.transpose(0, 2, 1))
        rows = np.repeat(dofs, 12, axis=1).ravel()
        cols = np.tile(dofs, (1, 12)).ravel()
        np.add.at(M, (rows, cols), (0.5 * (Me + Me.transpose(0, 2, 1))).ravel())
        np.add.at(G, (rows, cols), Ge.ravel())
        for node, Mn in model.lumped:
            d = np.r_[6 * node:6 * node + 6]
            M[np.ix_(d, d)] += Mn
    crs = [ElementCR(D7[k], Bs[k], F7[k], Js[k, :pk["NP"][k]], POS[k, :pk["NP"][k]], GV[k], KR[k])
           for k in range(ne)]
    return Assembly(fint, Fcf, Kmat_e, Kg, M, G, crs)


def assemble_internal(model: FEModel, st: State, want_tangent: bool = True, Omega: float = 0.0) -> Assembly:
    """Elastic part only (Ω = 0 by default: no centrifugal terms)."""
    return assemble(model, st, Omega, want_tangent, False)


def assemble_Kmat(model: FEModel, sa: Assembly, scales=None) -> np.ndarray:
    if _FAST:
        from . import beam_fast
        return beam_fast.assemble_Kmat(model, sa, scales)
    n = model.ndof
    dtype = complex if scales is not None and np.iscomplexobj(np.asarray(scales)) else float
    K = np.zeros((n, n), dtype)
    if not sa.Kmat_e:
        return K
    dofs = np.array([_edofs(e) for e in model.elements])
    Ke = np.array(sa.Kmat_e).astype(dtype)
    if scales is not None:
        Ke = Ke * np.asarray(scales, dtype)[:, None, None]
    rows = np.repeat(dofs, 12, axis=1).ravel()
    cols = np.tile(dofs, (1, 12)).ravel()
    np.add.at(K, (rows, cols), Ke.ravel())
    return K


def element_point_maps(e: Element, st: State, xi: float, s_local_yz):
    """Position r (3,), offset s_H, translation Jacobian L (3×12) and rotation map Nr (3×12)
    of a material point (xi, local y, z) — same CR kinematics as the mass points."""
    pts = np.array([[float(xi), float(s_local_yz[0]), float(s_local_yz[1]), 0.0]])
    Ra, Rb = _endR(e, st)
    _, _, _, _, J, pos, _ = _element_derivs(st.x[e.n0], st.x[e.n1], Ra, Rb, e.E0,
                                            e.L0, e.k2, e.K7, pts, 0.0, False, _NOW)
    xa, xb = st.x[e.n0], st.x[e.n1]
    l, Rr = _cr_frame(xa, xb, Ra, Rb, e.E0)
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
                 state0: State | None = None, n_steps: int = 4, tol: float = 1e-6, max_iter: int = 30,
                 verbose: bool = False, extra_fixed=(), lam0: float | None = None,
                 ext_tangent: Callable[[State], np.ndarray] | None = None,
                 max_total: int = 120) -> tuple[State, NewtonInfo]:
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
        """Full Newton (non-monotone: centrifugal problems overshoot before settling),
        fresh tangent each iteration, 0.1 rad rotation cap, step halving only on blow-up."""
        nonlocal total_it
        Om = Omega * np.sqrt(lam)
        r, res = residual(st, Om, lam)
        best = (res, st)
        hist = []
        K = None
        for it in range(max_it):
            if res < tol:
                return st, True, res, ""
            if len(hist) >= 3 and res < 1e-5 and res > 0.3 * hist[-3]:
                return st, True, res, f"stagnated at residual {res:.1e} (FD precision)"
            if K is not None and res < 1e-2 and len(hist) and res < 0.3 * hist[-1]:
                dq = lu_solve_(-r)
                du = np.zeros(model.ndof)
                du[free] = dq
                hist.append(res)
                st = update_state(st, du)
                r, res = residual(st, Om, lam)
                if res < best[0]:
                    best = (res, st)
                total_it += 1
                if verbose:
                    print(f"  lam {lam:.3f} it {it} (modified) res {hist[-1]:.3e} -> {res:.3e}")
                continue
            sa = assemble(model, st, Om, True, False)
            Km = assemble_Kmat(model, sa)
            if res > 0.2:
                # unbalanced state: predict prestress from a (mass-regularised) linear
                # solve and weight the initial-stress Hessian with it
                sm = assemble(model, st, Om, False, True)
                Kp = Km + (Om * Om + 1e-12) * np.diag(np.diag(sm.M))
                du_l = np.zeros(model.ndof)
                try:
                    du_l[free] = np.linalg.solve(Kp[np.ix_(free, free)], -r)
                    sa = assemble(model, st, Om, True, False, du_pred=du_l)
                except np.linalg.LinAlgError:
                    pass
            Kt = Km + sa.Kgeo
            if ext_tangent is not None:
                Kt = Kt - lam * ext_tangent(st)          # d(F_ext)/dq (e.g. aero stiffness)
            K = Kt[np.ix_(free, free)]
            try:
                lu = sl_lu_factor(K, check_finite=False)
                lu_solve_ = lambda b, lu=lu: sl_lu_solve(lu, b)   # noqa: E731
            except Exception:
                Kc = K.copy()
                lu_solve_ = lambda b, Kc=Kc: np.linalg.lstsq(Kc, b, rcond=None)[0]   # noqa: E731
            dq = lu_solve_(-r)
            du = np.zeros(model.ndof)
            du[free] = dq
            rot = np.abs(du.reshape(-1, 6)[:, 3:]).max()
            if not np.isfinite(rot):
                return best[1], False, best[0], "non-finite Newton step"
            if rot > 0.1:
                du *= 0.1 / rot
            hist.append(res)
            for _ in range(6):
                st2 = update_state(st, du)
                r2, res2 = residual(st2, Om, lam)
                if np.isfinite(res2) and res2 < max(1e3 * res, 10.0):
                    break
                du *= 0.25
            if verbose:
                print(f"  lam {lam:.3f} it {it} res {res:.3e} -> {res2:.3e} |du| {np.abs(du).max():.3e}")
            st, r, res = st2, r2, res2
            if res < best[0]:
                best = (res, st)
            total_it += 1
        return best[1], best[0] < tol, best[0], ""

    lam = 0.0
    dlam = 1.0 if state0 is not None else (lam0 or 1.0 / max(n_steps, 1))
    n_fail = 0
    res = np.inf
    while lam < 1.0 - 1e-12:
        lt = min(1.0, lam + dlam)
        st_try, ok, res, note = newton_at(st, lt, 12 if lt < 1.0 else max_iter)
        if ok:
            st, lam = st_try, lt
            msg = note or msg
            dlam = min(1.0, dlam * 2.0)
        else:
            n_fail += 1
            dlam *= 0.35
            if dlam < 2e-3 or n_fail > 6 or total_it > max_total:
                st = st_try
                ok_all = False
                msg = f"Newton did not converge at λ=(Ω/Ω₀)²={lt:.3f} (residual {res:.1e})"
                break
    return st, NewtonInfo(ok_all, total_it, float(res), msg)


# --------------------------------------------------------------------------- element recovery


def element_end_forces(e: Element, c: ElementCR, st: Optional[State] = None) -> tuple[np.ndarray, np.ndarray]:
    """Section forces [N, V2, V3, T, M2, M3] at end a and end b.

    st=None: K12·d7 in the element CR frame (legacy). Its rotational components are conjugate to the
    LOG end rotations, not spatial moments: off by ~½θ×M (θ = end rotation vs CR frame), which with
    a large in-plane M3 leaks several mN·m into the soft out-of-plane M2 and makes M2 jump at nodes.
    st given: the nodal action Bᵀf7 (spatial force/moment, global) resolved in the actual end-section
    frame R_node·Q_slot·E0 — continuous across nodes, consistent with nodal equilibrium."""
    if st is not None and e.L0 > 0.0:
        g = c.B.T @ c.f7
        Ra, Rb = _endR(e, st)
        Fa, Fb = Ra @ e.E0, Rb @ e.E0
        return (-np.r_[Fa.T @ g[0:3], Fa.T @ g[3:6]], np.r_[Fb.T @ g[6:9], Fb.T @ g[9:12]])
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
