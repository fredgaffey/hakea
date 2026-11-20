"""Allocation-free numba kernels for the co-rotational beam assembly (OWNER: performance).

Same mathematics, same finite-difference stencils and step sizes as the reference
kernels in beam.py (`_eval`, `_element_derivs`, `_all_derivs`); only the
implementation differs:

* 3×3 algebra is written out in scalars / preallocated buffers (the reference calls
  BLAS and allocates on the heap for every 3×3 product — that was ~90 % of its cost,
  and heap allocation serialises numba's prange threads);
* the per-element products R·E0 are formed once per element and perturbed by
  exp(ω) on the left (associativity changes rounding at the 1e-16 level only);
* exp(φ) of the section rotation is evaluated once per Gauss station instead of once
  per mass point (mass points come in groups sharing ξ — identical inputs, identical
  result);
* mass-point positions are only carried through the FD stencils when they are needed
  (Ω ≠ 0 or Jacobians wanted);
* the material tangent BᵀK7B, the mass / Coriolis element matrices and the scatter
  into the global arrays are done in the kernel (flat indices, no np.ix_ / add.at).

Thread policy (`_use_parallel`): serial kernels inside parallel workers
(FLEXBLADE_WORKER=1 — processes already fill the cores) and in non-main threads
unless numba's threading layer is thread-safe (tbb/omp; workqueue is not — the GUI
and MCP server run evaluations in thread pools); otherwise at most
FLEXBLADE_NUMBA_THREADS (default 4) threads — the element loop is only ~50 elements.
"""
from __future__ import annotations

import os
import threading

import numpy as np
import numba
from numba import njit, prange


def _n_threads() -> int:
    n = int(os.environ.get("FLEXBLADE_NUMBA_THREADS", "4"))
    return max(1, min(n, numba.config.NUMBA_NUM_THREADS))


def _use_parallel() -> bool:
    """Decide per call (env / thread can change after import, e.g. forked workers)."""
    if os.environ.get("FLEXBLADE_WORKER") == "1" or _n_threads() == 1:
        return False
    if threading.current_thread() is not threading.main_thread():
        try:
            if numba.threading_layer() not in ("tbb", "omp"):
                return False
        except ValueError:          # no parallel kernel launched yet: layer unknown
            return False
    numba.set_num_threads(_n_threads())    # thread-local in numba: set on every call
    return True


# --------------------------------------------------------------------------- scalar 3×3 helpers


@njit(cache=True)
def _expm_into(w0, w1, w2, out):
    th2 = w0 * w0 + w1 * w1 + w2 * w2
    if th2 < 1e-6:
        a = 1.0 - th2 / 6.0 + th2 * th2 / 120.0
        b = 0.5 - th2 / 24.0 + th2 * th2 / 720.0
    else:
        th = np.sqrt(th2)
        a = np.sin(th) / th
        b = (1.0 - np.cos(th)) / th2
    # K = skew(w); K@K entries as the reference's matrix product (same sums)
    out[0, 0] = 1.0 + b * (-w2 * w2 - w1 * w1)
    out[0, 1] = a * (-w2) + b * (w1 * w0)
    out[0, 2] = a * w1 + b * (w2 * w0)
    out[1, 0] = a * w2 + b * (w0 * w1)
    out[1, 1] = 1.0 + b * (-w2 * w2 - w0 * w0)
    out[1, 2] = a * (-w0) + b * (w2 * w1)
    out[2, 0] = a * (-w1) + b * (w0 * w2)
    out[2, 1] = a * w0 + b * (w1 * w2)
    out[2, 2] = 1.0 + b * (-w1 * w1 - w0 * w0)


@njit(cache=True)
def _logm3(R):
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
    return f * x, f * y, f * z


@njit(cache=True)
def _mm(A, B, out):
    for i in range(3):
        for j in range(3):
            out[i, j] = A[i, 0] * B[0, j] + A[i, 1] * B[1, j] + A[i, 2] * B[2, j]


@njit(cache=True)
def _mtm(A, B, out):
    """out = Aᵀ B"""
    for i in range(3):
        for j in range(3):
            out[i, j] = A[0, i] * B[0, j] + A[1, i] * B[1, j] + A[2, i] * B[2, j]


# --------------------------------------------------------------------------- one configuration


@njit(cache=True)
def _eval(xa, xb, RaE, RbE, L0, k2, mp, npts, d7, pos, Rr, T1, E):
    """d7 and (first npts) mass-point positions for node positions xa, xb and
    nodal frames RaE = R_a E0, RbE = R_b E0.  Rr, T1, E: 3×3 scratch."""
    if L0 <= 0.0:
        d7[0] = 0.0
        dx0 = xb[0] - xa[0]
        dx1 = xb[1] - xa[1]
        dx2 = xb[2] - xa[2]
        for i in range(3):
            d7[1 + i] = RaE[0, i] * dx0 + RaE[1, i] * dx1 + RaE[2, i] * dx2
        _mtm(RaE, RbE, T1)
        a0, a1, a2 = _logm3(T1)
        d7[4] = a0
        d7[5] = a1
        d7[6] = a2
        return
    d0 = xb[0] - xa[0]
    d1 = xb[1] - xa[1]
    d2_ = xb[2] - xa[2]
    l = np.sqrt(d0 * d0 + d1 * d1 + d2_ * d2_)
    r10, r11, r12 = d0 / l, d1 / l, d2_ / l
    q0 = 0.5 * (RaE[0, 1] + RbE[0, 1])
    q1 = 0.5 * (RaE[1, 1] + RbE[1, 1])
    q2 = 0.5 * (RaE[2, 1] + RbE[2, 1])
    r30 = r11 * q2 - r12 * q1
    r31 = r12 * q0 - r10 * q2
    r32 = r10 * q1 - r11 * q0
    nn = np.sqrt(r30 * r30 + r31 * r31 + r32 * r32)
    r30 /= nn
    r31 /= nn
    r32 /= nn
    Rr[0, 0] = r10
    Rr[1, 0] = r11
    Rr[2, 0] = r12
    Rr[0, 1] = r31 * r12 - r32 * r11
    Rr[1, 1] = r32 * r10 - r30 * r12
    Rr[2, 1] = r30 * r11 - r31 * r10
    Rr[0, 2] = r30
    Rr[1, 2] = r31
    Rr[2, 2] = r32
    _mtm(Rr, RaE, T1)
    ta0, ta1, ta2 = _logm3(T1)
    _mtm(Rr, RbE, T1)
    tb0, tb1, tb2 = _logm3(T1)
    bow = l * (2 * ta2 ** 2 - ta2 * tb2 + 2 * tb2 ** 2
               + 2 * ta1 ** 2 - ta1 * tb1 + 2 * tb1 ** 2) / 30.0
    d7[0] = l - L0 + bow + 0.5 * k2 * (tb0 - ta0) ** 2 / l
    d7[1] = ta0
    d7[2] = ta1
    d7[3] = ta2
    d7[4] = tb0
    d7[5] = tb1
    d7[6] = tb2
    xi_prev = -1e300
    for p in range(npts):
        xi = mp[p, 0]
        H2 = xi - 2 * xi**2 + xi**3
        H4 = -xi**2 + xi**3
        if xi != xi_prev:
            dd2 = 1 - 4 * xi + 3 * xi**2
            dd4 = -2 * xi + 3 * xi**2
            _expm_into((1 - xi) * ta0 + xi * tb0, dd2 * ta1 + dd4 * tb1, dd2 * ta2 + dd4 * tb2, E)
            xi_prev = xi
        sy = mp[p, 1]
        sz = mp[p, 2]
        o0 = E[0, 1] * sy + E[0, 2] * sz
        o1 = E[1, 1] * sy + E[1, 2] * sz
        o2 = E[2, 1] * sy + E[2, 2] * sz
        l0 = xi * l + o0
        l1 = l * (H2 * ta2 + H4 * tb2) + o1
        l2 = -l * (H2 * ta1 + H4 * tb1) + o2
        for c in range(3):
            pos[p, c] = xa[c] + (Rr[c, 0] * l0 + Rr[c, 1] * l1 + Rr[c, 2] * l2)


@njit(cache=True)
def _vcf(pos, pos0, mp, npts, W2):
    v = 0.0
    for p in range(npts):
        a = (pos[p, 0] - pos0[p, 0]) * (pos[p, 0] + pos0[p, 0]) \
            + (pos[p, 1] - pos0[p, 1]) * (pos[p, 1] + pos0[p, 1])
        v += -0.5 * W2 * mp[p, 3] * a
    return v


@njit(cache=True)
def _pert(xa, xb, RaE, RbE, dl, xap, xbp, Rap, Rbp, E):
    for c in range(3):
        xap[c] = xa[c] + dl[c]
        xbp[c] = xb[c] + dl[6 + c]
    if dl[3] != 0.0 or dl[4] != 0.0 or dl[5] != 0.0:
        _expm_into(dl[3], dl[4], dl[5], E)
        _mm(E, RaE, Rap)
    else:
        Rap[:, :] = RaE
    if dl[9] != 0.0 or dl[10] != 0.0 or dl[11] != 0.0:
        _expm_into(dl[9], dl[10], dl[11], E)
        _mm(E, RbE, Rbp)
    else:
        Rbp[:, :] = RbE


# --------------------------------------------------------------------------- one element


@njit(cache=True)
def _element(xa, xb, Ra, Rb, E0, L0, k2, K7, mp, n, W2, want_h, fw, use_fw, want_J,
             d0, f7, B, gV, J, pos0, Kr):
    """Fills d0 (7), f7 (7), B (7×12), gV (12), J (npmax,3,12), pos0 (npmax,3), Kr (12×12)."""
    RaE = np.empty((3, 3))
    RbE = np.empty((3, 3))
    _mm(Ra, E0, RaE)
    _mm(Rb, E0, RbE)
    Rr = np.empty((3, 3))
    T1 = np.empty((3, 3))
    E = np.empty((3, 3))
    xap = np.empty(3)
    xbp = np.empty(3)
    Rap = np.empty((3, 3))
    Rbp = np.empty((3, 3))
    h = np.empty(12)
    ht = 1e-4 * L0 if L0 > 0.0 else 1e-7
    for i in range(12):
        h[i] = ht if (i % 6) < 3 else 1e-4
    _eval(xa, xb, RaE, RbE, L0, k2, mp, n, d0, pos0, Rr, T1, E)
    for a in range(7):
        s = 0.0
        for b in range(7):
            s += K7[a, b] * d0[b]
        f7[a] = s
    npp = n if (W2 != 0.0 or want_J) else 0
    dp = np.empty((12, 7))
    dm = np.empty((12, 7))
    Vp = np.zeros(12)
    Vm = np.zeros(12)
    tp = np.empty((max(n, 1), 3))
    tm = np.empty((max(n, 1), 3))
    dl = np.zeros(12)
    for i in range(12):
        dl[i] = h[i]
        _pert(xa, xb, RaE, RbE, dl, xap, xbp, Rap, Rbp, E)
        _eval(xap, xbp, Rap, Rbp, L0, k2, mp, npp, dp[i], tp, Rr, T1, E)
        dl[i] = -h[i]
        _pert(xa, xb, RaE, RbE, dl, xap, xbp, Rap, Rbp, E)
        _eval(xap, xbp, Rap, Rbp, L0, k2, mp, npp, dm[i], tm, Rr, T1, E)
        dl[i] = 0.0
        for a in range(7):
            B[a, i] = (dp[i, a] - dm[i, a]) / (2.0 * h[i])
        if W2 != 0.0:
            Vp[i] = _vcf(tp, pos0, mp, n, W2)
            Vm[i] = _vcf(tm, pos0, mp, n, W2)
        gV[i] = (Vp[i] - Vm[i]) / (2.0 * h[i])
        if want_J:
            for p in range(n):
                for c in range(3):
                    J[p, c, i] = (tp[p, c] - tm[p, c]) / (2.0 * h[i])
    if want_h:
        fH = fw if use_fw else f7
        phi0 = 0.0
        for a in range(7):
            phi0 += fH[a] * d0[a]
        fp = np.empty(12)
        fm = np.empty(12)
        for i in range(12):
            sp = 0.0
            sm = 0.0
            for a in range(7):
                sp += fH[a] * dp[i, a]
                sm += fH[a] * dm[i, a]
            fp[i] = sp + Vp[i]
            fm[i] = sm + Vm[i]
            Kr[i, i] = (fp[i] - 2.0 * phi0 + fm[i]) / (h[i] * h[i])
        npos = n if W2 != 0.0 else 0
        dt = np.empty(7)
        for i in range(12):
            for j in range(i + 1, 12):
                dl[i] = h[i]
                dl[j] = h[j]
                _pert(xa, xb, RaE, RbE, dl, xap, xbp, Rap, Rbp, E)
                _eval(xap, xbp, Rap, Rbp, L0, k2, mp, npos, dt, tp, Rr, T1, E)
                spp = 0.0
                for a in range(7):
                    spp += fH[a] * dt[a]
                spp += _vcf(tp, pos0, mp, npos, W2)
                dl[i] = -h[i]
                dl[j] = -h[j]
                _pert(xa, xb, RaE, RbE, dl, xap, xbp, Rap, Rbp, E)
                _eval(xap, xbp, Rap, Rbp, L0, k2, mp, npos, dt, tp, Rr, T1, E)
                smm = 0.0
                for a in range(7):
                    smm += fH[a] * dt[a]
                smm += _vcf(tp, pos0, mp, npos, W2)
                dl[i] = 0.0
                dl[j] = 0.0
                v = (spp - fp[i] - fp[j] + 2.0 * phi0 - fm[i] - fm[j] + smm) / (2.0 * h[i] * h[j])
                Kr[i, j] = v
                Kr[j, i] = v


def all_derivs(XA, XB, RA, RB, E0s, L0s, k2s, K7s, MP, NP, W2, want_h, FW, use_fw, want_J=True):
    """Drop-in for beam._all_derivs (same outputs; Js is left zero if not want_J)."""
    f = _all_derivs_par if _use_parallel() else _all_derivs_ser
    return f(XA, XB, RA, RB, E0s, L0s, k2s, K7s, MP, NP, float(W2), bool(want_h), FW, use_fw, bool(want_J))


@njit(cache=True)
def _all_derivs_ser(XA, XB, RA, RB, E0s, L0s, k2s, K7s, MP, NP, W2, want_h, FW, use_fw, want_J):
    ne = XA.shape[0]
    npmax = MP.shape[1]
    D7 = np.zeros((ne, 7))
    F7 = np.zeros((ne, 7))
    Bs = np.zeros((ne, 7, 12))
    GV = np.zeros((ne, 12))
    Js = np.zeros((ne, npmax, 3, 12))
    POS = np.zeros((ne, npmax, 3))
    KR = np.zeros((ne, 12, 12))
    for k in range(ne):
        _element(XA[k], XB[k], RA[k], RB[k], E0s[k], L0s[k], k2s[k], K7s[k], MP[k], NP[k], W2,
                 want_h, FW[k], use_fw[k], want_J, D7[k], F7[k], Bs[k], GV[k], Js[k], POS[k], KR[k])
    return D7, F7, Bs, GV, Js, POS, KR


@njit(cache=True, parallel=True)
def _all_derivs_par(XA, XB, RA, RB, E0s, L0s, k2s, K7s, MP, NP, W2, want_h, FW, use_fw, want_J):
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
        _element(XA[k], XB[k], RA[k], RB[k], E0s[k], L0s[k], k2s[k], K7s[k], MP[k], NP[k], W2,
                 want_h, FW[k], use_fw[k], want_J, D7[k], F7[k], Bs[k], GV[k], Js[k], POS[k], KR[k])
    return D7, F7, Bs, GV, Js, POS, KR


# --------------------------------------------------------------------------- element matrices + scatter


def element_matrices(Bs, K7s, KR, Js, MP, NP, Omega, want_tangent, want_mass):
    """Symmetrised Kmat_e = BᵀK7B, Kr_e, M_e and skew G_e per element."""
    f = _element_matrices_par if _use_parallel() else _element_matrices_ser
    return f(Bs, K7s, KR, Js, MP, NP, float(Omega), bool(want_tangent), bool(want_mass))


@njit(cache=True)
def _element_matrix(k, Bs, K7s, KR, Js, MP, NP, Omega, want_tangent, want_mass, Kme, Kre, Me, Ge):
    if want_tangent:
        KB = np.zeros((7, 12))
        for a in range(7):
            for j in range(12):
                s = 0.0
                for b in range(7):
                    s += K7s[k, a, b] * Bs[k, b, j]
                KB[a, j] = s
        for i in range(12):
            for j in range(12):
                s = 0.0
                for a in range(7):
                    s += Bs[k, a, i] * KB[a, j]
                Kme[k, i, j] = s
        for i in range(12):
            for j in range(i, 12):
                v = 0.5 * (Kme[k, i, j] + Kme[k, j, i])
                Kme[k, i, j] = v
                Kme[k, j, i] = v
                w = 0.5 * (KR[k, i, j] + KR[k, j, i])
                Kre[k, i, j] = w
                Kre[k, j, i] = w
    if want_mass:
        for p in range(NP[k]):
            m = MP[k, p, 3]
            for i in range(12):
                a0 = Js[k, p, 0, i]
                a1 = Js[k, p, 1, i]
                a2 = Js[k, p, 2, i]
                for j in range(12):
                    b0 = Js[k, p, 0, j]
                    b1 = Js[k, p, 1, j]
                    Me[k, i, j] += m * (a0 * b0 + a1 * b1 + a2 * Js[k, p, 2, j])
                    # Jᵀ [ẑ×] J : [ẑ×] = [[0,-1,0],[1,0,0],[0,0,0]]
                    Ge[k, i, j] += 2.0 * Omega * m * (-a0 * b1 + a1 * b0)
        for i in range(12):
            for j in range(i, 12):
                v = 0.5 * (Me[k, i, j] + Me[k, j, i])
                Me[k, i, j] = v
                Me[k, j, i] = v
                g = 0.5 * (Ge[k, i, j] - Ge[k, j, i])
                Ge[k, i, j] = g
                Ge[k, j, i] = -g


@njit(cache=True)
def _element_matrices_ser(Bs, K7s, KR, Js, MP, NP, Omega, want_tangent, want_mass):
    ne = Bs.shape[0]
    Kme = np.zeros((ne, 12, 12))
    Kre = np.zeros((ne, 12, 12))
    Me = np.zeros((ne, 12, 12))
    Ge = np.zeros((ne, 12, 12))
    for k in range(ne):
        _element_matrix(k, Bs, K7s, KR, Js, MP, NP, Omega, want_tangent, want_mass, Kme, Kre, Me, Ge)
    return Kme, Kre, Me, Ge


@njit(cache=True, parallel=True)
def _element_matrices_par(Bs, K7s, KR, Js, MP, NP, Omega, want_tangent, want_mass):
    ne = Bs.shape[0]
    Kme = np.zeros((ne, 12, 12))
    Kre = np.zeros((ne, 12, 12))
    Me = np.zeros((ne, 12, 12))
    Ge = np.zeros((ne, 12, 12))
    for k in prange(ne):
        _element_matrix(k, Bs, K7s, KR, Js, MP, NP, Omega, want_tangent, want_mass, Kme, Kre, Me, Ge)
    return Kme, Kre, Me, Ge


@njit(cache=True)
def scatter_vec(out, dofs, Bs, F7, sign_vec, GV):
    """out_fint += Bᵀ F7 ; sign_vec += −GV (both scattered by element dofs)."""
    ne = dofs.shape[0]
    for k in range(ne):
        for i in range(12):
            s = 0.0
            for a in range(7):
                s += Bs[k, a, i] * F7[k, a]
            out[dofs[k, i]] += s
            sign_vec[dofs[k, i]] -= GV[k, i]


@njit(cache=True)
def scatter_mat(out, dofs, Ke):
    ne = dofs.shape[0]
    for k in range(ne):
        for i in range(12):
            r = dofs[k, i]
            for j in range(12):
                out[r, dofs[k, j]] += Ke[k, i, j]


@njit(cache=True)
def scatter_mat_c(out, dofs, Ke, scales):
    ne = dofs.shape[0]
    for k in range(ne):
        sc = scales[k]
        for i in range(12):
            r = dofs[k, i]
            for j in range(12):
                out[r, dofs[k, j]] += Ke[k, i, j] * sc


# --------------------------------------------------------------------------- material point maps (aero)


@njit(cache=True)
def point_maps(XA, XB, RA, RB, E0s, L0s, k2s, PTS):
    """Batched beam.element_point_maps for points PTS (np,4) = [xi, y, z, 0] on elements
    given per point (XA… already gathered per point).  Returns r (np,3), J (np,3,12),
    Rr (np,3,3), l (np)."""
    npnt = XA.shape[0]
    R = np.zeros((npnt, 3))
    J = np.zeros((npnt, 3, 12))
    RR = np.zeros((npnt, 3, 3))
    LL = np.zeros(npnt)
    K7 = np.zeros((7, 7))
    fw = np.zeros(7)
    for k in range(npnt):
        d0 = np.empty(7)
        f7 = np.empty(7)
        B = np.empty((7, 12))
        gV = np.empty(12)
        Jk = np.zeros((1, 3, 12))
        pos = np.zeros((1, 3))
        Kr = np.empty((12, 12))
        mp = PTS[k:k + 1]
        _element(XA[k], XB[k], RA[k], RB[k], E0s[k], L0s[k], k2s[k], K7, mp, 1, 0.0, False, fw, False,
                 True, d0, f7, B, gV, Jk, pos, Kr)
        R[k] = pos[0]
        J[k] = Jk[0]
        # chord frame at the base configuration
        RaE = np.empty((3, 3))
        RbE = np.empty((3, 3))
        _mm(RA[k], E0s[k], RaE)
        _mm(RB[k], E0s[k], RbE)
        Rr = np.empty((3, 3))
        T1 = np.empty((3, 3))
        E = np.empty((3, 3))
        dd = np.empty(7)
        tmp = np.empty((1, 3))
        _eval(XA[k], XB[k], RaE, RbE, L0s[k], k2s[k], mp, 0, dd, tmp, Rr, T1, E)
        RR[k] = Rr
        dx = XB[k] - XA[k]
        LL[k] = np.sqrt(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2])
    return R, J, RR, LL


# --------------------------------------------------------------------------- python drivers (called from beam.py)


def assemble(model, st, Omega=0.0, want_tangent=True, want_mass=False, du_pred=None):
    """Fast path of beam.assemble — same Assembly contents."""
    from .beam import Assembly, ElementCR, _ZX, _pack  # noqa: F401  (lazy: beam imports us lazily)
    n = model.ndof
    pk = _pack(model)
    ne = len(model.elements)
    n0, n1 = pk["n0"], pk["n1"]
    XA, XB = np.ascontiguousarray(st.x[n0]), np.ascontiguousarray(st.x[n1])
    RA, RB = np.ascontiguousarray(st.R[n0]), np.ascontiguousarray(st.R[n1])
    W2 = float(Omega) ** 2
    FW = np.zeros((ne, 7))
    use_fw = np.zeros(ne, np.bool_)
    args = (XA, XB, RA, RB, pk["E0"], pk["L0"], pk["k2"], pk["K7"], pk["MP"], pk["NP"])
    if du_pred is not None and want_tangent:
        D7, F7, Bs, *_ = all_derivs(*args, W2, False, FW, use_fw, False)
        FW = F7 + np.einsum("eij,ejk,ek->ei", pk["K7"], Bs, du_pred[pk["dofs"]])
        use_fw[:] = True
    D7, F7, Bs, GV, Js, POS, KR = all_derivs(*args, W2, want_tangent, FW, use_fw, bool(want_mass))
    dofs = pk["dofs"]
    fint = np.zeros(n)
    Fcf = np.zeros(n)
    scatter_vec(fint, dofs, Bs, F7, Fcf, GV)
    Kg = M = G = None
    Kmat_e = []
    Kme = None
    if want_tangent or want_mass:
        Kme, Kre, Me, Ge = element_matrices(Bs, pk["K7"], KR, Js, pk["MP"], pk["NP"], float(Omega),
                                            bool(want_tangent), bool(want_mass))
    if want_tangent:
        Kg = np.zeros((n, n))
        scatter_mat(Kg, dofs, Kre)
        Kmat_e = list(Kme)
    if want_mass:
        M = np.zeros((n, n))
        G = np.zeros((n, n))
        scatter_mat(M, dofs, Me)
        scatter_mat(G, dofs, Ge)
        for node, Mn in model.lumped:
            M[6 * node:6 * node + 6, 6 * node:6 * node + 6] += Mn
    NP = pk["NP"]
    crs = [ElementCR(D7[k], Bs[k], F7[k], Js[k, :NP[k]], POS[k, :NP[k]], GV[k], KR[k]) for k in range(ne)]
    sa = Assembly(fint, Fcf, Kmat_e, Kg, M, G, crs)
    if want_tangent:
        sa._Kme = Kme
    return sa


def assemble_Kmat(model, sa, scales=None):
    """Fast path of beam.assemble_Kmat."""
    from .beam import _pack
    n = model.ndof
    cplx = scales is not None and np.iscomplexobj(np.asarray(scales))
    K = np.zeros((n, n), complex if cplx else float)
    if not len(sa.Kmat_e):
        return K
    Ke = getattr(sa, "_Kme", None)
    if Ke is None:
        Ke = np.ascontiguousarray(np.array(sa.Kmat_e, float))
    dofs = _pack(model)["dofs"]
    if scales is None:
        scatter_mat(K, dofs, Ke)
    elif cplx:
        scatter_mat_c(K, dofs, Ke, np.asarray(scales, complex))
    else:
        scatter_mat_c(K, dofs, Ke, np.asarray(scales, float))
    return K


def element_point_maps_batch(model, st, elem_idx, xis, yz):
    """Batched beam.element_point_maps: lists r (np,3), sH (np,3), L (np,3,12), Nrg (np,3,12)."""
    from .beam import _pack
    pk = _pack(model)
    ei = np.asarray(elem_idx, np.int64)
    xis = np.asarray(xis, float)
    n0, n1 = pk["n0"][ei], pk["n1"][ei]
    XA, XB = np.ascontiguousarray(st.x[n0]), np.ascontiguousarray(st.x[n1])
    PTS = np.zeros((len(ei), 4))
    PTS[:, 0] = xis
    PTS[:, 1:3] = np.asarray(yz, float).reshape(-1, 2)
    r, L, Rr, l = point_maps(XA, XB, np.ascontiguousarray(st.R[n0]), np.ascontiguousarray(st.R[n1]),
                             np.ascontiguousarray(pk["E0"][ei]), pk["L0"][ei], pk["k2"][ei], PTS)
    # Nr (local rotation map, as beam.hermite_maps) → global: Rr Nr blockdiag(Rrᵀ)
    xi = xis
    d1, d2, d3, d4 = -6 * xi + 6 * xi**2, 1 - 4 * xi + 3 * xi**2, 6 * xi - 6 * xi**2, -2 * xi + 3 * xi**2
    Nr = np.zeros((len(ei), 3, 12))
    Nr[:, 0, 3], Nr[:, 0, 9] = 1 - xi, xi
    Nr[:, 1, 2], Nr[:, 1, 4], Nr[:, 1, 8], Nr[:, 1, 10] = -d1 / l, d2, -d3 / l, d4
    Nr[:, 2, 1], Nr[:, 2, 5], Nr[:, 2, 7], Nr[:, 2, 11] = d1 / l, d2, d3 / l, d4
    Nr4 = Nr.reshape(-1, 3, 4, 3)
    Nrg = np.einsum("pij,pjkl,pml->pikm", Rr, Nr4, Rr).reshape(-1, 3, 12)
    sH = r - ((1 - xi)[:, None] * XA + xi[:, None] * XB)
    return r, sH, L, Nrg

