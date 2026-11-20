"""3D solid FEA spot checks (scikit-fem meshes/dofs, vectorised P2 tetra assembly).

OWNER: fea agent.

Quadratic (P2, 10-node) tetrahedra on the linear gmsh tet mesh from
:func:`flexblade.geometry.build_solid_mesh`; scikit-fem provides the mesh, P2 dof numbering
and quadrature rules, element matrices are assembled here with numpy einsum (anisotropic
per-element 6×6 Voigt stiffness from ``Material.stiffness_matrix(orientation)``).

Physics (rotating hub frame, rotation +Ω about +z, linear):
* centrifugal body force  f = ρ Ω² (x, y, 0)
* spin softening          K_spin = −Ω² ∫ ρ Nᵀ diag(1,1,0) N
* geometric stiffness     K_σ = ∫ ∇Nᵀ σ₀ ∇N  from the static centrifugal stress σ₀
* Coriolis is neglected (modes are rotating-frame, undamped).

Public functions:
* ``root_stiffness(design, ...)``  6×6 flexure stiffness at the hinge point (frame H)
* ``static_centrifugal(design, rpm)``  stresses / interlayer stress / radial stretch
* ``modes(design, rpm)``  blade + flexure natural frequencies (clamped at hub side)
Low-level: ``FEModel`` (build from points/tets/C/rho), ``box_mesh``.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

from .materials import get_material
from .schema import Design

# local P2 edge order used by scikit-fem ElementTetP2 (verified at build time)
P2_EDGES = [(0, 1), (1, 2), (0, 2), (0, 3), (1, 3), (2, 3)]
_CHUNK = 20000


# =========================================================================== model


@dataclass
class FEModel:
    points: np.ndarray            # (Np,3)
    tets: np.ndarray              # (Ne,4)
    C: np.ndarray                 # (Ne,6,6) Voigt (11,22,33,23,13,12), engineering shear
    rho: np.ndarray               # (Ne,)
    build_dir: Optional[np.ndarray] = None   # (Ne,3) unit, for interlayer stress
    elem_part: Optional[np.ndarray] = None
    part_names: list[str] = field(default_factory=list)
    part_materials: list[str] = field(default_factory=list)

    def __post_init__(self):
        from skfem import Basis, ElementTetP2, MeshTet
        P = self.points
        t = np.array(self.tets, int)
        v = np.einsum("ij,ij->i", P[t[:, 1]] - P[t[:, 0]],
                      np.cross(P[t[:, 2]] - P[t[:, 0]], P[t[:, 3]] - P[t[:, 0]]))
        t[v < 0] = t[v < 0][:, [0, 2, 1, 3]]
        self.tets = t
        self.mesh = MeshTet(np.ascontiguousarray(self.points.T), np.ascontiguousarray(self.tets.T))
        self.sbasis = Basis(self.mesh, ElementTetP2(), intorder=2)
        self.edofs = self.sbasis.element_dofs.T            # (Ne,10) scalar dofs
        self.doflocs = self.sbasis.doflocs.T               # (Nd,3)
        self.n_sdof = self.sbasis.N
        self.ndof = 3 * self.n_sdof
        X = self.mesh.p.T[self.mesh.t.T]                   # (Ne,4,3)
        self.X = X
        # verify edge-dof order
        mid = 0.5 * (X[:, [i for i, _ in P2_EDGES]] + X[:, [j for _, j in P2_EDGES]])
        if not np.allclose(self.doflocs[self.edofs[:, 4:]], mid, atol=1e-12 + 1e-9 * np.abs(X).max()):
            raise RuntimeError("unexpected scikit-fem P2 dof ordering")
        if self.build_dir is None:
            self.build_dir = np.tile([0.0, 0.0, 1.0], (len(self.tets), 1))
        J = np.stack([X[:, 1] - X[:, 0], X[:, 2] - X[:, 0], X[:, 3] - X[:, 0]], axis=2)
        self.detJ = np.linalg.det(J)
        if (self.detJ <= 0).any():
            raise ValueError(f"{(self.detJ <= 0).sum()} inverted/degenerate tets")
        Jinv = np.linalg.inv(J)                            # rows: grad xi_a
        g = np.empty((len(X), 4, 3))
        g[:, 1:] = Jinv
        g[:, 0] = -Jinv.sum(1)
        self.glam = g                                      # (Ne,4,3) grad barycentric
        # global vector dof indices (Ne,30), local ordering 3*k + c
        self.vdofs = (3 * self.edofs[:, :, None] + np.arange(3)[None, None, :]).reshape(len(X), 30)

    # ----------------------------------------------------------------- shape functions
    @staticmethod
    def _quad(order):
        from skfem.quadrature import get_quadrature
        from skfem.refdom import RefTet
        xi, w = get_quadrature(RefTet, order)
        lam = np.vstack([1 - xi.sum(0), xi]).T             # (Nq,4)
        return lam, w

    @staticmethod
    def _N(lam):
        """P2 shape values (Nq,10)."""
        N = np.empty((len(lam), 10))
        for k in range(4):
            N[:, k] = lam[:, k] * (2 * lam[:, k] - 1)
        for e, (i, j) in enumerate(P2_EDGES):
            N[:, 4 + e] = 4 * lam[:, i] * lam[:, j]
        return N

    def _dN(self, lam, sl):
        """P2 shape gradients (ne,Nq,10,3) for element slice sl."""
        g = self.glam[sl]                                  # (ne,4,3)
        ne, nq = g.shape[0], len(lam)
        dN = np.empty((ne, nq, 10, 3))
        for k in range(4):
            dN[:, :, k, :] = (4 * lam[None, :, k, None] - 1) * g[:, None, k, :]
        for e, (i, j) in enumerate(P2_EDGES):
            dN[:, :, 4 + e, :] = 4 * (lam[None, :, i, None] * g[:, None, j, :]
                                      + lam[None, :, j, None] * g[:, None, i, :])
        return dN

    @staticmethod
    def _B(dN):
        """Voigt strain-displacement (ne,Nq,6,30)."""
        ne, nq = dN.shape[:2]
        B = np.zeros((ne, nq, 6, 10, 3))
        dx, dy, dz = dN[..., 0], dN[..., 1], dN[..., 2]
        B[:, :, 0, :, 0] = dx
        B[:, :, 1, :, 1] = dy
        B[:, :, 2, :, 2] = dz
        B[:, :, 3, :, 1] = dz
        B[:, :, 3, :, 2] = dy
        B[:, :, 4, :, 0] = dz
        B[:, :, 4, :, 2] = dx
        B[:, :, 5, :, 0] = dy
        B[:, :, 5, :, 1] = dx
        return B.reshape(ne, nq, 6, 30)

    def _assemble(self, blocks, dofs):
        I = np.repeat(dofs, dofs.shape[1], axis=1).ravel()
        Jx = np.tile(dofs, (1, dofs.shape[1])).ravel()
        return sp.coo_matrix((blocks.ravel(), (I, Jx)), shape=(self.ndof, self.ndof)).tocsr()

    # ----------------------------------------------------------------- matrices
    def stiffness(self, C_scale: Optional[np.ndarray] = None) -> sp.csr_matrix:
        lam, w = self._quad(2)
        K = None
        for s in range(0, len(self.tets), _CHUNK):
            sl = slice(s, s + _CHUNK)
            B = self._B(self._dN(lam, sl))
            C = self.C[sl] if C_scale is None else self.C[sl] * C_scale[sl, None, None]
            wd = w[None, :] * self.detJ[sl, None]
            Ke = np.einsum("eqai,eab,eqbj,eq->eij", B, C, B, wd, optimize=True)
            Kc = self._assemble(Ke, self.vdofs[sl])
            K = Kc if K is None else K + Kc
        return K

    def _scalar_mass_blocks(self, weight=None):
        lam, w = self._quad(4)
        N = self._N(lam)
        Me = np.einsum("q,qk,ql->kl", w, N, N)            # reference (10,10)
        dens = self.rho * self.detJ if weight is None else weight * self.detJ
        return dens[:, None, None] * Me[None]

    def mass(self, comps=(0, 1, 2)) -> sp.csr_matrix:
        Ms = self._scalar_mass_blocks()
        ne = len(self.tets)
        blocks = np.zeros((ne, 10, 3, 10, 3))
        for c in comps:
            blocks[:, :, c, :, c] = Ms
        return self._assemble(blocks.reshape(ne, 30, 30), self.vdofs)

    def centrifugal_load(self, omega: float) -> np.ndarray:
        """Consistent nodal load of body force ρΩ²(x,y,0)."""
        lam, w = self._quad(4)
        N = self._N(lam)                                   # (Nq,10)
        xq = np.einsum("qk,ekd->eqd", lam, self.X)         # (Ne,Nq,3)
        fe = np.einsum("q,qk,eqd,e->ekd", w, N, xq, self.rho * self.detJ) * omega ** 2
        fe[:, :, 2] = 0.0
        f = np.zeros(self.ndof)
        np.add.at(f, self.vdofs.ravel(), fe.reshape(len(self.tets), 30).ravel())
        return f

    def stress_qp(self, u: np.ndarray, C_scale=None) -> np.ndarray:
        """Cauchy stress at quadrature points (Ne,Nq,6) Voigt."""
        lam, _ = self._quad(2)
        out = []
        for s in range(0, len(self.tets), _CHUNK):
            sl = slice(s, s + _CHUNK)
            B = self._B(self._dN(lam, sl))
            ue = u[self.vdofs[sl]]
            eps = np.einsum("eqai,ei->eqa", B, ue)
            C = self.C[sl] if C_scale is None else self.C[sl] * C_scale[sl, None, None]
            out.append(np.einsum("eab,eqb->eqa", C, eps))
        return np.concatenate(out)

    def geometric_stiffness(self, sig_qp: np.ndarray) -> sp.csr_matrix:
        lam, w = self._quad(2)
        K = None
        for s in range(0, len(self.tets), _CHUNK):
            sl = slice(s, s + _CHUNK)
            dN = self._dN(lam, sl)
            sv = sig_qp[sl]
            S = np.empty(sv.shape[:2] + (3, 3))
            S[..., 0, 0], S[..., 1, 1], S[..., 2, 2] = sv[..., 0], sv[..., 1], sv[..., 2]
            S[..., 1, 2] = S[..., 2, 1] = sv[..., 3]
            S[..., 0, 2] = S[..., 2, 0] = sv[..., 4]
            S[..., 0, 1] = S[..., 1, 0] = sv[..., 5]
            wd = w[None, :] * self.detJ[sl, None]
            Ks = np.einsum("eqka,eqab,eqlb,eq->ekl", dN, S, dN, wd, optimize=True)
            ne = Ks.shape[0]
            blocks = np.zeros((ne, 10, 3, 10, 3))
            for c in range(3):
                blocks[:, :, c, :, c] = Ks
            Kc = self._assemble(blocks.reshape(ne, 30, 30), self.vdofs[sl])
            K = Kc if K is None else K + Kc
        return K

    # ----------------------------------------------------------------- helpers
    def dofs_of_nodes(self, mask: np.ndarray) -> np.ndarray:
        """Vector dofs of scalar dof locations where mask (Nd,) is True."""
        idx = np.where(mask)[0]
        return (3 * idx[:, None] + np.arange(3)).ravel()

    def volume(self) -> float:
        return float(self.detJ.sum() / 6)

    def total_mass(self) -> float:
        return float((self.rho * self.detJ).sum() / 6)


# =========================================================================== constraints


def _transform(model: FEModel, fixed: np.ndarray, slaves: Optional[np.ndarray] = None,
               master: Optional[np.ndarray] = None):
    """u = T q; q = [free independent dofs..., (6 master dofs)].

    fixed: scalar-dof mask clamped; slaves: scalar-dof mask rigidly linked to master point.
    Returns T (ndof × nq), index of master dofs in q (or None).
    """
    nd = model.n_sdof
    fixed_v = np.zeros(model.ndof, bool)
    fixed_v[model.dofs_of_nodes(fixed)] = True
    slave_v = np.zeros(model.ndof, bool)
    if slaves is not None:
        slaves = slaves & ~fixed
        slave_v[model.dofs_of_nodes(slaves)] = True
    free = np.where(~fixed_v & ~slave_v)[0]
    nf = len(free)
    rows, cols, vals = list(free), list(range(nf)), [1.0] * nf
    nq = nf
    mdofs = None
    if slaves is not None and slaves.any():
        mdofs = np.arange(nf, nf + 6)
        nq += 6
        sidx = np.where(slaves)[0]
        r = model.doflocs[sidx] - np.asarray(master)[None, :]
        # u = U + Θ × r  ->  ux = Ux + θy rz − θz ry ; uy = Uy + θz rx − θx rz ; uz = Uz + θx ry − θy rx
        for c in range(3):
            rows += list(3 * sidx + c)
            cols += [nf + c] * len(sidx)
            vals += [1.0] * len(sidx)
        terms = [(0, 4, r[:, 2]), (0, 5, -r[:, 1]), (1, 5, r[:, 0]), (1, 3, -r[:, 2]),
                 (2, 3, r[:, 1]), (2, 4, -r[:, 0])]
        for c, m, v in terms:
            rows += list(3 * sidx + c)
            cols += [nf + m] * len(sidx)
            vals += list(v)
    T = sp.csr_matrix((vals, (rows, cols)), shape=(model.ndof, nq))
    return T, mdofs


def factorize(A):
    """Return solve(b) for sparse symmetric A. Uses pypardiso (multithreaded MKL) when
    installed, else SuperLU (COLAMD). SuperLU: ~0.2 s at 13k dof, ~9 s at 60k dof."""
    A = A.tocsr()
    try:
        import pypardiso  # type: ignore
        solver = pypardiso.PyPardisoSolver()
        solver.set_matrix_type(-2)          # real symmetric indefinite (covers SPD)
        Au = sp.triu(A, format="csr")
        solver.factorize(Au)
        return lambda b: solver.solve(Au, np.asarray(b, float))
    except ImportError:
        pass
    lu = spla.splu(A.tocsc(), permc_spec="COLAMD")
    return lu.solve


def _solve(A, b):
    return factorize(A)(b)


# =========================================================================== design -> model


def _material_arrays(sm, freq_hz: Optional[float] = None):
    from .materials import rotation_matrix
    C_parts, rho_parts, b_parts = [], [], []
    for p in sm.parts:
        mat = get_material(p.material)
        C = mat.stiffness_matrix(p.orientation)
        if freq_hz is not None:
            C = C * mat.storage_scale(freq_hz)
        C_parts.append(C)
        rho_parts.append(mat.rho)
        b_parts.append(rotation_matrix(p.orientation)[:, 2])
    C = np.array(C_parts)[sm.elem_part]
    rho = np.array(rho_parts)[sm.elem_part]
    bd = np.array(b_parts)[sm.elem_part]
    return C, rho, bd


def model_from_design(design: Design, region: str = "root", h: Optional[float] = None,
                      h_member: Optional[float] = None, freq_hz: Optional[float] = None):
    from .geometry import build_solid_mesh
    t0 = time.perf_counter()
    sm = build_solid_mesh(design, region=region, h=h, h_member=h_member)
    t1 = time.perf_counter()
    C, rho, bd = _material_arrays(sm, freq_hz)
    model = FEModel(sm.points, sm.tets, C, rho, bd, sm.elem_part,
                    [p.name for p in sm.parts], [p.material for p in sm.parts])
    model.solid_mesh = sm
    model.timings = dict(mesh=t1 - t0, setup=time.perf_counter() - t1)
    return model


def _end_face_mask(model: FEModel, members, end: int, tol: float) -> np.ndarray:
    """Scalar-dof mask of points on member end faces (end 0 = hub side p0, 1 = p1)."""
    from .geometry import member_frame
    X = model.doflocs
    mask = np.zeros(len(X), bool)
    for m in members:
        if m.kind == "rigid":
            continue
        p0, p1, L, t, w, n = member_frame(m)
        c = p0 if end == 0 else p1
        th = m.thickness * (1.0 if end == 0 else m.taper)
        wid = m.width if m.shape == "rect" else th
        d = X - c
        on = np.abs(d @ t) < tol
        on &= np.abs(d @ w) <= wid / 2 + tol
        on &= np.abs(d @ n) <= th / 2 + tol
        mask |= on
    return mask


def _rpm_or_default(design: Design, rpm: Optional[float], warnings: list) -> float:
    if rpm is not None:
        return float(rpm)
    if design.op.rpm:
        return float(design.op.rpm)
    try:
        from .aero import AeroModel
        tr = AeroModel(design).trim()
        return float(tr.rpm)
    except Exception as e:  # noqa: BLE001
        # momentum fallback: T = CT ρ A (ΩR)², CT ≈ 0.012
        R = design.blade.radius
        T = design.op.vehicle_mass * 9.81
        om = math.sqrt(T / (0.012 * design.op.rho * math.pi * R ** 2)) / R
        warnings.append(f"rpm from CT=0.012 estimate (aero trim unavailable: {e})")
        return om * 60 / (2 * math.pi)


def _blade_outboard_mass(design: Design):
    """Mass and first radial moment of blade-1 parts outboard of the flexure
    (blade/skin/core/rod + blade-root fitting) from the surface meshes."""
    import trimesh
    from .geometry import build_surface
    surf = build_surface(design)
    m_tot, mx = 0.0, 0.0
    for p in surf["parts"]:
        if p["blade"] != 1 or p["role"] not in ("blade", "skin", "core", "rod") and \
                not p["name"].endswith("root_fitting"):
            continue
        tm = trimesh.Trimesh(p["vertices"], p["faces"], process=False)
        rho = get_material(p["material"]).rho
        vol = tm.volume
        com = tm.center_mass
        if p["role"] in ("skin",):
            # skin part is the outer envelope; core subtracted below
            pass
        if p["role"] == "core":
            # core lies inside skin envelope: replace skin density by core density
            skin_rho = get_material(design.blade.layup.skin_material).rho
            rho = rho - skin_rho
        m_tot += rho * vol
        mx += rho * vol * com[0]
    return m_tot, mx


# =========================================================================== public API


def root_stiffness(design: Design, rpm: Optional[float] = 0.0, h_member: Optional[float] = None,
                   freq_hz: Optional[float] = None, return_model: bool = False) -> dict:
    """6×6 stiffness of blade-1 flexure members at the hinge point (frame H).

    Hub-side member end faces clamped; blade-side end faces rigidly linked (RBE2 by exact
    elimination) to a master node at the hinge point P = hinge_axis()[0]. K = Schur
    complement onto the 6 master dofs [ux,uy,uz,rx,ry,rz]. If rpm > 0, geometric stiffness
    from the centrifugal prestress is added (member body force + outboard blade centrifugal
    force applied at the master, radial). freq_hz: scale moduli by storage modulus at freq.

    Returns dict(K (6×6 list), point, axis, compliance, k_hinge (rotational stiffness about
    the hinge axis with other dofs free), k_hinge_clamped (a·K_rr·a), pitch_lag_coupling
    (−θx/θz under a pure lag moment; ≈ tan δ for an ideal skewed hinge), n_dof, timings).
    """
    timings = {}
    t0 = time.perf_counter()
    model = model_from_design(design, "root", h_member=h_member, freq_hz=freq_hz)
    timings.update(model.timings)
    sm = model.solid_mesh
    P, a = sm.hinge
    tol = 1e-3 * max(m.thickness for m in sm.members) + 1e-7
    fixed = _end_face_mask(model, sm.members, 0, tol)
    slaves = _end_face_mask(model, sm.members, 1, tol)
    if not fixed.any() or not slaves.any():
        raise RuntimeError("could not find member end faces for BCs")
    t1 = time.perf_counter()
    K = model.stiffness()
    timings["assemble"] = time.perf_counter() - t1
    T, md = _transform(model, fixed, slaves, P)
    warnings: list[str] = []
    F_cf = 0.0
    if rpm:
        om = rpm * 2 * math.pi / 60
        f = model.centrifugal_load(om)
        m_b, mx_b = _blade_outboard_mass(design)
        F_cf = om ** 2 * mx_b
        Kr = (T.T @ K @ T).tocsc()
        fr = T.T @ f
        fr[md[0]] += F_cf
        q = _solve(Kr, fr)
        u = T @ q
        sig = model.stress_qp(u)
        K = K + model.geometric_stiffness(sig)
    t2 = time.perf_counter()
    Kr = (T.T @ K @ T).tocsc()
    ii = np.setdiff1d(np.arange(Kr.shape[0]), md)
    Kii = Kr[ii][:, ii].tocsc()
    Kim = Kr[ii][:, md].toarray()
    Kmm = Kr[md][:, md].toarray()
    X = _solve(Kii, Kim)
    K6 = Kmm - Kim.T @ X
    K6 = 0.5 * (K6 + K6.T)
    timings["solve"] = time.perf_counter() - t2
    timings["total"] = time.perf_counter() - t0
    Cm = np.linalg.inv(K6)
    a = np.asarray(a, float) / np.linalg.norm(a)
    e = np.concatenate([np.zeros(3), a])
    k_h = 1.0 / float(e @ Cm @ e)
    th = Cm[3:, 3:] @ np.array([0, 0, -1.0])       # unit lag moment (about −z)
    coupling = float(th[0] / -th[2]) if abs(th[2]) > 0 else float("nan")
    out = dict(K=K6.tolist(), point=list(map(float, P)), axis=list(map(float, a)),
               compliance=Cm.tolist(), k_hinge=k_h, k_hinge_clamped=float(a @ K6[3:, 3:] @ a),
               pitch_lag_coupling=coupling, rpm=rpm or 0.0, centrifugal_force=F_cf,
               n_dof=int(model.ndof), n_tets=int(len(model.tets)), timings=timings,
               members_source="see geometry", warnings=warnings + sm.warnings)
    if return_model:
        out["model"] = model
    return out


def _clamp_hub_side(model: FEModel, sm, hub_radius: float = 0.0) -> np.ndarray:
    """Clamp mask: hub-side member end faces (root/blade1), or the shaft core
    r < hub_radius/2 of the hub cylinder (full)."""
    if sm.region == "full":
        r = np.linalg.norm(model.doflocs[:, :2], axis=1)
        return r <= 0.5 * hub_radius
    tol = 1e-3 * max([m.thickness for m in sm.members] or [1e-3]) + 1e-7
    fixed = _end_face_mask(model, sm.members, 0, tol)
    if not fixed.any():
        raise RuntimeError("no hub-side member end faces found to clamp")
    return fixed


def static_centrifugal(design: Design, rpm: Optional[float] = None, h: Optional[float] = None,
                       h_member: Optional[float] = None, return_model: bool = False) -> dict:
    """Linear static centrifugal load case of blade 1 + flexure (hub side clamped).

    Returns max von Mises, max principal, max interlayer normal stress (σ along the print
    build_dir, tension +), per-part maxima + utilisation vs material strength, radial
    stretch (tip and flexure blade-end mean u_x), centrifugal root force and max strain.
    Linear small-strain: for soft TPU with >~5 % strain treat as indicative only.
    """
    warnings: list[str] = []
    rpm = _rpm_or_default(design, rpm, warnings)
    om = rpm * 2 * math.pi / 60
    t0 = time.perf_counter()
    model = model_from_design(design, "blade1", h=h, h_member=h_member)
    sm = model.solid_mesh
    fixed = _clamp_hub_side(model, sm)
    K = model.stiffness()
    f = model.centrifugal_load(om)
    T, _ = _transform(model, fixed)
    q = _solve((T.T @ K @ T).tocsc(), T.T @ f)
    u = T @ q
    sig = model.stress_qp(u)
    res = _stress_summary(model, sig)
    X = model.doflocs
    R = design.blade.radius
    tip = X[:, 0] > 0.98 * R
    flex_end = _end_face_mask(model, sm.members, 1, 1e-3 * min(m.thickness for m in sm.members) + 1e-7)
    ux = u[0::3]
    res.update(dict(
        rpm=rpm, radial_stretch_tip=float(ux[tip].mean()) if tip.any() else float("nan"),
        radial_stretch_flexure=float(ux[flex_end].mean()) if flex_end.any() else float("nan"),
        centrifugal_force_root=float(f[0::3].sum()),
        max_displacement=float(np.abs(u).max()),
        mass=model.total_mass(), n_dof=int(model.ndof), n_tets=int(len(model.tets)),
        timings=dict(model.timings, total=time.perf_counter() - t0), warnings=warnings + sm.warnings))
    if return_model:
        res["model"], res["u"], res["sigma"] = model, u, sig
    return res


def _stress_summary(model: FEModel, sig: np.ndarray) -> dict:
    s = sig
    vm = np.sqrt(0.5 * ((s[..., 0] - s[..., 1]) ** 2 + (s[..., 1] - s[..., 2]) ** 2
                        + (s[..., 2] - s[..., 0]) ** 2) + 3 * (s[..., 3] ** 2 + s[..., 4] ** 2 + s[..., 5] ** 2))
    S = np.empty(s.shape[:2] + (3, 3))
    S[..., 0, 0], S[..., 1, 1], S[..., 2, 2] = s[..., 0], s[..., 1], s[..., 2]
    S[..., 1, 2] = S[..., 2, 1] = s[..., 3]
    S[..., 0, 2] = S[..., 2, 0] = s[..., 4]
    S[..., 0, 1] = S[..., 1, 0] = s[..., 5]
    p1 = np.linalg.eigvalsh(S)[..., -1]
    b = model.build_dir
    sn = np.einsum("ea,eqab,eb->eq", b, S, b)
    per = {}
    ep = model.elem_part if model.elem_part is not None else np.zeros(len(s), int)
    for pi in np.unique(ep):
        sel = ep == pi
        name = model.part_names[pi] if model.part_names else str(pi)
        mat = get_material(model.part_materials[pi]) if model.part_materials else None
        d = dict(max_von_mises=float(vm[sel].max()), p999_von_mises=float(np.percentile(vm[sel], 99.9)),
                 max_principal=float(p1[sel].max()), max_interlayer_normal=float(sn[sel].max()))
        if mat is not None:
            d["material"] = mat.name
            d["util_principal_vs_Xt"] = d["max_principal"] / mat.Xt
            d["util_interlayer_vs_Zt"] = d["max_interlayer_normal"] / mat.Zt
        per[name] = d
    return dict(max_von_mises=float(vm.max()), max_principal=float(p1.max()),
                max_interlayer_normal=float(sn.max()),
                p999_von_mises=float(np.percentile(vm, 99.9)), parts=per)


def _label_mode(model: FEModel, phi: np.ndarray, R: float) -> tuple[str, dict]:
    X = model.doflocs
    tip = X[:, 0] > 0.9 * X[:, 0].max()
    U = np.stack([phi[0::3], phi[1::3], phi[2::3]], 1)[tip]
    Xt = X[tip] - X[tip].mean(0)
    u0 = U.mean(0)
    # least-squares small rotation of tip section: U - u0 ≈ θ × r
    A = np.zeros((3 * len(Xt), 3))
    A[0::3, 1], A[0::3, 2] = Xt[:, 2], -Xt[:, 1]
    A[1::3, 0], A[1::3, 2] = -Xt[:, 2], Xt[:, 0]
    A[2::3, 0], A[2::3, 1] = Xt[:, 1], -Xt[:, 0]
    th = np.linalg.lstsq(A, (U - u0).ravel(), rcond=None)[0]
    half_chord = max(np.ptp(Xt[:, 1]) / 2, 1e-6)
    comp = dict(axial=abs(u0[0]), lag=abs(u0[1]), flap=abs(u0[2]), pitch=abs(th[0]) * half_chord)
    tot = sum(comp.values()) or 1.0
    comp = {k: v / tot for k, v in comp.items()}
    return max(comp, key=comp.get), comp


def modes(design: Design, rpm: Optional[float] = 0.0, n_modes: int = 6,
          prestress: bool = True, spin_softening: bool = True, h: Optional[float] = None,
          h_member: Optional[float] = None, freq_hz: Optional[float] = None) -> dict:
    """Natural frequencies of blade 1 + flexure, hub-side member ends clamped.

    rpm > 0: centrifugal prestress (K_σ) and spin softening; rotating-frame frequencies,
    Coriolis neglected. freq_hz: evaluate moduli at this frequency (TPU storage modulus);
    default quasi-static. Returns dict(freq_hz list, labels, participation, ...).
    """
    warnings: list[str] = []
    rpm = float(rpm or 0.0)
    om = rpm * 2 * math.pi / 60
    t0 = time.perf_counter()
    model = model_from_design(design, "blade1", h=h, h_member=h_member, freq_hz=freq_hz)
    sm = model.solid_mesh
    fixed = _clamp_hub_side(model, sm)
    K = model.stiffness()
    M = model.mass()
    T, _ = _transform(model, fixed)
    if om > 0:
        if prestress:
            f = model.centrifugal_load(om)
            q = _solve((T.T @ K @ T).tocsc(), T.T @ f)
            K = K + model.geometric_stiffness(model.stress_qp(T @ q))
        if spin_softening:
            K = K - om ** 2 * model.mass(comps=(0, 1))
    Kr = (T.T @ K @ T).tocsc()
    Mr = (T.T @ M @ T).tocsc()
    t1 = time.perf_counter()
    k = min(n_modes, Kr.shape[0] - 2)
    shift = -1.0
    solve = factorize((Kr - shift * Mr).tocsc())
    OPinv = spla.LinearOperator(Kr.shape, matvec=solve, dtype=float)
    lam, vec = spla.eigsh(Kr, k=k, M=Mr, sigma=shift, which="LM", OPinv=OPinv)
    order = np.argsort(lam)
    lam, vec = lam[order], vec[:, order]
    freqs = np.sqrt(np.maximum(lam, 0)) / (2 * np.pi)
    labels, parts = [], []
    for i in range(k):
        lab, comp = _label_mode(model, T @ vec[:, i], design.blade.radius)
        labels.append(lab)
        parts.append(comp)
    if (lam < 0).any():
        warnings.append("negative eigenvalue: unstable (spin softening > stiffness)")
    return dict(rpm=rpm, freq_hz=freqs.tolist(),
                per_rev=(freqs / (rpm / 60)).tolist() if rpm > 0 else None,
                labels=labels, participation=parts, n_dof=int(model.ndof),
                n_tets=int(len(model.tets)), mass=model.total_mass(),
                timings=dict(model.timings, eig=time.perf_counter() - t1,
                             total=time.perf_counter() - t0),
                warnings=warnings + sm.warnings)


# =========================================================================== test helpers


def box_mesh(L: float, b: float, t: float, nx: int, ny: int, nz: int):
    """Structured tet mesh of [0,L]×[−b/2,b/2]×[−t/2,t/2] (points, tets)."""
    from skfem import MeshTet
    m = MeshTet.init_tensor(np.linspace(0, L, nx + 1), np.linspace(-b / 2, b / 2, ny + 1),
                            np.linspace(-t / 2, t / 2, nz + 1))
    return m.p.T.copy(), m.t.T.copy()


def model_from_arrays(points, tets, material: str = None, C=None, rho=None,
                      orientation=None) -> FEModel:
    ne = len(tets)
    if material is not None:
        mat = get_material(material)
        C = mat.stiffness_matrix(orientation)
        rho = mat.rho
    C = np.broadcast_to(np.asarray(C, float), (ne, 6, 6)).copy()
    rho = np.broadcast_to(np.asarray(rho, float), (ne,)).copy()
    return FEModel(np.asarray(points, float), np.asarray(tets, int), C, rho)


def isotropic_C(E: float, nu: float) -> np.ndarray:
    lam = E * nu / ((1 + nu) * (1 - 2 * nu))
    mu = E / (2 * (1 + nu))
    C = np.zeros((6, 6))
    C[:3, :3] = lam
    C[np.arange(3), np.arange(3)] += 2 * mu
    C[3:, 3:] = np.eye(3) * mu
    return C
