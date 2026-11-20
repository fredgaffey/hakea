"""Voxel design domain for flexure topology optimisation (numpy only).

Domain: an axis-aligned box in hub frame H (blade 1) from the hub face to the blade-root
fitting, discretised by a structured grid of nx×ny×nz trilinear hexahedra (sizes
hx, hy, hz; near-cubic). Element (i, j, k) has its lower corner at
origin + (i hx, j hy, k hz); node (i, j, k) likewise. Arrays of element fields have shape
(nx, ny, nz); nodal vector fields (3, nx+1, ny+1, nz+1).

Regions (element masks):
  hub    first element layer(s) inside the hub radius: passive solid, its x = x0 face is
         clamped (the hub interface).
  plate  element layers outboard of the outermost FACT member end (rotor.py's master
         node radius): passive stiff "load-introduction plate" = blade-root fitting.
         Unit wrenches at the hinge point are distributed over its nodes by the
         rigid-body least-squares operator N = G (GᵀG)⁻¹, and the plate motion is read
         back as Nᵀu (exact rigid-plate RBE2 equivalent if the plate is rigid).
  design everything else inside the envelope; outside the envelope = passive void.

Element stiffness: 2×2×2 Gauss, anisotropic Voigt C (11,22,33,23,13,12, engineering
shear) from ``Material.stiffness_matrix(orientation)``; local node n = 4a + 2b + c for the
corner offset (a, b, c) ∈ {0,1}³; local dof 3n + component.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .. import fact
from ..materials import get_material, rotation_matrix
from ..schema import Design, FlexureMember, PrintOrientation

OFFSETS = [(a, b, c) for a in (0, 1) for b in (0, 1) for c in (0, 1)]   # local node n = 4a+2b+c


# =========================================================================== element matrices


def _shape_grads(xi: np.ndarray, h) -> np.ndarray:
    """dN/dx (8,3) at reference point xi ∈ [0,1]³ for a box element of size h."""
    g = np.zeros((8, 3))
    for n, (a, b, c) in enumerate(OFFSETS):
        fx = xi[0] if a else 1 - xi[0]
        fy = xi[1] if b else 1 - xi[1]
        fz = xi[2] if c else 1 - xi[2]
        dx = 1.0 if a else -1.0
        dy = 1.0 if b else -1.0
        dz = 1.0 if c else -1.0
        g[n] = [dx * fy * fz / h[0], fx * dy * fz / h[1], fx * fy * dz / h[2]]
    return g


def B_matrix(xi, h) -> np.ndarray:
    """Voigt strain-displacement (6,24): [ε11, ε22, ε33, γ23, γ13, γ12]."""
    g = _shape_grads(np.asarray(xi, float), h)
    B = np.zeros((6, 8, 3))
    B[0, :, 0] = g[:, 0]
    B[1, :, 1] = g[:, 1]
    B[2, :, 2] = g[:, 2]
    B[3, :, 1] = g[:, 2]
    B[3, :, 2] = g[:, 1]
    B[4, :, 0] = g[:, 2]
    B[4, :, 2] = g[:, 0]
    B[5, :, 0] = g[:, 1]
    B[5, :, 1] = g[:, 0]
    return B.reshape(6, 24)


def _bubble_B(xi, h) -> np.ndarray:
    """Strain of Wilson's incompatible modes (6,9): b_d = 4ξ_d(1−ξ_d) per direction d, for each
    displacement component c (internal dof 3d + c)."""
    db = [(4.0 - 8.0 * xi[d]) / h[d] for d in range(3)]
    g = np.zeros((3, 3))                      # g[d] = gradient of bubble d
    for d in range(3):
        g[d, d] = db[d]
    B = np.zeros((6, 3, 3))
    B[0, :, 0] = g[:, 0]
    B[1, :, 1] = g[:, 1]
    B[2, :, 2] = g[:, 2]
    B[3, :, 1] = g[:, 2]
    B[3, :, 2] = g[:, 1]
    B[4, :, 0] = g[:, 2]
    B[4, :, 2] = g[:, 0]
    B[5, :, 0] = g[:, 1]
    B[5, :, 1] = g[:, 0]
    return B.reshape(6, 9)


def hex_stiffness(C: np.ndarray, h, incompatible: bool = True) -> np.ndarray:
    """24×24 trilinear hex stiffness, 2×2×2 Gauss. incompatible=True adds Wilson's nine
    incompatible bubble modes, statically condensed (QM6 / Wilson-Taylor; exact patch test on
    box elements): removes the parasitic shear locking of Q1 in bending, which matters for
    thin flexure plates only 2–4 voxels thick. Centroid strains are unaffected (bubble
    gradients vanish there)."""
    gp = 0.5 + np.array([-0.5, 0.5]) / np.sqrt(3.0)
    vol = h[0] * h[1] * h[2]
    K = np.zeros((24, 24))
    Kua = np.zeros((24, 9))
    Kaa = np.zeros((9, 9))
    for x in gp:
        for y in gp:
            for z in gp:
                B = B_matrix((x, y, z), h)
                w = vol / 8.0
                K += B.T @ C @ B * w
                if incompatible:
                    Bb = _bubble_B((x, y, z), h)
                    Kua += B.T @ C @ Bb * w
                    Kaa += Bb.T @ C @ Bb * w
    if incompatible:
        K = K - Kua @ np.linalg.solve(Kaa, Kua.T)
    return 0.5 * (K + K.T)


def tsai_hill_form(mat_name: str, orientation: PrintOrientation | None, sf: float = 1.0) -> np.ndarray:
    """6×6 M with FI = σ_Hᵀ M σ_H (Tsai-Hill, material axes 1 raster, 2 in-layer, 3 interlayer,
    tensile strengths Xt, Yt, Zt and shear S), σ_H Voigt in frame H, divided by sf²."""
    from ..materials import _voigt_stress_transform
    m = get_material(mat_name)
    R = rotation_matrix(orientation)
    T = _voigt_stress_transform(R)          # σ_H = T σ_loc
    Ti = np.linalg.inv(T)
    X2, Y2, Z2 = m.Xt ** -2, m.Yt ** -2, m.Zt ** -2
    F = np.zeros((6, 6))
    F[0, 0], F[1, 1], F[2, 2] = X2, Y2, Z2
    F[0, 1] = F[1, 0] = -0.5 * (X2 + Y2 - Z2)
    F[0, 2] = F[2, 0] = -0.5 * (X2 + Z2 - Y2)
    F[1, 2] = F[2, 1] = -0.5 * (Y2 + Z2 - X2)
    F[3, 3] = F[4, 4] = F[5, 5] = m.S ** -2
    return Ti.T @ F @ Ti / sf ** 2


def equiv_strain_form(nu: float = 0.5) -> np.ndarray:
    """6×6 M with ε_eq² = εᵀ M ε: von Mises equivalent strain of the (engineering-shear Voigt)
    strain, ε_eq = 1/(1+ν') √(½Σ(εi−εj)² + ¾Σγ²). Equals the axial strain in uniaxial
    tension of a material with Poisson ratio ν' (0.5 for TPU); γ·√3/(2(1+ν')) in pure shear.
    Volumetric strain is ignored (elastomers; small for PA6-CF)."""
    M = np.zeros((6, 6))
    for i, j in ((0, 1), (1, 2), (2, 0)):
        v = np.zeros(6)
        v[i], v[j] = 1.0, -1.0
        M += 0.5 * np.outer(v, v)
    M[3, 3] = M[4, 4] = M[5, 5] = 0.75
    return M / (1.0 + nu) ** 2


def interlayer_form(mat_name: str, orientation: PrintOrientation | None, sf: float = 1.0) -> np.ndarray:
    """6×6 Q with FI_il = σ_Hᵀ Q σ_H = (σ_n/Zt)² + (τ/S)² on the layer plane (normal = build
    direction b): traction t = σ b, σ_n = b·t, τ² = |t|² − σ_n²; divided by sf². Compression
    counts like tension (smooth, conservative)."""
    m = get_material(mat_name)
    b = rotation_matrix(orientation)[:, 2]
    idx = [(0, 0), (1, 1), (2, 2), (1, 2), (0, 2), (0, 1)]
    A = np.zeros((3, 6))                        # traction t = A σ_voigt
    for I, (i, j) in enumerate(idx):
        A[i, I] += b[j]
        if i != j:
            A[j, I] += b[i]
    n = b @ A                                   # σ_n = n · σ_voigt
    Q = np.outer(n, n) / m.Zt ** 2 + (A.T @ A - np.outer(n, n)) / m.S ** 2
    return Q / sf ** 2


# =========================================================================== domain


@dataclass
class Domain:
    origin: np.ndarray              # (3,) lower corner
    h: np.ndarray                   # (3,) element size
    shape: tuple[int, int, int]     # elements
    hinge_point: np.ndarray
    hinge_axis: np.ndarray
    x_hub: float
    x_root: float
    hub: np.ndarray                 # (nx,ny,nz) bool passive solid, clamped face
    plate: np.ndarray               # bool passive stiff plate
    design: np.ndarray              # bool design elements
    seed_rho: np.ndarray            # (nx,ny,nz) FACT voxelisation (0/1)
    seed_phi: np.ndarray            # stiff-phase fraction of the seed (0/1)
    mg_levels: int = 1
    Lc: float = 0.008
    envelope: str = "box"
    info: dict = field(default_factory=dict)

    @property
    def n_elem(self) -> int:
        return int(np.prod(self.shape))

    @property
    def node_shape(self) -> tuple[int, int, int]:
        return tuple(s + 1 for s in self.shape)

    def elem_centers(self) -> np.ndarray:
        """(nx,ny,nz,3)."""
        ax = [self.origin[d] + (np.arange(self.shape[d]) + 0.5) * self.h[d] for d in range(3)]
        return np.stack(np.meshgrid(*ax, indexing="ij"), -1)

    def node_coords(self) -> np.ndarray:
        """(nx+1,ny+1,nz+1,3)."""
        ax = [self.origin[d] + np.arange(self.shape[d] + 1) * self.h[d] for d in range(3)]
        return np.stack(np.meshgrid(*ax, indexing="ij"), -1)

    def fixed_nodes(self) -> np.ndarray:
        """Node mask clamped: the hub face x = origin_x."""
        m = np.zeros(self.node_shape, bool)
        m[0] = True
        return m

    def plate_nodes(self) -> np.ndarray:
        m = np.zeros(self.node_shape, bool)
        for a, b, c in OFFSETS:
            nx, ny, nz = self.shape
            m[a:a + nx, b:b + ny, c:c + nz] |= self.plate
        return m

    def rigid_load_operator(self) -> np.ndarray:
        """N (6, 3, X, Y, Z): nodal forces on the plate statically equivalent to unit
        wrenches [Fx,Fy,Fz,Mx,My,Mz] at the hinge point, N = G (GᵀG)⁻¹ with G the
        rigid-body modes of the plate nodes about the hinge point; NᵀG = I so Nᵀu is the
        least-squares rigid motion [u; θ] of the plate referred to the hinge point."""
        X = self.node_coords()
        pm = self.plate_nodes()
        r = X[pm] - self.hinge_point[None, :]            # (np,3)
        npn = len(r)
        G = np.zeros((npn, 3, 6))
        G[:, :, :3] = np.eye(3)[None]
        # u = θ × r = −r × θ  →  −skew(r)
        G[:, 0, 4], G[:, 0, 5] = r[:, 2], -r[:, 1]
        G[:, 1, 3], G[:, 1, 5] = -r[:, 2], r[:, 0]
        G[:, 2, 3], G[:, 2, 4] = r[:, 1], -r[:, 0]
        Gm = G.reshape(-1, 6)
        Nm = Gm @ np.linalg.inv(Gm.T @ Gm)               # (3np, 6)
        N = np.zeros((6, 3) + self.node_shape)
        Nn = Nm.reshape(npn, 3, 6)
        for k in range(6):
            for c in range(3):
                N[k, c][pm] = Nn[:, c, k]
        return N

    def hinge_frame(self) -> np.ndarray:
        """3×3 columns (a, x', n): hinge axis, ⟂ direction closest to +x, a × x'."""
        a = self.hinge_axis
        xp = np.array([1.0, 0, 0]) - a * a[0]
        xp /= np.linalg.norm(xp)
        return np.column_stack([a, xp, np.cross(a, xp)])


# --------------------------------------------------------------------------- voxelisation


def _member_frame(m: FlexureMember):
    p0, p1 = np.asarray(m.p0, float), np.asarray(m.p1, float)
    L = float(np.linalg.norm(p1 - p0))
    t = (p1 - p0) / L
    w = np.asarray(m.width_dir, float)
    w = w - t * (t @ w)
    if np.linalg.norm(w) < 1e-9:
        w = np.cross([0, 0, 1.0], t) if abs(t[2]) < 0.9 else np.cross([0, 1.0, 0], t)
    w /= np.linalg.norm(w)
    return p0, p1, L, t, w, np.cross(t, w)


def _is_stiff(material: Optional[str]) -> bool:
    return material is not None and get_material(material).E1 > 1.0e9


def _member_corners(m: FlexureMember) -> np.ndarray:
    p0, p1, L, t, w, n = _member_frame(m)
    out = []
    for c, th in ((p0, m.thickness), (p1, m.thickness * m.taper)):
        wid = m.width if m.shape == "rect" else th
        out += [c + sw * wid / 2 * w + sn * th / 2 * n for sw in (-1, 1) for sn in (-1, 1)]
    return np.array(out)


def voxelise_members(members: list[FlexureMember], X: np.ndarray, x_hub: float, x_root: float,
                     connectors: bool = True, gap: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """(rho, phi) 0/1 element fields from member solids at element centres X (...,3).

    phi = 1 where the voxel is a stiff material (E1 > 1 GPa: pa6_cf, CF rod, steel → the
    stiff printable phase). Layups: solid; core_skin (skin of skin_thickness); rod_insert
    (rods of insert_diameter along the member at width fractions insert_positions).
    connectors: extrude each member end face along ∓x to the hub face / blade-root plate as
    stiff phase (the rigid spokes/connectors of rotor.py's beam model); a connector stays
    `gap` clear of the other members so it does not weld them together."""
    rho = np.zeros(X.shape[:-1], bool)
    phi = np.zeros(X.shape[:-1], bool)
    inflated = []
    for m in members:
        p0, p1, L, t, w, n = _member_frame(m)
        d = X - p0
        s_ = d @ t
        th = max(m.thickness, m.thickness * m.taper)
        wd = m.width if m.shape == "rect" else th
        inflated.append((np.abs(d @ w) <= wd / 2 + gap) & (np.abs(d @ n) <= th / 2 + gap)
                        & (s_ >= -gap) & (s_ <= L + gap))
    for im, m in enumerate(members):
        others = np.zeros_like(rho)
        for jm, inf in enumerate(inflated):
            if jm != im:
                others |= inf
        p0, p1, L, t, w, n = _member_frame(m)
        d = X - p0
        s = d @ t
        dw, dn = d @ w, d @ n
        xi = np.clip(s / L, 0, 1)
        th = m.thickness * (1 + (m.taper - 1) * xi)
        if m.shape == "circle":
            inside = (dw ** 2 + dn ** 2 <= (th / 2) ** 2)
            hw = th / 2
        else:
            hw = m.width / 2
            inside = (np.abs(dw) <= hw) & (np.abs(dn) <= th / 2)
        inside &= (s >= 0) & (s <= L)
        lay = m.layup
        if m.kind == "rigid":
            stiff = inside
        elif lay.kind == "core_skin" and lay.skin_material:
            core = (np.abs(dw) <= hw - lay.skin_thickness) & (np.abs(dn) <= th / 2 - lay.skin_thickness)
            stiff = inside & ((~core & _is_stiff(lay.skin_material)) | (core & _is_stiff(lay.material)))
        elif lay.kind == "rod_insert" and lay.insert_material:
            stiff = inside & _is_stiff(lay.material)
            for f in lay.insert_positions:
                yw = (f - 0.5) * 2 * hw
                rr = np.maximum(lay.insert_diameter / 2, 0.0)
                stiff |= inside & ((dw - yw) ** 2 + dn ** 2 <= rr ** 2) & _is_stiff(lay.insert_material)
        else:
            stiff = inside & _is_stiff(lay.material)
        rho |= inside
        phi |= stiff
        if connectors and m.kind != "rigid" and abs(t[0]) > 0.15:
            for end, c, sgn, lim in ((0, p0, -1.0, x_hub), (1, p1, 1.0, x_root)):
                thc = m.thickness * (1.0 if end == 0 else m.taper)
                # c_elem = q + sgn·s·x̂, q on the end plane (normal t) → s = sgn (X − c)·t / t_x
                sx = sgn * ((X - c) @ t) / t[0]
                q = X - (sgn * sx)[..., None] * np.array([1.0, 0, 0])
                dq = q - c
                if m.shape == "circle":
                    ins = ((dq @ w) ** 2 + (dq @ n) ** 2 <= (thc / 2) ** 2)
                else:
                    ins = (np.abs(dq @ w) <= m.width / 2) & (np.abs(dq @ n) <= thc / 2)
                ins &= sx > 0
                ins &= (X[..., 0] >= lim) if sgn < 0 else (X[..., 0] <= lim)
                ins &= ~others
                rho |= ins
                phi |= ins
    return rho.astype(float), (phi & rho).astype(float)


# --------------------------------------------------------------------------- build


def _choose_grid(lengths: np.ndarray, n_target: int, max_coarse_dofs: int = 4000):
    """Element counts (multiples of 2^(L−1)) and MG level count L."""
    h0 = (np.prod(lengths) / n_target) ** (1 / 3)
    best = None
    for L in range(1, 8):
        m = 2 ** (L - 1)
        n = np.maximum(np.round(lengths / h0 / m), 2).astype(int) * m
        nc = n // m
        if (nc < 2).any():
            break
        dofs = 3 * np.prod(nc + 1)
        best = (n, L)
        if dofs <= max_coarse_dofs:
            break
    return best


def build_domain(design: Design, n_elem: int = 100_000, h: Optional[float] = None,
                 envelope: str = "box", margin: Optional[float] = None,
                 plate_layers: int = 2, hub_layers: int = 1,
                 members: Optional[list[FlexureMember]] = None) -> Domain:
    """Design domain for blade 1's flexure.

    x: hub face (flexure.hub_radius) − hub_layers elements … outermost member end
    (rotor.py master radius) + plate_layers elements. y, z: bounding box of the FACT
    members (incl. end faces; plus the twisted root airfoil section for envelope
    "airfoil"; y at least ±root chord/4 for "box"), + margin.
    envelope: "box" (whole box designable) or "airfoil" (designable only inside the root
    airfoil section extruded along x, + margin; the plate keeps the full box section so
    FACT-seed connectors still land on it)."""
    f = design.flexure
    members = fact.generate_members(design) if members is None else members
    members = [m for m in members]
    H0, a = fact.hinge_axis(design)
    x_hub = f.hub_radius
    x_root = max(max(m.p0[0], m.p1[0]) for m in members) if members else f.hinge_radius + 0.5 * f.length
    x_root = max(x_root, H0[0] + 1e-4)
    from ..bladegeom import airfoil_coords, resolve_stations, section_polygon_H
    stations = resolve_stations(design.blade)
    st0 = stations[0]
    xy = airfoil_coords(design.blade.airfoil, st0.thickness_frac, n=61)
    poly = section_polygon_H(xy, st0.chord, np.radians(st0.twist_deg + design.op.collective_deg),
                             x_root, design.blade.elastic_axis_chord_frac)
    pts = [poly] if (envelope == "airfoil" or not members) else []
    if members:
        pts.append(np.vstack([_member_corners(m) for m in members]))
    P = np.vstack(pts)
    lo, hi = P.min(0), P.max(0)
    if envelope == "box":
        # room to spread along the chord: at least ±chord/4 in y around the axis
        lo[1] = min(lo[1], -0.25 * st0.chord)
        hi[1] = max(hi[1], 0.25 * st0.chord)
    if margin is None:
        margin = max(0.08 * (x_root - x_hub), 0.0005)
    # rough element size to place hub/plate layers
    ext = np.array([x_root - x_hub, hi[1] - lo[1] + 2 * margin, hi[2] - lo[2] + 2 * margin])
    h_est = h if h is not None else (np.prod(ext) / n_elem) ** (1 / 3)
    lo_b = np.array([x_hub - hub_layers * h_est, lo[1] - margin, lo[2] - margin])
    hi_b = np.array([x_root + plate_layers * h_est, hi[1] + margin, hi[2] + margin])
    lengths = hi_b - lo_b
    if h is not None:
        n_elem = int(np.prod(lengths) / h ** 3)
    n, L = _choose_grid(lengths, n_elem)
    hh = lengths / n
    dom = Domain(origin=lo_b, h=hh, shape=tuple(int(v) for v in n), hinge_point=H0,
                 hinge_axis=a, x_hub=x_hub, x_root=x_root,
                 hub=np.zeros(tuple(n), bool), plate=np.zeros(tuple(n), bool),
                 design=np.zeros(tuple(n), bool), seed_rho=np.zeros(tuple(n)),
                 seed_phi=np.zeros(tuple(n)), mg_levels=L, Lc=max(f.length, 1e-4),
                 envelope=envelope)
    Xc = dom.elem_centers()
    dom.hub = Xc[..., 0] < x_hub
    dom.plate = Xc[..., 0] > x_root
    inside = np.ones(dom.shape, bool)
    if envelope == "airfoil":
        from matplotlib.path import Path as MPath
        yz = poly[:, 1:]
        cen = yz.mean(0)
        # grow polygon by `margin` (radially about its centroid; adequate for a thin section)
        d = yz - cen
        nd = np.linalg.norm(d, axis=1, keepdims=True)
        yz_g = cen + d * (1 + margin / np.maximum(nd, 1e-9))
        inside = MPath(yz_g).contains_points(Xc[..., 1:].reshape(-1, 2)).reshape(dom.shape)
    elif envelope != "box":
        raise ValueError(f"envelope {envelope!r}: box | airfoil")
    dom.design = inside & ~dom.hub & ~dom.plate
    rho, phi = voxelise_members(members, Xc, x_hub, x_root, gap=1.5 * float(hh.max()))
    dom.seed_rho = np.where(dom.design, rho, 0.0)
    dom.seed_phi = np.where(dom.design, phi, 0.0)
    dom.info = dict(n_members=len(members), lengths=lengths.tolist(),
                    seed_volume_fraction=float(dom.seed_rho[dom.design].mean()) if dom.design.any() else 0.0)
    return dom
