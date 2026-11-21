"""2D composite cross-section properties (OWNER: structures).

Section coordinates: (y, z) in the beam element's local frame — y along the element
axis e2, z along e3 (both ⟂ beam axis t_hat = e1). The origin is the beam
*reference line* (where the FE nodes sit). All stiffness/mass quantities are
modulus/density weighted over the materials present.

Integration: cell-centred pixel grid over the shape (square-ish cells, ≥ ~16 cells
across the thinnest direction). Axial modulus per cell = Material.axial_modulus along
t_hat for that cell's print orientation; torsion by the Prandtl stress function
(finite differences, per-cell orthotropic shear moduli G_xy, G_xz, harmonic face
averaging so multi-material sections are handled). Solid single-material rectangles
and circles have a closed-form fast path (Saint-Venant series, orthotropic via an
affine stretch) that the tests check against the grid.

Approximations (documented, intentional):
  * shear centre ≈ Prandtl-stress-function-weighted centroid (exact for doubly
    symmetric sections, a few % of chord off for cambered airfoils);
  * shear areas = κ Σ G dA with κ = 5/6 (rect/polygon) or 0.9 (circle);
  * no warping restraint, no Poisson coupling between materials.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache

import dataclasses
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from scipy import ndimage

from .materials import Material, get_material
from .schema import Layup, PrintOrientation

# --------------------------------------------------------------------------- shapes


@dataclass(frozen=True)
class RectShape:
    """width along local y, thickness along local z, centred at (cy, cz)."""
    width: float
    thickness: float
    cy: float = 0.0
    cz: float = 0.0


@dataclass(frozen=True)
class CircleShape:
    diameter: float
    cy: float = 0.0
    cz: float = 0.0


@dataclass(frozen=True)
class PolygonShape:
    """Closed polygon (N,2) of local (y, z). Optional mid-line for rod placement:
    `chord` and `le_y` describe a chord line from y = le_y (leading edge, chord
    fraction 0) toward −y; `midline` is a tuple of (x/c, z) samples of the camber line."""
    pts: tuple
    chord: float = 0.0
    le_y: float = 0.0
    midline: tuple = ()

    @property
    def array(self) -> np.ndarray:
        return np.asarray(self.pts, float)


def airfoil_shape(xy: np.ndarray, chord: float, ref_chord_frac: float = 0.25) -> PolygonShape:
    """Airfoil (x/c, y/c) → PolygonShape in the *untwisted chord frame* of the section:
    local y = −(x/c − ref)·c (LE toward +y, matching frame H at zero pitch),
    local z = (y/c)·c (thickness / lift direction)."""
    xy = np.asarray(xy, float)
    pts = np.column_stack([-(xy[:, 0] - ref_chord_frac) * chord, xy[:, 1] * chord])
    # camber line samples: average of upper/lower at matching x/c
    n = len(xy)
    i_le = int(np.argmin(xy[:, 0]))
    up = xy[: i_le + 1][::-1]          # LE -> TE
    lo = xy[i_le:]
    xs = np.linspace(0.0, 1.0, 41)
    zu = np.interp(xs, up[:, 0], up[:, 1])
    zl = np.interp(xs, lo[:, 0], lo[:, 1])
    mid = tuple((float(x), float(0.5 * (a + b) * chord)) for x, a, b in zip(xs, zu, zl))
    del n
    return PolygonShape(pts=tuple(map(tuple, pts)), chord=float(chord),
                        le_y=float(ref_chord_frac * chord), midline=mid)


# --------------------------------------------------------------------------- results


@dataclass
class Fibre:
    """Stress-recovery point: local (y, z), axial modulus, material name, orientation."""
    y: float
    z: float
    E: float
    material: str
    orientation: PrintOrientation


@dataclass
class SectionProps:
    EA: float
    EIy: float                 # ∫E (z−zc)² — bending about local y
    EIz: float                 # ∫E (y−yc)²
    EIyz: float                # ∫E (y−yc)(z−zc)
    GJ: float
    GAy: float
    GAz: float
    mass_per_len: float
    Ip_mass: float             # ∫ρ (y²+z²) about the reference line
    Iyy_mass: float            # ∫ρ z²  (about reference)
    Izz_mass: float            # ∫ρ y²
    Iyz_mass: float            # ∫ρ y z
    centroid: tuple[float, float]       # modulus-weighted (tension centre)
    shear_centre: tuple[float, float]
    mass_centre: tuple[float, float]
    area: float
    tau_per_twist: float = 0.0          # max torsional shear stress per unit twist rate, Pa/(rad/m)
    fibres: list[Fibre] = field(default_factory=list)
    kappa_shear: float = 5.0 / 6.0
    method: str = "grid"

    # elastic offset relative to reference axis (alias)
    @property
    def elastic_offset(self) -> tuple[float, float]:
        return self.centroid

    @property
    def mass_offset(self) -> tuple[float, float]:
        return self.mass_centre

    def stiffness6(self) -> np.ndarray:
        """6×6 section stiffness at the reference axis.

        Generalised strains [ε, γ2, γ3, κ1, κ2, κ3] ↔ forces [N, V2, V3, T, M2, M3],
        with axial strain at (y,z): ε0 + z κ2 − y κ3, M2 = ∫σ z, M3 = −∫σ y.
        """
        yc, zc = self.centroid
        ys, zs = self.shear_centre
        # axial-bending about the tension centre, shifted to the reference axis
        # [ε, κ2, κ3] basis, a = [1, z, −y] about the centroid: off-diagonal ∫E z(−y) = −EIyz
        Kc = np.array([[self.EA, 0.0, 0.0], [0.0, self.EIy, -self.EIyz], [0.0, -self.EIyz, self.EIz]])
        T = np.array([[1.0, zc, -yc], [0, 1, 0], [0, 0, 1]])   # ε_c = ε0 + zc κ2 − yc κ3
        Kab = T.T @ Kc @ T
        Ts = np.array([[1.0, 0, -zs], [0, 1, ys], [0, 0, 1]])
        Ksh = Ts.T @ np.diag([self.GAy, self.GAz, self.GJ]) @ Ts
        S = np.zeros((6, 6))
        ia = [0, 4, 5]
        ish = [1, 2, 3]
        S[np.ix_(ia, ia)] = Kab
        S[np.ix_(ish, ish)] = Ksh
        return S

    def mass_moments(self) -> tuple[float, np.ndarray, np.ndarray]:
        """(m, first moment [m·ym, m·zm], second moments [[∫ρy², ∫ρyz],[∫ρyz, ∫ρz²]])."""
        m = self.mass_per_len
        s1 = np.array([m * self.mass_centre[0], m * self.mass_centre[1]])
        s2 = np.array([[self.Izz_mass, self.Iyz_mass], [self.Iyz_mass, self.Iyy_mass]])
        return m, s1, s2

    def scaled(self, k: complex | float) -> "SectionProps":
        """Copy with all elastic stiffnesses multiplied by k (real part used)."""
        import copy
        o = copy.copy(self)
        k = float(np.real(k))
        for a in ("EA", "EIy", "EIz", "EIyz", "GJ", "GAy", "GAz"):
            setattr(o, a, getattr(self, a) * k)
        return o


# --------------------------------------------------------------------------- helpers


def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def default_axes(t_hat) -> tuple[np.ndarray, np.ndarray]:
    """Local section axes (e2, e3) in H for beam axis t_hat when none are given."""
    t = _unit(t_hat)
    ref = np.array([0.0, 0.0, 1.0]) if abs(t[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e3 = ref - t * (t @ ref)
    e3 = _unit(e3)
    e2 = np.cross(e3, t)
    return e2, e3


def rod_orientation(t_hat) -> PrintOrientation:
    """Fibre/raster along the rod (beam) axis."""
    t = _unit(t_hat)
    e2, e3 = default_axes(t)
    return PrintOrientation(build_dir=tuple(e3), raster_dir=tuple(t))


@dataclass(frozen=True)
class _MatCell:
    name: str
    E: float
    Gy: float      # shear modulus in plane (t, e2) → τ_xy
    Gz: float      # shear modulus in plane (t, e3) → τ_xz
    rho: float
    orientation: PrintOrientation


def _mat_cell(name: str, orientation: PrintOrientation, t, e2, e3) -> _MatCell:
    m = get_material(name)
    return _MatCell(name, m.axial_modulus(orientation, t), m.shear_modulus(orientation, t, e2),
                    m.shear_modulus(orientation, t, e3), m.rho, orientation)


def _points_in_polygon(px: np.ndarray, py: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Even-odd ray casting, vectorised over points."""
    inside = np.zeros(px.shape, bool)
    x0, y0 = poly[:, 0], poly[:, 1]
    x1, y1 = np.roll(x0, -1), np.roll(y0, -1)
    for a, b, c, d in zip(x0, y0, x1, y1):
        if b == d:
            continue
        cond = ((b > py) != (d > py))
        xint = a + (py - b) * (c - a) / (d - b)
        inside ^= cond & (px < xint)
    return inside


def _beta_rect(b: float, t: float, nterms: int = 50) -> float:
    """Saint-Venant torsion constant factor: J = β b t³ (b ≥ t not required)."""
    if t > b:
        b, t = t, b
    n = np.arange(1, 2 * nterms, 2, dtype=float)
    s = np.sum(np.tanh(n * np.pi * b / (2 * t)) / n**5)
    return (1.0 / 3.0) * (1.0 - 192.0 / np.pi**5 * (t / b) * s)


def rect_torsion_constant(b: float, t: float) -> float:
    """J for an isotropic b×t rectangle."""
    return _beta_rect(b, t) * max(b, t) * min(b, t) ** 3


# --------------------------------------------------------------------------- grid core


def _rasterize(shape, h_target: float | None, n_thin: int, max_cells: int):
    if isinstance(shape, RectShape):
        w, t = shape.width, shape.thickness
        h = h_target or min(w, t) / n_thin
        h = max(h, np.sqrt(w * t / max_cells))
        ny, nz = max(2, int(round(w / h))), max(2, int(round(t / h)))
        dy, dz = w / ny, t / nz
        y = shape.cy - w / 2 + dy * (np.arange(ny) + 0.5)
        z = shape.cz - t / 2 + dz * (np.arange(nz) + 0.5)
        Y, Z = np.meshgrid(y, z, indexing="ij")
        return Y, Z, np.ones_like(Y, bool), dy, dz
    if isinstance(shape, CircleShape):
        d = shape.diameter
        h = h_target or d / max(n_thin, 40)
        n = max(8, int(round(d / h)))
        dy = d / n
        y = shape.cy - d / 2 + dy * (np.arange(n) + 0.5)
        z = shape.cz - d / 2 + dy * (np.arange(n) + 0.5)
        Y, Z = np.meshgrid(y, z, indexing="ij")
        mask = (Y - shape.cy) ** 2 + (Z - shape.cz) ** 2 <= (d / 2) ** 2
        return Y, Z, mask, dy, dy
    poly = shape.array
    lo, hi = poly.min(0), poly.max(0)
    ext = hi - lo
    # thinnest direction estimate: area / max extent
    area = 0.5 * abs(np.dot(poly[:, 0], np.roll(poly[:, 1], -1)) - np.dot(poly[:, 1], np.roll(poly[:, 0], -1)))
    t_eff = area / max(ext.max(), 1e-12) * 1.5
    h = h_target or t_eff / n_thin
    h = max(h, np.sqrt(ext[0] * ext[1] / max_cells))
    ny, nz = max(4, int(np.ceil(ext[0] / h)) + 1), max(4, int(np.ceil(ext[1] / h)) + 1)
    dy, dz = ext[0] / (ny - 1), ext[1] / (nz - 1)
    y = lo[0] + dy * np.arange(ny)
    z = lo[1] + dz * np.arange(nz)
    # cell centres at half offsets inside bbox
    y = y[:-1] + dy / 2
    z = z[:-1] + dz / 2
    Y, Z = np.meshgrid(y, z, indexing="ij")
    mask = _points_in_polygon(Y, Z, poly)
    return Y, Z, mask, dy, dz


def _prandtl(mask: np.ndarray, Gy: np.ndarray, Gz: np.ndarray, dy: float, dz: float):
    """Solve −∂y(1/Gxz ∂yφ) − ∂z(1/Gxy ∂zφ) = 2 (unit twist rate), φ = 0 outside.

    Gy = G_xy (τ_xy = ∂φ/∂z), Gz = G_xz (τ_xz = −∂φ/∂y). Returns (GJ, phi, tau_max)."""
    ny, nz = mask.shape
    idx = -np.ones(mask.shape, int)
    idx[mask] = np.arange(mask.sum())
    n = int(mask.sum())
    if n == 0:
        return 0.0, np.zeros(mask.shape), 0.0
    cy = np.where(mask, 1.0 / np.where(mask, Gz, 1.0), 0.0)    # coefficient for y-derivatives: 1/G_xz
    cz = np.where(mask, 1.0 / np.where(mask, Gy, 1.0), 0.0)    # z-derivatives: 1/G_xy
    rows, cols, vals = [], [], []
    diag = np.zeros(n)
    I, J = np.nonzero(mask)
    p = idx[I, J]
    for (di, dj, c, hh) in ((1, 0, cy, dy), (-1, 0, cy, dy), (0, 1, cz, dz), (0, -1, cz, dz)):
        I2, J2 = I + di, J + dj
        inb = (I2 >= 0) & (I2 < ny) & (J2 >= 0) & (J2 < nz)
        nb_in = np.zeros_like(inb)
        nb_in[inb] = mask[I2[inb], J2[inb]]
        c0 = c[I, J]
        # c = 1/G acts as a conductivity; two half cells in series → harmonic mean.
        c1 = np.ones_like(c0)
        c1[nb_in] = c[I2[nb_in], J2[nb_in]]
        face = np.where(nb_in, 2.0 * c0 * c1 / (c0 + c1), 2.0 * c0)  # wall at half cell → 2c/h²
        coef = face / hh**2
        diag += coef
        rows.append(p[nb_in])
        cols.append(idx[I2[nb_in], J2[nb_in]])
        vals.append(-coef[nb_in])
    rows.append(np.arange(n))
    cols.append(np.arange(n))
    vals.append(diag)
    A = sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(n, n))
    phi_v = spla.spsolve(A.tocsc(), np.full(n, 2.0))
    phi = np.zeros(mask.shape)
    phi[mask] = phi_v
    GJ = 2.0 * phi_v.sum() * dy * dz
    # shear stress magnitude (unit twist): τ_xy = ∂φ/∂z, τ_xz = −∂φ/∂y, φ=0 at walls
    pad = np.pad(phi, 1)
    dphidy = np.maximum(np.abs(pad[2:, 1:-1] - phi), np.abs(phi - pad[:-2, 1:-1])) / dy
    dphidz = np.maximum(np.abs(pad[1:-1, 2:] - phi), np.abs(phi - pad[1:-1, :-2])) / dz
    # boundary half-cell gradient ≈ 2φ/h; interior one-sided differences are fine
    tau = np.sqrt(dphidy**2 + dphidz**2) * mask
    tau_max = float(tau.max()) if tau.size else 0.0
    return float(GJ), phi, tau_max


def _boundary_fibres(Y, Z, mask, matid, mats, n_max=48) -> list[Fibre]:
    """Sample extreme fibres: boundary cells of each material (subsampled)."""
    out = []
    er = ndimage.binary_erosion(mask, border_value=0)
    for k, mc in enumerate(mats):
        sel = (matid == k) & mask
        if not sel.any():
            continue
        bd = sel & ~ndimage.binary_erosion(sel, border_value=0) | (sel & ~er)
        ii, jj = np.nonzero(bd)
        if len(ii) == 0:
            ii, jj = np.nonzero(sel)
        # pick points extreme in several directions
        ys, zs = Y[ii, jj], Z[ii, jj]
        picks = set()
        for ang in np.linspace(0, 2 * np.pi, 16, endpoint=False):
            d = ys * np.cos(ang) + zs * np.sin(ang)
            picks.add(int(np.argmax(d)))
        if len(ii) > len(picks):
            step = max(1, len(ii) // max(1, n_max - len(picks)))
            picks.update(range(0, len(ii), step))
        for q in sorted(picks):
            out.append(Fibre(float(ys[q]), float(zs[q]), mc.E, mc.name, mc.orientation))
    return out


# --------------------------------------------------------------------------- public API


def section_properties(shape, layup: Layup, orientation: PrintOrientation | None,
                       t_hat=(1.0, 0.0, 0.0), axes=None, *, n_thin: int = 16,
                       max_cells: int = 12000, h: float | None = None,
                       force_grid: bool = False) -> SectionProps:
    """Composite section properties.

    shape: RectShape | CircleShape | PolygonShape in local (y,z).
    layup: solid / core_skin / rod_insert (Layup schema).
    orientation: print orientation (frame H) of printed materials.
    t_hat: beam axis in H; axes: (e2, e3) local section axes in H (default: default_axes).
    """
    orientation = orientation or PrintOrientation()
    t = _unit(t_hat)
    e2, e3 = axes if axes is not None else default_axes(t)
    e2, e3 = _unit(e2), _unit(e3)
    key = (shape, layup.model_dump_json(), orientation.model_dump_json(),
           tuple(np.round(t, 12)), tuple(np.round(e2, 12)), tuple(np.round(e3, 12)),
           n_thin, max_cells, h, force_grid)
    return _section_cached(key)


@lru_cache(maxsize=4096)
def _section_cached(key) -> SectionProps:
    shape, layup_json, orient_json, t, e2, e3, n_thin, max_cells, h, force_grid = key
    layup = Layup.model_validate_json(layup_json)
    orientation = PrintOrientation.model_validate_json(orient_json)
    t, e2, e3 = np.array(t), np.array(e2), np.array(e3)
    base = _mat_cell(layup.material, orientation, t, e2, e3)
    if layup.kind == "solid" and not force_grid and isinstance(shape, (RectShape, CircleShape)):
        return _closed_form(shape, base)
    return _grid_section(shape, layup, orientation, t, e2, e3, base, n_thin, max_cells, h)


def _closed_form(shape, mc: _MatCell) -> SectionProps:
    if isinstance(shape, RectShape):
        b, tt = shape.width, shape.thickness
        A = b * tt
        Iy, Iz = b * tt**3 / 12, tt * b**3 / 12
        # orthotropic torsion via affine stretch of y by sqrt(Gxz/Gxy)
        bp = b * np.sqrt(mc.Gz / mc.Gy)
        J = _beta_rect(bp, tt) * bp * tt**3 * (b / bp) if bp >= tt else None
        if J is None:
            # thick in z: stretch z instead
            tp = tt * np.sqrt(mc.Gy / mc.Gz)
            J = _beta_rect(b, tp) * tp * b**3 * (tt / tp)
            GJ = mc.Gz * J
        else:
            GJ = mc.Gy * J
        kap = 5.0 / 6.0
        tau = max(mc.Gy, mc.Gz) * min(b, tt)    # thin-strip bound τ = G θ' t
        cy, cz = shape.cy, shape.cz
        fib = [Fibre(cy + sy * b / 2, cz + sz * tt / 2, mc.E, mc.name, mc.orientation)
               for sy in (-1, 0, 1) for sz in (-1, 0, 1) if (sy, sz) != (0, 0)]
        Ixx = Iz + A * cy**2      # ∫y²
        Izz_ = Iy + A * cz**2     # ∫z²
        return SectionProps(EA=mc.E * A, EIy=mc.E * Iy, EIz=mc.E * Iz, EIyz=0.0, GJ=GJ,
                            GAy=kap * mc.Gy * A, GAz=kap * mc.Gz * A,
                            mass_per_len=mc.rho * A, Ip_mass=mc.rho * (Ixx + Izz_),
                            Iyy_mass=mc.rho * Izz_, Izz_mass=mc.rho * Ixx,
                            Iyz_mass=mc.rho * A * cy * cz,
                            centroid=(cy, cz), shear_centre=(cy, cz), mass_centre=(cy, cz),
                            area=A, tau_per_twist=tau, fibres=fib, kappa_shear=kap,
                            method="closed_form")
    d = shape.diameter
    A = np.pi * d**2 / 4
    I = np.pi * d**4 / 64
    G = np.sqrt(mc.Gy * mc.Gz)
    cy, cz = shape.cy, shape.cz
    fib = [Fibre(cy + d / 2 * np.cos(a), cz + d / 2 * np.sin(a), mc.E, mc.name, mc.orientation)
           for a in np.linspace(0, 2 * np.pi, 8, endpoint=False)]
    return SectionProps(EA=mc.E * A, EIy=mc.E * I, EIz=mc.E * I, EIyz=0.0, GJ=G * 2 * I,
                        GAy=0.9 * mc.Gy * A, GAz=0.9 * mc.Gz * A, mass_per_len=mc.rho * A,
                        Ip_mass=mc.rho * (2 * I + A * (cy**2 + cz**2)),
                        Iyy_mass=mc.rho * (I + A * cz**2), Izz_mass=mc.rho * (I + A * cy**2),
                        Iyz_mass=mc.rho * A * cy * cz, centroid=(cy, cz), shear_centre=(cy, cz),
                        mass_centre=(cy, cz), area=A, tau_per_twist=G * d / 2, fibres=fib,
                        kappa_shear=0.9, method="closed_form")


def _grid_section(shape, layup, orientation, t, e2, e3, base, n_thin, max_cells, h) -> SectionProps:
    Y, Z, mask, dy, dz = _rasterize(shape, h, n_thin, max_cells)
    if layup.kind == "core_skin" and layup.core_fraction < 1.0:   # hollow / low-infill core
        f = max(layup.core_fraction, 1e-6)
        base = dataclasses.replace(base, E=base.E * f, Gy=base.Gy * f, Gz=base.Gz * f, rho=base.rho * f)
    mats = [base]
    matid = np.zeros(mask.shape, int)
    if layup.kind == "core_skin" and layup.skin_material and layup.skin_thickness > 0:
        mats.append(_mat_cell(layup.skin_material, orientation, t, e2, e3))
        # distance from cell centre to the outside (anisotropic sampling)
        dist = ndimage.distance_transform_edt(np.pad(mask, 1), sampling=(dy, dz))[1:-1, 1:-1]
        # edt measures centre-to-centre distance to the nearest outside cell centre;
        # the boundary is half a cell nearer.
        dist = dist - 0.5 * min(dy, dz)
        matid[mask & (dist <= layup.skin_thickness)] = 1
    elif layup.kind == "rod_insert" and layup.insert_material and layup.insert_diameter > 0:
        mats.append(_mat_cell(layup.insert_material, rod_orientation(t), t, e2, e3))
        for frac in layup.insert_positions:
            cy, cz = _insert_point(shape, frac)
            sel = (Y - cy) ** 2 + (Z - cz) ** 2 <= (layup.insert_diameter / 2) ** 2
            if not (sel & mask).any():   # rod smaller than a cell: mark the nearest cell
                k = np.argmin(np.where(mask, (Y - cy) ** 2 + (Z - cz) ** 2, np.inf))
                sel = np.zeros_like(mask)
                sel.flat[k] = True
            matid[sel & mask] = 1
    E = np.array([m.E for m in mats])[matid]
    Gy = np.array([m.Gy for m in mats])[matid]
    Gz = np.array([m.Gz for m in mats])[matid]
    rho = np.array([m.rho for m in mats])[matid]
    dA = dy * dz
    w = mask * dA
    EA = float((E * w).sum())
    yc = float((E * w * Y).sum() / EA)
    zc = float((E * w * Z).sum() / EA)
    EIy = float((E * w * (Z - zc) ** 2).sum())
    EIz = float((E * w * (Y - yc) ** 2).sum())
    EIyz = float((E * w * (Y - yc) * (Z - zc)).sum())
    GJ, phi, tau = _prandtl(mask, Gy, Gz, dy, dz)
    sphi = phi.sum()
    ys = float((phi * Y).sum() / sphi) if sphi > 0 else yc
    zs = float((phi * Z).sum() / sphi) if sphi > 0 else zc
    kap = 0.9 if isinstance(shape, CircleShape) else 5.0 / 6.0
    m = float((rho * w).sum())
    my = float((rho * w * Y).sum())
    mz = float((rho * w * Z).sum())
    Iyy_m = float((rho * w * Z**2).sum())
    Izz_m = float((rho * w * Y**2).sum())
    Iyz_m = float((rho * w * Y * Z).sum())
    fib = _boundary_fibres(Y, Z, mask, matid, mats)
    return SectionProps(EA=EA, EIy=EIy, EIz=EIz, EIyz=EIyz, GJ=GJ,
                        GAy=kap * float((Gy * w).sum()), GAz=kap * float((Gz * w).sum()),
                        mass_per_len=m, Ip_mass=Iyy_m + Izz_m, Iyy_mass=Iyy_m, Izz_mass=Izz_m,
                        Iyz_mass=Iyz_m, centroid=(yc, zc), shear_centre=(ys, zs),
                        mass_centre=(my / m, mz / m), area=float(w.sum()), tau_per_twist=tau,
                        fibres=fib, kappa_shear=kap, method="grid")


def _insert_point(shape, frac: float) -> tuple[float, float]:
    if isinstance(shape, PolygonShape) and shape.midline:
        ml = np.asarray(shape.midline)
        z = float(np.interp(frac, ml[:, 0], ml[:, 1]))
        return shape.le_y - frac * shape.chord, z
    if isinstance(shape, RectShape):
        return shape.cy + shape.width / 2 - frac * shape.width, shape.cz
    if isinstance(shape, CircleShape):
        return shape.cy + shape.diameter / 2 - frac * shape.diameter, shape.cz
    p = shape.array
    lo, hi = p.min(0), p.max(0)
    return hi[0] - frac * (hi[0] - lo[0]), 0.5 * (lo[1] + hi[1])


def rotate_props(sp_: SectionProps, angle: float) -> SectionProps:
    """Express props in axes rotated by +angle about the beam axis (y→z positive).

    A vector with components (y, z) in the old axes has components
    (c y + s z, −s y + c z) in the new axes."""
    import copy
    c, s = np.cos(angle), np.sin(angle)
    Rm = np.array([[c, s], [-s, c]])
    o = copy.copy(sp_)
    rot = lambda p: tuple(Rm @ np.asarray(p))   # noqa: E731
    o.centroid, o.shear_centre, o.mass_centre = rot(sp_.centroid), rot(sp_.shear_centre), rot(sp_.mass_centre)
    # bending tensor in (y,z): [[EIz, EIyz],[EIyz, EIy]] (∫E y², ∫E yz, ∫E z²)
    Tb = Rm @ np.array([[sp_.EIz, sp_.EIyz], [sp_.EIyz, sp_.EIy]]) @ Rm.T
    o.EIz, o.EIyz, o.EIy = Tb[0, 0], Tb[0, 1], Tb[1, 1]
    Tm = Rm @ np.array([[sp_.Izz_mass, sp_.Iyz_mass], [sp_.Iyz_mass, sp_.Iyy_mass]]) @ Rm.T
    o.Izz_mass, o.Iyz_mass, o.Iyy_mass = Tm[0, 0], Tm[0, 1], Tm[1, 1]
    Tg = Rm @ np.diag([sp_.GAy, sp_.GAz]) @ Rm.T
    o.GAy, o.GAz = Tg[0, 0], Tg[1, 1]
    o.fibres = [Fibre(*(Rm @ np.array([f.y, f.z])), f.E, f.material, f.orientation) for f in sp_.fibres]
    return o
