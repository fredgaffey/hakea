"""Interlocking-tooth (dovetail finger) joint between two blade materials.

Pure geometry/section/strength helpers shared by geometry.py (OCC clip polygon), rotor.py
(graded section properties, checks). Frame: blade-1 frame H; blade spans +x, chord along y
(LE at +y), thickness along z (= build direction). The fingers live in the x-y plane and are
extruded through the full thickness along z, so the walls are vertical along the build dir.

Layout: inboard material A (e.g. TPU), outboard material B (e.g. CF). B occupies x >= r_joint
plus n fingers that protrude inboard into A; A fills everything else (its "fingers" are the
gaps between B's fingers). Each B finger is a trapezoid: neck_width at x = r_joint widening to
dovetail_width at the inboard tip (undercut -> mechanical lock against centrifugal pull).
Fingers span the middle (1 - 2*EDGE_MARGIN) of the chord at r_joint.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional

import numpy as np

from .bladegeom import airfoil_coords, resolve_stations
from .materials import get_material
from .schema import Check, Design

EDGE_MARGIN = 0.10       # fraction of chord left finger-free at LE and TE

# A-B interface shear strength as a fraction of min(S_A, S_B). GUESSES (no coupon data yet):
# TPU-on-PA fuses poorly/mechanically only, TPU-on-PETG bonds well (similar processing temps).
BOND_FACTORS: dict[frozenset, float] = {
    frozenset({"tpu", "pa"}): 0.5,
    frozenset({"tpu", "petg"}): 0.8,
}
DEFAULT_BOND_FACTOR = 0.3      # any other pair


def _family(name: str) -> str:
    n = name.lower()
    for pre, fam in (("tpu", "tpu"), ("petg", "petg"), ("pa", "pa"), ("pla", "pla")):
        if n.startswith(pre):
            return fam
    return n


def bond_factor(mat_a: str, mat_b: str, override: Optional[float] = None) -> float:
    if override is not None:
        return float(override)
    return BOND_FACTORS.get(frozenset({_family(mat_a), _family(mat_b)}), DEFAULT_BOND_FACTOR)


@dataclass
class JointSpec:
    x_j: float
    mat_a: str
    mat_b: str
    lengths: np.ndarray          # (n,) finger lengths
    centres: np.ndarray          # (n,) finger centre y
    neck: float
    dove: float
    pitch: float
    chord: float
    y_cum: np.ndarray            # y grid (ascending) and cumulative section area for weighting
    a_cum: np.ndarray
    bond: float

    @property
    def n(self) -> int:
        return len(self.lengths)

    @property
    def x_min(self) -> float:
        return self.x_j - float(self.lengths.max())

    # ---- thickness-weighted chord fractions
    def area_frac(self, ya: float, yb: float) -> float:
        a = np.interp([ya, yb], self.y_cum, self.a_cum)
        return float((a[1] - a[0]) / self.a_cum[-1])

    def finger_thickness(self) -> np.ndarray:
        """Mean section thickness over each finger's neck width (m)."""
        out = []
        for yc in self.centres:
            ya, yb = yc - self.neck / 2, yc + self.neck / 2
            a = np.interp([ya, yb], self.y_cum, self.a_cum)
            out.append((a[1] - a[0]) / max(yb - ya, 1e-12))
        return np.array(out)

    def b_fraction(self, x: float) -> float:
        """Area fraction of material B in the section at radius x (0..1)."""
        if x >= self.x_j:
            return 1.0
        d = self.x_j - x
        f = 0.0
        for yc, L in zip(self.centres, self.lengths):
            if L > d:
                w = self.neck + (self.dove - self.neck) * d / L
                f += self.area_frac(yc - w / 2, yc + w / 2)
        return float(min(f, 1.0))

    def b_fraction_avg(self, x0: float, x1: float, n: int = 24) -> float:
        xs = x0 + (np.arange(n) + 0.5) * (x1 - x0) / n
        return float(np.mean([self.b_fraction(x) for x in xs]))

    # ---- OCC clip polygon
    def polygon(self, y_big: float, stub: float = 0.0005) -> np.ndarray:
        """Closed CCW polygon (N,2) of the finger prism used to split the inboard loft
        (which ends at x_j): fingers with straight neck stubs reaching `stub` beyond x_j (so no
        coincident faces with the loft cap) joined by a slab outboard of x_j + stub."""
        xs = self.x_j + stub
        pts = [(xs + stub, -y_big), (xs + stub, y_big), (xs, y_big)]
        for i in np.argsort(-self.centres):          # +y first
            yc, L = self.centres[i], self.lengths[i]
            pts += [(xs, yc + self.neck / 2), (self.x_j, yc + self.neck / 2),
                    (self.x_j - L, yc + self.dove / 2), (self.x_j - L, yc - self.dove / 2),
                    (self.x_j, yc - self.neck / 2), (xs, yc - self.neck / 2)]
        pts.append((xs, -y_big))
        return np.array(pts)

    # ---- wall areas
    def flank_length(self) -> np.ndarray:
        return np.hypot(self.lengths, (self.dove - self.neck) / 2)

    def wall_area(self) -> float:
        """Total bonded finger flank area (two flanks per finger)."""
        return float(np.sum(2.0 * self.flank_length() * self.finger_thickness()))

    def pullout_tear_length(self) -> float:
        """Sum over fingers of (dovetail - neck): TPU shear-tear width around the heads."""
        return self.n * (self.dove - self.neck)


def _thickness_profile(xy: np.ndarray, chord: float, ref: float):
    """(y grid ascending, cumulative section area) for the airfoil at the given chord."""
    n_le = int(np.argmin(xy[:, 0]))
    up, lo = xy[: n_le + 1][::-1], xy[n_le:]       # both LE -> TE
    xs = np.linspace(0.0, 1.0, 400)
    yu = np.interp(xs, up[:, 0], up[:, 1])
    yl = np.interp(xs, lo[:, 0], lo[:, 1])
    t = np.clip(yu - yl, 0.0, None) * chord
    y = -(xs - ref) * chord                         # LE at +y
    y, t = y[::-1], t[::-1]                         # ascending y
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (t[1:] + t[:-1]) * np.diff(y))])
    return y, cum


def resolve(design: Design) -> Optional[JointSpec]:
    """JointSpec for design.blade.joint (None if no joint). Raises ValueError if infeasible."""
    b = design.blade
    j = b.joint
    if j is None:
        return None
    R = b.radius
    mat_b = j.material_b or b.layup.material
    mat_a = j.material_a or design.flexure.layup.material
    get_material(mat_a), get_material(mat_b)
    sts = resolve_stations(b)
    st_r = np.array([s.r_frac * R for s in sts])
    x_j = j.r_frac * R
    chord = float(np.interp(x_j, st_r, [s.chord for s in sts]))
    tcs = [s.thickness_frac for s in sts]
    tc = None
    if all(t is not None for t in tcs):
        tc = float(np.interp(x_j, st_r, tcs))
    xy = airfoil_coords(b.airfoil, tc, n=81)
    ref = b.elastic_axis_chord_frac
    y_hi = ref * chord - EDGE_MARGIN * chord
    y_lo = -(1.0 - ref) * chord + EDGE_MARGIN * chord
    n = j.n_fingers
    pitch = (y_hi - y_lo) / n
    if j.neck_width >= pitch:
        raise ValueError(f"joint neck_width {j.neck_width*1e3:.2f} mm >= finger pitch "
                         f"{pitch*1e3:.2f} mm (reduce n_fingers or neck)")
    if j.dovetail_width >= 0.98 * pitch:
        raise ValueError(f"joint dovetail_width {j.dovetail_width*1e3:.2f} mm leaves no gap at "
                         f"pitch {pitch*1e3:.2f} mm")
    centres = y_hi - (np.arange(n) + 0.5) * pitch
    y, cum = _thickness_profile(xy, chord, ref)
    return JointSpec(x_j=x_j, mat_a=mat_a, mat_b=mat_b, lengths=np.array(j.lengths(), float),
                     centres=centres, neck=j.neck_width, dove=j.dovetail_width, pitch=pitch,
                     chord=chord, y_cum=y, a_cum=cum, bond=bond_factor(mat_a, mat_b, j.bond_factor))


# --------------------------------------------------------------------------- sections


def blend_sections(sa, sb, f: float):
    """Rule-of-mixtures SectionProps: A and B sections side by side across the chord with
    B area fraction f. Stiffnesses/inertias are linear in f (parallel arrangement); centres
    are stiffness-/mass-weighted; stress-recovery fibres of both materials are kept."""
    if f <= 0.0:
        return sa
    if f >= 1.0:
        return sb
    names = ("EA", "EIy", "EIz", "EIyz", "GJ", "GAy", "GAz", "mass_per_len", "Ip_mass",
             "Iyy_mass", "Izz_mass", "Iyz_mass", "area", "tau_per_twist")
    kw = {a: (1 - f) * getattr(sa, a) + f * getattr(sb, a) for a in names}
    wa, wb = (1 - f) * sa.EA, f * sb.EA
    ma, mb = (1 - f) * sa.mass_per_len, f * sb.mass_per_len

    def mix(pa, pb, wa_, wb_):
        s = max(wa_ + wb_, 1e-300)
        return (float((wa_ * pa[0] + wb_ * pb[0]) / s), float((wa_ * pa[1] + wb_ * pb[1]) / s))

    kw["centroid"] = mix(sa.centroid, sb.centroid, wa, wb)
    kw["shear_centre"] = mix(sa.shear_centre, sb.shear_centre, wa, wb)
    kw["mass_centre"] = mix(sa.mass_centre, sb.mass_centre, ma, mb)
    kw["fibres"] = list(sa.fibres) + list(sb.fibres)
    kw["method"] = "joint_mix"
    return replace(sa, **kw)


def ea_fraction_b(sa, sb, f: float) -> float:
    if f <= 0:
        return 0.0
    if f >= 1:
        return 1.0
    a, b = (1 - f) * sa.EA, f * sb.EA
    return float(b / (a + b))


# --------------------------------------------------------------------------- strength checks


def joint_checks(spec: JointSpec, N: float, M: float, member: str = "joint") -> list[Check]:
    """Joint strength checks for axial force N (centrifugal, tension, N) and bending moment M
    (N m) at the joint.

    bond_wall_shear: equivalent load F = N + M/(chord_usable/2) (bending carried as a couple
      across the finger span) shared over the finger flank walls, tau = F / wall area vs
      min(S_A, S_B) * bond_factor.
    dovetail_pullout: N / (n * (dovetail - neck) * thickness * S_A) -- TPU tears around the
      dovetail heads. With dovetail == neck there is no lock: capacity falls back to the bond
      (flank shear) capacity and the note says so.
    """
    ma, mb = get_material(spec.mat_a), get_material(spec.mat_b)
    N = abs(float(N))
    lever = max(0.5 * spec.n * spec.pitch, 1e-6)
    F = N + abs(float(M)) / lever
    Aw = spec.wall_area()
    tau_allow = min(ma.S, mb.S) * spec.bond
    tau = F / max(Aw, 1e-18)
    out = [Check(name="joint_wall_shear", member=member, value=tau, allowable=tau_allow,
                 utilisation=tau / tau_allow,
                 note=f"F={F:.1f} N (N={N:.1f}, M={abs(M)*1e3:.2f} N mm / {lever*1e3:.1f} mm) over "
                      f"{Aw*1e6:.1f} mm2 flank; bond factor {spec.bond:.2f} (guess)")]
    t_f = spec.finger_thickness()
    tear = float(np.sum((spec.dove - spec.neck) * t_f)) * ma.S
    if tear > 0:
        out.append(Check(name="joint_pullout", member=member, value=N, allowable=tear,
                         utilisation=N / tear,
                         note=f"centrifugal N vs TPU tear around {spec.n} dovetails "
                              f"((dove-neck)*t*S_A)"))
    else:
        cap = tau_allow * Aw
        out.append(Check(name="joint_pullout", member=member, value=N, allowable=cap,
                         utilisation=N / cap,
                         note="no undercut (dovetail == neck): pull-out relies on bond only"))
    return out
