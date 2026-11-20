"""Strip aerodynamics for the flexblade rotor (OWNER: aero).

Contents
--------
* Section polars from NeuralFoil, tabulated once per airfoil shape over
  (alpha, log10 Re), blended to flat-plate post-stall, cached on disk
  (``~/.cache/flexblade/polars`` or ``$FLEXBLADE_POLAR_CACHE``) and wrapped in bicubic
  splines (values + analytic d/dalpha, d/dlogRe).  Prandtl–Glauert on cl, cm.
* :meth:`AeroModel.trim` — hover blade-element-momentum theory (BEMT) with
  Prandtl tip + root loss; solves rpm for a thrust, thrust for an rpm, or
  collective for (rpm, thrust).
* :meth:`AeroModel.linearized` — per-station complex 3x4 strip-theory matrices
  (quasi-steady or Theodorsen, plus apparent mass) for the 1/rev harmonic model.
* :meth:`AeroModel.pitt_peters_hover` — hover Pitt–Peters cyclic inflow gain,
  plus helpers to turn a fixed-frame hub moment into λ(r, ψ) and into the
  rotating-frame 1/rev inflow each blade sees.

Frames and signs (DESIGN.md)
----------------------------
Hub frame H, rotor spins +Ω about +z, blade 1 along +x, travels toward +y, thrust +z.
At a blade-1 section at radius r the air velocity *relative to the section* is

    v_rel = (0, -U_T, -U_P),   U_T = Ωr (+ in-plane section velocity),
                               U_P = v_i (+ downward inflow) + section heave rate

so the inflow angle is φ = atan2(U_P, U_T) and the angle of attack α = θ − φ, θ the
geometric pitch (twist + collective + elastic, nose-up about +x).  Unit vectors
of drag (along v_rel) and lift (⟂ v_rel, upward component positive) are

    e_D = (0, -cosφ, -sinφ),   e_L = (0, -sinφ, cosφ)

hence, per unit span (forces on blade 1 in H, moment about +x at the quarter chord):

    Fz =  L cosφ − D sinφ           L = ½ρW²c·cl(α, Re, M)
    Fy = −L sinφ − D cosφ           D = ½ρW²c·cd(α, Re, M)
    Mx =  ½ρW²c²·cm(α, Re, M)       W² = U_T² + U_P²

(drag on blade 1 therefore points toward −y, torque required Q = B∫ r(−Fy) dr).
Harmonic convention x(t) = Re{X e^{iωt}}.
"""
from __future__ import annotations

import hashlib
import os
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.interpolate import RectBivariateSpline
from scipy.optimize import brentq
from scipy.special import hankel2

from .bladegeom import airfoil_coords, resolve_stations
from .schema import Design

G0 = 9.80665
GAMMA_AIR = 1.4
R_AIR = 287.05

POLAR_VERSION = "nf-0.3.3-v1"
NF_MODEL_SIZE = "xlarge"
N_CRIT = 9.0
_ALPHA_DEG = np.unique(np.concatenate([np.arange(-90.0, -30.0, 2.5),
                                       np.arange(-30.0, 30.0 + 1e-9, 0.5),
                                       np.arange(32.5, 90.0 + 1e-9, 2.5)]))
_LOGRE = np.arange(3.5, 6.5 + 1e-9, 0.25)
STALL_BLEND_DEG = (20.0, 35.0)   # NeuralFoil → flat plate blend over this |alpha| range
CD_FLAT_PLATE = 1.9
MACH_CLAMP = 0.7


def cache_dir() -> Path:
    d = Path(os.environ.get("FLEXBLADE_POLAR_CACHE", Path.home() / ".cache" / "flexblade")) / "polars"
    d.mkdir(parents=True, exist_ok=True)
    return d


def air_viscosity(temperature_C: float) -> float:
    """Dynamic viscosity of air, Sutherland's law [Pa s]."""
    T = temperature_C + 273.15
    return 1.716e-5 * (T / 273.15) ** 1.5 * (273.15 + 110.4) / (T + 110.4)


def speed_of_sound(temperature_C: float) -> float:
    return float(np.sqrt(GAMMA_AIR * R_AIR * (temperature_C + 273.15)))


def theodorsen(k) -> np.ndarray:
    """Theodorsen function C(k) = H1⁽²⁾(k) / (H1⁽²⁾(k) + i H0⁽²⁾(k)) for e^{iωt}.

    k = ωb/U (b = c/2).  C(0) = 1, C(∞) = ½.  Vectorised; k ≤ 1e-9 → 1.
    """
    k = np.asarray(k, float)
    out = np.ones(k.shape, complex)
    m = k > 1e-9
    if np.any(m):
        h1 = hankel2(1, k[m])
        h0 = hankel2(0, k[m])
        out[m] = h1 / (h1 + 1j * h0)
    return out


# =============================================================================== polars


def _smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def _flat_plate(alpha):
    """Flat-plate post-stall model: cn = CD90 sinα, xcp: 0.25c → 0.5c at 90°."""
    cl = CD_FLAT_PLATE * np.sin(alpha) * np.cos(alpha)
    cd = CD_FLAT_PLATE * np.sin(alpha) ** 2 + 0.02
    cn = CD_FLAT_PLATE * np.sin(alpha)
    xcp = 0.25 + 0.25 * np.abs(np.sin(alpha))
    cm = -cn * (xcp - 0.25)
    return cl, cd, cm


def _compute_polar_table(coords: np.ndarray) -> dict[str, np.ndarray]:
    import neuralfoil as nf
    A, LR = np.meshgrid(_ALPHA_DEG, _LOGRE, indexing="ij")
    r = nf.get_aero_from_coordinates(coordinates=coords, alpha=A.ravel(), Re=10.0 ** LR.ravel(),
                                     n_crit=N_CRIT, model_size=NF_MODEL_SIZE)
    shp = A.shape
    cl = np.asarray(r["CL"]).reshape(shp)
    cd = np.asarray(r["CD"]).reshape(shp)
    cm = np.asarray(r["CM"]).reshape(shp)
    conf = np.asarray(r["analysis_confidence"]).reshape(shp)
    a = np.radians(A)
    w = _smoothstep((np.abs(A) - STALL_BLEND_DEG[0]) / (STALL_BLEND_DEG[1] - STALL_BLEND_DEG[0]))
    fcl, fcd, fcm = _flat_plate(a)
    cd = np.maximum(cd, 1e-3)
    return dict(alpha=np.radians(_ALPHA_DEG), logre=_LOGRE.copy(),
                cl=(1 - w) * cl + w * fcl, cd=(1 - w) * cd + w * fcd,
                cm=(1 - w) * cm + w * fcm, conf=conf)


class PolarTable:
    """Bicubic-spline polar of one airfoil shape over (alpha [rad], log10 Re).

    `eval(alpha, Re, mach)` returns cl, cd, cm and their partials with respect
    to alpha (per rad) and to the local speed W at fixed alpha (per m/s, through
    Re ∝ W and M ∝ W).  Prandtl–Glauert: cl, cm ← cl₀/β, cm₀/β, β = √(1−M²),
    M clamped at MACH_CLAMP.  alpha clipped to ±90°, Re to the table range.
    """

    def __init__(self, coords: np.ndarray):
        coords = np.asarray(coords, float)
        h = hashlib.sha1()
        h.update(POLAR_VERSION.encode())
        h.update(NF_MODEL_SIZE.encode())
        h.update(np.float64(N_CRIT).tobytes())
        h.update(np.round(coords, 6).tobytes())
        h.update(_ALPHA_DEG.tobytes())
        h.update(_LOGRE.tobytes())
        self.key = h.hexdigest()[:20]
        path = cache_dir() / f"{self.key}.npz"
        tab = None
        if path.exists():
            try:
                with np.load(path) as z:
                    tab = {k: z[k] for k in z.files}
            except Exception:  # corrupt cache → recompute
                tab = None
        if tab is None:
            tab = _compute_polar_table(coords)
            tmp = path.with_suffix(f".{os.getpid()}.{threading.get_ident()}.tmp.npz")
            np.savez(tmp, **tab)
            os.replace(tmp, path)
        self.table = tab
        a, lr = tab["alpha"], tab["logre"]
        self._lim_a = (a[0], a[-1])
        self._lim_lr = (lr[0], lr[-1])
        self._sp = {k: RectBivariateSpline(a, lr, tab[k], kx=3, ky=3, s=0) for k in ("cl", "cd", "cm")}

    def eval(self, alpha, Re, mach, derivs: bool = True) -> dict[str, np.ndarray]:
        alpha = np.asarray(alpha, float)
        lr_raw = np.log10(np.maximum(np.asarray(Re, float), 1.0))
        lr = np.clip(lr_raw, *self._lim_lr)
        in_re = (lr_raw > self._lim_lr[0]) & (lr_raw < self._lim_lr[1])
        a = np.clip(alpha, *self._lim_a)
        M = np.minimum(np.asarray(mach, float), MACH_CLAMP)
        beta = np.sqrt(1.0 - M**2)
        out = {}
        for k, sp in self._sp.items():
            v = sp.ev(a, lr)
            if k == "cd":
                v = np.maximum(v, 1e-3)
            out[k] = v / beta if k != "cd" else v
            if derivs:
                da = sp.ev(a, lr, dx=1)
                dlr = sp.ev(a, lr, dy=1) * in_re
                out[k + "_a"] = da / beta if k != "cd" else da
                out[k + "_lr"] = dlr / beta if k != "cd" else dlr
                # d/dW at fixed alpha: dlogRe/dW = 1/(W ln10); d(1/β)/dW = M²/(β³ W)
                pg = (M**2 / beta**3) * (np.asarray(mach) < MACH_CLAMP) if k != "cd" else 0.0
                out[k + "_pg"] = v * pg   # multiply by 1/W later
        return out


_TABLES: dict[tuple, PolarTable] = {}
_TLOCK = threading.Lock()


def get_polar(airfoil, thickness_frac: Optional[float]) -> PolarTable:
    """Process-wide memoised :class:`PolarTable` for an airfoil at a t/c."""
    coords = airfoil_coords(airfoil, thickness_frac)
    key = hashlib.sha1(np.round(coords, 6).tobytes()).hexdigest()
    with _TLOCK:
        if key not in _TABLES:
            _TABLES[key] = PolarTable(coords)
        return _TABLES[key]


def _default_tc(airfoil) -> Optional[float]:
    if airfoil.kind == "naca4":
        return int(airfoil.naca[2:4]) / 100.0
    return None


# =============================================================================== trim result


@dataclass
class AeroTrim:
    """Hover trim state. Arrays are per aero station (radius `r`, ascending).

    Per-unit-span loads are for **blade 1 in frame H** at the quarter chord:
    Fz (+up), Fy (+y; drag gives Fy<0), Fx (radial, from coning), Mx (+nose-up).
    Angles in rad.  U_T, U_P, W, v_i in m/s.  dT_dr, dQ_dr are whole-rotor (×B).
    """
    rpm: float
    omega: float
    rho: float
    mu: float
    a_sound: float
    n_blades: int
    R: float
    collective: float               # rad, added to twist
    r: np.ndarray
    w_int: np.ndarray               # trapezoid weights: ∫ f dr ≈ Σ w_int f
    chord: np.ndarray
    theta: np.ndarray               # total geometric pitch (twist+collective+elastic)
    beta: np.ndarray                # flap slope used (rad)
    U_T: np.ndarray
    U_P: np.ndarray
    W: np.ndarray
    phi: np.ndarray
    alpha: np.ndarray
    Re: np.ndarray
    mach: np.ndarray
    F_loss: np.ndarray
    v_i: np.ndarray
    cl: np.ndarray
    cd: np.ndarray
    cm: np.ndarray
    cl_a: np.ndarray
    cd_a: np.ndarray
    cm_a: np.ndarray
    cl_W: np.ndarray                # ∂cl/∂W at fixed alpha (Re + Mach), per m/s
    cd_W: np.ndarray
    cm_W: np.ndarray
    Fz: np.ndarray
    Fy: np.ndarray
    Fx: np.ndarray
    Mx: np.ndarray
    dT_dr: np.ndarray
    dQ_dr: np.ndarray
    T: float
    Q: float
    P: float
    FoM: float
    CT: float
    CQ: float
    lambda0: float                  # momentum inflow ratio √(CT/2)
    converged: bool = True
    notes: list[str] = field(default_factory=list)
    v_ext: Optional[np.ndarray] = None   # imposed external inflow (coaxial), m/s + down
    Q_root: float = 0.0   # root-hardware drag torque (included in Q, P), N m

    @property
    def v_own(self) -> np.ndarray:
        """Own induced velocity (total U_P minus imposed external inflow)."""
        return self.U_P - (0.0 if self.v_ext is None else self.v_ext)

    @property
    def area(self) -> float:
        return np.pi * self.R**2

    @property
    def tip_speed(self) -> float:
        return self.omega * self.R

    def stations_dict(self) -> dict[str, list[float]]:
        """Plain-list per-station table for `TrimResult.stations`."""
        d = dict(r=self.r, chord=self.chord, theta_deg=np.degrees(self.theta),
                 phi_deg=np.degrees(self.phi), alpha_deg=np.degrees(self.alpha),
                 Re=self.Re, mach=self.mach, cl=self.cl, cd=self.cd, cm=self.cm,
                 v_i=self.v_i, F_loss=self.F_loss, Fz=self.Fz, Fy=self.Fy, Mx=self.Mx,
                 dT_dr=self.dT_dr, dQ_dr=self.dQ_dr)
        return {k: [float(x) for x in np.asarray(v)] for k, v in d.items()}

    def summary(self) -> dict[str, float]:
        return dict(rpm=self.rpm, thrust=self.T, torque=self.Q, power=self.P,
                    figure_of_merit=self.FoM, collective_deg=float(np.degrees(self.collective)),
                    CT=self.CT, CQ=self.CQ, lambda0=self.lambda0,
                    tip_mach=float(self.mach[-1]), tip_speed=self.tip_speed)


# =============================================================================== cyclic inflow


@dataclass
class CyclicInflow:
    """Linear cyclic inflow  v(r, ψ) = ΩR·(r/R)·(λc cosψ + λs sinψ)  [m/s, + down].

    ψ is the fixed-frame azimuth measured from fixed +x, counter-clockwise from
    above (blade 1 sits at ψ = Ωt).  λc, λs are complex harmonic amplitudes at
    the fixed-frame frequency `omega_fixed` (0 for the steady control moment).
    """
    lam_c: complex
    lam_s: complex
    omega_rotor: float
    R: float
    omega_fixed: float = 0.0

    def field(self, r, psi) -> np.ndarray:
        """Inflow perturbation (real part, at t = 0 of the fixed-frame harmonic), m/s."""
        r = np.asarray(r, float)
        psi = np.asarray(psi, float)
        return np.real(self.omega_rotor * r * (self.lam_c * np.cos(psi) + self.lam_s * np.sin(psi)))

    def blade_1p(self, r, blade: int = 1, n_blades: int = 2) -> np.ndarray:
        """Rotating-frame 1/rev complex amplitude dλ(r) seen by blade `blade`.

        Valid for a steady fixed-frame inflow (omega_fixed = 0).  Blade k is at
        ψ_k = Ωt + 2π(k−1)/B, so
            dλ_k(t) = ΩR (r/R)(λc cosψ_k + λs sinψ_k) = Re{ ΩR(r/R)(λc − iλs) e^{iψ0_k} e^{iΩt} }.
        For omega_fixed ≠ 0 the blade sees Ω ± omega_fixed; not represented here.
        """
        psi0 = 2.0 * np.pi * (blade - 1) / n_blades
        return self.omega_rotor * np.asarray(r, float) * (self.lam_c - 1j * self.lam_s) * np.exp(1j * psi0)


# =============================================================================== model


class AeroModel:
    """Strip-theory aerodynamics of one design.

    Parameters
    ----------
    design : Design
    r : optional array of aero station radii [m] (e.g. the structure's beam
        nodes).  Default: ``analysis.n_blade_elements + 1`` points from the root
        cut-out to the tip, clustered toward the tip (r = r0 + (R−r0) sin(πs/2)).
        Chord, twist and t/c are interpolated linearly in r/R from
        :func:`bladegeom.resolve_stations`.  Polars are built (or loaded from
        the disk cache) here, grouped by t/c rounded to 0.005.
    """

    def __init__(self, design: Design, r: Optional[np.ndarray] = None):
        self.design = design
        blade = design.blade
        self.R = blade.radius
        self.B = blade.n_blades
        st = resolve_stations(blade)
        rf = np.array([s.r_frac for s in st])
        r0 = max(blade.root_cutout_frac, rf[0]) * self.R
        if r is None:
            n = max(int(design.analysis.n_blade_elements) + 1, 5)
            s = np.linspace(0.0, 1.0, n)
            r = r0 + (self.R - r0) * np.sin(0.5 * np.pi * s)
        self.r = np.asarray(r, float)
        self.r_root = float(min(r0, self.r[0]))
        x = self.r / self.R
        self.chord = np.interp(x, rf, [s.chord for s in st])
        self.twist = np.radians(np.interp(x, rf, [s.twist_deg for s in st]))
        tc0 = _default_tc(blade.airfoil)
        tcs = [s.thickness_frac if s.thickness_frac is not None else tc0 for s in st]
        self._groups: list[tuple[PolarTable, np.ndarray]] = []
        if any(t is None for t in tcs):
            self._groups.append((get_polar(blade.airfoil, None), np.arange(len(self.r))))
        else:
            tc = np.round(np.interp(x, rf, tcs) / 0.005) * 0.005
            for t in np.unique(tc):
                self._groups.append((get_polar(blade.airfoil, float(t)), np.nonzero(tc == t)[0]))
        self.thickness_frac = None if any(t is None for t in tcs) else np.interp(x, rf, tcs)
        # trapezoid weights
        w = np.zeros_like(self.r)
        d = np.diff(self.r)
        w[:-1] += 0.5 * d
        w[1:] += 0.5 * d
        self.w_int = w
        op = design.op
        self.rho = op.rho
        self.mu = air_viscosity(op.temperature_C)
        self.a_sound = speed_of_sound(op.temperature_C)
        self.v_ext = self.external_inflow(op.extra_inflow)

    def external_inflow(self, prof) -> np.ndarray:
        """Imposed axial velocity (m/s, + down) on the aero stations from an
        `InflowProfile` (linear in r/R, zero outside its r_frac range)."""
        if prof is None or not prof.r_frac:
            return np.zeros_like(self.r)
        rf = np.asarray(prof.r_frac, float)
        v = np.asarray(prof.v, float)
        return np.interp(self.r / self.R, rf, v, left=0.0, right=0.0)

    # ------------------------------------------------------------------ polars
    def polar(self, alpha, W, derivs: bool = True, idx: Optional[np.ndarray] = None) -> dict:
        """cl, cd, cm (+ _a, _W partials if derivs) at every station (or `idx`)."""
        chord = self.chord if idx is None else self.chord[idx]
        alpha = np.broadcast_to(np.asarray(alpha, float), chord.shape)
        W = np.broadcast_to(np.asarray(W, float), chord.shape)
        Re = self.rho * W * chord / self.mu
        M = W / self.a_sound
        keys = ("cl", "cd", "cm") + (("cl_a", "cd_a", "cm_a", "cl_W", "cd_W", "cm_W") if derivs else ())
        out = {k: np.zeros(chord.shape) for k in keys}
        pos = np.arange(len(self.r)) if idx is None else np.asarray(idx)
        single = len(self._groups) == 1
        for tab, gi in self._groups:
            m = slice(None) if single else np.isin(pos, gi)
            if not single and not np.any(m):
                continue
            e = tab.eval(alpha[m], Re[m], M[m], derivs)
            for k in ("cl", "cd", "cm"):
                out[k][m] = e[k]
                if derivs:
                    out[k + "_a"][m] = e[k + "_a"]
                    Wm = np.maximum(W[m], 1e-9)
                    out[k + "_W"][m] = (e[k + "_lr"] / np.log(10.0) + e[k + "_pg"]) / Wm
        out["Re"], out["mach"] = Re, M
        return out

    # ------------------------------------------------------------------ section loads
    def section_loads(self, U_T, U_P, theta, idx=None):
        """Steady strip loads per unit span (Fz, Fy, Mx) for given velocities/pitch.

        Equations in the module docstring.  Used by `trim` and as the reference
        for finite-difference checks of :meth:`linearized`.
        """
        chord = self.chord if idx is None else self.chord[idx]
        U_T, U_P = np.asarray(U_T, float), np.asarray(U_P, float)
        W = np.hypot(U_T, U_P)
        phi = np.arctan2(U_P, U_T)
        p = self.polar(np.asarray(theta) - phi, W, derivs=False, idx=idx)
        q = 0.5 * self.rho * chord
        Fz = q * W * (U_T * p["cl"] - U_P * p["cd"])
        Fy = -q * W * (U_P * p["cl"] + U_T * p["cd"])
        Mx = q * chord * W**2 * p["cm"]
        return Fz, Fy, Mx

    # ------------------------------------------------------------------ BEMT
    def _loss(self, phi):
        s = np.maximum(np.abs(np.sin(phi)), 1e-6)
        B = self.B
        ft = np.exp(-B * (self.R - self.r) / (2.0 * self.r * s))
        fr = np.exp(-B * (self.r - self.r_root) / (2.0 * self.r_root * s))
        F = (2 / np.pi) ** 2 * np.arccos(np.clip(ft, 0, 1)) * np.arccos(np.clip(fr, 0, 1))
        return np.maximum(F, 1e-6)

    def _solve_phi(self, Omega, theta, v_ext=None):
        """Hover BEMT for inflow angle φ at every station (vectorised bisection).

        Annulus momentum with Prandtl loss F vs blade elements (no swirl), with an
        imposed external axial velocity v_e (+ down, e.g. the upper rotor's wake
        on a coaxial lower rotor) treated like a climb velocity:
            ρ 4πr F |v_e + v| v = B ½ρW²c (cl cosφ − cd sinφ),
            U_P = v_e + v = U_T tanφ,  W = U_T/cosφ.
        Multiplying by 2cos²φ/(ρU_T²) gives the smooth residual (e = v_e/U_T)
            g(φ) = 8πr F |sinφ| (sinφ − e cosφ) − B c (cl cosφ − cd sinφ)
        (e = 0: ρ4πrF v|v| = …, plain hover).  Bracket φ ∈ [min(−0.4, atan e − 0.4),
        max(θ, atan e) + 0.4]: g<0 at the lower end (α large, cl>0) and g>0 at
        the upper end (α ≈ −23°, cl<0).
        """
        UT = Omega * self.r
        e = np.zeros_like(UT) if v_ext is None else v_ext / np.maximum(UT, 1e-9)
        ae = np.arctan(e)
        lo = np.minimum(-0.4, ae - 0.4)
        hi = np.clip(np.maximum(theta, ae) + 0.4, 0.1, 1.5)

        def g(phi):
            W = UT / np.cos(phi)
            p = self.polar(theta - phi, W, derivs=False)
            F = self._loss(phi)
            return 8 * np.pi * self.r * F * np.abs(np.sin(phi)) * (np.sin(phi) - e * np.cos(phi)) \
                - self.B * self.chord * (p["cl"] * np.cos(phi) - p["cd"] * np.sin(phi))

        glo = g(lo)
        ok = (glo < 0) & (g(hi) > 0)
        for _ in range(36):            # bracket ≤ 1.9 rad → 3e-11 rad
            mid = 0.5 * (lo + hi)
            gm = g(mid)
            neg = (gm < 0) == (glo < 0)
            lo = np.where(neg, mid, lo)
            glo = np.where(neg, gm, glo)
            hi = np.where(neg, hi, mid)
        return 0.5 * (lo + hi), bool(np.all(ok))

    def _root_torque(self, Omega, inflow):
        if not getattr(self.design.analysis, "root_drag", True):
            return 0.0
        try:
            from .rootdrag import root_drag_torque
            return root_drag_torque(self.design, Omega, self.rho, float(self.r[0]), inflow)
        except Exception:
            return 0.0

    def _shape(self, blade_shape):
        th = np.zeros_like(self.r)
        be = np.zeros_like(self.r)
        if blade_shape:
            rr = blade_shape.get("r")
            for key, arr in (("theta", th), ("beta", be)):
                if key in blade_shape and blade_shape[key] is not None:
                    v = np.asarray(blade_shape[key], float)
                    arr[:] = np.interp(self.r, rr, v) if rr is not None else np.broadcast_to(v, self.r.shape)
        return th, be

    def _evaluate(self, rpm, collective, inflow=None, blade_shape=None) -> AeroTrim:
        Omega = rpm * 2 * np.pi / 60.0
        th_el, beta = self._shape(blade_shape)
        theta = self.twist + collective + th_el
        U_T = Omega * self.r
        notes = []
        v_ext = self.v_ext
        if inflow is not None:
            U_P = np.broadcast_to(np.asarray(inflow, float), self.r.shape).astype(float) + v_ext
            phi = np.arctan2(U_P, U_T)
            conv = True
        else:
            phi, conv = self._solve_phi(Omega, theta, v_ext)
            U_P = U_T * np.tan(phi)
            if not conv:
                notes.append("BEMT bracket failed at some stations")
        W = np.hypot(U_T, U_P)
        alpha = theta - phi
        p = self.polar(alpha, W)
        q = 0.5 * self.rho * self.chord
        L = q * W**2 * p["cl"]
        D = q * W**2 * p["cd"]
        N = L * np.cos(phi) - D * np.sin(phi)
        Fz = N * np.cos(beta)
        Fx = -N * np.sin(beta)
        Fy = -L * np.sin(phi) - D * np.cos(phi)
        Mx = q * self.chord * W**2 * p["cm"]
        dT = self.B * Fz
        dQ = self.B * self.r * (-Fy)
        T = float(np.sum(self.w_int * dT))
        Q = float(np.sum(self.w_int * dQ))
        Q_root = self._root_torque(Omega, float(U_P[0]))
        Q += Q_root
        P = Q * Omega
        A = np.pi * self.R**2
        FoM = float(max(T, 0.0) ** 1.5 / np.sqrt(2 * self.rho * A) / P) if P > 0 else 0.0
        denom = self.rho * A * (Omega * self.R) ** 2
        CT = T / denom if Omega > 0 else 0.0
        CQ = Q / (denom * self.R) if Omega > 0 else 0.0
        return AeroTrim(
            rpm=float(rpm), omega=Omega, rho=self.rho, mu=self.mu, a_sound=self.a_sound,
            n_blades=self.B, R=self.R, collective=float(collective), r=self.r.copy(),
            w_int=self.w_int.copy(), chord=self.chord.copy(), theta=theta, beta=beta,
            U_T=U_T, U_P=U_P, W=W, phi=phi, alpha=alpha, Re=p["Re"], mach=p["mach"],
            F_loss=self._loss(phi), v_i=U_P, cl=p["cl"], cd=p["cd"], cm=p["cm"],
            cl_a=p["cl_a"], cd_a=p["cd_a"], cm_a=p["cm_a"], cl_W=p["cl_W"], cd_W=p["cd_W"],
            cm_W=p["cm_W"], Fz=Fz, Fy=Fy, Fx=Fx, Mx=Mx, dT_dr=dT, dQ_dr=dQ, T=T, Q=Q, P=P,
            FoM=FoM, CT=CT, CQ=CQ, lambda0=float(np.sqrt(max(CT, 0.0) / 2.0)),
            converged=conv, notes=notes, v_ext=v_ext.copy(), Q_root=Q_root)

    def thrust_at(self, rpm, collective_deg=None, blade_shape=None) -> float:
        c = np.radians(self.design.op.collective_deg if collective_deg is None else collective_deg)
        return self._evaluate(rpm, c, blade_shape=blade_shape).T

    def trim(self, rpm: Optional[float] = None, thrust: Optional[float] = None,
             inflow=None, blade_shape: Optional[dict] = None,
             collective_deg: Optional[float] = None,
             rpm_guess: Optional[float] = None) -> AeroTrim:
        """Hover BEMT trim.

        Modes (collective defaults to ``design.op.collective_deg``):
          * rpm given, thrust None  → thrust at that rpm.
          * thrust given, rpm None  → solve rpm for the thrust (brentq on T(rpm)).
          * both given              → solve collective for thrust at rpm.
          * neither                 → ``op.rpm`` if set, else thrust =
                                      vehicle_mass·g·(op.thrust_share or 1).
        ``op.extra_inflow`` (coaxial interference, m/s + down) is always added to the
        rotor's own induced velocity: U_P = v_own + v_ext, momentum on the total
        mass flow (see `_solve_phi`).  `AeroTrim.v_i` is the total, `.v_own` own part.
        inflow : optional prescribed *own* inflow (scalar or per-station, m/s, + down);
            skips the momentum balance (rpm must be resolvable as above).
        blade_shape : optional dict from the structural model, arrays per station:
            ``theta`` elastic pitch [rad, nose-up] added to geometric pitch,
            ``beta`` flap slope dz/dr [rad] (thrust tilts: Fz = N cosβ, Fx = −N sinβ),
            ``r`` radii of those arrays (else they must already be on `self.r`).
        rpm_guess : optional warm start for the rpm solve (speeds up iterations).
        """
        op = self.design.op
        coll = np.radians(op.collective_deg if collective_deg is None else collective_deg)
        if rpm is None and thrust is None:
            if op.rpm is not None:
                rpm = op.rpm
            else:
                share = 1.0 if op.thrust_share is None else op.thrust_share
                thrust = op.vehicle_mass * G0 * share
        if rpm is not None and thrust is None:
            return self._evaluate(rpm, coll, inflow, blade_shape)
        if rpm is not None and thrust is not None:
            f = lambda c: self._evaluate(rpm, c, inflow, blade_shape).T - thrust  # noqa: E731
            # T(collective) is non-monotone past stall: take the first (attached-
            # flow) crossing scanning upward from −25°.
            grid = np.radians(np.arange(-25.0, 30.01, 2.5))
            fv = f(grid[0])
            for a, b in zip(grid[:-1], grid[1:]):
                fb = f(b)
                if fv < 0 <= fb:
                    c = brentq(f, a, b, xtol=1e-7)
                    return self._evaluate(rpm, c, inflow, blade_shape)
                fv = fb
            raise ValueError(f"thrust {thrust:.2f} N not reachable at {rpm:.0f} rpm by collective")
        f = lambda n: self._evaluate(n, coll, inflow, blade_shape).T - thrust  # noqa: E731
        if rpm_guess:
            a, b = rpm_guess * 0.97, rpm_guess * 1.03
            fa, fb = f(a), f(b)
            if fa < 0 < fb:
                n = brentq(f, a, b, xtol=1e-3, rtol=1e-7)
                return self._evaluate(n, coll, inflow, blade_shape)
        lo, hi = 200.0, 2.0e5
        if f(hi) < 0:
            raise ValueError(f"thrust {thrust:.2f} N not reachable below {hi:.0f} rpm "
                             f"(T={f(hi) + thrust:.2f} N); increase pitch/chord")
        # tighten bracket from a momentum-ish guess: T ∝ rpm²
        t1 = f(20000.0) + thrust
        if t1 > 0:
            guess = 20000.0 * np.sqrt(thrust / t1)
            a, b = guess * 0.8, guess * 1.25
            if f(a) < 0 < f(b):
                lo, hi = a, b
        n = brentq(f, lo, hi, xtol=1e-3, rtol=1e-7)
        return self._evaluate(n, coll, inflow, blade_shape)

    # ------------------------------------------------------------------ linearisation
    def linearized(self, trim: AeroTrim, omega: float, model: Optional[str] = None) -> list[np.ndarray]:
        """Per-station complex 3×4 matrices A_i(ω):

            [dFz, dFy, dMx]ᵀ (per unit span, blade 1, frame H) = A_i · [uz, uy, θ, dλ]ᵀ

        uz, uy: section translation at the quarter chord (frame H), θ: section pitch
        about +x (nose-up), dλ: local inflow perturbation [m/s, + down].  Harmonic
        e^{iωt}; ω is the frequency seen in the rotating frame (rad/s).

        Derivation (all perturbations first order, s = iω):
          Section velocity in the inertial frame expressed in H is Ω ẑ×(r+u) + u̇, so
          the in-plane velocity is Ωr + s·uy and the heave rate s·uz (the Ω×u term
          from uy is radial, −Ω uy x̂, and drops out of 2-D strip theory).  Thus
              δU_T = s uy,          δU_P = dλ + s uz
              δW   = (U_T δU_T + U_P δU_P)/W
              δφ   = (U_T δU_P − U_P δU_T)/W²
              δα   = θ − δφ                         (quasi-steady)
              δα_c = δα + b s θ / W                 (3/4-chord pitch-rate term, b=c/2,
                                                     reference axis at c/4, a = −½)
          Lift, drag, moment (q = ½ρc):
              δL = q(2W cl + W² cl_W) δW + q W² cl_α C(k) δα_c + L_nc
              δD = q(2W cd + W² cd_W) δW + q W² cd_α δα
              δM = qc(2W cm + W² cm_W) δW + q c W² cm_α δα + M_nc
          Theodorsen (a = −½) apparent-mass terms, with the normal "plunge" rate
          ḣ = −W δφ (positive down, ⟂ chord; equals −cosφ(s uz + dλ) + sinφ s uy):
              L_nc = πρb² (s ḣ + W s θ + (b/2) s² θ)      (along e_L)
              M_nc = πρb² (−(b/2) s ḣ − W b s θ − (3/8) b² s² θ)   (nose-up, about c/4)
          Projection onto H (from Fz = L cosφ − D sinφ, Fy = −L sinφ − D cosφ):
              dFz = cosβ [cosφ δL − sinφ δD − (L sinφ + D cosφ) δφ]
              dFy = −sinφ δL − cosφ δD − (L cosφ − D sinφ) δφ
              dMx = δM
          C(k) = Theodorsen with k = ω b / W if model == 'theodorsen', 1 if
          'quasi_steady' (apparent mass kept in both), all zeros if 'none'.
          At ω → 0 the θ and dλ columns equal the steady derivatives ∂(Fz,Fy,Mx)/∂θ
          and ∂/∂U_P of :meth:`section_loads`; the uz, uy columns are s × (∂/∂U_P,
          ∂/∂U_T) (tested).  NB: the apparent mass (−ω² πρb² term) is aerodynamic;
          the structure must not add it again.  Sign check: plunge damping
          Im(A[0,0]) < 0, apparent mass Re(A[0,0]) > 0 at high ω (force = −m_a ü).
        """
        model = model or self.design.analysis.aero
        n = len(trim.r)
        if model == "none":
            return [np.zeros((3, 4), complex) for _ in range(n)]
        s = 1j * float(omega)
        U_T, U_P, W, phi = trim.U_T, trim.U_P, trim.W, trim.phi
        c = trim.chord
        b = 0.5 * c
        q = 0.5 * trim.rho * c
        rho = trim.rho
        Ck = theodorsen(abs(omega) * b / W) if model == "theodorsen" else np.ones(n, complex)
        if omega < 0:
            Ck = np.conj(Ck)
        # basis inputs, shape (n, 4): columns uz, uy, theta, dlambda
        Z, O = np.zeros(n, complex), np.ones(n, complex)
        dUT = np.stack([Z, s * O, Z, Z], axis=1)
        dUP = np.stack([s * O, Z, Z, O], axis=1)
        th = np.stack([Z, Z, O, Z], axis=1)
        UTc, UPc, Wc = U_T[:, None], U_P[:, None], W[:, None]
        dW = (UTc * dUT + UPc * dUP) / Wc
        dphi = (UTc * dUP - UPc * dUT) / Wc**2
        da = th - dphi
        da_c = da + b[:, None] * s * th / Wc
        hdot = -Wc * dphi
        pib2 = np.pi * rho * b[:, None] ** 2
        L_nc = pib2 * (s * hdot + Wc * s * th + 0.5 * b[:, None] * s**2 * th)
        M_nc = pib2 * (-0.5 * b[:, None] * s * hdot - Wc * b[:, None] * s * th
                       - 0.375 * b[:, None] ** 2 * s**2 * th)
        qc = q[:, None]
        cl, cd, cm = trim.cl[:, None], trim.cd[:, None], trim.cm[:, None]
        dL = qc * (2 * Wc * cl + Wc**2 * trim.cl_W[:, None]) * dW \
            + qc * Wc**2 * trim.cl_a[:, None] * Ck[:, None] * da_c + L_nc
        dD = qc * (2 * Wc * cd + Wc**2 * trim.cd_W[:, None]) * dW + qc * Wc**2 * trim.cd_a[:, None] * da
        dM = qc * c[:, None] * ((2 * Wc * cm + Wc**2 * trim.cm_W[:, None]) * dW
                                + Wc**2 * trim.cm_a[:, None] * da) + M_nc
        L = qc * Wc**2 * cl
        D = qc * Wc**2 * cd
        cp, sp = np.cos(phi)[:, None], np.sin(phi)[:, None]
        dFz = np.cos(trim.beta)[:, None] * (cp * dL - sp * dD - (L * sp + D * cp) * dphi)
        dFy = -sp * dL - cp * dD - (L * cp - D * sp) * dphi
        A = np.stack([dFz, dFy, dM], axis=1)   # (n, 3, 4)
        return list(A)

    def linearized_array(self, trim: AeroTrim, omega: float, model: Optional[str] = None) -> np.ndarray:
        """Same as :meth:`linearized` stacked to shape (n_stations, 3, 4)."""
        return np.stack(self.linearized(trim, omega, model))

    # ------------------------------------------------------------------ dynamic inflow
    def pitt_peters_hover(self, trim: AeroTrim, omega: float = 0.0) -> np.ndarray:
        """Hover Pitt–Peters cyclic gain G(ω) (complex 2×2, fixed frame):

            [λc, λs]ᵀ = G · [C_Mx, C_My]ᵀ,    C_M = M / (ρ πR² (ΩR)² R)

        Mx, My: aerodynamic hub moment in the fixed frame about +x, +y (right-hand,
        z up), i.e. Mx = Σ∫ y dFz, My = −Σ∫ x dFz.  Inflow (+ down):
        v(r, ψ) = ΩR (r/R)(λc cosψ + λs sinψ).

        Pitt–Peters (hover, χ = 0): M λ' + L⁻¹ λ = C with (in Peters' notation)
        L_cyc = −2/v_m, v_m = 2λ0 (mass-flow parameter in hover, λ0 = √(CT/2)),
        M_cyc = 16/(45π); ' = d/d(Ωt).  In this module's physical sign convention
        (extra lift on the ψ = 0 side ⇒ My < 0 and more downwash there ⇒ λc > 0):

            (16/(45π)) iω̄ λc + λ0 λc = −C_My,     (16/(45π)) iω̄ λs + λ0 λs = +C_Mx,

        ω̄ = ω/Ω, ω the fixed-frame frequency (0 for the steady control moment).
        The static gain 1/λ0 equals annulus-momentum theory (4ρv₀δv per area).
        """
        lam0 = max(trim.lambda0, 1e-4)
        den = lam0 + 1j * (omega / trim.omega) * 16.0 / (45.0 * np.pi)
        return np.array([[0.0, -1.0], [1.0, 0.0]], complex) / den

    def pitt_peters_hover_full(self, trim: AeroTrim, omega: float = 0.0) -> np.ndarray:
        """3×3 version: [λ0', λc, λs] = G · [C_T', C_Mx, C_My] (collective row:
        (128/(75π)) iω̄ λ0' + 4λ0 λ0' = C_T', from L_00 = 1/(2v_m), v_m = 2λ0)."""
        lam0 = max(trim.lambda0, 1e-4)
        wb = omega / trim.omega
        G = np.zeros((3, 3), complex)
        G[0, 0] = 1.0 / (4 * lam0 + 1j * wb * 128.0 / (75.0 * np.pi))
        G[1:, 1:] = self.pitt_peters_hover(trim, omega)
        return G

    @staticmethod
    def moment_coeff(trim: AeroTrim, M):
        """C_M = M / (ρ πR² (ΩR)² R)."""
        return np.asarray(M) / (trim.rho * np.pi * trim.R**3 * (trim.omega * trim.R) ** 2)

    def cyclic_inflow(self, trim: AeroTrim, Mx: complex, My: complex,
                      omega_fixed: float = 0.0) -> CyclicInflow:
        """Cyclic inflow produced by a fixed-frame aerodynamic hub moment harmonic
        (Mx, My in N m; complex amplitudes at omega_fixed, or real for steady).

        Typical use by the rotor model (steady cyclic control):
            ci = aero.cyclic_inflow(trim, Mx_mean, My_mean)
            dlam_blade_k = ci.blade_1p(trim.r, blade=k, n_blades=B)   # m/s, e^{iΩt}
        then feed dlam into column 3 of :meth:`linearized` and iterate / solve the
        linear loop (moment depends on dλ).
        """
        G = self.pitt_peters_hover(trim, omega_fixed)
        lc, ls = G @ np.array([self.moment_coeff(trim, Mx), self.moment_coeff(trim, My)], complex)
        return CyclicInflow(complex(lc), complex(ls), trim.omega, trim.R, omega_fixed)

    @staticmethod
    def mean_hub_moment(trim: AeroTrim, dFz_blades: np.ndarray) -> tuple[float, float]:
        """Mean (0/rev) fixed-frame hub moment from 1/rev rotating-frame lift.

        dFz_blades: complex (B, n_stations), amplitude of dFz per unit span on blade
        k at e^{iΩt}; blade k azimuth ψ_k = Ωt + ψ0_k, ψ0_k = 2π(k−1)/B.
            Mx = Σ_k ∫ r sinψ_k Fz_k dr,   My = −Σ_k ∫ r cosψ_k Fz_k dr
        Time-mean of Re{F e^{iΩt}}·Re{b e^{iΩt}} is ½Re{F b̄}; sinψ_k → b = −i e^{iψ0},
        cosψ_k → b = e^{iψ0}:
            Mx = ½ Σ Re{ i e^{−iψ0_k} ∫ r F_k dr },  My = −½ Σ Re{ e^{−iψ0_k} ∫ r F_k dr }
        """
        F = np.atleast_2d(np.asarray(dFz_blades, complex))
        B = F.shape[0]
        I = F @ (trim.w_int * trim.r)
        e = np.exp(-1j * 2 * np.pi * np.arange(B) / B)
        Mx = 0.5 * float(np.sum(np.real(1j * e * I)))
        My = -0.5 * float(np.sum(np.real(e * I)))
        return Mx, My

    # ------------------------------------------------------------------ diagnostics
    def unsteady_summary(self, trim: AeroTrim, omega: Optional[float] = None) -> dict:
        """Reduced frequency k = ω c /(2W) per station and Theodorsen C(k);
        default ω = Ω (1/rev).  Also the Pitt–Peters cyclic time constant."""
        omega = trim.omega if omega is None else omega
        k = omega * trim.chord / (2 * trim.W)
        C = theodorsen(k)
        tau = 16.0 / (45.0 * np.pi * max(trim.lambda0, 1e-4)) / trim.omega
        return dict(r=trim.r, W=trim.W, k=k, C_re=C.real, C_im=C.imag, C_abs=np.abs(C),
                    C_phase_deg=np.degrees(np.angle(C)), k_75=float(np.interp(0.75 * trim.R, trim.r, k)),
                    pitt_peters_tau_s=tau, pitt_peters_tau_rev=tau * trim.omega / (2 * np.pi))


def unsteady_summary(trim: AeroTrim, omega: Optional[float] = None) -> dict:
    """Module-level alias of :meth:`AeroModel.unsteady_summary` (needs only `trim`)."""
    return AeroModel.unsteady_summary(None, trim, omega)  # type: ignore[arg-type]
