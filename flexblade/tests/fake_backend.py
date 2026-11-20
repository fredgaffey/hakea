"""Fake physics backend for UI tests and demos (OWNER: ui).

Monkeypatches flexblade.api.{evaluate, optimize, export_geometry, presets} with cheap,
deterministic, schema-valid stand-ins so the CLI / MCP / GUI can be exercised before the
real modules exist. Numbers are *plausible-looking only*; they are not physics.

    import fake_backend; fake_backend.install()                 # patch everything
    fake_backend.install(only_missing=True)                     # patch only what doesn't import
    FLEXBLADE_FAKE=1 flexblade gui                              # same, via the CLI (dev only)
"""
from __future__ import annotations

import importlib
import json
import math
import random
from pathlib import Path

import numpy as np

from flexblade import api
from flexblade.bladegeom import airfoil_coords, resolve_stations, section_polygon_H
from flexblade.schema import (BladeHarmonic, CampbellResult, Check, CoaxialResult, Design, HarmonicResult, Mode,
                              OptimizeRequest, OptimizeResult, Results, TrimResult)

G = 9.80665


def _d(design) -> Design:
    return design if isinstance(design, Design) else Design.model_validate(design)


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


# --------------------------------------------------------------------------- evaluate


def _lower_design(d: Design) -> Design:
    c = d.coaxial
    upd = {"layout": "single"}
    if c.lower_blade is not None:
        upd["blade"] = c.lower_blade
    if c.lower_flexure is not None:
        upd["flexure"] = c.lower_flexure
    if c.lower_motor is not None:
        upd["motor"] = c.lower_motor
    op = d.op.model_copy(update={"collective_deg": d.op.collective_deg + c.lower_collective_offset_deg,
                                 "rpm": c.lower_rpm if c.trim == "fixed_rpm" else d.op.rpm,
                                 "torque_mod_frac": c.lower_torque_mod_frac if c.lower_torque_mod_frac is not None
                                 else d.op.torque_mod_frac, "thrust_share": 0.48})
    upd["op"] = op
    return d.model_copy(update=upd)


def _mirror_lower(h: HarmonicResult, z0: float) -> None:
    """Express the lower rotor's nodal field in hub frame H: mirror y (counter-rotating) and drop by z0."""
    if not h.nodes:
        return
    h.nodes = [(x, -y, z + z0) for x, y, z in h.nodes]
    for arr in (h.displacement_re, h.displacement_im):
        for row in arr:
            row[1], row[3], row[5] = -row[1], -row[3], -row[5]


def fake_evaluate(design, what=None) -> Results:
    d = _d(design)
    if d.layout == "coaxial":
        return _fake_coaxial(d, what)
    return _fake_single(d, what)


def _fake_coaxial(d: Design, what) -> Results:
    du = d.model_copy(update={"layout": "single", "op": d.op.model_copy(update={"thrust_share": 0.52})})
    dl = _lower_design(d)
    up, lo = _fake_single(du, what), _fake_single(dl, what)
    R = d.blade.radius
    if lo.harmonic:
        _mirror_lower(lo.harmonic, -d.coaxial.spacing_frac * R)
        for b in lo.harmonic.blades:
            b.pitch_phase_deg, b.lag_phase_deg, b.flap_phase_deg = -b.pitch_phase_deg, -b.lag_phase_deg, -b.flap_phase_deg
    W = d.op.vehicle_mass * G
    Mmax = (up.harmonic.hub_moment_mean if up.harmonic else 0) + (lo.harmonic.hub_moment_mean if lo.harmonic else 0)
    P1 = up.trim.power + lo.trim.power
    coax = CoaxialResult(
        upper=up, lower=lo, thrust_total=up.trim.thrust + lo.trim.thrust, power_total=P1,
        yaw_torque_net=up.trim.torque - lo.trim.torque, thrust_share_upper=0.52,
        rpm_upper=up.trim.rpm, rpm_lower=lo.trim.rpm, figure_of_merit=0.58,
        control_moment_max=Mmax, control_ratio=Mmax / (W * R), yaw_authority=0.3 * up.trim.torque,
        moment_2p_combined=0.2 * Mmax, interference_power_factor=1.12)
    res = up.model_copy(deep=True)
    res.design_hash = d.hash()
    res.coaxial = coax
    res.checks = [c.model_copy(update={"member": f"upper_{c.member}"}) for c in up.checks] + \
                 [c.model_copy(update={"member": f"lower_{c.member}"}) for c in lo.checks]
    res.summary = dict(up.summary, control_ratio=coax.control_ratio, power_total=P1,
                       yaw_authority=coax.yaw_authority,
                       max_utilisation=max(c.utilisation for c in res.checks))
    return res


def _fake_single(d: Design, what=None) -> Results:
    what = what or ["trim", "modes", "harmonic", "checks"]
    R = d.blade.radius
    fx = d.flexure
    T = d.op.vehicle_mass * G * (d.op.thrust_share or 1.0)
    rpm = d.op.rpm or 60.0 * math.sqrt(T / (d.op.rho * 0.1 * (2 * R) ** 4))
    Om = rpm * 2 * math.pi / 60
    n_rps = rpm / 60
    P = 0.045 * d.op.rho * n_rps ** 3 * (2 * R) ** 5
    Q = P / Om if Om else 0.0
    blade_mass = 0.0035 * (R / 0.0635) ** 3
    cf = blade_mass * Om ** 2 * 0.55 * R
    E = {"tpu_95a": 26e6, "tpu_64d": 90e6, "pa6_cf": 7.5e9, "petg_cf": 4e9}.get(fx.layup.material, 1e9)
    k_lag = E * fx.width * fx.thickness ** 3 / 12 / max(fx.length, 1e-4)
    J_blade = blade_mass * (0.55 * R) ** 2
    f_lag = math.sqrt(k_lag / J_blade + (0.3 * Om) ** 2) / (2 * math.pi)
    skew = math.radians(fx.skew_deg)
    res = Results(design_hash=d.hash())
    stations = resolve_stations(d.blade)
    r_st = [s.r_frac * R for s in stations]
    res.trim = TrimResult(
        rpm=rpm, thrust=T, torque=Q, power=P, figure_of_merit=0.62,
        collective_deg=d.op.collective_deg + 2.0, centrifugal_force_root=cf,
        tip_deflection=(0.0, -0.002 * R, 0.01 * R), coning_deg=3.1,
        root_pitch_change_deg=-0.4 * math.tan(skew), radial_stretch=0.01 * cf / (E * fx.width * fx.thickness) * fx.length,
        stations={"r": r_st, "alpha_deg": [6 - 2 * s.r_frac for s in stations],
                  "cl": [0.7] * len(stations), "cd": [0.02] * len(stations),
                  "Re": [3e4 * s.r_frac / 0.5 for s in stations],
                  "dT_dr": [T / R * 2 * s.r_frac ** 2 for s in stations]})
    if "modes" in what or "campbell" in what:
        labels = ["hub torsion", "lag sym", "lag antisym", "flap sym", "flap antisym", "pitch"]
        base = [3.2, f_lag / max(rpm / 60, 1e-3), f_lag / max(rpm / 60, 1e-3) * 1.05, 1.1, 1.12, 4.5]
        res.modes = [Mode(index=i, freq_hz=b * rpm / 60, damping_ratio=0.05 + 0.01 * i, label=lab,
                          participation={"lag": 0.8 if "lag" in lab else 0.1})
                     for i, (lab, b) in enumerate(zip(labels, base))]
        grid = d.op.rpm_grid or list(np.linspace(0, 1.3 * rpm, 14))
        freqs = []
        for rr in grid:
            om = rr * 2 * math.pi / 60
            fl = math.sqrt(k_lag / J_blade + (0.3 * om) ** 2) / (2 * math.pi)
            ff = math.sqrt((40.0) ** 2 + (1.05 * om / (2 * math.pi)) ** 2)
            freqs.append([3.2 * rpm / 60 * (0.3 + 0.7 * rr / max(rpm, 1)), fl, fl * 1.05, ff, ff * 1.02,
                          math.sqrt(300 ** 2 + (om / (2 * math.pi)) ** 2)])
        res.campbell = CampbellResult(rpm=[float(x) for x in grid], freqs_hz=freqs, labels=labels)
    if "harmonic" in what:
        frac = f_lag / (rpm / 60)
        lag_amp = 3.0 / max(abs(frac ** 2 - 1.0), 0.15)
        lag_amp = min(lag_amp, 12.0)
        lag_ph = -90.0 if frac > 1 else 90.0
        pitch = lag_amp * math.tan(skew) * 0.6
        blades = []
        for k in (1, 2):
            sgn = 1 if k == 1 else -1
            blades.append(BladeHarmonic(blade=k, pitch_amp_deg=pitch, pitch_phase_deg=lag_ph + (0 if sgn > 0 else 180),
                                        lag_amp_deg=lag_amp, lag_phase_deg=lag_ph,
                                        flap_amp_deg=0.3 * pitch, flap_phase_deg=lag_ph - 60))
        Mh = 0.02 * pitch * T * R
        nodes, re_, im_ = _beam_field(d, blades)
        res.harmonic = HarmonicResult(
            torque_amp=d.op.torque_mod_frac * Q, hub_speed_amp_rpm=25 * d.op.torque_mod_frac,
            blades=blades, hub_moment_mean=Mh, hub_moment_phase_deg=lag_ph + 15,
            hub_moment_2p=0.3 * Mh, thrust_1p=0.02 * T, moment_per_torque=Mh / max(d.op.torque_mod_frac * Q, 1e-9),
            control_ratio=Mh / (T * R), tilt_equiv_deg=math.degrees(math.atan(Mh / (T * R * 0.75))),
            nodes=nodes, displacement_re=re_, displacement_im=im_)
    if "checks" in what:
        strain = fx.thickness / 2 * math.radians(3.0) / max(fx.length, 1e-4)
        res.checks = [
            Check(name="bending_strain_amp", member="flexure_a", value=strain, allowable=0.02,
                  utilisation=strain / 0.02, note="1/rev lag amplitude"),
            Check(name="creep_strain", member="flexure_a", value=res.trim.radial_stretch / fx.length,
                  allowable=0.015, utilisation=res.trim.radial_stretch / fx.length / 0.015),
            Check(name="interlayer_stress", member="blade_root", value=cf / (fx.width * fx.thickness) * 0.02,
                  allowable=20e6, utilisation=cf / (fx.width * fx.thickness) * 0.02 / 20e6),
            Check(name="tip_clearance", member="blade", value=0.2, allowable=1.0, utilisation=0.2),
        ]
    res.summary = {"lag_freq_per_rev": f_lag / (rpm / 60), "control_ratio":
                   res.harmonic.control_ratio if res.harmonic else 0.0,
                   "max_utilisation": max((c.utilisation for c in res.checks), default=0.0),
                   "blade_mass": blade_mass}
    res.warnings = ["FAKE BACKEND: numbers are placeholders, not physics"]
    res.timings_s = {"total": 0.001}
    return res


def _beam_field(d: Design, blades: list[BladeHarmonic]):
    """Beam nodes of all blades (frame H) and complex 1/rev nodal displacements."""
    R = d.blade.radius
    rh = d.flexure.hinge_radius
    rs = list(np.linspace(d.flexure.hub_radius, rh + d.flexure.length / 2, 4)) + \
        list(np.linspace(rh + d.flexure.length, R, 10))
    nodes, re_, im_ = [[0.0, 0.0, 0.0]], [[0.0] * 6], [[0.0] * 6]
    n = d.blade.n_blades
    for k in range(n):
        Rk = _rot_z(2 * math.pi * k / n)
        b = blades[min(k, len(blades) - 1)]
        th = math.radians(b.pitch_amp_deg) * np.exp(1j * math.radians(b.pitch_phase_deg))
        ze = math.radians(b.lag_amp_deg) * np.exp(1j * math.radians(b.lag_phase_deg))
        be = math.radians(b.flap_amp_deg) * np.exp(1j * math.radians(b.flap_phase_deg))
        for r in rs:
            arm = max(r - rh, 0.0)
            w = min(max((r - d.flexure.hub_radius) / max(rh - d.flexure.hub_radius, 1e-6), 0), 1)
            u_loc = np.array([0, -ze * arm, be * arm], complex)
            r_loc = np.array([th * w, -be * w, -ze * w], complex)
            u = Rk @ u_loc
            rot = Rk @ r_loc
            p = Rk @ np.array([r, 0, 0])
            nodes.append([float(x) for x in p])
            re_.append([float(x) for x in np.concatenate([u.real, rot.real])])
            im_.append([float(x) for x in np.concatenate([u.imag, rot.imag])])
    return nodes, re_, im_


# --------------------------------------------------------------------------- optimize


def fake_optimize(request, progress=None) -> OptimizeResult:
    from flexblade.uicommon import get_path, set_values
    req = request if isinstance(request, OptimizeRequest) else OptimizeRequest.model_validate(request)
    rng = random.Random(req.seed)
    best = None
    hist = []
    n = min(req.max_evals, 60)
    for i in range(n):
        vals = {}
        for v in req.variables:
            u = rng.random()
            vals[v.path] = v.lo * (v.hi / v.lo) ** u if v.log else v.lo + (v.hi - v.lo) * u
        d = set_values(req.design, vals)
        r = fake_evaluate(d)
        obj = 0.0
        for t in req.targets:
            q = float(get_path(r.model_dump(mode="json"), t.quantity)) if not t.quantity.startswith("checks.max") \
                else max(c.utilisation for c in r.checks)
            tv = t.value or 0.0
            obj += t.weight * {"min": q, "max": -q, "eq": (q - tv) ** 2,
                               "le": max(q - tv, 0) ** 2, "ge": max(tv - q, 0) ** 2}[t.kind]
        hist.append({"eval": i, "objective": obj, "values": vals})
        if best is None or obj < best[0]:
            best = (obj, d, r)
        if progress:
            progress(i + 1, n, obj, best[0])
    return OptimizeResult(design=best[1], results=best[2], objective=best[0], n_evals=n, history=hist,
                          converged=True, message="fake random search")


# --------------------------------------------------------------------------- geometry


def _box(p0, p1, width, thick, wdir):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    ax = p1 - p0
    ax /= np.linalg.norm(ax)
    w = np.asarray(wdir, float)
    w = w - ax * (w @ ax)
    w /= np.linalg.norm(w)
    t = np.cross(ax, w)
    V = []
    for p in (p0, p1):
        for sw in (-1, 1):
            for st in (-1, 1):
                V.append(p + sw * w * width / 2 + st * t * thick / 2)
    F = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
         [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
    return np.array(V), F


def fake_geometry(design) -> dict:
    d = _d(design)
    if d.layout != "coaxial":
        return _fake_geometry_single(d)
    up = _fake_geometry_single(d)
    lo = _fake_geometry_single(_lower_design(d))
    z0 = -d.coaxial.spacing_frac * d.blade.radius
    parts = [dict(p, name=f"upper_{p['name']}", rotor="upper") for p in up["parts"]]
    for p in lo["parts"]:
        V = [[x, -y, z + z0] for x, y, z in p["vertices"]]
        parts.append(dict(p, name=f"lower_{p['name']}", rotor="lower", vertices=V))
    return {"units": "m", "frame": "H", "parts": parts}


def _fake_geometry_single(d: Design) -> dict:
    R = d.blade.radius
    fx = d.flexure
    parts = []
    # hub: hexagonal-ish cylinder
    nseg = 24
    V, F = [], []
    for z in (-0.004, 0.004):
        for i in range(nseg):
            a = 2 * math.pi * i / nseg
            V.append([fx.hub_radius * math.cos(a), fx.hub_radius * math.sin(a), z])
    V += [[0, 0, -0.004], [0, 0, 0.004]]
    for i in range(nseg):
        j = (i + 1) % nseg
        F += [[i, j, nseg + j], [i, nseg + j, nseg + i], [2 * nseg, j, i], [2 * nseg + 1, nseg + i, nseg + j]]
    parts.append(dict(name="hub", role="hub", blade=None, material="pa6_cf", vertices=V, faces=F))
    st = resolve_stations(d.blade)
    for k in range(d.blade.n_blades):
        Rk = _rot_z(2 * math.pi * k / d.blade.n_blades)
        sk = math.radians(fx.skew_deg) * (1 if k % 2 == 0 else -1)
        wdir = (0.0, math.sin(sk), math.cos(sk))
        for m, ang in (("a", 1), ("b", -1)):
            a = math.radians(fx.cross_angle_deg / 2) * ang * 0.3
            p0 = [fx.hub_radius, 0, 0]
            p1 = [fx.hinge_radius + fx.length / 2, 0, 0]
            V, F = _box(p0, p1, fx.width, fx.thickness, (wdir[0], wdir[1] + a, wdir[2]))
            parts.append(dict(name=f"blade{k + 1}_flexure_{m}", role="flexure", blade=k + 1,
                              material=fx.layup.material, vertices=(V @ Rk.T).tolist(), faces=F))
        xy = airfoil_coords(d.blade.airfoil, None, n=21)
        rings = []
        r0 = fx.hinge_radius + fx.length / 2
        for s in st:
            r = max(s.r_frac * R, r0)
            rings.append(section_polygon_H(xy, s.chord, math.radians(s.twist_deg + d.op.collective_deg), r))
        npt = len(xy)
        V = np.vstack(rings) @ Rk.T
        F = []
        for i in range(len(rings) - 1):
            for j in range(npt - 1):
                a0, a1 = i * npt + j, i * npt + j + 1
                b0, b1 = a0 + npt, a1 + npt
                F += [[a0, b0, b1], [a0, b1, a1]]
        last = (len(rings) - 1) * npt
        for j in range(1, npt - 2):
            F += [[0, j + 1, j], [last, last + j, last + j + 1]]
        parts.append(dict(name=f"blade{k + 1}", role="blade", blade=k + 1, material=d.blade.layup.material,
                          vertices=V.tolist(), faces=F))
    return {"units": "m", "frame": "H", "parts": parts}


STEP_TEMPLATE = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('flexblade FAKE geometry placeholder'),'2;1');
FILE_NAME('{name}.step','',(''),(''),'flexblade-fake','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));
ENDSEC;
DATA;
ENDSEC;
END-ISO-10303-21;
"""


def fake_export(design, fmt, path) -> str:
    d = _d(design)
    g = fake_geometry(d)
    p = Path(path)
    if fmt == "json":
        p.write_text(json.dumps(g))
    elif fmt in ("stl", "glb"):
        import trimesh
        meshes = {pt["name"]: trimesh.Trimesh(pt["vertices"], pt["faces"], process=False) for pt in g["parts"]}
        if fmt == "stl":
            trimesh.util.concatenate(list(meshes.values())).export(p)
        else:
            trimesh.Scene(meshes).export(p)
    elif fmt == "step":
        p.write_text(STEP_TEMPLATE.format(name=d.name))
    else:
        raise ValueError(f"unknown fmt {fmt!r}")
    return str(p)


# --------------------------------------------------------------------------- presets


def fake_presets() -> dict[str, Design]:
    out = {}
    for dia in (5, 7, 10):
        for mat in ("tpu_64d", "pa6_cf"):
            d = Design(name=f"fake_{dia}in_{mat}")
            d.blade.diameter_in = dia
            d.flexure.layup.material = mat
            if mat == "pa6_cf":
                d.flexure.thickness = 0.0006
            out[d.name] = Design.model_validate(d.model_dump())
    return out


# --------------------------------------------------------------------------- install

_PATCHES = {"evaluate": ("rotor", fake_evaluate), "optimize": ("optimize", fake_optimize),
            "export_geometry": ("geometry", fake_export), "presets": ("presets", fake_presets)}


def _module_ok(name: str) -> bool:
    try:
        importlib.import_module(f"flexblade.{name}")
        return True
    except Exception:  # noqa: BLE001
        return False


def install(monkeypatch=None, only_missing: bool = False) -> list[str]:
    """Patch api functions; returns names patched. With pytest's monkeypatch they auto-revert."""
    patched = []
    for fn, (mod, fake) in _PATCHES.items():
        if only_missing and _module_ok(mod):
            continue
        if monkeypatch is not None:
            monkeypatch.setattr(api, fn, fake)
        else:
            setattr(api, fn, fake)
        patched.append(fn)
    return patched
