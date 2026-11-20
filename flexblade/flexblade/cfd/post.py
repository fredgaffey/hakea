"""Parsers for OpenFOAM function-object output and harmonic (amplitude/phase) fitting."""
from __future__ import annotations

import re
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.special import hankel2


def read_dat(path: str | Path) -> tuple[list[str], np.ndarray]:
    """Read a postProcessing .dat file (forceCoeffs coefficient.dat, forces force.dat …).

    Column names come from the last '#' header line (e.g. '# Time Cd Cl CmPitch').
    Vector entries in parentheses '(x y z)' are expanded into name_x/_y/_z columns.
    """
    names: list[str] = []
    rows = []
    for line in Path(path).read_text().splitlines():
        s = line.strip()
        if not s:
            continue
        if s.startswith("#"):
            toks = s.lstrip("#").split()
            if toks and toks[0].lower() == "time":
                names = toks
            continue
        nums = re.findall(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|nan|inf", s)
        rows.append([float(x) for x in nums])
    if not rows:
        return names, np.zeros((0, len(names)))
    n = min(len(r) for r in rows)
    A = np.array([r[:n] for r in rows])
    if names and len(names) != n:
        # parenthesised vectors: names like 'total' expand to 3 columns
        expanded = [names[0]]
        rest = names[1:]
        ncomp = (n - 1) // max(len(rest), 1)
        for nm in rest:
            expanded += [nm] if ncomp == 1 else [f"{nm}_{c}" for c in "xyz"[:ncomp]]
        names = expanded[:n]
    if not names:
        names = ["Time"] + [f"c{i}" for i in range(1, n)]
    # keep last value for duplicated times (restarts)
    _, idx = np.unique(A[::-1, 0], return_index=True)
    A = A[::-1][idx]
    return names, A


def find_dat(case: str | Path, func: str, fname: str) -> list[Path]:
    """All postProcessing/<func>/<time>/<fname>* files sorted by start time."""
    base = Path(case) / "postProcessing" / func
    if not base.exists():
        return []
    files = []
    for d in base.iterdir():
        for f in d.glob(fname + "*"):
            try:
                files.append((float(d.name), f))
            except ValueError:
                files.append((0.0, f))
    return [f for _, f in sorted(files)]


def load_function_object(case, func: str, fname: str) -> dict[str, np.ndarray]:
    out: dict[str, list] = {}
    for f in find_dat(case, func, fname):
        names, A = read_dat(f)
        for i, nm in enumerate(names):
            out.setdefault(nm, []).append(A[:, i])
    return {k: np.concatenate(v) for k, v in out.items()}


def harmonic_fit(t, y, omega: float, n_periods: Optional[float] = 2.0, n_harm: int = 2) -> dict:
    """Least-squares y ≈ y0 + Σ_n Re{Y_n e^{i n ω t}} over the last n_periods.

    Returns dict(mean, amp, phase_deg, Y (complex 1st harmonic), harmonics [Y_n],
    residual_rms). Convention x(t) = Re{X e^{iωt}}: y = a cos ωt + b sin ωt → Y = a − i b.
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    if n_periods:
        T = 2 * np.pi / omega
        m = t >= t.max() - n_periods * T
        t, y = t[m], y[m]
    cols = [np.ones_like(t)]
    for n in range(1, n_harm + 1):
        cols += [np.cos(n * omega * t), np.sin(n * omega * t)]
    A = np.column_stack(cols)
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    Ys = [c[2 * n - 1] - 1j * c[2 * n] for n in range(1, n_harm + 1)]
    res = y - A @ c
    Y = Ys[0]
    return dict(mean=float(c[0]), amp=float(abs(Y)), phase_deg=float(np.degrees(np.angle(Y))),
                Y=Y, harmonics=Ys, residual_rms=float(np.sqrt(np.mean(res ** 2))),
                n_samples=int(len(t)))


def theodorsen_C(k: float) -> complex:
    if k < 1e-9:
        return 1.0 + 0j
    h1, h0 = hankel2(1, k), hankel2(0, k)
    return complex(h1 / (h1 + 1j * h0))


def theodorsen_pitch_cl(k: float, pivot_chord_frac: float = 0.25,
                        cl_alpha: float = 2 * np.pi) -> complex:
    """Thin-airfoil Cl per unit pitch amplitude (complex, x=Re{X e^{iωt}}), pitch about
    pivot at x/c = pivot_chord_frac; a = 2·x_p/c − 1 (semichords aft of midchord).

    Cl/α = (cl_α/2π)·[π(i k + a k²) + 2π C(k)(1 + i k (½ − a))]
    """
    a = 2 * pivot_chord_frac - 1
    C = theodorsen_C(k)
    return (cl_alpha / (2 * np.pi)) * (np.pi * (1j * k + a * k ** 2)
                                       + 2 * np.pi * C * (1 + 1j * k * (0.5 - a)))


def pitching_response(case, alpha_amp_deg: float, omega: float, func: str = "forceCoeffs",
                      n_periods: float = 2.0, coeffs=("Cl", "Cd", "CmPitch")) -> dict:
    """Amplitude/phase of force coefficients for α(t) = α0 + A sin(ωt) (= Re{−iA e^{iωt}}).

    Transfer per radian of pitch: H = Y / (−i A). Returned per coefficient:
    mean, amp, phase_deg (of the coefficient itself, vs cos ωt), H, |H|, phase_H_deg
    (lag of the coefficient relative to α; negative = lags).
    """
    data = load_function_object(case, func, "coefficient")
    if not data:
        data = load_function_object(case, func, "forceCoeffs")
    t = data.get("Time")
    if t is None:
        raise FileNotFoundError(f"no {func} output under {case}/postProcessing")
    A = np.radians(alpha_amp_deg)
    out = {}
    for c in coeffs:
        if c not in data:
            continue
        fit = harmonic_fit(t, data[c], omega, n_periods)
        H = fit["Y"] / (-1j * A)
        fit.update(H=H, H_abs=float(abs(H)), phase_H_deg=float(np.degrees(np.angle(H))))
        out[c] = fit
    return out


def hover_forces(case, n_blades: int, func: str = "forces", n_last: int = 200) -> dict:
    """Thrust/torque from a single-blade sector run: mean of last n_last iterations,
    times n_blades. Force on the blade by the fluid: thrust = +Fz; torque Q = −Mz."""
    f = load_function_object(case, func, "force")
    m = load_function_object(case, func, "moment")
    if not f or not m:
        raise FileNotFoundError(f"no {func} force/moment output under {case}/postProcessing")

    def col(d, *names):
        for nm in names:
            if nm in d:
                return d[nm]
        raise KeyError(f"none of {names} in {list(d)}")
    Fz = col(f, "total_z", "Fz", "total(z)")
    Mz = col(m, "total_z", "Mz", "total(z)")
    n = min(n_last, len(Fz))
    T = n_blades * float(np.mean(Fz[-n:]))
    Q = -n_blades * float(np.mean(Mz[-n:]))
    return dict(thrust=T, torque=Q, thrust_std=n_blades * float(np.std(Fz[-n:])),
                torque_std=n_blades * float(np.std(Mz[-n:])), n_iter=int(len(Fz)))
