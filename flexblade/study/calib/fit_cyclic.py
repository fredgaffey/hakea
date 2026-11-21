"""Offline bench-log -> cyclic gain/phase schedule (per-build calibration, BUILD.md item 4-FC schedule).

Log CSV columns (header names, case-insensitive; aliases in brackets):
  t [s], erpm, azimuth_deg [azimuth, az_deg] OR rev [cumulative mech revs]  (if neither: integrated
  from erpm/pole_pairs), cmd_amp [pulse amplitude, N m or any unit], cmd_phase_deg [cmd_phase],
  mx,my [imu_x,imu_y | Mx,My]  fixed-frame moment proxy (frame fixed to the motor can).
Command convention (same as rotor.harmonic_1p): torque pulse = cmd_amp*cos(psi - cmd_phase), psi = azimuth;
the fixed-frame hub moment is M = mx + j my = G * cmd_amp * exp(j cmd_phase) + nuisance, G = |G|exp(j phi),
phi = moment azimuth minus torque-peak azimuth (the quantity the FC schedules).
The command phase MUST be rotated/dithered during the log (e.g. 1-3 Hz ramp, or +-90 deg steps) so G is
observable apart from DC/gravity and 1P imbalance vibration; the fit warns if the design matrix is ill-conditioned.

Per rpm window: complex least squares of z = mx + j my on [u = cmd_amp e^{j cmd_phase}, 1, e^{+-jk psi}
(k=1..K, synchronous lock-in terms that absorb nP vibration)]  -> G_w.  Then a Huber-IRLS (median-like,
robust to outlier windows) fit of ln|G| and unwrapped phase as piecewise-linear functions of ln(erpm) at N nodes,
with 95 % CIs from the robust residual scale. Gain scale is that of the sensor (N m for a load cell;
IMU: multiply by the bench sensitivity via --sens).

  python fit_cyclic.py log.csv --nodes 5 --pole-pairs 7 --out schedule   -> schedule.csv, schedule.md
"""
from __future__ import annotations
import argparse, csv, sys
from pathlib import Path
import numpy as np

ALIASES = dict(t=["t", "time"], erpm=["erpm"], az=["azimuth_deg", "azimuth", "az_deg"], rev=["rev", "revs"],
               amp=["cmd_amp", "amp", "pulse_amp"], ph=["cmd_phase_deg", "cmd_phase", "phase_deg"],
               mx=["mx", "imu_x", "ax", "x"], my=["my", "imu_y", "ay", "y"])


def lockin(y, phase, harmonic=1):
    """Synchronous demodulation of y against a reference phase [rad]: complex amplitude a such that
    y ~ Re(a e^{j h phase}) for real y (whole cycles in the record); for complex y returns <y e^{-j h phase}>."""
    y = np.asarray(y)
    e = np.exp(-1j * harmonic * np.asarray(phase))
    return 2 * np.mean(y * e) if not np.iscomplexobj(y) else np.mean(y * e)


def read_log(path, pole_pairs=7):
    with open(path, newline="") as f:
        rd = csv.DictReader(f)
        cols = {k.strip().lower(): k for k in rd.fieldnames}
        pick = lambda key, req=True: next((cols[a] for a in ALIASES[key] if a in cols), None)  # noqa: E731
        m = {k: pick(k) for k in ALIASES}
        for k in ("t", "erpm", "amp", "ph", "mx", "my"):
            if m[k] is None:
                raise SystemExit(f"missing column for {k}: one of {ALIASES[k]}")
        rows = list(rd)
    g = lambda key: np.array([float(r[m[key]]) for r in rows])  # noqa: E731
    t, erpm = g("t"), g("erpm")
    if m["az"]:
        psi = np.radians(g("az"))
    elif m["rev"]:
        psi = 2 * np.pi * g("rev")
    else:
        w = erpm / pole_pairs * 2 * np.pi / 60
        psi = np.concatenate([[0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(t))])
    return dict(t=t, erpm=erpm, psi=psi, amp=g("amp"), ph=np.radians(g("ph")), z=g("mx") + 1j * g("my"))


def window_G(z, psi, amp, ph, K=4):
    """Complex LS gain of one window. -> (G, sigma_G (complex-LS std of G), cond)."""
    n = len(z)
    u = amp * np.exp(1j * ph)
    cols = [u, np.ones(n, complex)] + [np.exp(1j * k * psi) for k in range(-K, K + 1) if k != 0]
    A = np.stack(cols, 1)
    coef, *_ = np.linalg.lstsq(A, z, rcond=None)
    res = z - A @ coef
    s2 = np.real(np.vdot(res, res)) / max(n - A.shape[1], 1)
    cov = s2 * np.linalg.inv(A.conj().T @ A)
    return coef[0], float(np.sqrt(np.real(cov[0, 0]))), float(np.linalg.cond(A))


def windows(log, width=1.0, step=0.5, min_amp=1e-9, K=4):
    t = log["t"]
    out = []
    t0 = t[0]
    while t0 + width <= t[-1] + 1e-9:
        i = (t >= t0) & (t < t0 + width)
        if i.sum() > 40 and np.mean(np.abs(log["amp"][i])) > min_amp:
            G, s, c = window_G(log["z"][i], log["psi"][i], log["amp"][i], log["ph"][i], K)
            out.append((np.exp(np.mean(np.log(log["erpm"][i]))), G, s, c))
        t0 += step
    if not out:
        raise SystemExit("no usable windows (need command amplitude > 0 and enough samples)")
    a = np.array(out)
    return dict(erpm=a[:, 0].real, G=a[:, 1], sig=a[:, 2].real, cond=a[:, 3].real)


def _hat(x, nodes):
    """piecewise-linear basis in x (rows) at nodes (cols)."""
    B = np.zeros((len(x), len(nodes)))
    j = np.clip(np.searchsorted(nodes, x) - 1, 0, len(nodes) - 2)
    f = (x - nodes[j]) / (nodes[j + 1] - nodes[j])
    B[np.arange(len(x)), j] = 1 - f
    B[np.arange(len(x)), j + 1] = f
    return B


def _huber_fit(B, y, iters=30, c=1.5):
    w = np.ones(len(y))
    for _ in range(iters):
        W = np.sqrt(w)[:, None]
        beta, *_ = np.linalg.lstsq(B * W, y * W[:, 0], rcond=None)
        r = y - B @ beta
        s = max(1.4826 * np.median(np.abs(r - np.median(r))), 1e-12)
        w = np.minimum(1.0, c * s / np.maximum(np.abs(r), 1e-300))
    W = w[:, None]
    cov = s ** 2 * np.linalg.pinv((B * W).T @ B)
    return beta, np.sqrt(np.diag(cov)), s


def fit_schedule(win, nodes=5, edge_pct=2.0):
    """-> rows (erpm, gain, phase_deg, gain_ci, phase_ci). CIs are 95 % half-widths (gain abs, phase deg)."""
    if nodes < 2:
        raise ValueError("nodes >= 2")
    x = np.log(win["erpm"])
    lo, hi = np.percentile(x, edge_pct), np.percentile(x, 100 - edge_pct)
    keep = (x >= lo) & (x <= hi)
    xn = np.linspace(lo, hi, nodes)
    B = _hat(x[keep], xn)
    G = win["G"][keep]
    ph0 = np.angle(np.median(G.real) + 1j * np.median(G.imag))
    ph = np.unwrap(np.angle(G * np.exp(-1j * ph0))) + ph0
    lg, lg_se, _ = _huber_fit(B, np.log(np.abs(G)))
    pf, pf_se, _ = _huber_fit(B, ph)
    gain = np.exp(lg)
    return [dict(erpm=float(np.exp(xn[i])), gain=float(gain[i]), phase_deg=float(np.degrees(pf[i])),
                 gain_ci=float(1.96 * lg_se[i] * gain[i]), phase_ci=float(np.degrees(1.96 * pf_se[i])))
            for i in range(nodes)]


def evaluate(rows, erpm):
    """Schedule lookup the FC would do: linear in ln(erpm) -> complex G."""
    xn = np.log([r["erpm"] for r in rows])
    g = np.interp(np.log(erpm), xn, np.log([r["gain"] for r in rows]))
    p = np.interp(np.log(erpm), xn, np.radians([r["phase_deg"] for r in rows]))
    return np.exp(g) * np.exp(1j * p)


def write_tables(rows, stem, meta=""):
    stem = Path(stem)
    with open(stem.with_suffix(".csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["erpm", "gain", "phase_deg", "gain_ci", "phase_ci"])
        w.writeheader()
        for r in rows:
            w.writerow({k: f"{v:.6g}" for k, v in r.items()})
    md = ["# Cyclic schedule (fit_cyclic.py)", "", meta, "", "Linear interpolation in ln(erpm); CIs are 95 % half-widths.", "",
          "| erpm | gain | phase_deg | gain_ci | phase_ci |", "|---|---|---|---|---|"]
    md += [f"| {r['erpm']:.0f} | {r['gain']:.4f} | {r['phase_deg']:.2f} | {r['gain_ci']:.4f} | {r['phase_ci']:.2f} |" for r in rows]
    stem.with_suffix(".md").write_text("\n".join(md) + "\n")


def run(log, nodes=5, width=1.0, step=0.5, K=4, sens=1.0):
    win = windows(log, width, step, K=K)
    if np.median(win["cond"]) > 1e4:
        print("WARNING: ill-conditioned windows (command phase not rotating?)", file=sys.stderr)
    win["G"] = win["G"] / sens
    return fit_schedule(win, nodes), win


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log")
    ap.add_argument("--nodes", type=int, default=5)
    ap.add_argument("--pole-pairs", type=int, default=7)
    ap.add_argument("--window", type=float, default=1.0)
    ap.add_argument("--step", type=float, default=0.5)
    ap.add_argument("--harmonics", type=int, default=4, help="nuisance nP lock-in terms")
    ap.add_argument("--sens", type=float, default=1.0, help="sensor units per N m (IMU proxy)")
    ap.add_argument("--out", default="schedule")
    a = ap.parse_args(argv)
    if a.nodes < 5:
        print("WARNING: system requirement is >= 5 nodes", file=sys.stderr)
    rows, win = run(read_log(a.log, a.pole_pairs), a.nodes, a.window, a.step, a.harmonics, a.sens)
    write_tables(rows, a.out, f"log {a.log}: {len(win['erpm'])} windows of {a.window} s, {a.nodes} nodes")
    for r in rows:
        print(r)


if __name__ == "__main__":
    main()
