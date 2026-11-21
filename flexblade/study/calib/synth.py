"""Synthetic bench logs from the model truth (truth_10in_v5.json = rotor.harmonic_1p over throttle 0.7-1.4).
Usage: python synth.py [--dur 20] [--amp 0.05] [--imu-noise 0.05] -> validation printout (5 % / 3 deg check)."""
from __future__ import annotations
import argparse, csv, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import truth, fit_cyclic as fc                                    # noqa: E402

PP = 7


def truth_G(erpm_mech):
    r = truth.load()
    x = np.log([a["rpm"] for a in r])
    g = np.interp(np.log(erpm_mech), x, np.log([a["gain"] for a in r]))
    p = np.interp(np.log(erpm_mech), x, np.unwrap(np.radians([a["phase_deg"] for a in r])))
    return np.exp(g) * np.exp(1j * p)


def make_log(dur=20.0, fs=2000.0, amp=0.05, imu_noise=0.05, vib=0.15, jitter=0.01, dither_hz=2.0,
             seed=0, thr=(0.7, 1.4)):
    """Slow linear throttle sweep. Moment proxy = fixed-frame Mx+jMy [N m]:
    G*amp*e^{j cmd_phase} + DC offset + rotating nP vibration (n=1..4, random amp/phase) + white IMU noise.
    eRPM log has 1 % white jitter; azimuth is integrated from the jittered eRPM (as an ESC log would be)."""
    rng = np.random.default_rng(seed)
    t = np.arange(0, dur, 1 / fs)
    r = truth.load()
    f = np.interp(t / dur, [0, 1], thr)
    rpm = np.interp(f, [a["throttle"] for a in r], [a["rpm"] for a in r])      # true mech rpm
    w = rpm * 2 * np.pi / 60
    psi_true = np.cumsum(w) / fs
    ph = 2 * np.pi * dither_hz * t + 0.7
    z = truth_G(rpm) * amp * np.exp(1j * ph) + (0.02 + 0.01j)
    for n in range(1, 5):
        a = vib * rng.uniform(0.3, 1) / n
        z = z + a * np.exp(1j * (n * psi_true + rng.uniform(0, 6.28))) + 0.3 * a * np.exp(-1j * (n * psi_true + rng.uniform(0, 6.28)))
    z = z + imu_noise * (rng.standard_normal(len(t)) + 1j * rng.standard_normal(len(t)))
    erpm = rpm * PP * (1 + jitter * rng.standard_normal(len(t)))
    return dict(t=t, erpm=erpm, psi=np.cumsum(erpm / PP * 2 * np.pi / 60) / fs, amp=np.full(len(t), amp),
                ph=np.mod(ph, 2 * np.pi), z=z), rpm


def validate(nodes=5, **kw):
    log, rpm = make_log(**kw)
    rows, win = fc.run(log, nodes)
    # truth at the fitted nodes (erpm node -> mech rpm), and evaluated across the swept range
    xs = np.linspace(rpm.min() * 1.03, rpm.max() * 0.97, 50) * PP
    Gf, Gt = fc.evaluate(rows, xs), truth_G(xs / PP)
    gerr = np.max(np.abs(np.abs(Gf) / np.abs(Gt) - 1)) * 100
    perr = np.max(np.abs(np.degrees(np.angle(Gf / Gt))))
    return rows, gerr, perr


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dur", type=float, default=20.0)
    ap.add_argument("--amp", type=float, default=0.05)
    ap.add_argument("--imu-noise", type=float, default=0.05)
    ap.add_argument("--csv")
    a = ap.parse_args()
    if a.csv:
        L, _ = make_log(a.dur, amp=a.amp, imu_noise=a.imu_noise)
        with open(a.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "erpm", "azimuth_deg", "cmd_amp", "cmd_phase_deg", "mx", "my"])
            for i in range(len(L["t"])):
                w.writerow([f"{L['t'][i]:.5f}", f"{L['erpm'][i]:.1f}", f"{np.degrees(L['psi'][i]) % 360:.3f}", L["amp"][i],
                            f"{np.degrees(L['ph'][i]):.3f}", f"{L['z'][i].real:.5f}", f"{L['z'][i].imag:.5f}"])
    # requirement scan: worst of 3 seeds, pulse amplitude expressed as SNR = amp / per-sample IMU noise
    for dur in (10.0, a.dur):
        for snr in (0.5, 1, 2, 4):
            e = [validate(5, dur=dur, amp=snr * a.imu_noise, imu_noise=a.imu_noise, seed=s)[1:] for s in range(3)]
            print(f"dur {dur:4.0f}s amp/sigma_imu {snr:3.1f}: worst gain err {max(x[0] for x in e):5.2f} %  phase err {max(x[1] for x in e):5.2f} deg")
