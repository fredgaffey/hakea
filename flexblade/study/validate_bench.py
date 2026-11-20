"""Validate flexblade motor.py against Hakea bench logs (notes/sim/bench).

Logs 33 (prop off) / 34 (prop on): BNCH (t,test,pwm_us,f_Hz,rpm @~430 Hz via bidir DShot),
BAT.Volt (10 Hz), ESC (Temp = ESC MCU temp; Curr/Volt mis-scaled, unusable). No thrust, no
current, no winding temperature are logged.  Test 1 = 3 s steps; test 3 = 1300+-200 us sweep
1-50 Hz; test 4 = 1500+-400 us sweep.
Run: PYTHONPATH=flexblade python flexblade/study/validate_bench.py
"""
import json
import pathlib

import matplotlib
import numpy as np
import pandas as pd
from scipy.optimize import brentq

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from flexblade.motor import get_motor, kt, steady_duty

ROOT = pathlib.Path(__file__).resolve().parents[2]
B = ROOT / "notes/sim/bench/"
OUT = pathlib.Path(__file__).parent / "results"
M0 = get_motor("tmotor_2307_1300kv")
KQ = 1.8425e-8
J_P = 2.53e-6
RUNS = {"prop off": ("00000033", 0.0, 0.0), "prop on": ("00000034", KQ, J_P)}


def load(tag):
    return pd.read_csv(B / f"{tag}_BNCH.csv"), pd.read_csv(B / f"{tag}_BAT.csv")


def ss_rpm(m, d, kq, V):
    k = kt(m)

    def f(w):
        return m.voltage_gain * V * d - (k * w + m.R * (kq * w * w + m.T0 + m.b * w) / k)
    return brentq(f, 0, 6000) * 60 / 2 / np.pi if f(0) > 0 else 0.0


def steps(tag):
    bn, bat = load(tag)
    s = bn[bn.test == 1]
    rows = []
    for p in sorted(s.pwm.unique()):
        x = s[s.pwm == p]
        if p < 1100 or len(x) < 500:
            continue
        x = x[x.t > x.t.max() - 1.0]
        rows.append((p, x.rpm.mean(), np.interp(x.t.mean(), bat.t, bat.Volt)))
    return np.array(rows)


def duty_of(p):
    return (p - 1000) / 1000.0


def fit_steady(rows, kq):
    best = None
    for kv in np.arange(1100, 1700, 2.0):
        m = M0.model_copy(update={"kv": float(kv)})
        e = [np.log(ss_rpm(m, duty_of(p), kq, V) / r) for p, r, V in rows if r > 500]
        s = np.sqrt(np.mean(np.square(e)))
        if best is None or s < best[1]:
            best = (kv, s)
    return best


def lockin(t, y, f):
    return 2 * np.mean((y - y.mean()) * np.exp(-2j * np.pi * f * t))


def freq_resp(tag, test):
    bn, _ = load(tag)
    s = bn[(bn.test == test) & (bn.f > 0)]
    fr = np.round(s.f.values)
    out = []
    for f in np.unique(fr):
        x = s[fr == f]
        if len(x) < 200 or f < 1:
            continue
        t = np.arange(x.t.min(), x.t.max(), 1 / 400.0)
        if len(t) < 100:
            continue
        fm = x.f.mean()
        p = np.interp(t, x.t, x.pwm)
        r = np.interp(t, x.t, x.rpm)
        out.append((fm, lockin(t, r, fm) / lockin(t, p, fm), r.mean(), p.mean()))
    return np.array(out, dtype=object)


def model_resp(m, f, rpm0, V, kq, J):
    """rpm per us of pwm, linearised about rpm0."""
    w0 = rpm0 * 2 * np.pi / 60
    k = kt(m)
    s = 2j * np.pi * f
    Z = m.R + s * m.L
    G = k * m.voltage_gain * V / Z
    damp = k * k / Z + m.b + 2 * kq * w0
    return G * 1e-3 / (J * s + damp) * 60 / 2 / np.pi


def main():
    OUT.mkdir(exist_ok=True)
    res = {"steady": {}, "dynamic": {}, "proposals": {}, "notes": []}
    fig, ax = plt.subplots(2, 3, figsize=(16, 9))
    for name, (tag, kq, Jp) in RUNS.items():
        rows = steps(tag)
        pred = np.array([ss_rpm(M0, duty_of(p), kq, V) for p, r, V in rows])
        err = pred / rows[:, 1] - 1
        kv, s = fit_steady(rows, kq)
        res["steady"][name] = dict(
            pwm=rows[:, 0].tolist(), rpm_bench=rows[:, 1].tolist(), rpm_model=pred.tolist(),
            err_pct=(100 * err).tolist(), rms_err_pct=float(100 * np.sqrt(np.mean(err**2))),
            mean_err_pct=float(100 * err.mean()), fitted_kv=kv, fitted_rms_log=float(s))
        ax[0, 0].plot(rows[:, 0], rows[:, 1], "o-", label=f"bench {name}")
        ax[0, 0].plot(rows[:, 0], pred, "x--", label=f"model {name}")
        ax[0, 1].plot(rows[:, 0], 100 * err, "o-", label=name)
        Jt = M0.J_rotor + Jp
        for test, c in ((3, "C0"), (4, "C1")):
            fr = freq_resp(tag, test)
            if len(fr) == 0:
                continue
            H = fr[:, 1].astype(complex)
            f = fr[:, 0].astype(float)
            mod = np.array([model_resp(M0, ff, float(rp), 24.5, kq, Jt) for ff, rp in zip(f, fr[:, 2])])
            gr = np.abs(H) / np.abs(mod)
            ph = np.degrees(np.angle(H / mod))
            sel = (f >= 2) & (f <= 40)
            res["dynamic"][f"{name} test{test}"] = dict(
                gain_err_pct_2_40Hz=float(100 * (np.median(gr[sel]) - 1)),
                phase_diff_deg_median=float(np.median(ph[sel])),
                mean_rpm=float(fr[:, 2].astype(float).mean()),
                gain_err_pct_by_f={int(a): round(float(100 * (g - 1)), 1) for a, g in zip(f[::6], gr[::6])})
            ls = "-" if name == "prop on" else ":"
            ax[1, 0].semilogx(f, 20 * np.log10(np.abs(H)), c + ls, label=f"bench {name} T{test}")
            ax[1, 0].semilogx(f, 20 * np.log10(np.abs(mod)), c + "--", alpha=.4)
            ax[1, 1].semilogx(f, np.degrees(np.angle(H)), c + ls)
            ax[1, 1].semilogx(f, np.degrees(np.angle(mod)), c + "--", alpha=.4)
    ax[0, 0].set_title("steady rpm vs pwm (steps)")
    ax[0, 0].legend(fontsize=7)
    ax[0, 1].set_title("model error [%]")
    ax[0, 1].legend()
    ax[1, 0].set_title("duty->rpm gain [dB rpm/us] (dashed=model)")
    ax[1, 0].legend(fontsize=6)
    ax[1, 1].set_title("phase [deg] (dashed=model)")
    for tag, nm in (("00000033", "prop off"), ("00000034", "prop on")):
        e = pd.read_csv(B / f"{tag}_ESC.csv")
        e = e[e.Temp > 0]
        ax[0, 2].plot(e.t - e.t.min(), e.Temp, label=nm)
        res["notes"].append(f"{nm}: ESC MCU temp {e.Temp.iloc[0]:.0f}->{e.Temp.max():.0f} C over "
                            f"{e.t.max() - e.t.min():.0f} s (ESC, not winding)")
    ax[0, 2].set_title("ESC MCU temperature (NOT winding)")
    ax[0, 2].legend()
    ax[1, 2].axis("off")
    ax[1, 2].text(0, .5, "No winding temp / current / thrust logged:\nthermal_resistance_KW not identifiable.\n"
                  "ESC Curr/Volt mis-scaled (73 A at 0 rpm).", fontsize=10)
    on, off = res["steady"]["prop on"], res["steady"]["prop off"]
    res["proposals"] = dict(
        kv_prop_off=off["fitted_kv"], kv_prop_on=on["fitted_kv"],
        thermal_resistance_KW="NO CHANGE: not identifiable from logs (keep 1.5 GUESS; needs thermocouple test)")
    fig.tight_layout()
    fig.savefig(OUT / "validate_bench.png", dpi=110)
    (OUT / "validate_bench.json").write_text(json.dumps(res, indent=1))
    print(json.dumps({k: res[k] for k in ("dynamic", "proposals", "notes")}, indent=1))
    for n in res["steady"]:
        print(n, {k: v for k, v in res["steady"][n].items() if k not in ("pwm", "rpm_bench", "rpm_model")})
        print(np.round(res["steady"][n]["rpm_bench"]), np.round(res["steady"][n]["rpm_model"]))


main()
