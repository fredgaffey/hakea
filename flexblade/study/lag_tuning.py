"""Lag-tuning and hub-inertia sweeps from the 5in TPU/rod winner (hypotheses I12, I13).
Knob 1: flexure.thickness scaled (lag stiffness ~ t^3; skew, width fixed).
Knob 2: J_hub (and J_hub+J_rotor) scaled x0.3..x3.
Run: FLEXBLADE_WORKER=1 PYTHONPATH=<wt>/flexblade python study/lag_tuning.py"""
import json, os, sys
os.environ["FLEXBLADE_WORKER"] = "1"
import numpy as np
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__))
DES = os.path.join(HERE, "designs", "5in_tpu_rod_winner.json")
OUT = os.path.join(HERE, "results")

def run(args):
    kind, s = args
    from flexblade.schema import Design
    from flexblade.rotor import evaluate
    d = Design.model_validate(json.load(open(DES)))
    if kind == "thick": d.flexure.thickness *= s
    elif kind == "jhub": d.motor.J_hub *= s
    elif kind == "jall": d.motor.J_hub *= s; d.motor.J_rotor *= s
    try:
        r = evaluate(d)
    except Exception as e:
        return dict(kind=kind, scale=s, error=str(e)[:80])
    h, S = r.harmonic, r.summary
    b = h.blades[0]
    struct = [c for c in r.checks if c.name not in ("motor_thermal", "motor_torque_1p", "esc_duty_slew", "esc_duty_peak")]
    w = max(r.checks, key=lambda c: c.utilisation)
    ws = max(struct, key=lambda c: c.utilisation)
    return dict(kind=kind, scale=s, nu_lag=S.get("lag_freq_per_rev"), lag_amp_deg=b.lag_amp_deg,
                pitch_amp_deg=b.pitch_amp_deg, control_ratio=h.control_ratio, lag_phase_deg=b.lag_phase_deg,
                hub_speed_amp_rpm=h.hub_speed_amp_rpm, max_util=S.get("max_utilisation"), binding=w.name,
                max_struct_util=ws.utilisation, struct_binding=ws.name, torque_amp=h.torque_amp)

if __name__ == "__main__":
    import pandas as pd
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    jobs = [("thick", s) for s in np.geomspace(0.5, 1.8, 15)] + \
           [(k, s) for k in ("jhub", "jall") for s in np.geomspace(0.3, 3, 8)]
    with Pool(16) as p: rows = p.map(run, jobs, chunksize=1)
    df = pd.DataFrame(rows)
    df[df.kind == "thick"].to_csv(f"{OUT}/lag_tuning_stiffness.csv", index=False)
    df[df.kind != "thick"].to_csv(f"{OUT}/lag_tuning_inertia.csv", index=False)
    print(df.to_string())
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    t = df[df.kind == "thick"]
    ax[0].plot(t.nu_lag, t.control_ratio, "o-", label="control_ratio")
    ax[0].plot(t.nu_lag, t.max_struct_util * t.control_ratio.max(), "s--", c="r", label="max struct util (scaled)")
    ax[0].set_xlabel("lag freq [/rev] (flexure thickness sweep)"); ax[0].set_ylabel("control_ratio"); ax[0].legend(); ax[0].grid()
    for k in ("jhub", "jall"):
        j = df[df.kind == k]; ax[1].loglog(j.scale, j.control_ratio, "o-", label=k)
    ax[1].set_xlabel("inertia scale"); ax[1].set_ylabel("control_ratio"); ax[1].legend(); ax[1].grid(which="both")
    fig.tight_layout(); fig.savefig(f"{OUT}/lag_tuning.png", dpi=110)
