"""Hakea 5" anti-torque vane ring (single rotor, 2307 1300KV, 6S, AUW 0.245 kg). Everything here is an estimate.
Wake: the model's BEM stations (v_i, dQ/dr) per annulus, then ideal actuator-disc contraction to the vane plane.
Vanes: a cascade strip model (momentum + thin cambered plate at the vector-mean flow angle), solved per annulus.
Usage: vanes_5in.py [rerun]. "rerun" re-runs the rotor model at T0.7/1.0/1.25 (~40 s each) into results/vanes_5in.json."""
import sys, os, json, math
import numpy as np
from scipy.optimize import brentq
FB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW = FB + "/study/results/vanes_5in.json"
# ---- assumptions (est.) ----
CLA = 2 * math.pi * 0.9 * 0.9      # /rad: thin plate, x0.9 low Re (2-6e4), x0.9 cascade (sigma ~1)
CAM = 0.05; AL0 = -2 * CAM          # 5 % circular-arc plate, thin-airfoil zero-lift angle (rad)
CLMAX, CLMIN = 1.0, -0.3            # cambered plate at Re 2-6e4 (reverse-camber side stalls early)
CD0, KD = 0.035, 0.03               # 1 mm blunt plate
RH, RT = 0.017, 0.056               # vane root (clears 2307 bell, r 14.3 mm) and tip / outer ring (m)
NS = 2                              # servo vanes, diametrically opposite (side forces cancel)
E, RHO_P, T_V = 2.0e9, 1270.0, 1.0e-3   # printed PETG modulus, density; vane wall (2 lines x 0.5 mm)
KAPPA_CYC, EPS_BLADE, LEAN = 1.5, 0.03, math.radians(15)
XF = 0.5       # twisted vane: chord-line stagger = XF x local T1.0 swirl angle at the vane plane (design choice)
D1 = None      # T1.0 rotor record (sets the twist)


def rotor_eval():
    sys.path.insert(0, FB + "/study")
    import vehicle_v0 as VV
    from flexblade import api, robust
    d, J = VV.cases()["single_norm_5in"]
    spec = {x["tag"].split(":")[0]: x for x in json.load(open(FB + "/study/results/vehicle_spec_v0.json"))
            if x["case"] == "single_norm_5in"}
    out = []
    for tag, dv in robust.variants(d, (0.7, 1.0, 1.25), None, [J]):
        k = tag.split(":")[0]
        r = api.evaluate(dv, ["trim", "harmonic"]).model_dump(mode="json")
        tr, h = r["trim"], r["harmonic"]
        out.append(dict(tag=k, rho=dv.op.rho, R=dv.blade.diameter_in * 0.0254 / 2, rpm=tr["rpm"], T=tr["thrust"],
                        Q=tr["torque"], s=spec[k]["s"], st={q: tr["stations"][q] for q in ("r", "v_i", "dQ_dr")},
                        h={q: h[q] for q in ("torque_amp", "hub_speed_amp_rpm", "hub_moment_mean", "hub_moment_2p",
                                             "thrust_1p")}))
        print(k, "done", flush=True)
    json.dump(out, open(RAW, "w"), indent=1)


def wake(rec, z):
    """Annuli at the vane plane (z below the disc): r, dr, mdot, v_z, v_theta. Swirl from Q = mdot r v_theta."""
    r0, vi, dq = (np.array(rec["st"][q]) for q in ("r", "v_i", "dQ_dr"))
    w = np.gradient(r0) * 1.0; w[0] /= 2; w[-1] /= 2; w[0] += (r0[1] - r0[0]) / 2   # trapezoid cells, root half-cell
    dQ = dq * w; scale = rec["Q"] / dQ.sum(); dQ *= scale
    dm = rec["rho"] * 2 * math.pi * r0 * vi * w
    fz = 1 + z / math.hypot(z, rec["R"])                      # actuator disc: v_z = v_i (1 + z/sqrt(z^2+R^2))
    rw = r0 / math.sqrt(fz)
    out = dict(r0=r0, r=rw, dr=w / math.sqrt(fz), dm=dm, vz=vi * fz, vth=dQ / (dm * rw), vth0=dQ / (dm * r0),
               vi=vi, dQ=dQ, scale=scale, rho=rec["rho"])
    d = out if (D1 is None or rec is D1) else wake(D1, z)
    out["xi"] = XF * np.interp(rw, d["r"], np.arctan2(d["vth"], d["vz"]))
    return out


def ring(wk, N, c, ds=0.0, dvth=0.0, fvth=1.0, fvz=1.0):
    """All-N-vanes cascade at stagger wk['xi'] + ds: captured torque, down force on vanes, max CL, max DF, alpha range."""
    Q = Fd = 0.0; cls, dfs, als = [], [], []
    for r, dr, dm, vz, vin, xi in zip(wk["r"], wk["dr"], wk["dm"], wk["vz"] * fvz, wk["vth"] * fvth - dvth * wk["r"], wk["xi"]):
        ov = max(0.0, min(r + dr / 2, RT) - max(r - dr / 2, RH)) / dr
        if ov <= 0:
            continue
        dmdr = dm / dr

        def f(vo):
            vm = (vin + vo) / 2; b = math.atan2(vm, vz); a = b - xi - ds
            cl = min(max(CLA * (a - AL0), CLMIN), CLMAX); cd = CD0 + KD * cl * cl
            q = 0.5 * wk["rho"] * (vz * vz + vm * vm) * c
            return q * (cl * math.cos(b) + cd * math.sin(b)), q * (cd * math.cos(b) - cl * math.sin(b)), cl, a
        vo = brentq(lambda v: N * f(v)[0] - dmdr * (vin - v), -3 * vz, 3 * vz)
        ft, fd, cl, a = f(vo)
        Q += dmdr * ov * dr * r * (vin - vo); Fd += N * fd * ov * dr
        W1, W2 = math.hypot(vz, vin), math.hypot(vz, vo)
        cls.append(cl); als.append(math.degrees(a))
        dfs.append(1 - W2 / W1 + abs(vin - vo) / (2 * N * c / (2 * math.pi * r) * W1))
    return dict(Q=Q, Fd=Fd, cl=max(cls), clmin=min(cls), df=max(dfs), a=(min(als), max(als)))


def vset(wk, N, c, ds=0.0, **k):
    """N vanes, NS of them deflected by ds (passage-weighted)."""
    a, b = ring(wk, N, c, 0.0, **k), ring(wk, N, c, ds, **k)
    return dict(Q=((N - NS) * a["Q"] + NS * b["Q"]) / N, Fd=((N - NS) * a["Fd"] + NS * b["Fd"]) / N,
                cl=max(a["cl"], b["cl"]), clmin=min(a["clmin"], b["clmin"]), df=max(a["df"], b["df"]), a=a["a"])


def mass(N, c, t=T_V):
    hub = math.pi * (0.020 ** 2 - 0.004 ** 2) * 0.003 * RHO_P * 0.7       # 3 mm plate, 70 % fill, motor holes
    vanes = N * (RT - RH) * c * t * RHO_P
    boss = NS * (RT - RH) * math.pi / 4 * (3.4e-3 ** 2 - 2.0e-3 ** 2) * RHO_P
    rng = 2 * math.pi * RT * 0.006 * 1.6e-3 * RHO_P                         # 6 mm tall x 1.6 mm wall
    rods = NS * (RT - RH + 0.012) * math.pi / 4 * 2e-3 ** 2 * 7850
    return dict(hub=hub, vanes=vanes, boss=boss, ring=rng, rods=rods, link=1.5e-3, tot=hub + vanes + boss + rng + rods + 1.5e-3)


def modes(N, c, ms):
    L = RT - RH; h = CAM * c
    I = c * T_V ** 3 / 12 + T_V * c * 4 / 45 * h * h; A = c * T_V
    ff = 22.37 / (2 * math.pi * L * L) * math.sqrt(E * I / (RHO_P * A))
    cant = 3.516 / (2 * math.pi * L * L) * math.sqrt(E * I / (RHO_P * A))
    Jr = (ms["ring"] + (N - NS) * ms["vanes"] / N / 3 + NS * (ms["vanes"] / N + ms["boss"] / NS + ms["rods"] / NS) / 2) * RT ** 2
    kb, ka = 12 * E * I / L ** 3, E * A / L
    tor = [math.sqrt((N - NS) * (kb * math.cos(g) ** 2 + ka * math.sin(g) ** 2) * RT ** 2 / Jr) / (2 * math.pi) for g in (0, LEAN)]
    Ls = 2 * math.pi * RT / N
    seg = 22.37 / (2 * math.pi * Ls * Ls) * 1.6e-3 * math.sqrt(E / (12 * RHO_P))
    return dict(vane_ff=ff, cantilever=cant, ring_tors_radial=tor[0], ring_tors_lean15=tor[1], ring_segment=seg)


def izz(mv):
    """Body yaw inertia (spinning bell+rotor excluded), kg m^2: (g, radius of gyration mm), est."""
    parts = dict(battery=(112, 23.1), motor_stator=(20, 9), fc_esc_rx_enc=(27, 12.2), body=(15, 15), servo=(5, 20),
                 legs=(8, 52.9), wiring=(8, 15), hardware=(5, 10), vanes=(mv["vanes"] * 1e3, 39), ring=(mv["ring"] * 1e3, RT * 1e3),
                 hub_rods=((mv["hub"] + mv["rods"] + mv["boss"]) * 1e3, 25))
    return sum(m * r * r for m, r in parts.values()) * 1e-9


def cg_lever(zle, c):
    """Vertical distance vane mid-chord -> CG (m); stack below max(motor base 25 mm, vane TE). est."""
    top = max(0.025, zle + c) + 0.003
    items = [(15, 0), (35, 0.014), (15, 0.025), (17, zle + c / 2), (5, top), (27, top + 0.008), (8, top + 0.01),
             (112, top + 0.016 + 0.014), (8, top + 0.04), (5, top)]
    zcg = sum(m * z for m, z in items) / sum(m for m, _ in items)
    return zcg - (zle + c / 2), zcg


def main():
    if sys.argv[1:2] == ["rerun"] or not os.path.exists(RAW):
        rotor_eval()
    global D1
    recs = {r["tag"]: r for r in json.load(open(RAW))}
    D1 = recs["T1"]
    R, r1 = recs["T1"]["R"], recs["T1"]
    print("1. rotor: tag rpm T(N) Q(Nm) | v_i mean | v_th, swirl angle at disc (r/R=0.4,0.6,0.8) | Q-scale")
    for k, rec in recs.items():
        wk = wake(rec, 0.0)
        vi = math.sqrt(rec["T"] / (2 * rec["rho"] * math.pi * R ** 2))
        pr = [(np.interp(x * R, wk["r0"], wk["vth0"]), math.degrees(math.atan2(np.interp(x * R, wk["r0"], wk["vth0"]),
              np.interp(x * R, wk["r0"], wk["vi"])))) for x in (0.4, 0.6, 0.8)]
        print(f"  {k:5s} {rec['rpm']:6.0f} {rec['T']:.2f} {rec['Q']:.4f} | {vi:.1f} | " +
              " ".join(f"{v:.1f}m/s {b:.0f}deg" for v, b in pr) + f" | {wk['scale']:.2f}")
    res = {}
    for name, zle in (("close", 0.15 * R), ("spaced", 0.5 * R)):
        print(f"\n== {name}: vane LE {zle * 1e3:.0f} mm = {zle / R:.2f}R below disc")
        for N in (4, 6, 8):
            def solve(c):
                return vset(wake(r1, zle + c / 2), N, c)["Q"] - r1["Q"]
            c = brentq(solve, 0.005, 0.08, xtol=1e-5); wk = wake(r1, zle + c / 2); v = vset(wk, N, c)
            ms = mass(N, c)
            print(f"  N{N}: chord {c * 1e3:.1f} mm, sigma@0.6R {N * c / (2 * math.pi * 0.6 * R):.2f}, alpha {v['a'][0]:.1f}..{v['a'][1]:.1f} deg, "
                  f"CLmax {v['cl']:.2f}, DF {v['df']:.2f}, down {v['Fd'] / r1['T'] * 100:+.1f}% T, ring mass {ms['tot'] * 1e3:.1f} g, "
                  f"wake edge {wk['r'][-1] / R:.2f}R, stagger {', '.join(f'{math.degrees(np.interp(x, wk['r'], wk['xi'])):.0f}' for x in (RH, 0.6 * R, 0.8 * R))} deg (hub/0.6R/0.8R)")
            res[(name, N)] = (c, zle)
    print("\n2./3. N=6 spaced: trim, authority, damping, thrust")
    c, zle = res[("spaced", 6)]
    zm = zle + c / 2
    ms = mass(6, c); I = izz(ms)
    wk = wake(r1, zm); ip = lambda x, q: float(np.interp(x * R, wk["r0"], wk[q]))
    print(f"  vane plane T1.0, mid-chord z {zm * 1e3:.0f} mm: " + "; ".join(
        f"disc {x:.2f}R v_i {ip(x, 'vi'):.1f} -> {ip(x, 'r') / R:.2f}R v_z {ip(x, 'vz'):.1f} v_th {ip(x, 'vth'):.1f} m/s "
        f"beta {math.degrees(math.atan2(ip(x, 'vth'), ip(x, 'vz'))):.0f}" for x in (0.35, 0.6, 0.8, 0.95)))
    for k, rec in recs.items():
        wk = wake(rec, zm); v0 = vset(wk, 6, c)
        ds = brentq(lambda d: vset(wk, 6, c, math.radians(d))["Q"] - rec["Q"], -30, 30)
        vt = vset(wk, 6, c, math.radians(ds))
        print(f"  {k:5s} fixed vanes give {v0['Q'] / rec['Q'] * 100:.1f}% Q -> servo trim {ds:+.1f} deg (CL {vt['clmin']:.2f}..{vt['cl']:.2f}, "
              f"DF {vt['df']:.2f}); vane down force {vt['Fd'] / rec['T'] * 100:+.2f}% T; v_z mean {np.average(wk['vz'], weights=wk['dm']):.1f} m/s")
    wk = wake(r1, zm); Q1 = r1["Q"]
    for d in (-20, -10, -5, 5, 10, 15, 20, 25):
        v = vset(wk, 6, c, math.radians(d))
        print(f"  ds {d:+3d}: dQ {(v['Q'] - Q1) * 1e3:+.2f} mNm ({(v['Q'] - Q1) / Q1 * 100:+.1f}% Q), yaw acc "
              f"{(v['Q'] - Q1) / I:+.0f} rad/s2, CL {v['clmin']:.2f}..{v['cl']:.2f}, down {v['Fd'] / r1['T'] * 100:+.2f}% T")
    Cv = (vset(wk, 6, c)["Q"] - vset(wk, 6, c, dvth=1.0)["Q"]) / 1.0
    print(f"  Izz {I:.2e} kg m2; vane yaw damping {Cv:.2e} Nm s -> tau {I / Cv * 1e3:.0f} ms; "
          f"mdot {wk['dm'].sum():.3f} kg/s; ring mass {', '.join(f'{q} {x * 1e3:.1f}' for q, x in ms.items())} g "
          f"(0.8 mm vanes: {mass(6, c, 0.8e-3)['tot'] * 1e3:.1f} g)")
    Jr = r1.get("J_hub", 9.09e-7) + 2.4e-3 * 0.035 ** 2 + 5e-3 * 0.010 ** 2   # bell + 5" blades + hub, est.
    acc = (recs["T1.25"]["rpm"] - r1["rpm"]) * math.pi / 30 / 0.1
    print(f"  throttle punch T1.0->1.25 in 0.1 s: J_rot {Jr:.1e} -> reaction {Jr * acc * 1e3:.1f} mNm = {Jr * acc / Q1 * 100:.0f}% Q")
    print("\n4. coupling at T1.0 (pulse-limited, s=%.2f)" % r1["s"])
    s, h, T, Om = r1["s"], r1["h"], r1["T"], r1["rpm"] * math.pi / 30
    M, dQm, dT1, dW = s * h["hub_moment_mean"], s * h["torque_amp"], s * h["thrust_1p"], s * h["hub_speed_amp_rpm"] * math.pi / 30
    vi = math.sqrt(T / (2 * r1["rho"] * math.pi * R ** 2)); tau_i = 2 * R / (3 * math.pi * vi)
    Adyn = 1 / math.hypot(1, Om * tau_i)
    eT = dT1 / T; kap = 1 + 0.5 * Adyn
    print(f"  cyclic M {M * 1e3:.1f} mNm, motor 1P torque {dQm * 1e3:.0f} mNm, thrust_1p {dT1:.2f} N ({eT * 100:.0f}% T), "
          f"rpm ripple {dW / Om * 100:.2f}%, gyro-z 1P from motor reaction {dQm / I / Om:.2f} rad/s, inflow lag {tau_i * 1e3:.2f} ms -> 1P inflow gain {Adyn:.2f}; 2P hub moment {s * h['hub_moment_2p'] * 1e3:.1f} mNm")
    for name in ("close", "spaced"):
        c, zle = res[(name, 6)]; wk = wake(r1, zle + c / 2); v = vset(wk, 6, c)
        GQ = (vset(wk, 6, c, fvth=1.01)["Q"] / v["Q"] - 1) / 0.01
        Gz = (vset(wk, 6, c, fvz=1.01)["Q"] / v["Q"] - 1) / 0.01
        Vm = np.average(np.hypot(wk["vz"], wk["vth"]), weights=wk["dm"])
        S1, S2 = (1 / math.sqrt(1 + 2 * math.pi * n * Om * c / (2 * Vm)) for n in (1, 2))
        rv = np.average(wk["r"], weights=wk["dQ"])
        dQv = S1 * v["Q"] * (GQ * (kap * eT + 2 * dW / Om) + abs(Gz) * 0.5 * Adyn * eT)
        F1 = S1 * EPS_BLADE * GQ * v["Q"] / (2 * rv)
        Fdc = GQ * KAPPA_CYC * 2 * M / (T * 0.75 * R) * v["Q"] / (2 * rv)
        dpot = math.exp(-2 * zle / rv); F2 = Fdc * (0.3 + 0.7 * dpot) * S2
        l, zcg = cg_lever(zle, c)
        print(f"  {name}: G_Q {GQ:.2f} G_z {Gz:+.2f}, Sears 1P {S1:.2f} 2P {S2:.2f}, r_v {rv * 1e3:.0f} mm, CG {zcg * 1e3:.0f} mm below disc, "
              f"lever {l * 1e3:.0f} mm, 2P potential decay {dpot:.2f}")
        print(f"    1P vane yaw ripple {dQv * 1e3:.2f} mNm = {dQv / M * 100:.0f}% M, {dQv / dQm * 100:.1f}% of motor 1P; "
              f"1P side force (3% blade mismatch) {F1 * 1e3:.1f} mN -> {F1 * l / M * 100:.1f}% M")
        print(f"    DC side force (cyclic) {Fdc * 1e3:.0f} mN -> cross-axis {Fdc * l / M * 100:.0f}% M ({math.degrees(math.atan2(Fdc * l, M)):.0f} deg phase); "
              f"2P side force {F2 * 1e3:.0f} mN -> {F2 * l / M * 100:.0f}% M")
    print("\n5. structure (N=6 spaced), Hz; 1P 190-240, 2P 380-480")
    c, _ = res[("spaced", 6)]
    print("  " + ", ".join(f"{k} {f:.0f}" for k, f in modes(6, c, mass(6, c)).items()))


if __name__ == "__main__":
    main()
