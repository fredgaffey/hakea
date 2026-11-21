"""Forced response of a 1-DOF modal arm (vertical z, lateral) to hub harmonics from results/two_per_rev.json.
Only T1.0 forces exist in the json; other throttles scale F ~ rpm^2 (assumption), rpm ~ sqrt(T)."""
import json, numpy as np
from pathlib import Path
H = Path(__file__).parent
D = json.load(open(H / "results/two_per_rev.json"))
M = 0.25 * 1.6971          # kg, modal mass per arm = system_check (0.25 m_vehicle)
Z = 0.02
Ts = np.linspace(0.7, 1.4, 15)
fa = np.arange(80, 401, 1.0)
def resp(F, f, fn):
    k = M * (2*np.pi*fn)**2; r = f/fn
    h = 1/np.sqrt((1-r**2)**2 + (2*Z*r)**2)          # dynamic magnification
    x = F/k*h
    return x*1e6, F*h*np.sqrt(1+(2*Z*r)**2), h       # um, transmitted N, mag
def cases(d):
    f1 = d['f1p']; z = lambda a: abs(a[2]); xy = lambda a: float(np.hypot(a[0], a[1]))
    f1p, f2p = d['hubF_1p_fixed'], d['hubF_2p_fixed']
    # (name, harmonic, force at T1.0)
    return [('1P vert', 1, z(f1p['1'])), ('2P vert', 2, z(f2p['2'])), ('2P lat(from 1P-flap)', 2, xy(f1p['2'])),
            ('3P lat', 3, xy(f2p['3']))]
out = {}
lines = []
for d in D:
    pc = d['precone']; f1 = d['f1p']; res = {}
    for name, h, F0 in cases(d):
        amp = np.zeros_like(fa); mag = np.zeros_like(fa); tf = np.zeros_like(fa)
        for T in Ts:
            s = np.sqrt(T); f = h*f1*s; F = F0*s**4 if False else F0*T   # F ~ rpm^2 ~ T
            a, t, m = resp(F, f, fa); amp = np.maximum(amp, a); tf = np.maximum(tf, t); mag = np.maximum(mag, m)
        res[name] = (amp, tf, mag)
        bad = amp > 50
        runs = []; i = 0
        while i < len(fa):
            if bad[i]:
                j = i
                while j+1 < len(fa) and bad[j+1]: j += 1
                runs.append((fa[i], fa[j])); i = j+1
            else: i += 1
        lines.append(f"precone {pc}: {name:22s} F(T1)={F0:5.2f} N  f={h*f1*np.sqrt(.7):.0f}-{h*f1*np.sqrt(1.4):.0f} Hz  "
                     f"forbidden(>50um): {runs}  peak@fa=120/150/200: "
                     + "/".join(f"{amp[fa==v][0]:.0f}um" for v in (120, 150, 200)))
    out[pc] = {n: dict(amp_um=v[0].tolist(), Ftr=v[1].tolist()) for n, v in res.items()}
    # quasi-static amplitude for reference
    lines.append(f"  static F/k at 120 Hz per N: {1e6/(M*(2*np.pi*120)**2):.2f} um/N")
print("\n".join(lines))
json.dump(dict(fa=fa.tolist(), res=out), open(H / "results/arm_vertical.json", "w"))
