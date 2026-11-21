"""Soft 10in mount (v5b): single-washer vs decoupled twin washers. Run from flexblade/ with PYTHONPATH=$PWD.
usage: mount_soft_10in.py [floq]"""
import sys, numpy as np
from concurrent.futures import ProcessPoolExecutor
from flexblade.schema import Mount
from flexblade.stability import mount_stiffness, mount_modes
mh, ma, ri = 0.124, 0.300, 4.0
G, inf = 6.5, 0.4
Ec = 3.4 * G * 1e6 * inf
def wash(ro, t): return Mount(G_mpa=G, eta=0.15, r_in_mm=ri, r_out_mm=ro, length_mm=t, infill=inf)
single = wash(6.5, 10)
twin = wash(5.8, 12)          # each of two; parallel lateral k = 2x
def ktheta_single(m):
    r0, r1, t = m.r_out_mm*1e-3, m.r_in_mm*1e-3, m.length_mm*1e-3
    return Ec*np.pi*(r0**4-r1**4)/4/t
def report(name, km, kth, kax, d=None):
    f0 = np.sqrt(km/mh)/2/np.pi
    print(f"== {name}: k_lat {km:.0f} N/m  hub mode {f0:.1f} Hz  k_tilt {kth:.2f} N m/rad")
    for M in (0.17, 0.3): print(f"   tilt @ {M} Nm: {np.degrees(M/kth):.2f} deg")
    print(f"   sag lateral 1g {mh*9.81/km*1e3:.3f} mm, 3g {3*mh*9.81/km*1e3:.3f} mm; axial @16.65 N {16.65/kax*1e3:.3f} mm")
    for f1p in (108, 128, 152):
        r = f1p/f0; x = 2e-6/mh*r*r/np.hypot(1-r*r, 2*.07*r)
        print(f"   1P {f1p}: 2 g.mm amp {x*1e6:.1f} um")
    for fa in (190, 225, 260):
        r = mount_modes(mh, ma, (mh+ma)*(2*np.pi*fa)**2, 0.02, km, 0.15)
        print("   f_arm", fa, "hub mode %.1f Hz zeta %.3f" % (r[0], r[1]))
def zeq(): return Mount(G_mpa=G, eta=0.15, r_in_mm=ri, r_out_mm=5.8, length_mm=6, infill=inf)  # == 2 twin washers
if __name__ == "__main__":
    k1 = mount_stiffness(single)
    report("single ro6.5 t10", k1, ktheta_single(single), Ec*np.pi*((6.5e-3)**2-(4e-3)**2)/10e-3)
    kt = mount_stiffness(zeq())
    for d in (0.02, 0.03):
        kax = 2*Ec*np.pi*((5.8e-3)**2-(4e-3)**2)/12e-3          # preloaded, both washers carry thrust
        kth = 2*Ec*np.pi*((5.8e-3)**2-(4e-3)**2)/12e-3*(d/2)**2 + 2*ktheta_single(twin)
        report(f"twin ro5.8 t12 x2, spacing {d*1e3:.0f} mm", kt, kth, kax)
    if len(sys.argv) > 1:
        sys.path.insert(0, "study")
        import verify_10in_v5 as V
        from flexblade import api, robust
        from flexblade.schema import Airframe
        def job(a):
            tag, d, fa = a
            af = Airframe(f_hz=fa, f_y_hz=1.15*fa, zeta=0.02, mount=zeq())
            dd = d.model_copy(update=dict(analysis=d.analysis.model_copy(update=dict(stability="floquet")),
                                          op=d.op.model_copy(update=dict(airframe=af))))
            r = api.evaluate(dd, ["trim", "modes"]).model_dump(mode="json")["summary"]
            return tag, fa, r.get("air_resonance_zeta"), r.get("zeta_min")
        d0 = V.D.model_copy(update=dict(motor=V.D.motor.model_copy(update=dict(J_hub=3.76e-6))))
        jobs = [(t, d, fa) for t, d in robust.variants(d0) if t in ("T0.7", "T1", "T1.4") for fa in (190, 225, 260)]
        with ProcessPoolExecutor(4) as ex:
            for r in ex.map(job, jobs): print("floq", r)
