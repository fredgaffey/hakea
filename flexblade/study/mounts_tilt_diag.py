"""Diagnose the 7in_v6 tilt-Floquet zeta floor (twin mount, T1.4, arm 190 Hz): mode id, participation,
zeta vs f_arm / zeta_af, and fixes.  Run from flexblade/ with PYTHONPATH=$PWD:$PWD/study:$PWD/study/build."""
import sys
import numpy as np
from flexblade import rotor, robust, stability as S
from flexblade.schema import Airframe, Design, Mount
import tilt_whirl as TW
import mounts_tilt as MTS

MNT = MTS.mk("7in_v6")


def setup(T=1.4, J=3.76e-6, fn="7in_tt_strap_steelx_v6", d0=None):
    d0 = d0 or Design.model_validate_json((TW.HERE / "designs" / f"{fn}.json").read_text())
    d = [dd for t, dd in robust.variants(d0, thrust_factors=(T,), j_hub=[J])][0]
    d = d.model_copy(update=dict(analysis=d.analysis.model_copy(update=dict(stability="floquet"))))
    if d.flexure.pre_lag_deg == "auto":
        d = rotor.resolve_pre_lag(d)
    model = rotor.build_model(d)
    aero = rotor.make_aero(model)
    ss = rotor.static_trim(model, aero)
    lin = rotor.linearize(model, ss, d.analysis.coriolis)
    lam, B = S._basis(model, ss, lin, d.analysis.stability_basis, hub=True, tilt=True)
    ok = np.sqrt(np.abs(lam)) > 0.05 * ss.Omega
    fe = float(np.sqrt(abs(lam[ok][0])) / (2 * np.pi))
    rs = S.reduced_system(model, ss, lin, aero, B, fe, hub=True, tilt=True)
    Jp = d.motor.J_rotor + d.motor.J_hub
    It0 = 0.5 * Jp + MNT["mass_motor_kg"] * 0.015 ** 2
    return dict(model=model, ss=ss, lin=lin, aero=aero, lam=lam, B=B, rs=rs, It0=It0, d=d, fe=fe)


def label_basis(C):
    """per rotor basis mode: f (Hz, rotating), tip [lag, flap, pitch*c] of blade 0 and 1 in blade axes."""
    model, B, lam = C["model"], C["B"], C["lam"]
    out = []
    for j in range(len(lam)):
        v = B[:, j]
        row = []
        for bi in model.blades:
            n = bi.chain_nodes[-1]
            u = bi.Rk.T @ v[6 * n:6 * n + 3]
            r = bi.Rk.T @ v[6 * n + 3:6 * n + 6]
            row.append([u[1], u[2], r[0] * 0.02])
        row = np.array(row)
        out.append((float(np.sqrt(abs(lam[j])) / 2 / np.pi), row / np.abs(row).max()))
    return out


def af_of(C, fa, ix=1.0, zaf=0.02, fy=1.15, mnt=None, kt=None):
    m = dict(mnt or MNT)
    if kt is not None:
        m["k_tilt"] = kt
    return Airframe(f_hz=fa, f_y_hz=fy * fa, zeta=zaf, mount=Mount(**{**m, "I_t_kgm2": ix * C["It0"]}))


def solve(C, afs, detail=False):
    rs, model, ss = C["rs"], C["model"], C["ss"]
    I_rt = 0.5 * (rs.M[rs.it[0], rs.it[0]] + rs.M[rs.it[1], rs.it[1]])
    af = S.make_airframe(afs, model.design.op.vehicle_mass, rs.m_rotor, model.design.motor, I_rt)
    s, V = S.coupled_eigs(rs, af)
    z = S.frame_zeta(rs, af, s, V)
    keep = np.abs(s) > 0.05 * ss.Omega
    i = int(np.argmin(np.where(keep, z, np.inf)))
    if not detail:
        return float(z[i])
    Mc = S.coupled_matrices(rs, af, average=True)[0]
    n = len(Mc)
    q = V[:n, i]
    e = np.real(np.conj(q) * (Mc @ q))
    e = e / e.sum()
    Om = ss.Omega
    mt = af.m + rs.m_rotor
    return dict(z=float(z[i]), s=complex(s[i]), f_rot=abs(s[i].imag) / 2 / np.pi, per_rev=abs(s[i].imag) / Om,
                f1p=Om / 2 / np.pi, part=e, fx=np.sqrt(af.kx / mt) / 2 / np.pi, fy=np.sqrt(af.ky / mt) / 2 / np.pi,
                f_tilt=np.sqrt(af.kt / (af.It + I_rt)) / 2 / np.pi, ih=rs.ih, it=rs.it, af=af)


def report(C, fa=190.0, **kw):
    r = solve(C, af_of(C, fa, **kw), detail=True)
    print("zmin %.4f  s %s  f_rot %.1f Hz = %.3f/rev  1P %.1f  hub fx %.1f fy %.1f (coupled w/ mount)  tilt0 %.1f" % (
        r["z"], r["s"], r["f_rot"], r["per_rev"], r["f1p"], r["fx"], r["fy"], r["f_tilt"]))
    print("fixed-frame candidates f_rot +/- 1P:", round(r["f_rot"] + r["f1p"], 1), round(abs(r["f_rot"] - r["f1p"]), 1))
    for j, (f, row) in enumerate(label_basis(C)):
        print("basis %2d %7.1f Hz %5.2f/rev part %.3f  b0[lag,flap,pitch] %s  b1 %s" % (
            j, f, f / r["f1p"], r["part"][j], np.round(row[0], 2), np.round(row[1], 2)))
    print("hub x,y part", np.round(r["part"][list(r["ih"])], 3), "tilt part", np.round(r["part"][list(r["it"])], 3))
    return r


def zeta_dom(C, afs):
    """Per root: frame_zeta (library) and zeta with the fixed-frame frequency of the DOMINANT hub harmonic.
    Rotating hub amplitude u=(ux,uy) e^{st} -> fixed frame (ux+i uy) at w+Omega and (ux-i uy) at |w-Omega|
    (same for tilt; kinetic weights m_rotor, I_rt).  Returns the worst root by each measure."""
    rs, model, ss = C["rs"], C["model"], C["ss"]
    Om = ss.Omega
    I_rt = 0.5 * (rs.M[rs.it[0], rs.it[0]] + rs.M[rs.it[1], rs.it[1]])
    af = S.make_airframe(afs, model.design.op.vehicle_mass, rs.m_rotor, model.design.motor, I_rt)
    s, V = S.coupled_eigs(rs, af)
    z = S.frame_zeta(rs, af, s, V)
    Mc = S.coupled_matrices(rs, af, average=True)[0]
    n = len(Mc)
    keep = (np.abs(s) > 0.05 * Om) & (s.imag > 0)
    zd = np.full(len(s), np.inf)
    info = {}
    for k in np.where(keep)[0]:
        q = V[:n, k]
        tot = np.real(np.conj(q) @ Mc @ q)
        fh = sum(np.real(np.conj(q[h]) @ Mc[np.ix_(h, h)] @ q[h]) for h in (list(rs.ih), list(rs.it))) / tot
        w = s[k].imag
        if fh > 0.5:
            ux, uy = q[list(rs.ih)]
            tx, ty = q[list(rs.it)]
            ap = abs(ux + 1j * uy) ** 2 * rs.m_rotor + abs(tx + 1j * ty) ** 2 * I_rt
            am = abs(ux - 1j * uy) ** 2 * rs.m_rotor + abs(tx - 1j * ty) ** 2 * I_rt
            wf = w + Om if ap > am else abs(w - Om)
            info[k] = (ap / (ap + am), (w + Om) / 2 / np.pi, abs(w - Om) / 2 / np.pi)
        else:
            wf = abs(s[k])
        zd[k] = -s[k].real / np.hypot(s[k].real, wf)
    i = int(np.argmin(np.where(keep, z, np.inf)))
    j = int(np.argmin(zd))
    return dict(z_lib=float(z[i]), re_lib=float(s[i].real), f_rot_lib=float(s[i].imag / 2 / np.pi), inf_lib=info.get(i),
                z_dom=float(zd[j]), re_dom=float(s[j].real), f_rot_dom=float(s[j].imag / 2 / np.pi), inf_dom=info.get(j))


def fmt(r):
    g = lambda t: None if t is None else tuple(round(x, 2) for x in t)
    return "lib %.4f (Re %.1f, f_rot %.1f, [P+ frac, f+, f-] %s) | dom %.4f (Re %.1f, f_rot %.1f, %s)" % (
        r["z_lib"], r["re_lib"], r["f_rot_lib"], g(r["inf_lib"]), r["z_dom"], r["re_dom"], r["f_rot_dom"], g(r["inf_dom"]))


if __name__ == "__main__" and "dom" in sys.argv:
    C = setup()
    for fa in (150, 170, 190, 225, 260, 300, 350):
        for ix in (1.0, 2.0):
            print(fa, ix, fmt(zeta_dom(C, af_of(C, fa, ix=ix))), flush=True)
    for kt in (100, 250, 430):
        print("kt", kt, fmt(zeta_dom(C, af_of(C, 190, kt=kt))), flush=True)
    for za in (0.02, 0.08):
        print("zaf", za, fmt(zeta_dom(C, af_of(C, 190, zaf=za))), flush=True)
    sys.exit()

if __name__ == "__main__":
    C = setup()
    report(C)
    if "scan" in sys.argv:
        print("f_arm scan (zaf .02):", {fa: round(solve(C, af_of(C, fa)), 4) for fa in range(150, 360, 10)}, flush=True)
        print("iso (fy=fx):", {fa: round(solve(C, af_of(C, fa, fy=1.0)), 4) for fa in (165, 190, 218.5, 225)}, flush=True)
        print("zaf scan @190:", {za: round(solve(C, af_of(C, 190.0, zaf=za)), 4) for za in (0.02, 0.04, 0.06, 0.08)}, flush=True)
