"""2P (second-order) response of the 1P-pulsed rotor: quadratic forcing from the 1P solution.

harmonic_1p is a 1P-only linear harmonic balance about the static trim, so 2P from products of 1P
motions (pitch x heave/lag rate in the strip aero, flap x lag / CF kinematics in the corotational
beam, follower aero loads) is absent; forced.py only adds empirical nP fractions at crossings.
Here: U1 = s* x U(harmonic_1p); F2 = -(1/4) D2R[U1,U1] (structure + dead/follower aero, central FD
on the nonlinear residual) + (1/4) D2f_strip[q,q] (quasi-steady section_loads) + motor 2P torque;
solve Z(2W) U2 = F2 (aero Ka at 2W, no 2P inflow). Strip stress via checks(..., forced=[2P]).
PYTHONPATH=flexblade python flexblade/study/two_per_rev.py   -> results/two_per_rev.json
"""
from __future__ import annotations
import json, sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import pulse_limited as pl
from flexblade import robust, rotor, forced as frc
from flexblade.beam import assemble, update_state
from flexblade.rotor import _edofs


def d2(fun, U, h=0.1):
    """D2 fun[U,U] for complex U (polarisation of real central second differences, Richardson in h:
    the rigid links are stiff penalties whose O(h^2) quartic terms would otherwise swamp the result)."""
    f0 = fun(np.zeros(len(U)))
    A, B = U.real, U.imag
    def D(hh):
        Q = lambda v: (fun(hh * v) - 2 * f0 + fun(-hh * v)) / hh ** 2
        return Q(A) - Q(B) + 0.5j * (Q(A + B) - Q(A - B))
    return (4 * D(h / 2) - D(h)) / 3


def strip_2p(model, ss, U1, U2):
    """Per blade max axial strain amplitude in the flexure strips at 1P (U1) and 2P (lin U2 + quadratic)."""
    from flexblade.beam import assemble_internal, element_end_forces, element_end_forces_lin
    fe, st = model.fe, ss.state
    els = [i for i, e in enumerate(fe.elements) if e.kind == "flexure"]
    def ef(du):
        s_ = update_state(st, du)
        cr = assemble_internal(fe, s_, False).cr
        return np.concatenate([np.r_[element_end_forces(fe.elements[i], cr[i], s_)] for i in els])
    q = 0.25 * d2(ef, U1).reshape(len(els), 2, 6)
    cr0 = assemble_internal(fe, st, False).cr
    kfac = rotor.strip_clamp_factor(model.design.flexure)[0]
    out = {}
    for k, i in enumerate(els):
        e = fe.elements[i]
        t_hat = rotor.unit(st.x[e.n1] - st.x[e.n0])
        l1 = element_end_forces_lin(e, cr0[i], U1[_edofs(e)], st)
        l2 = element_end_forces_lin(e, cr0[i], U2[_edofs(e)], st)
        for j, sec in enumerate((e.sections[0], e.sections[-1])):
            a1 = rotor._fibre_amp(sec, l1[j], t_hat, 1.0, False, None, 1.0, {})[2] * kfac
            a2 = rotor._fibre_amp(sec, l2[j] + q[k, j], t_hat, 2.0, False, None, 1.0, {})[2] * kfac
            a2l = rotor._fibre_amp(sec, l2[j], t_hat, 2.0, False, None, 1.0, {})[2] * kfac
            o = out.setdefault(f"b{e.blade}", dict(e1=0.0, e2=0.0, e2_lin_only=0.0))
            o["e1"], o["e2"], o["e2_lin_only"] = max(o["e1"], a1), max(o["e2"], a2), max(o["e2_lin_only"], a2l)
    return out


def second_order(model, ss, aero, lin, hd, s, torque2=True):
    d, fe, w = model.design, model.fe, ss.Omega
    n, f = fe.ndof, lin.free
    U1, X1 = s * hd.U, s * hd.X
    tr = ss.trim

    def Rs(du):
        st = update_state(ss.state, du)
        sa = assemble(fe, st, w, False)
        return sa.fint - sa.Fcf - rotor.aero_load_vector(model, st, tr)
    N2 = 0.25 * d2(Rs, U1)
    # strip aero quadratic terms (velocity x pitch, W^2 products), quasi-steady
    Fa = np.zeros(n, complex)
    maps = rotor._aero_maps(model, ss.state)
    for bi in model.blades:
        ph = np.exp(1j * bi.psi0)
        for e, xi, wl, r, qc, ia, sw in rotor._aero_points(model, bi):
            rr, sH, L, Nr = next(maps)
            P = rotor._blade_proj(bi, L, Nr)
            q = P @ U1[_edofs(e)]
            dq = np.array([1j * w * q[1], X1 * r * ph + 1j * w * q[0], q[2]])   # dU_T, dU_P, dtheta
            f2 = np.zeros(3, complex)
            for k, wk in ((ia, 1 - sw), (ia + 1, sw)):
                x0 = np.array([tr.U_T[k], tr.U_P[k], tr.theta[k]])
                fn = lambda v, k=k: np.array(aero.section_loads(x0[0] + v[0], x0[1] + v[1], x0[2] + v[2],
                                                                idx=np.array([k]))).ravel()
                f2 += wk * 0.25 * d2(fn, dq)
            Fa[_edofs(e)] += wl * P.T @ f2
    F2 = -N2 + Fa
    hubd = 6 * model.hub + 5
    if torque2:
        F2[hubd] += frc.torque_frac(2) * s * hd.dQ
    w2 = 2 * w
    Z = rotor.K_complex(model, lin, w2 / 2 / np.pi) + 1j * w2 * lin.G - w2 * w2 * lin.M
    if hd.drive.get("k_hub") is not None:
        Z[hubd, hubd] += hd.drive["k_hub"].real + 2j * hd.drive["k_hub"].imag   # damping ~ i w b
    if d.analysis.aero != "none":
        Z = Z - rotor.aero_linear(model, ss.state, aero, tr, w2, False).Ka
    U2 = np.zeros(n, complex)
    U2[f] = np.linalg.solve(Z[np.ix_(f, f)], F2[f])
    hub6 = np.r_[6 * model.hub:6 * model.hub + 6]
    R2 = Z[hub6] @ U2 - F2[hub6]
    return U2, -R2[0:3], -R2[3:6], F2


def fixed(ph, nrot):
    """rotating-frame vector phasor (3,) at nrot/rev -> fixed-frame amplitudes {harmonic: (|Fx|,|Fy|,|Fz|)}."""
    psi = np.linspace(0, 2 * np.pi, 128, endpoint=False)
    v = np.real(np.outer(ph, np.exp(1j * nrot * psi)))
    c, sn = np.cos(psi), np.sin(psi)
    F = np.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1], v[2]])
    out = {}
    for h in (nrot - 1, nrot, nrot + 1):
        a = np.abs(2 * np.mean(F * np.exp(-1j * h * psi), axis=1)) / (2 if h == 0 else 1)
        if a.max() > 1e-12:
            out[h] = a.tolist()
    return out


def blade_amps(model, U, hubd):
    out = []
    for bi in model.blades:
        u, rv = rotor.blade_local_motion_c(model, U, bi)
        i75 = np.argmin(np.abs(bi.chain_r - 0.75 * model.R))
        out.append(dict(flap=float(np.degrees(abs(rv[0, 1]))), lag=float(np.degrees(abs(rv[0, 2] - U[hubd]))),
                        pitch=float(np.degrees(abs(rv[i75, 0])))))
    return out


def case(precone):
    d0 = pl.load_design("10in_tt_strap_steelx_v8")
    d0 = d0.model_copy(update=dict(blade=d0.blade.model_copy(update=dict(precone_deg=precone))))
    d = robust.variants(d0, thrust_factors=(1.0,))[0][1]
    ps = pl._eval_point(d)
    s = float(ps["pulse_scale"])
    if d.flexure.pre_lag_deg == "auto":
        d = rotor.resolve_pre_lag(d)
    model = rotor.build_model(d)
    aero = rotor.make_aero(model)
    ss = rotor.static_trim(model, aero)
    lin = rotor.linearize(model, ss, d.analysis.coriolis)
    hd = rotor.harmonic_1p(model, ss, aero, lin=lin)
    hubd = 6 * model.hub + 5
    U2, F2h, M2h, F2 = second_order(model, ss, aero, lin, hd, s)
    U2q, F2hq, M2hq, _ = second_order(model, ss, aero, lin, hd, s, torque2=False)
    fz = ss.Omega / 2 / np.pi
    strip = strip_2p(model, ss, s * hd.U, U2)
    hd1 = replace(hd, U=s * hd.U)
    cr = SimpleNamespace(U=U2, freq_hz=2 * fz, label="2P quad", per_rev=2.0, n=2)
    ch = rotor.checks(model, ss, hd1, [cr])
    info = {k: dict(v) for k, v in model.np_info.items()}
    for k, v in info.items():
        E = v["stress"] / max(v["strain"], 1e-30)
        v["stress_1p"] = v["strain_1p"] * E
    chk = {c.name: float(c.value) for c in ch if "fatigue" in c.name or "strip" in c.name}
    ch0 = {c.name: float(c.value) for c in rotor.checks(model, ss, hd1, None) if c.name in chk}
    return dict(precone=precone, pulse_scale=s, rpm=ss.rpm, f1p=fz,
                blades_1p=blade_amps(model, s * hd.U, hubd), blades_2p=blade_amps(model, U2, hubd),
                blades_2p_quad_only=blade_amps(model, U2q, hubd),
                hubF_1p_fixed=fixed(s * hd.hub_force_H, 1), hubM_1p_fixed=fixed(s * hd.hub_moment_H, 1),
                hubF_2p_fixed=fixed(F2h, 2), hubM_2p_fixed=fixed(M2h, 2),
                hubF_2p_quad_fixed=fixed(F2hq, 2), hubM_2p_quad_fixed=fixed(M2hq, 2),
                strip=info, strip_2p=strip, checks_with_2p=chk, checks_1p_only=ch0)


if __name__ == "__main__":
    pcs = [float(x) for x in sys.argv[1:]] or [0.0, 1.3]
    with ProcessPoolExecutor(2) as ex:
        res = list(ex.map(case, pcs))
    (HERE / "results" / "two_per_rev.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps(res, indent=1, default=float))
