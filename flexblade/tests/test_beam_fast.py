"""beam_fast kernels vs the reference beam.py kernels, on randomised deformed states of
the real 5" tpu_rod model; consistent aero tangent of the trim Newton.

Tolerances (relative to the max |entry| of each array):
  values (d7, f7, positions, f_int):              1e-12
  first FD derivatives (B, ∂V, J, F_cf, K_mat, M):  1e-9
  second FD derivatives (K_geo, Hessian):          1e-6
The last is the reference's own noise floor: a mixed central difference with
h = 1e-4 amplifies 1e-16 rounding in φ = f7·d7 + V by 1/h² — reordering a 3×3 product
(numba-BLAS vs explicit scalars) changes the result at that level.
"""
import numpy as np
import pytest

from flexblade import beam, beam_fast, presets, rotor


@pytest.fixture(scope="module")
def model():
    return rotor.build_model(presets.make_preset(5, "tpu_rod"))


def _state(fe, seed, u=1e-3, rot=0.05):
    rng = np.random.default_rng(seed)
    du = np.zeros(fe.ndof)
    du.reshape(-1, 6)[:, :3] = u * rng.standard_normal((fe.nn, 3))
    du.reshape(-1, 6)[:, 3:] = rot * rng.standard_normal((fe.nn, 3))
    return beam.update_state(fe.initial_state(), du)


def _rel(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return np.abs(a - b).max() / max(np.abs(a).max(), 1e-300)


def _args(fe, st):
    pk = beam._pack(fe)
    n0, n1 = pk["n0"], pk["n1"]
    return (np.ascontiguousarray(st.x[n0]), np.ascontiguousarray(st.x[n1]), np.ascontiguousarray(st.R[n0]),
            np.ascontiguousarray(st.R[n1]), pk["E0"], pk["L0"], pk["k2"], pk["K7"], pk["MP"], pk["NP"])


TOL = dict(D7=1e-12, F7=1e-12, B=1e-9, GV=1e-9, J=1e-9, POS=1e-12, KR=1e-6)


@pytest.mark.parametrize("seed", [0, 1, 2])
@pytest.mark.parametrize("Omega", [0.0, 2500.0])
def test_all_derivs_matches_reference(model, seed, Omega):
    fe = model.fe
    st = _state(fe, seed)
    a = _args(fe, st)
    ne = len(fe.elements)
    rng = np.random.default_rng(seed + 10)
    FW = rng.standard_normal((ne, 7)) * 100.0
    use_fw = rng.random(ne) < 0.5          # mixed: half the elements with predictor weights
    ref = beam._all_derivs(*a, Omega**2, True, FW, use_fw)
    new = beam_fast.all_derivs(*a, Omega**2, True, FW, use_fw, True)
    for name, r, n in zip(TOL, ref, new):
        if np.abs(r).max() == 0:
            assert np.abs(n).max() == 0, name
            continue
        assert _rel(r, n) < TOL[name], (name, _rel(r, n))


def test_serial_and_parallel_kernels_identical(model):
    fe = model.fe
    a = _args(fe, _state(fe, 5))
    ne = len(fe.elements)
    FW, uf = np.zeros((ne, 7)), np.zeros(ne, np.bool_)
    s = beam_fast._all_derivs_ser(*a, 2500.0**2, True, FW, uf, True)
    p = beam_fast._all_derivs_par(*a, 2500.0**2, True, FW, uf, True)
    for x, y in zip(s, p):
        assert np.array_equal(x, y)


@pytest.mark.parametrize("Omega", [0.0, 2500.0])
def test_assemble_matches_reference(model, Omega, monkeypatch):
    fe = model.fe
    st = _state(fe, 3)
    du_pred = 1e-4 * np.random.default_rng(4).standard_normal(fe.ndof)
    out = {}
    for fast in (False, True):
        monkeypatch.setattr(beam, "_FAST", fast)
        sa = beam.assemble(fe, st, Omega, True, True)
        sp = beam.assemble(fe, st, Omega, True, False, du_pred=du_pred)
        sc = np.linspace(0.5, 1.5, len(fe.elements)) * (1 + 0.1j)
        out[fast] = dict(fint=sa.fint, Fcf=sa.Fcf, Kmat=beam.assemble_Kmat(fe, sa), Kgeo=sa.Kgeo, M=sa.M,
                         G=sa.G, Kgeo_pred=sp.Kgeo, Kc=beam.assemble_Kmat(fe, sa, sc),
                         d7=np.array([c.d7 for c in sa.cr]))
    tol = dict(fint=1e-9, Fcf=1e-9, Kmat=1e-9, Kgeo=1e-6, M=1e-9, G=1e-9, Kgeo_pred=1e-6, Kc=1e-9, d7=1e-12)
    for k, t in tol.items():
        r, n = out[False][k], out[True][k]
        if np.abs(r).max() == 0:
            assert np.abs(n).max() == 0, k
            continue
        assert _rel(r, n) < t, (k, _rel(r, n))


def test_point_maps_batch(model):
    fe = model.fe
    st = _state(fe, 6)
    pts = [p for bi in model.blades for p in rotor._aero_points(model, bi)]
    idx = {id(e): k for k, e in enumerate(fe.elements)}
    r, sH, L, Nr = beam_fast.element_point_maps_batch(fe, st, [idx[id(p[0])] for p in pts],
                                                       [p[1] for p in pts], [p[4] for p in pts])
    for k, p in enumerate(pts):
        rr, ss, LL, NN = beam.element_point_maps(p[0], st, p[1], p[4])
        assert _rel(rr, r[k]) < 1e-14
        assert np.abs(ss - sH[k]).max() < 1e-15
        assert _rel(LL, L[k]) < 1e-9
        assert _rel(NN, Nr[k]) < 1e-12


def test_aero_shape_tangent_is_consistent(model):
    """Directional FD of the trim follower load f(q) = aero_load_vector(q, trim(mean shape(q))),
    minus its frozen-geometry part (load direction following the element, deliberately
    not in the tangent), must equal the tangent."""
    aero = rotor.make_aero(model)
    if aero is None:
        pytest.skip("aero unavailable")
    fe = model.fe
    st = _state(fe, 7, u=2e-5, rot=2e-3)
    rpm = 24000.0

    def shape(s):
        th, be = rotor._blade_shape(model, s)
        return dict(theta=th.mean(0), beta=be.mean(0), r=model.r_aero)

    tr0 = aero.trim(rpm=rpm, blade_shape=shape(st))

    def f(s, frozen=False):
        return rotor.aero_load_vector(model, s, tr0 if frozen else aero.trim(rpm=rpm, blade_shape=shape(s)))

    K = rotor.aero_shape_tangent(model, st, rotor.aero_shape_sens(model, aero, rpm, shape(st)))
    rng = np.random.default_rng(8)
    for comp in range(6):
        v = np.zeros(fe.ndof)
        v.reshape(-1, 6)[:, comp] = rng.standard_normal(fe.nn)
        h = 1e-5 if comp >= 3 else 1e-7
        sp, sm = beam.update_state(st, h * v), beam.update_state(st, -h * v)
        fd = ((f(sp) - f(sm)) - (f(sp, True) - f(sm, True))) / (2 * h)
        assert np.abs(fd - K @ v).max() <= 2e-3 * max(np.abs(fd).max(), 1e-3), (comp, _rel(fd, K @ v))


def test_trim_fast_matches_reference(monkeypatch):
    d = presets.make_preset(5, "tpu_rod")
    d.op.rpm = 24000.0
    res = {}
    for fast in (True, False):
        monkeypatch.setattr(beam, "_FAST", fast)
        m = rotor.build_model(d)
        ss = rotor.static_trim(m, rotor.make_aero(m))
        assert ss.converged and not any("converge" in w for w in ss.warnings), ss.warnings
        res[fast] = ss.state.x - m.fe.Xref
    # same equilibrium to well within the static tolerance (soft antisymmetric pitch mode)
    assert _rel(res[False], res[True]) < 1e-4
