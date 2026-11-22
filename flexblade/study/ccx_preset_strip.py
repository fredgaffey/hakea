"""Loaded ccx check of ONE flexure strip lamina at hover, with the slot preset (I34) as an imposed
clamp-end rotation (verify_10in_v8.md gap: ccx compare_strip_flexure meshes the strips unloaded).

Beam side: rotor.static_trim at T0.7 / T1.0 (robust.variants). For each strip the effective clamp
frames are F = R_node @ Q_slot (beam.slot_frames), so F already contains the preset. The relative
blade-clamp motion in the hub-clamp frame (u = p - L e1, rotation log(Fa^T Fb)) is imposed on a
straight hex20 C3D20R lamina (hub face encastre, blade face rigid body), NLGEOM. One 0.1 mm
lamina sees the same end kinematics as each of the 8 (laminae unbonded -> no scaling needed for
stress). Compared at the clamp edges: beam max lamina fibre stress (sm, x Kt like the Goodman
diag) vs ccx surface S11 (linear through-thickness fit of the Gauss-point S11 at the clamp row).

usage: DN=<design> TF=1.0,1.25 python study/ccx_preset_strip.py [strip|axis|stopped|preset]
"""
import sys
import tempfile
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from flexblade import ccx, robust, rotor as R
from flexblade import materials as M
from flexblade.beam import assemble, element_end_forces, slot_frames
from flexblade.schema import Design

HERE = Path(__file__).resolve().parent
DES = HERE / "designs" / (__import__("os").environ.get("DN","10in_tt_strap_steelx_v8") + ".json")
G3 = 1 / np.sqrt(3)
# ccx gauss3d2 order (xi fastest) and C3D20 corner natural coords
IPS = np.array([(x, y, z) for z in (-G3, G3) for y in (-G3, G3) for x in (-G3, G3)])
CN = np.array([(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1), (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)])


def beam_hover(d: Design, stopped: bool = False):
    if d.flexure.pre_lag_deg == "auto":
        d = R.resolve_pre_lag(d)
    if stopped:     # bench/stopped self-stressed state (fact.stopped_state: no CF, no aero)
        from flexblade.fact import stopped_state
        r = stopped_state(d, n_steps=4)
        model, st, Om = r["model"], r["state"], 0.0
    else:
        model = R.build_model(d)
        ss = R.static_trim(model, R.make_aero(model))
        st, Om = ss.state, ss.Omega
    fe = model.fe
    sa = assemble(fe, st, Om, want_tangent=False)
    kt = R.strip_kt(d.flexure)
    out = []
    for bi in model.blades:
        for j, m in enumerate(bi.members):
            ids = [i for i, e in enumerate(fe.elements) if e.tag == f"b{bi.k}.flex{j}" and e.kind == "flexure"]
            if not ids or getattr(m, "cable", False):
                continue
            ea, eb = fe.elements[ids[0]], fe.elements[ids[-1]]
            fa, _ = element_end_forces(ea, sa.cr[ids[0]], st)
            _, fb = element_end_forces(eb, sa.cr[ids[-1]], st)
            sec = ea.sections[0]
            sm_all = max(max(R._fibre_mean(s_, f_)) for i in ids for s_, f_ in
                         zip((fe.elements[i].sections[0], fe.elements[i].sections[-1]),
                             element_end_forces(fe.elements[i], sa.cr[i], st))) * kt
            oop = []
            for s_, f_ in ((ea.sections[0], fa), (eb.sections[-1], fb)):
                ep = np.linalg.solve(s_.stiffness6(), f_)
                Es = s_.fibres[0].E
                oop.append((Es * abs(ep[4]) * m.thickness / 2, Es * abs(ep[5]) * m.width / 2))
            sm = [max(R._fibre_mean(s, f)) * kt for s, f in ((ea.sections[0], fa), (eb.sections[-1], fb))]
            mem = [float(np.mean(R._fibre_mean(s, f))) for s, f in ((ea.sections[0], fa), (eb.sections[-1], fb))]
            q = slot_frames(ea), slot_frames(eb)
            Qa = np.eye(3) if q[0] is None else q[0][0]
            Qb = np.eye(3) if q[1] is None else q[1][1]
            Fa, Fb = st.R[ea.n0] @ Qa, st.R[eb.n1] @ Qb
            E0 = ea.E0
            L = sum(fe.elements[i].L0 for i in ids)
            p = E0.T @ Fa.T @ (st.x[eb.n1] - st.x[ea.n0])
            Rrel = E0.T @ Fa.T @ Fb @ E0
            zs = np.array([fb_.z for fb_ in sec.fibres])
            ys = np.array([fb_.y for fb_ in sec.fibres])
            S6, Ef = sec.stiffness6(), sec.fibres[0].E
            nlam = max(S6[4, 4], S6[5, 5]) / (Ef * m.thickness * m.width ** 3 / 12)   # in-plane EI = n x lamina
            out.append(dict(tag=f"b{bi.k}.flex{j}", oop=oop, nlam=nlam, fb_hub=np.r_[Rrel @ fb[:3], Rrel @ fb[3:]], sm_all=sm_all, sm_hub=sm[0], sm_blade=sm[1], memb=mem, L=L,
                            u=p - [L, 0, 0], rot=Rotation.from_matrix(Rrel).as_rotvec(), Rrel=Rrel,
                            preset=np.degrees(np.r_[ea.d7_0[1:4], eb.d7_0[4:7]]) if ea.d7_0 is not None else None,
                            zr=(zs.min(), zs.max()), yr=(ys.min(), ys.max()), width=m.width, t=m.thickness))
    return out


def ccx_strip(s, mat, wd: Path, nl=40, nw=12, nt=2, mixed=True, s0=None, inplane=False, loads=False):
    """s0: optional first-step kinematics (stopped/preset-only state): the strip is first bent into the
    slot-misfit shape (step 1), then taken to s (step 2). inplane: also impose u_y / theta_z (in-plane
    bending M3; u_x stays free). loads: mixed BCs + the beam's blade-end axial / in-plane force and
    in-plane moment per lamina (fb_hub / nlam) as *CLOAD (fixed hub-frame directions). Returns {hub, blade[, pre]} ({hub, blade} of step 1 under 'pre')."""
    L, wid, th = s["L"] * 1e3, s["width"] * 1e3, s["t"] * 1e3
    # thickness along the local axis with the small fibre spread
    thick_z = (s["zr"][1] - s["zr"][0]) < (s["yr"][1] - s["yr"][0])
    ext = np.array([L, wid, th]) if thick_z else np.array([L, th, wid])
    key, nodes, elems, eidx = {}, [], [], []
    n = (2 * nl, 2 * nw, 2 * nt) if thick_z else (2 * nl, 2 * nt, 2 * nw)

    def nid(i, j, k):
        if (i, j, k) not in key:
            key[(i, j, k)] = len(nodes)
            nodes.append(np.array([i / n[0], j / n[1] - 0.5, k / n[2] - 0.5]) * ext)
        return key[(i, j, k)]
    for a in range(n[0] // 2):
        for b in range(n[1] // 2):
            for c in range(n[2] // 2):
                elems.append([nid(2 * a + di, 2 * b + dj, 2 * c + dk) for di, dj, dk in ccx._HEX20])
                eidx.append(a)
    X = np.array(nodes)
    hub = [v + 1 for (i, _, _), v in key.items() if i == 0]
    bl = [v + 1 for (i, _, _), v in key.items() if i == n[0]]
    ref, rot = len(X) + 1, len(X) + 2
    o = ["*NODE"] + [f"{i + 1}, {x:.9g}, {y:.9g}, {z:.9g}" for i, (x, y, z) in enumerate(X)]
    o += [f"{ref}, {L:.9g}, 0, 0", f"{rot}, {L:.9g}, 0, 0", "*ELEMENT, TYPE=C3D20R, ELSET=STRIP"]
    o += [f"{k + 1}, " + ", ".join(str(v + 1) for v in e[:15]) + ",\n" + ", ".join(str(v + 1) for v in e[15:])
          for k, e in enumerate(elems)]
    for nm, ns in (("HUB", hub), ("BL", bl)):
        o.append(f"*NSET, NSET={nm}")
        o += [", ".join(map(str, ns[i:i + 12])) for i in range(0, len(ns), 12)]
    o += ["*MATERIAL, NAME=STEEL", ccx._elastic_card(mat).rstrip(), "*SOLID SECTION, ELSET=STRIP, MATERIAL=STEEL",
          f"*RIGID BODY, NSET=BL, REF NODE={ref}, ROT NODE={rot}",
          ]
    soft_u, soft_r = ((2,), (0, 1)) if thick_z else ((1,), (0, 2))
    if inplane:
        soft_u, soft_r = soft_u + ((1,) if thick_z else (2,)), soft_r + ((2,) if thick_z else (1,))
    for k_ in ([s0] if s0 is not None else []) + [s]:
        um, rv = k_["u"] * 1e3, k_["rot"]
        o += ["*STEP, NLGEOM, INC=500", "*STATIC", "0.05, 1., 1e-5, 0.1", "*BOUNDARY", "HUB, 1, 3, 0."]
        o += [f"{ref}, {i + 1}, {i + 1}, {um[i]:.9g}" for i in (soft_u if mixed else range(3))]
        o += [f"{rot}, {i + 1}, {i + 1}, {rv[i]:.9g}" for i in (soft_r if mixed else range(3))]
        if loads:
            f_ = k_["fb_hub"] / k_["nlam"]
            iu, ir = (1, 2) if thick_z else (2, 1)
            o += ["*CLOAD", f"{ref}, 1, {f_[0]:.9g}", f"{ref}, {iu + 1}, {f_[iu]:.9g}",
                  f"{rot}, {ir + 1}, {f_[3 + ir] * 1e3:.9g}"]
        o += ["*EL PRINT, ELSET=STRIP", "S", "*END STEP"]
    wd.mkdir(parents=True, exist_ok=True)
    (wd / "model.inp").write_text("\n".join(o) + "\n")
    res = ccx.run(wd, threads=1)
    out = _recover(res["stress"][-1], X, elems, eidx, nl, s["Rrel"], thick_z, th)
    if s0 is not None:      # step-1 (time <= 1) blocks only: truncate the .dat at the first later header
        lines = (wd / "model.dat").read_text().splitlines()
        cut = next(i for i, ln in enumerate(lines) if ln.strip().lower().startswith("stresses (elem")
                   and float(ln.split()[-1]) > 1.0 + 1e-6)
        (wd / "step1.dat").write_text("\n".join(lines[:cut]) + "\n")
        out["pre"] = _recover(ccx._parse_dat(wd / "step1.dat")["stress"][-1], X, elems, eidx, nl, s0["Rrel"],
                              thick_z, th)
    return out


def _recover(S, X, elems, eidx, nl, Rrel, thick_z, th):
    # Gauss-point reference coordinates
    E = np.array(elems)
    out = {}
    for end, arow, ipx, Rt in (("hub", 0, -G3, np.eye(3)), ("blade", nl - 1, G3, Rrel)):
        rows = []
        for r_ in S:
            k, g = int(r_[0]) - 1, int(r_[1]) - 1
            if eidx[k] != arow or abs(IPS[g][0] - ipx) > 1e-9:
                continue
            xi = IPS[g]
            N = np.prod(1 + CN * xi, axis=1) / 8
            pos = N @ X[E[k][:8]]
            sx, sy, sz, sxy, sxz, syz = r_[2:8]
            T = np.array([[sx, sxy, sxz], [sxy, sy, syz], [sxz, syz, sz]])
            rows.append((pos, (Rt.T @ T @ Rt)[0, 0]))
        P = np.array([r[0] for r in rows])
        s11 = np.array([r[1] for r in rows])
        ti, wi = (2, 1) if thick_z else (1, 2)
        surf, memb = [], []
        for wv in np.unique(np.round(P[:, wi], 6)):
            m = np.abs(P[:, wi] - wv) < 1e-6
            A = np.c_[np.ones(m.sum()), P[m, ti]]
            c, *_ = np.linalg.lstsq(A, s11[m], rcond=None)
            surf.append(max(c[0] + c[1] * th / 2, c[0] - c[1] * th / 2))
            memb.append(c[0])
        surf = np.array(surf)
        out[end] = dict(peak=float(surf.max()) * 1e6, mid=float(np.median(surf)) * 1e6,
                        memb=float(np.median(memb)) * 1e6)
    return out


def preset_stage():
    """Preset-curved strip: ccx step 1 = beam stopped (slot-misfit only) end kinematics, step 2 = hover
    end kinematics (TF env), mixed (oop only) and +in-plane (u_y, theta_z imposed) BCs. Beam split:
    stopped (preset) vs hover sm, out-of-plane / in-plane bending parts. Design material."""
    import os
    D = Design.model_validate_json(DES.read_text())
    mat = M.get_material(D.flexure.layup.material)
    kt = R.strip_kt(D.flexure)
    with R.part_temperature(D.op.ambient_C):
        stop = {s_["tag"]: s_ for s_ in beam_hover(D.model_copy(deep=True), stopped=True)}
    tfs = tuple(float(x) for x in os.environ.get("TF", "1.0,1.25").split(","))
    for tag, d in robust.variants(D, thrust_factors=tfs):
        with R.part_temperature(d.op.ambient_C):
            strips = beam_hover(d)
        b = max(strips, key=lambda s_: max(s_["sm_hub"], s_["sm_blade"]))
        b0 = stop[b["tag"]]
        print(f"== {tag} binding {b['tag']} kt {kt:.2f} preset_deg {np.round(b['preset'], 2) if b['preset'] is not None else None}")
        for nm, k_ in (("stopped", b0), ("hover", b)):
            print(f"   beam {nm}: u_mm {np.round(k_['u'] * 1e3, 4)} rot_deg {np.round(np.degrees(k_['rot']), 3)}; "
                  + "; ".join(f"{e} surf {k_['sm_' + e] / kt / 1e6:.1f} (oop {k_['oop'][i][0] / 1e6:.1f}, in-plane "
                              f"{k_['oop'][i][1] / 1e6:.1f}, memb {k_['memb'][i] / 1e6:.1f})"
                              for i, e in enumerate(("hub", "blade"))) + f"; strip max xKt {k_['sm_all'] / 1e6:.1f}")
        print(f"   beam blade-end action (hub frame, per lamina /{b['nlam']:.2f}): F_N {np.round(b['fb_hub'][:3] / b['nlam'], 4)}"
              f" M_Nmm {np.round(b['fb_hub'][3:] / b['nlam'] * 1e3, 3)}")
        modes = {"mixed": dict(), "inplane": dict(inplane=True), "loads": dict(loads=True)}
        for mn in os.environ.get("CCXMODES", "mixed,inplane,loads").split(","):
            c = ccx_strip(b, mat, Path(tempfile.mkdtemp(prefix="v8pre_", dir="/tmp")), s0=b0, **modes[mn])
            for st_, cc, bb in (("step1 preset", c["pre"], b0), ("step2 hover", c, b)):
                print(f"   ccx {mn} {st_}: " + "; ".join(
                    f"{e} peak {cc[e]['peak'] / 1e6:.1f} med {cc[e]['mid'] / 1e6:.1f} memb {cc[e]['memb'] / 1e6:.1f}"
                    f" ccx/beam(peak) {cc[e]['peak'] / (bb['sm_' + e] / kt):.2f}" for e in ("hub", "blade")))


def strip_stage():
    D = Design.model_validate_json(DES.read_text())
    for tag, d in robust.variants(D, thrust_factors=tuple(float(x) for x in __import__("os").environ.get("TF","0.7,1.0").split(","))):
        with R.part_temperature(d.op.ambient_C):
            strips = beam_hover(d)
            mat = M.get_material("steel_1095_shim")
        b = max(strips, key=lambda s: max(s["sm_hub"], s["sm_blade"]))
        print(f"== {tag}: strips (beam sm hub/blade MPa):",
              ", ".join(f"{s['tag']} {s['sm_hub']/1e6:.0f}/{s['sm_blade']/1e6:.0f} (max {s['sm_all']/1e6:.0f})" for s in strips))
        print(f"   binding {b['tag']} u_mm={np.round(b['u']*1e3, 4)} rot_deg={np.round(np.degrees(b['rot']), 3)}"
              f" preset_deg={np.round(b['preset'], 3)} fibre z={b['zr']} y={b['yr']}")
        c = ccx_strip(b, mat, Path(tempfile.mkdtemp(prefix="v8strip_", dir="/tmp")))
        for end in ("hub", "blade"):
            sb = b[f"sm_{end}"] / R.strip_kt(d.flexure)
            print(f"   {end}: beam sm(xKt) {b[f'sm_{end}']/1e6:.1f}  beam surf {sb/1e6:.1f}  "
                  f"ccx surf peak {c[end]['peak']/1e6:.1f} median {c[end]['mid']/1e6:.1f}  "
                  f"ccx/beam(peak) {c[end]['peak']/sb:.3f} (median) {c[end]['mid']/sb:.3f}  "
                  f"beam out-of-plane/in-plane bending {b['oop'][0 if end == 'hub' else 1][0]/1e6:.1f}/"
                  f"{b['oop'][0 if end == 'hub' else 1][1]/1e6:.1f}  membrane beam {b['memb'][0 if end == 'hub' else 1]/1e6:.1f} ccx {c[end]['memb']/1e6:.1f}")


def stopped_stage():
    """Stopped rotor (slot misfit only), binding strip; mixed BCs as in strip_stage."""
    d = Design.model_validate_json(DES.read_text())
    strips = beam_hover(d, stopped=True)
    mat = M.get_material("steel_1095_shim")
    kt = R.strip_kt(d.flexure)
    for b in sorted(strips, key=lambda s: -max(s["sm_hub"], s["sm_blade"]))[:2]:
        print(f"== stopped {b['tag']} u_mm={np.round(b['u'] * 1e3, 4)} rot_deg={np.round(np.degrees(b['rot']), 3)}")
        c = ccx_strip(b, mat, Path(tempfile.mkdtemp(prefix="v8stop_", dir="/tmp")))
        for end in ("hub", "blade"):
            sb = b[f"sm_{end}"] / kt
            print(f"   {end}: beam surf {sb / 1e6:.1f} (oop {b['oop'][0 if end == 'hub' else 1][0] / 1e6:.1f}, in-plane "
                  f"{b['oop'][0 if end == 'hub' else 1][1] / 1e6:.1f})  ccx peak {c[end]['peak'] / 1e6:.1f} median "
                  f"{c[end]['mid'] / 1e6:.1f}  membrane beam {b['memb'][0 if end == 'hub' else 1] / 1e6:.1f} "
                  f"ccx {c[end]['memb'] / 1e6:.1f}")


def axis_stage():
    """Beam principal pitch-axis offset vs design axis: v8, v8 presets zeroed, v8 at skew 60 (v5)."""
    from flexblade import geometry as G
    D = Design.model_validate_json(DES.read_text())

    def off(d):
        members, (hp, ax), _ = G.get_members(d)
        members = [m for m in members if m.kind != "wire" and m.layup.material != "aramid_yarn"]
        d2 = d.model_copy(deep=True)
        d2.flexure.strap_strands = 0
        d2.flexure.axial_release = False
        d2.analysis.member_anchorage = "none"
        K = R.flexure_only_model(d2, members).stiffness_at(np.asarray(hp, float))
        Cr = np.linalg.inv(K)[3:, 3:]
        v = np.linalg.eigh(Cr)[1][:, -1]
        a = np.asarray(ax, float) / np.linalg.norm(ax)
        return float(np.degrees(np.arccos(min(1.0, abs(v @ a)))))
    z = D.model_copy(deep=True)
    z.flexure.strip_preset_deg = [[[0.0] * 6 for _ in b] for b in D.flexure.strip_preset_deg]
    s60 = D.model_copy(deep=True)
    s60.flexure.skew_deg = 60.0
    s60z = z.model_copy(deep=True)
    s60z.flexure.skew_deg = 60.0
    for nm, d in (("v8", D), ("v8 presets=0", z), ("v8 skew60", s60), ("v8 skew60 presets=0", s60z)):
        print(f"axis offset {nm}: {off(d):.4f} deg")


if __name__ == "__main__":
    {"strip": strip_stage, "axis": axis_stage, "stopped": stopped_stage, "preset": preset_stage}[sys.argv[1] if len(sys.argv) > 1 else "strip"]()
