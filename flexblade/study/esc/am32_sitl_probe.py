#!/usr/bin/env python3
"""AM32 SITL probe for 1P torque pulsing on 2-3" rotors (unmodified firmware).

Build the SITL from a copy of Fred's am32-hakea fork (no source edits):
  rsync -a --exclude=.git am32-hakea/ /tmp/claude-1000/esc/am32/
  make -C /tmp/claude-1000/esc/am32 AM32_SITL_CAN SITL_COV_FLAGS=-Wno-error
(-Wno-error only because cyclic.c has unused static helpers.)

Modes
  sweep : DShot throttle staircase, records firmware e_rpm, commutation
          interval, zero-cross filter, desyncs and model omega per step.
  mod   : holds an operating point, then modulates the DShot throttle
          sinusoidally at f_mod (sim time) and measures the realised PWM
          duty and bus current at f_mod: amplitude ratio and phase lag.
          This is the only modulation path in the unmodified firmware
          (FRAME_CYCLIC is a TODO in dshot.c, cyclic_apply() returns duty).
"""
import argparse, json, math, os, socket, struct, subprocess, sys, tempfile, threading, time

ELF = "/tmp/claude-1000/esc/am32/obj/AM32_AM32_SITL_CAN_2.21.elf"
WATCH = [("e_rpm", 2), ("commutation_interval", 4), ("average_interval", 4),
         ("zc_index_conter", 1), ("zero_crosses", 4), ("desync_happened", 4),
         ("duty_cycle_setpoint", 2), ("adjusted_duty_cycle", 2), ("filter_level", 1),
         ("tim1_arr", 2)]


def dshot(val, telem=0):
    v = (val << 1) | telem
    crc = (v ^ (v >> 4) ^ (v >> 8)) & 0xF
    return (v << 4) | crc


class Sitl:
    def __init__(self, model, speedup, inport, stport, workdir):
        cfg = os.path.join(workdir, "model.json")
        json.dump(model, open(cfg, "w"))
        self.p = subprocess.Popen([ELF, "--config", cfg, "--eeprom", os.path.join(workdir, "ee.bin"),
                                   "--input-port", str(inport), "--state-port", str(stport),
                                   "--speedup", str(speedup), "--can-uri", "mcast:7", "--exit-on-reset"],
                                  stdout=subprocess.DEVNULL, stderr=open(os.path.join(workdir, "sitl.log"), "w"),
                                  cwd=workdir)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.inaddr, self.staddr = ("127.0.0.1", inport), ("127.0.0.1", stport)
        self.lock = threading.Lock()
        self.samples, self.events, self.last = [], [], {}
        self.period_ns, self.run = 50000, True
        time.sleep(0.5)
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._refresh, daemon=True).start()

    def _refresh(self):
        while self.run:
            self.sock.sendto(struct.pack("<HBBI", 0x5353, 0, 2, self.period_ns), self.staddr)
            body = b"".join(struct.pack("<B", s) + n.encode() + b"\0" for n, s in WATCH)
            self.sock.sendto(struct.pack("<HBBI", 0x5353, 8, len(WATCH), 0) + body, self.staddr)
            time.sleep(0.5)

    def _rx(self):
        while self.run:
            try:
                d, _ = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            mag, ver, cnt = struct.unpack_from("<HBB", d)
            if mag == 0x5354 and ver == 3:
                with self.lock:
                    for k in range(cnt):
                        o = 4 + 100 * k
                        t, om, th, the = struct.unpack_from("<Qfff", d, o)
                        iu, iv, iw, vu, vv, vw, vb, ib = struct.unpack_from("<8f", d, o + 20)
                        duty, des = struct.unpack_from("<fI", d, o + 92)
                        self.samples.append((t, om, th, the, ib, duty, des, iu, iv, iw))
            elif mag == 0x535A:
                with self.lock:
                    for k in range(cnt):
                        t, idx, raw = struct.unpack_from("<QIQ", d, 4 + 20 * k)
                        self.events.append((t, idx, raw))
                        self.last[WATCH[idx][0]] = raw

    def send(self, val):
        self.sock.sendto(struct.pack("<HBBHH", 0x4453, 3, 4, 0, dshot(val)), self.inaddr)

    def speed(self, s):
        self.sock.sendto(struct.pack("<HBBf", 0x5353, 2, 0, s), self.staddr)

    def hold(self, val, wall_s, rate=500):
        t_end = time.time() + wall_s
        while time.time() < t_end:
            self.send(val)
            time.sleep(1.0 / rate)

    def close(self):
        self.run = False
        self.p.kill()


def model_1103(kv, R, L, V, J, kload):
    return {"motor": {"kv": kv, "poles": 14, "resistance": R, "inductance": L, "inertia": J,
                      "damping": 1e-8, "static_friction": 1e-4, "load_k_omega2": kload},
            "battery": {"voltage": V, "resistance": 0.05}}


def sweep(a, s):
    rows = []
    s.hold(0, 2.0)  # arm
    for thr in range(150, 2048, a.step):
        s.samples.clear()
        s.hold(thr, a.dwell)
        with s.lock:
            sm = s.samples[-200:]
            lv = dict(s.last)
        om = sum(x[1] for x in sm) / max(1, len(sm))
        rpm = om * 60 / (2 * math.pi)
        row = dict(thr=thr, mech_rpm=round(rpm), erpm_model=round(rpm * 7),
                   erpm_fw=lv.get("e_rpm", 0) * 100, com_int_halfus=lv.get("commutation_interval"),
                   avg_int=lv.get("average_interval"), filt=lv.get("filter_level"),
                   desync=lv.get("desync_happened"), duty=round(sm[-1][5], 3) if sm else None,
                   tim1_arr=lv.get("tim1_arr"), zc=lv.get("zero_crosses"))
        rows.append(row)
        print(json.dumps(row), flush=True)
        if thr == 150 and rpm < 4000:
            raise RuntimeError("startup failed")
    return rows


def fit(ts, ys, f):
    n = len(ts)
    c = sum(math.cos(2 * math.pi * f * t) for t in ts)
    m = sum(ys) / n
    a = sum((y - m) * math.cos(2 * math.pi * f * t) for t, y in zip(ts, ys)) * 2 / n
    b = sum((y - m) * math.sin(2 * math.pi * f * t) for t, y in zip(ts, ys)) * 2 / n
    return m, math.hypot(a, b), math.degrees(math.atan2(b, a))


def modulate(a, s):
    s.hold(0, 2.0)
    for thr in range(150, a.base, 150):  # ramp: a step start desyncs
        s.hold(thr, 0.7)
    s.hold(a.base, 2.0)
    with s.lock:
        om = s.samples[-1][1] if s.samples else 0
    print("pre-mod rpm", round(om * 60 / 2 / math.pi), dict(s.last), file=sys.stderr, flush=True)
    if om * 60 / 2 / math.pi < a.min_rpm:
        raise RuntimeError("startup failed")  # AM32 startup on tiny J is flaky in SITL; retry
    out = []
    for f in a.freqs:
        sp = a.speedup_mod
        s.speed(sp)
        s.period_ns = 2000
        time.sleep(0.6)
        with s.lock:
            s.samples.clear(); s.events.clear()
        cmd_log = []
        t0 = time.time()
        while time.time() - t0 < a.mod_wall:
            tw = time.time() - t0
            v = int(a.base * (1 + a.depth * math.sin(2 * math.pi * f * tw * sp)))
            s.send(v)
            time.sleep(1.0 / a.rate)
        with s.lock:
            sm = list(s.samples); ev = list(s.events)
        ts = [x[0] * 1e-9 for x in sm]
        _, ad, pd = fit(ts, [x[5] for x in sm], f)
        md = sum(x[5] for x in sm) / len(sm)
        _, ai, pi_ = fit(ts, [x[4] for x in sm], f)
        mi = sum(x[4] for x in sm) / len(sm)
        sp_ev = [(e[0] * 1e-9, e[2]) for e in ev if WATCH[e[1]][0] == "duty_cycle_setpoint"]
        if len(sp_ev) > 10:
            ms, aset, pset = fit([x[0] for x in sp_ev], [x[1] for x in sp_ev], f)
        else:
            ms, aset, pset = 0, 0, 0
        zc = [e[0] * 1e-9 for e in ev if WATCH[e[1]][0] == "zc_index_conter"]
        rpm = sum(x[1] for x in sm) / len(sm) * 60 / (2 * math.pi)
        row = dict(f_mod=f, mech_rpm=round(rpm), duty_mean=round(md, 3),
                   duty_rel_amp=round(ad / md, 3), setpoint_rel_amp=round(aset / max(ms, 1), 3),
                   lag_setpoint_to_duty_deg=round((pset - pd + 180) % 360 - 180, 1),
                   ibus_mean=round(mi, 2), ibus_rel_amp=round(ai / max(mi, 1e-3), 3),
                   lag_duty_to_ibus_deg=round((pd - pi_ + 180) % 360 - 180, 1),
                   n=len(sm), zc_events=len(zc), sim_s=round(ts[-1] - ts[0], 4) if ts else 0)
        out.append(row)
        print(json.dumps(row), flush=True)
        s.speed(1.0); s.period_ns = 50000
        s.hold(a.base, 1.0)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["sweep", "mod"])
    ap.add_argument("--kv", type=float, default=11000)
    ap.add_argument("--R", type=float, default=0.25)
    ap.add_argument("--L", type=float, default=8e-6)
    ap.add_argument("--V", type=float, default=16.0)
    ap.add_argument("--J", type=float, default=1.5e-7)
    ap.add_argument("--kload", type=float, default=3.8e-10)
    ap.add_argument("--step", type=int, default=150)
    ap.add_argument("--dwell", type=float, default=1.5)
    ap.add_argument("--base", type=int, default=900)
    ap.add_argument("--depth", type=float, default=0.2)
    ap.add_argument("--freqs", type=float, nargs="+", default=[300, 500, 750, 1000])
    ap.add_argument("--speedup-mod", dest="speedup_mod", type=float, default=0.01)
    ap.add_argument("--mod-wall", dest="mod_wall", type=float, default=4.0)
    ap.add_argument("--rate", type=float, default=200)
    ap.add_argument("--port", type=int, default=58733)
    ap.add_argument("--out")
    ap.add_argument("--min-rpm", dest="min_rpm", type=float, default=15000)
    ap.add_argument("--tries", type=int, default=6)
    a = ap.parse_args()
    wd = tempfile.mkdtemp(prefix="am32sitl_", dir="/tmp/claude-1000/esc")
    for attempt in range(a.tries):
        s = Sitl(model_1103(a.kv, a.R, a.L, a.V, a.J, a.kload), 1.0, a.port, a.port + 1, wd)
        try:
            res = sweep(a, s) if a.mode == "sweep" else modulate(a, s)
            break
        except RuntimeError as e:
            print("attempt", attempt, e, file=sys.stderr)
        finally:
            s.close()
            time.sleep(0.5)
    if a.out:
        json.dump(dict(args=vars(a), rows=res), open(a.out, "w"), indent=1)
    print("workdir", wd, file=sys.stderr)
