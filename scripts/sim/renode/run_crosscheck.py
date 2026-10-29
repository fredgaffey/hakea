#!/usr/bin/env python3
"""Run one Hakea throttle-sine stimulus on an AM32 firmware in Renode.

Usage (from ~/vehicles/hakea, with ~/venv-ardupilot active):

  scripts/sim/renode/run_crosscheck.py FOXEER_F421 \
      am32-renode/variants/FOXEER_F421_ramp16.elf --name F421_ramp16

What happens:
 1. ESCSim's Renode generator (python -m escsim.renode.generator) builds
    the emulated board for TARGET and starts Renode with our ELF, Fred's
    EEPROM image (~/Downloads/esc1_config.bin) loaded into the emulated
    flash settings page, and the Hakea motor model JSON given to the
    native motor physics (libam32sim, the same motor.c as AM32's SITL).
    Renode starts paused, with its command monitor on a local TCP port.
 2. We load hakea_stim.py into Renode's Python, switch the throttle
    generator to DShot300 at one frame per --frame-us, hold zero until
    the firmware's own `armed` flag is set, then play the throttle
    profile step by step in virtual time, logging every step to CSV.
 3. Renode is told to quit; if it does not, its process group is killed.

The profile (virtual time):
  ramp 48 -> BASE over 0.5 s, hold BASE 1.0 s,
  BASE + 200 sin(2 pi 250 t) for SINE_S, hold BASE 0.3 s,
  BASE + 600 sin(2 pi 250 t) for SINE_S, then 0 for 0.3 s.
A segments JSON next to the CSV records where each part starts/ends.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from renode_monitor import Monitor  # noqa: E402

HAKEA = os.path.expanduser("~/vehicles/hakea")
DEFAULT_MODEL = os.path.join(HAKEA, "scripts/sim/models/hakea_2307_5in.json")
DEFAULT_EEPROM = os.path.expanduser("~/Downloads/esc1_config.bin")
DEFAULT_TARGETS_H = os.path.join(HAKEA, "am32-renode/Inc/targets.h")
FIELDS = ['armed', 'running', 'input', 'adjusted_input', 'duty_cycle',
          'adjusted_duty_cycle', 'last_duty_cycle', 'max_ramp_high_rpm',
          'e_com_time', 'commutation_interval', 'average_interval',
          'zero_crosses', 'desync_happened', 'bemf_timeout_happened',
          'actual_current', 'battery_voltage', 'dshot_goodcounts',
          'dshot_badcounts']


def elf_symbols(path, names):
    from elftools.elf.elffile import ELFFile
    out = {}
    with open(path, "rb") as f:
        for sec in ELFFile(f).iter_sections():
            if sec.header.sh_type != "SHT_SYMTAB":
                continue
            for s in sec.iter_symbols():
                if s.name in names:
                    out[s.name] = (int(s.entry.st_value), int(s.entry.st_size))
    return out


def build_profile(base, step_us, sine_s, f_hz=250.0, amps=(200, 600)):
    """list of DShot values, one per step, plus named segment bounds"""
    dt = step_us * 1e-6
    vals, segs = [], []

    def seg(name, seconds, fn):
        n = int(round(seconds / dt))
        start = len(vals)
        for i in range(n):
            vals.append(int(round(fn(i * dt))))
        segs.append({"name": name, "start_step": start, "end_step": len(vals),
                     "seconds": n * dt})

    seg("ramp", 0.5, lambda t: 48 + (base - 48) * t / 0.5)
    seg("settle", 1.0, lambda t: base)
    for k, a in enumerate(amps):
        seg("sine%d" % a, sine_s,
            lambda t, a=a: min(2047, max(48, base + a * math.sin(2 * math.pi * f_hz * t))))
        if k + 1 < len(amps):
            seg("hold%d" % (k + 1), 0.3, lambda t: base)
    seg("zero", 0.3, lambda t: 0)
    return vals, segs


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target")
    ap.add_argument("elf")
    ap.add_argument("--name", required=True, help="run name, used for file names")
    ap.add_argument("--outdir", default=os.path.join(HAKEA, "notes/sim/renode/raw"))
    ap.add_argument("--workdir", default=os.path.join(HERE, "work"))
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--eeprom", default=DEFAULT_EEPROM)
    ap.add_argument("--targets-h", default=DEFAULT_TARGETS_H)
    ap.add_argument("--monitor-port", type=int, default=34571,
                    help="TCP port for the Renode monitor (not UDP 58xxx)")
    ap.add_argument("--can-bus", type=int, default=-1,
                    help="CAN targets: -1 leaves the emulated bxCAN unconnected; "
                    "N joins loopback-only multicast bus 239.65.82.N:57732")
    ap.add_argument("--batch-us", type=int, default=0,
                    help="override the bridge/physics exchange period "
                    "(ESCSim default 20 us) via gen_batch_wrapper.py")
    ap.add_argument("--base", type=int, default=1047)
    ap.add_argument("--step-us", type=int, default=100,
                    help="virtual time per throttle update / log row")
    ap.add_argument("--frame-us", type=int, default=100,
                    help="DShot frame period on the emulated wire")
    ap.add_argument("--sine-s", type=float, default=0.8)
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    work = os.path.join(args.workdir, args.name)
    os.makedirs(work, exist_ok=True)

    vals, segs = build_profile(args.base, args.step_us, args.sine_s)
    prof_path = os.path.join(work, "profile.txt")
    with open(prof_path, "w") as f:
        f.write("\n".join(str(v) for v in vals) + "\n")

    env = dict(os.environ)
    if args.batch_us:
        env["BATCH_US"] = str(args.batch_us)
        gen = [sys.executable, os.path.join(HERE, "gen_batch_wrapper.py")]
    else:
        gen = [sys.executable, "-m", "escsim.renode.generator"]
    cmd = gen + [args.target,
           "--outdir", work, "--targets-file", args.targets_h,
           "--elf", os.path.abspath(args.elf), "--eeprom", args.eeprom,
           "--model", args.model, "--monitor-port", str(args.monitor_port),
           "--can-bus", str(args.can_bus),
           "--run"]
    log = open(os.path.join(work, "renode.log"), "w")
    print("launching:", " ".join(cmd))
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, env=env)
    t0 = time.time()
    csv_path = os.path.join(args.outdir, args.name + ".csv")
    try:
        m = Monitor(args.monitor_port, connect_timeout=180)
        print(m.cmd('python "execfile(\'%s\')"' % os.path.join(HERE, "hakea_stim.py")))
        syms = elf_symbols(args.elf, set(FIELDS) | {"eepromBuffer"})
        print(m.cmd('python "set_syms(%r)"' % syms))
        print(m.cmd('python "setup_dshot(300, %d)"' % args.frame_us))
        armed = m.cmd('python "print wait_armed(6.0)"', timeout=900)
        print("armed:", armed, "vt:", m.cmd('python "print vtime()"'))
        if "True" not in armed:
            print(m.cmd('python "status()"'))
            raise RuntimeError("firmware never armed")
        # Fred's EEPROM as the firmware actually holds it, for the record
        eb = m.cmd('python "print list(_sb.ReadBytes(%d, 48))"' % syms["eepromBuffer"][0])
        t_run = time.time()
        print(m.cmd('python "run_profile(\'%s\', %d, \'%s\')"' % (prof_path, args.step_us, csv_path),
                    timeout=36000))
        wall = time.time() - t_run
        vt_s = len(vals) * args.step_us * 1e-6
        meta = {"name": args.name, "target": args.target, "elf": os.path.abspath(args.elf),
                "model": args.model, "model_json": json.load(open(args.model)),
                "eeprom": args.eeprom, "eeprom_buffer_first48": eb,
                "bridge_batch_us": args.batch_us or 20, "base": args.base, "step_us": args.step_us, "frame_us": args.frame_us,
                "dshot_kbaud": 300, "sine_hz": 250.0, "segments": segs,
                "profile_virtual_s": vt_s, "profile_wall_s": wall,
                "realtime_factor": vt_s / wall if wall else None}
        with open(os.path.join(args.outdir, args.name + ".json"), "w") as f:
            json.dump(meta, f, indent=1)
        print("profile %.2f s virtual in %.1f s wall (%.3fx realtime)" % (vt_s, wall, vt_s / wall))
        try:
            m.sock.sendall(b"quit\n")
        except OSError:
            pass
        m.close()
        proc.wait(timeout=30)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        # Renode may outlive the generator wrapper; kill the whole group
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    print("total wall %.1f s; csv %s" % (time.time() - t0, csv_path))


if __name__ == "__main__":
    main()
