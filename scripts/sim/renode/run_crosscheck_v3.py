#!/usr/bin/env python3
"""v3: one IN-ESC throttle sine (SIM_PROFILE firmware) in Renode.

  run_crosscheck_v3.py ELF --name NAME --freq F --amp A [--batch-us 2]

Boots FOXEER_F421 under ESCSim's Renode platform (bridge batch via
gen_batch_wrapper.py unless --batch-us 20), Fred's EEPROM, the 24.0 V
calibrated model. DShot300 unidirectional at one frame per 100 us of
virtual time. After arming: DShot ramp 48 -> BASE over 0.5 s, hold, a
logged 0.3 s baseline, then the firmware's internal sine is started by
writing sim_profile_phase=0, sim_profile_dphase, sim_profile_amp through
the bus (hakea_stim_v3.py). dphase = round(F / 19607.843 * 2^32): the
control loop runs at TMR14 1 MHz / 51 = 19607.843 Hz. Logging is at
--log-us (25 us) virtual for the baseline, the sine and 0.05 s after it.
"""
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from renode_monitor import Monitor  # noqa: E402
from run_crosscheck import elf_symbols  # noqa: E402

HAKEA = os.path.expanduser("~/vehicles/hakea")
LOOP_HZ = 1e6 / 51
FIELDS = ['armed', 'running', 'input', 'duty_cycle_setpoint', 'last_duty_cycle',
          'sim_profile_out', 'sim_profile_amp', 'sim_profile_phase', 'sim_profile_dphase',
          'max_ramp_high_rpm', 'tim1_arr', 'commutation_interval',
          'zero_crosses', 'desync_happened', 'bemf_timeout_happened',
          'actual_current', 'dshot_goodcounts', 'dshot_badcounts',
          'thiszctime', 'stress_window_peak_delta',
          'stress_window_reset_requested', 'pending_status_events', 'eepromBuffer']


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("elf")
    ap.add_argument("--name", required=True)
    ap.add_argument("--freq", type=float, required=True)
    ap.add_argument("--amp", type=int, required=True, help="duty units (0..2000 scale)")
    ap.add_argument("--sine-s", type=float, default=0.0,
                    help="default: 0.9 s at <=250 Hz, else 0.4 s")
    ap.add_argument("--batch-us", type=int, default=2)
    ap.add_argument("--log-us", type=int, default=25)
    ap.add_argument("--window-ms", type=float, default=50.0)
    ap.add_argument("--base", type=int, default=1048)
    ap.add_argument("--hold-s", type=float, default=0.7)
    ap.add_argument("--baseline-s", type=float, default=0.3)
    ap.add_argument("--post-s", type=float, default=0.05)
    ap.add_argument("--frame-us", type=int, default=100)
    ap.add_argument("--monitor-port", type=int, default=34571)
    ap.add_argument("--target", default="FOXEER_F421")
    ap.add_argument("--model", default=os.path.join(
        HAKEA, "scripts/sim/models/hakea_calibrated_prop_24v0.json"))
    ap.add_argument("--eeprom", default=os.path.expanduser("~/Downloads/esc1_config.bin"))
    ap.add_argument("--targets-h", default=os.path.join(HAKEA, "am32-renode-pr438/Inc/targets.h"))
    ap.add_argument("--outdir", default=os.path.join(HAKEA, "notes/sim/renode/v3/raw"))
    ap.add_argument("--workdir", default=os.path.join(HERE, "work_v3"))
    args = ap.parse_args()
    args.elf = os.path.abspath(args.elf)
    args.outdir = os.path.abspath(args.outdir)
    sine_s = args.sine_s or (0.9 if args.freq <= 250 else 0.4)
    dphase = int(round(args.freq / LOOP_HZ * 2 ** 32))
    window_steps = int(round(args.window_ms * 1000 / args.log_us)) if args.window_ms else 0

    os.makedirs(args.outdir, exist_ok=True)
    work = os.path.join(args.workdir, args.name)
    os.makedirs(work, exist_ok=True)
    env = dict(os.environ)
    if args.batch_us != 20:
        env["BATCH_US"] = str(args.batch_us)
        gen = [sys.executable, os.path.join(HERE, "gen_batch_wrapper.py")]
    else:
        gen = [sys.executable, "-m", "escsim.renode.generator"]
    cmd = gen + [args.target, "--outdir", work, "--targets-file", args.targets_h,
                 "--elf", args.elf, "--eeprom", args.eeprom, "--model", args.model,
                 "--monitor-port", str(args.monitor_port), "--can-bus", "-1", "--run"]
    csv_path = os.path.join(args.outdir, args.name + ".csv")
    if os.path.exists(csv_path):
        os.remove(csv_path)
    proc = subprocess.Popen(cmd, stdout=open(os.path.join(work, "renode.log"), "w"),
                            stderr=subprocess.STDOUT, start_new_session=True, env=env)
    t0 = time.time()
    try:
        m = Monitor(args.monitor_port, connect_timeout=180)
        m.cmd('python "execfile(\'%s\')"' % os.path.join(HERE, "hakea_stim_v3.py"))
        syms = elf_symbols(args.elf, set(FIELDS))
        missing = [f for f in FIELDS if f not in syms]
        if missing:
            raise RuntimeError("ELF lacks symbols %s (not a SIM_PROFILE build?)" % missing)
        m.cmd('python "set_syms(%r)"' % syms)
        m.cmd('python "setup_dshot(300, %d)"' % args.frame_us)
        armed = m.cmd('python "print wait_armed(6.0)"', timeout=900)
        if "True" not in armed:
            raise RuntimeError("firmware never armed")
        eb = m.cmd('python "print list(_sb.ReadBytes(%d, 48))"' % syms["eepromBuffer"][0])
        t_run = time.time()
        out = m.cmd('python "run_v3(\'%s\', %d, 0.5, %f, %f, %f, %f, %d, %d, %d, %d)"' % (
            csv_path, args.base, args.hold_s, args.baseline_s, sine_s, args.post_s,
            args.amp, dphase, args.log_us, window_steps), timeout=36000)
        print(out)
        if not os.path.exists(csv_path) or "v3 done" not in out:
            raise RuntimeError("Renode wrote no CSV: %s" % out)
        wall = time.time() - t_run
        meta = {"name": args.name, "target": args.target, "elf": args.elf,
                "elf_sha256": sha(args.elf), "model": args.model,
                "model_sha256": sha(args.model), "model_json": json.load(open(args.model)),
                "eeprom": args.eeprom, "eeprom_sha256": sha(args.eeprom),
                "eeprom_buffer_first48": eb, "freq_requested": args.freq,
                "amp": args.amp, "dphase": dphase, "loop_hz_assumed": LOOP_HZ,
                "sine_s": sine_s, "baseline_s": args.baseline_s, "post_s": args.post_s,
                "base": args.base, "frame_us": args.frame_us, "log_us": args.log_us,
                "window_ms": args.window_ms, "bridge_batch_us": args.batch_us,
                "wall_s": wall, "renode_out": out}
        with open(os.path.join(args.outdir, args.name + ".json"), "w") as f:
            json.dump(meta, f, indent=1)
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
                pass
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    print("%s done in %.0f s wall" % (args.name, time.time() - t0))


if __name__ == "__main__":
    main()
