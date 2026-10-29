#!/usr/bin/env python3
"""v3: run Fred's PROFILE_MODE firmware (unmodified ELF) through its whole
programme in Renode, then a gate-drop restart and a full disarm/re-arm.

  run_profile_v3.py [--elf ELF] [--name NAME] [--plain]

--plain runs another ELF (e.g. the stock PR #438 image) at the same
operating point without the programme, for the timing comparison.
DShot300 unidirectional, one frame per 100 us virtual (10 kHz), base 1048,
ESCSim platform FOXEER_F421, bridge batch 2 us (gen_batch_wrapper.py),
Fred's esc1_config.bin, model hakea_calibrated_prop_24v0.json.
Writes <name>_coarse.csv (1 ms rows), <name>_fine.csv (25 us rows in
windows) and <name>.json to notes/sim/renode/v3/raw/.
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
from elfsyms_v3 import elf_symbols_v3  # noqa: E402

HAKEA = os.path.expanduser("~/vehicles/hakea")
FIELDS = ['armed', 'running', 'input', 'zero_crosses', 'duty_cycle', 'last_duty_cycle',
          'adjusted_duty_cycle', 'tim1_arr', 'duty_cycle_maximum',
          'hakea_profile.c:step', 'hakea_profile.c:step_ticks',
          'hakea_profile.c:step_length', 'hakea_profile.c:phase_inc',
          'hakea_profile.c:modulation', 'tenkhzcounter', 'signaltimeout',
          'desync_happened', 'bemf_timeout_happened', 'actual_current',
          'dshot_goodcounts', 'thiszctime', 'stress_window_peak_delta',
          'stress_window_reset_requested', 'pending_status_events', 'eepromBuffer']


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--elf", default=os.path.join(
        HAKEA, "firmware/AM32_FOXEER_F421_PROFILE_profile-v1_bdbc75e-dirty.elf"))
    ap.add_argument("--name", default="profile_v1")
    ap.add_argument("--plain", action="store_true")
    ap.add_argument("--plain-hold-s", type=float, default=2.0)
    ap.add_argument("--base", type=int, default=1048)
    ap.add_argument("--frame-us", type=int, default=100)
    ap.add_argument("--fine-us", type=int, default=25)
    ap.add_argument("--batch-us", type=int, default=2)
    ap.add_argument("--drop-s", type=float, default=1.0)
    ap.add_argument("--nosig-s", type=float, default=0.8)
    ap.add_argument("--restart-hold-s", type=float, default=4.5)
    ap.add_argument("--monitor-port", type=int, default=34571)
    ap.add_argument("--model", default=os.path.join(
        HAKEA, "scripts/sim/models/hakea_calibrated_prop_24v0.json"))
    ap.add_argument("--eeprom", default=os.path.expanduser("~/Downloads/esc1_config.bin"))
    ap.add_argument("--targets-h", default=os.path.join(HAKEA, "am32-renode-pr438/Inc/targets.h"))
    ap.add_argument("--outdir", default=os.path.join(HAKEA, "notes/sim/renode/v3/raw"))
    ap.add_argument("--workdir", default=os.path.join(HERE, "work_v3"))
    a = ap.parse_args()
    a.elf, a.outdir = os.path.abspath(a.elf), os.path.abspath(a.outdir)
    os.makedirs(a.outdir, exist_ok=True)
    work = os.path.join(a.workdir, a.name)
    os.makedirs(work, exist_ok=True)
    env = dict(os.environ, BATCH_US=str(a.batch_us))
    gen = [sys.executable, os.path.join(HERE, "gen_batch_wrapper.py")]
    cmd = gen + ["FOXEER_F421", "--outdir", work, "--targets-file", a.targets_h,
                 "--elf", a.elf, "--eeprom", a.eeprom, "--model", a.model,
                 "--monitor-port", str(a.monitor_port), "--can-bus", "-1", "--run"]
    coarse = os.path.join(a.outdir, a.name + "_coarse.csv")
    fine = os.path.join(a.outdir, a.name + "_fine.csv")
    for p in (coarse, fine):
        if os.path.exists(p):
            os.remove(p)
    proc = subprocess.Popen(cmd, stdout=open(os.path.join(work, "renode.log"), "w"),
                            stderr=subprocess.STDOUT, start_new_session=True, env=env)
    t0 = time.time()
    try:
        m = Monitor(a.monitor_port, connect_timeout=180)
        m.cmd('python "execfile(\'%s\')"' % os.path.join(HERE, "hakea_profile_stim_v3.py"))
        syms = elf_symbols_v3(a.elf, FIELDS)
        print("symbols missing:", [f for f in FIELDS if f not in syms])
        m.cmd('python "set_syms(%r)"' % syms)
        m.cmd('python "setup_dshot(300, %d)"' % a.frame_us)
        armed = m.cmd('python "print wait_armed(6.0)"', timeout=900)
        if "True" not in armed:
            raise RuntimeError("never armed")
        t_arm = float(m.cmd('python "print vtime()"'))
        eb = m.cmd('python "print list(_sb.ReadBytes(%d, 48))"' % syms["eepromBuffer"][0])
        t1 = time.time()
        if a.plain:
            out = m.cmd('python "run_plain(\'%s\', \'%s\', %d, %d, %f)"' % (
                coarse, fine, a.base, a.fine_us, a.plain_hold_s), timeout=36000)
        else:
            out = m.cmd('python "run_programme(\'%s\', \'%s\', %d, %d, %f, %f, %f, 40.0, %d)"' % (
                coarse, fine, a.base, a.fine_us, a.drop_s, a.nosig_s, a.restart_hold_s,
                a.frame_us), timeout=36000)
        print(out)
        if "done" not in out:
            raise RuntimeError("run failed: %s" % out)
        meta = {"name": a.name, "elf": a.elf, "elf_sha256": sha(a.elf), "model": a.model,
                "model_sha256": sha(a.model), "model_json": json.load(open(a.model)),
                "eeprom": a.eeprom, "eeprom_sha256": sha(a.eeprom),
                "eeprom_buffer_first48": eb, "plain": a.plain, "base": a.base,
                "frame_us": a.frame_us, "dshot_rate_hz": 1e6 / a.frame_us,
                "fine_us": a.fine_us, "bridge_batch_us": a.batch_us, "t_armed": t_arm,
                "drop_s": a.drop_s, "nosig_s": a.nosig_s, "restart_hold_s": a.restart_hold_s,
                "wall_s": time.time() - t1, "renode_out": out}
        json.dump(meta, open(os.path.join(a.outdir, a.name + ".json"), "w"), indent=1)
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
    print("%s done in %.0f s wall" % (a.name, time.time() - t0))


if __name__ == "__main__":
    main()
