#!/usr/bin/env python3
"""Run one AM32 SITL experiment deterministically and return its log.

How it works (see ../../notes/sim/am32-ramp-study.md for the why):

  1. A DShot stimulus is computed here in Python as a list of frame
     values at a fixed frame period, and written to a small binary
     "wave" file (format in am32-sim/Mcu/SITL/Src/sitl_input.c).
  2. The SITL (built from ~/vehicles/hakea/am32-sim) is started with
     AM32_SITL_WAVE=<wave file>. The SITL itself injects the frames at
     exact SIMULATED times, so the stimulus does not depend on how fast
     the host is - no UDP, no Python timing loop.
  3. The SITL writes a CSV "study log" (AM32_SITL_STUDY_LOG) at 20 kHz
     of simulated time: rpm, bus voltage/current, phase current rms,
     torque, firmware duty, desync counter, commutation step, rotor
     electrical angle, losses. It exits by itself after the last frame.
  4. It runs free (--speedup 0, as fast as the host allows); the
     firmware/physics scheduler keeps simulated time consistent.

Every run gets its own temporary directory (eeprom copy, wave file,
log), so many runs can go in parallel.

Example:
  from sitl_run import Stim, run
  st = Stim(period_ns=100_000)        # 10 kHz DShot frames
  st.const(0, 1.5)                    # arm: 1.5 s of zero throttle
  st.ramp(48, 1047, 0.5)              # ramp to ~50 %
  st.const(1047, 1.0)                 # settle
  st.sine(1047, 200, 250, 0.5)        # 250 Hz, +-200 DShot units
  log = run(st, model='models/hakea_sitl.json', elf='variants/ramp16.elf')
"""
import json
import os
import shutil
import struct
import subprocess
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
HAKEA = os.path.expanduser('~/vehicles/hakea')
SIM_ROOT = os.path.join(HAKEA, 'am32-sim')
DEFAULT_ELF = os.path.join(SIM_ROOT, 'obj', 'AM32_AM32_SITL_CAN_2.21.elf')
ESC_EEPROM = os.path.expanduser('~/Downloads/esc1_config.bin')

TYPE_DSHOT300 = 2

# AM32 eeprom byte offsets (Inc/eeprom.h) for settings we vary
EE = dict(max_ramp=5, minimum_duty_cycle=6, comp_pwm=20, variable_pwm=21,
          advance_level=23, pwm_frequency=24, startup_power=25, motor_kv=26,
          motor_poles=27, beep_volume=30, input_type=46, auto_advance=47,
          brake_on_stop=28, driving_brake_strength=42)


class Stim(object):
    """DShot frame list at a fixed frame period (simulated time)."""

    def __init__(self, period_ns=100_000, bidir=False, ptype=TYPE_DSHOT300):
        self.period_ns = int(period_ns)
        self.bidir = bidir
        self.ptype = ptype
        self.vals = []
        self.marks = {}  # name -> (t_start, t_end) seconds

    @property
    def t(self):
        return len(self.vals) * self.period_ns * 1e-9

    def _n(self, dur):
        return int(round(dur / (self.period_ns * 1e-9)))

    def mark(self, name, dur_fn):
        t0 = self.t
        dur_fn()
        self.marks[name] = (t0, self.t)

    def const(self, value, dur):
        self.vals += [int(value)] * self._n(dur)

    def ramp(self, v0, v1, dur):
        n = self._n(dur)
        self.vals += list(np.round(np.linspace(v0, v1, n)).astype(int))

    def sine(self, base, amp, freq, dur, phase=0.0):
        n = self._n(dur)
        tt = np.arange(n) * self.period_ns * 1e-9
        v = base + amp * np.sin(2 * np.pi * freq * tt + phase)
        self.vals += list(np.clip(np.round(v), 48, 2047).astype(int))

    def samples(self, t, values):
        """zero-order hold of an arbitrary (t, value) series, t from 0"""
        n = self._n(t[-1])
        tt = np.arange(n) * self.period_ns * 1e-9
        idx = np.clip(np.searchsorted(t, tt, side='right') - 1, 0, len(values) - 1)
        self.vals += list(np.asarray(values)[idx].astype(int))

    def write(self, fn):
        v = np.clip(np.asarray(self.vals, dtype=np.int64), 0, 2047).astype('<u2')
        with open(fn, 'wb') as f:
            f.write(b'HKW1')
            f.write(struct.pack('<IBBHI', self.period_ns, self.ptype, 1 if self.bidir else 0, 0, len(v)))
            f.write(v.tobytes())


def pwm_to_dshot(pwm):
    """ArduPilot ChibiOS RCOutput: value = 2*(pwm-1000), +48 if nonzero"""
    pwm = np.clip(np.asarray(pwm, float), 1000, 2000)
    v = np.minimum(np.round(2 * (pwm - 1000)), 1999)
    return np.where(v > 0, v + 48, 0).astype(int)


def make_eeprom(fn, overrides=None):
    """Fred's real ESC settings (esc1_config.bin, 184 bytes) padded to the
    192 byte AM32 eeprom block, with optional {name or offset: value}"""
    data = bytearray(open(ESC_EEPROM, 'rb').read())
    data += b'\xff' * (192 - len(data))
    data[30] = 0  # beep volume 0: startup tones only cost sim time
    for k, v in (overrides or {}).items():
        off = EE[k] if isinstance(k, str) else int(k)
        data[off] = int(v) & 0xFF
    open(fn, 'wb').write(bytes(data))


def write_model(fn, model):
    with open(fn, 'w') as f:
        json.dump(model, f, indent=1)


def load_log(fn):
    rows = []
    end = None
    with open(fn) as f:
        header = f.readline().strip().split(',')
        for line in f:
            if line.startswith('#'):
                end = line.strip()
                continue
            rows.append(line)
    if not rows:
        arr = np.zeros((0, len(header)))
    else:
        arr = np.loadtxt(rows, delimiter=',', ndmin=2)
    out = {h: arr[:, i] for i, h in enumerate(header)}
    out['_end'] = end
    return out


def run(stim, model, elf=DEFAULT_ELF, eeprom_overrides=None, keep_dir=None,
        timeout=3600, dt_ns=50000, verbose=False, extra_env=None):
    """Run one experiment. `model` is a dict (ESCSim model JSON) or a
    path. Returns the study log as a dict of numpy arrays."""
    d = tempfile.mkdtemp(prefix='hakea_sitl_')
    try:
        wave = os.path.join(d, 'wave.bin')
        stim.write(wave)
        ee = os.path.join(d, 'eeprom.bin')
        make_eeprom(ee, eeprom_overrides)
        if isinstance(model, dict):
            mpath = os.path.join(d, 'model.json')
            write_model(mpath, model)
        else:
            mpath = os.path.abspath(model)
        log = os.path.join(d, 'study.csv')
        env = dict(os.environ, AM32_SITL_WAVE=wave, AM32_SITL_STUDY_LOG=log,
                   AM32_SITL_STUDY_DT_NS=str(int(dt_ns)))
        env.update(extra_env or {})
        cmd = [os.path.abspath(elf), '--config', mpath, '--eeprom', ee, '--can-uri', 'none',
               '--input-port', '0', '--state-port', '0', '--speedup', '0']
        with open(os.path.join(d, 'stderr.txt'), 'w') as errf:
            p = subprocess.Popen(cmd, cwd=d, env=env, stdout=errf, stderr=subprocess.STDOUT)
            try:
                p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
                raise
        if verbose:
            print(open(os.path.join(d, 'stderr.txt')).read()[-2000:])
        out = load_log(log)
        out['_stderr'] = open(os.path.join(d, 'stderr.txt')).read()[-4000:]
        out['_marks'] = dict(stim.marks)
        return out
    finally:
        if keep_dir:
            os.makedirs(keep_dir, exist_ok=True)
            for fn in os.listdir(d):
                shutil.copy(os.path.join(d, fn), keep_dir)
        shutil.rmtree(d, ignore_errors=True)


def model_from_params(p, battery_v=None):
    """ESCSim SITL model dict from calibrated parameters. Keys:
    kv, R_phase, L_phase, J, kq, T0, b, Vbat, Rb, rds_on"""
    return {
        'motor': {
            'kv': p['kv'], 'poles': 14,
            'resistance': p['R_phase'], 'inductance': p['L_phase'],
            'mutual_inductance': p.get('M_phase', 0.0),
            'inertia': p['J'], 'damping': p.get('b', 0.0),
            'static_friction': p.get('T0', 0.0), 'load_k_omega2': p.get('kq', 0.0),
        },
        'battery': {'voltage': battery_v or p['Vbat'], 'resistance': p['Rb'],
                    'capacitance': p.get('C_bus', 0.0)},
        'esc': {'rds_on': p.get('rds_on', 0.002), 'diode_vf': 0.7,
                'commutation_transfer': p.get('commutation_transfer', 1.0),
                'temperature_c': 38},
    }


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='smoke test: arm, ramp to 50 %, hold')
    ap.add_argument('--model', default=os.path.join(HERE, 'models', 'hakea_2307_5in.json'))
    ap.add_argument('--elf', default=DEFAULT_ELF)
    ap.add_argument('--keep', default=None)
    a = ap.parse_args()
    st = Stim(period_ns=100_000)
    st.const(0, 1.5)
    st.ramp(48, 1047, 0.5)
    st.const(1047, 1.0)
    import time
    t0 = time.time()
    lg = run(st, a.model, elf=a.elf, keep_dir=a.keep, verbose=True)
    print('wall %.1fs, sim %.2fs, %d rows, end %s' % (time.time() - t0, lg['t'][-1] if len(lg['t']) else 0, len(lg['t']), lg['_end']))
    m = lg['t'] > lg['t'][-1] - 0.5
    for k in ('rpm', 'vbus', 'ibus', 'irms', 'duty', 'desync', 'arr'):
        print('%-6s mean %.2f min %.2f max %.2f' % (k, lg[k][m].mean(), lg[k][m].min(), lg[k][m].max()))
