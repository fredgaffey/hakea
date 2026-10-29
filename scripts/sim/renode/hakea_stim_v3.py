# Runs INSIDE Renode's monitor (IronPython 2.7). v3: in-ESC throttle sine.
#
# The firmware is built with -DSIM_PROFILE (scripts/sim/firmware-profile-
# study.patch): every control tick it adds sim_profile_amp*sin(phase) to the
# duty setpoint before the ramp limiter. This script starts and stops that
# sine by writing the firmware globals through the bus, at chosen VIRTUAL
# times, and logs at a fine step (25 us) while it runs.
#
# Sequence (run_v3):
#   ramp DShot 48->base over ramp_s, 100 us steps, not logged
#   hold base, 100 us steps, not logged, then baseline_s logged at log_us
#   write sim_profile_phase=0, sim_profile_dphase=dphase, sim_profile_amp=amp
#   sine for sine_s, logged at log_us
#   write sim_profile_amp=0, post_s logged at log_us
# DShot stays at `base` throughout (frames every frame_us).
#
# Logged per row: virtual time, physics truth (rpm, theta, phase currents,
# bus current/voltage), firmware RAM (sim_profile_out, duty values, ...),
# plus a host replica of PR #438's dshot_take_commutation_stress() every
# window_steps rows (as in v2).

from Antmicro.Renode.Time import TimeInterval

_sb = monitor.Machine['sysbus']
_bridge = monitor.Machine['sysbus.bridge']
_thr = monitor.Machine['sysbus.throttle']
_emu = emulationManager.CurrentEmulation

HSYMS = {}
HFIELDS = ['armed', 'running', 'input', 'duty_cycle_setpoint', 'last_duty_cycle',
           'sim_profile_out', 'sim_profile_amp', 'sim_profile_phase',
           'max_ramp_high_rpm', 'tim1_arr', 'commutation_interval',
           'zero_crosses', 'desync_happened', 'bemf_timeout_happened',
           'actual_current', 'dshot_goodcounts', 'dshot_badcounts',
           'thiszctime', 'stress_window_peak_delta',
           'stress_window_reset_requested', 'pending_status_events']


def set_syms(d):
    HSYMS.clear()
    HSYMS.update(d)


def _rd(name):
    if name not in HSYMS:
        return ''
    addr, size = HSYMS[name]
    if size == 1:
        return _sb.ReadByte(addr)
    if size == 2:
        v = _sb.ReadWord(addr)
        if name == 'sim_profile_out' and v >= 0x8000:  # int16
            v -= 0x10000
        return v
    return _sb.ReadDoubleWord(addr)


def _wr(name, v):
    addr, size = HSYMS[name]
    if size == 1:
        _sb.WriteByte(addr, v & 0xFF)
    elif size == 2:
        _sb.WriteWord(addr, v & 0xFFFF)
    else:
        _sb.WriteDoubleWord(addr, v & 0xFFFFFFFF)


def vtime():
    return monitor.Machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds


def setup_dshot(kbaud, frame_us):
    _thr.Protocol = kbaud
    _thr.Bidirectional = False
    _thr.DshotValue = 0
    _thr.DshotFrameUs = frame_us
    _thr.Enabled = True


def run_idle(seconds, value=0):
    _thr.DshotValue = value
    _emu.RunFor(TimeInterval.FromMicroseconds(int(seconds * 1e6)))


def wait_armed(max_s=5.0, step_s=0.05):
    t = 0.0
    while t < max_s:
        if _rd('armed') == 1:
            return True
        run_idle(step_s, 0)
        t += step_s
    return _rd('armed') == 1


def _take_stress():
    s = 0
    peak = ''
    if _rd('running') and not _rd('stress_window_reset_requested'):
        _wr('stress_window_reset_requested', 1)
        peak = _rd('stress_window_peak_delta')
        cur = _rd('thiszctime')
        if cur:
            s = min(255, peak * 255 // cur)
    ev = _rd('pending_status_events')
    _wr('pending_status_events', 0)
    return s, peak, ev


class _Log(object):
    def __init__(self, path, log_us, window_steps):
        self.f = open(path, 'w')
        self.f.write(','.join(['t_s', 'seg', 'rpm_true', 'theta', 'ia', 'ib', 'ic',
                               'ibus_true', 'vbus_true'] + HFIELDS
                              + ['stress_take', 'peak_take', 'status_take']) + '\n')
        self.step = TimeInterval.FromMicroseconds(log_us)
        self.window_steps = window_steps
        self.n = 0

    def run(self, seg, nsteps):
        for i in range(nsteps):
            _emu.RunFor(self.step)
            self.n += 1
            row = ['%.6f' % vtime(), seg, '%.2f' % _bridge.Rpm, '%.6f' % _bridge.Theta,
                   '%.3f' % _bridge.CurrentA, '%.3f' % _bridge.CurrentB,
                   '%.3f' % _bridge.CurrentC, '%.3f' % _bridge.BusCurrent,
                   '%.3f' % _bridge.BusVoltage]
            for name in HFIELDS:
                row.append(str(_rd(name)))
            if self.window_steps and self.n % self.window_steps == 0:
                s, peak, ev = _take_stress()
                row += [str(s), str(peak), str(ev)]
            else:
                row += ['', '', '']
            self.f.write(','.join(row) + '\n')

    def close(self):
        self.f.close()


def run_v3(out_path, base, ramp_s, hold_s, baseline_s, sine_s, post_s,
           amp, dphase, log_us, window_steps):
    step = TimeInterval.FromMicroseconds(100)
    n = int(round(ramp_s / 100e-6))
    for i in range(n):
        _thr.DshotValue = int(round(48 + (base - 48) * float(i) / n))
        _emu.RunFor(step)
    _thr.DshotValue = base
    _emu.RunFor(TimeInterval.FromMicroseconds(int(hold_s * 1e6)))
    lg = _Log(out_path, log_us, window_steps)
    _take_stress()  # first window starts clean
    lg.run('base', int(round(baseline_s / (log_us * 1e-6))))
    t_start = vtime()
    _wr('sim_profile_phase', 0)
    _wr('sim_profile_dphase', dphase)
    _wr('sim_profile_amp', amp)
    lg.run('sine', int(round(sine_s / (log_us * 1e-6))))
    _wr('sim_profile_amp', 0)
    t_stop = vtime()
    lg.run('post', int(round(post_s / (log_us * 1e-6))))
    lg.close()
    _thr.DshotValue = 0
    print 'v3 done: sine %.6f..%.6f vt=%.4f rows=%d' % (t_start, t_stop, vtime(), lg.n)
