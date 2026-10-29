# Runs INSIDE Renode's monitor (IronPython 2.7). v2 of hakea_stim.py.
#
# Same idea as v1: step virtual time in fixed steps, set the DShot value
# before each step, sample firmware RAM + motor truth after it.
#
# New in v2 - PR #438 (EDTv2) commutation stress / status:
#   The firmware collects a running peak of |zero-cross interval change|
#   (stress_window_peak_delta, in Src/dshot.c) on every zero cross, and ORs
#   status events (pending_status_events: ALERT=bemf timeout, WARNING=desync,
#   ERROR=stuck rotor) as they happen. Turning those into a stress number and
#   clearing them is done by dshot_take_commutation_stress() /
#   dshot_take_status_events(), which only run inside make_dshot_package(),
#   i.e. only when the ESC sends bidirectional-DShot telemetry. On a
#   unidirectional wire they never run, so here the host does exactly what
#   dshot_take_commutation_stress() does, every `window_steps` steps:
#       if !running or reset_requested: stress = 0
#       else: reset_requested = 1; stress = min(255, peak*255/thiszctime)
#   and then reads + clears pending_status_events. Only those PR #438
#   variables are written; nothing else in the firmware reads them.
#   max_commutation_stress (sent in status frames as >>4) is tracked here
#   as max over windows (the firmware only raises it while armed).

from Antmicro.Renode.Time import TimeInterval

_sb = monitor.Machine['sysbus']
_bridge = monitor.Machine['sysbus.bridge']
_thr = monitor.Machine['sysbus.throttle']
_emu = emulationManager.CurrentEmulation

HSYMS = {}
HFIELDS = ['armed', 'running', 'input', 'last_duty_cycle',
           'max_ramp_high_rpm', 'e_com_time', 'commutation_interval',
           'average_interval', 'zero_crosses', 'desync_happened',
           'bemf_timeout_happened', 'actual_current', 'battery_voltage',
           'dshot_goodcounts', 'dshot_badcounts', 'thiszctime',
           'stress_window_peak_delta', 'stress_window_reset_requested',
           'pending_status_events', 'max_commutation_stress',
           'dshot_telemetry', 'dshot_extended_telemetry']


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
        return _sb.ReadWord(addr)
    return _sb.ReadDoubleWord(addr)


def _wr8(name, v):
    _sb.WriteByte(HSYMS[name][0], v)


def vtime():
    return monitor.Machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds


def setup_dshot(kbaud, frame_us, bidir=False):
    _thr.Protocol = kbaud
    _thr.Bidirectional = bidir
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


def send_command(cmd, repeats=10, seconds=0.05):
    """hold a DShot command value (1..47) on the wire for a while, then 0"""
    run_idle(seconds, cmd)
    run_idle(0.02, 0)


def _take_stress():
    """host copy of dshot_take_commutation_stress() + status take"""
    s = 0
    peak = ''
    if _rd('running') and not _rd('stress_window_reset_requested'):
        _wr8('stress_window_reset_requested', 1)
        peak = _rd('stress_window_peak_delta')
        cur = _rd('thiszctime')
        if cur:
            s = min(255, peak * 255 // cur)
    ev = _rd('pending_status_events')
    _wr8('pending_status_events', 0)
    return s, peak, ev


def run_profile(values_path, step_us, out_path, window_steps=0):
    """window_steps > 0: host-side stress take every window_steps steps
    (unidirectional runs). 0: leave the firmware's own EDT path alone."""
    f = open(values_path)
    values = [int(l) for l in f if l.strip()]
    f.close()
    out = open(out_path, 'w')
    out.write(','.join(['t_s', 'cmd', 'rpm_true', 'ia', 'ib', 'ic'] + HFIELDS
                       + ['stress_take', 'peak_take', 'status_take']) + '\n')
    step = TimeInterval.FromMicroseconds(step_us)
    n = 0
    if window_steps:
        _take_stress()  # start the first window clean
    for v in values:
        _thr.DshotValue = v
        _emu.RunFor(step)
        n += 1
        row = ['%.6f' % vtime(), str(v), '%.2f' % _bridge.Rpm,
               '%.3f' % _bridge.CurrentA, '%.3f' % _bridge.CurrentB,
               '%.3f' % _bridge.CurrentC]
        for name in HFIELDS:
            row.append(str(_rd(name)))
        if window_steps and n % window_steps == 0:
            s, peak, ev = _take_stress()
            row += [str(s), str(peak), str(ev)]
        else:
            row += ['', '', '']
        out.write(','.join(row) + '\n')
    out.close()
    print 'profile done: %d steps, vt=%.4f' % (len(values), vtime())
