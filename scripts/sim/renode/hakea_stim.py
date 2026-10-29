# Runs INSIDE Renode's monitor (IronPython 2.7 - print is a statement,
# no f-strings). Loaded by run_crosscheck.py with
#     python "execfile('.../hakea_stim.py')"
#
# What it does: steps the emulated ESC through virtual time in fixed
# steps (default 100 us). Before each step it sets the DShot throttle
# value the throttle generator will put on the wire; after each step it
# samples firmware globals (read straight out of emulated SRAM) and the
# motor physics "truth" from the bridge peripheral, and appends a CSV row.
#
# Everything is in VIRTUAL time: Renode only advances the emulated clock
# inside RunFor(), so the 250 Hz sine is 250 Hz as the firmware sees it,
# however slowly the host runs.

from Antmicro.Renode.Time import TimeInterval

_sb = monitor.Machine['sysbus']
_bridge = monitor.Machine['sysbus.bridge']
_thr = monitor.Machine['sysbus.throttle']
_emu = emulationManager.CurrentEmulation

# name -> (address, size in bytes); filled by set_syms() from the host,
# which reads them out of the ELF that is actually loaded
HSYMS = {}

# firmware globals logged every step, in CSV column order. Any the ELF
# lacks are written as empty cells.
HFIELDS = ['armed', 'running', 'input', 'adjusted_input', 'duty_cycle',
           'adjusted_duty_cycle', 'last_duty_cycle', 'max_ramp_high_rpm',
           'e_com_time', 'commutation_interval', 'average_interval',
           'zero_crosses', 'desync_happened', 'bemf_timeout_happened',
           'actual_current', 'battery_voltage', 'dshot_goodcounts',
           'dshot_badcounts']


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


def vtime():
    return monitor.Machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds


def setup_dshot(kbaud, frame_us):
    """DShot on the emulated wire, zero throttle, one frame per frame_us."""
    _thr.Protocol = kbaud
    _thr.DshotValue = 0
    _thr.DshotFrameUs = frame_us
    _thr.Enabled = True


def run_idle(seconds, value=0):
    """hold one throttle value for a while, unlogged (boot, arming)"""
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


def run_profile(values_path, step_us, out_path):
    """Play a throttle profile (one DShot value per line, one line per
    step_us of virtual time) and log one CSV row per step."""
    f = open(values_path)
    values = [int(l) for l in f if l.strip()]
    f.close()
    out = open(out_path, 'w')
    out.write(','.join(['t_s', 'cmd', 'rpm_true', 'ia', 'ib', 'ic'] + HFIELDS) + '\n')
    step = TimeInterval.FromMicroseconds(step_us)
    for v in values:
        _thr.DshotValue = v
        _emu.RunFor(step)
        row = ['%.6f' % vtime(), str(v), '%.2f' % _bridge.Rpm,
               '%.3f' % _bridge.CurrentA, '%.3f' % _bridge.CurrentB,
               '%.3f' % _bridge.CurrentC]
        for name in HFIELDS:
            row.append(str(_rd(name)))
        out.write(','.join(row) + '\n')
    out.close()
    print 'profile done: %d steps, vt=%.4f' % (len(values), vtime())
