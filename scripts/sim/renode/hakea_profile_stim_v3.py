# Runs INSIDE Renode's monitor (IronPython 2.7). v3: Fred's PROFILE_MODE
# firmware (Src/hakea_profile.c), run unmodified. Nothing in the firmware is
# written except the two PR #438 stress/status variables at each 50 ms
# stress "take" (same host replica as v2/v3).
#
# run_programme() does, in VIRTUAL time:
#   A. DShot ramp 48 -> base over 0.5 s, then hold base until the programme
#      (hakea_profile.c:step) reaches PROGRAMME_STEPS (15), plus 1 s.
#   B. gate drop: DShot 0 for drop_s (motor stops), ramp back to base,
#      hold restart_hold_s.
#   C. disarm: stop the DShot wire for nosig_s (AM32: >0.5 s signal loss
#      while armed -> NVIC_SystemReset), wire back at 0, wait for armed,
#      ramp to base, hold restart_hold_s.
# Logging:
#   coarse CSV: one row per 1 ms of virtual time, the whole run;
#   fine CSV:   one row per fine_us (25 us) inside windows: each sine step
#               from +0.5 s for 1.0 s, the settle step from +2.0 s for
#               0.5 s (baseline), and the first restart's sine step 1.

from Antmicro.Renode.Time import TimeInterval

_sb = monitor.Machine['sysbus']
_bridge = monitor.Machine['sysbus.bridge']
_thr = monitor.Machine['sysbus.throttle']
_iwdg = monitor.Machine['sysbus.iwdg']
_emu = emulationManager.CurrentEmulation

HSYMS = {}
SIGNED16 = ('hakea_profile.c:modulation',)
FIELDS = ['armed', 'running', 'input', 'zero_crosses', 'duty_cycle', 'last_duty_cycle',
          'adjusted_duty_cycle', 'tim1_arr', 'duty_cycle_maximum',
          'hakea_profile.c:step', 'hakea_profile.c:step_ticks',
          'hakea_profile.c:step_length', 'hakea_profile.c:phase_inc',
          'hakea_profile.c:modulation', 'tenkhzcounter', 'signaltimeout',
          'desync_happened', 'bemf_timeout_happened', 'actual_current',
          'dshot_goodcounts', 'thiszctime', 'stress_window_peak_delta',
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
        if name in SIGNED16 and v >= 0x8000:
            v -= 0x10000
        return v
    return _sb.ReadDoubleWord(addr)


def _wr8(name, v):
    _sb.WriteByte(HSYMS[name][0], v)


def vtime():
    return monitor.Machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds


def setup_dshot(kbaud, frame_us):
    _thr.Protocol = kbaud
    _thr.Bidirectional = False
    _thr.DshotValue = 0
    _thr.DshotFrameUs = frame_us
    _thr.Enabled = True


def wait_armed(max_s=6.0, step_s=0.05):
    t = 0.0
    while t < max_s:
        if _rd('armed') == 1:
            return True
        _thr.DshotValue = 0
        _emu.RunFor(TimeInterval.FromMicroseconds(int(step_s * 1e6)))
        t += step_s
    return _rd('armed') == 1


def _take_stress():
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


class Runner(object):
    def __init__(self, coarse_path, fine_path, fine_us):
        hdr = ['t_s', 'phase', 'dshot', 'rpm_true', 'theta', 'ia', 'ib', 'ic',
               'ibus_true', 'iwdg_kicks'] + FIELDS
        self.fc = open(coarse_path, 'w')
        self.fc.write(','.join(hdr + ['stress_take', 'peak_take', 'status_take']) + '\n')
        self.ff = open(fine_path, 'w')
        self.ff.write(','.join(hdr) + '\n')
        self.fine = TimeInterval.FromMicroseconds(fine_us)
        self.ms = TimeInterval.FromMicroseconds(1000)
        self.next_take = 0.0
        self.phase = 'A'
        self.dshot = 0
        self.fine_until = -1.0
        self.last_step = -1
        self.step_seen_at = 0.0
        self.fine_restart_done = False
        self.fine_start = None
        self.fine_len = 0

    def _row(self):
        r = ['%.6f' % vtime(), self.phase, str(self.dshot), '%.2f' % _bridge.Rpm,
             '%.6f' % _bridge.Theta, '%.3f' % _bridge.CurrentA,
             '%.3f' % _bridge.CurrentB, '%.3f' % _bridge.CurrentC,
             '%.3f' % _bridge.BusCurrent, str(_iwdg.Kicks)]
        for n in FIELDS:
            r.append(str(_rd(n)))
        return r

    def set_dshot(self, v):
        self.dshot = v
        _thr.DshotValue = v

    def _maybe_fine(self):
        """open a fine window shortly after a programme step starts"""
        s = _rd('hakea_profile.c:step')
        t = vtime()
        if s != self.last_step:
            self.last_step = s
            self.step_seen_at = t
            self.fine_start, self.fine_len = None, 0
            if s == 0 and self.phase == 'A':
                self.fine_start, self.fine_len = t + 2.0, 0.5
            elif s in (1, 3, 5, 7, 9, 11, 13) and (self.phase == 'A' or (
                    s == 1 and not self.fine_restart_done)):
                self.fine_start, self.fine_len = t + 0.5, 1.0
                if self.phase != 'A':
                    self.fine_restart_done = True
        if self.fine_start is not None and t >= self.fine_start:
            self.fine_until = self.fine_start + self.fine_len
            self.fine_start = None

    def advance(self, seconds):
        """run `seconds` of virtual time, logging coarse/fine as needed"""
        end = vtime() + seconds
        next_coarse = vtime()
        while vtime() < end - 1e-9:
            self._maybe_fine()
            t = vtime()
            if t < self.fine_until:
                _emu.RunFor(self.fine)
                self.ff.write(','.join(self._row()) + '\n')
            else:
                _emu.RunFor(self.ms)
            t = vtime()
            if t >= next_coarse - 1e-9:
                row = self._row()
                if t >= self.next_take - 1e-9:
                    self.next_take = t + 0.05
                    s, peak, ev = _take_stress()
                    row += [str(s), str(peak), str(ev)]
                else:
                    row += ['', '', '']
                self.fc.write(','.join(row) + '\n')
                next_coarse = t + 0.001

    def ramp(self, base, seconds=0.5):
        n = int(round(seconds / 1e-3))
        for i in range(n):
            self.set_dshot(int(round(48 + (base - 48) * float(i) / n)))
            self.advance(0.001)
        self.set_dshot(base)

    def close(self):
        self.fc.close()
        self.ff.close()


def run_programme(coarse_path, fine_path, base, fine_us, drop_s, nosig_s,
                  restart_hold_s, max_prog_s, frame_us):
    R = Runner(coarse_path, fine_path, fine_us)
    R.phase = 'A'
    R.ramp(base)
    t0 = vtime()
    while _rd('hakea_profile.c:step') != 15 and vtime() - t0 < max_prog_s:
        R.advance(0.1)
    R.advance(1.0)
    # B: gate drop by zero throttle, then back to base
    R.phase = 'B'
    R.set_dshot(0)
    R.advance(drop_s)
    R.ramp(base)
    R.advance(restart_hold_s)
    # C: signal loss -> AM32 disarm (NVIC_SystemReset), then re-arm
    R.phase = 'C'
    R.set_dshot(0)
    _thr.Enabled = False
    R.advance(nosig_s)
    setup_dshot(300, frame_us)
    R.set_dshot(0)
    tw = vtime()
    while _rd('armed') != 1 and vtime() - tw < 8.0:
        R.advance(0.05)
    R.ramp(base)
    R.advance(restart_hold_s)
    R.close()
    print 'programme run done vt=%.3f step=%s armed=%s' % (
        vtime(), _rd('hakea_profile.c:step'), _rd('armed'))


def run_plain(coarse_path, fine_path, base, fine_us, hold_s):
    """stock-ELF comparison: same operating point, no programme"""
    R = Runner(coarse_path, fine_path, fine_us)
    R.phase = 'S'
    R.ramp(base)
    R.advance(2.5)
    R.fine_until = vtime() + 0.5
    R.advance(0.5)  # 0.5 s fine baseline
    R.advance(hold_s)
    R.close()
    print 'plain run done vt=%.3f' % vtime()
