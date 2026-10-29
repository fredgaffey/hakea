#!/usr/bin/env python3
"""Commutation-averaged model of an AM32 ESC + BLDC motor + prop.

Helper module for motorid.py. Pure numpy; numba is used when it is
installed (about 50x faster) and the code falls back to plain Python
otherwise (then a coarser integration step is used by default so a fit
still finishes in reasonable time).

Signal chain (SI units), adapted from Hakea's scripts/sim/avgmodel.py:

  ArduPilot pwm (us) -> DShot value = 2000*(pwm-PWM_MIN)/(PWM_MAX-PWM_MIN) + 48
                                      (AP_HAL RCOutput, 48..2047)
  DShot -> duty setpoint = map(dshot, 47..2047 -> min_duty..2000) (AM32 main.c)
  setpoint -> startup clamp, low-rpm duty clamp, ramp limiter
              (all per 20 kHz AM32 control tick)
  duty d (0..1) -> line voltage g(rpm)*d*Vbus  (g = effective voltage gain:
                   dead time, timing advance, commutation losses; a small
                   table vs rpm)
  L dI/dt = g d Vbus - Ke w - R I     (R = 2 R_phase + 2 Rds_on: line-to-line
                                       DC-equivalent of the 6-step drive)
  J dw/dt = Kt I - kq w|w| - b w - T0 (prop torque ~ w^2, bearing/windage)
  Ibus = g d I ;  Vbus = Voc(t) - Rb Ibus,  Voc(t) = V0 + V1 t (per run)
  Ke = Kt = 60 / (2 pi Kv), Kv fixed at the nominal value; g absorbs the
  difference between nominal and effective speed constant.

Zero throttle: with comp_pwm = 1 AM32 keeps commutating and ramps the
duty to 0, which shorts the windings through the low-side FETs: strong
dynamic braking (the averaged model lets the current go negative). With
comp_pwm = 0 (or brake_on_zero_throttle = 1) the bridge is not
synchronous and the current cannot reverse (free coast). Below
`rpm_stop` AM32 loses back-EMF tracking: the bridge turns off and the
telemetry reports 0 rpm.

Optional lumped winding thermal model: copper loss heats a single
thermal mass (Rth K/W to ambient, Cth J/K); the winding share of R rises
0.393 %/K with copper temperature. Cth = 0 disables it.

Scale symmetry (important, see scale_params): with only rpm and battery
voltage measured, every rpm prediction is unchanged by
  J, kq, b, T0, Cth, Isat -> s * (...)    R, Rb, L, Rth -> (...) / s
(the motor current scales by s). One absolute quantity (R, current or
torque) must be known to fix s.
"""
import math

import numpy as np

try:  # numba is optional
    from numba import njit
    HAVE_NUMBA = True
except ImportError:  # pragma: no cover - depends on the environment
    HAVE_NUMBA = False

    def njit(*args, **kwargs):
        """no-op stand-in for numba.njit"""
        if len(args) == 1 and callable(args[0]):
            return args[0]
        return lambda f: f

TICK = 50e-6        # AM32 control loop tick (20 kHz)
ALPHA_CU = 0.00393  # copper resistance temperature coefficient (1/K)
RW_FRAC = 0.95      # share of R that is copper (rest: FETs, wiring)
T_AMB = 25.0        # ambient / reference temperature (C)

# default speed knots (rpm) of the effective voltage gain table g(rpm)
G_KNOTS = (0.0, 8000.0, 14000.0, 20000.0, 30000.0)

# ---------------------------------------------------------------- ESC


class Am32Settings(object):
    """The AM32 settings that change how throttle becomes duty cycle.

    Built from an AM32 EEPROM image (from_eeprom) or from defaults; any
    attribute can be overridden afterwards. Names follow AM32 main.c.
    """

    def __init__(self):
        # eeprom-derived
        self.minimum_duty_cycle = 10     # duty units (0..2000) at DShot 48
        self.startup_power = 100         # eeprom startup_power (50..150)
        self.max_ramp = 160              # eeprom max_ramp (0.1 %/ms units)
        self.comp_pwm = 1                # complementary (synchronous) PWM
        self.brake_on_zero_throttle = 0  # 1 = coast, 2 = brake on zero throttle
        self.stall_protection = 0
        self.motor_kv = 1300             # eeprom motor_kv*40+20
        self.motor_poles = 14
        self.driving_brake_strength = 10
        # target (firmware build) constants
        self.ramp_speed_startup = 2      # RAMP_SPEED_STARTUP (targets.h default)
        self.ramp_speed_low_rpm = 6      # RAMP_SPEED_LOW_RPM
        self.ramp_speed_high_rpm = 16    # RAMP_SPEED_HIGH_RPM
        self.dead_time = 35              # DEAD_TIME (FOXEER_F421)
        self.throttle_max_at_low_rpm = 400
        self.low_rpm_throttle_limit = 1
        # behaviour seen on the bench, not a setting
        self.rpm_stop = 2000.0           # below this (mech rpm, throttle 0) AM32 stops tracking
        self.source = 'AM32 defaults'

    @classmethod
    def from_eeprom(cls, data):
        """Parse an AM32 EEPROM image (layout: am32 Inc/eeprom.h, >= 48 bytes)."""
        s = cls()
        b = bytearray(data)
        if len(b) < 48:
            raise ValueError('AM32 eeprom image too short (%d bytes)' % len(b))
        s.max_ramp = b[5]
        s.minimum_duty_cycle = b[6] * 10 if 0 < b[6] < 51 else 0
        s.brake_on_zero_throttle = b[13] if b[13] != 0xFF else 0
        s.comp_pwm = b[20]
        s.startup_power = b[25]
        s.motor_kv = b[26] * 40 + 20
        s.motor_poles = b[27]
        s.stall_protection = b[29] if b[29] != 0xFF else 0
        s.driving_brake_strength = b[42]
        s.source = 'eeprom'
        s.eeprom_version = b[1]
        s.firmware_version = '%d.%d' % (b[3], b[4])
        return s

    def derived(self):
        """The values main.c computes at boot from the settings."""
        min_duty = self.minimum_duty_cycle
        tmax_low = self.throttle_max_at_low_rpm
        if self.driving_brake_strength < 10:  # main.c dead_time_override
            dto = min(self.dead_time + (150 - self.driving_brake_strength * 10), 200)
            min_duty += dto
            tmax_low += dto
        min_startup = min_duty + (self.startup_power if 49 < self.startup_power < 151 else 0)
        startup_max = min_duty + 400
        r_st, r_lo, r_hi = self.ramp_speed_startup, self.ramp_speed_low_rpm, self.ramp_speed_high_rpm
        if self.max_ramp < 10:  # main.c: small values are per-tick limits with a divider (not modelled)
            r_st = r_lo = r_hi = max(self.max_ramp, 1)
        else:
            r_st = min(r_st, self.max_ramp // 10)
            r_lo = min(r_lo, self.max_ramp // 10)
            r_hi = min(r_hi, self.max_ramp // 10)
        div = 32 // self.motor_poles if self.motor_poles else 0
        low_lvl = self.motor_kv // 100 // div if div else 0
        high_lvl = self.motor_kv // 12 // div if div else 0
        return dict(min_duty=min_duty, min_startup=min_startup, startup_max=startup_max,
                    ramp_startup=r_st, ramp_low=r_lo, ramp_high=r_hi,
                    low_rpm_level=low_lvl, high_rpm_level=high_lvl,
                    throttle_max_at_low_rpm=tmax_low,
                    low_rpm_limit=int(self.low_rpm_throttle_limit and self.motor_kv > 20),
                    drive_sync=int(self.comp_pwm != 0),
                    zero_sync=int(self.comp_pwm != 0 and self.brake_on_zero_throttle != 1),
                    startup_zc=30 >> self.stall_protection,
                    rpm_stop=self.rpm_stop, poles=self.motor_poles)

    def as_array(self):
        d = self.derived()
        keys = ['min_duty', 'ramp_startup', 'ramp_low', 'ramp_high', 'low_rpm_level', 'high_rpm_level',
                'throttle_max_at_low_rpm', 'low_rpm_limit', 'drive_sync', 'zero_sync', 'rpm_stop',
                'poles', 'min_startup', 'startup_max', 'startup_zc']
        return np.array([float(d[k]) for k in keys])

    def to_dict(self):
        out = {k: v for k, v in self.__dict__.items()}
        out['derived'] = self.derived()
        return out


def pwm_to_dshot(pwm, pwm_min=1000.0, pwm_max=2000.0):
    """ArduPilot output pwm (us) -> DShot throttle value (0 = stop, 48..2047)."""
    pwm = np.asarray(pwm, float)
    frac = np.clip((pwm - pwm_min) / (pwm_max - pwm_min), 0.0, 1.0)
    v = np.minimum(np.floor(frac * 2000.0), 1999.0)
    return np.where(v > 0, v + 48, 0).astype(np.int64)


def static_duty(dshot, esc):
    """Duty (0..1) AM32 settles at for a DShot value (no rpm limits)."""
    d = esc.derived()
    ds = np.clip(np.asarray(dshot, float), 47, 2047)
    sp = d['min_duty'] + np.floor((ds - 47) * (2000 - d['min_duty']) / 2000.0)
    return np.where(np.asarray(dshot) >= 48, sp / 2000.0, 0.0)

# -------------------------------------------------------------- kernel


@njit(cache=True)
def _amap(x, in_min, in_max, out_min, out_max):
    # AM32 map(): integer arithmetic, clamps to the input range
    if x < in_min:
        x = in_min
    if x > in_max:
        x = in_max
    if in_max == in_min:
        return out_min
    return out_min + (x - in_min) * (out_max - out_min) // (in_max - in_min)


@njit(cache=True)
def _interp(x, xk, yk):
    # linear interpolation, flat outside the knots (like np.interp)
    n = xk.shape[0]
    if x <= xk[0]:
        return yk[0]
    if x >= xk[n - 1]:
        return yk[n - 1]
    j = 1
    while xk[j] < x:
        j += 1
    f = (x - xk[j - 1]) / (xk[j] - xk[j - 1])
    return yk[j - 1] + f * (yk[j] - yk[j - 1])


@njit(cache=True)
def _kernel(ds, p, esc, gk, gv, w0, every, tps):
    """Integrate the averaged model.

    ds    DShot value per integration step (already delayed), int64
    p     R, J, kq, b, T0, L, Rb, V0, V1, Isat, Rth, Cth, Tw0, kv
    esc   Am32Settings.as_array()
    every output decimation (in steps); tps = AM32 ticks per step
    """
    R25 = p[0]; J = p[1]; kq = p[2]; b = p[3]; T0 = p[4]; L = p[5]; Rb = p[6]
    V0 = p[7]; V1 = p[8]; Isat = p[9]; Rth = p[10]; Cth = p[11]; Tw = p[12]; kv = p[13]
    min_duty = int(esc[0]); r_st = esc[1]; r_lo = esc[2]; r_hi = esc[3]
    lo_lvl = int(esc[4]); hi_lvl = int(esc[5]); tmax_lo = int(esc[6]); lim = esc[7] > 0
    drive_sync = esc[8] > 0; zero_sync = esc[9] > 0; rpm_stop = esc[10]; pp = int(esc[11]) // 2
    min_start = esc[12]; start_max = esc[13]; start_zc = esc[14]
    if Tw <= 0:
        Tw = T_AMB
    R = R25 * (1 - RW_FRAC + RW_FRAC * (1 + ALPHA_CU * (Tw - T_AMB)))
    ke = 60.0 / (2 * math.pi * kv)
    h = TICK * tps
    n = ds.shape[0]
    nout = n // every
    o_w = np.zeros(nout); o_i = np.zeros(nout); o_d = np.zeros(nout); o_v = np.zeros(nout)
    o_ib = np.zeros(nout); o_tw = np.zeros(nout); o_run = np.zeros(nout); o_tq = np.zeros(nout)
    w = w0
    cur = 0.0
    duty = 0.0
    zc = 0.0
    if w0 > 100:
        zc = 1e9
    w_stop = rpm_stop * 2 * math.pi / 60
    k = 0
    acc_w = 0.0; acc_i = 0.0; acc_d = 0.0; acc_v = 0.0; acc_ib = 0.0; acc_tq = 0.0
    for i in range(n):
        t = i * h
        d_in = ds[i]
        # --- AM32 throttle -> duty setpoint
        if d_in >= 48:
            sp = float(_amap(d_in, 47, 2047, min_duty, 2000))
        else:
            sp = 0.0
        rpm = w * 60 / (2 * math.pi)
        if lim:  # low rpm power protection, k_erpm = electrical krpm
            dmax = float(_amap(int(rpm * pp / 1000.0), lo_lvl, hi_lvl, tmax_lo, 2000))
            if sp > dmax:
                sp = dmax
        if d_in >= 48 and zc < start_zc:  # startup clamp
            if sp < min_start:
                sp = min_start
            if sp > start_max:
                sp = start_max
        # --- ramp limiter (duty units per tick)
        if zc < 150 or duty < 150:
            mx = r_st
        else:
            erev_us = 1e6 / max(rpm / 60.0 * pp, 1e-3)
            mx = r_lo if erev_us / 3 > 500 else r_hi
        mx = mx * tps
        if sp - duty > mx:
            duty = duty + mx
        elif duty - sp > mx:
            duty = duty - mx
        else:
            duty = sp
        running = w > w_stop or d_in >= 48
        if not running:
            duty = 0.0
        d = duty / 2000.0
        g = _interp(rpm, gk, gv)
        if w > 50:  # zero crossings: 6 per electrical revolution
            zc += rpm / 60.0 * pp * 6 * h
        else:
            zc = 0.0
        voc = V0 + V1 * t
        # --- electrical: exact exponential step of
        #     L dI/dt = a Voc - ke w - (R + Rb a^2) I,  a = g d
        if running:
            a = g * d
            rt = R + Rb * a * a
            iss = (a * voc - ke * w) / rt
            cur = iss + (cur - iss) * math.exp(-rt * h / L)
            sync = drive_sync if d_in >= 48 else zero_sync
            if (not sync) and cur < 0:
                cur = 0.0  # bridge not synchronous: no reverse current
            ibus = a * cur
        else:
            cur = 0.0
            ibus = 0.0
        # --- mechanical
        kt = ke if Isat <= 0 else ke / (1.0 + abs(cur) / Isat)
        tq_em = kt * cur
        tl = kq * w * abs(w) + b * w
        tf = b * w
        if w > 0.5:
            tl += T0
            tf += T0
        w += (tq_em - tl) / J * h
        if w < 0:
            w = 0.0
        # --- winding temperature
        if Cth > 0:
            pcu = cur * cur * R * RW_FRAC
            Tw += (pcu - (Tw - T_AMB) / Rth) / Cth * h
            R = R25 * (1 - RW_FRAC + RW_FRAC * (1 + ALPHA_CU * (Tw - T_AMB)))
        # stator reaction torque a stand would measure: EM torque minus
        # the bearing friction the stator itself takes back
        acc_w += w; acc_i += cur; acc_d += d; acc_v += voc - Rb * ibus; acc_ib += ibus
        acc_tq += tq_em - tf
        if (i + 1) % every == 0:
            o_w[k] = w  # speed is sampled, the rest averaged over the output step
            o_i[k] = acc_i / every; o_d[k] = acc_d / every; o_v[k] = acc_v / every
            o_ib[k] = acc_ib / every; o_tq[k] = acc_tq / every
            o_tw[k] = Tw
            o_run[k] = 1.0 if running else 0.0
            acc_w = 0.0; acc_i = 0.0; acc_d = 0.0; acc_v = 0.0; acc_ib = 0.0; acc_tq = 0.0
            k += 1
    return o_w, o_i, o_d, o_v, o_ib, o_tw, o_run, o_tq


PARAM_ORDER = ['R', 'J', 'kq', 'b', 'T0', 'L', 'Rb', 'V0', 'V1', 'Isat', 'Rth', 'Cth', 'Tw0', 'kv']
DEFAULT_STEP_US = 50.0 if HAVE_NUMBA else 1000.0


def simulate(q, esc, t_cmd, pwm_cmd, t0, t1, w0=0.0, pwm_min=1000.0, pwm_max=2000.0,
             out_dt=0.0005, step_us=None, g_knots=G_KNOTS):
    """Simulate one run from t0 to t1 (s) driven by the commanded pwm
    (zero-order hold of the samples t_cmd, pwm_cmd).

    q: parameter dict (PARAM_ORDER keys, g0..gN, delay). V1 is volts per
       second counted from t0.
    Returns a dict of arrays on a uniform grid (out_dt): t, w (rad/s),
    rpm, rpm_telem (0 when AM32 has stopped tracking), i (motor current),
    duty, vbus, ibus, tw (winding C), running, torque (stator reaction).
    """
    step_us = step_us or DEFAULT_STEP_US
    tps = max(1, int(round(step_us * 1e-6 / TICK)))
    h = TICK * tps
    p = np.array([float(q.get(k, 0.0)) for k in PARAM_ORDER])
    if p[13] <= 0:
        p[13] = esc.motor_kv
    if p[5] <= 0:
        p[5] = 30e-6
    gv = np.array([float(q.get('g%d' % k, q.get('g', 1.0))) for k in range(len(g_knots))])
    gk = np.asarray(g_knots, float)
    delay = float(q.get('delay', 0.0))
    n = int(round((t1 - t0) / h))
    tt = t0 + np.arange(n) * h
    idx = np.searchsorted(t_cmd, tt - delay, side='right') - 1
    dsh = pwm_to_dshot(pwm_cmd, pwm_min, pwm_max)
    ds = np.where(idx >= 0, dsh[np.clip(idx, 0, None)], 0).astype(np.int64)
    every = max(1, int(round(out_dt / h)))
    w, cur, d, v, ib, tw, run, tq = _kernel(ds, p, esc.as_array(), gk, gv, float(w0), every, tps)
    t = t0 + (np.arange(len(w)) + 1) * every * h
    rpm = w * 60 / (2 * np.pi)
    return dict(t=t, w=w, rpm=rpm, rpm_telem=np.where(run > 0, rpm, 0.0), i=cur, duty=d,
                vbus=v, ibus=ib, tw=tw, running=run, torque=tq)


def g_of_rpm(q, rpm, g_knots=G_KNOTS):
    gv = np.array([q.get('g%d' % k, q.get('g', 1.0)) for k in range(len(g_knots))])
    return np.interp(rpm, g_knots, gv)


def steady_state(q, duty, vbus_oc, kv, g_knots=G_KNOTS):
    """Analytic steady speed (rad/s) at a duty (0..1) and battery
    open-circuit voltage, including battery sag; iterates on g(rpm)."""
    R, kq, b, T0, Rb = (q.get(k, 0.0) for k in ('R', 'kq', 'b', 'T0', 'Rb'))
    ke = 60.0 / (2 * np.pi * kv)
    d = np.asarray(duty, float)
    w = kv * d * vbus_oc * 2 * np.pi / 60
    for _ in range(40):
        g = g_of_rpm(q, w * 60 / (2 * np.pi), g_knots)
        vth = g * d * vbus_oc
        rth = R + Rb * (g * d) ** 2
        B = b + ke * ke / rth
        C = T0 - ke * vth / rth
        if kq == 0:
            w = -C / B
        else:
            w = (-B + np.sqrt(B * B - 4 * kq * C)) / (2 * kq)
        w = np.maximum(w, 0.0)
    return w


def linear_response(q, w0, freqs, kv, d0, vbus, g_knots=G_KNOTS):
    """Small-signal duty -> speed transfer function (rad/s per unit duty)
    around speed w0, without the ramp limiter and the chain delay."""
    R, J, kq, b, L, Rb = (q.get(k, 0.0) for k in ('R', 'J', 'kq', 'b', 'L', 'Rb'))
    L = L or 30e-6
    g = g_of_rpm(q, w0 * 60 / (2 * np.pi), g_knots)
    ke = 60.0 / (2 * np.pi * kv)
    s = 2j * np.pi * np.asarray(freqs, float)
    Z = L * s + R + Rb * (g * d0) ** 2
    return (ke * g * vbus / Z) / (J * s + 2 * kq * w0 + b + ke * ke / Z)


# parameters that scale with s (multiply) or 1/s (divide) under the
# rpm-invariant symmetry; everything else (g, delay, V0, V1, kv) is unchanged
SCALE_UP = ('J', 'J_m', 'J_p', 'kq', 'b', 'T0', 'Cth', 'Isat')
SCALE_DOWN = ('R', 'Rb', 'L', 'Rth')


def scale_params(q, s):
    """Apply the rpm-invariant scale transformation to a parameter dict
    (keys may carry an '@suffix', e.g. 'kq@1')."""
    out = dict(q)
    for k, v in q.items():
        base = k.split('@')[0]
        if not isinstance(v, (int, float)) or v is None:
            continue
        if base in SCALE_UP:
            out[k] = v * s
        elif base in SCALE_DOWN:
            out[k] = v / s
    return out


def escsim_model(motor, prop, battery_v, rds_on=0.002, poles=14, kv=1300.0):
    """ESCSim SITL model JSON (format: ESCSim/SITL/example.json).

    The averaged model's R is the line-to-line loop resistance of the
    6-step drive (2 phases + 2 conducting FETs), so
    R_phase = (R - 2 rds_on)/2, and the per-phase inductance is L/2.
    The g(rpm) gain table has no SITL equivalent: the SITL's effective
    voltage gain is emergent (dead time, advance, commutation), see notes.
    """
    return {
        'motor': {
            'kv': kv, 'poles': poles,
            'resistance': max((motor['R'] - 2 * rds_on) / 2, 1e-4),
            'inductance': motor.get('L', 30e-6) / 2,
            'mutual_inductance': 0.0,
            'inertia': motor['J_m'] + (prop.get('J_p', 0.0) if prop else 0.0),
            'damping': motor.get('b', 0.0),
            'static_friction': motor.get('T0', 0.0),
            'load_k_omega2': prop.get('kq', 0.0) if prop else 0.0,
        },
        'battery': {'voltage': battery_v, 'resistance': motor.get('Rb', 0.0), 'capacitance': 0.0},
        'esc': {'rds_on': rds_on, 'diode_vf': 0.7, 'commutation_transfer': 1.0, 'temperature_c': 38},
    }
