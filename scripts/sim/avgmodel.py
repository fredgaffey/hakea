#!/usr/bin/env python3
"""Averaged (commutation-averaged) model of Hakea's ESC + motor + prop.

This is the "simple physics" model used to calibrate against the bench
logs. It is deliberately much simpler than the ESCSim SITL: no PWM
switching, no commutation, no back-EMF zero-crossing detection. It
treats the 6-step BLDC drive as an equivalent DC motor, which is a good
approximation for the *average* torque and speed below a few hundred Hz.

Signal chain (all in SI units):

  ArduPilot pwm (us) --> DShot value = 2*(pwm-1000)+48     (AP RCOutput.cpp)
  DShot --> duty setpoint = map(dshot, 47..2047 -> min_duty..2000) (AM32 main.c)
  setpoint --> low-rpm duty clamp, ramp limiter (per 20 kHz tick)  (AM32 main.c)
  duty d (0..1) --> line voltage g*d*Vbus   (g = effective voltage gain:
                     dead time, timing advance, commutation losses)
  L dI/dt = g*d*Vbus - Ke*w - R*I           (R = 2*R_phase + 2*Rds_on)
  J dw/dt = Kt*I - kq*w*|w| - b*w - T0      (prop torque ~ w^2)
  Ibus = g*d*I ;  Vbus = Voc - Rb*Ibus      (battery internal resistance)

Ke = Kt = 60/(2*pi*Kv) (line-to-line, DC-equivalent). Kv is fixed at the
nominal 1300 and g absorbs the difference between nominal and
effective speed constant; see the report for what this means for
identifiability.

The bench measurement chain (Lua -> ArduPilot loop -> DShot -> ESC ->
bidir DShot telemetry -> Lua) is modelled as a pure delay `delay` on
the input.

Everything heavy is numba-compiled so a 60 s log simulates in ~0.1 s.
"""
import numpy as np
from numba import njit

KV_NOMINAL = 1300.0
POLES = 14

# AM32 constants for Fred's ESC (esc1_config.bin + FOXEER_F421 target)
MIN_DUTY = 40          # eeprom minimum_duty_cycle 4 * 10
RAMP_STARTUP = 2       # RAMP_SPEED_STARTUP
RAMP_LOW = 6           # RAMP_SPEED_LOW_RPM
RAMP_HIGH_DEFAULT = 16  # RAMP_SPEED_HIGH_RPM (eeprom max_ramp 160 -> 16)
LOW_RPM_LEVEL = 6      # k_erpm: motor_kv/100/(32/poles)
HIGH_RPM_LEVEL = 54    # k_erpm: motor_kv/12/(32/poles)
TICK = 50e-6           # 20 kHz control loop

# parameter vector layout used by simulate()
PNAMES = ['Isat', 'R', 'J', 'kq', 'b', 'T0', 'L', 'Rb', 'V0', 'V1', 'Rth', 'Cth', 'Tw0', 'delay']
# Rth (K/W), Cth (J/K): lumped winding thermal model, Cth = 0 disables it.
# R is the total loop resistance at 25 C; the winding share RW_FRAC of it
# rises with copper temperature (0.393 %/K). Tw0 = initial winding temp.
RW_FRAC = 0.95
ALPHA_CU = 0.00393
T_AMB = 25.0


def pwm_to_dshot(pwm):
    pwm = np.clip(np.asarray(pwm, float), 1000, 2000)
    v = np.minimum(2 * (pwm - 1000), 1999)
    return np.where(v > 0, v + 48, 0)


@njit(cache=True)
def _amap(x, in_min, in_max, out_min, out_max):
    # AM32 map(): integer, clamps to the output range
    if x < in_min:
        x = in_min
    if x > in_max:
        x = in_max
    return out_min + (x - in_min) * (out_max - out_min) // (in_max - in_min)


# speed knots (rpm) for the effective voltage gain table g(rpm). g is
# interpolated linearly between knots; params g0..g4 hold the values.
G_KNOTS = np.array([0.0, 8000.0, 14000.0, 20000.0, 30000.0])
RPM_STOP = 2000.0  # below this AM32 loses BEMF tracking (running=0)


@njit(cache=True)
def _sim(dshot_ticks, p, w0, ramp_high, kv, out_every, gk, gv, dmod):
    """Integrate the averaged model at the 20 kHz firmware tick.

    dshot_ticks: DShot value per tick (already delayed)
    returns arrays sampled every `out_every` ticks: w, I, duty(0..1),
    vbus, ibus, duty_setpoint(0..1)
    """
    Isat, R25, J, kq, b, T0, L, Rb, V0, V1 = p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9]
    Rth, Cth, Tw = p[10], p[11], p[12]
    if Tw == 0:
        Tw = T_AMB
    R = R25
    ke = 60.0 / (2 * np.pi * kv)
    n = dshot_ticks.shape[0]
    nout = n // out_every
    o_w = np.zeros(nout)
    o_i = np.zeros(nout)
    o_d = np.zeros(nout)
    o_v = np.zeros(nout)
    o_ib = np.zeros(nout)
    o_sp = np.zeros(nout)
    o_tw = np.zeros(nout)
    o_pcu = np.zeros(nout)
    pcu_acc = 0.0
    w = w0
    cur = 0.0
    duty = 0  # AM32 duty_cycle, 0..2000
    zero_crosses = 0.0
    if w0 > 100:
        zero_crosses = 1e9
    ibus = 0.0
    # sub-steps per tick for the electrical pole (L/R ~ 0.3 ms)
    nsub = 5
    h = TICK / nsub
    k = 0
    for i in range(n):
        t = i * TICK
        ds = dshot_ticks[i]
        # duty setpoint from input
        if ds >= 48:
            sp = _amap(ds, 47, 2047, MIN_DUTY, 2000)
        else:
            sp = 0
        # low rpm power protection (k_erpm = mech rpm * pole pairs / 1000)
        rpm = w * 60 / (2 * np.pi)
        k_erpm = int(rpm * (POLES // 2) / 1000.0)
        dmax = _amap(k_erpm, LOW_RPM_LEVEL, HIGH_RPM_LEVEL, 400, 2000)
        if sp > dmax:
            sp = dmax
        # ramp limiter
        if zero_crosses < 150 or duty < 150:
            mx = RAMP_STARTUP
        else:
            # AM32: average_interval = e_com_time / 3, e_com_time = one
            # electrical revolution in us; > 500 -> low rpm ramp (< ~5.7k rpm)
            erev_us = 1e6 / max(rpm / 60.0 * (POLES // 2), 1e-3)
            mx = RAMP_LOW if erev_us / 3 > 500 else ramp_high
        if sp - duty > mx:
            duty = duty + mx
        elif duty - sp > mx:
            duty = duty - mx
        else:
            duty = sp
        running = w > RPM_STOP * 2 * np.pi / 60 or ds >= 48
        if not running:
            duty = 0
        d = duty / 2000.0
        # optional in-loop modulation added AFTER the ramp limiter (Hakea
        # PROFILE_MODE v1): clamp to [180, 2000] duty units
        if dmod[i] != 0.0 and duty > 0:
            dd = duty + dmod[i]
            if dd < 180.0:
                dd = 180.0
            if dd > 2000.0:
                dd = 2000.0
            d = dd / 2000.0
        g = np.interp(rpm, gk, gv)
        # zero crosses counted while spinning (6 per electrical rev)
        if w > 50:
            zero_crosses += rpm / 60.0 * (POLES // 2) * 6 * TICK
        else:
            zero_crosses = 0.0
        voc = V0 + V1 * t
        for s in range(nsub):
            vbus = voc - Rb * ibus
            if running:
                # complementary PWM: the bridge is a synchronous buck,
                # current can reverse (regenerative braking). At zero
                # input AM32 (comp_pwm=1, brake_on_zero_throttle=0)
                # ramps the duty to 0 while still commutating, which
                # shorts the windings: strong dynamic braking
                vapp = g * d * vbus
                di = (vapp - ke * w - R * cur) / L
                cur += di * h
                ibus = g * d * cur
            else:
                # not running: bridge off (brake_on_stop 0), free coast
                cur = 0.0
                ibus = 0.0
            tl = kq * w * abs(w) + b * w
            if w > 0.5:
                tl += T0
            # optional torque-constant roll-off at high current
            # (Isat = 0: ideal, torque = Ke * I)
            kt = ke if Isat <= 0 else ke / (1.0 + abs(cur) / Isat)
            dw = (kt * cur - tl) / J
            w += dw * h
            if w < 0:
                w = 0.0
        # copper loss and winding temperature (once per tick)
        pcu = cur * cur * R * RW_FRAC
        pcu_acc += pcu
        if Cth > 0:
            Tw += (pcu - (Tw - T_AMB) / Rth) / Cth * TICK
            R = R25 * (1 - RW_FRAC + RW_FRAC * (1 + ALPHA_CU * (Tw - T_AMB)))
        if (i + 1) % out_every == 0:
            o_tw[k] = Tw
            o_pcu[k] = pcu_acc / out_every
            pcu_acc = 0.0
            o_w[k] = w
            o_i[k] = cur
            o_d[k] = d
            o_v[k] = voc - Rb * ibus
            o_ib[k] = ibus
            o_sp[k] = sp / 2000.0
            k += 1
    return o_w, o_i, o_d, o_v, o_ib, o_sp, o_tw, o_pcu


def simulate(params, t_cmd, pwm_cmd, t0, t1, w0=0.0, ramp_high=RAMP_HIGH_DEFAULT,
             kv=KV_NOMINAL, out_dt=0.0005, dshot_cmd=None, duty_mod=None):
    """Simulate from t0 to t1 with the commanded pwm (zero-order hold).

    params: dict with PNAMES keys. V1 is volts per second from t0.
    returns dict of arrays on a uniform time grid (out_dt).
    """
    p = np.array([params.get(k, 0.0) for k in PNAMES[:13]], float)
    gv = np.array([params.get('g%d' % k, params.get('g', 1.0)) for k in range(len(G_KNOTS))], float)
    delay = params.get('delay', 0.0)
    n = int(round((t1 - t0) / TICK))
    tt = t0 + np.arange(n) * TICK
    # zero-order hold of the command, shifted by the chain delay
    idx = np.searchsorted(t_cmd, tt - delay, side='right') - 1
    if dshot_cmd is None:
        dshot_cmd = pwm_to_dshot(pwm_cmd)
    ds = np.where(idx >= 0, np.asarray(dshot_cmd)[np.clip(idx, 0, None)], 0).astype(np.int64)
    every = max(1, int(round(out_dt / TICK)))
    dm = np.zeros(n) if duty_mod is None else np.asarray(duty_mod(tt), float)
    w, cur, d, v, ib, sp, tw, pcu = _sim(ds, p, float(w0), int(ramp_high), float(kv), every, G_KNOTS, gv, dm)
    t = t0 + (np.arange(len(w)) + 1) * every * TICK
    return dict(t=t, w=w, rpm=w * 60 / (2 * np.pi), i=cur, duty=d, vbus=v, ibus=ib, sp=sp, tw=tw, pcu=pcu)


def g_of_rpm(params, rpm):
    gv = np.array([params.get('g%d' % k, params.get('g', 1.0)) for k in range(len(G_KNOTS))])
    return np.interp(rpm, G_KNOTS, gv)


def steady_state(params, duty, vbus_oc, kq=None, kv=KV_NOMINAL):
    """Analytic steady-state speed (rad/s) at a given duty (0..1),
    including battery sag. Solves Kt*I = kq w^2 + b w + T0 with
    I = (g d Vbus - Ke w)/R, Vbus = Voc - Rb g d I, iterating on the
    speed-dependent gain g(rpm)."""
    R, kq_, b, T0, Rb = (params.get(k, 0.0) for k in ('R', 'kq', 'b', 'T0', 'Rb'))
    if kq is not None:
        kq_ = kq
    ke = 60.0 / (2 * np.pi * kv)
    d = np.asarray(duty, float)
    w = kv * d * vbus_oc * 2 * np.pi / 60
    for _ in range(30):
        g = g_of_rpm(params, w * 60 / (2 * np.pi))
        vth = g * d * vbus_oc
        rth = R + Rb * (g * d) ** 2
        A = kq_
        B = b + ke * ke / rth
        C = T0 - ke * vth / rth
        if np.all(np.asarray(A) == 0):
            w = -C / B
        else:
            w = (-B + np.sqrt(B * B - 4 * A * C)) / (2 * A)
    return w


def linear_response(params, w0, freqs, kv=KV_NOMINAL, d0=None, vbus=None):
    """Small-signal transfer function duty -> speed around speed w0,
    from the linearised model (no ramp limit). Returns complex
    d(w)/d(duty) in rad/s per unit duty, at the given frequencies.

    Linearisation: L sI = g Vbus dd - Ke dw - R dI
                   J s dw = Kt dI - (2 kq w0 + b) dw
    (battery sag folded into R as + Rb (g d0)^2 and Vbus as the loaded
    bus voltage)
    """
    R, J, kq, b, L, Rb = (params.get(k, 0.0) for k in ('R', 'J', 'kq', 'b', 'L', 'Rb'))
    g = g_of_rpm(params, w0 * 60 / (2 * np.pi))
    ke = 60.0 / (2 * np.pi * kv)
    s = 2j * np.pi * np.asarray(freqs, float)
    Rt = R + (Rb * (g * d0) ** 2 if d0 is not None else 0)
    Bm = 2 * kq * w0 + b
    # dI = (g V dd - ke dw)/(L s + Rt);  J s dw = ke dI - Bm dw
    # => dw (J s + Bm + ke^2/(L s + Rt)) = ke g V dd/(L s + Rt)
    Z = L * s + Rt
    return (ke * g * vbus / Z) / (J * s + Bm + ke * ke / Z)
