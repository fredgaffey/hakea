#!/usr/bin/env python3
"""1P torque-pulse requirements for 2-3" rotors on 11xx 12N14P motors.

Uncompensated: a first-order current/torque loop with corner fc lags
atan(1P/fc); <= 10 deg needs fc >= 1P/tan(10 deg) = 5.7 x 1P.
Compensated: the lag is fixed and calibratable, so fc only has to keep the
amplitude (>= 0.7 at fc = 1P; >= 0.9 at 2 x 1P). AM32 control tick 20 kHz.
"""
import math

PP = 7            # pole pairs, 12N14P
TICK = 20e3       # AM32 tenKhzRoutine (LOOP_FREQUENCY_HZ)
L_R = 8e-6 / 0.25  # 11xx phase L/R ~ 32 us (L ~ 8 uH, R ~ 0.25 ohm)

print("| size | rpm | 1P Hz | e-freq Hz | eRPM | comm/rev (deg each) | PWM/rev @24/48 kHz "
      "| AM32 ticks/rev (deg) | fc for <=10 deg | fc comp. (>=0.9 amp) | L/R lag @1P |")
print("|---|---|---|---|---|---|---|---|---|---|---|")
for size, rpm in (("2\"", 60000), ("2.5\"", 45000), ("3\"", 30000)):
    f1 = rpm / 60
    fe = f1 * PP
    comm = 6 * PP
    lag = math.degrees(math.atan(2 * math.pi * f1 * L_R))
    print(f"| {size} | {rpm//1000}k | {f1:.0f} | {fe:.0f} | {fe*60/1e3:.0f}k | {comm} ({360/comm:.1f}) "
          f"| {24e3/f1:.0f} / {48e3/f1:.0f} | {TICK/f1:.0f} ({360*f1/TICK:.0f}) "
          f"| {f1/math.tan(math.radians(10))/1e3:.1f} kHz | {2*f1/1e3:.1f} kHz | {lag:.0f} deg |")
