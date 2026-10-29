#!/usr/bin/env python3
"""Dump the bench-test messages from an ArduPilot dataflash log to CSV.

Writes <out>/<log>_<MSG>.csv for SWP (Lua sweep: pwm, f, rpm), ESC
(instance 0 only), BAT, RCOU and SINE (if present). Time is in seconds
(TimeUS / 1e6).

usage: bench_extract.py ~/vehicles/hakea/logs/sdcard/00000032.BIN [--out DIR]
"""
import argparse
import csv
import os

from pymavlink import mavutil

KEEP = {
    'SWP': ['pwm', 'f', 'rpm'],
    'BNCH': None,
    'SINE': None,
    'ESC': ['RPM', 'RawRPM', 'Volt', 'Curr', 'Temp', 'Err'],
    'BAT': ['Volt', 'VoltR', 'Curr', 'Res'],
    'RCOU': ['C1'],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--out', default=os.path.expanduser('~/vehicles/hakea/notes/sim/bench'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    name = os.path.splitext(os.path.basename(a.log))[0]
    m = mavutil.mavlink_connection(a.log)
    rows = {k: [] for k in KEEP}
    cols = dict(KEEP)
    while True:
        msg = m.recv_match(type=list(KEEP))
        if msg is None:
            break
        t = msg.get_type()
        d = msg.to_dict()
        if t == 'ESC' and d.get('Instance', 0) != 0:
            continue
        if cols[t] is None:
            cols[t] = [k for k in d if k not in ('mavpackettype', 'TimeUS')]
        rows[t].append([d['TimeUS'] / 1e6] + [d.get(k) for k in cols[t]])
    for t, r in rows.items():
        if not r:
            continue
        fn = os.path.join(a.out, '%s_%s.csv' % (name, t))
        with open(fn, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t'] + cols[t])
            w.writerows(r)
        print('%s: %d rows' % (fn, len(r)))


if __name__ == '__main__':
    main()
