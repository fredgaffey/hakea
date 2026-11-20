#!/bin/bash
# after the running steelx DE: robust ideal seeds 4-6 (Mac may be offline -> local only)
cd "$(dirname "$0")/../.."
while pgrep -f "bin/python.* study/run_study.py" >/dev/null; do sleep 120; done
rm -f study/campaign/STOP_BURN
ROBUST=0.7,1.0,1.4 TAG=robust_ SEEDS="4 5 6" study/campaign/ideal_seeds.sh
