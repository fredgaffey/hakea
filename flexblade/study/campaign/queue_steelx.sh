#!/bin/bash
# after the running ideal seed finishes: robust DE on tt_strap_steelx + tt_strap (pitch preset auto), 7/10/5 in
cd "$(dirname "$0")/../.."
while pgrep -f "^bash study/campaign/ideal_seeds.sh" >/dev/null || pgrep -f "bin/python.* study/ideal_spec.py" >/dev/null; do sleep 60; done
rm -f study/campaign/STOP_BURN
H="local:26,mac:9"
echo "=== $(date -Iseconds) steelx robust DE model $(git rev-parse --short HEAD)" >> study/campaign/steelx_de.log
PYTHONPATH=. ~/venv-flexblade/bin/python study/run_study.py --hosts "$H" --sizes 7,10,5 --concepts tt_strap_steelx,tt_strap \
   --maxiter 60 --popmult 10 --seed 21 --redo --robust 0.7,1.0,1.4 >> study/campaign/steelx_de.log 2>&1
mkdir -p study/campaign/steelx_designs && cp study/designs/*tt_strap*.json study/campaign/steelx_designs/ 2>/dev/null
