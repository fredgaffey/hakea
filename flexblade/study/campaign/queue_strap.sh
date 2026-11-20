#!/usr/bin/env bash
# After the running ideal seed: robust DE on tt_strap (5/7/10"), then resume ideal seeds 2-3.
cd "$(dirname "$0")/../.."
touch study/campaign/STOP_BURN                      # ideal_seeds.sh stops after current seed
while pgrep -f "bin/python.* study/ideal_spec.py" >/dev/null; do sleep 60; done
rm -f study/campaign/STOP_BURN
H="local:26,mac:9"; ssh -o ConnectTimeout=5 mac true 2>/dev/null || H="local:30"
echo "=== $(date -Is) tt_strap robust DE model $(git rev-parse --short HEAD)" >> study/campaign/strap_de.log
PYTHONPATH=. ~/venv-flexblade/bin/python study/run_study.py --hosts "$H" --sizes 5,7,10 --concepts tt_strap,tt_strap_xplate \
   --maxiter 60 --popmult 10 --seed 11 --redo --robust 0.7,1.0,1.4 >> study/campaign/strap_de.log 2>&1
mkdir -p study/campaign/strap_designs && cp study/designs/*tt_strap*.json study/campaign/strap_designs/ 2>/dev/null
ROBUST=0.7,1.0,1.4 TAG=robust_ SEEDS="2 3" study/campaign/ideal_seeds.sh
