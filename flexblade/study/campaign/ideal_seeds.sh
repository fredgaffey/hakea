#!/usr/bin/env bash
# Targeted compute: ideal-flexure DE from several seeds -> cluster into ideal families.
cd "$(dirname "$0")/../.."
LOG=study/campaign/ideal_seeds.log
for seed in ${SEEDS:-1 2 3 4 5 6}; do
  [ -f study/campaign/STOP_BURN ] && break
  if ssh -o ConnectTimeout=5 mac true 2>/dev/null; then H="local:26,mac:9"; else H="local:30"; fi
  echo "=== $(date -Is) ideal seed $seed hosts $H model $(git rev-parse --short HEAD)" >> $LOG
  PYTHONPATH=. ~/venv-flexblade/bin/python study/ideal_spec.py --hosts "$H" --sizes ${SIZES:-5,7,10} \
      --popmult 10 --maxiter 60 --seed $seed --enforce-thermal ${ROBUST:+--robust $ROBUST} >> $LOG 2>&1
  mkdir -p study/campaign/ideal_seeds/${TAG:-}seed$seed
  cp study/results/ideal_spec_*in.json study/designs/ideal_*in.json study/campaign/ideal_seeds/${TAG:-}seed$seed/ 2>/dev/null
done
