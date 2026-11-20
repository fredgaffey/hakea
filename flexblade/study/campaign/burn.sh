#!/usr/bin/env bash
# Never-idle compute: after any running run_study finishes, keep launching new DE runs
# (new seeds, rotating size sets / concepts / coax) on the CURRENT working-tree model.
# Stop: touch study/campaign/STOP_BURN
cd "$(dirname "$0")/../.."          # flexblade/
LOG=study/campaign/burn.log
CONCEPTS="tpu_rod,torsion_bar,tpu_split,cf_flexure_sym,tpu_cf_skin,cf_flexure,tpu95_split"
SIZESETS=("4,5,6,7,8" "9,10,11,12" "5,6,7,8,10" "4,6,8,10,12")
COAX=("--coax 7:tpu_rod --coax 10:tpu_rod --coax 7:torsion_bar" "--coax 5:cf_flexure_sym --coax 8:torsion_bar --coax 12:tpu_rod")
seed=${SEED0:-2}
while [ ! -f study/campaign/STOP_BURN ]; do
  while pgrep -f "study/run_study.py" >/dev/null; do sleep 120; done
  i=$(( (seed - 2) % ${#SIZESETS[@]} ))
  if ssh -o ConnectTimeout=5 mac true 2>/dev/null; then H="local:26,mac:9"; else H="local:30"; fi
  extra=""; [ $((seed % 2)) -eq 0 ] && extra="${COAX[$(( (seed/2) % 2 ))]}"
  echo "=== $(date -Is) seed $seed sizes ${SIZESETS[$i]} hosts $H $extra (model $(git rev-parse --short HEAD))" >> "$LOG"
  PYTHONPATH=. ~/venv-flexblade/bin/python study/run_study.py --hosts "$H" --sizes "${SIZESETS[$i]}" \
      --concepts "$CONCEPTS" --maxiter 80 --popmult 12 --seed "$seed" --redo $extra >> "$LOG" 2>&1
  # designs/<case>.json is overwritten by every run -> snapshot per seed; index cases.jsonl lines
  mkdir -p study/campaign/burn_designs/seed$seed && cp study/designs/*.json study/campaign/burn_designs/seed$seed/ 2>/dev/null
  echo "seed $seed model $(git rev-parse --short HEAD) cases_lines_end $(wc -l < study/results/cases.jsonl)" >> study/campaign/burn_index.txt
  seed=$((seed + 1))
  sleep 30
done
