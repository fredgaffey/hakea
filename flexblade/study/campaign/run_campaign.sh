#!/usr/bin/env bash
# Restart loop: each pass re-probes the Mac (it comes and goes) and resumes the grid.
cd "$(dirname "$0")/../.."          # flexblade/
LOG=study/campaign/full_grid.log
while true; do
  PYTHONPATH=. ~/venv-flexblade/bin/python study/campaign/full_grid.py --hosts "${HOSTS:-local:12,mac:5}" >> "$LOG" 2>&1
  grep -q "^0 to do" "$LOG" && break
  sleep 600
done
