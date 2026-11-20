#!/bin/bash
# Memory guard (overnight OOM 2026-11-21): make compute workers the OOM killer's first choice,
# and if MemAvailable < 1.5 GB kill the largest flexblade python process (logged), not the session.
LOG="$(dirname "$0")/memguard.log"
while true; do
  for p in $(pgrep -f "flexblade.remote|study/run_study.py|study/ideal_spec.py|pytest"); do
    echo 800 > /proc/$p/oom_score_adj 2>/dev/null
  done
  avail=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
  if [ "$avail" -lt 1500 ]; then
    victim=$(ps -eo pid,rss,args --sort=-rss | grep "[v]env-flexblade/bin/python" | head -1)
    echo "$(date -Iseconds) MemAvailable ${avail} MB -> kill $victim" >> "$LOG"
    kill $(echo "$victim" | awk '{print $1}') 2>/dev/null
    sleep 30
  fi
  sleep 15
done
