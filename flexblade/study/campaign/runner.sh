#!/bin/bash
# Reboot-safe compute runner. Jobs are lines in jobs.txt: "<name> <command...>" (run from flexblade/).
# A job is skipped once done/<name> exists; after a reboot the runner (started by @reboot cron)
# continues with the first unfinished job (grids resume themselves; DE restarts that case).
cd "$(dirname "$0")/../.."
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMBA_NUM_THREADS=1
C=study/campaign; mkdir -p $C/done
# Expiry (Fred away 2 days): after this time the runner removes its own @reboot crontab line,
# starts no new jobs and exits (a job already running finishes normally).
EXPIRES="${RUNNER_EXPIRES:-2026-11-23T10:30}"
expire_check() {
  if [ "$(date +%s)" -ge "$(date -d "$EXPIRES" +%s)" ]; then
    crontab -l 2>/dev/null | grep -v "study/campaign/runner.sh" | crontab -
    echo "=== $(date -Iseconds) EXPIRED ($EXPIRES): removed @reboot crontab entry, runner exiting" >> $C/runner.log
    exit 0
  fi
}
expire_check
exec 9>$C/.runner.lock; flock -n 9 || exit 0          # one runner at a time
while pgrep -f "^/bin/bash study/campaign/queue_steelx.sh" >/dev/null; do sleep 120 9>&-; done   # let the legacy queue finish
pgrep -f "study/campaign/memguard.sh" >/dev/null || (setsid nohup $C/memguard.sh 9>&- </dev/null >/dev/null 2>&1 &)
while true; do
  job=""; while read -r name cmd; do
    [[ -z "$name" || "$name" == \#* ]] && continue
    [ -e "$C/done/$name" ] && continue
    [ "$(cat $C/done/$name.fail 2>/dev/null || echo 0)" -ge 3 ] && continue   # gave up after 3 failures
    job="$name"; jcmd="$cmd"; break
  done < $C/jobs.txt
  expire_check
  [ -z "$job" ] && { sleep 300 9>&-; continue; }           # idle: re-read jobs.txt (I append new jobs)
  [ -e $C/STOP_BURN ] && { sleep 60 9>&-; continue; }
  echo "=== $(date -Iseconds) START $job model $(git rev-parse --short HEAD)" >> $C/runner.log
  if ssh -o ConnectTimeout=5 mac true 2>/dev/null; then export H="local:${LOCALW:-20},mac:9"; else export H="local:${LOCALW:-22}"; fi
  bash -c "$jcmd" 9>&- >> $C/runner.log 2>&1; rc=$?
  if [ $rc -eq 0 ]; then touch $C/done/$job; else echo $(( $(cat $C/done/$job.fail 2>/dev/null || echo 0) + 1 )) > $C/done/$job.fail; fi
  echo "=== $(date -Iseconds) END $job rc=$rc" >> $C/runner.log
done
