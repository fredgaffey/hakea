#!/bin/bash
# stop_job.sh <job> [note]
# Stop the job the runner is currently executing and everything under it (DE main, resolve pools,
# local flexblade.remote pools, ssh links to remote workers), walking the process tree by PID.
# Marks done/<job> with the note so the runner skips it, unless note is "-" (then it is retried later).
# Never kills the runner itself, memguard, or anything outside the job's tree.
set -u
C="$(cd "$(dirname "$0")" && pwd)"
job="${1:?usage: stop_job.sh <job> [note|-]}"; note="${2:-stopped by stop_job.sh}"
runner=$(pgrep -f "^bash study/campaign/runner.sh" | head -1)
[ -z "$runner" ] && { echo "runner not running"; exit 1; }
last=$(grep "START" "$C/runner.log" | tail -1)
[[ "$last" == *" START $job "* ]] || { echo "current job is not $job: $last"; exit 1; }
top=$(ps -eo pid=,ppid=,args= | awk -v r="$runner" '$2==r && $3=="bash" && $4=="-c" {print $1}' | head -1)
[ -z "$top" ] && { echo "no job wrapper under runner $runner"; exit 1; }
tree() { local p; for p in $(ps -eo pid=,ppid= | awk -v q="$1" '$2==q {print $1}'); do tree "$p"; echo "$p"; done; }
pids=$(tree "$top")
[ -n "${DRY:-}" ] && { echo "DRY: would stop $job wrapper $top pids:" $pids; exit 0; }
[ "$note" != "-" ] && echo "$note" > "$C/done/$job"
echo "stopping $job: wrapper $top + $(echo $pids | wc -w) descendants"
kill $pids 2>/dev/null; sleep 5; kill -9 $pids 2>/dev/null
echo "left: $(for p in $pids; do ps -p $p >/dev/null 2>&1 && echo -n "$p "; done)"
