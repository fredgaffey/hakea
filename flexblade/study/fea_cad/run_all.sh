#!/bin/bash
# Mac batch: build meshes, write decks, run ccx (OMP 8, waits while load > 12). Run from /tmp/fea/5in_301.
PY=/tmp/fea/env/bin/python3; CCX=/tmp/fea/env/bin/ccx
wait_load() { while [ "$(sysctl -n vm.loadavg | awk '{print int($2)}')" -gt 12 ]; do sleep 30; done; }
run() { (cd "$1" && wait_load && echo "$(date +%T) $1/$2 start" && OMP_NUM_THREADS=8 $CCX -i "$2" > "$2.log" 2>&1; echo "$(date +%T) $1/$2 done $(tail -1 $2.log)"); }
set -x
$PY build.py m05 0.5 40 8 > m05.build.log 2>&1 && $PY cases.py m05 >> m05.build.log 2>&1
for c in ${CASES:-c1_STIFF c1_PACF c1_PETG c1s_STIFF c1s_PACF c2_PACF c2_PETG c3_PACF c3s_PACF c3_PETG}; do run m05 $c; done
if [ -z "$NOREF" ]; then
  $PY build.py m036 0.36 60 12 > m036.build.log 2>&1 && $PY cases.py m036 >> m036.build.log 2>&1
  run m036 c1_PACF
fi
echo ALLDONE
