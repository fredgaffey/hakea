#!/bin/bash
# Second Mac batch (the NLGEOM hover run stalled at 1/256 increments): modal at rest, LINEAR hover,
# case-1 mesh refinement, then the CF-prestressed modal capped at 12 min. Run from /tmp/fea/5in_301.
PY=/tmp/fea/env/bin/python3; CCX=/tmp/fea/env/bin/ccx
run() { (cd "$1" && echo "$(date +%T) $1/$2 start" && OMP_NUM_THREADS=8 perl -e 'alarm shift; exec @ARGV' "${3:-3000}" $CCX -i "$2" > "$2.log" 2>&1; echo "$(date +%T) $1/$2 done $(tail -1 $2.log)"); }
$PY cases.py m05 > m05.cases2.log 2>&1
run m05 c3s_PACF
run m05 c2_PACF
run m05 c2_PETG
$PY build.py m036 0.36 60 12 > m036.build.log 2>&1 && $PY cases.py m036 >> m036.build.log 2>&1
run m036 c1_PACF
run m05 c3_PACF 720
echo ALLDONE
