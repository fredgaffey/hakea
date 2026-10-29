#!/usr/bin/env bash
# 20 us bridge batch amplitude sweep at 300 Hz (ESCSim's stock coupling), to
# see the PR #438 stress metric vs amplitude where it is not pinned at 255
# by the half-step zero crosses the 2 us batch shows. 600 is in run_all_v2.sh.
set -uo pipefail
H=$HOME/vehicles/hakea; R=$H/scripts/sim/renode; V=$H/am32-renode-pr438/variants
OUT=$H/notes/sim/renode/v2/raw; L=$H/notes/sim/renode/v2/logs
source $HOME/venv-ardupilot/bin/activate
declare -A ELF=([stock]=$H/firmware/AM32_FOXEER_F421_head-pr438_bdbc75e.elf
  [ramp64]=$V/FOXEER_F421_ramp64.elf [ramp255]=$V/FOXEER_F421_ramp255.elf)
declare -A PORT=([stock]=34571 [ramp64]=34572 [ramp255]=34573)
for v in stock ramp64 ramp255; do
  ( for a in 100 200 400 1000; do
      python "$R/run_crosscheck_v2.py" FOXEER_F421 "${ELF[$v]}" --name "${v}_b20_f300_a${a}" \
        --freq 300 --amp $a --batch-us 20 --monitor-port ${PORT[$v]} --outdir "$OUT" \
        > "$L/${v}_b20_f300_a${a}.log" 2>&1
    done ) &
done
wait
python "$R/analyze_v2.py"
