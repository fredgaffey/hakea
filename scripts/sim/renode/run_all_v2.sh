#!/usr/bin/env bash
# v2 Renode matrix: PR #438 firmware, calibrated prop model, Fred's EEPROM.
#   variants: stock = Fred's exact flashed ELF; ramp64/ramp255 built from
#             the same commit (bdbc75e) in am32-renode-pr438/variants/
#   f = 250, 300 Hz; amp = 100 200 400 600 1000 DShot; base 1048
#   bridge/physics batch 2 us for all, plus 300 Hz +-600 at 20 us.
# The three variants run in parallel (one Renode each, TCP monitor ports
# 34571-34573). Each run is ~30-40 s wall.
set -uo pipefail
H=$HOME/vehicles/hakea
R=$H/scripts/sim/renode
V=$H/am32-renode-pr438/variants
OUT=$H/notes/sim/renode/v2/raw
source $HOME/venv-ardupilot/bin/activate
mkdir -p "$OUT" "$H/notes/sim/renode/v2/logs"
cp "$H/scripts/sim/models/hakea_calibrated_prop_24v3.json" "$H/notes/sim/renode/v2/model_used.json"

declare -A ELF=(
  [stock]=$H/firmware/AM32_FOXEER_F421_head-pr438_bdbc75e.elf
  [ramp64]=$V/FOXEER_F421_ramp64.elf
  [ramp255]=$V/FOXEER_F421_ramp255.elf
)
declare -A PORT=([stock]=34571 [ramp64]=34572 [ramp255]=34573)

one_variant() {
  local v=$1
  for f in 250 300; do
    for a in 100 200 400 600 1000; do
      python "$R/run_crosscheck_v2.py" FOXEER_F421 "${ELF[$v]}" \
        --name "${v}_b2_f${f}_a${a}" --freq $f --amp $a --batch-us 2 \
        --monitor-port ${PORT[$v]} --outdir "$OUT" \
        > "$H/notes/sim/renode/v2/logs/${v}_b2_f${f}_a${a}.log" 2>&1
    done
  done
  python "$R/run_crosscheck_v2.py" FOXEER_F421 "${ELF[$v]}" \
    --name "${v}_b20_f300_a600" --freq 300 --amp 600 --batch-us 20 \
    --monitor-port ${PORT[$v]} --outdir "$OUT" \
    > "$H/notes/sim/renode/v2/logs/${v}_b20_f300_a600.log" 2>&1
}

for v in stock ramp64 ramp255; do
  one_variant $v &
done
wait
python "$R/analyze_v2.py"
