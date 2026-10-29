#!/usr/bin/env bash
# Run the whole Renode cross-check and summarise it.
#   1. builds nothing - run build_variants.sh first
#   2. one Renode run per firmware variant (each ~20-40 s wall)
#   3. analyze.py -> notes/sim/renode/summary.csv + markdown table
# Uses TCP 34571 for the Renode monitor; no UDP ports (CAN left unconnected).
set -euo pipefail
H=$HOME/vehicles/hakea
R=$H/scripts/sim/renode
V=$H/am32-renode/variants
OUT=$H/notes/sim/renode/raw
source $HOME/venv-ardupilot/bin/activate
mkdir -p "$OUT"
# record exactly which model this batch used
cp "$H/scripts/sim/models/hakea_2307_5in.json" "$H/notes/sim/renode/model_used.json"

for v in ramp16 ramp255 ramp64; do
  python "$R/run_crosscheck.py" FOXEER_F421 "$V/FOXEER_F421_$v.elf" \
      --name "FOXEER_F421_$v" --outdir "$OUT"
done
# The CAN build needs a real DroneCAN node id: Fred's esc1_config.bin has
# 0xFD in every CAN byte (node 253 is invalid, so the firmware sits in
# dynamic node allocation), and in that state the emulated CAN firmware
# cannot start this motor. esc1_config_can11.bin is Fred's file with only
# bytes 176 (can_node=11) and 177 (esc_index=0) changed.
for v in ramp16 ramp255 ramp64; do
  python "$R/run_crosscheck.py" VIMDRONES_L431_CAN "$V/VIMDRONES_L431_CAN_$v.elf" \
      --name "VIMDRONES_L431_CAN_$v" --outdir "$OUT" \
      --eeprom "$R/eeprom/esc1_config_can11.bin"
done
python "$R/analyze.py"
