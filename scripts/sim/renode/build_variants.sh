#!/usr/bin/env bash
# Build the AM32 firmware variants used by the Renode cross-check.
#
# Runs in the am32-renode worktree (branch sim/renode-check, with
# scripts/sim/ramp-study.patch applied). Each variant is rebuilt from
# scratch (make -B) because every variant writes the same obj/ file name,
# then copied to variants/<TARGET>_<variant>.elf so they can't clobber
# each other.
#
#   ramp16  = stock firmware (max_ramp_high_rpm = RAMP_SPEED_HIGH_RPM = 16,
#             possibly lowered by EEPROM max_ramp/10)
#   ramp64  = -DSIM_RAMP_HIGH_OVERRIDE=64  (forced after the EEPROM clamp)
#   ramp255 = -DSIM_RAMP_HIGH_OVERRIDE=255
#
# ARM_SDK_PREFIX points make at the arm-none-eabi-gcc in PATH instead of
# the xpack toolchain AM32's make/tools.mk expects under tools/linux.
set -euo pipefail
WT=${WT:-$HOME/vehicles/hakea/am32-renode}
TARGETS=${TARGETS:-"FOXEER_F421 VIMDRONES_L431_CAN"}
cd "$WT"
mkdir -p variants
for t in $TARGETS; do
  for v in ramp16 ramp64 ramp255; do
    case $v in
      ramp16)  extra="" ;;
      ramp64)  extra="-DSIM_RAMP_HIGH_OVERRIDE=64" ;;
      ramp255) extra="-DSIM_RAMP_HIGH_OVERRIDE=255" ;;
    esac
    echo "=== $t $v SIM_CFLAGS='$extra'"
    rm -rf obj
    make -B "$t" ARM_SDK_PREFIX=arm-none-eabi- SIM_CFLAGS="$extra" \
      > "variants/build_${t}_${v}.log" 2>&1
    elf=$(ls obj/AM32_${t}_*.elf)
    cp "$elf" "variants/${t}_${v}.elf"
    cp "${elf%.elf}.bin" "variants/${t}_${v}.bin"
  done
done
ls -l variants/*.elf
