#!/bin/bash
# Build the AM32 SITL variants used by the ramp study, from the
# ~/vehicles/hakea/am32-sim worktree (branch sim/ramp-study).
#   ramp16  = stock ramp limit (RAMP_SPEED_HIGH_RPM 16)
#   rampN   = max_ramp_high_rpm forced to N per 20 kHz tick
# All use DEAD_TIME 47 (= 294 ns at the SITL's 160 MHz TIM1), matching the
# FOXEER_F421 target's DEAD_TIME 35 at 120 MHz.
set -e
cd ~/vehicles/hakea/am32-sim
mkdir -p variants
for r in ${@:-16 32 64 255}; do
  if [ "$r" = 16 ]; then extra=""; else extra="-DSIM_RAMP_HIGH_OVERRIDE=$r"; fi
  make -B AM32_SITL_CAN SIM_CFLAGS="-DSIM_DEAD_TIME=47 $extra" 2>&1 | grep -v "^Makefile:" | grep -v "^$" || true
  cp obj/AM32_AM32_SITL_CAN_2.21.elf variants/sitl_ramp$r.elf
  echo "built variants/sitl_ramp$r.elf ($extra)"
done
