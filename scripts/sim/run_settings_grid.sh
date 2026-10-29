#!/bin/bash
# ESC-settings search at 1P frequencies, nominal (commutation_transfer 1) and
# pessimistic-demag (0.8) SITL motor models. Results: notes/sim/grid/settings_*.csv
cd ~/vehicles/hakea/scripts/sim
OUT=~/vehicles/hakea/notes/sim/grid
PY=~/venv-ardupilot/bin/python3
common="--variants 255 --freqs 250,300 --amps 0,10,20,30,40 --workers 12"
for ct in 1.0 0.8; do
  $PY grid.py $common --set commutation_transfer=$ct --tag base_ct$ct --out $OUT/settings_base_ct$ct.csv
  $PY grid.py $common --set commutation_transfer=$ct --ee pwm_frequency=48 --tag pwm48_ct$ct --out $OUT/settings_pwm48_ct$ct.csv
  $PY grid.py $common --set commutation_transfer=$ct --ee pwm_frequency=16 --tag pwm16_ct$ct --out $OUT/settings_pwm16_ct$ct.csv
  $PY grid.py $common --set commutation_transfer=$ct --ee comp_pwm=0 --tag comp0_ct$ct --out $OUT/settings_comp0_ct$ct.csv
  $PY grid.py $common --set commutation_transfer=$ct --ee variable_pwm=0 --tag varpwm0_ct$ct --out $OUT/settings_varpwm0_ct$ct.csv
  for adv in 18 26 34 42; do
    $PY grid.py $common --set commutation_transfer=$ct --ee auto_advance=0 --ee advance_level=$adv --tag adv${adv}_ct$ct --out $OUT/settings_adv${adv}_ct$ct.csv
  done
done
