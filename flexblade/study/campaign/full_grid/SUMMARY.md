| size | method | feasible/done | best | ctrl | pitch1p° | max util | governing |
|---|---|---|---|---|---|---|---|
| 5" | cfsym | 0/840 | 5|cfsym|sk30|t1.5|L0.6|w1.25|g1.2 | 0.005 | 0.21 | 1.19 | creep_strain:flex2 |
| 5" | rod | 30955/48384 | 5|rod|tpu_95a|d1.4|n3|sk45|t3.5|hr1.2|L1.25|w0.8 | 0.319 | 16.54 | 0.99 | motor_torque_1p:motor |
| 5" | strap | 4318/14112 | 5|strap|tpu_64d|d1.2|sk45|t2.0|hr1.2|L0.8|w1.25 | 0.013 | 0.86 | 0.94 | esc_duty_peak:esc |
| 5" | tbar | 2341/4200 | 5|tbar|tpu_64d|b1.5|L8|s0.6|p1.5|sk60 | 0.095 | 3.23 | 0.95 | motor_torque_1p:motor |
| 6" | cfsym | 0/840 | 6|cfsym|sk30|t1.5|L0.6|w1.25|g0.8 | 0.007 | 0.30 | 1.02 | motor_torque_1p:motor |
| 6" | rod | 15796/48384 | 6|rod|tpu_95a|d1.4|n3|sk35|t3.5|hr0.8|L1.25|w0.8 | 0.384 | 18.48 | 0.99 | fatigue_strain:flex0 |
| 6" | strap | 4329/14112 | 6|strap|tpu_64d|d1.2|sk50|t2.5|hr1.2|L0.6|w1.25 | 0.017 | 0.86 | 0.94 | motor_torque_1p:motor |
| 6" | tbar | 776/4200 | 6|tbar|tpu_64d|b1.2|L8|s0.6|p1.5|sk55 | 0.115 | 4.17 | 0.94 | fatigue_strain:flex2 |
| 7" | rod | 14791/47104 | 7|rod|tpu_95a|d1.2|n3|sk35|t2.0|hr0.8|L1.25|w0.8 | 0.325 | 14.57 | 0.99 | fatigue_strain:flex0 |
| 7" | strap | 4659/13824 | 7|strap|tpu_64d|d0.9|sk55|t2.5|hr1.2|L0.6|w1.25 | 0.025 | 1.05 | 0.97 | creep_strain:flex0 |

Top 25 feasible overall:

- 6|rod|tpu_95a|d1.4|n3|sk35|t3.5|hr0.8|L1.25|w0.8: ctrl 0.384, pitch 18.48°, util 0.99 (fatigue_strain:flex0)
- 6|rod|tpu_95a|d1.4|n3|sk40|t4.0|hr0.8|L1.25|w0.8: ctrl 0.380, pitch 18.26°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.4|n3|sk50|t3.5|hr1.0|L1.25|w0.8: ctrl 0.369, pitch 17.36°, util 1.00 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.4|n3|sk55|t3.5|hr1.0|L1.25|w1.0: ctrl 0.365, pitch 17.02°, util 1.00 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.6|n2|sk45|t2.5|hr0.8|L1.25|w1.0: ctrl 0.360, pitch 16.97°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.4|n3|sk45|t2.5|hr0.8|L1.0|w0.8: ctrl 0.358, pitch 16.85°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.4|n3|sk50|t4.0|hr0.8|L1.25|w1.0: ctrl 0.354, pitch 16.58°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.4|n3|sk45|t2.0|hr1.0|L1.25|w0.8: ctrl 0.341, pitch 16.27°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.4|n3|sk40|t2.0|hr0.8|L1.0|w0.8: ctrl 0.340, pitch 16.47°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.4|n3|sk35|t2.5|hr0.8|L1.25|w1.25: ctrl 0.339, pitch 16.28°, util 0.98 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.6|n2|sk50|t2.0|hr0.8|L1.0|w1.0: ctrl 0.337, pitch 15.82°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_64d|d1.4|n3|sk50|t2.5|hr0.8|L1.0|w0.8: ctrl 0.336, pitch 15.72°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.4|n3|sk50|t1.5|hr0.8|L1.0|w1.25: ctrl 0.336, pitch 15.75°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.4|n2|sk45|t4.0|hr1.0|L1.25|w1.0: ctrl 0.335, pitch 15.86°, util 1.00 (fatigue_strain:flex0)
- 6|rod|tpu_95a|d1.4|n3|sk50|t3.0|hr0.8|L1.0|w0.8: ctrl 0.333, pitch 15.63°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.6|n2|sk45|t3.0|hr0.8|L1.25|w0.8: ctrl 0.329, pitch 15.50°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.6|n2|sk45|t3.0|hr0.8|L1.0|w1.0: ctrl 0.327, pitch 15.65°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.6|n2|sk55|t2.0|hr1.0|L1.0|w0.8: ctrl 0.325, pitch 15.27°, util 1.00 (motor_torque_1p:motor)
- 7|rod|tpu_95a|d1.2|n3|sk35|t2.0|hr0.8|L1.25|w0.8: ctrl 0.325, pitch 14.57°, util 0.99 (fatigue_strain:flex0)
- 6|rod|tpu_95a|d1.6|n2|sk40|t4.0|hr0.8|L1.25|w1.0: ctrl 0.325, pitch 15.57°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.4|n3|sk45|t2.0|hr1.0|L1.25|w1.0: ctrl 0.325, pitch 15.46°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_64d|d1.4|n3|sk45|t1.5|hr1.0|L1.25|w1.0: ctrl 0.325, pitch 15.42°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.6|n2|sk45|t3.0|hr0.8|L1.0|w0.8: ctrl 0.324, pitch 15.51°, util 0.99 (motor_torque_1p:motor)
- 6|rod|tpu_95a|d1.4|n2|sk40|t1.5|hr1.0|L1.25|w0.8: ctrl 0.323, pitch 15.07°, util 0.98 (fatigue_strain:flex0)
- 6|rod|tpu_95a|d1.6|n2|sk45|t3.0|hr0.8|L1.25|w1.0: ctrl 0.323, pitch 15.20°, util 0.99 (motor_torque_1p:motor)

196000 results, Fri Nov 20 20:25:35 2026
