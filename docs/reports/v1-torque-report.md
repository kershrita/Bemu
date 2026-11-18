# Version 1 torque report

Generated 2026-11-18 at commit `e400b57` by `tools/torque_report.py`.

## Method

Each scenario runs the full MuJoCo robot with the walker, with the actuator torque clip raised to the servo's stall torque (2.94 N·m) instead of the 1.47 N·m design limit, so the peaks show what the motion demands. Torque and speed statistics are taken at every 2 ms physics step. Peak torque and speed cover the whole run; RMS torque, power, stability margin and slip cover the steady part after a warm-up walk. Speeds are fractions of each gait's top speed.

Rule: a row passes when its worst peak is under 50% of stall (1.47 N·m) and its worst RMS is under the rated torque (0.98 N·m). `UNSTABLE` marks a stability margin of 15 mm or less.

Placeholder constants (not datasheet values, to be confirmed on the real servo): `stall_current_a` = 2.7 A, `idle_current_a` = 0.1 A, `supply_voltage_v` = 12.0 V. They only feed the power estimate; torque, speed and the verdicts do not depend on them.

## Summary

48 of 48 rows pass the peak and RMS rules.
- Largest peak: 1.26 N·m (43% of stall), wave sideways at 2.5 kg.
- Largest RMS: 0.43 N·m (44% of rated), tripod sideways at 2.5 kg.
- Rows at the actuator limit: 0.
- Unstable rows: 0.

## 2.0 kg

| Scenario | Speed | Peak (joint) | Peak % stall | RMS (joint) | RMS % rated | Max speed % | Power W | Steps at limit | Min margin mm | Slip mm | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | 0 | 0.57 N·m (lm_femur_joint) | 19% | 0.19 N·m (rm_femur_joint) | 19% | 22% | 36 | 0 | - | - | PASS |
| tripod forward 25% | 30 mm/s | 0.52 N·m (lm_femur_joint) | 18% | 0.31 N·m (rm_femur_joint) | 32% | 24% | 42 | 0 | 89 | 2.6 | PASS |
| tripod forward 50% | 60 mm/s | 0.82 N·m (lf_femur_joint) | 28% | 0.33 N·m (rm_femur_joint) | 34% | 49% | 46 | 0 | 83 | 2.8 | PASS |
| tripod forward 100% | 120 mm/s | 0.90 N·m (lf_tibia_joint) | 31% | 0.34 N·m (lm_femur_joint) | 34% | 58% | 51 | 0 | 70 | 3.4 | PASS |
| tripod sideways | 120 mm/s | 1.06 N·m (lm_tibia_joint) | 36% | 0.35 N·m (lm_femur_joint) | 36% | 69% | 50 | 0 | 81 | 5.4 | PASS |
| tripod turn | 0.51 rad/s | 0.91 N·m (rf_femur_joint) | 31% | 0.33 N·m (lm_femur_joint) | 34% | 52% | 49 | 0 | 94 | 2.7 | PASS |
| ripple forward 25% | 15 mm/s | 0.46 N·m (rr_femur_joint) | 16% | 0.24 N·m (lr_femur_joint) | 24% | 24% | 41 | 0 | 92 | 2.1 | PASS |
| ripple forward 50% | 30 mm/s | 0.84 N·m (rf_femur_joint) | 28% | 0.26 N·m (rr_femur_joint) | 26% | 49% | 43 | 0 | 88 | 2.7 | PASS |
| ripple forward 100% | 60 mm/s | 0.94 N·m (lr_tibia_joint) | 32% | 0.26 N·m (lf_femur_joint) | 27% | 61% | 47 | 0 | 82 | 3.7 | PASS |
| ripple sideways | 60 mm/s | 1.11 N·m (rm_tibia_joint) | 38% | 0.27 N·m (lm_femur_joint) | 28% | 73% | 46 | 0 | 87 | 4.7 | PASS |
| ripple turn | 0.26 rad/s | 1.21 N·m (rf_femur_joint) | 41% | 0.26 N·m (lm_femur_joint) | 26% | 63% | 46 | 0 | 83 | 2.2 | PASS |
| wave forward 25% | 6 mm/s | 0.83 N·m (lr_femur_joint) | 28% | 0.22 N·m (lm_femur_joint) | 23% | 37% | 39 | 0 | 97 | 2.6 | PASS |
| wave forward 50% | 12 mm/s | 0.97 N·m (lr_femur_joint) | 33% | 0.24 N·m (lm_femur_joint) | 24% | 56% | 40 | 0 | 98 | 3.0 | PASS |
| wave forward 100% | 24 mm/s | 1.06 N·m (rf_tibia_joint) | 36% | 0.24 N·m (lm_femur_joint) | 24% | 68% | 41 | 0 | 102 | 3.5 | PASS |
| wave sideways | 24 mm/s | 1.26 N·m (rm_tibia_joint) | 43% | 0.25 N·m (rf_femur_joint) | 26% | 83% | 42 | 0 | 86 | 5.1 | PASS |
| wave turn | 0.10 rad/s | 1.00 N·m (lr_femur_joint) | 34% | 0.25 N·m (lm_femur_joint) | 26% | 52% | 42 | 0 | 87 | 3.0 | PASS |

## 2.2 kg

| Scenario | Speed | Peak (joint) | Peak % stall | RMS (joint) | RMS % rated | Max speed % | Power W | Steps at limit | Min margin mm | Slip mm | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | 0 | 0.59 N·m (lm_femur_joint) | 20% | 0.21 N·m (lm_femur_joint) | 22% | 22% | 38 | 0 | - | - | PASS |
| tripod forward 25% | 30 mm/s | 0.58 N·m (lm_femur_joint) | 20% | 0.34 N·m (rm_femur_joint) | 35% | 24% | 43 | 0 | 89 | 2.8 | PASS |
| tripod forward 50% | 60 mm/s | 0.82 N·m (lf_femur_joint) | 28% | 0.36 N·m (rm_femur_joint) | 37% | 49% | 47 | 0 | 83 | 3.0 | PASS |
| tripod forward 100% | 120 mm/s | 0.90 N·m (lf_tibia_joint) | 31% | 0.37 N·m (lm_femur_joint) | 37% | 58% | 52 | 0 | 70 | 3.6 | PASS |
| tripod sideways | 120 mm/s | 1.06 N·m (lm_tibia_joint) | 36% | 0.38 N·m (lm_femur_joint) | 39% | 69% | 52 | 0 | 81 | 5.6 | PASS |
| tripod turn | 0.51 rad/s | 0.91 N·m (rf_femur_joint) | 31% | 0.36 N·m (lm_femur_joint) | 37% | 52% | 50 | 0 | 94 | 3.1 | PASS |
| ripple forward 25% | 15 mm/s | 0.46 N·m (rr_tibia_joint) | 16% | 0.26 N·m (rr_femur_joint) | 27% | 24% | 42 | 0 | 92 | 2.3 | PASS |
| ripple forward 50% | 30 mm/s | 0.83 N·m (lf_femur_joint) | 28% | 0.28 N·m (rr_femur_joint) | 29% | 49% | 45 | 0 | 88 | 2.9 | PASS |
| ripple forward 100% | 60 mm/s | 0.94 N·m (lr_tibia_joint) | 32% | 0.29 N·m (lf_femur_joint) | 29% | 61% | 48 | 0 | 82 | 3.9 | PASS |
| ripple sideways | 60 mm/s | 1.11 N·m (rm_tibia_joint) | 38% | 0.30 N·m (lm_femur_joint) | 30% | 73% | 48 | 0 | 87 | 4.8 | PASS |
| ripple turn | 0.26 rad/s | 1.21 N·m (rf_femur_joint) | 41% | 0.28 N·m (rm_femur_joint) | 29% | 63% | 47 | 0 | 83 | 2.4 | PASS |
| wave forward 25% | 6 mm/s | 0.83 N·m (lr_femur_joint) | 28% | 0.25 N·m (lm_femur_joint) | 26% | 37% | 41 | 0 | 97 | 2.8 | PASS |
| wave forward 50% | 12 mm/s | 0.97 N·m (lr_femur_joint) | 33% | 0.26 N·m (lm_femur_joint) | 27% | 56% | 42 | 0 | 98 | 3.3 | PASS |
| wave forward 100% | 24 mm/s | 1.06 N·m (rf_tibia_joint) | 36% | 0.26 N·m (lm_femur_joint) | 27% | 68% | 43 | 0 | 102 | 3.8 | PASS |
| wave sideways | 24 mm/s | 1.26 N·m (rm_tibia_joint) | 43% | 0.28 N·m (rf_femur_joint) | 28% | 83% | 44 | 0 | 86 | 5.3 | PASS |
| wave turn | 0.10 rad/s | 0.99 N·m (lr_femur_joint) | 34% | 0.28 N·m (lm_femur_joint) | 28% | 52% | 44 | 0 | 87 | 3.3 | PASS |

## 2.5 kg

| Scenario | Speed | Peak (joint) | Peak % stall | RMS (joint) | RMS % rated | Max speed % | Power W | Steps at limit | Min margin mm | Slip mm | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | 0 | 0.63 N·m (lm_femur_joint) | 21% | 0.25 N·m (rm_femur_joint) | 25% | 22% | 42 | 0 | - | - | PASS |
| tripod forward 25% | 30 mm/s | 0.67 N·m (lm_femur_joint) | 23% | 0.39 N·m (rm_femur_joint) | 40% | 24% | 45 | 0 | 89 | 3.1 | PASS |
| tripod forward 50% | 60 mm/s | 0.88 N·m (rm_tibia_joint) | 30% | 0.41 N·m (rm_femur_joint) | 42% | 49% | 49 | 0 | 83 | 3.3 | PASS |
| tripod forward 100% | 120 mm/s | 0.92 N·m (rm_tibia_joint) | 31% | 0.41 N·m (lm_femur_joint) | 42% | 58% | 54 | 0 | 70 | 3.9 | PASS |
| tripod sideways | 120 mm/s | 1.06 N·m (lm_tibia_joint) | 36% | 0.43 N·m (lm_femur_joint) | 44% | 70% | 54 | 0 | 81 | 5.9 | PASS |
| tripod turn | 0.51 rad/s | 0.91 N·m (rf_femur_joint) | 31% | 0.41 N·m (lm_femur_joint) | 42% | 52% | 53 | 0 | 94 | 3.5 | PASS |
| ripple forward 25% | 15 mm/s | 0.48 N·m (rr_tibia_joint) | 16% | 0.30 N·m (rr_femur_joint) | 30% | 24% | 45 | 0 | 92 | 2.5 | PASS |
| ripple forward 50% | 30 mm/s | 0.83 N·m (lf_femur_joint) | 28% | 0.32 N·m (lm_femur_joint) | 32% | 49% | 48 | 0 | 89 | 3.1 | PASS |
| ripple forward 100% | 60 mm/s | 0.94 N·m (lr_tibia_joint) | 32% | 0.33 N·m (lf_femur_joint) | 33% | 61% | 51 | 0 | 82 | 4.1 | PASS |
| ripple sideways | 60 mm/s | 1.11 N·m (rm_tibia_joint) | 38% | 0.33 N·m (lm_femur_joint) | 34% | 73% | 51 | 0 | 87 | 4.9 | PASS |
| ripple turn | 0.26 rad/s | 1.21 N·m (rf_femur_joint) | 41% | 0.32 N·m (lm_femur_joint) | 32% | 63% | 50 | 0 | 83 | 2.6 | PASS |
| wave forward 25% | 6 mm/s | 0.83 N·m (lr_femur_joint) | 28% | 0.29 N·m (lm_femur_joint) | 30% | 36% | 43 | 0 | 97 | 3.1 | PASS |
| wave forward 50% | 12 mm/s | 0.97 N·m (lr_femur_joint) | 33% | 0.30 N·m (lm_femur_joint) | 31% | 56% | 44 | 0 | 98 | 3.6 | PASS |
| wave forward 100% | 24 mm/s | 1.06 N·m (rf_tibia_joint) | 36% | 0.30 N·m (lm_femur_joint) | 31% | 68% | 46 | 0 | 102 | 4.1 | PASS |
| wave sideways | 24 mm/s | 1.26 N·m (rm_tibia_joint) | 43% | 0.32 N·m (lr_femur_joint) | 32% | 83% | 46 | 0 | 86 | 5.5 | PASS |
| wave turn | 0.10 rad/s | 0.99 N·m (lr_femur_joint) | 34% | 0.32 N·m (lm_femur_joint) | 33% | 52% | 47 | 0 | 87 | 3.7 | PASS |

## Per-joint torque: tripod forward at top speed, 2.5 kg

| Joint | Peak N·m | RMS N·m |
|---|---|---|
| lf_coxa_joint | 0.420 | 0.125 |
| lf_femur_joint | 0.861 | 0.313 |
| lf_tibia_joint | 0.898 | 0.188 |
| lm_coxa_joint | 0.581 | 0.176 |
| lm_femur_joint | 0.814 | 0.412 |
| lm_tibia_joint | 0.913 | 0.144 |
| lr_coxa_joint | 0.412 | 0.119 |
| lr_femur_joint | 0.860 | 0.290 |
| lr_tibia_joint | 0.905 | 0.237 |
| rf_coxa_joint | 0.410 | 0.125 |
| rf_femur_joint | 0.859 | 0.313 |
| rf_tibia_joint | 0.896 | 0.188 |
| rm_coxa_joint | 0.578 | 0.176 |
| rm_femur_joint | 0.815 | 0.412 |
| rm_tibia_joint | 0.917 | 0.144 |
| rr_coxa_joint | 0.414 | 0.119 |
| rr_femur_joint | 0.857 | 0.290 |
| rr_tibia_joint | 0.898 | 0.237 |

## Per-joint torque: ripple forward at top speed, 2.5 kg

| Joint | Peak N·m | RMS N·m |
|---|---|---|
| lf_coxa_joint | 0.406 | 0.094 |
| lf_femur_joint | 0.851 | 0.325 |
| lf_tibia_joint | 0.890 | 0.175 |
| lm_coxa_joint | 0.558 | 0.113 |
| lm_femur_joint | 0.814 | 0.321 |
| lm_tibia_joint | 0.714 | 0.119 |
| lr_coxa_joint | 0.429 | 0.085 |
| lr_femur_joint | 0.878 | 0.309 |
| lr_tibia_joint | 0.941 | 0.170 |
| rf_coxa_joint | 0.460 | 0.091 |
| rf_femur_joint | 0.864 | 0.320 |
| rf_tibia_joint | 0.929 | 0.174 |
| rm_coxa_joint | 0.562 | 0.116 |
| rm_femur_joint | 0.814 | 0.299 |
| rm_tibia_joint | 0.802 | 0.119 |
| rr_coxa_joint | 0.401 | 0.091 |
| rr_femur_joint | 0.855 | 0.311 |
| rr_tibia_joint | 0.882 | 0.165 |

## Per-joint torque: wave forward at top speed, 2.5 kg

| Joint | Peak N·m | RMS N·m |
|---|---|---|
| lf_coxa_joint | 0.412 | 0.060 |
| lf_femur_joint | 0.852 | 0.276 |
| lf_tibia_joint | 0.893 | 0.123 |
| lm_coxa_joint | 0.532 | 0.071 |
| lm_femur_joint | 0.807 | 0.302 |
| lm_tibia_joint | 0.772 | 0.096 |
| lr_coxa_joint | 0.408 | 0.060 |
| lr_femur_joint | 0.974 | 0.280 |
| lr_tibia_joint | 0.877 | 0.107 |
| rf_coxa_joint | 0.685 | 0.060 |
| rf_femur_joint | 0.900 | 0.276 |
| rf_tibia_joint | 1.057 | 0.123 |
| rm_coxa_joint | 0.630 | 0.071 |
| rm_femur_joint | 0.811 | 0.302 |
| rm_tibia_joint | 0.778 | 0.096 |
| rr_coxa_joint | 0.408 | 0.060 |
| rr_femur_joint | 0.855 | 0.280 |
| rr_tibia_joint | 0.880 | 0.107 |

