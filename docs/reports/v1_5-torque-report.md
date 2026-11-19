# Version 1.5 (CAD masses and body) torque report

Generated 2026-11-19 at commit `3266571` by `tools/torque_report.py`.

## Method

Each scenario runs the full MuJoCo robot with the walker, with the actuator torque clip raised to the servo's stall torque (2.94 N·m) instead of the 1.47 N·m design limit, so the peaks show what the motion demands. Torque and speed statistics are taken at every 2 ms physics step. Peak torque and speed cover the whole run; RMS torque, power, stability margin and slip cover the steady part after a warm-up walk. Speeds are fractions of each gait's top speed.

Rule: a row passes when its worst peak is under 50% of stall (1.47 N·m) and its worst RMS is under the rated torque (0.98 N·m). `UNSTABLE` marks a stability margin of 15 mm or less.

Power estimate inputs: `stall_current_a` = 2.7 A and `idle_current_a` = 0.03 A are from the Feetech ST-3215-C047 (12 V) datasheet; `supply_voltage_v` = 12.0 V is a placeholder (the servo's nominal voltage; the real battery voltage will move the figures). The estimate assumes supply current equals winding current, so it is an upper bound. Torque, speed and the verdicts do not depend on any of these.

## Summary

48 of 48 rows pass the peak and RMS rules and keep a stability margin above 15 mm.
- Largest peak: 1.27 N·m (43% of stall), wave sideways at 2.5 kg.
- Largest RMS: 0.44 N·m (45% of rated), tripod sideways at 2.5 kg.
- Rows with steps over the design limit: 0.
- 0 unstable rows.

## 2.0 kg

| Scenario | Speed | Peak (joint) | Peak % stall | RMS (joint) | RMS % rated | Max speed % | Power W | Steps at limit | Min margin mm | Slip mm | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | 0 | 0.57 N·m (lm_femur_joint) | 19% | 0.19 N·m (rm_femur_joint) | 20% | 22% | 22 | 0 | - | - | PASS |
| tripod forward 25% | 30 mm/s | 0.54 N·m (lm_femur_joint) | 18% | 0.32 N·m (rm_femur_joint) | 33% | 24% | 27 | 0 | 89 | 2.7 | PASS |
| tripod forward 50% | 60 mm/s | 0.82 N·m (lf_femur_joint) | 28% | 0.35 N·m (rm_femur_joint) | 35% | 49% | 31 | 0 | 83 | 2.8 | PASS |
| tripod forward 100% | 120 mm/s | 0.90 N·m (lf_tibia_joint) | 31% | 0.35 N·m (lm_femur_joint) | 35% | 58% | 36 | 0 | 70 | 3.4 | PASS |
| tripod sideways | 120 mm/s | 1.07 N·m (rm_tibia_joint) | 36% | 0.36 N·m (lm_femur_joint) | 37% | 70% | 35 | 0 | 81 | 5.4 | PASS |
| tripod turn | 0.51 rad/s | 0.90 N·m (rf_femur_joint) | 31% | 0.35 N·m (rm_femur_joint) | 35% | 52% | 34 | 0 | 94 | 3.0 | PASS |
| ripple forward 25% | 15 mm/s | 0.45 N·m (rr_tibia_joint) | 15% | 0.25 N·m (rr_femur_joint) | 25% | 24% | 26 | 0 | 92 | 2.1 | PASS |
| ripple forward 50% | 30 mm/s | 0.83 N·m (rf_femur_joint) | 28% | 0.27 N·m (rr_femur_joint) | 27% | 49% | 29 | 0 | 89 | 2.7 | PASS |
| ripple forward 100% | 60 mm/s | 0.95 N·m (lr_tibia_joint) | 32% | 0.27 N·m (lf_femur_joint) | 28% | 62% | 32 | 0 | 83 | 3.7 | PASS |
| ripple sideways | 60 mm/s | 1.12 N·m (rm_tibia_joint) | 38% | 0.28 N·m (lm_femur_joint) | 28% | 73% | 32 | 0 | 87 | 4.6 | PASS |
| ripple turn | 0.26 rad/s | 1.20 N·m (lr_femur_joint) | 41% | 0.27 N·m (lm_femur_joint) | 27% | 63% | 31 | 0 | 83 | 2.2 | PASS |
| wave forward 25% | 6 mm/s | 0.82 N·m (lr_femur_joint) | 28% | 0.23 N·m (lm_femur_joint) | 24% | 37% | 24 | 0 | 97 | 2.6 | PASS |
| wave forward 50% | 12 mm/s | 0.97 N·m (lr_femur_joint) | 33% | 0.25 N·m (lm_femur_joint) | 25% | 56% | 25 | 0 | 98 | 3.1 | PASS |
| wave forward 100% | 24 mm/s | 1.06 N·m (rf_tibia_joint) | 36% | 0.24 N·m (lm_femur_joint) | 25% | 69% | 27 | 0 | 102 | 3.5 | PASS |
| wave sideways | 24 mm/s | 1.27 N·m (rm_tibia_joint) | 43% | 0.26 N·m (rf_femur_joint) | 27% | 83% | 27 | 0 | 86 | 5.0 | PASS |
| wave turn | 0.10 rad/s | 0.99 N·m (lr_femur_joint) | 34% | 0.26 N·m (lm_femur_joint) | 27% | 52% | 27 | 0 | 87 | 3.0 | PASS |

## 2.2 kg

| Scenario | Speed | Peak (joint) | Peak % stall | RMS (joint) | RMS % rated | Max speed % | Power W | Steps at limit | Min margin mm | Slip mm | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | 0 | 0.60 N·m (rm_femur_joint) | 20% | 0.22 N·m (rm_femur_joint) | 22% | 22% | 24 | 0 | - | - | PASS |
| tripod forward 25% | 30 mm/s | 0.60 N·m (lm_femur_joint) | 20% | 0.36 N·m (rm_femur_joint) | 36% | 24% | 28 | 0 | 89 | 2.9 | PASS |
| tripod forward 50% | 60 mm/s | 0.85 N·m (rm_tibia_joint) | 29% | 0.38 N·m (rm_femur_joint) | 38% | 49% | 32 | 0 | 83 | 3.0 | PASS |
| tripod forward 100% | 120 mm/s | 0.90 N·m (lf_tibia_joint) | 31% | 0.38 N·m (lm_femur_joint) | 39% | 58% | 37 | 0 | 70 | 3.6 | PASS |
| tripod sideways | 120 mm/s | 1.07 N·m (rm_tibia_joint) | 36% | 0.39 N·m (lm_femur_joint) | 40% | 70% | 37 | 0 | 81 | 5.6 | PASS |
| tripod turn | 0.51 rad/s | 0.90 N·m (rf_femur_joint) | 31% | 0.38 N·m (lm_femur_joint) | 39% | 52% | 35 | 0 | 94 | 3.3 | PASS |
| ripple forward 25% | 15 mm/s | 0.47 N·m (rr_tibia_joint) | 16% | 0.27 N·m (lr_femur_joint) | 28% | 24% | 28 | 0 | 92 | 2.3 | PASS |
| ripple forward 50% | 30 mm/s | 0.82 N·m (lf_femur_joint) | 28% | 0.29 N·m (rr_femur_joint) | 30% | 49% | 30 | 0 | 89 | 2.9 | PASS |
| ripple forward 100% | 60 mm/s | 0.95 N·m (lr_tibia_joint) | 32% | 0.30 N·m (lf_femur_joint) | 30% | 62% | 34 | 0 | 82 | 3.9 | PASS |
| ripple sideways | 60 mm/s | 1.12 N·m (rm_tibia_joint) | 38% | 0.30 N·m (lm_femur_joint) | 31% | 73% | 33 | 0 | 87 | 4.7 | PASS |
| ripple turn | 0.26 rad/s | 1.20 N·m (lr_femur_joint) | 41% | 0.29 N·m (lm_femur_joint) | 30% | 63% | 33 | 0 | 83 | 2.4 | PASS |
| wave forward 25% | 6 mm/s | 0.82 N·m (lr_femur_joint) | 28% | 0.26 N·m (lm_femur_joint) | 26% | 37% | 26 | 0 | 97 | 2.8 | PASS |
| wave forward 50% | 12 mm/s | 0.97 N·m (lr_femur_joint) | 33% | 0.27 N·m (lm_femur_joint) | 28% | 56% | 27 | 0 | 98 | 3.3 | PASS |
| wave forward 100% | 24 mm/s | 1.06 N·m (rf_tibia_joint) | 36% | 0.27 N·m (lm_femur_joint) | 28% | 69% | 28 | 0 | 102 | 3.8 | PASS |
| wave sideways | 24 mm/s | 1.27 N·m (rm_tibia_joint) | 43% | 0.29 N·m (rf_femur_joint) | 29% | 83% | 29 | 0 | 86 | 5.2 | PASS |
| wave turn | 0.10 rad/s | 0.98 N·m (lr_femur_joint) | 33% | 0.29 N·m (lm_femur_joint) | 29% | 52% | 29 | 0 | 87 | 3.3 | PASS |

## 2.5 kg

| Scenario | Speed | Peak (joint) | Peak % stall | RMS (joint) | RMS % rated | Max speed % | Power W | Steps at limit | Min margin mm | Slip mm | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | 0 | 0.63 N·m (lm_femur_joint) | 22% | 0.26 N·m (rm_femur_joint) | 26% | 22% | 27 | 0 | - | - | PASS |
| tripod forward 25% | 30 mm/s | 0.68 N·m (lm_femur_joint) | 23% | 0.40 N·m (rm_femur_joint) | 41% | 24% | 31 | 0 | 89 | 3.3 | PASS |
| tripod forward 50% | 60 mm/s | 0.90 N·m (rm_tibia_joint) | 31% | 0.43 N·m (rm_femur_joint) | 43% | 49% | 35 | 0 | 83 | 3.5 | PASS |
| tripod forward 100% | 120 mm/s | 0.92 N·m (rm_tibia_joint) | 31% | 0.43 N·m (lm_femur_joint) | 44% | 58% | 40 | 0 | 70 | 3.9 | PASS |
| tripod sideways | 120 mm/s | 1.07 N·m (rm_tibia_joint) | 36% | 0.44 N·m (lm_femur_joint) | 45% | 70% | 39 | 0 | 82 | 5.9 | PASS |
| tripod turn | 0.51 rad/s | 0.91 N·m (rm_tibia_joint) | 31% | 0.43 N·m (rm_femur_joint) | 43% | 52% | 38 | 0 | 94 | 3.8 | PASS |
| ripple forward 25% | 15 mm/s | 0.49 N·m (rr_tibia_joint) | 17% | 0.31 N·m (rr_femur_joint) | 31% | 24% | 30 | 0 | 92 | 2.5 | PASS |
| ripple forward 50% | 30 mm/s | 0.83 N·m (lf_femur_joint) | 28% | 0.33 N·m (rr_femur_joint) | 33% | 49% | 33 | 0 | 89 | 3.2 | PASS |
| ripple forward 100% | 60 mm/s | 0.95 N·m (lr_tibia_joint) | 32% | 0.33 N·m (lf_femur_joint) | 34% | 62% | 36 | 0 | 82 | 4.2 | PASS |
| ripple sideways | 60 mm/s | 1.12 N·m (rm_tibia_joint) | 38% | 0.34 N·m (lm_femur_joint) | 35% | 73% | 36 | 0 | 87 | 5.0 | PASS |
| ripple turn | 0.26 rad/s | 1.20 N·m (lr_femur_joint) | 41% | 0.33 N·m (lm_femur_joint) | 34% | 63% | 36 | 0 | 83 | 2.6 | PASS |
| wave forward 25% | 6 mm/s | 0.83 N·m (lr_femur_joint) | 28% | 0.30 N·m (lm_femur_joint) | 31% | 36% | 29 | 0 | 97 | 3.1 | PASS |
| wave forward 50% | 12 mm/s | 0.97 N·m (lr_femur_joint) | 33% | 0.31 N·m (lm_femur_joint) | 32% | 56% | 30 | 0 | 98 | 3.6 | PASS |
| wave forward 100% | 24 mm/s | 1.06 N·m (rf_tibia_joint) | 36% | 0.31 N·m (lm_femur_joint) | 32% | 69% | 31 | 0 | 102 | 4.1 | PASS |
| wave sideways | 24 mm/s | 1.27 N·m (rm_tibia_joint) | 43% | 0.33 N·m (rf_femur_joint) | 33% | 83% | 32 | 0 | 86 | 5.5 | PASS |
| wave turn | 0.10 rad/s | 0.99 N·m (lr_tibia_joint) | 34% | 0.33 N·m (lm_femur_joint) | 34% | 52% | 32 | 0 | 87 | 3.7 | PASS |

## Per-joint torque: tripod forward at top speed, 2.5 kg

| Joint | Peak N·m | RMS N·m |
|---|---|---|
| lf_coxa_joint | 0.421 | 0.124 |
| lf_femur_joint | 0.857 | 0.322 |
| lf_tibia_joint | 0.899 | 0.190 |
| lm_coxa_joint | 0.576 | 0.174 |
| lm_femur_joint | 0.804 | 0.427 |
| lm_tibia_joint | 0.920 | 0.142 |
| lr_coxa_joint | 0.413 | 0.118 |
| lr_femur_joint | 0.849 | 0.302 |
| lr_tibia_joint | 0.921 | 0.232 |
| rf_coxa_joint | 0.411 | 0.124 |
| rf_femur_joint | 0.854 | 0.322 |
| rf_tibia_joint | 0.896 | 0.190 |
| rm_coxa_joint | 0.572 | 0.174 |
| rm_femur_joint | 0.804 | 0.427 |
| rm_tibia_joint | 0.924 | 0.142 |
| rr_coxa_joint | 0.410 | 0.118 |
| rr_femur_joint | 0.846 | 0.302 |
| rr_tibia_joint | 0.915 | 0.232 |

## Per-joint torque: ripple forward at top speed, 2.5 kg

| Joint | Peak N·m | RMS N·m |
|---|---|---|
| lf_coxa_joint | 0.408 | 0.093 |
| lf_femur_joint | 0.848 | 0.335 |
| lf_tibia_joint | 0.891 | 0.177 |
| lm_coxa_joint | 0.563 | 0.112 |
| lm_femur_joint | 0.807 | 0.331 |
| lm_tibia_joint | 0.722 | 0.121 |
| lr_coxa_joint | 0.434 | 0.084 |
| lr_femur_joint | 0.863 | 0.321 |
| lr_tibia_joint | 0.952 | 0.168 |
| rf_coxa_joint | 0.462 | 0.090 |
| rf_femur_joint | 0.860 | 0.329 |
| rf_tibia_joint | 0.930 | 0.176 |
| rm_coxa_joint | 0.567 | 0.116 |
| rm_femur_joint | 0.807 | 0.310 |
| rm_tibia_joint | 0.808 | 0.119 |
| rr_coxa_joint | 0.405 | 0.090 |
| rr_femur_joint | 0.843 | 0.322 |
| rr_tibia_joint | 0.896 | 0.163 |

## Per-joint torque: wave forward at top speed, 2.5 kg

| Joint | Peak N·m | RMS N·m |
|---|---|---|
| lf_coxa_joint | 0.414 | 0.059 |
| lf_femur_joint | 0.849 | 0.283 |
| lf_tibia_joint | 0.893 | 0.124 |
| lm_coxa_joint | 0.537 | 0.072 |
| lm_femur_joint | 0.798 | 0.311 |
| lm_tibia_joint | 0.784 | 0.098 |
| lr_coxa_joint | 0.412 | 0.059 |
| lr_femur_joint | 0.969 | 0.288 |
| lr_tibia_joint | 0.886 | 0.107 |
| rf_coxa_joint | 0.686 | 0.059 |
| rf_femur_joint | 0.899 | 0.283 |
| rf_tibia_joint | 1.061 | 0.124 |
| rm_coxa_joint | 0.636 | 0.072 |
| rm_femur_joint | 0.802 | 0.311 |
| rm_tibia_joint | 0.784 | 0.098 |
| rr_coxa_joint | 0.412 | 0.059 |
| rr_femur_joint | 0.843 | 0.288 |
| rr_tibia_joint | 0.888 | 0.107 |

