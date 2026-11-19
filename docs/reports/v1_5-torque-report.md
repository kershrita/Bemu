# Version 1.5 (CAD masses, centres of mass and inertias) torque report

Generated 2026-11-19 at commit `3f05dec` by `tools/torque_report.py`.

## Method

Each scenario runs the full MuJoCo robot with the walker, with the actuator torque clip raised to the servo's stall torque (2.94 N·m) instead of the 1.47 N·m design limit, so the peaks show what the motion demands. Torque and speed statistics are taken at every 2 ms physics step. Peak torque and speed cover the whole run; RMS torque, power, stability margin and slip cover the steady part after a warm-up walk. Speeds are fractions of each gait's top speed.

Rule: a row passes when its worst peak is under 50% of stall (1.47 N·m) and its worst RMS is under the rated torque (0.98 N·m). `UNSTABLE` marks a stability margin of 15 mm or less.

Power estimate inputs: `stall_current_a` = 2.7 A and `idle_current_a` = 0.03 A are from the Feetech ST-3215-C047 (12 V) datasheet; `supply_voltage_v` = 12.0 V is a placeholder (the servo's nominal voltage; the real battery voltage will move the figures). The estimate assumes supply current equals winding current, so it is an upper bound. Torque, speed and the verdicts do not depend on any of these.

## Summary

48 of 48 rows pass the peak and RMS rules and keep a stability margin above 15 mm.
- Largest peak: 1.26 N·m (43% of stall), wave sideways at 2.5 kg.
- Largest RMS: 0.42 N·m (43% of rated), tripod sideways at 2.5 kg.
- Rows with steps over the design limit: 0.
- 0 unstable rows.

## 2.0 kg

| Scenario | Speed | Peak (joint) | Peak % stall | RMS (joint) | RMS % rated | Max speed % | Power W | Steps at limit | Min margin mm | Slip mm | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | 0 | 0.55 N·m (lm_femur_joint) | 19% | 0.16 N·m (lm_femur_joint) | 16% | 22% | 20 | 0 | - | - | PASS |
| tripod forward 25% | 30 mm/s | 0.51 N·m (lm_femur_joint) | 17% | 0.30 N·m (rm_femur_joint) | 31% | 24% | 26 | 0 | 89 | 2.6 | PASS |
| tripod forward 50% | 60 mm/s | 0.84 N·m (lf_femur_joint) | 29% | 0.33 N·m (rm_femur_joint) | 33% | 49% | 31 | 0 | 83 | 2.8 | PASS |
| tripod forward 100% | 120 mm/s | 0.90 N·m (lf_tibia_joint) | 31% | 0.33 N·m (rm_femur_joint) | 33% | 58% | 36 | 0 | 70 | 3.4 | PASS |
| tripod sideways | 120 mm/s | 1.06 N·m (rm_tibia_joint) | 36% | 0.34 N·m (lm_femur_joint) | 35% | 70% | 35 | 0 | 81 | 5.4 | PASS |
| tripod turn | 0.51 rad/s | 0.92 N·m (rf_femur_joint) | 31% | 0.33 N·m (lm_femur_joint) | 33% | 52% | 34 | 0 | 93 | 2.9 | PASS |
| ripple forward 25% | 15 mm/s | 0.48 N·m (rr_femur_joint) | 16% | 0.22 N·m (lr_femur_joint) | 23% | 24% | 25 | 0 | 92 | 2.1 | PASS |
| ripple forward 50% | 30 mm/s | 0.86 N·m (rf_femur_joint) | 29% | 0.24 N·m (rr_femur_joint) | 25% | 48% | 28 | 0 | 89 | 2.7 | PASS |
| ripple forward 100% | 60 mm/s | 0.95 N·m (lr_tibia_joint) | 32% | 0.25 N·m (lf_femur_joint) | 25% | 62% | 31 | 0 | 83 | 3.7 | PASS |
| ripple sideways | 60 mm/s | 1.11 N·m (rm_tibia_joint) | 38% | 0.26 N·m (lm_femur_joint) | 26% | 73% | 31 | 0 | 87 | 4.7 | PASS |
| ripple turn | 0.26 rad/s | 1.22 N·m (lr_femur_joint) | 42% | 0.24 N·m (lm_femur_joint) | 25% | 62% | 30 | 0 | 83 | 2.2 | PASS |
| wave forward 25% | 6 mm/s | 0.85 N·m (lr_femur_joint) | 29% | 0.20 N·m (lm_femur_joint) | 21% | 36% | 22 | 0 | 97 | 2.6 | PASS |
| wave forward 50% | 12 mm/s | 0.98 N·m (lr_femur_joint) | 33% | 0.22 N·m (rm_femur_joint) | 22% | 56% | 24 | 0 | 98 | 3.1 | PASS |
| wave forward 100% | 24 mm/s | 1.06 N·m (rf_tibia_joint) | 36% | 0.22 N·m (rm_femur_joint) | 22% | 68% | 25 | 0 | 102 | 3.5 | PASS |
| wave sideways | 24 mm/s | 1.26 N·m (rm_tibia_joint) | 43% | 0.24 N·m (rf_femur_joint) | 24% | 83% | 25 | 0 | 87 | 5.0 | PASS |
| wave turn | 0.10 rad/s | 1.03 N·m (lr_femur_joint) | 35% | 0.23 N·m (lm_femur_joint) | 24% | 51% | 25 | 0 | 88 | 3.0 | PASS |

## 2.2 kg

| Scenario | Speed | Peak (joint) | Peak % stall | RMS (joint) | RMS % rated | Max speed % | Power W | Steps at limit | Min margin mm | Slip mm | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | 0 | 0.57 N·m (lm_femur_joint) | 19% | 0.18 N·m (lm_femur_joint) | 19% | 22% | 22 | 0 | - | - | PASS |
| tripod forward 25% | 30 mm/s | 0.57 N·m (lm_femur_joint) | 19% | 0.33 N·m (rm_femur_joint) | 34% | 24% | 28 | 0 | 89 | 2.9 | PASS |
| tripod forward 50% | 60 mm/s | 0.84 N·m (lf_femur_joint) | 29% | 0.36 N·m (rm_femur_joint) | 36% | 49% | 32 | 0 | 83 | 3.0 | PASS |
| tripod forward 100% | 120 mm/s | 0.90 N·m (lf_tibia_joint) | 31% | 0.36 N·m (rm_femur_joint) | 37% | 58% | 37 | 0 | 70 | 3.7 | PASS |
| tripod sideways | 120 mm/s | 1.06 N·m (rm_tibia_joint) | 36% | 0.37 N·m (lm_femur_joint) | 38% | 70% | 37 | 0 | 81 | 5.6 | PASS |
| tripod turn | 0.51 rad/s | 0.92 N·m (rf_femur_joint) | 31% | 0.36 N·m (lm_femur_joint) | 36% | 52% | 35 | 0 | 93 | 3.3 | PASS |
| ripple forward 25% | 15 mm/s | 0.48 N·m (rr_femur_joint) | 16% | 0.25 N·m (lr_femur_joint) | 25% | 24% | 27 | 0 | 92 | 2.3 | PASS |
| ripple forward 50% | 30 mm/s | 0.85 N·m (rf_femur_joint) | 29% | 0.27 N·m (rr_femur_joint) | 27% | 48% | 29 | 0 | 89 | 2.9 | PASS |
| ripple forward 100% | 60 mm/s | 0.95 N·m (lr_tibia_joint) | 32% | 0.27 N·m (lf_femur_joint) | 28% | 62% | 33 | 0 | 83 | 3.8 | PASS |
| ripple sideways | 60 mm/s | 1.11 N·m (rm_tibia_joint) | 38% | 0.28 N·m (lm_femur_joint) | 29% | 73% | 32 | 0 | 87 | 4.7 | PASS |
| ripple turn | 0.26 rad/s | 1.22 N·m (lr_femur_joint) | 42% | 0.27 N·m (rm_femur_joint) | 27% | 62% | 32 | 0 | 83 | 2.4 | PASS |
| wave forward 25% | 6 mm/s | 0.85 N·m (lr_femur_joint) | 29% | 0.23 N·m (lm_femur_joint) | 24% | 36% | 24 | 0 | 97 | 2.8 | PASS |
| wave forward 50% | 12 mm/s | 0.98 N·m (lr_femur_joint) | 33% | 0.24 N·m (lm_femur_joint) | 25% | 56% | 25 | 0 | 98 | 3.3 | PASS |
| wave forward 100% | 24 mm/s | 1.06 N·m (rf_tibia_joint) | 36% | 0.24 N·m (lm_femur_joint) | 25% | 68% | 27 | 0 | 102 | 3.8 | PASS |
| wave sideways | 24 mm/s | 1.26 N·m (rm_tibia_joint) | 43% | 0.26 N·m (rf_femur_joint) | 27% | 83% | 27 | 0 | 87 | 5.2 | PASS |
| wave turn | 0.10 rad/s | 1.01 N·m (lr_femur_joint) | 34% | 0.26 N·m (lm_femur_joint) | 26% | 51% | 27 | 0 | 88 | 3.3 | PASS |

## 2.5 kg

| Scenario | Speed | Peak (joint) | Peak % stall | RMS (joint) | RMS % rated | Max speed % | Power W | Steps at limit | Min margin mm | Slip mm | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | 0 | 0.61 N·m (lm_femur_joint) | 21% | 0.22 N·m (lm_femur_joint) | 23% | 22% | 25 | 0 | - | - | PASS |
| tripod forward 25% | 30 mm/s | 0.65 N·m (lm_femur_joint) | 22% | 0.38 N·m (rm_femur_joint) | 39% | 24% | 30 | 0 | 89 | 3.3 | PASS |
| tripod forward 50% | 60 mm/s | 0.89 N·m (rm_tibia_joint) | 30% | 0.40 N·m (rm_femur_joint) | 41% | 49% | 34 | 0 | 83 | 3.4 | PASS |
| tripod forward 100% | 120 mm/s | 0.92 N·m (rm_tibia_joint) | 31% | 0.41 N·m (rm_femur_joint) | 42% | 58% | 39 | 0 | 70 | 3.9 | PASS |
| tripod sideways | 120 mm/s | 1.06 N·m (rm_tibia_joint) | 36% | 0.42 N·m (lm_femur_joint) | 43% | 70% | 39 | 0 | 81 | 5.9 | PASS |
| tripod turn | 0.51 rad/s | 0.92 N·m (rf_femur_joint) | 31% | 0.40 N·m (lm_femur_joint) | 41% | 52% | 38 | 0 | 94 | 3.7 | PASS |
| ripple forward 25% | 15 mm/s | 0.48 N·m (rr_tibia_joint) | 16% | 0.28 N·m (lr_femur_joint) | 29% | 24% | 29 | 0 | 92 | 2.5 | PASS |
| ripple forward 50% | 30 mm/s | 0.85 N·m (lf_femur_joint) | 29% | 0.30 N·m (rr_femur_joint) | 31% | 48% | 32 | 0 | 89 | 3.1 | PASS |
| ripple forward 100% | 60 mm/s | 0.95 N·m (lr_tibia_joint) | 32% | 0.31 N·m (lf_femur_joint) | 32% | 62% | 35 | 0 | 83 | 4.2 | PASS |
| ripple sideways | 60 mm/s | 1.11 N·m (rm_tibia_joint) | 38% | 0.32 N·m (lm_femur_joint) | 32% | 73% | 35 | 0 | 87 | 5.0 | PASS |
| ripple turn | 0.26 rad/s | 1.22 N·m (lr_femur_joint) | 42% | 0.30 N·m (lm_femur_joint) | 31% | 62% | 35 | 0 | 83 | 2.6 | PASS |
| wave forward 25% | 6 mm/s | 0.85 N·m (lr_femur_joint) | 29% | 0.27 N·m (lm_femur_joint) | 28% | 35% | 27 | 0 | 97 | 3.1 | PASS |
| wave forward 50% | 12 mm/s | 0.98 N·m (lr_femur_joint) | 33% | 0.28 N·m (lm_femur_joint) | 29% | 56% | 28 | 0 | 98 | 3.6 | PASS |
| wave forward 100% | 24 mm/s | 1.06 N·m (rf_tibia_joint) | 36% | 0.28 N·m (lm_femur_joint) | 29% | 68% | 29 | 0 | 102 | 4.1 | PASS |
| wave sideways | 24 mm/s | 1.26 N·m (rm_tibia_joint) | 43% | 0.30 N·m (rf_femur_joint) | 31% | 83% | 30 | 0 | 86 | 5.4 | PASS |
| wave turn | 0.10 rad/s | 1.01 N·m (lr_femur_joint) | 34% | 0.30 N·m (lm_femur_joint) | 31% | 51% | 30 | 0 | 88 | 3.7 | PASS |

## Per-joint torque: tripod forward at top speed, 2.5 kg

| Joint | Peak N·m | RMS N·m |
|---|---|---|
| lf_coxa_joint | 0.402 | 0.125 |
| lf_femur_joint | 0.883 | 0.302 |
| lf_tibia_joint | 0.900 | 0.190 |
| lm_coxa_joint | 0.607 | 0.180 |
| lm_femur_joint | 0.830 | 0.407 |
| lm_tibia_joint | 0.920 | 0.143 |
| lr_coxa_joint | 0.429 | 0.120 |
| lr_femur_joint | 0.878 | 0.285 |
| lr_tibia_joint | 0.914 | 0.235 |
| rf_coxa_joint | 0.393 | 0.125 |
| rf_femur_joint | 0.880 | 0.302 |
| rf_tibia_joint | 0.897 | 0.190 |
| rm_coxa_joint | 0.607 | 0.180 |
| rm_femur_joint | 0.830 | 0.407 |
| rm_tibia_joint | 0.920 | 0.143 |
| rr_coxa_joint | 0.429 | 0.120 |
| rr_femur_joint | 0.876 | 0.285 |
| rr_tibia_joint | 0.909 | 0.235 |

## Per-joint torque: ripple forward at top speed, 2.5 kg

| Joint | Peak N·m | RMS N·m |
|---|---|---|
| lf_coxa_joint | 0.389 | 0.093 |
| lf_femur_joint | 0.871 | 0.309 |
| lf_tibia_joint | 0.891 | 0.175 |
| lm_coxa_joint | 0.534 | 0.113 |
| lm_femur_joint | 0.832 | 0.305 |
| lm_tibia_joint | 0.719 | 0.120 |
| lr_coxa_joint | 0.412 | 0.085 |
| lr_femur_joint | 0.890 | 0.297 |
| lr_tibia_joint | 0.946 | 0.169 |
| rf_coxa_joint | 0.439 | 0.090 |
| rf_femur_joint | 0.886 | 0.304 |
| rf_tibia_joint | 0.930 | 0.174 |
| rm_coxa_joint | 0.539 | 0.115 |
| rm_femur_joint | 0.831 | 0.286 |
| rm_tibia_joint | 0.805 | 0.118 |
| rr_coxa_joint | 0.384 | 0.092 |
| rr_femur_joint | 0.869 | 0.296 |
| rr_tibia_joint | 0.889 | 0.166 |

## Per-joint torque: wave forward at top speed, 2.5 kg

| Joint | Peak N·m | RMS N·m |
|---|---|---|
| lf_coxa_joint | 0.397 | 0.060 |
| lf_femur_joint | 0.873 | 0.255 |
| lf_tibia_joint | 0.894 | 0.123 |
| lm_coxa_joint | 0.512 | 0.072 |
| lm_femur_joint | 0.824 | 0.282 |
| lm_tibia_joint | 0.775 | 0.097 |
| lr_coxa_joint | 0.394 | 0.059 |
| lr_femur_joint | 0.984 | 0.261 |
| lr_tibia_joint | 0.881 | 0.107 |
| rf_coxa_joint | 0.657 | 0.060 |
| rf_femur_joint | 0.921 | 0.255 |
| rf_tibia_joint | 1.060 | 0.123 |
| rm_coxa_joint | 0.607 | 0.072 |
| rm_femur_joint | 0.828 | 0.282 |
| rm_tibia_joint | 0.776 | 0.097 |
| rr_coxa_joint | 0.394 | 0.059 |
| rr_femur_joint | 0.872 | 0.261 |
| rr_tibia_joint | 0.883 | 0.107 |

