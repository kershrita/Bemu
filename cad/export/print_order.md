# Print order

**Status: NOT READY TO ORDER.** These fit-critical values are not yet `measured` (caliper on the first servo, or the printed coupon): `servo.case_hole_thread`, `servo.case_z_range`, `servo.horn_disc_diameter`, `servo.horn_z_range`, `servo.rear_idler_z_range`, `servo.rear_hub_diameter`, `servo.rear_hub_z_range`, `printing.pocket_allowance`.

Two steps come first (docs/08): print `tolerance_coupon` alone and measure it, and caliper the first servo received (and check which screws come in its accessory bag: the datasheet does not give the bracket-hole thread).

All values still marked unverified in `config/components.yaml` (15): `servo.rear_idler_z_range`, `servo.horn_disc_diameter`, `servo.horn_z_range`, `servo.case_hole_thread`, `printing.pocket_allowance`, `insert_m3.hole_diameter`, `insert_m3.length`, `raspberry_pi_5.board`, `raspberry_pi_5.hole_pattern`, `raspberry_pi_5.hole_diameter`, `raspberry_pi_5.hole_inset`, `raspberry_pi_5.height`, `raspberry_pi_5.pilot_diameter`, `battery.size`, `battery.mass`.

Settings for every part unless stated: 0.2 mm layers, 4 perimeters, 35% infill (docs/08). Quantities are for six legs plus spares; the body rows (plates, posts, battery tray, Pi plate) are the whole body. Hard stops and cable slots come with the first leg (version 2).

| File | Material | Quantity | Orientation | Supports | Note |
|---|---|---|---|---|---|
| tolerance_coupon.stl | PETG | 1 | base down | print this first, alone |
| coxa_bracket.stl | PETG | 7 | base plate down (the leg's vertical axis points up) (+z up) | yes (195 mm²) |  |
| femur_plate_horn.stl | PETG | 7 | lie flat on the large face (the leg's lateral axis points up) (-y up) | no |  |
| femur_plate_back.stl | PETG | 7 | lie flat on the large face (the leg's lateral axis points up) (+y up) | no |  |
| knee_spacer_horn.stl | PETG | 7 | lie flat on the large face (the leg's lateral axis points up) (-y up) | no |  |
| knee_spacer_back.stl | PETG | 7 | lie flat on the large face (the leg's lateral axis points up) (+y up) | no |  |
| tibia_plate_horn.stl | PETG | 7 | lie flat on the large face (the leg's lateral axis points up) (-y up) | yes (36 mm²) |  |
| tibia_plate_back.stl | PETG | 7 | lie flat on the large face (the leg's lateral axis points up) (+y up) | yes (45 mm²) |  |
| foot_bridge.stl | PETG | 7 | base plate down (the leg's vertical axis points up) (-z up) | yes (40 mm²) |  |
| foot.stl | TPU | 8 | as modelled: dome down, stem up; the dome needs a small support (+z up) | yes (99 mm²) | rubber tips may replace the printed foot |
| top_plate_rear.stl | PETG | 1 | flat, large face down (-z up) | no | body |
| top_plate_centre.stl | PETG | 1 | flat, large face down (-z up) | no | body |
| top_plate_front.stl | PETG | 1 | flat, large face down (-z up) | no | body |
| bottom_plate_rear.stl | PETG | 1 | flat, large face down (+z up) | no | body |
| bottom_plate_centre.stl | PETG | 1 | flat, large face down (+z up) | no | body |
| bottom_plate_front.stl | PETG | 1 | flat, large face down (+z up) | no | body |
| hip_plate.stl | PETG | 6 | flat, large face down (+z up) | no | body |
| post_main.stl | PETG | 8 | upright (+z up) | no | body |
| post_hip.stl | PETG | 12 | upright (+z up) | no | body |
| splice_top.stl | PETG | 4 | flat, large face down (+z up) | no | body |
| splice_bottom.stl | PETG | 4 | flat, large face down (+z up) | no | body |
| battery_tray.stl | PETG | 1 | flat, large face down (+z up) | yes (1224 mm²) | body |
| pi_plate.stl | PETG | 1 | flat, large face down (+z up) | no | body |

## Hardware (buy with the parts)

Screw lengths are computed from the stack heights plus 3 mm of thread engagement and rounded up to a standard ISO 4762 length. They assume the horn holes and the servo's bracket holes are as in the STEP models; **check against the first servo** before buying in bulk (the datasheet does not state the bracket-hole thread; M2 is assumed).

| Where | Thread | Per leg | Length (mm) |
|---|---|---|---|
| coxa horn | M3 | 4 | 10 |
| femur horn side | M3 | 4 | 10 |
| femur back side | M3 | 4 | 10 |
| knee horn side | M3 | 4 | 16 |
| knee back side | M3 | 4 | 16 |
| clevis walls | M2 | 8 | 6 |
| knee servo | M2 | 8 | 12 |
| foot bridge | M2 | 4 | 10 |

Plus per servo: the centre screw that comes with the horn (M3 x 6), and 3 servos per leg.

### Body hardware

| Where | Thread | Count | Length (mm) | Nut |
|---|---|---|---|---|
| top plate to coxa servo (M2) | M2 | 24 | 8 | no |
| hip plate to coxa servo (M2) | M2 | 24 | 6 | no |
| post ends into the plates and hip plates (M3) | M3 | 40 | 10 | no |
| splice plates (M3, with nuts) | M3 | 32 | 12 | yes |
| insert grid: heat-set inserts M3 (not screws) | insert | 42 | - | no |
| battery tray to the bottom plate: heat-set inserts M3 (not screws) | insert | 4 | - | no |
| battery tray to the bottom plate (M3) | M3 | 4 | 6 | no |
| Pi plate to the top deck (M3, into the grid inserts) | M3 | 4 | 6 | no |
| Raspberry Pi 5 to the plate (M2.5, self-tapping) | M2.5 | 4 | 6 | no |

Use low-profile (button or countersunk) M3 screws on the underside of the bottom plate, or the real sit height is the plate thickness plus a head.
