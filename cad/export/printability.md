# Printability review

Computed from the CAD (`bemu_cad.printability`). The support estimate counts surfaces facing down more steeply than 45 degrees above the first layer; "small" means under about 25 mm² in the better of the two directions along the print axis. Masses use 60% fill (an estimate). Sizes are footprint × height in the print orientation.

**Largest footprint: 190 mm. Fits a 200 × 200 mm bed: yes** (the plates are split at x = ±40 mm for this; a bigger bed would allow fewer pieces).

| Part | Copies | Size (mm) | Orientation | Overhang (mm²) | Supports | One part (g) |
|---|---|---|---|---|---|---|
| coxa_bracket | 7 | 97 × 38 × 31 | +z up | 198 | yes, small | 13.1 |
| femur_plate_horn | 6 | 116 × 28 × 6 | -y up | 0 | no | 6.8 |
| femur_plate_back | 6 | 116 × 28 × 7 | +y up | 0 | no | 7.0 |
| knee_spacer_horn | 6 | 51 × 28 × 6 | -y up | 0 | no | 4.3 |
| knee_spacer_back | 6 | 51 × 28 × 6 | +y up | 0 | no | 4.2 |
| tibia_plate_horn | 6 | 138 × 28 × 11 | -y up | 36 | yes, small | 8.5 |
| tibia_plate_back | 6 | 138 × 28 × 12 | +y up | 45 | yes, small | 8.8 |
| foot_bridge | 6 | 55 × 24 × 14 | -z up | 41 | yes, small | 13.9 |
| foot | 8 | 16 × 16 × 12 | +z up | 87 | yes, small | 0.8 |
| top_plate_rear | 1 | 166 × 83 × 4 | -z up | 0 | no | 31.3 |
| top_plate_centre | 1 | 190 × 80 × 4 | -z up | 0 | no | 37.5 |
| top_plate_front | 1 | 166 × 83 × 4 | -z up | 0 | no | 31.3 |
| bottom_plate_rear | 1 | 166 × 83 × 4 | +z up | 0 | no | 33.8 |
| bottom_plate_centre | 1 | 190 × 80 × 4 | +z up | 0 | no | 40.3 |
| bottom_plate_front | 1 | 166 × 83 × 4 | +z up | 0 | no | 33.8 |
| hip_plate | 6 | 62 × 28 × 3 | +z up | 0 | no | 2.7 |
| post_main | 8 | 8 × 8 × 50 | +z up | 5 | no | 1.8 |
| post_hip | 12 | 8 × 8 × 15 | +z up | 0 | no | 0.5 |
| splice_top | 4 | 30 × 12 × 3 | +z up | 0 | no | 0.8 |
| splice_bottom | 4 | 30 × 12 × 3 | +z up | 0 | no | 0.8 |

Material for the whole robot: PETG 664 g, TPU 7 g; with 14% for supports, purge and a failed print: PETG 764 g (one 1 kg spool is enough), TPU 8 g (or buy rubber tips).

Orientation: "+y up" means the part is placed so that the +y axis of its file points up; plates lie flat, the coxa bracket and the bridge sit on their base, the foot with its flat face down (turn it over). Where a direction is given as minus, flip the part about a horizontal axis (the better side up). Every part fits with its large face on the bed; no part needs more than a few small supports.

## What a print service must offer

- Build volume of at least 200 × 200 × 80 mm (the tibia plates are 138 mm long, the centre plates 190 mm); a larger bed lets the plates be printed in fewer pieces.
- PETG (or ABS/ASA/nylon) with 0.2 mm layers; TPU for the feet is optional.
- Settings from the print order: 4 perimeters (1.6 mm walls), 35% infill, no brim on the tall posts.
- The hole allowances (servo pockets, screw and insert holes) are not known until the tolerance coupon is printed and measured: order it first, alone, and measure it with a caliper.

## Parts to ask about

- `coxa_bracket` and `foot` need a few small supports; the posts are 8 mm wide and 50 mm tall (ask for a brim or print them on their side if they wobble).
- The 3 mm plates are close to the minimum for four perimeters: they print solid.
