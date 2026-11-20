# Printability review

Computed from the CAD (`bemu_cad.printability`). The support estimate counts surfaces facing down more steeply than 45 degrees above the first layer, in the better of the two directions along the print axis. A part needs supports when that area is over 25 mm²; the column gives the area. Masses use 60% fill (an estimate). Sizes are footprint × height in the print orientation.

**Largest footprint: 190 mm, tallest part 50 mm. Fits a 200 × 200 × 80 mm bed with 3 mm free on each side: yes** (the plates are split at x = ±40 mm for this; a bigger bed would allow fewer pieces).

| Part | Copies | Size (mm) | Orientation | Supports (overhang) | One part (g) |
|---|---|---|---|---|---|
| coxa_bracket | 7 | 97 × 38 × 31 | +z up | yes (195 mm²) | 13.1 |
| femur_plate_horn | 7 | 116 × 28 × 6 | -y up | no | 6.8 |
| femur_plate_back | 7 | 116 × 28 × 7 | +y up | no | 7.0 |
| knee_spacer_horn | 7 | 51 × 28 × 6 | -y up | no | 4.3 |
| knee_spacer_back | 7 | 51 × 28 × 6 | +y up | no | 4.2 |
| tibia_plate_horn | 7 | 138 × 28 × 11 | -y up | yes (36 mm²) | 8.5 |
| tibia_plate_back | 7 | 138 × 28 × 12 | +y up | yes (45 mm²) | 8.8 |
| foot_bridge | 7 | 55 × 24 × 14 | -z up | yes (40 mm²) | 13.9 |
| foot | 8 | 16 × 16 × 12 | +z up | yes (99 mm²) | 0.8 |
| top_plate_rear | 1 | 166 × 83 × 4 | -z up | no | 31.2 |
| top_plate_centre | 1 | 190 × 80 × 4 | -z up | no | 37.3 |
| top_plate_front | 1 | 166 × 83 × 4 | -z up | no | 31.2 |
| bottom_plate_rear | 1 | 166 × 83 × 4 | +z up | no | 33.7 |
| bottom_plate_centre | 1 | 190 × 80 × 4 | +z up | no | 40.3 |
| bottom_plate_front | 1 | 166 × 83 × 4 | +z up | no | 33.7 |
| hip_plate | 6 | 62 × 28 × 3 | +z up | no | 2.7 |
| post_main | 8 | 8 × 8 × 50 | +z up | no | 1.8 |
| post_hip | 12 | 8 × 8 × 15 | +z up | no | 0.5 |
| splice_top | 4 | 30 × 12 × 3 | +z up | no | 0.8 |
| splice_bottom | 4 | 30 × 12 × 3 | +z up | no | 0.8 |
| battery_tray | 1 | 142 × 68 × 15 | +z up | yes (1224 mm²) | 18.5 |
| pi_plate | 1 | 96 × 66 × 9 | +z up | no | 14.0 |

Material for the whole robot: PETG 750 g, TPU 7 g; with 15% for supports, purge and a failed print: PETG 862 g (one 1 kg spool is enough), TPU 8 g (or buy rubber tips).

Orientation: "+y up" means the part is placed so that the +y axis of its file points up; plates lie flat, the coxa bracket and the bridge sit on their base. Where a direction is given as minus, flip the part about a horizontal axis (the better side up). The foot is printed as modelled: dome down on a small support, stem up (rubber tips can replace it). Every part fits with its large face on the bed.

## What a print service must offer

- Build volume of at least 200 × 200 × 80 mm (the tibia plates are 138 mm long, the centre plates 190 mm); a larger bed lets the plates be printed in fewer pieces.
- PETG (or ABS/ASA/nylon) with 0.2 mm layers; TPU for the feet is optional.
- Settings from the print order: 4 perimeters (1.6 mm walls), 35% infill.
- The hole allowances (servo pockets, screw and insert holes) are not known until the tolerance coupon is printed and measured: order it first, alone, and measure it with a caliper.

## Parts to ask about

- Parts that need supports (over 25 mm² of overhang): `coxa_bracket` (195 mm²), `tibia_plate_horn` (36 mm²), `tibia_plate_back` (45 mm²), `foot_bridge` (40 mm²), `foot` (99 mm²), `battery_tray` (1224 mm²). Everything else prints without.
- The posts are 8 mm wide and 50 mm tall: if they wobble on the bed, ask for a brim.
- The 3 mm plates are thinner than four perimeters side by side are wide, so they print solid (the walls fill the thickness).
