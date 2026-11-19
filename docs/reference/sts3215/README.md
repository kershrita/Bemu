# Feetech STS3215 (12 V, 30 kg·cm) reference files

Downloaded 2026-11-19; hashes in `SHA256SUMS`. These are the source for every servo dimension in
`config/components.yaml` (version 1.5a).

| File | What | Source |
|---|---|---|
| `seeed-ST-3215-C047-datasheet.pdf` | Feetech product specification, 12 V variant ST-3215-C047 (2023-7-20): electrical (stall 2.7 A, idle 30 mA, 12 V), mechanical (45.2 × 24.7 × 35 mm, 55 g, 25T/OD5.9 mm horn, M3×6 horn screw), horn drawing | files.seeedstudio.com (Seeed Studio wiki) |
| `waveshare-ST3215-2D-drawing.pdf`, `.dxf` | Dimension drawing (body 45.22 × 24.72, output axis 10.11 mm from one end and 14 mm / Ø19.2 horn circle, 20.5 mm hole spacing, 32 / 35 / 37.25 mm heights) | Waveshare ST3215 wiki |
| `waveshare-ST3215.step` | 3D model, output axis along Y at (x = −25.5, z = 0) | Waveshare ST3215 wiki |
| `feetech-ST3215-20200328.stp` | Feetech's own 3D model (2020-03-28) | Aifitlab (Feetech store) |

## Mounting holes (measured from the STEP files, version 1.5a task 2)

Servo frame: origin on the output axis; x along the body toward the long side (the far end is
35.11 mm away); y across the width; the output horn is on the +z side. Both manufacturer models
agree to 0.1 mm.

| Face | Hole x (mm) | Hole y (mm) | Pattern | Pilot Ø | Counterbore Ø | Depth |
|---|---|---|---|---|---|---|
| Horn face | 8.3 and 29.0 | ±10.25 | 20.7 × 20.5 (drawing) | 1.6 | 3.5 | 3.9 mm |
| Back face | 8.3 and 32.75 | ±10.25 | 24.45 × 20.5 (drawing) | 1.6 | 3.5 | 4.8 mm |
| Horn (output) | ±7 on both axes | | 4 holes on a Ø14 circle | 2.5 | | |

The thread of the bracket holes is **not stated** in the datasheet or the drawing. A 1.6 mm pilot
suggests an M2 self-tapping screw; the horn's four holes are M3 (datasheet "4-M3"). **To confirm
on the first servo received:** which screws come in its accessory bag, and measure the pilot holes
with a caliper. The tolerance coupon carries a row of pilot sizes for this.

The case also has end holes at x = −7.1 and 32.75 (z ±9.35 to ±10) that hold the case together;
they are not used for brackets.
