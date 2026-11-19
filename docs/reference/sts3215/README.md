# Feetech STS3215 (12 V, 30 kg·cm) reference files

Downloaded 2026-11-19; hashes in `SHA256SUMS`. These are the source for every servo dimension in
`config/components.yaml` (version 1.5a).

| File | What | Source |
|---|---|---|
| `seeed-ST-3215-C047-datasheet.pdf` | Feetech product specification, 12 V variant ST-3215-C047 (2023-7-20): electrical (stall 2.7 A, idle 30 mA, 12 V), mechanical (45.2 × 24.7 × 35 mm, 55 g, 25T/OD5.9 mm horn, M3×6 horn screw), horn drawing | files.seeedstudio.com (Seeed Studio wiki) |
| `waveshare-ST3215-2D-drawing.pdf`, `.dxf` | Dimension drawing (body 45.22 × 24.72, output axis 10.11 mm from one end and 14 mm / Ø19.2 horn circle, 20.5 mm hole spacing, 32 / 35 / 37.25 mm heights) | Waveshare ST3215 wiki |
| `waveshare-ST3215.step` | 3D model, output axis along Y at (x = −25.5, z = 0) | Waveshare ST3215 wiki |
| `feetech-ST3215-20200328.stp` | Feetech's own 3D model (2020-03-28) | Aifitlab (Feetech store) |

The mounting-hole list is **not** dimensioned on the drawing; it is measured from the STEP files
(see plan 1.5a) and must be confirmed with a caliper on the first servo received.
