# 08 — Mechanical design

The body, the legs, and how the parts stay easy to change.

All dimensions here are **starting values**. Versions 1 and 1.5 tune them in
simulation before anything is printed.

## Layout

Eighteen joints: six legs with three joints each. The body is an elongated hexagon
with a leg at each corner and one in the middle of each long side.

```
                 front (+x)
          LF  ●───────────●  RF
             ╱             ╲
            ╱               ╲
       LM  ●                 ●  RM         +y is left
            ╲               ╱
             ╲             ╱
          LR  ●───────────●  RR
                 rear
```

### Leg mounting

Position of each hip swing axis in the body frame, and the direction the leg points
at rest (angle from straight ahead, counter-clockwise positive).

| Leg | x (mm) | y (mm) | Resting direction |
|---|---|---|---|
| LF | +100 | +60 | +45° |
| LM | 0 | +80 | +90° |
| LR | −100 | +60 | +135° |
| RF | +100 | −60 | −45° |
| RM | 0 | −80 | −90° |
| RR | −100 | −60 | −135° |

### Leg segments

| Segment | Symbol | Length (mm) |
|---|---|---|
| Coxa | `Lc` | 45 |
| Femur | `Lf` | 65 |
| Tibia | `Lt` | 110 |

### Resulting size

| Measure | Value |
|---|---|
| Body, approximate outline | 240 × 160 mm, 56 mm thick (the sit pose needs at least 48 mm; see [07](07-locomotion.md)) |
| Standing footprint (foot tip to foot tip) | about 370 mm long × 400 mm wide |
| Standing body height (hip axes above the foot centres) | 90 mm; 98 mm above the floor, since the 8 mm foot sphere's centre is 8 mm up |
| Ground clearance under the body | 70 mm with the 56 mm body |

### Joint ranges

| Joint | Range | Limited by |
|---|---|---|
| Coxa | ±45° | Neighbouring legs colliding |
| Femur | −60° to +90° | Body and bracket interference |
| Tibia | −150° to −20° | Tibia folding into the femur; knee over-extension |

Final limits are set from the CAD by checking for collisions, and written into the
parameter file.

## Mass budget

| Item | Mass (g) | Basis |
|---|---|---|
| 18 servos at 55 g | 990 | Datasheet |
| Printed parts | 550 | Estimate |
| Battery | 350 | Estimate for a 3-cell pack of 4000–5000 mAh |
| Pi 5 with cooler | 75 | Estimate |
| Power electronics and wiring | 120 | Estimate |
| Fasteners, bearings, inserts | 100 | Estimate |
| **Robot** | **about 2,200** | |
| Reserved for add-on modules | 300 | Design choice |
| **Design mass** | **2,500** | Used for all sizing |

## Sizing the servos: the torque estimate

The femur joint carries the most load. This is the hand calculation; the simulation
replaces it with measured values in version 1.

```
Weight at design mass            2.5 kg × 9.81         = 24.5 N
Horizontal distance, hip lift
  axis to foot, in neutral stance                        75 mm

Typical: three legs share evenly  24.5 ÷ 3 = 8.2 N  →  8.2 × 0.075 = 0.61 N·m   (6.3 kg·cm)
Worst case: one leg carries half  24.5 ÷ 2 = 12.3 N →  12.3 × 0.075 = 0.92 N·m  (9.4 kg·cm)
With 30% added for motion                              0.92 × 1.3  = 1.2 N·m   (12.2 kg·cm)
```

**Design peak: 1.2 N·m. Design average: 0.6 N·m.**

The rule from the roadmap: peak under 50% of stall torque, average under rated
torque. The candidates are compared against these numbers in
[10 — Components](10-components.md).

Two things reduce the required torque if needed: a shorter femur, or a stance with
the feet closer in. Both are parameter changes.

## Parametric design

### The parameter file

Every dimension lives in `config/robot_params.yaml`. An illustrative excerpt:

```yaml
units: {length: mm, angle: deg, mass: g}

leg:
  coxa_length: 45
  femur_length: 65
  tibia_length: 110
  joint_limits:
    coxa:  [-45, 45]
    femur: [-60, 90]
    tibia: [-150, -20]

body:
  mounts:
    lf: {x: 100,  y: 60,  yaw: 45}
    lm: {x: 0,    y: 80,  yaw: 90}
    lr: {x: -100, y: 60,  yaw: 135}
    rf: {x: 100,  y: -60, yaw: -45}
    rm: {x: 0,    y: -80, yaw: -90}
    rr: {x: -100, y: -60, yaw: -135}
  plate_thickness: 4
  deck_spacing: 40

stance:
  foot_radius: 120
  body_height: 90

servo:
  model: STS3215
  body: {x: 45.2, y: 24.7, z: 35.0}
  mass: 55

mounting_grid:
  pitch: 20
  thread: M3
```

### What reads it

| Consumer | Uses |
|---|---|
| CAD scripts | Every part dimension |
| Model generator | Link lengths, joint limits, mount positions, masses |
| Kinematics library | Link lengths, mount positions, joint limits |
| Walker | Neutral stance |

### What "flexible parts" means in practice

| Change wanted | What is edited | What regenerates |
|---|---|---|
| Longer tibia | `tibia_length` | Tibia part, model, leg maths |
| Wider body | `mounts` | Body plates, model, leg maths |
| Different servo model | `servo` block | Every bracket, model masses |
| Thicker plates | `plate_thickness` | Plates, spacer heights |
| A new module mount | Nothing in the body | Only the new adapter plate |

### CAD tool

**Chosen: build123d**, a Python library for parametric CAD. The owner left the
choice to the designer; see [decision 0007](decisions/0007-code-driven-cad.md).

| Reason | Detail |
|---|---|
| Parametric by construction | A part is a function of the parameters |
| Lives in git | The design is text; changes show up in a diff |
| Reads the parameter file directly | No copying numbers between tools |
| Exports STL for printing and STEP for exchange | Standard formats |
| Can compute mass and inertia | Feeds the simulation model |

**Alternative: FreeCAD**, a free visual CAD program. Easier for shaping complex
brackets by eye, and it supports spreadsheets of parameters. The cost is that the
link to the parameter file is manual or scripted, and design files do not diff
well.

A mixed approach stays available: code CAD for the parts that scale with
parameters, visual CAD for a one-off shape if code proves too slow for it.

## The component library and the printed leg (version 1.5a)

Every bought part that touches a printed part has its real dimensions in `config/components.yaml`.
Each value carries a source (a file in `docs/reference/`, or the ISO table) and a status:
`datasheet`, `cad` (measured from the manufacturer's STEP), `standard`, `measured` (a caliper on
the real part) or `unverified`. `python3 -c "from bemu_cad.components import load_components, unverified_fields"`
lists the unverified ones; the print order names them and says it is **not ready to order** until
the two checks below are done.

**Feetech STS3215, 12 V, 1:345, 30 kg·cm** (datasheet ST-3215-C047, sold as C018; the 7.4 V
version looks the same but is not the one the torque report validated):

| Feature | Value | Source |
|---|---|---|
| Body | 45.22 × 24.72 mm; 35 mm high on the drawing, 35.7 mm in the STEP (the pocket uses the larger) | drawing and STEP |
| Output axis | 10.11 mm from one end, on the centre line | drawing |
| Output horn | 25-tooth spline Ø5.9, disc Ø19.95, 4 × M3 holes on a Ø14 circle, M3×6 centre screw | datasheet |
| Bracket holes, horn face | 4 holes, 20.7 × 20.5 mm, pilot Ø1.6, counterbore Ø3.5, 3.9 mm deep | STEP, drawing |
| Bracket holes, back face | 4 holes, 24.45 × 20.5 mm, pilot Ø1.6, counterbore Ø3.5, 4.8 mm deep | STEP, drawing |
| Rear (second) horn | same 4-hole pattern; its face sits 0.8 mm inside the back face | STEP |
| Mass | 55 ± 1 g; stall current 2.7 A; idle current 30 mA; rated current 0.9 A | datasheet |

The thread of the bracket holes is not given anywhere. The 1.6 mm pilot suggests M2; the design
assumes M2 clearance holes (2.4 mm) until it is checked.

**Two steps only the owner can do, before the final print order**
1. Print the tolerance coupon alone (`python3 tools/generate_cad.py --coupon-only`, file
   `cad/export/tolerance_coupon.stl`), measure which servo pocket (allowance +0.1, +0.2 or +0.3 mm,
   marked with 1, 2 or 3 notches) and which hole sizes fit, and put the choice in `config/`.
2. Caliper the first servo received (body size, hole spacing, pilot diameters) and see which
   screws come in its accessory bag; set `servo.case_hole_thread` and mark the values `measured`.

**The leg.** Generated by `bemu_cad.leg.build_leg` from `robot_params.yaml` and the library; one
change to a leg length regenerates everything. The coxa servo is vertical (horn down, body inboard)
under a base plate with a clevis that holds the femur servo. The femur plates sit outside the
clevis walls on **both** horns of the dual-axis servo and carry the knee servo through 3.8 mm
spacers; the tibia plates sit outside them on the knee servo's two horns and are joined by the foot
bridge, with a TPU hemisphere foot whose centre is the foot centre. All servos are centred on the
leg's radial plane. Nine part files per leg. Checked in tests: axis distances equal the parameter
file; no printed part penetrates a servo; screw-hole edge distance ≥ 1.5 hole diameters and
1.2 mm between holes; every part fits the bed in its print orientation; the joints sweep their full
range without collisions (skipping poses with the foot closer than 60 mm to the hip axis: folded
under the body, never used by a gait). Hard stops and cable slots come with the body (1.5b).

**First finding for the mass budget:** with real servos and estimated printed masses the leg weighs
about 241 g (printed 64 g, servos 165 g, fasteners 12 g) against the 185 g assumed in version 1;
six legs are 0.34 kg heavier. Version 1.5b reconciles the 2.5 kg design mass.

## Part families

| Family | Parts | Notes |
|---|---|---|
| Body | Bottom plate, top deck, spacers or side walls | Electronics bay between the plates; battery low and central |
| Hip | Coxa bracket, joining the swing servo to the lift servo | The most highly loaded printed part |
| Femur | Two side links joining the lift servo to the knee servo | Symmetric, simple |
| Tibia | Lower leg holding the knee servo's output, ending in the foot | Longest part; stiffness matters |
| Foot | Rounded tip in flexible material | Replaceable; later holds a contact switch |
| Mounts | Electronics tray, battery tray, sensor adapters | Bolt to the grid |

## Joint design rules

1. **Support both sides.** Each joint is carried by the servo's output on one side
   and a pivot on the opposite side, so bending loads do not rest on the output
   shaft alone. The STS3215 is sold as a dual-axis servo with a rear pivot opposite
   the output; its dimensions are confirmed against the datasheet before the
   brackets are drawn.
2. **Servos are structural.** Brackets wrap the servo body and bolt to its mounting
   points; the servo is part of the leg.
3. **No load through layer lines.** Parts are oriented for printing so that the
   main bending load runs along the layers, not across them.
4. **Hard stops.** Each joint has a physical stop slightly beyond its software
   limit.
5. **Cables.** Each leg has a cable path with slack at every joint, clear of pinch
   points.

## The mounting standard

Any add-on bolts to the robot without modifying the body.

| Feature | Specification |
|---|---|
| Top deck grid | M3 heat-set threaded inserts on a 20 mm pitch |
| Grid size | At least 5 × 7 positions |
| Front sensor mount | Two M3 inserts 20 mm apart on a tilting bracket |
| Underside | Four M3 inserts for a belly-mounted sensor |
| Adapter plates | Each module gets its own plate that matches the grid |
| Cable pass-throughs | Slots in the top deck at front, centre and rear |

## Materials and printing

| Part | Material | Reason |
|---|---|---|
| Body plates, brackets, links | PETG or a tough PLA | Stiff enough, tolerant of knocks; PETG handles warm servos better |
| Feet | TPU (flexible) | Grip and shock absorption |
| Highly loaded brackets, if they fail | Nylon or carbon-filled filament | Only if testing shows a need |

| Print guideline | Starting value |
|---|---|
| Wall thickness | At least 4 perimeters on structural parts |
| Infill | 30–40% for structural parts |
| Layer height | 0.2 mm |
| Hole tolerance | Add 0.2 mm to nominal diameter; verify with a test print |
| Print bed assumed | 200 × 200 mm until the print service's size is confirmed. Body plates are split with a lap joint if the bed is too small for a single piece |

A tolerance test piece (holes, slots, a servo pocket) is printed and measured before
any real part.

## Printing through someone else

There is no printer at hand, so every part is ordered. That changes how the parts
are designed and released.

| Consequence | What the design does about it |
|---|---|
| Each design iteration costs a delivery wait | Check fit in CAD and in the simulation model first; order in few, complete batches |
| The printer's accuracy is unknown | The first order is the tolerance test piece. Its measurements set the hole and pocket allowances in the parameter file before real parts are ordered |
| The printer's size is unknown | Assume 200 × 200 mm until confirmed; split larger parts |
| The person printing chooses settings unless told | Every order includes a print specification: material, layer height, perimeters, infill, and the orientation of each part |
| Flexible material may not be offered | Feet can use bought rubber tips instead of printed TPU |
| A failed part means another order | Order one spare of each small, highly loaded part, such as the hip bracket |
| Threaded inserts are fitted after printing | Done at home with a soldering iron; holes are sized for the inserts |

### What is sent with each order

| Item | Format |
|---|---|
| Part files | STL for printing; STEP as well, in case the service wants to re-slice or check dimensions |
| Print specification | One page: material, colour, layer height, perimeters, infill, quantity per part |
| Orientation sheet | A picture per part showing which face goes on the bed |
| Critical dimensions | The few dimensions per part that must be right, so the service can check them |

All of these are generated from the CAD scripts, so an order always matches the
current parameter file.

### Order sequence

| Order | Contents | Roadmap step |
|---|---|---|
| 1 | Tolerance test piece | 2.4 |
| 2 | One leg, a leg stand, the zero-position jig | 2.4 |
| 3 | Body, five more legs, trays, spares | 3.2 |
| Later | One adapter plate per new module | Each version |

## Fasteners

| Use | Fastener |
|---|---|
| Structure | M3 screws into heat-set brass inserts |
| Servo mounting | The screws supplied with the servo, or the size its datasheet specifies |
| Servo output | Supplied horn and horn screw |
| Rear pivots | Flanged bearing or a shoulder screw, sized to the servo's rear pivot |

One screw family (M3) wherever possible, so one driver and one box of spares.

## Assembly and calibration

| Step | Purpose |
|---|---|
| Assign each servo its bus address before assembly | Addresses 1–18, labelled on the servo |
| Centre each servo before fitting the horn | So mechanical zero is near the middle of travel |
| Assemble using a zero-position jig | A printed jig holds each leg in the defined zero pose |
| Record each joint's offset and direction | Stored in `servos.yaml`; corrects for horn spline steps |
| Verify against the model | Command the neutral stance; measure foot positions |

## From CAD to simulation

From version 1.5, each part's mass, centre of mass and inertia are computed from
its CAD shape and material density and written into the robot model, with servo and
electronics masses added as point masses at their real positions. The finished
robot is weighed in version 3 to check.

## Open mechanical questions

| Question | Resolved in |
|---|---|
| The print service's build size and materials | Step 1.5.6 (question Q5) |
| Body plates versus a printed shell | Version 1.5 |
| Final joint limits | Version 1.5, from collision checks |
| Whether feet need switches or load sensing is enough | Version 5 |

## References

| Source | Used here for | Checked |
|---|---|---|
| [Feetech STS3215, Core Electronics](https://core-electronics.com.au/feetech-sts3215-smart-servo.html) | STS3215 dimensions (45.2 × 24.7 × 35 mm) and mass (55 g) | 2026-11-13 |
| [Feetech STS3215, AIFITLAB](https://aifitlab.com/collections/servo-motor/products/feetech-sts3215-servo-motor) | STS3215 torque figures used in the sizing estimate | 2026-11-13 |
| [Feetech STS3215, Servo Database](https://servodatabase.com/servo/feetech/sts3215) | STS3215 sub-models | 2026-11-13 |
| [build123d documentation](https://build123d.readthedocs.io/) | The chosen CAD tool | — |
| [FreeCAD](https://www.freecad.org/) | The visual CAD alternative | — |

The full list for the project is in [References](references.md). A dash means the source
is listed for reference and was not re-read on a specific date.
