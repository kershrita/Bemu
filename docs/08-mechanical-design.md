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
| Body, approximate outline | 240 × 160 mm |
| Standing footprint (foot tip to foot tip) | about 370 mm long × 400 mm wide |
| Standing body height (hip axes above ground) | 90 mm |
| Ground clearance under the body | about 60 mm, depending on body thickness |

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

**Proposed: build123d**, a Python library for parametric CAD.

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

This is open question Q3 in [02 — Design](02-design.md). A mixed approach is also
workable: code CAD for the parts that scale with parameters, visual CAD for one-off
shapes.

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
| Print bed assumed | 200 × 200 mm. Body plates are split with a lap joint if the bed is too small for a single piece |

A tolerance test piece (holes, slots, a servo pocket) is printed and measured before
any real part.

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
| Print bed size, or use of a print service | Open question Q2 |
| Body plates versus a printed shell | Version 1.5 |
| Final joint limits | Version 1.5, from collision checks |
| Whether feet need switches or load sensing is enough | Version 5 |
