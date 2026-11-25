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
| Coxa | ±45° | Neighbouring legs: free while the angles toward each other sum to 45° or less (checked in CAD; the gaits peak at about 43°) |
| Femur | −60° to +90° | Body and bracket interference |
| Tibia | −150° to −20° | Tibia folding into the femur; knee over-extension |

Final limits are set from the CAD by checking for collisions, and written into the
parameter file.

## Mass budget

| Item | Mass (g) | Basis |
|---|---|---|
| 18 servos at 55 g | 990 | Datasheet |
| Printed parts: six legs 405 g, body and trays 285 g | 690 | CAD volume, PETG at 60% fill: an estimate until weighed |
| Battery | 359 | Ovonic 3S 5000 mAh listing (unverified until weighed) |
| Pi 5 with cooler | 75 | Estimate |
| Power electronics and wiring | 120 | Estimate |
| Fasteners (legs 193 g, body 30 g), inserts | 223 | Screw model from the CAD; body is an estimate |
| **Robot (version 1.5d estimate)** | **about 2,457** | |
| Left of the 300 g add-on reserve | 43 | The reserve is almost used up; hard stops and cable slots are still to come |
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
| Body | 45.22 × 24.72 mm. Along the output axis the screws seat on shelves **32.0 mm apart** (the drawing's 32); a raised lid strip (x 9..34.3, y ±7) stands 1.1 mm above the horn-side shelf and a rear motor-end boss (x 17.4..29.7, y ±9.2) 1.9 mm below the back shelf; the drawing's 35 is strip to back boss | STEP (both models), drawing |
| Output axis | 10.11 mm from one end, on the centre line | drawing |
| Output horn | 25-tooth spline Ø5.9, disc Ø19.95, 4 × M3 holes on a Ø14 circle, M3×6 centre screw | datasheet |
| Bracket holes, horn face | 4 holes, 20.7 × 20.5 mm, pilot Ø1.6, counterbore Ø3.5, 3.9 mm deep | STEP, drawing |
| Bracket holes, back face | 4 holes, 24.45 × 20.5 mm, pilot Ø1.6, counterbore Ø3.5, 4.8 mm deep | STEP, drawing |
| Rear (second) horn | same 4-hole pattern; its disc stands proud of the back shelf, with a fixed Ø6 hub 0.55 mm beyond it that a boss must clear | STEP |
| Horn height | 9.2 mm (Feetech model) or 9.6 mm (Waveshare model and the drawing) above the horn-side shelf: **the models disagree**; the design uses 9.6 so nothing overlaps, and the caliper decides | STEP, drawing |
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
under a base plate with a clevis that holds the femur servo. The walls and spacers are cut with the servo's real outline (lid strip, rear boss, idler, hub) so they seat on the
shelves. The femur plates sit outside the clevis walls on **both** horns of the dual-axis servo and carry the knee servo through
spacers; the tibia plates sit outside them on the knee servo's two horns and are joined by the foot
bridge, with a TPU hemisphere foot whose centre is the foot centre. All servos are centred on the
leg's radial plane. Nine part files per leg. Checked in tests: axis distances equal the parameter
file; no printed part penetrates a servo; screw-hole edge distance ≥ 1.5 hole diameters and
1.2 mm between holes; every part fits the bed in its print orientation; the joints sweep their full
range without collisions (skipping poses with the foot closer than 60 mm to the hip axis: folded
under the body, never used by a gait). Screw heads are modelled (every sliding gap holds one) and the whole assembly is
checked against the manufacturer's own STEP models of the servo, which caught a rear hub collision the envelope
hid. The bracket-hole thread is assumed M2; the print order lists screw lengths for each joint and says it is not
ready to order until the fit-critical values are measured. Hard stops, cable slots and leg-to-leg collision checks are not done yet (1.5c).

**Masses from the CAD (version 1.5b).** `python3 tools/generate_cad.py --sync-masses` writes the link
masses, **centres of mass and inertias**, the chassis budget and the body thickness into
`config/robot_params.yaml`; the model generators (MuJoCo and URDF) use them. Coxa link 75.1 g (bracket,
femur servo, screws), femur link 89.7 g (plates, spacers, knee servo, screws), tibia link 44.9 g (plates,
bridge, foot, screws): 209.7 g per leg without the coxa servo, which belongs to the body. Centres of mass
from each link's joint: coxa 54 mm, femur 70 mm, tibia 73 mm (the model used the link midpoints, 22, 32
and 55 mm). Version 1 assumed 75 / 25 / 85 g: the knee servo is in the femur link, so the femur is much
heavier and the tibia lighter, and the legs are about 25 g each heavier (screws and the printed bridge).
Six legs weigh 1.258 kg, so the chassis budget for the 2.5 kg design mass is **1.242 kg** (it was 1.39 kg).
Printed masses are estimates (PETG, 60% fill) until the parts are weighed. (An earlier note here said the
leg was 0.34 kg heavier: that wrongly counted the coxa servo, which the chassis mass already included.)

**The body (version 1.5b).** Generated by `bemu_cad.body.build_body`. Two 4 mm plates (top and bottom),
each in three pieces split at x = ±40 mm so every piece fits a 200 × 200 mm bed, joined by 3 mm splice
plates (four per plate). The outline is the 240 mm decagon through the hip mounts plus a padded
footprint of each coxa servo (about 246 mm long). Each coxa servo hangs horn-down and is held at **both**
faces: its back shelf seats against the top plate (recess for the idler and hub, four M2 holes aligned
with the servo's pilot holes), its horn-side shelf against a 3 mm support plate per leg that stands on
two 15 mm posts from the bottom plate; the horn passes through the support plate's window and the coxa bracket's base
plate, with its 2.4 mm boss, turns below it with 0.6 mm to spare. Eight 50 mm posts join the plates (they stand at x = ±88, y = ±15 and x = ±21, y = ±29 so that a 50 mm wide channel along x stays free for the battery); the top
plate carries a 20 mm grid of 42 M3 insert holes (Ø4.1), and the coxa servo's hub passes through a hole in it. The body is 58.08 mm thick (the simulation's body box now uses it: the sit height is
29.04 mm). Checked in tests: plates cover every hip and servo footprint; pieces fit the bed; the coxa
servo seats on the real pilot holes at both faces; no body part touches a real coxa servo (both STEP
models); a swinging left-front and left-middle leg (coxa ±45° and 0, five femur/knee poses including sit and
stand) never touches any body part or the screw heads under the support plates. Neighbouring legs are
checked against each other in version 1.5d (below); the limits alone do allow folded-under poses that no gait reaches. **Mass (version 1.5d):** printed body 285 g (estimate, including the two trays), six coxa
servos 330 g, electronics 584 g (Pi 75, battery 359 from the listing, regulator and wiring 120, fasteners 30):
1.199 kg against the chassis budget of 1.242 kg, a margin of 43 g: the 300 g add-on reserve is
almost gone. Hard stops and cable slots are still to come (version 2, once the first leg exists).

**Trays and neighbour check (version 1.5d).** `bemu_cad.trays` adds a **battery tray** (PETG, 142 × 68 × 15 mm:
a base, two end walls with a lead notch, two side lips, strap flanges with four 12 × 3 mm slots joined by a 2 mm channel under the base (a 10 mm strap passes through), four M3 × 6
screws into four heat-set inserts in the bottom plate) around the pack, which lies along x between the
posts (the Ovonic 3S 5000 mAh listing: 138 × 42 × 28 mm, 359 g, unverified until a pack is bought), and a
**Pi plate** on the top deck (96 × 66 × 3 mm, four 6 mm standoffs at the board's 58 × 49 mm hole pattern, four M3
screws into grid inserts, notches to keep the post screws reachable). The Pi (85 × 56 mm, 17 mm assumed
height, hanging 10 mm past the hole pattern toward its connector end, which points forward) is drawn on the top deck, the pack in the bay. Tests check that the pack clears every post, hip
plate and coxa servo by at least 1 mm, that the tray clears the posts, that the Pi board clears the coxa
servos and top plates, that tray and plate screws land on insert holes, that every plate piece has two
posts under it, and that the Pi standoffs sit on the hole pattern. **Neighbouring legs:** with both legs
in the same femur and knee pose (stand, sit, two extended poses and one mid pose), two neighbours do not touch
while the sum of the angles they swing toward each other is 45° or less (checked on a 15° grid); at a sum of
60° they do. The gaits stay inside this (the largest sum is about 43° for the wave gait at top speed), and a
test keeps them there. The per-joint coxa limit stays ±45°: body shifts and yaw change the neighbours' angles unequally and can bring them together, but with the gaits at top speed over shifted and yawed poses the sum peaked at 42.8°, and the gait test (including gait changes and body poses) keeps it under 45°. A new gait or pose command that could exceed that must widen the CAD check first. **The splice plates moved to y = ±50 mm** to free the battery
tray's strap flanges.

**Known limitation: swapping the battery.** The pack is 138 × 42 mm and posts block every way out: the front and rear post pairs leave 22 mm, the hip posts 41 mm, the centre posts 34 mm, and the top plate is solid. Taking the pack out means opening the body (the tray screws are under the top plate). Version 3, when the body is built, should add a hatch in the top plate over the pack (the Pi plate would move to other grid holes) or an open end; until then plan to charge the pack in place or open the body.

**The same design in the simulator.** `python3 tools/generate_cad.py` also writes visual meshes of every part
(and the three servos as envelopes) into `ros2_ws/src/bemu_description/meshes/` with a manifest
(`meshes.yaml`) saying which link of the model each mesh belongs to. The MuJoCo model and the URDF draw them
(RViz and the viewer), while the capsules and box the physics uses stay as hidden collision shapes; masses,
centres of mass and inertias come from the same CAD. Pictures: `docs/images/robot-stand-*.png`.

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
| 3 | Body (including the battery tray and the Pi plate), five more legs, spares | 3.2 |
| Later | One adapter plate per new module | Each version |

## Printability and the first order (version 1.5c)

`python3 tools/generate_cad.py` writes `cad/export/printability.md` next to the print order. It is
computed from the CAD, not typed: for every printed part the size in its print orientation, the
direction along its print axis that needs less support, the area of surfaces facing down steeper than
45 degrees above the first layer, whether that needs supports (over about 25 mm² of overhang), the
mass of one part at 60% fill (an estimate), and the material for the whole robot. The `print_order.md`
table shows the same orientation and a Supports column per file.

Results at version 1.5c (200 × 200 mm bed assumed):

| Question | Answer |
|---|---|
| Does every part fit a 200 × 200 mm bed? | Yes. The largest footprint is 190 × 80 mm (the centre plates); the plates are split at x = ±40 mm for this. The tallest part is the 50 mm post |
| Which parts need supports? | A few, with small overhangs: the coxa bracket (about 195 mm²), the tibia plates (about 40 mm² each), the foot bridge (about 40) and the foot's dome (about 100). The foot is printed as modelled, dome down, stem up. All plates, posts, spacers and splices print without |
| How much material? | PETG about 752 g and TPU about 7 g for the robot (with one spare of every leg part); about 864 g of PETG with 15% for supports, purge and a failed print: one 1 kg spool |
| What must the service offer? | 200 × 200 × 80 mm build volume, PETG (or ABS/ASA/nylon), 0.2 mm layers; TPU optional (rubber tips can replace it) |

The first order is the tolerance coupon alone. `docs/reports/v1_5-print-quote-request.md` is the
request to send to two or three services (English and Arabic); `docs/reports/v1_5-purchase-list.md`
is the priced draft of everything to buy, with the import notes and the arrival checks.

The battery tray and the Pi plate are drawn for the listed dimensions of the Ovonic pack and the Pi (see above); both must be re-checked against the parts when they arrive (the library entries are `unverified`).
Left for version 2: hard stops and cable slots, because the servo's wire exit is not in the verified data and the stops need the first real leg.

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
| The print service's build size and materials | Quotes requested in version 1.5c (question Q5); the design fits a 200 × 200 mm bed |
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
