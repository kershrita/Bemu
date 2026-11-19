# Version 1.5: Printable design, exact component fit, and the first purchase list

**Status:** approved by the owner; plans 1.5a, 1.5b and 1.5c are executed (2026-11-19). Open: the owner's measurements, a print-service quote and the purchase approval.

## 1. Intent

Version 1 proved in simulation that the Feetech STS3215 12 V servo can walk the robot. Version 1.5
turns the simulated robot into parts that can be **ordered and assembled without surprises**:
printed brackets, links and body plates whose pockets, holes and spacings match the real
components exactly, and a priced list of components the owner can buy from China, or from
anywhere, and have delivered to Saudi Arabia at good price and quality. It ends at Purchase
gate 1 (the first purchase is approved). The owner's words: use the exact sizes and holes of the
components so they sit in the printed parts and connect together well.

## 2. Goals, non-goals, anti-goals

**Goals**
- A **component library** (`config/components.yaml`): every bought part that touches a printed part
  (servo, horn, screws, nuts, heat-set inserts, bearings, battery, Raspberry Pi 5, regulator,
  servo-bus board, switch, connectors) with its real dimensions, each value carrying a source
  (datasheet page or drawing, link, date) and a status: `verified` or `unverified`.
- Parametric **build123d** part generators read that library and `robot_params.yaml`: leg
  parts (coxa bracket, femur links, tibia, foot), body plates, mounts, a **tolerance coupon**.
- **Fit checks in code**, run in tests: no printed part intersects a component outside its
  designed pocket; clearances and wall thicknesses meet minimums; every screw has edge distance and
  access; every part fits the assumed bed; the joints sweep their full range without parts
  colliding (this also settles the open question "final joint limits").
- Masses, centres of mass and inertias flow from the CAD into the simulation model; the version 1
  tests and the torque report are re-run on it.
- A written **print order** (files, material, settings, orientation, quantities) and a **priced
  component list for Saudi Arabia** with suppliers, shipping and import notes.

**Non-goals:** buying or printing anything; electronics wiring design; a learned or new gait; a
final enclosure or cosmetic design; legs for rough terrain.

**Anti-goals (a failure even if the code runs):** a part that fits in CAD but not in reality
because a dimension was guessed; a component list that names the wrong servo variant (for
example the 7.4 V version of the same-looking servo); an import surprise at the border.

## 3. Constraints

- All dimensions come from `config/` and are never typed into a script. A change to one leg
  length regenerates parts, model and maths with no manual edit (roadmap exit test).
- **No dimension is taken from memory.** A mounting feature built on an `unverified` value makes
  the part generator fail its release check; unverified values are allowed only for drafts.
- Print bed assumed 200 × 200 mm until the print service is chosen (docs/08); larger parts are
  split with a designed joint.
- The robot's design mass stays 2.5 kg; torque criteria are unchanged (peak < 50% of stall,
  RMS < rated), and the V1 caveats on simulator torque peaks and placeholder currents stay in
  the report until version 2 measures the real servo.
- No subagents without the owner asking; geometry kernel and tools run in the cloud session
  (build123d installs from PyPI); the owner's PC is used to view and print-check STEP files.

## 4. Approaches considered

| Approach | What it is | Trade-off |
|---|---|---|
| **A. Verified component library plus code-driven CAD with in-code fit checks (picked)** | Dimensions sit in one file with sources and a verified flag; every printed part is generated; checks enforce fit | Slowest to start, but the owner's main worry (exact sizes and holes) is a test failure, not a hope; one change regenerates everything |
| B. Simple block parts first, refine later | Generate rough brackets quickly, add detail in later passes | Fast feedback, but risks printing a first order from approximate shapes |
| C. Adapt an open-source servo-bracket design (e.g. SO-ARM-style) | Start from a proven geometry for this servo family | Good for cross-checking hole positions; the licence and the leg geometry differ, so used as a reference, not as the base |

**Pick: A**, cross-checked against C for the servo's hole pattern.

## 5. Design

### 5.1 The component library

`config/components.yaml`, loaded by a small `bemu_params`-style loader (SI units, validation).
Each component: name, supplier part numbers, variant (the servo is the **12 V, 1:345, 30 kg·cm**
model; the 7.4 V model has the same body but different torque and wiring), outline, hole list (x,
y, diameter, thread, depth), features (horn spline, rear pivot, connector exit), mass, and
`source` plus `status`. A `check_components` command lists every unverified value; the release
check requires none on any feature a part mounts to.

The **servo** is the critical entry. Its drawing must come from the manufacturer's dimension
drawing or STEP file (Feetech datasheet, or a vendor copy), then be confirmed by the **caliper
measurement of the first servo received** before the final print order.

### 5.2 Parts and checks

Part families as in docs/08 (body plates, hip/coxa bracket, femur links, tibia with foot, mounts).
Joint rules from docs/08 apply: servo is structural; both sides supported; load along layers; hard
stops beyond software limits; cable paths. Checks, all in `pytest`:
1. **Interference:** boolean intersection of every printed part with every component envelope is
   empty except the designed pockets; pocket clearance equals the tolerance allowance.
2. **Fasteners:** hole diameter, counterbore, edge distance ≥ 1.5 × diameter, wall ≥ 4
   perimeters (1.6 mm), insert pockets sized from the library.
3. **Bed:** bounding box of each part in its print orientation ≤ the bed.
4. **Motion:** sweep every joint through its limits; no part-to-part intersection.
5. **Mass properties** exported to JSON and consumed by the model generator.

### 5.3 Tolerance coupon

A small printed piece with servo pockets, hole rows and insert holes at several tolerance steps
(+0.1, +0.2, +0.3 mm). It is the **first order**: the owner measures it, picks the allowance, and
the choice goes into `config/` (docs/08 already requires this).

### 5.4 Sourcing for Saudi Arabia

A `docs/10-components.md` revision gives, per item, two or three real suppliers with current price
(dated), shipping to Saudi Arabia and import notes: official or authorised Feetech sellers (not
unverified marketplace listings, because counterfeit and wrong-voltage servos are common), local
Saudi resellers, AliExpress official stores, Amazon.sa and Noon; for each, what to check on arrival.
Items with import rules (lithium batteries, radio-free parts) get an explicit note. Spares: two
extra servos and one extra hip bracket of each part. Prices are researched at the time of step
1.5.7 and carry a "checked on" date.

## 6. Plans (in order)

1. **1.5a Component library and leg:** components file and loader, servo model and verification
   workflow, leg part generators, fit checks, tolerance coupon, exports.
2. **1.5b Body and the model:** body plates, mounts, mass properties into the simulation, version 1
   tests and the torque report re-run, final joint limits.
3. **1.5c Printing and buying:** printability review, print order, supplier research, priced
   Saudi component list, purchase decision.

## 7. Blocker and risks

| Risk | Plan |
|---|---|
| The servo's exact mounting-hole drawing is not available to the cloud session (the manufacturer and vendor sites are not reachable from here) | The owner supplies the Feetech datasheet / dimension drawing PDF and the STEP file for the 12 V STS3215 (links in the hand-off message), saved under `docs/reference/` |
| The real servo differs from the drawing | First-article caliper measurement before the final print order; tolerance coupon first |
| Print service unknown | Bed 200 × 200; split parts; service chosen in 1.5c |
| Wrong variant or fake servos bought | Library names the exact variant; buy from official/authorised sellers; check on arrival |
| Import problems (batteries, customs) | Documented per item in 1.5c; the battery is chosen with shipping in mind |

## 8. Left to the builder

Part shapes and names; how the library loader is organised; the exact check thresholds not given above;
how the STEP reference is used (as the servo envelope or only to validate hole positions).
