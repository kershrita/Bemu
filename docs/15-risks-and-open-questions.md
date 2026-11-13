# 15 — Risks and open questions

## Open questions for the owner

These change what gets built. Each has a default that applies if it is left
unanswered.

### Q1 — What is the development PC?

**Asking:** its operating system, and whether it has an NVIDIA graphics card.

**Why it matters:** ROS 2 runs natively only on Ubuntu; on anything else it runs in
a container or under WSL, which changes the setup steps. A graphics card decides
how learned walking is trained in version 7.

**Default:** Ubuntu 26.04, natively or in a container. Training approach decided at
version 7.

### Q2 — Is there access to a 3D printer?

**Asking:** whether one is available, and its build size.

**Why it matters:** the build size sets the largest single part. Without a printer,
parts are ordered from a print service, which changes cost and the pace of design
iteration.

**Default:** design for a 200 × 200 mm bed, splitting larger parts.

### Q3 — Code-driven CAD or visual CAD?

**Asking:** build123d (parts written as Python) or FreeCAD (parts drawn in a
program).

**Why it matters:** it decides how the "flexible parts" requirement is met and what
the CAD folder contains.

| | build123d | FreeCAD |
|---|---|---|
| Reads the parameter file | Directly | Through a script or by hand |
| Version control | Text; changes are readable | Binary-like files; changes are not readable |
| Shaping complex brackets | Slower, by code | Faster, by eye |
| Learning curve for a programmer | Low | Moderate |

**Default:** build123d.

### Q4 — Does the robot have a name?

**Why it matters:** package and topic names. Cheap to decide now, tedious to change
later.

**Default:** packages are named `hexapod_*`.

---

## Risks

Likelihood and impact are rated low, medium or high.

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R1 | The simulation does not match the real robot, so tuned behaviour fails on hardware | Medium | High | Feedback servos; servo measurement in version 2; CAD-derived masses; comparison tests at every hardware version |
| R2 | The servos are too weak or overheat | Medium | High | Torque report before buying; factor-of-two margin on peak; buy 4 before buying 20; geometry can be shortened by parameter |
| R3 | A needed package is missing or immature on ROS 2 Lyrical | Medium | Low | Fallback to Jazzy; nothing in the code is specific to Lyrical |
| R4 | The MuJoCo bridge cannot express something the model needs | Low | Medium | Generate the MuJoCo model directly from the parameter file; the bridge only needs to load it |
| R5 | No good ROS 2 driver exists for the chosen servo | Medium | Medium | Evaluate community drivers in version 2; otherwise write a thin plugin on the vendor's library. Budgeted as step 2.3 |
| R6 | The servo bus is too slow for 18 servos at the control rate | Low | Medium | Group read and write commands; low-latency adapter settings; split into two buses if needed |
| R7 | A full 3-cell battery (12.6 V) exceeds the servo's maximum input | Medium | Medium | Check the datasheet before the first battery run; regulate the servo rail if needed |
| R8 | The Pi 5 browns out when the servos draw current spikes | Medium | Medium | Separate converter for the Pi; bulk capacitor on the servo rail; short thick leads; tested under load |
| R9 | Printed parts flex or crack at the hips | Medium | Medium | Support joints on both sides; print orientation rules; tougher material for the hip bracket if tests show a need |
| R10 | Component prices or availability differ from the estimates | High | Medium | Re-price at step 1.5.7 with real suppliers before committing; the staged plan limits exposure |
| R11 | Scope creep: adding sensors before the robot walks | Medium | Medium | The roadmap order and purchase gates; "what not to buy yet" list |
| R12 | Lithium battery misuse | Low | High | Balance charger, charging bag, voltage cut-offs, never charged unattended |
| R13 | Code-driven CAD turns out too slow for bracket design | Medium | Low | Mixed approach: code CAD for scaling parts, visual CAD for one-off shapes |
| R14 | The project stalls after a long pause | Medium | Medium | These documents; a version tag and a written state at every stop |

## Assumptions

Things taken as true without confirmation. If one is wrong, the design is revisited.

| # | Assumption | Affects |
|---|---|---|
| A1 | The Pi 5 has at least 4 GB of memory | Running the full stack on the robot |
| A2 | A PC is available that can run Ubuntu in some form | Versions 0 and 1 |
| A3 | Indoor use on flat, hard floors until version 5 | Foot design, gait defaults |
| A4 | Parts can be printed, by the owner or a service | Version 2 onward |
| A5 | Wi-Fi is available where the robot is operated | Remote control and monitoring |
| A6 | One builder, working part-time | Preference for existing tools; no fixed deadlines |

## Estimates awaiting measurement

| Estimate | Current value | Replaced in |
|---|---|---|
| Robot mass | 2.2 kg | Version 1.5 (CAD), version 3 (scale) |
| Peak femur torque | 1.2 N·m | Version 1 (simulation), version 2 (measurement) |
| Average servo current while walking | 0.4 A each | Version 3 |
| Running time on battery | 20–30 min | Version 3 |
| Servo bus rate | 50–100 per second | Version 2 |
| Simulated servo stiffness and damping | Placeholders | Version 2 |
| Component prices other than servos | Unverified | Step 1.5.7 |

## Things deliberately not decided yet

| Topic | Decided at |
|---|---|
| Final servo model | End of version 1.5 |
| Final leg lengths and body size | Version 1.5 |
| Which community servo driver, or a custom one | Version 2 |
| First perception sensor | Start of version 6 |
| Training framework and hardware | Start of version 7 |
| Order of the version 8+ capabilities | After version 6 |
