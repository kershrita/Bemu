# 10 — Components

What to buy, in what order, and why. Purchases are grouped into stages tied to the
roadmap. A stage is bought only after the gate before it has passed.

## About the prices

- Prices are in US dollars and are **indicative**, to compare options and size the
  budget.
- Servo prices were checked against online listings on 2026-11-13; sources are in
  [References](references.md).
- All other prices are rough figures from general knowledge and are **not
  verified**. Step 1.5.7 of the roadmap re-prices everything with real suppliers.
- Shipping, import duty and local availability are not included and can change the
  totals substantially.
- Printing is not priced. Parts are made by a print service, and the cost needs a
  quote once the part files exist.

## Summary

| Stage | Roadmap version | What it enables | Indicative cost |
|---|---|---|---|
| 0 | 0, 1, 1.5 | Simulation and CAD | $0 |
| 1 | 2 | One real leg on the bench | $170–240 + printing |
| 2 | 3 | The full walking robot | $505–745 + printing |
| 3 | 4 | Balance sensing | $10–25 |
| 4 | 5 | Foot contact | $0–10 |
| 5 | 6 | Perception | $25–100 |
| 6 | 8+ | Extras | Varies |
| | | **Through a walking robot (stages 1–2)** | **about $675–985, plus printing** |

Stage totals count the "Must" items only and exclude printing.

---

## Stage 0 — Nothing to buy

| Priority | Item | Have it? |
|---|---|---|
| Required | Raspberry Pi 5 | Yes |
| Required | Development PC: Windows 11, Core i9, 32 GB memory, RTX 5060 Ti 16 GB, running Ubuntu under WSL 2 | Yes |
| Required | Software: ROS 2, MuJoCo, CAD | Free |

---

## Stage 1 — One real leg

**Gate:** version 1.5 passed, servo chosen from the torque report.

| Priority | Item | Qty | Indicative cost | Notes |
|---|---|---|---|---|
| Must | Smart bus servo (chosen model) | 4 | $100–130 | 3 for the leg, 1 spare |
| Must | USB bus-servo adapter for that servo family | 1 | $5–15 | Also used from the PC for setup |
| Must | 12 V bench power supply with current limit, 10 A or more | 1 | $25–40 | Safer than a battery for first tests |
| Must | Pi 5 active cooler | 1 | $5–10 | |
| Must | Official Pi 5 27 W power supply | 1 | $12–15 | If not already owned |
| Must | microSD card, 64 GB or larger, fast class | 1 | $10–15 | If not already owned |
| Must | Print order: tolerance test piece, then the parts for one leg | 2 orders | Quote needed | PETG or tough PLA |
| Must | M3 screws, heat-set inserts, assortment | 1 | $10–15 | |
| Should | Multimeter | 1 | $15–25 | If not already owned |
| Should | Soldering iron | 1 | $15–25 | For threaded inserts and wiring, if not already owned |
| Should | Spare servo cables, extension leads | few | $5–10 | |
| Could | Rubber foot tip, or a flexible printed foot if the service offers it | 1 | $1–5 | Can wait until stage 2 |

---

## Stage 2 — The full robot

**Gate:** version 2 passed: one real leg matches the simulation.

| Priority | Item | Qty | Indicative cost | Notes |
|---|---|---|---|---|
| Must | Smart bus servo (same model) | 16 | $400–530 | 15 for the remaining legs, 1 more spare |
| Must | 3-cell lithium battery, 4000–5000 mAh, XT60 | 1 | $30–50 | |
| Must | Balance charger for lithium packs | 1 | $25–50 | |
| Must | Fire-resistant charging bag | 1 | $5–10 | |
| Must | Step-down converter, 5.1 V, rated well above 5 A | 1 | $5–15 | For the Pi |
| Must | Main switch, fuse holder, 20 A fuses | 1 set | $5–10 | |
| Must | Servo bus hub or distribution board | 1 | $5–15 | Or hand-wired |
| Must | Silicone wire (14, 16, 20 AWG), XT60 connectors | 1 set | $10–20 | |
| Must | Bulk capacitor for the servo rail | 1 | $2–5 | |
| Must | Print order: body, five more legs, trays, spares | 1 order | Quote needed | |
| Must | Rubber foot tips, or flexible printed feet | 6 | $5–15 | |
| Must | More M3 hardware; bearings or shoulder screws for rear pivots | 1 set | $15–25 | |
| Should | Low-voltage cell alarm | 1 | $3–5 | Independent backup |
| Should | Second step-down converter for the add-on 5 V rail | 1 | $5–10 | Reserves capacity for later modules |
| Should | Gamepad | 1 | $20–40 | If not already owned; keyboard works meanwhile |
| Could | Second battery | 1 | $30–50 | Longer sessions |
| Could | NVMe drive and adapter for the Pi | 1 | $30–60 | Faster, more robust storage |

---

## Stage 3 — Balance (version 4)

| Priority | Item | Qty | Indicative cost | Notes |
|---|---|---|---|---|
| Must | Inertial sensor (IMU) breakout, 9-axis with onboard fusion | 1 | $10–25 | For example a BNO08x-class board; simpler 6-axis boards cost less but need more software |

## Stage 4 — Touch (version 5)

| Priority | Item | Qty | Indicative cost | Notes |
|---|---|---|---|---|
| Try first | Servo load readings | — | $0 | No hardware |
| If needed | Micro switches or force-sensitive pads | 6 | $5–10 | One per foot |

## Stage 5 — Perception (version 6)

Choose one to start.

| Option | Indicative cost | Good for | Trade-off |
|---|---|---|---|
| Pi camera module | $25–35 | Seeing objects and people; AI vision | Needs more processing; no direct distance |
| 2D laser scanner | $70–100 | Obstacle avoidance and mapping | Sees one horizontal slice only |
| Time-of-flight distance sensors | $5–15 each | Simple forward obstacle stop | Very coarse |

## Stage 6 — Extras (version 8+)

Unpriced; each is chosen when its version is planned.

- Microphone and speaker for voice.
- AI accelerator for the Pi 5.
- Depth camera.
- Status display or light ring.
- Small arm or gripper servos.
- Charging dock contacts.

---

## The servo choice

This is the decision that sets most of the budget. It is made at the end of
version 1.5 using measured simulation torque. The table below uses the hand
estimate from [08 — Mechanical design](08-mechanical-design.md): **design peak
1.2 N·m, design average 0.6 N·m**.

| Candidate | Stall torque | Rated torque | Feedback | Supply | Unit price | For 20 | Peak as % of stall | Passes? |
|---|---|---|---|---|---|---|---|---|
| **Feetech STS3215, 12 V version** | 2.94 N·m (30 kg·cm) | 0.98 N·m (10 kg·cm) | Position, speed, load, voltage, current, temperature | 12 V | $26–33 | $520–660 | 41% | Yes |
| Feetech STS3215, 7.4 V version | 1.91 N·m (19.5 kg·cm) | 0.49–0.64 N·m (5–6.5 kg·cm; listings differ) | Same | 6–8.4 V | $25–33 | $500–660 | 63% | No, marginal |
| Dynamixel XL430-W250-T | 1.5 N·m at 12 V | Not published as one number | Position, speed, load, and more | 6.5–12 V | about $50 | about $1,000 | 80% | No |
| Dynamixel XM430-W210 | 3.0 N·m at 12 V | Not published as one number | Position, speed, current, and more | 10–14.8 V | Several times the XL430; to be priced | — | 40% | Yes, at high cost |
| Hobby PWM servo (no feedback) | Varies | Varies | **None** | 5–7.4 V | $5–15 | $100–300 | — | **Ruled out** |

**Simulated check (version 1 torque report).** The hand estimate above is 1.2 N·m peak. The
simulated walks give a worst peak of 1.26 N·m, 43% of the STS3215's stall torque, with a worst
RMS of 0.43 N·m against the 0.98 N·m rated torque, across every gait, direction and mass in the
matrix (`docs/reports/v1-torque-report.md`). The STS3215 12 V passes
on these numbers, with two cautions. First, the peaks barely change with total mass (the worst
peak is the same at 2.0, 2.2 and 2.5 kg) and come from swing and start-up transients of the
simulated position loop (kp 20, kv 0.5), not from carrying the body, so a real servo's stiffer
internal loop could peak differently; version 2's one-leg measurement checks this. Second, joint
speed reaches 83% of the 4.7 rad/s *no-load* speed (wave sideways) and 73% for ripple, and the
loaded speed is lower and unknown, so tracking headroom is thin. The report's power figures
(about 21–39 W for 18 servos while standing and walking) use the Feetech ST-3215-C047 (12 V)
datasheet's stall current (2.7 A) and idle current (30 mA); the 12 V supply is the servo's
nominal voltage, so the real battery voltage will move them. The estimate assumes supply current
equals winding current, so it is an upper bound. Size the battery and wiring for the stall
current as well (18 × 2.7 A is the worst case if every servo stalls), not only for the walking
average. Datasheet values, same source: no-load running current 180 mA, rated current 900 mA,
winding resistance 1.0 Ω, Kt 11 kg·cm/A, operating voltage 4–14 V, mass 55 g ± 1 g.

### Reading the table

- The **12 V STS3215** is the first candidate: it passes the torque rule with
  margin, it reports everything the software needs, and it is the least expensive
  option that passes.
- The **7.4 V STS3215** fails the peak rule at the design mass. It becomes viable
  if the robot is made lighter or the femur shorter, which would also allow a
  smaller 2-cell battery.
- **Dynamixel** servos have the best documentation and an official ROS 2 driver.
  The affordable model is too weak for this robot's size; the strong models
  multiply the budget.
- **Hobby servos** are ruled out regardless of price. Without feedback, the
  simulation cannot be matched to the robot and learned walking is not realistic.
  See [decision 0004](decisions/0004-smart-bus-servos.md).

### Things to confirm before ordering servos

- [ ] The simulation torque report agrees with the hand estimate.
- [ ] The exact sub-model: the STS3215 is sold in several variants with different
      voltages and gear ratios under similar names.
- [ ] Maximum input voltage against a full 3-cell battery (12.6 V).
- [ ] A working ROS 2 driver, or a documented protocol and vendor library.
- [ ] Mounting hole pattern and rear pivot dimensions, for the brackets.
- [ ] Availability and delivered price from a supplier that ships locally.

---

## Priority across the whole project

If money or time is short, this is the order that protects the plan.

| Rank | Item | Why it ranks here |
|---|---|---|
| 1 | Servos with feedback | Everything depends on them; the wrong choice is the most expensive to undo |
| 2 | Safe power: bench supply, fuse, switch, proper charger | Protects the servos, the Pi and the builder |
| 3 | Pi 5 essentials: cooler, supply, storage | Reliable computing |
| 4 | Printed parts and hardware | The body |
| 5 | Battery | The robot can walk tethered without it |
| 6 | Tilt sensor | First capability beyond walking; cheap |
| 7 | Foot contact | Often free through servo load |
| 8 | Camera or scanner | First real autonomy |
| 9 | Everything else | Chosen when its version is planned |

## What not to buy yet

- A full hexapod kit. The point of the plan is that the simulation chooses the
  geometry and servos.
- All 18 servos at once. Buy four, prove one leg, then buy the rest.
- A camera, scanner or AI accelerator before the robot walks.
- A servo driver board for PWM servos. Bus servos do not use one.

## References

| Source | Used here for | Checked |
|---|---|---|
| [Feetech STS3215, AIFITLAB](https://aifitlab.com/collections/servo-motor/products/feetech-sts3215-servo-motor) | STS3215 7.4 V and 12 V figures; unit price | 2026-11-13 |
| [Feetech STS3215, Core Electronics](https://core-electronics.com.au/feetech-sts3215-smart-servo.html) | STS3215 specifications; unit price | 2026-11-13 |
| [Feetech STS3215, Servo Database](https://servodatabase.com/servo/feetech/sts3215) | STS3215 sub-model codes and voltages | 2026-11-13 |
| [Feetech STS3215, Switch Science](https://www.switch-science.com/products/7489) | STS3215 feedback values | 2026-11-13 |
| [Dynamixel XL430-W250 manual](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/) | XL430-W250 torque by voltage; the XM430-W210 comparison | 2026-11-13 |
| [XL430-W250-T distributor listing](https://ichome.com/product-detail/robotis/902-0135-000) | XL430-W250-T current unit price | 2026-11-13 |
| [XL430 price notice, ROBOTIS](https://www.robotis.us/robotis-blog/xl430-price-change-xseries-accessories/) | XL430 list price history | 2026-11-13 |
| [Powering a Raspberry Pi 5, Pololu forum](https://forum.pololu.com/t/powering-a-raspberry-pi-5/26012) | Step-down converter rating for the Pi 5 | 2026-11-13 |

The full list for the project is in [References](references.md). A dash means the source
is listed for reference and was not re-read on a specific date.
