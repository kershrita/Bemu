# 15 — Risks and open questions

## Questions for the owner

### Answered on 2026-11-14

| # | Question | Answer | Effect |
|---|---|---|---|
| Q1 | What is the development PC? | Windows 11, Intel Core i9 (14900 series), 32 GB memory, NVIDIA RTX 5060 Ti with 16 GB | Ubuntu 26.04 under WSL 2; the graphics card is used for training in version 7 |
| Q2 | Is there a 3D printer? | No; designs can be sent to someone to print | Parts are designed for a print service |
| Q3 | Code-driven or visual CAD? | The designer's choice | build123d |
| Q4 | The robot's name? | Bemu | Package prefix `bemu_` |

### Still open

#### Q5 — Which print service, and what can it print?

**Asking:** who will print the parts, the largest part they can print, and which
materials they offer (in particular PETG, and a flexible material for feet).

**Why it matters:** the largest part size decides whether the body plates are one
piece or split. The materials decide the foot design.

**Needed by:** step 1.5.6 of the roadmap. It does not block approval or versions 0
and 1.

**Default:** 200 × 200 mm bed; PETG or tough PLA; bought rubber foot tips.

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
| R15 | 3D viewers under WSL fall back to slow software rendering instead of the NVIDIA card | Medium | Low | Checked in step 0.2; dual-boot Ubuntu as the fallback |
| R16 | ROS 2 on the PC (inside WSL) and on the Pi cannot find each other across WSL's virtual network | Medium | Low | Change WSL's networking mode; tested in version 2; dual-boot Ubuntu as the fallback |
| R17 | Ordering prints makes each design iteration slow, and a wrong part costs a re-order | High | Medium | Tolerance test piece first; fit checked in CAD; complete batches; spares of small loaded parts; a written print specification with every order |

## Assumptions

Things taken as true without confirmation. If one is wrong, the design is revisited.

| # | Assumption | Affects |
|---|---|---|
| A1 | The Pi 5 has at least 4 GB of memory | Running the full stack on the robot |
| A2 | Ubuntu 26.04 under WSL 2 on the Windows PC is adequate for simulation and development | Versions 0 and 1 |
| A3 | Indoor use on flat, hard floors until version 5 | Foot design, gait defaults |
| A4 | A print service is available with acceptable price and turnaround | Version 2 onward |
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
| Which print service; its build size and materials | Step 1.5.6 |
| First perception sensor | Start of version 6 |
| Training framework and hardware | Start of version 7 |
| Order of the version 8+ capabilities | After version 6 |

## References

| Source | Used here for | Checked |
|---|---|---|
| [ROS 2 Lyrical vs Jazzy](https://openelab.io/cs/blogs/learn/ros-2-lyrical-vs-jazzy) | Lyrical maturity and the Jazzy fallback (risk R3) | 2026-11-13 |
| [mujoco_ros2_control on ROS Index](https://index.ros.org/r/mujoco_ros2_control/) | The MuJoCo bridge (risk R4) | 2026-11-13 |
| [Feetech STS3215, Servo Database](https://servodatabase.com/servo/feetech/sts3215) | Servo voltage variants (risk R7) | 2026-11-13 |
| [Powering a Raspberry Pi 5, Pololu forum](https://forum.pololu.com/t/powering-a-raspberry-pi-5/26012) | Pi 5 supply under load (risk R8) | 2026-11-13 |
| [WSL visualization, NVIDIA forum](https://forums.developer.nvidia.com/t/wsl-visualization/294703) | WSL graphics falling back to software rendering (risk R15) | 2026-11-14 |

The full list for the project is in [References](references.md). A dash means the source
is listed for reference and was not re-read on a specific date.
