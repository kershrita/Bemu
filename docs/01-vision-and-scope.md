# 01 — Vision and scope

## What this is

A six-legged walking robot with a Raspberry Pi 5 as its computer, built as a long
project that grows one capability at a time. It starts as a robot that can stand and
walk, and each later version adds a sensor, a module or a behaviour.

## Why it is built this way

Stated by the owner:

- Build it **incrementally**. Each version adds a new functionality.
- Start by getting it to **balance and move**.
- Then **add sensors or modules** and implement what they enable, expanding it step
  by step.
- The **system must scale** to any functionality added later.
- The **3D-printed parts must be flexible** to change.
- **No hardware spending for now.** Start with the simulator and the design.
- **Plan well first**: list and prioritize the components before building.
- Version 1 walks with **programmed** motion. Learned walking comes later.

## What success looks like

| Horizon | Success means |
|---|---|
| Version 1 | The robot walks in simulation under keyboard or gamepad control: forward, backward, sideways, turning, with adjustable body height and tilt |
| First hardware | One real leg follows the same commands as the simulated leg, using the same code |
| Full robot | The assembled robot walks on a flat floor on battery power |
| Long term | Adding a sensor or behaviour touches only its own module, plus one mounting part and one launch entry |

## Goals

1. A walking hexapod whose every version is usable on its own.
2. One code path for simulation and the real robot. Only the lowest layer differs.
3. A software structure where modules are added without editing existing ones.
4. A body that can be resized or re-shaped by changing parameters, not redrawing parts.
5. A purchase plan staged so that money is spent only after the design it depends on
   has been tested in simulation.
6. Documentation good enough that the project can be picked up after a long pause.

## Non-goals

These are deliberately left out. They can be reconsidered in a later version.

- Outdoor or rough-terrain operation in early versions. Flat indoor floors first.
- Carrying heavy payloads. The design budget is about 300 g of add-on modules.
- Running or jumping. The robot always keeps at least three feet on the ground.
- Waterproofing or dust protection.
- A custom circuit board. Off-the-shelf boards and wiring until the design settles.
- A manipulator arm or gripper in the first hardware versions.
- Learned walking before programmed walking works on the real robot.

## Anti-goals

Outcomes that would make the project a failure even if the robot walks.

- **A simulation that does not match the robot.** If code tuned in simulation has to
  be rewritten for hardware, the simulation-first plan has failed.
- **A monolith.** If adding a camera means editing the walking code, the structure
  has failed.
- **Parts that cannot be changed.** If lengthening a leg means redrawing five parts
  by hand, the mechanical design has failed.
- **Buying ahead of knowledge.** Parts bought before the simulation says what is
  needed, then left unused.
- **A robot that is unsafe to be near.** Lithium batteries and eighteen strong
  servos need fuses, limits and a way to cut power.

## Who it is for

One builder, working alone, with a software and AI engineering background. The
documents assume comfort with Python, Linux and git, and explain robotics terms
where they first appear. See the [glossary](glossary.md).
