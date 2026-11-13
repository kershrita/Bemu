# 0004 — Smart bus servos with feedback

**Status:** Proposed
**Date:** 2026-11-13
**Decided by:** —

## Context

The robot needs 18 actuators. Hobby servos are cheap and common on small hexapods,
but they accept a target and report nothing back. Smart bus servos cost more and
report position, speed, load, voltage and temperature over a shared data line.

The plan depends on matching the simulation to the real robot, and intends learned
walking later.

## Decision

Use smart serial-bus servos with feedback. The first candidate is the Feetech
STS3215 in its 12 V version. The exact model is confirmed at the end of
version 1.5 from the simulation's torque report, using the decision rule in the
roadmap.

## Alternatives considered

| Alternative | Strength | Why not |
|---|---|---|
| Hobby PWM servos with a driver board | Lowest cost | No feedback: the simulation cannot be matched to reality, overloads go undetected, learned walking is not realistic |
| Dynamixel XL430 | Excellent documentation; official ROS 2 driver | Higher unit price for about half the torque; fails the torque rule at this robot's size |
| Dynamixel XM430 class | Strong, precise, well supported | Multiplies the servo budget several times |
| Feetech STS3215, 7.4 V version | Slightly cheaper; smaller battery | Fails the peak torque rule at the design mass; viable only for a lighter robot |

## Consequences

- The software knows where every joint really is, how hard it is working and how
  hot it is.
- Battery voltage can be read through the servos.
- Wiring is simpler: one data line per leg chain instead of 18 signal wires.
- A ROS 2 driver for this servo family must be found or written (risk R5).
- Higher cost than hobby servos: roughly $520–660 for twenty.
- The servo's maximum input voltage must be checked against a full 3-cell battery
  (risk R7).
