# 0003 — Programmed walking first, learned walking later

**Status:** Accepted
**Date:** 2026-11-13
**Decided by:** Owner

## Context

The owner's first question was how to teach the robot to balance and move. Two
routes exist: write the motion from leg geometry and gait timing, or train a
network by trial and error in simulation.

A six-legged robot can always keep three feet on the ground in a triangle, so it is
stable without any learned balance.

## Decision

Version 1 walks with programmed motion: inverse kinematics plus gait patterns.
Learned walking is version 7. It is built as a second walking module with the same
inputs and outputs, selected by a launch argument.

## Alternatives considered

| Alternative | Why not |
|---|---|
| Learned walking from the start | Needs an accurate model, feedback servos and a tuned training setup before anything moves; failures are hard to diagnose; no baseline to compare with |
| Programmed walking only, never learned | Closes off adaptive and rough-terrain walking, and an area the owner is interested in |

## Consequences

- Walking works early and every behaviour can be explained.
- The programmed walker becomes the baseline and the safety fallback for the
  learned one.
- Preparation for learning is spread through earlier versions at little cost:
  ROS-free maths libraries, MuJoCo as the simulator, feedback servos, measured
  servo behaviour.
- The walker's interface must stay narrow and stable, so that replacing it is a
  one-module change.

## References

| Source | Used here for | Checked |
|---|---|---|
| [mjlab technical report](https://arxiv.org/pdf/2601.22074v2) | What a learned-walking setup involves | 2026-11-13 |

The full list for the project is in [References](../references.md). A dash means the source
is listed for reference and was not re-read on a specific date.
