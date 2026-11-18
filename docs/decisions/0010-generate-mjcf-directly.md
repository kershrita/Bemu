# 0010 — Generate the MuJoCo model directly from the parameter file

**Status:** Accepted
**Date:** 2026-11-18
**Decided by:** Designer, within the version 0 plan

## Context

The simulator needs a MuJoCo model (MJCF) and ROS needs a URDF. The parameter file is
the only source of dimensions ([decision 0005](0005-single-parameter-file.md)). The
`mujoco_ros2_control` bridge includes a URDF-to-MJCF converter, but its package page
marks it highly experimental. The roadmap said to try it first and fall back to
writing both formats directly.

## Decision

`bemu_description` writes both the MJCF and the URDF itself, from the parameter file
and the simulated servo model. The URDF carries the `ros2_control` block that points
the bridge at the generated MJCF. Neither format is converted from the other.

## Alternatives considered

| Option | Why not |
|---|---|
| Convert the URDF with the bridge's tool | Marked highly experimental; the model needs position actuators named exactly as their joints, a keyframe and tuned gains, which a converter may not express |
| Hand-written MJCF | Dimensions would live in two places and drift from the parameter file |

## Consequences

- One generator, with tests that compare the model's foot position to the leg maths.
- The two formats can disagree if one generator changes alone. Tests cover joint
  limits and neutral angles in both.
- Adding the full 18-joint body in version 1 extends the same generator.

## References

No external sources beyond those listed in
[06 — Simulation](../06-simulation.md). The project's full list is in
[References](../references.md).
