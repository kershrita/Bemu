# 0005 — One parameter file for CAD, model and maths

**Status:** Accepted
**Date:** 2026-11-13
**Decided by:** Owner, by approving the design on 2026-11-14

## Context

The owner wants the 3D parts to be flexible. A robot's dimensions appear in three
places: the printed parts, the simulation model and the leg maths. If they are
entered separately they drift apart, and a mismatch between the model and the real
legs is exactly the failure the simulation-first plan must avoid.

## Decision

All dimensions, masses and joint limits live in one file,
`config/robot_params.yaml`. The CAD scripts, the model generator and the kinematics
library all read it. No dimension is typed anywhere else.

## Alternatives considered

| Alternative | Why not |
|---|---|
| Dimensions in the CAD, exported to the model | Makes the CAD tool the source; harder to change a number quickly and re-simulate before any CAD exists |
| Dimensions in the URDF, copied to CAD by hand | Hand copying is where drift starts |
| Separate values kept in sync by discipline | Works until it does not |

## Consequences

- Resizing a leg is one edit, after which parts, model and maths agree.
- The simulation can explore leg lengths before any CAD is drawn.
- A model generator and a validation step for the file must be written in
  version 0.
- The CAD must be able to read the file. This favours code-driven CAD
  ([decision 0007](0007-code-driven-cad.md)).
- The file's structure becomes a contract; changing it touches three consumers.

## References

No external sources. This document draws only on the owner's decisions and the
other documents in this repository. The project's full list is in
[References](../references.md).
