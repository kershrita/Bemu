# 0001 — Simulation and design before hardware

**Status:** Accepted
**Date:** 2026-11-13
**Decided by:** Owner

## Context

The owner has a Raspberry Pi 5 and no other robot hardware. Money is available but
committed elsewhere for now. Eighteen servos are the largest cost of a hexapod, and
the right servo depends on leg lengths and body mass that are not yet known.

## Decision

Versions 0, 1 and 1.5 are simulation and CAD only. The robot stands and walks in a
simulator before any part is bought. The simulation's measured joint torques choose
the servo. Purchases are staged behind gates in the roadmap.

## Alternatives considered

| Alternative | Why not |
|---|---|
| Buy a hexapod kit and write software for it | Fixes the geometry and servos before anything is known; works against the goal of a flexible, self-designed body |
| Buy servos first, design around them | Risks buying servos that are too weak or needlessly strong |
| Design the CAD first, simulate later | Leg lengths would be guesses; the simulation is what tests them |

## Consequences

- No cost until the design has been tested.
- Leg sizes and servo strength are chosen from data.
- The simulation must later be matched to the real robot. That work is scheduled in
  version 2 and is the main technical risk (R1).
- The first physical result arrives later than with a kit.

## References

No external sources. This document draws only on the owner's decisions and the
other documents in this repository. The project's full list is in
[References](../references.md).
