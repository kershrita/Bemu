# Decision records

One short record per significant decision: what was chosen, what else was
considered, and what follows from it.

## Index

| # | Decision | Status | Decided by |
|---|---|---|---|
| [0001](0001-simulation-first.md) | Simulation and design before hardware | Accepted | Owner |
| [0002](0002-ros2-plus-mujoco.md) | ROS 2 backbone with MuJoCo physics | Accepted | Owner |
| [0003](0003-programmed-gait-before-learning.md) | Programmed walking first, learned walking later | Accepted | Owner |
| [0004](0004-smart-bus-servos.md) | Smart bus servos with feedback | Accepted | Owner, with the design approval |
| [0005](0005-single-parameter-file.md) | One parameter file for CAD, model and maths | Accepted | Owner, with the design approval |
| [0006](0006-ros2-distribution.md) | ROS 2 Lyrical Luth, with Jazzy as fallback | Accepted | Owner, with the design approval |
| [0007](0007-code-driven-cad.md) | Code-driven CAD with build123d | Accepted | Designer, delegated by the owner |
| [0008](0008-wsl2-development.md) | Develop under WSL 2 on the Windows PC | Accepted | Owner, with the design approval |
| [0009](0009-printing-by-service.md) | Parts are printed by a service | Accepted | Owner |
| [0010](0010-generate-mjcf-directly.md) | Generate the MuJoCo model directly from the parameter file | Accepted | Designer |
| [0011](0011-first-purchase-sourcing.md) | Where the first purchase comes from | Proposed | Designer, owner approves at Purchase gate 1 |
| [0012](0012-docker-dev-container.md) | A Docker container for development and validation inside WSL 2 | Accepted | Owner |

## Status values

| Status | Meaning |
|---|---|
| Proposed | Written into the design, not yet confirmed by the owner |
| Accepted | Confirmed |
| Superseded | Replaced by a later record, which is linked |

## Rules

- A record is never rewritten after it is accepted. A changed decision gets a new
  record that supersedes the old one.
- Write a record when a choice between real alternatives is made, when a message
  contract changes, or when a lower layer is edited by a later version.
- Keep it to one page.

## Template

```markdown
# NNNN — Title

**Status:** Proposed | Accepted | Superseded by NNNN
**Date:** YYYY-MM-DD
**Decided by:** Owner | —

## Context
What situation forces a choice.

## Decision
What was chosen.

## Alternatives considered
What else was on the table, and why it was not chosen.

## Consequences
What this makes easier, what it makes harder, what it commits the project to.
```

## References

No external sources. This document draws only on the owner's decisions and the
other documents in this repository. The project's full list is in
[References](../references.md).
