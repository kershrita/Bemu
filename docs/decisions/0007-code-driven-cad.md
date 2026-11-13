# 0007 — Code-driven CAD with build123d

**Status:** Accepted
**Date:** 2026-11-14
**Decided by:** Designer, delegated by the owner

## Context

The owner wants the 3D parts to be flexible to change, and left the choice of CAD
tool to the designer. The project already has a single parameter file that drives
the simulation model and the leg maths
([decision 0005](0005-single-parameter-file.md)). Parts are printed by a service,
so every order needs part files, a print specification and an orientation sheet
that match the current design.

## Decision

Parts are written as Python using build123d. The scripts read
`config/robot_params.yaml` and export STL and STEP files, part masses and
inertias, and the documents that accompany a print order.

## Alternatives considered

| Alternative | Strength | Why not first choice |
|---|---|---|
| FreeCAD | Visual; faster for shaping a complex bracket by eye; supports parameter spreadsheets | The link to the parameter file is manual or scripted; design files do not show readable changes in git |
| A cloud or commercial CAD program | Polished modelling tools | Licence terms and cloud storage; no direct link to the parameter file; not text in the repository |

## Consequences

- Changing a parameter regenerates every affected part, the model and the print
  order documents in one step.
- The design is text in git: changes can be reviewed and reverted.
- Part mass and inertia flow straight into the simulation.
- Shaping parts in code is slower than drawing them for one-off organic shapes. If
  that becomes a real obstacle, a mixed approach is used: code for parts that
  scale with parameters, FreeCAD for the exception (risk R13).
- A 3D viewer for the scripts is part of the development setup.

## References

| Source | Used here for | Checked |
|---|---|---|
| [build123d documentation](https://build123d.readthedocs.io/) | The chosen tool | — |
| [FreeCAD](https://www.freecad.org/) | The alternative | — |

The full list for the project is in [References](../references.md). A dash means the source
is listed for reference and was not re-read on a specific date.
