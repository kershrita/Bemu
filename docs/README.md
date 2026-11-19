# Documentation index

Read in this order the first time. After that, each document stands on its own.

## Core

| # | Document | What it answers |
|---|---|---|
| 01 | [Vision and scope](01-vision-and-scope.md) | Why this exists, what counts as success, what is out of scope |
| 02 | [Design](02-design.md) | The approved design: goals, constraints, approaches considered, decisions |
| 03 | [Roadmap](03-roadmap.md) | Versions, what each adds, and the test each must pass |

## Software

| # | Document | What it answers |
|---|---|---|
| 04 | [System architecture](04-system-architecture.md) | The layers, and how simulation and the real robot share one code path |
| 05 | [Software stack](05-software-stack.md) | Packages, topics, naming, repository layout, conventions |
| 06 | [Simulation](06-simulation.md) | How the MuJoCo model is built and kept honest |
| 07 | [Locomotion](07-locomotion.md) | Leg maths, gaits, body pose, balance |
| 11 | [Learning roadmap](11-learning-roadmap.md) | How learned walking is added later |
| 12 | [Adding a module](12-adding-a-module.md) | The checklist for any new sensor or capability |

## Hardware

| # | Document | What it answers |
|---|---|---|
| 08 | [Mechanical design](08-mechanical-design.md) | Geometry, parametric CAD, mounting standard, printing |
| 09 | [Electronics and power](09-electronics-and-power.md) | Wiring, power budget, safety |
| 10 | [Components](10-components.md) | Prioritized list of what to buy, in which stage |

## Working on the project

| # | Document | What it answers |
|---|---|---|
| 13 | [Development environment](13-dev-environment.md) | What to install on the PC and the Pi |
| 14 | [Testing and validation](14-testing-and-validation.md) | How each layer is tested |
| 15 | [Risks and open questions](15-risks-and-open-questions.md) | What could go wrong, what is undecided |
| — | [Version 1 release notes](releases/v1.md), [torque report](reports/v1-torque-report.md) | What version 1 delivers, its known limitations, and the servo evidence |
| — | [Glossary](glossary.md) | Terms used in these documents |
| — | [References](references.md) | External sources, with the date each was checked |
| — | [Decision records](decisions/README.md) | Why each significant choice was made |

## Implementation plans

| Version | Plan | Status |
|---|---|---|
| 0 | [Foundations](plans/2026-11-14-v0-foundations.md) | Written, awaiting review |

One plan per roadmap version, written just before that version starts.

## How these documents are kept up to date

- Every version in the roadmap ends with a documentation pass: anything the version
  changed is updated here before the version is tagged.
- A significant choice gets a decision record in `decisions/`. Records are never
  rewritten; a changed decision gets a new record that supersedes the old one.
- Numbers that came from an estimate are marked **(estimate)** until a measurement
  replaces them.

## References

No external sources. This document draws only on the owner's decisions and the
other documents in this repository. The project's full list is in
[References](references.md).
