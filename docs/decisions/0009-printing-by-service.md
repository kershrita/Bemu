# 0009 — Parts are printed by a service

**Status:** Accepted
**Date:** 2026-11-14
**Decided by:** Owner

## Context

The owner does not have a 3D printer and can send part designs to someone else to
print. The printer's size, accuracy and materials are not yet known.

## Decision

All printed parts are ordered from a print service. The design process is arranged
around that: a tolerance test piece is the first order, parts are checked for fit
in CAD and in the simulation model before ordering, and each order goes out with a
written print specification.

## Alternatives considered

| Alternative | Strength | Why not |
|---|---|---|
| Buy a 3D printer | Fast iteration; cheaper per part over the whole project | A purchase the owner has not chosen to make; can be revisited if print orders become a bottleneck |
| Laser-cut or machined plates | Stiff, accurate flat parts | Suits plates only, not brackets; an option for the body plates later |

## Consequences

- No printer cost, and no time spent tuning a printer.
- Each design iteration takes a delivery wait, so parts must be right more often
  on the first order (risk R17).
- Hole and pocket allowances are set from the measured test piece, not assumed.
- Parts are designed for a 200 × 200 mm bed until the service's size is known
  (question Q5).
- Feet may use bought rubber tips if the service does not offer a flexible
  material.
- Printing cost is not in the component budget until a quote exists.
