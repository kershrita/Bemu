# Version 1.5d: Battery and Pi trays, neighbour-leg check, spares Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans. Steps use checkbox syntax.

**Goal:** Finish the version 1.5 scope that 1.5a-c deferred: the battery bay and the Raspberry Pi mount (roadmap step 1.5.3), the neighbouring-leg collision check, the spares the spec asks for, and an honest ruling on hard stops and cable slots.

**Architecture:** Same build123d generators and component library. The body's main posts move outward to open a clear battery channel along x. A battery tray (PETG) bolts to heat-set inserts in the bottom plate; a Pi mount plate with four standoffs bolts to four grid inserts on the top deck. Both are new `Part`s in `body.py`, so exports, masses, fit checks and the print order pick them up.

**Tech Stack:** build123d, pytest, ruff (as 1.5a-c).

**Spec:** `docs/superpowers/specs/2026-11-19-v1_5-printable-design.md` (sections 2, 5.2, 5.4, 6; roadmap step 1.5.3).

## Global Constraints

- No dimension from memory: each new library value carries a source and a status; unverified values keep the print order at NOT READY TO ORDER.
- 200 × 200 mm bed; M3 insert holes 4.1 mm, insert length 4 mm (plates are 4 mm); design mass 2.5 kg with `margin_kg > 0`.
- All body checks of 1.5b (coverage, seating, sweeps, edge distances, thickness) keep passing.
- No PR until the owner has validated on the PC.

## Review Focus

- The battery (138 × 42 × 28 mm, 359 g listing) must not intersect any post, hip plate, hip post or coxa servo, and must clear them by at least 2 mm.
- The Pi (85 × 56) on its standoffs must not intersect a coxa servo, the top plate or a leg in any swing/sweep pose, and must clear the legs' joints.
- Moving the posts must not break plate stiffness assumptions: every plate piece keeps at least two posts under it.
- The mass margin after the trays must stay positive.

### Task 1: Component entries

**Files:** Modify `config/components.yaml`, `ros2_ws/src/bemu_cad/test/test_components.py`.
**Produces:** `components.fields["battery"]["size"|"mass"]` (Ovonic 3S 5000 mAh listing: 138 × 42 × 28 mm, 359 g, status unverified), Pi 5 entries with a sharper source note.
- [ ] Test: the battery size/mass are the listing values, status unverified, and appear in `unverified_fields`.
- [ ] Edit the YAML.
- [ ] Run test_components.

### Task 2: Trays and posts

**Files:** Modify `ros2_ws/src/bemu_cad/bemu_cad/body.py`, `export_body.py`, `masses.py` if it lists body parts; Test `ros2_ws/src/bemu_cad/test/test_body.py`.
**Interfaces:** `build_body` adds parts `battery_tray` (family `battery_tray`, one part) and `pi_plate` (family `pi_plate`), insert holes (Ø4.1) in the bottom plate at the tray ears and uses four grid points for the Pi plate; new `screws` rows; `BodyParts.electronics_mass_kg` drops the battery's and Pi's placeholder masses only where the library now gives them (battery 359 g from the library).
- [ ] Tests (write first): battery box clears every post, hip post, hip plate, coxa servo by ≥ 2 mm; tray fits bed; Pi box above the top plate clears all legs in the swing test; tray ears' holes sit over bottom-plate insert holes; mass margin > 0; every plate piece has ≥ 2 posts.
- [ ] Move `MAIN_POSTS` to (±88, ±15) and (±21, ±29); regenerate; keep 1.5b tests green.
- [ ] Implement tray and plate generators.
- [ ] Export (print order rows, hardware rows) and printability for the new families.

### Task 3: Neighbour-leg check

**Files:** Modify `fit.py` (add `neighbour_collisions(body_hip_transforms, leg, limits)`), Test `test_body.py`.
- [ ] Test: adjacent legs (lf-lm, lm-lr, rf-rm, rm-rr, lf-rf, lr-rr) at the coxa limits pointing at each other, over three femur/knee poses (stand, sit, extended), have no part-to-part intersection beyond contact volume.

### Task 4: Spares, bed margin, hard stops ruling

**Files:** Modify `export.py` (`SPARES`), `export_printability.py` (bed margin), docs.
- [ ] One spare of every leg part (feet: two), per spec 5.4; update the quantity test.
- [ ] Fit is judged with a 5 mm margin per side.
- [ ] Ruling recorded: hard stops and cable slots are not modelled; they depend on the real servo's wire exit and first-leg behaviour (version 2).

### Task 5: Regenerate, docs, suite, push

- [ ] `python3 tools/generate_cad.py` (CAD, meshes), `--sync-masses`; update docs 08/03/10/14/15, release notes, purchase list (quantities, PETG grams), README; run full suite; commit; push; CI; Opus review; one fix pass.
