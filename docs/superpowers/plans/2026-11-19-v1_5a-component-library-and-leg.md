# Version 1.5a: Component Library and Printable Leg Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (native execution, as chosen for V1). Steps use checkbox syntax.

**Goal:** A verified component library (starting with the Feetech STS3215 12 V servo) and build123d generators for the printed leg (coxa bracket, femur links, tibia, foot) and a tolerance coupon, with fit checks enforced in tests.

**Architecture:** A new ROS-free package `bemu_cad` holds the component loader, servo solid model, fasteners, part generators, fit checks and exporters. Dimensions live in `config/components.yaml` (sources and status per value) and `config/robot_params.yaml` (leg lengths, limits). Parts are generated and checked in code; STEP/STL and mass properties go to `cad/export/` (generated, git-ignored except the coupon and print order).

**Tech Stack:** Python 3.13, build123d (OCP), PyYAML, pytest; the repo's ruff settings.

**Spec:** `docs/superpowers/specs/2026-11-19-v1_5-printable-design.md`.

## Global Constraints

- Every dimension of a bought part comes from `config/components.yaml`; a value has `source` (file in `docs/reference/` plus page or measurement method) and `status`: `datasheet` (printed dimension or table), `cad` (measured from the manufacturer's STEP), `standard` (an ISO/DIN table), `unverified`, or `measured` (caliper on the real part). Release check: no mounting feature may use `unverified`; the final print order additionally needs `measured` for the servo.
- Reference files (already in the repo): `docs/reference/sts3215/seeed-ST-3215-C047-datasheet.pdf`, `waveshare-ST3215-2D-drawing.pdf`/`.dxf`, `waveshare-ST3215.step`, `feetech-ST3215-20200328.stp`.
- Known servo facts (datasheet and drawing): body 45.22 × 24.72 mm; output axis 10.11 mm from one end and on the centre line; horn circle Ø19.2 (drawing) / Ø19.95 (horn drawing), 4 × M3 on Ø14, Ø9 boss, 25T spline Ø5.9, horn screw M3×6; heights 29 / 32 / 35 / 37.25 mm; case hole pattern 20.7 (along the body) × 20.5 (across); mass 55 ± 1 g. From the STEP: output axis along Y at (x = −25.5, z = 0), horn-face holes at (−18.5, 0), (−32.5, 0), (−25.5, ±7); the case's through-screw holes run along Y at x ∈ {−17.2, 3.5} (20.7 apart) and x ∈ {−32.6, 7.25} with z ≈ ±10.25 (20.5 apart), Ø1.6–3.5 along their length. Which of these are the external bracket holes is decided in Task 2 from the STEP and drawing, not assumed.
- Print assumptions from docs/08: bed 200 × 200 mm, 4 perimeters, 0.2 mm layers, hole allowance a parameter (+0.2 mm until the coupon sets it).
- Leg geometry must reproduce `robot_params.yaml`: joint-axis distances coxa 45 mm, femur 65 mm, tibia 110 mm (to the foot-sphere centre line per `bemu_kinematics`), joint ranges ±45° / femur and tibia limits as in the file.
- `build123d` goes into `requirements-dev.txt` (CI `python` job installs it); tests use `pytest.importorskip("build123d")` so the ROS image job skips them.
- Ruff clean; no subagents unless the owner asks.

## Review Focus

1. A `components.yaml` value that is unverified, missing a source, or negative/zero: the loader rejects it with a one-line message naming the component and field.
2. Changing a leg length in `robot_params.yaml` by ±10 mm regenerates every leg part with the same checks passing (the roadmap exit test).
3. A part that fits the bed only in one orientation: the bed check uses the stated print orientation and fails with the part name and the overshoot.
4. Clearance too small or negative (a pocket tighter than the servo): the interference check fails; a hole allowance of 0 fails the check for any press-fit-free pocket.
5. The STEP-derived hole table drifting from `components.yaml` (someone edits one): a test compares them.

### Task 1: `bemu_cad` package, component loader, library file

**Files:** Create `ros2_ws/src/bemu_cad/{package.xml,setup.py,setup.cfg,resource/bemu_cad,bemu_cad/__init__.py,bemu_cad/components.py}`, `config/components.yaml`; modify `pyproject.toml` (first-party, pythonpath), `requirements-dev.txt`; Test `ros2_ws/src/bemu_cad/test/test_components.py`.

**Interfaces:**
```python
@dataclass(frozen=True)
class Hole: x: float; y: float; diameter: float; thread: str; depth: float | None; status: str  # metres, in the component frame
@dataclass(frozen=True)
class ServoSpec:
    model: str; variant: str; size: tuple[float,float,float]  # body length, width, height, metres
    output_axis: tuple[float,float]          # position of the output axis in the body outline (from the length end, from the width centre)
    horn_circle: float; horn_holes: tuple[Hole, ...]; case_holes: tuple[Hole, ...]
    heights: dict[str, float]; mass: float; sources: dict[str, str]; status: dict[str, str]
def load_components(path=None) -> Components   # .servo: ServoSpec, .fasteners, .inserts, ...; raises ComponentsError
def unverified_fields(components) -> list[str]  # dotted names with status 'unverified'
```
`ComponentsError` is a `ValueError`. Units in the file are mm (like `robot_params.yaml`), converted to SI by the loader.

- [ ] **Step 1: Write failing tests:** loader returns the datasheet values (body 45.22 × 24.72 × 35 mm → metres; mass 0.055; horn screw M3×6; 4 horn holes on Ø14); every field carries a source naming a file that exists under `docs/reference/`; statuses are in the allowed set; a value ≤ 0, a missing `source`, an unknown status, or a missing field raises `ComponentsError` naming component and field (Review Focus 1); `unverified_fields` lists exactly the fields marked unverified (use a temp file).
- [ ] **Step 2:** `pytest ros2_ws/src/bemu_cad -q`. Expected: FAIL.
- [ ] **Step 3:** Implement; write `config/components.yaml` with the servo from the facts above (case holes marked `cad` until Task 2 settles them), plus M3 screw (ISO 4762 head Ø5.5 × 3.0, ISO 7380 optional), M3 nut, M3 heat-set insert (hole Ø4.0–4.2 mm range marked `unverified` until a supplier is chosen in 1.5c), 623 and 683 bearings (`standard`), Raspberry Pi 5 board (hole pattern 58 × 49 mm, M2.5, `datasheet` link added in 1.5b; `unverified` here), and a `battery` placeholder `unverified`.
- [ ] **Step 4:** tests pass; `ruff check . && ruff format --check .`.
- [ ] **Step 5: Commit** `Add the bemu_cad package and the component library`.

### Task 2: Derive the servo hole table from the STEP, and pin it

**Files:** Create `ros2_ws/src/bemu_cad/bemu_cad/step_probe.py`, `tools/probe_step.py`; Modify `config/components.yaml`; Test `test_step_probe.py`.

**Interfaces:** `probe_cylinders(step_path) -> list[CylinderFeature]` (diameter, axis direction, point, face extent) from OCP's `BRepAdaptor_Surface`; `servo_holes(step_path) -> ServoHoleTable` grouping features into (a) horn-face holes, (b) case through-holes, (c) other, in the component frame of `ServoSpec`.

- [ ] **Step 1: Write failing tests:** the probe on `waveshare-ST3215.step` finds the four horn-face holes at (−18.5, 0), (−32.5, 0), (−25.5, ±7) of the output axis (1e-2 mm) with Ø≈2.5; finds the case through-holes at z = ±10.25 with x = −17.2 and 3.5 (20.7 apart); the body bounding box is 45.22 × 24.72 mm in X and Z; the two manufacturer STEP files agree on every hole centre within 0.1 mm (they are different revisions) or the differences are listed in the test's expected-differences table; Review Focus 5: every hole in `components.yaml` for the servo equals the probe's result within 0.05 mm.
- [ ] **Step 2:** run. Expected: FAIL.
- [ ] **Step 3:** Implement the probe and a short script that prints the table (Markdown) for review; from the table and the 2D drawing decide **which holes are external bracket holes** (the 20.7 × 20.5 pattern is the first candidate) and record the decision and the evidence in `docs/reference/sts3215/README.md` and a ledger Ruling; set those holes `cad` in `components.yaml`, the rest `cad` + `internal: true`.
- [ ] **Step 4:** tests pass.
- [ ] **Step 5: Commit** `Derive and pin the servo hole table from the STEP files`.

### Task 3: Servo solid and pocket generators

**Files:** Create `bemu_cad/servo.py`; Test `test_servo.py`.

**Interfaces:** `servo_solid(spec) -> Solid` (body box, horn disc and boss, rear pivot boss, in the component frame, matching the STEP bounding boxes); `servo_pocket(spec, allowance) -> Solid` (body envelope grown by `allowance` on every face, plus the wire exit slot); `servo_mass_properties(spec) -> MassProperties` (mass 0.055 kg, centre and inertia of the solid scaled to that mass).

- [ ] **Step 1: Failing tests:** `servo_solid` bounding box is 45.22 × 24.72 × 37.25 mm within 0.05 mm of the STEP; the solid lies within the STEP's bounding box; `servo_pocket(spec, 0.2)` contains `servo_solid` with ≥ 0.2 mm clearance on every side (min distance via boolean/offset) and `servo_pocket(spec, 0)` is rejected (Review Focus 4); mass properties sum to 0.055 kg.
- [ ] **Step 2–4:** run, implement, run.
- [ ] **Step 5: Commit** `Add the servo solid and pocket`.

### Task 4: Printed leg parts

**Files:** Create `bemu_cad/leg.py`, `bemu_cad/printing.py`; Test `test_leg_parts.py`.

**Interfaces:** `build_leg(params, components, *, allowance) -> LegParts` with named `Part` objects (`coxa_bracket`, `femur_link_left`, `femur_link_right`, `tibia`, `foot`), each with `solid`, `print_orientation` (a rotation), `material`, `count_per_leg`; `printing.fits_bed(part, bed=(0.2, 0.2, 0.2)) -> list[str]` (empty when OK). Design rules from docs/08: servo is structural; each joint supported on both sides; load runs along layers; hard stops 3° beyond the software limits; cable path slot at every joint; heat-set insert holes sized from the library; joint-axis distances reproduce 45 / 65 / 110 mm.

- [ ] **Step 1: Failing tests:** axis-to-axis distances measured on the assembled leg equal `params.leg.*_length` within 0.05 mm; each servo sits in a pocket with the allowance clearance and no part penetrates a servo solid (the interference check from the spec); every screw hole has edge distance ≥ 1.5 × diameter and wall ≥ 1.6 mm; every part fits the bed in its print orientation (Review Focus 3: also a negative test with a 100 mm bed); changing the femur length by +10 mm regenerates with all checks passing and the axis distance 75 mm (Review Focus 2); the joint sweeps (coxa ±45°, femur and tibia limits) show no part-to-part collision.
- [ ] **Step 2–4:** run, implement, run.
- [ ] **Step 5: Commit** `Add the printed leg parts`.

### Task 5: Tolerance coupon, exports, print documents

**Files:** Create `bemu_cad/coupon.py`, `bemu_cad/export.py`, `tools/generate_cad.py`; Test `test_coupon.py`, `test_export.py`.

**Interfaces:** `build_coupon(components) -> Part` (servo pockets and hole rows at +0.1, +0.2, +0.3 mm, insert holes at 4.0 / 4.1 / 4.2 mm, each cell engraved with its allowance); `export_all(params, components, out_dir)` writes STEP and STL per part, `mass_properties.json` (mass, centre of mass, inertia per part), and `print_order.md` (file, material, quantity, orientation, settings from docs/08); `tools/generate_cad.py [--out DIR] [--coupon-only]`, one-line `generate_cad: ...` error and exit 1 on bad input.

- [ ] **Step 1: Failing tests:** the coupon fits the bed and every cell's allowance is the engraved one; `export_all` writes the files, the JSON round-trips, quantities match 6 legs × parts, the order document lists every file; the tool's error path (unwritable directory) prints one line and exits 1.
- [ ] **Step 2–4:** run, implement, run the full `pytest -m "not sim_launch"`.
- [ ] **Step 5: Commit** `Add the tolerance coupon and CAD exports`.

### Task 6: Documentation and CI

**Files:** Modify `docs/08-mechanical-design.md` (library, statuses, coupon workflow, the exact servo facts above), `docs/13-dev-environment.md` (the generate command and the `build123d` install note), `docs/14-testing-and-validation.md`, `docs/03-roadmap.md` (1.5.2 progress), `docs/10-components.md` (servo verified dimensions and the C018/C047 variant note), `.github/workflows/checks.yml` only if CI needs system libraries for OCP.

- [ ] **Step 1:** Update the docs; record in `docs/08` the two outstanding human steps: print the tolerance coupon and caliper the first servo.
- [ ] **Step 2:** `ruff check . && ruff format --check . && pytest -m "not sim_launch" -q`. Expected: PASS.
- [ ] **Step 3: Commit** `Document the component library and CAD workflow`; push; read the GitHub `python` job result and fix a red build (OCP wheels and CI memory are the likely problems).
