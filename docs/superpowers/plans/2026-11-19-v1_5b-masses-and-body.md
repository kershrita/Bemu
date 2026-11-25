# Version 1.5b: CAD Masses into the Model, and the Body Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (native execution). Steps use checkbox syntax.

**Goal:** Put the CAD-derived link masses into the simulation and show the version 1 tests and torque report still pass (roadmap 1.5.4 and 1.5.5), then design the printed body (plates, coxa mounts, mounting grid) with the same fit checks as the leg (1.5.3).

**Architecture:** `bemu_cad.masses` turns the leg assembly into per-link masses (coxa, femur, tibia link) that `tools/generate_cad.py --sync-masses` writes into `config/robot_params.yaml`; a guard test keeps them equal. The body generator `bemu_cad.body` builds the top and bottom plates (split to fit the bed) and the coxa mounts from the same library, and is checked against the real servo STEP and the leg's coxa sweep.

**Tech Stack:** as 1.5a (build123d, pytest, PyYAML); the headless simulation for the V1 re-run.

**Spec:** `docs/superpowers/specs/2026-11-19-v1_5-printable-design.md` sections 5.2 and 6; plan 1.5a.

## Global Constraints

- Link masses are the sum of printed parts (PETG, fill factor 0.6, an estimate until weighed), servos (55 g each), and screws (M2: 0.2 g + 0.025 g/mm; M3: 0.5 g + 0.055 g/mm). Groups: **coxa link** = coxa bracket + femur servo + 8 clevis M2 + 4 coxa-horn M3; **femur link** = femur plates + knee spacers + knee servo + 8 femur-horn M3 + 8 knee-servo M2; **tibia link** = tibia plates + foot bridge + foot + 8 knee-horn M3 + 4 bridge M2. The coxa servo belongs to the body.
- V1 assumed coxa 75 g, femur 25 g, tibia 85 g (185 g per leg, the knee servo counted in the tibia). The CAD puts the knee servo in the femur link, so the split changes while the total stays close.
- The V1 exit tests and the torque report (48 rows) must pass with the new masses; failing rows are findings to report, never numbers to adjust.
- Body: top and bottom plates outline = the elongated hexagon through the hip mounts, 240 × 160 mm (`robot_params.yaml` body.size), split so no piece exceeds 200 × 200 mm; plate thickness 3 mm; the coxa servo (vertical, horn down) is held at both ends; electronics trays follow only when their component dimensions are verified (1.5c), so this plan builds the plates, coxa mounts and the 20 mm M3 insert grid only.
- Ruff clean; tests needing build123d use `importorskip`.

## Review Focus

1. A change of a leg length must change the link masses on the next sync, and the guard test must fail when the yaml and the CAD disagree.
2. The new masses must not break standing height or sit pose assumptions (torque and stability tests re-run).
3. Every body plate piece fits the bed; the coxa bracket's ±45° sweep never touches the plates or posts.
4. The coxa servo is supported at both faces (a tilt moment of roughly 1 N·m acts on it from a loaded leg).

### Task 1: Link masses from the CAD, synced into the model

**Files:** Create `ros2_ws/src/bemu_cad/bemu_cad/masses.py`, `ros2_ws/src/bemu_cad/test/test_masses.py`; modify `tools/generate_cad.py` (`--sync-masses`), `config/robot_params.yaml` (comment and values), docs.

**Interfaces:**
```python
@dataclass(frozen=True)
class LinkMasses: coxa: float; femur: float; tibia: float  # kg
def link_masses(leg: LegParts, components: Components) -> LinkMasses
def sync_robot_params(path: Path, masses: LinkMasses) -> None   # rewrites the three link_mass lines (grams), keeping comments
```

- [ ] **Step 1: Write failing tests:** the three masses are in the expected ranges (coxa 60–80 g, femur 70–95 g, tibia 25–45 g) and their sum plus the coxa servo equals the leg total of `mass_properties.json` within 1 g; lengthening the femur by 10 mm raises the femur and tibia link masses (plates are longer) and leaves the coxa link within 1 g; `sync_robot_params` on a temp copy changes only the three numbers and keeps every comment line (Review Focus 1); a guard test fails when `config/robot_params.yaml` differs from `link_masses` by more than 0.5 g.
- [ ] **Step 2:** run; FAIL. **Step 3:** implement. **Step 4:** run; PASS.
- [ ] **Step 5:** `python3 tools/generate_cad.py --sync-masses` writes the new values; commit `Add CAD-derived link masses and sync them into the parameters`.

### Task 2: Re-run version 1 on the CAD masses

- [ ] **Step 1:** run the whole suite (`pytest -m "not sim_launch"`) and `python3 tools/torque_report.py`. Expected: pass; read every changed row. Failing tests or rows are findings: report them and stop for a ruling rather than loosening a bound.
- [ ] **Step 2:** update `docs/reports/v1-torque-report.*` (rename to `v1_5-torque-report.*` with the CAD-mass result and keep the V1 file), `docs/10-components.md` (servo table vs the new worst case), `docs/06`, `docs/08` (fix the "+0.34 kg" statement: the leg groups weigh about 186 g against 185 g; the split changed), the roadmap exit test box "The simulation with CAD-derived masses still passes the version 1 exit test".
- [ ] **Step 3:** commit `Re-run version 1 on the CAD masses`; push; check CI.

### Task 3: The coxa mount (the leg becomes body-ready)

**Files:** modify `bemu_cad/leg.py` (coxa servo raised by the support plate thickness and a boss on the coxa bracket through a support-plate window), `bemu_cad/body.py` (new: the coxa support plate and the top-plate seat), tests.

**Interfaces:** `build_leg` gains `coxa_support: float = 3.0` (mm, the lower support plate between the servo horn face and the rotating base plate; the base plate gets a Ø18 boss through its window, like the femur plates). Layout facts: hip plane z = 0; the coxa servo's horn face is the lower shelf; its back shelf, idler and hub face up; the top plate seats on the back shelf with a hub and idler relief.

- [ ] **Step 1: Write failing tests:** the coxa servo's two shelves are 32.0 mm apart and each is within 0.2 mm of a plate seat face; the base plate does not touch the support plate (clearance ≥ 0.5 mm) and its boss reaches the coxa horn within 0.5 mm in both STEP models; the interference check against the real STEP models still passes for every part and screw head; the axis distances still equal the parameter file.
- [ ] **Step 2–4:** run, implement, run. **Step 5:** commit.

### Task 4: Body plates

**Files:** create `bemu_cad/body.py` (continued), `test_body.py`.

**Interfaces:** `build_body(params, components, leg) -> BodyParts` with parts `bottom_plate_front|centre|rear`, `top_plate_front|centre|rear` (split at x = ±40), `coxa_support_plate` ×6 (small, one per hip), `post` ×N, each with outline, holes, print axis, bed fit; plus the 20 mm M3 insert grid on the top plate.

- [ ] **Step 1: Write failing tests:** the assembled outline equals the hexagon through the mounts and spans 240 × 160 mm; every piece fits a 200 × 200 mm bed (and a negative test with 100 mm); every coxa servo pocket seat matches the servo's real shelf holes (alignment against the STEP, as in 1.5a); the coxa bracket sweep over ±45° and the leg's joint range never touch any body part or screw head (Review Focus 3); piece joints are bridged by posts or splice plates; plate holes meet the edge-distance rules; the top plate has at least 5 × 7 grid positions.
- [ ] **Step 2–4:** run, implement, run. **Step 5:** export the body in `generate_cad.py`, extend `print_order.md` and `mass_properties.json` (body mass from CAD, servos, and placeholder electronics), commit.

### Task 5: Chassis mass, final joint limits, documentation

- [ ] **Step 1:** body mass = plates + posts + 6 coxa servos + electronics placeholders (Pi 75 g, battery 350 g, regulator and wiring 120 g, from docs/08), compared with the 2.5 kg design mass; write `body.mass` through the same sync mechanism and re-run Task 2's checks. Report the margin.
- [ ] **Step 2:** record the final joint limits from the sweep (the limits in `robot_params.yaml` stay unless a gait-reachable pose collides).
- [ ] **Step 3:** update docs 08, 03, 10, 13, 14; commit; push; read CI; one reviewer pass (Opus) over the 1.5b range, then a fix pass.
