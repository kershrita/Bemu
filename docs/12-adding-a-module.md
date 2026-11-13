# 12 — Adding a module

The checklist for adding any new sensor, actuator or capability. If a new module
cannot be added by following this page, the architecture needs fixing, not the
module.

## The principle

A module is added by **adding** things: a package, a mounting part, a launch entry,
a document section. It should not require **editing** existing packages. The one
permitted edit is a single line in the top-level launch file to include it.

## Physical side

| Step | What to do | Reference |
|---|---|---|
| 1. Mount | Design an adapter plate that bolts to the top-deck grid (M3, 20 mm pitch) or the front sensor mount. The body is not modified | [08 — Mechanical design](08-mechanical-design.md) |
| 2. Mass | Add the module's mass and position to `robot_params.yaml`. Check the total stays within the payload budget | [08 — Mechanical design](08-mechanical-design.md) |
| 3. Power | Choose a rail: battery voltage, 5 V, or 3.3 V. Check the rail's remaining current | [09 — Electronics and power](09-electronics-and-power.md) |
| 4. Data | Choose a port: USB, I2C, serial, camera connector, or network | [09 — Electronics and power](09-electronics-and-power.md) |
| 5. Cable | Route through the body's cable channels; add strain relief | |

## Software side

| Step | What to do |
|---|---|
| 1. Package | Create `hexapod_<name>` in `ros2_ws/src/` |
| 2. Contract | Decide what it publishes and subscribes. Use standard message types where they exist. Add the topics to [05 — Software stack](05-software-stack.md) |
| 3. Frame | Add the module's frame (for example `camera_link`) to the model via the parameter file |
| 4. Simulated twin | Add the equivalent sensor to the MuJoCo model, publishing the same message on the same topic |
| 5. Launch | Give the package its own launch file. Add one include, behind an argument, to the top-level launch file |
| 6. Configuration | Put settings in a YAML file inside the package |
| 7. Diagnostics | Publish health on `/diagnostics` |
| 8. Tests | Unit tests for logic; one simulated test that exercises the contract |

## Behaviour side

If the module enables a new behaviour, the behaviour is a separate package from the
driver. The driver publishes data; the behaviour consumes it.

| Example | Driver package | Behaviour package |
|---|---|---|
| Tilt sensor | `hexapod_imu` | Levelling lives in the walker as an optional input |
| Distance scanner | `hexapod_lidar` | `hexapod_navigation` |
| Camera | `hexapod_camera` | `hexapod_perception` |
| Microphone and speaker | `hexapod_audio` | `hexapod_voice` |

This split means a sensor can be replaced by a different model without touching the
behaviour.

## Documentation side

| Step | What to do |
|---|---|
| 1 | Add the module to the roadmap as a version, with an exit test written first |
| 2 | Add its parts to [10 — Components](10-components.md) |
| 3 | Add a decision record if a choice between alternatives was made |
| 4 | Update the power budget and mass budget |

## Worked example: adding a tilt sensor (version 4)

| Side | Action |
|---|---|
| Mount | A small plate at the centre of the top deck, near the centre of mass |
| Mass | About 5 g; negligible, but recorded |
| Power | 3.3 V from the Pi's header |
| Data | I2C on the Pi's header |
| Package | `hexapod_imu`, publishing `sensor_msgs/Imu` on `/imu/data` |
| Frame | `imu_link` added to the parameter file |
| Simulated twin | MuJoCo orientation, gyro and accelerometer sensors at `imu_link`, published on `/imu/data` |
| Launch | `imu:=true` argument |
| Behaviour | The walker subscribes to `/imu/data` if present and applies levelling |
| Tests | Simulated tilting-floor test |
| Walker edits | One optional subscription and the levelling function. No structural change |

## Reserved capacity

Designed in from the first hardware version so modules have somewhere to go.

| Resource | Reserved |
|---|---|
| Top-deck mounting area | A grid of at least 5 × 7 insert positions |
| Front sensor mount | One, with adjustable tilt |
| Payload mass | About 300 g |
| 5 V power | A second 5 V rail separate from the Pi's supply |
| Battery-voltage power | Two spare fused outputs |
| USB ports on the Pi | At least two free after the servo adapter |
| I2C, serial, SPI | Header left accessible |
| Camera connectors | Both of the Pi 5's left accessible |
| Processor headroom | The walking stack should use well under half of the Pi's capacity |
