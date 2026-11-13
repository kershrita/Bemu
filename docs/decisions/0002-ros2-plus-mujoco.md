# 0002 — ROS 2 backbone with MuJoCo physics

**Status:** Accepted
**Date:** 2026-11-13
**Decided by:** Owner

## Context

The project needs a simulator now and a modular software structure for years of
added capabilities. It also wants learned walking later, which favours a simulator
with accurate contact physics that can run many copies in parallel.

## Decision

ROS 2 is the software backbone. MuJoCo is the physics engine. They are joined
through `ros2_control` using the `mujoco_ros2_control` bridge. The same joint
controllers drive MuJoCo in simulation and the servo driver on the real robot.

## Alternatives considered

| Alternative | Strength | Why not |
|---|---|---|
| ROS 2 + Gazebo | Most tutorials; easy simulated sensors | Slow to train in; rougher leg-contact physics; a weak base for learned walking later |
| MuJoCo alone, plain Python | Fastest start; best physics | No module system; every new sensor needs custom plumbing, which works against the scalability goal |
| NVIDIA Isaac Lab | Strong for large-scale learning | Needs a powerful NVIDIA graphics card; far more than programmed walking needs |

## Consequences

- Module structure and standard message types come from ROS 2.
- Accurate leg physics, and a model reusable for training, come from MuJoCo.
- More setup in the first week than either tool alone.
- The project depends on a bridge package first released in early 2026. If it
  cannot express something the model needs, the MuJoCo model is generated directly
  and the bridge only loads it (risk R4).
- Simulated sensors for later versions (camera, scanner) are less turnkey in MuJoCo
  than in Gazebo and may need some custom work.
