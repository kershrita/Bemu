# 0006 — ROS 2 Lyrical Luth, with Jazzy as fallback

**Status:** Accepted
**Date:** 2026-11-13
**Decided by:** Owner, by approving the design on 2026-11-14

## Context

Two long-term-support ROS 2 releases are current: Jazzy (2024, Ubuntu 24.04,
supported to May 2029) and Lyrical Luth (May 2026, Ubuntu 26.04, supported to
May 2031). The project is starting fresh, has no existing code tied to either, and
is expected to run for years.

## Decision

Use ROS 2 Lyrical Luth on Ubuntu 26.04, on both the PC and the Pi 5. If a package
needed in version 0 is missing or broken on Lyrical, switch to Jazzy on
Ubuntu 24.04.

## Alternatives considered

| Alternative | Strength | Why not first choice |
|---|---|---|
| Jazzy | Two more years of community fixes; the most tutorials | Support ends two years earlier; starting on it means a migration mid-project |
| Rolling (the development branch) | Newest features | Changes continuously; unsuitable for a long project |

## Consequences

- The longest support window available.
- Newer releases have fewer worked examples and occasionally missing packages.
  Sensor drivers needed in later versions are checked for Lyrical support when each
  version is planned.
- The MuJoCo bridge lists Lyrical as supported, which was the main dependency to
  confirm.
- The code avoids anything specific to one release, so the fallback stays cheap.
- Step 0.2 records the exact versions installed.

## References

| Source | Used here for | Checked |
|---|---|---|
| [ROS 2 documentation](https://docs.ros.org/) | Lyrical Luth as the recommended long-term release | 2026-11-13 |
| [ROS 2 Lyrical vs Jazzy](https://openelab.io/cs/blogs/learn/ros-2-lyrical-vs-jazzy) | Support end dates; Ubuntu pairing | 2026-11-13 |
| [Lyrical Luth release page](https://docs.ros.org/en/lyrical/Releases/Release-Lyrical-Luth.html) | Release details and platforms | 2026-11-13 |
| [mujoco_ros2_control on ROS Index](https://index.ros.org/r/mujoco_ros2_control/) | The MuJoCo bridge lists Lyrical as supported | 2026-11-13 |

The full list for the project is in [References](../references.md). A dash means the source
is listed for reference and was not re-read on a specific date.
