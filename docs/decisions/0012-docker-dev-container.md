# 0012 — A Docker container for development and validation inside WSL 2

**Status:** Accepted (the owner asked for Docker; written 2026-11-20)
**Date:** 2026-11-20
**Decided by:** Owner

## Context

The development PC runs Ubuntu 26.04 under WSL 2 with a native ROS 2 Lyrical install (decision 0008).
Setting it up took many steps and went wrong in several ways (wrong paths, terminals without the
workspace sourced, a different Python for the CAD library). Version 1.5 adds the CAD tools, which need a
Python that has build123d wheels, and the owner prefers to use Docker containers for the project. The
cloud session that writes the code cannot run the PC's windows or GPU, so a reproducible environment that
CI can also build is worth having.

## Decision

The project ships a development image (`docker/Dockerfile`) built from `ros:lyrical-ros-base`, a compose
file, a `docker/dev` wrapper and `docker/validate.sh`. The container mounts the repository, keeps the ROS
build output in Docker volumes, shows windows through WSLg, and can use the Windows GPU through an
override file. CAD tools run in a Python 3.13 virtualenv in the same image. CI builds the image and runs
the quick validation inside it. The native WSL install (docs/13) stays supported as the fallback.

## Alternatives considered

| Alternative | Strength | Why not |
|---|---|---|
| Native install only (0008) | Fastest windows and GPU path, nothing extra | Setup is long and fragile; two Pythons; the cloud session cannot reproduce it |
| A VS Code dev container only | Editor integrated | Ties the workflow to one editor; the shell scripts work with any |
| Docker with host networking | ROS topics visible to native tools | Not supported the same way on Docker Desktop; one container with several shells avoids the need |

## Consequences

- One command builds an environment that matches CI; "works on my PC" and CI drift apart less.
- First build is large (about 4 GB) and needs internet; later builds use the cache.
- GUI and GPU inside the container depend on WSLg and the NVIDIA WSL driver; they are the part that is
  checked only on the owner's PC.
- The ROS build output is separate from any native build, so both can coexist.

## References

[13 — Development environment](../13-dev-environment.md); the PC validation guide
[reports/v1_5-pc-validation.md](../reports/v1_5-pc-validation.md). No external sources beyond the Docker,
WSLg and ROS documentation.
