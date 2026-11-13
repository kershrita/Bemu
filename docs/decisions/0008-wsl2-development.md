# 0008 — Develop under WSL 2 on the Windows PC

**Status:** Proposed
**Date:** 2026-11-14
**Decided by:** —

## Context

The development PC runs Windows 11 (Intel Core i9, 32 GB memory, NVIDIA RTX 5060 Ti
with 16 GB). The robot's Raspberry Pi 5 will run Ubuntu 26.04 with ROS 2 Lyrical.

ROS 2 Lyrical supports both Ubuntu 26.04 and Windows 11 as first-tier platforms.
On Ubuntu it installs from packages. On Windows it installs from a binary archive
that does not include every ROS 2 package.

## Decision

Development uses Ubuntu 26.04 under WSL 2 on the Windows PC. If graphics,
networking or training under WSL prove unreliable, the fallback is to dual-boot
Ubuntu 26.04 on the same PC.

## Alternatives considered

| Alternative | Strength | Why not first choice |
|---|---|---|
| ROS 2 natively on Windows | No Linux layer | Not every package is available; a different environment from the Pi; most robotics tooling and guides assume Ubuntu |
| Dual-boot Ubuntu from the start | Best graphics performance; simplest networking | Requires repartitioning and rebooting to switch; kept as the fallback |
| A virtual machine | Isolated | Weaker 3D graphics and no easy use of the graphics card for training |
| A container on Windows | Reproducible | Runs on WSL 2 underneath anyway; adds a layer for graphics |

## Consequences

- One environment, Ubuntu 26.04, on both the PC and the Pi.
- Windows stays available for everything else on the same machine.
- Three things must be verified rather than assumed: that 3D viewers use the
  NVIDIA card (step 0.2), that ROS 2 on the PC and the Pi can discover each other
  across WSL's network (version 2), and that USB devices can be passed into WSL
  (version 2). These are risks R15 and R16.
- The repository lives in the Linux file system for build speed.
