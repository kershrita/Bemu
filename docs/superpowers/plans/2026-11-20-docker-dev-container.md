# Docker development container Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans.

**Goal:** Let the owner build, test and run the whole project (ROS 2 Lyrical, MuJoCo, CAD tools) from one Docker container inside WSL 2, with a written validation procedure, and keep the native WSL install as the alternative.

**Architecture:** One image from `ros:lyrical-ros-base` (the image CI already uses) plus the control stack, rviz2, MuJoCo and a Python 3.13 virtualenv for build123d (so the CAD tools do not depend on the system Python's wheel availability). The repository is bind-mounted; ROS `build/install/log` live in named volumes so a native build and a container build never share paths. GUI windows reach Windows through WSLg; the GPU is optional through an override file. A `docker/dev` wrapper hides the compose commands and `docker/validate.sh` runs the automated checks. CI builds the image and runs the quick validation, because the cloud session cannot run Docker.

**Spec:** the owner's request (2026-11-20): a markdown file with the steps to validate on the WSL PC, using Docker containers. Decision record 0012.

## Global Constraints

- No secrets or personal paths in images or files; the repo is mounted at `/workspace/bemu`, the user's UID/GID are passed in.
- `BEMU_PARAMS` is set inside the container (`/workspace/bemu/config/robot_params.yaml`).
- Everything that cannot be verified in the cloud session (building the image on the PC, WSLg windows, GPU) is labelled "not yet run on your PC" in the docs.

### Task 1: Image, compose, wrapper, validation script
**Files:** `docker/Dockerfile`, `docker/compose.yaml`, `docker/compose.gpu.yaml`, `docker/entrypoint.sh`, `docker/dev`, `docker/validate.sh`, `.dockerignore`.
- [ ] `bash -n` on every script; `docker compose config` parses both files.
### Task 2: CI proof
**Files:** `.github/workflows/checks.yml` (a `docker` job: build with the runner's UID, run `docker/validate.sh quick`).
- [ ] CI run green on the pushed commit (the only place the image is built).
### Task 3: Docs
**Files:** `docs/reports/v1_5-pc-validation.md` (steps, Docker first, native second, what to send back), `docs/13-dev-environment.md` (Docker section), `docs/decisions/0012-docker-dev-container.md`, README, docs index, release notes.
