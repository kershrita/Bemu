#!/usr/bin/env bash
# Automated checks for a Bemu checkout, run inside the development container (or any machine
# with ROS 2 Lyrical, MuJoCo and /opt/cad). Prints one PASS/FAIL line per step and a summary.
#
#   docker/validate.sh quick    build, ROS tests (no simulated launch), headless walk, CAD smoke
#   docker/validate.sh full     quick + the full CI "python" tests (about 10 minutes) +
#                               the simulated launch tests of bemu_bringup (headless simulator)
#
# The log of every step is in /tmp/bemu-validate/. Exit status 0 only if every step passed.
set -uo pipefail
MODE="${1:-quick}"
case "$MODE" in quick|full) ;; *) echo "usage: $0 [quick|full]" >&2; exit 2 ;; esac

REPO="$(cd "$(dirname "$0")/.." && pwd)"
LOG=/tmp/bemu-validate
CADPY="${CADPY:-/opt/cad/bin/python}"
export BEMU_PARAMS="${BEMU_PARAMS:-$REPO/config/robot_params.yaml}"
mkdir -p "$LOG"
cd "$REPO"

results=()
failed=0
step() {  # step "name" command...
  local name="$1"; shift
  local file="$LOG/$(echo "$name" | tr ' /' '__').log"
  printf '... %s\n' "$name"
  local start=$SECONDS
  if "$@" >"$file" 2>&1; then
    results+=("PASS  $name ($((SECONDS - start)) s)")
  else
    results+=("FAIL  $name ($((SECONDS - start)) s)  -> $file")
    failed=1
    tail -n 15 "$file"
  fi
}

source /opt/ros/lyrical/setup.bash

check_env() {
  [ -f "$BEMU_PARAMS" ] || { echo "BEMU_PARAMS not found: $BEMU_PARAMS"; return 1; }
  command -v ros2 >/dev/null || { echo "ros2 not on PATH"; return 1; }
  python3 -c "import mujoco, numpy, yaml; print('mujoco', mujoco.__version__)" || return 1
  "$CADPY" -c "import build123d, mujoco; print('build123d', build123d.__version__)" || return 1
  ros2 pkg list | grep -q '^mujoco_ros2_control$' || { echo "mujoco_ros2_control missing"; return 1; }
}

build_ws() { (cd ros2_ws && colcon build --symlink-install); }

test_ws() {
  (cd ros2_ws && source install/setup.bash \
    && colcon test --pytest-args -m "not sim_launch" \
    && colcon test-result --verbose)
}

sim_launch_tests() {
  (cd ros2_ws && source install/setup.bash \
    && colcon test --packages-select bemu_bringup --pytest-args -m sim_launch \
    && colcon test-result --verbose)
}

headless_walk() {
  python3 tools/walk.py --gait tripod --vx 0.08 --seconds 8
}

cad_smoke() {  # regenerate into a scratch folder and compare with the committed outputs
  local out="$LOG/cad"
  rm -rf "$out"
  "$CADPY" tools/generate_cad.py --out "$out" --no-meshes || return 1
  local f rc=0
  for f in print_order.md printability.md; do
    if ! diff -q "$out/$f" "cad/export/$f" >/dev/null; then
      echo "differs from the committed cad/export/$f:"
      diff "$out/$f" "cad/export/$f" | head -20
      rc=1
    fi
  done
  "$CADPY" - "$out/mass_properties.json" cad/export/mass_properties.json <<'PY' || rc=1
import json, math, sys
a, b = (json.load(open(p)) for p in sys.argv[1:3])
def same(x, y, path=""):
    if isinstance(x, dict):
        return all(k in y and same(x[k], y[k], f"{path}.{k}") for k in x) and x.keys() == y.keys()
    if isinstance(x, list):
        return len(x) == len(y) and all(same(i, j, path) for i, j in zip(x, y))
    if isinstance(x, float):
        ok = math.isclose(x, y, rel_tol=1e-4, abs_tol=1e-9)
        if not ok:
            print("mass_properties.json differs at", path, x, y)
        return ok
    return x == y
sys.exit(0 if same(a, b) else 1)
PY
  return $rc
}

python_tests() { "$CADPY" -m pytest -m "not sim_launch" -q -p no:cacheprovider; }

step "environment" check_env
step "colcon build" build_ws
if [ -f ros2_ws/install/setup.bash ]; then
  step "ROS package tests (no simulated launch)" test_ws
fi
step "headless walk (tools/walk.py, 8 s)" headless_walk
step "CAD regenerates to the committed outputs" cad_smoke
if [ "$MODE" = full ]; then
  step "CI python tests (build123d, MuJoCo)" python_tests
  step "simulated launch tests (bemu_bringup)" sim_launch_tests
fi

echo
echo "== Bemu validation ($MODE) =="
printf '%s\n' "${results[@]}"
if [ "$failed" -eq 0 ]; then echo "ALL PASSED"; else echo "SOME STEPS FAILED (logs in $LOG)"; fi
exit "$failed"
