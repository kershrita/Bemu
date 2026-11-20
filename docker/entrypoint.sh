#!/usr/bin/env bash
# Load ROS and, when it has been built, the Bemu workspace, then run the command given.
set -e
source /opt/ros/lyrical/setup.bash
if [ -f /workspace/bemu/ros2_ws/install/setup.bash ]; then
  source /workspace/bemu/ros2_ws/install/setup.bash
fi
exec "$@"
