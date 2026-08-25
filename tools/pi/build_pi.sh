#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_WS="${REPO_ROOT}/ros2_ws"

source /opt/ros/jazzy/setup.bash
cd "${REPO_ROOT}"
rosdep install \
  --from-paths ros2_ws/src/hil_control_stub ros2_ws/src/hil_pi_runtime \
  --ignore-src -r -y

cd "${ROS_WS}"
colcon build \
  --packages-up-to hil_pi_runtime \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=ON

echo "Built the Pi-targeted workspace through hil_pi_runtime."
echo "Source it with: source ${ROS_WS}/install/setup.bash"
