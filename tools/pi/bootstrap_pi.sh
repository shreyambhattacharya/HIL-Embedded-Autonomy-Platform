#!/usr/bin/env bash
set -euo pipefail

# Install only the headless ROS/runtime prerequisites for hil_pi_runtime.
# This script does not configure networking, credentials, or a firewall.
if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "warning: expected aarch64; found $(uname -m)" >&2
fi
if [[ ! -f /etc/os-release ]] || ! grep -q '^VERSION_ID="24.04"' /etc/os-release; then
  echo "warning: this script targets Ubuntu 24.04 LTS" >&2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed. Configure the official ROS apt repository first." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-ros-base

echo "Pi runtime prerequisites installed. No Gazebo, RViz, desktop, or GPU packages were requested."
echo "Run tools/pi/build_pi.sh from the cloned repository next."
