#!/bin/bash
# Install UNAV dependencies
set -e

UNAV_WS=$(cd "$(dirname "$0")/.." && pwd)
PX4_TAG="v1.15.4"

echo "Setting up UNAV workspace at $UNAV_WS"

# Init submodules (px4_msgs)
git -C "$UNAV_WS" submodule update --init --recursive

# Clone PX4-Autopilot for SITL (optional, large download)
if [ "${INSTALL_PX4_SITL:-0}" = "1" ]; then
  if [ ! -d "$UNAV_WS/../PX4-Autopilot" ]; then
    git clone --branch $PX4_TAG --recursive https://github.com/PX4/PX4-Autopilot.git "$UNAV_WS/../PX4-Autopilot"
  else
    echo "PX4-Autopilot already present, skipping."
  fi
fi

# Install ROS2 package dependencies
cd "$UNAV_WS"
rosdep install --from-paths src --ignore-src -r -y

echo "Setup complete. Build with: colcon build --symlink-install"
