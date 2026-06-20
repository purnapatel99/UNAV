#!/bin/bash
# Install UNAV dependencies: px4_msgs and PX4-Autopilot SITL
set -e

UNAV_WS=$(cd "$(dirname "$0")/.." && pwd)
PX4_MSGS_TAG="release/1.15"
PX4_TAG="v1.15.4"

echo "Setting up UNAV workspace at $UNAV_WS"

# Clone px4_msgs into src/
if [ ! -d "$UNAV_WS/src/px4_msgs" ]; then
  git clone --branch $PX4_MSGS_TAG https://github.com/PX4/px4_msgs.git "$UNAV_WS/src/px4_msgs"
else
  echo "px4_msgs already present, skipping."
fi

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
