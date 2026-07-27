#!/usr/bin/env bash
# Build the IMU mocap ROS2 package (setuptools 72.x workaround).
# Usage: ./build_ros2_mocap.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/ros_ws"

source "/opt/ros/foxy/setup.bash"
cd "$WS_DIR"
colcon build --packages-select imu_mocap_node

# Fix executable bits (setuptools 72.x: entry_points broken, data_files loses +x)
find "$WS_DIR/install/imu_mocap_node/lib/imu_mocap_node" \
  -maxdepth 1 -type f -exec chmod +x {} + 2>/dev/null

echo "Done. Source: source $WS_DIR/install/setup.bash"
