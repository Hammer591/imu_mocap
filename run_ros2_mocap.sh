#!/usr/bin/env bash
# Run the IMU mocap ROS2 node.
# Usage: ./run_ros2_mocap.sh [--ros-args ...]

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/ros_ws"

source /opt/ros/foxy/setup.bash
source "$WS_DIR/install/setup.bash"

# Let the node find jetson_mocap_v2.py in the project root
export IMU_MOCAP_DIR="$SCRIPT_DIR"

# Add the installed package to PYTHONPATH (setuptools 72.x workaround)
export PYTHONPATH="$WS_DIR/install/imu_mocap_node/lib/python3.8/site-packages:$PYTHONPATH"

exec python3.8 -c "
from imu_mocap_node.mocap_node import main
main()
" "$@"
