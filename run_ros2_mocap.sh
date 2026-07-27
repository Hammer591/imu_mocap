#!/usr/bin/env bash
# Run the IMU mocap ROS2 node.
# Usage: ./run_ros2_mocap.sh [--ros-args ...]
#
# ROS2_DISTRO overridable env var (default: foxy for dev, override for deploy):
#   ROS2_DISTRO=humble ./run_ros2_mocap.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/ros_ws"

ROS2_DISTRO="${ROS2_DISTRO:-foxy}"

source "/opt/ros/${ROS2_DISTRO}/setup.bash"
source "$WS_DIR/install/setup.bash"

# Let the node find jetson_mocap_v2.py in the project root
export IMU_MOCAP_DIR="$SCRIPT_DIR"

# Add the installed package to PYTHONPATH (setuptools 72.x workaround)
_PY_VER="$(python3 --version | grep -oP '\d+\.\d+' | head -1)"
export PYTHONPATH="$WS_DIR/install/imu_mocap_node/lib/python${_PY_VER}/site-packages:$PYTHONPATH"

exec python3 -c "
from imu_mocap_node.mocap_node import main
main()
" "$@"
