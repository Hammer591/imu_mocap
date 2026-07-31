#!/usr/bin/env bash
# ORB-SLAM3-ROS2 一键启动（Intel RealSense D435i）
# 用法: ./run_orb_slam3.sh [use_pangolin:=true|false] [use_rviz:=true|false] [sensor_type:=imu-monocular|monocular]
#
# 传感器模式:
#   imu-monocular — 单目+IMU（需要移动相机初始化，精度更高）
#   monocular    — 纯单目（立刻初始化，适合桌面测试）
#
# 依赖: ROS2 Humble, realsense2_camera, D435i 已连接
#
# 输出话题:
#   /orb_odom           — Odometry（里程计）
#   /pose_array         — 累积位姿轨迹
#   /live_point_cloud   — 稀疏点云
#   /orb_camera/image   — 处理后图像
#   TF: odom → base_link
# 不使用 set -e（ros2 launch 的警告不影响运行）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ORB_WS_DIR="$SCRIPT_DIR/orb_slam3_ws"

# ============================================================
# 1. 退出 conda 环境（ROS2 需要系统 Python 3.10）
# ============================================================
unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_EXE CONDA_SHLVL CONDA_PACKAGE_DIRS
unset PYTHONPATH
export PATH="/usr/bin:/opt/ros/humble/bin:$PATH"

# ============================================================
# 2. Source ROS2 + 本地覆盖层（realsense2_description, xacro）
# ============================================================
source /opt/ros/humble/setup.bash

# 必须显式设置 LD_LIBRARY_PATH 包含 ROS2 C 扩展库
export LD_LIBRARY_PATH="/opt/ros/humble/lib:$LD_LIBRARY_PATH"

if [ -d "/tmp/ros2_deps/install/opt/ros/humble" ]; then
  export AMENT_PREFIX_PATH="/tmp/ros2_deps/install/opt/ros/humble:$AMENT_PREFIX_PATH"
  # xacro 和 realsense2_description 在覆盖层中
  export PATH="/tmp/ros2_deps/install/opt/ros/humble/bin:$PATH"
  # xacro 模块已 symlink 到 ~/.local/lib/python3.10/site-packages/
fi

# ============================================================
# 3. Source ORB-SLAM3 工作空间
# ============================================================
if [ -f "$ORB_WS_DIR/install/setup.bash" ]; then
  source "$ORB_WS_DIR/install/setup.bash"
else
  echo "错误: 找不到 $ORB_WS_DIR/install/setup.bash —— 先执行 colcon build"
  exit 1
fi

# ============================================================
# 4. 设置 Pangolin 和 ORB-SLAM3 库路径
# ============================================================
ORB_SLAM3_LIB_DIR="$ORB_WS_DIR/src/ORB_SLAM3_ROS2/ORB_SLAM3/lib"
ORB_SLAM3_3RD_DIR="$ORB_WS_DIR/src/ORB_SLAM3_ROS2/ORB_SLAM3/Thirdparty"
export LD_LIBRARY_PATH="/home/lab/.local/lib:$ORB_SLAM3_LIB_DIR:$ORB_SLAM3_3RD_DIR/DBoW2/lib:$ORB_SLAM3_3RD_DIR/g2o/lib:$LD_LIBRARY_PATH"

# ============================================================
# 5. 启动 ORB-SLAM3（imu-monocular + D435i）
# ============================================================
echo "=========================================="
echo "  ORB-SLAM3-ROS2 启动中..."
echo "  D435i: 640x480@30fps color + IMU 200Hz"
echo "  输出: /orb_odom /pose_array /live_point_cloud"
echo "=========================================="

# 默认参数
USE_PANGOLIN="${1:-true}"     # 设为 false 可禁用 Pangolin 窗口（无显示器时）
USE_RVIZ="${2:-true}"          # 设为 false 可禁用 RViz

# 默认传感器类型（可改为 "monocular" 跳过 IMU 初始化）
SENSOR_TYPE="${3:-imu-monocular}"

ros2 launch orb_slam3_ros2 mapping.launch.py \
  use_pangolin:="$USE_PANGOLIN" \
  use_rviz:="$USE_RVIZ" \
  sensor_type:="$SENSOR_TYPE"
