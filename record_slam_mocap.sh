#!/usr/bin/env bash
# Full pipeline: ORB-SLAM3 + IMU mocap + rosbag recording
#
# 一键启动 ORB-SLAM3 + 6段IMU动捕 + 足底力 + 相机IMU，并录制 rosbag
#
# 说明:
# - ORB-SLAM3 的 mapping.launch.py 内置了 D435i 相机启动，
#   不需要单独启动 realsense2_camera，避免冲突。
# - 不启动 imu_relay（/imu0）：实测 Python 中继订阅 /camera/camera/imu
#   会把 realsense 的 IMU 合成流拖慢，导致 ORB 卡顿。
#   原始 /camera/camera/imu 已录制，/imu0 是冗余数据。
# - 相机 IMU 原始数据直接从 /camera/camera/imu 录制。
#
# 录制话题:
#   ORB-SLAM3 轨迹:
#     /orb_odom              — 里程计（位姿+速度）
#     /pose_array            — 累积轨迹
#     /live_point_cloud      — 稀疏点云
#     /orb_camera/image      — 处理后图像
#     /orb_camera/info       — 相机内参
#   D435i 彩色流（由 ORB-SLAM3 内部启动）:
#     /camera/camera/color/image_raw  — 原始彩色图
#     /camera/camera/color/camera_info
#     /camera/camera/imu     — D435i 原始 IMU
#   IMU 动捕:
#     /left_thigh/imu        — 6段身体 IMU
#     /left_shank/imu
#     /right_thigh/imu
#     /right_shank/imu
#     /left_foot/imu
#     /right_foot/imu
#     /left_foot/wrench      — 足底反力
#     /right_foot/wrench
#   坐标变换:
#     /tf
#     /tf_static
#
# 前置条件:
#   - STM32 已上电，/dev/ttyACM0 可访问
#   - Intel RealSense D435i 已连接 USB
#   - ORB-SLAM3 已编译（orb_slam3_ws）
#   - IMU mocap 已编译（ros_ws）
#   - python3-serial 已安装
#
# 用法:
#   ./record_slam_mocap.sh                                    # 全部启动 + 录制到 bags/
#   ./record_slam_mocap.sh --duration 30                      # 录制 30 秒后自动停止
#   ./record_slam_mocap.sh --output experiment_1              # 指定 bag 名称，保存到 bags/
#   ./record_slam_mocap.sh --output /path/to/bag              # 绝对路径保存到别处
#   ./record_slam_mocap.sh --no-rviz                          # 不启动 RViz
#   ./record_slam_mocap.sh --no-pangolin                      # 不启动 Pangolin
#   ./record_slam_mocap.sh --monocular                        # 纯单目模式
#   ./record_slam_mocap.sh --duration 120 --no-rviz           # 组合使用
#
# 输出:
#   bags/<rosbag_YYYY-MM-DD_HH-MM-SS>/  （默认）
#
# 录制计时说明:
#   录制从 ORB-SLAM3 就绪后开始（检测到 /orb_odom 话题发布），
#  --duration 设定的时长从这一刻开始计算。
#
# 按 Ctrl+C 停止录制并关闭所有进程。

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ORB_WS_DIR="$SCRIPT_DIR/orb_slam3_ws"
MOCAP_WS_DIR="$SCRIPT_DIR/ros_ws"
ROS2_DISTRO="${ROS2_DISTRO:-humble}"

# ============================================================
# 0. 命令行参数
# ============================================================
USE_RVIZ=true
USE_PANGOLIN=true
SENSOR_TYPE="imu-monocular"
BAG_NAME=""
DURATION=0

while [ $# -gt 0 ]; do
    case "$1" in
        --no-rviz)       USE_RVIZ=false; shift ;;
        --no-pangolin)   USE_PANGOLIN=false; shift ;;
        --monocular)     SENSOR_TYPE="monocular"; shift ;;
        --output)        BAG_NAME="$2"; shift 2 ;;
        --duration)      DURATION="$2"; shift 2 ;;
        -h|--help)
            sed -n '/^#.*$/p' "$0" | grep -v '!/bin'
            exit 0
            ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# ============================================================
# 1. 环境准备
# ============================================================

for ws_dir in "$ORB_WS_DIR/install/setup.bash" "$MOCAP_WS_DIR/install/setup.bash"; do
    if [ ! -f "$ws_dir" ]; then
        echo "错误: 找不到 $ws_dir"
        echo "请先执行编译脚本。"
        exit 1
    fi
done

python3 -c "import serial" 2>/dev/null || {
    echo "错误: python3-serial 未安装"
    echo "请执行: sudo apt install python3-serial"
    exit 1
}

# 退出 conda（ORB-SLAM3 需要系统 Python 3.10）
unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_EXE CONDA_SHLVL CONDA_PACKAGE_DIRS
unset PYTHONPATH
export PATH="/usr/bin:/opt/ros/humble/bin:$PATH"

source "/opt/ros/${ROS2_DISTRO}/setup.bash"

# ORB-SLAM3 库路径
ORB_SLAM3_LIB_DIR="$ORB_WS_DIR/src/ORB_SLAM3_ROS2/ORB_SLAM3/lib"
ORB_SLAM3_3RD_DIR="$ORB_WS_DIR/src/ORB_SLAM3_ROS2/ORB_SLAM3/Thirdparty"
export LD_LIBRARY_PATH="/opt/ros/humble/lib:/home/lab/.local/lib:$ORB_SLAM3_LIB_DIR:$ORB_SLAM3_3RD_DIR/DBoW2/lib:$ORB_SLAM3_3RD_DIR/g2o/lib:$LD_LIBRARY_PATH"

source "$ORB_WS_DIR/install/setup.bash"
source "$MOCAP_WS_DIR/install/setup.bash"

# ROS2 覆盖层（realsense2_description, xacro）
if [ -d "/tmp/ros2_deps/install/opt/ros/humble" ]; then
    export AMENT_PREFIX_PATH="/tmp/ros2_deps/install/opt/ros/humble:$AMENT_PREFIX_PATH"
    export PATH="/tmp/ros2_deps/install/opt/ros/humble/bin:$PATH"
fi

# IMU mocap 环境
export IMU_MOCAP_DIR="$SCRIPT_DIR"
_PY_VER="$(python3 --version | grep -oP '\d+\.\d+' | head -1)"
export PYTHONPATH="$MOCAP_WS_DIR/install/imu_mocap_node/lib/python${_PY_VER}/site-packages:$PYTHONPATH"

# ============================================================
# 2. 工具函数
# ============================================================

wait_for_topic() {
    local topic="$1"
    local label="$2"
    local timeout="$3"
    local waited=0
    while [ "$waited" -lt "$timeout" ]; do
        if ros2 topic list 2>/dev/null | grep -qFx "$topic"; then
            echo "  ✓ $label 已就绪（${waited}s）"
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    echo "  ⚠ $label 未在 ${timeout}s 内就绪，继续..."
    return 1
}

# ============================================================
# 3. 清理函数
# ============================================================

_CLEANUP_DONE=0
PID_ORB_SLAM3=""
PID_MOCAP=""
PID_BAG=""

# 杀掉进程及其所有后代进程（避免 ros2 launch 子进程残留）
kill_tree() {
    local root="$1"
    [ -z "$root" ] && return
    # 先杀子进程（递归）
    local children
    children=$(pgrep -P "$root" 2>/dev/null) || true
    for child in $children; do
        kill_tree "$child"
    done
    kill -KILL "$root" 2>/dev/null || true
}

cleanup() {
    [ "$_CLEANUP_DONE" -ne 0 ] && return
    _CLEANUP_DONE=1

    echo ""
    echo "=========================================="
    echo "  正在停止所有进程..."
    echo "=========================================="

    # 1) 先停 rosbag（SIGINT 确保 bag 完整写入）
    [ -n "$PID_BAG" ] && { echo "  停止 rosbag 录制..."; kill -SIGINT "$PID_BAG" 2>/dev/null; sleep 2; }

    # 2) 再停其他节点
    # ORB-SLAM3 是 ros2 launch，会 spawn 多个子进程，需要整棵进程树一起杀
    if [ -n "$PID_ORB_SLAM3" ]; then
        echo "  停止 ORB-SLAM3 (进程树)..."
        kill -SIGTERM "$PID_ORB_SLAM3" 2>/dev/null
        sleep 2
        kill_tree "$PID_ORB_SLAM3"
    fi
    [ -n "$PID_MOCAP" ]     && { echo "  停止 mocap 节点..."; kill -SIGTERM "$PID_MOCAP" 2>/dev/null; sleep 0.5; kill -KILL "$PID_MOCAP" 2>/dev/null || true; }

    # 3) 兜底：清理 ORB-SLAM3 launch 可能遗留的任何子进程
    #    （防止 kill_tree 漏掉的孤儿）
    pkill -9 -f "orb_slam3_ros2/mapping.launch.py" 2>/dev/null || true
    pkill -9 -f "orb_slam3_ros2/lib/orb_slam3_ros2" 2>/dev/null || true
    pkill -9 -f "realsense2_camera/realsense2_camera_node" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true

    wait 2>/dev/null || true
    echo "=========================================="
    echo "  全部已停止。"
    echo "=========================================="
}

trap cleanup EXIT INT TERM

# ============================================================
# 4. 启动 STM32 身体 IMU 采集
# ============================================================

echo ""
echo "=========================================="
echo "  [1/3] 启动 6段身体 IMU 采集..."
echo "    STM32 → /dev/ttyACM0 @ 921600 baud"
echo "=========================================="
python3 -c "from imu_mocap_node.mocap_node import main; main()" &> /tmp/mocap_node.log &
PID_MOCAP=$!
echo "  PID $PID_MOCAP | 日志: /tmp/mocap_node.log"
echo ""

# ============================================================
# 5. 启动 ORB-SLAM3（内含 D435i、RViz、Pangolin）
# ============================================================

echo "=========================================="
echo "  [2/3] 启动 ORB-SLAM3..."
echo "    D435i 640×480@30fps color + IMU（内置启动）"
echo "    模式: $SENSOR_TYPE"
echo "=========================================="
ros2 launch orb_slam3_ros2 mapping.launch.py \
    use_pangolin:="$USE_PANGOLIN" \
    use_rviz:="$USE_RVIZ" \
    sensor_type:="$SENSOR_TYPE" \
    &> /tmp/orb_slam3.log &
PID_ORB_SLAM3=$!
echo "  PID $PID_ORB_SLAM3 | 日志: /tmp/orb_slam3.log"
echo ""

# ============================================================
# 6. 等待 ORB-SLAM3 就绪（最多 20s）
# ============================================================

echo "=========================================="
echo "  等待 ORB-SLAM3 就绪..."
echo "  （检测 /orb_odom 话题，最多 20s）"
echo "=========================================="
wait_for_topic "/orb_odom" "ORB-SLAM3" 20

# 非阻塞检查 STM32 状态
if grep -q "STM32 detected" /tmp/mocap_node.log 2>/dev/null; then
    echo "  ✓ STM32 动捕已就绪"
else
    echo "  ⚠ STM32 尚在连接中（后台重试，不影响录制）"
fi
echo ""

# ============================================================
# 7. 启动 rosbag 录制（从这里开始计时）
# ============================================================

BAGS_DIR="$SCRIPT_DIR/bags"
mkdir -p "$BAGS_DIR"

if [ -z "$BAG_NAME" ]; then
    BAG_PATH="$BAGS_DIR/rosbag_$(date +%Y-%m-%d_%H-%M-%S)"
else
    case "$BAG_NAME" in
        /*) BAG_PATH="$BAG_NAME" ;;
        *)  BAG_PATH="$BAGS_DIR/$BAG_NAME" ;;
    esac
fi

echo "=========================================="
echo "  [3/3] 开始录制 rosbag..."
echo "    输出: ${BAG_PATH}"
if [ "$DURATION" -gt 0 ]; then
    echo "    时长: ${DURATION} 秒（从此刻开始计时）"
fi
echo "=========================================="
echo ""
echo "  录制话题:"
echo "    ORB-SLAM3:  /orb_odom /pose_array /live_point_cloud /orb_camera/image"
echo "    D435i:      /camera/camera/color/image_raw /camera/camera/imu"
echo "    身体 IMU:   /*/imu（6段）"
echo "    足底反力:   /*/wrench（2段）"
echo "    TF:         /tf /tf_static"
echo ""
echo "  按 Ctrl+C 停止录制并关闭全部"
echo ""

TOPICS=(
    # ORB-SLAM3 输出
    "/orb_odom"
    "/pose_array"
    "/live_point_cloud"
    "/orb_camera/image"
    "/orb_camera/info"
    # D435i 彩色流（由 ORB-SLAM3 内部启动）
    "/camera/camera/color/image_raw"
    "/camera/camera/color/camera_info"
    "/camera/camera/imu"
    # 6段身体 IMU
    "/left_thigh/imu"  "/left_shank/imu"
    "/right_thigh/imu" "/right_shank/imu"
    "/left_foot/imu"   "/right_foot/imu"
    # 足底反力
    "/left_foot/wrench"  "/right_foot/wrench"
    # 坐标变换
    "/tf"  "/tf_static"
)

ros2 bag record "${TOPICS[@]}" \
    -o "$BAG_PATH" \
    --compression-mode file \
    --compression-format zstd \
    &> /tmp/rosbag_record.log &
PID_BAG=$!
echo "  rosbag PID $PID_BAG"

# ============================================================
# 8. 等待
# ============================================================

if [ "$DURATION" -gt 0 ]; then
    echo "  录制 ${DURATION} 秒后自动停止..."
    sleep "$DURATION"
    cleanup
else
    wait 2>/dev/null || true
fi
