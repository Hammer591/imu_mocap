#!/usr/bin/env bash
# One-click launch: D435i camera IMU + relay + STM32 6-segment body mocap
#
# 一键启动全部传感器采集：
#   - D435i 相机 IMU（陀螺仪 + 加速度计）  [可选]
#   - 相机 IMU 中继 → /imu0（Madgwick 姿态解算）
#   - STM32 6段身体 IMU + 足底反力
#
# 发布话题:
#   /imu0                  — D435i 相机 IMU（含四元数姿态）
#   /left_thigh/imu         — 左大腿 IMU
#   /left_shank/imu         — 左小腿 IMU
#   /right_thigh/imu        — 右大腿 IMU
#   /right_shank/imu        — 右小腿 IMU
#   /left_foot/imu          — 左脚 IMU
#   /right_foot/imu         — 右脚 IMU
#   /left_foot/wrench       — 左脚足底反力（CoP + GRF）
#   /right_foot/wrench      — 右脚足底反力（CoP + GRF）
#
# 前置条件:
#   - STM32 已上电，/dev/ttyACM0 可访问
#   - Intel RealSense D435i 已连接 USB（--no-realsense 可跳过）
#   - 已执行 ./build_ros2_mocap.sh 编译
#   - python3-serial 已安装（sudo apt install python3-serial）
#
# 用法:
#   ./run_all_mocap.sh                     # 启动全部（含 D435i）
#   ./run_all_mocap.sh --no-realsense       # 仅启动身体 IMU + 足底反力（跳过 D435i）
#   ROS2_DISTRO=foxy ./run_all_mocap.sh     # 指定 ROS2 发行版
#
# 按 Ctrl+C 停止所有进程。

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/ros_ws"
ROS2_DISTRO="${ROS2_DISTRO:-humble}"

# ============================================================
# 0. 命令行参数
# ============================================================
SKIP_REALSENSE=false
for arg in "$@"; do
    case "$arg" in
        --no-realsense) SKIP_REALSENSE=true ;;
        -h|--help)
            sed -n '/^#.*$/p' "$0" | grep -v '!/bin'
            exit 0
            ;;
    esac
done

# ============================================================
# 1. 环境准备
# ============================================================

# 检查工作空间是否已编译
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo "错误: 找不到 $WS_DIR/install/setup.bash"
    echo "请先执行: ./build_ros2_mocap.sh"
    exit 1
fi

# 检查 python3-serial
python3 -c "import serial" 2>/dev/null || {
    echo "错误: python3-serial 未安装"
    echo "请执行: sudo apt install python3-serial"
    exit 1
}

source "/opt/ros/${ROS2_DISTRO}/setup.bash"
source "$WS_DIR/install/setup.bash"

export IMU_MOCAP_DIR="$SCRIPT_DIR"

# PYTHONPATH 兼容（setuptools 72.x entry_points 问题）
_PY_VER="$(python3 --version | grep -oP '\d+\.\d+' | head -1)"
export PYTHONPATH="$WS_DIR/install/imu_mocap_node/lib/python${_PY_VER}/site-packages:$PYTHONPATH"

# ============================================================
# 2. 启动函数（各进程独立进程组，避免 shutdown 冲突）
# ============================================================

_CLEANUP_DONE=0
PID_D435I=""
PID_RELAY=""
PID_MOCAP=""

# 在独立进程组中启动 Python 节点，这样 Ctrl+C 不会直接传给它们
start_bg() {
    local label="$1"
    shift
    # 使用 setsid 创建新进程组，父进程 SIGINT 不影响子节点
    setsid -w "$@" &
    local pid=$!
    echo "  PID ${pid}"
    return 0
}

cleanup() {
    [ "$_CLEANUP_DONE" -ne 0 ] && return
    _CLEANUP_DONE=1

    echo ""
    echo "关闭所有进程..."

    # 先发 SIGTERM 让节点优雅退出
    for sig in TERM TERM TERM KILL; do
        any=""
        [ -n "$PID_MOCAP" ] && kill "$PID_MOCAP" 2>/dev/null && any=true
        [ -n "$PID_RELAY" ] && kill "$PID_RELAY" 2>/dev/null && any=true
        [ -n "$PID_D435I" ] && kill "$PID_D435I" 2>/dev/null && any=true
        [ -z "$any" ] && break
        sleep 0.3
    done
    wait 2>/dev/null || true

    echo "全部已停止。"
}

trap cleanup EXIT INT TERM

# ============================================================
# 3. 启动 D435i 相机（可选）
# ============================================================

if $SKIP_REALSENSE; then
    echo "跳过 D435i（--no-realsense）"
else
    echo "=========================================="
    echo "  [1/3] 启动 D435i 相机 IMU..."
    echo "    → /camera/camera/imu（200 Hz 陀螺仪+加速度计）"
    echo "=========================================="

    ros2 launch realsense2_camera rs_launch.py \
        enable_gyro:=true \
        enable_accel:=true \
        unite_imu_method:=2 \
        &> /tmp/d435i_camera.log &
    PID_D435I=$!
    echo "  PID ${PID_D435I} | 日志: /tmp/d435i_camera.log"

    # 等待相机初始化
    sleep 3

    if ! kill -0 "$PID_D435I" 2>/dev/null; then
        echo ""
        echo "错误: D435i 相机启动失败，请检查连接。"
        echo "日志摘要（/tmp/d435i_camera.log）:"
        tail -5 /tmp/d435i_camera.log 2>/dev/null
        echo ""
        echo "提示: 如果不需要 D435i，请使用 --no-realsense 参数"
        exit 1
    fi
    echo "  D435i 启动成功"
    echo ""
fi

# ============================================================
# 4. 启动相机 IMU 中继
# ============================================================

echo "=========================================="
if $SKIP_REALSENSE; then
    echo "  [1/2] 跳过相机 IMU 中继（无 D435i）"
else
    echo "  [2/3] 启动相机 IMU 中继..."
    echo "    → /camera/camera/imu → /imu0（Madgwick 姿态解算）"
fi
echo "=========================================="

if ! $SKIP_REALSENSE; then
    python3 -c "
import sys, os
# 从 setsid 启动后 PID=1，需要重新设置进程标题以便 kill 能找到
from imu_mocap_node.imu_relay import main
main()
" &
    PID_RELAY=$!
    echo "  PID ${PID_RELAY}"
    echo ""
fi

# ============================================================
# 5. 启动 STM32 6段身体传感器节点
# ============================================================

if $SKIP_REALSENSE; then
    echo "=========================================="
    echo "  [2/2] 启动身体 IMU 采集..."
else
    echo "=========================================="
    echo "  [3/3] 启动身体 IMU 采集..."
fi
echo "    STM32 → /dev/ttyACM0 @ 921600 baud"
echo "=========================================="
echo ""
echo "  6段 IMU:  /left_thigh  /left_shank"
echo "            /right_thigh /right_shank"
echo "            /left_foot   /right_foot"
echo "  足底反力:  /left_foot/wrench  /right_foot/wrench"
if ! $SKIP_REALSENSE; then
    echo "  相机 IMU:  /imu0"
fi
echo ""
echo "  查看数据: ros2 topic list"
echo "            ros2 topic echo /left_thigh/imu"
echo "            ros2 topic echo /left_foot/wrench"
echo "  按 Ctrl+C 停止全部"
echo ""

python3 -c "
import sys, os
from imu_mocap_node.mocap_node import main
main()
" &
PID_MOCAP=$!
echo "  PID ${PID_MOCAP}"

# ============================================================
# 6. 等待子进程结束
# ============================================================

wait 2>/dev/null || true
