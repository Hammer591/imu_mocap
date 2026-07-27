# IMU Mocap — STM32 身体动作捕捉

STM32 采集 6 段身体传感器数据，通过 USB CDC 发送到 Jetson 运行全套 mocap + SLAM。

## 运行环境

| | 开发期（当前） | 最终部署 |
|---|---|---|
| 硬件 | PC（Ubuntu 20.04） | Jetson（Ubuntu 22.04） |
| ROS2 | Foxy | **Humble** |
| 运行内容 | 仅 mocap 节点（验证 IMU 数据） | mocap + RTAB-Map SLAM 全栈 |

> 当前在 PC（Ubuntu 20.04）上做开发验证，全部代码最终在 Jetson（Ubuntu 22.04）上运行。
> IMU mocap 节点是纯 Python rclpy，Foxy 与 Humble API 兼容，迁移只需改一行 ROS2 环境路径。

## 数据排列

每个数据包包含 60 个 float32，分为 6 段：

| 范围 | 名称 | 内容 |
|---|---|---|
| 0..8 | left_thigh | roll pitch yaw gyrox gyroy gyroz ax ay az |
| 9..17 | left_shank | 同上 |
| 18..26 | right_thigh | 同上 |
| 27..35 | right_shank | 同上 |
| 36..47 | left_foot | copx copy grf roll pitch yaw gyrox gyroy gyroz ax ay az |
| 48..59 | right_foot | 同上 |

## 协议通信流程

```text
STM32 上电 → IDLE / 1Hz 心跳
Jetson 打开 /dev/ttyACM0 → 等待心跳
Jetson 发送 START_STREAM → STM32 确认并连续上传 60 个 float
Jetson 发送 STOP_STREAM → STM32 确认并停止上传，回到 IDLE / 心跳
```

## 文件结构

| 文件 / 目录 | 说明 |
|---|---|
| `jetson_mocap_v2.py` | V3 协议客户端：双向控制 + 6 段身体模型（268 字节/包），带心跳/命令/响应 |
| `run_ros2_mocap.sh` | ROS2 节点一键启动脚本 |
| `ros_ws/src/imu_mocap_node/` | ROS2 软件包（节点代码 + launch 配置） |
| `ros_ws/src/imu_mocap_node/imu_mocap_node/mocap_node.py` | ROS2 节点：发布 6 段 IMU + 足底力话题 |
| `ros_ws/src/imu_mocap_node/launch/mocap.launch.py` | ROS2 launch 文件 |
| `rtabmap/` | rtabmap C++ 核心库（humble-devel 分支），**Jetson 上编译** |
| `slam_ws/src/rtabmap_ros/` | rtabmap ROS2 包装（ros2 分支），**Jetson 上编译** |
| `notes/` | 开发记录、测试视频 |
| `.gitignore` | 忽略 `notes/*.mp4`、`ros_ws/build/`、`ros_ws/install/` |

## 快速启动（开发环境 / Ubuntu 20.04 + ROS2 Foxy）

### 1. 依赖

```bash
sudo apt update
sudo apt install python3-serial
```

### 2. 串口权限

```bash
echo 'SUBSYSTEM=="tty", KERNEL=="ttyACM*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ttyacm.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3. 验证脚本

```bash
# 自检（不连硬件）
python3 jetson_mocap_v2.py self-test

# 查看数据（STM32 已上电）
python3 jetson_mocap_v2.py --port /dev/ttyACM0 run --print-every 10
```

### 4. ROS2 mocap 节点

```bash
cd ~/imu_mocap/ros_ws
source /opt/ros/foxy/setup.bash    # 开发期；部署时改为 humble
colcon build --packages-select imu_mocap_node
./run_ros2_mocap.sh
```

## 使用方式

```bash
# 完整采集（Ctrl+C 退出时自动发送 STOP_STREAM）
python3 jetson_mocap_v2.py --port /dev/ttyACM0 run --csv data.csv --print-every 20

# 采集 30 秒自动停止
python3 jetson_mocap_v2.py --port /dev/ttyACM0 run --duration 30 --csv data.csv

# 仅监测心跳（不启动数据流）
python3 jetson_mocap_v2.py --port /dev/ttyACM0 idle
```

## ROS2 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/imu0` | `sensor_msgs/Imu` | D435i 相机内置 IMU（gyro + accel，无姿态） |
| `/left_thigh/imu` `/left_shank/imu` `/right_thigh/imu` `/right_shank/imu` | `sensor_msgs/Imu` | 四元数姿态（RPY→四元数）+ 角速度 + 线加速度 |
| `/left_foot/imu` `/right_foot/imu` | `sensor_msgs/Imu` | 同上 |
| `/left_foot/wrench` `/right_foot/wrench` | `geometry_msgs/WrenchStamped` | 足底压力中心 (torque) + 地面反作用力 (force.z) |

### D435i 相机 IMU（可选）

中继节点，将 `realsense2_camera` 发布的 `/camera/imu` 重映射到 `/imu0`，与 mocap 系统松耦合。

```bash
# 终端 1：启动 D435i 相机（Humble 环境）
ros2 launch realsense2_camera rs_launch.py

# 终端 2：启动中继 /camera/imu → /imu0
ros2 run imu_mocap_node imu_relay

# 终端 3：启动 mocap
ros2 run imu_mocap_node mocap_node
```

话题对应：`/imu0` = D435i 相机 IMU，`/left_*/imu` `/right_*/imu` = Mocap 6 段。

## 最终部署（Jetson Ubuntu 22.04 + ROS2 Humble）

### 迁移步骤

1. 代码无需修改（纯 Python rclpy，Foxy ↔ Humble API 兼容）
2. `run_ros2_mocap.sh` 中 `source /opt/ros/foxy/setup.bash` → `source /opt/ros/humble/setup.bash`
3. 编译 RTAB-Map 全栈（见下）

### 编译 RTAB-Map

```bash
# 1. 编译 rtabmap 核心库
cd ~/imu_mocap/rtabmap
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig

# 2. 编译 rtabmap_ros
cd ~/imu_mocap/slam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 3. 启动 SLAM
source install/setup.bash
ros2 launch rtabmap_launch rtabmap.launch.py
```

## 常见问题

- **Permission denied /dev/ttyACM0** → 执行 udev 规则后拔插 USB
- **数据全零 / seq 不变** → STM32 IMU 初始化失败，检查 IMU 接线或切断电源重启硬件，不可在运行时断开 IMU
- **ROS2 启动报 C 扩展缺失** → 系统 `python3` 与 ROS2 的 Python 版本不一致，使用 `python3.8` 运行
