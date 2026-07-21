# IMU Mocap — STM32 身体动作捕捉

STM32 采集 6 段身体传感器数据，通过 USB CDC 发送到上位机（Jetson / PC）。

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
上位机打开 /dev/ttyACM0 → 等待心跳
上位机发送 START_STREAM → STM32 确认并连续上传 60 个 float
上位机发送 STOP_STREAM → STM32 确认并停止上传，回到 IDLE / 心跳
```

## 文件结构

| 文件 | 说明 |
|---|---|
| `jetson_mocap_v2.py` | 协议客户端：双向控制 + 6 段身体模型（268 字节/包），带心跳/命令/响应 |
| `run_ros2_mocap.sh` | ROS2 节点一键启动脚本 |
| `ros_ws/` | ROS2 工作空间（含节点代码）|
| `notes/` | 开发记录、测试视频 |
| `.gitignore` | 忽略 `notes/*.mp4`、`ros_ws/build/`、`ros_ws/install/` |

## 快速部署（新环境）

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

### 3. 验证

```bash
# 自检
python3 jetson_mocap_v2.py self-test

# 查看数据（STM32 已上电）
python3 jetson_mocap_v2.py --port /dev/ttyACM0 run --print-every 10
```

### 4. ROS2 部署（可选）

目标环境为 ROS2 Humble（Ubuntu 22.04）：

```bash
# 装 ROS2 Humble（如果尚未安装）
sudo apt install ros-humble-ros-base

# 编译工作空间
cd ~/imu_mocap/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select imu_mocap_node

# 启动
./run_ros2_mocap.sh
```

ROS1 Noetic 用户需将 `rclpy` 改为 `rospy`，消息类型通用。

## 使用方式

```bash
# 完整采集（Ctrl+C 退出时会自动发送 STOP_STREAM）
python3 jetson_mocap_v2.py --port /dev/ttyACM0 run --csv data.csv --print-every 20

# 采集 30 秒自动停止
python3 jetson_mocap_v2.py --port /dev/ttyACM0 run --duration 30 --csv data.csv

# 仅监测心跳（不启动数据流）
python3 jetson_mocap_v2.py --port /dev/ttyACM0 idle
```

## ROS2 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/left_thigh/imu` `/left_shank/imu` `/right_thigh/imu` `/right_shank/imu` | `sensor_msgs/Imu` | 四元数姿态 + 角速度 + 线加速度 |
| `/left_foot/imu` `/right_foot/imu` | `sensor_msgs/Imu` | 同上 |
| `/left_foot/wrench` `/right_foot/wrench` | `geometry_msgs/WrenchStamped` | 足底压力中心 (torque) + 地面反作用力 (force.z) |

## 常见问题

- **Permission denied /dev/ttyACM0** → 执行 udev 规则后拔插 USB
- **数据全零 / seq 不变** → STM32 IMU 初始化失败，检查 IMU 接线或切断电源重启硬件，不可在运行时断开 IMU
- **ROS2 启动报 C 扩展缺失** → 系统 `python3` 与 ROS2 的 Python 版本不一致，使用 `python3.8` 运行
