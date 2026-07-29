# D435i 相机驱动安装与 IMU 话题配置

在 **Ubuntu 22.04 + ROS2 Humble** 上安装 Intel RealSense D435i 驱动，并通过中继节点将 IMU 数据发布到 `/imu0` 话题。

---

## 环境

| 项目 | 值 |
|---|---|
| 系统 | Ubuntu 22.04.5 LTS (Jammy) x86_64 |
| ROS2 | Humble |
| 相机 | Intel RealSense D435i |

---

## 1. 安装 librealsense2 SDK（硬件驱动层）

```bash
# 1. 添加 Intel 官方仓库密钥
#    方式 A：通过 curl 下载密钥文件到正确路径（推荐）
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo gpg --dearmor -o /usr/share/keyrings/librealsense-keyring.gpg

#    方式 B：通过 keyserver 补充获取最新签名密钥（如果 apt update 报 NO_PUBKEY）
gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys FB0B24895113F120
gpg --export FB0B24895113F120 | sudo tee -a /usr/share/keyrings/librealsense-keyring.gpg > /dev/null

# 2. 检查是否已有源列表（避免重复）
ls -la /etc/apt/sources.list.d/ | grep librealsense

#    如果 librealsense.list 不存在，手动创建：
echo "deb [signed-by=/usr/share/keyrings/librealsense-keyring.gpg] https://librealsense.intel.com/Debian/apt-repo jammy main" | sudo tee /etc/apt/sources.list.d/librealsense.list

#    如果已存在（且配置正确），跳过上一步。
#    如果 add-apt-repository 生成了重复的源列表（archive_uri-*），删掉它：
sudo rm /etc/apt/sources.list.d/archive_uri-https_librealsense_intel_com_debian_apt-repo-jammy.list 2>/dev/null

# 3. 更新源并安装 SDK
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

# 4. 插上 D435i，验证安装
realsense-viewer
```

> `realsense-viewer` 能检测到相机、看到 IMU 数据（Gyro + Accelerometer）即表示驱动正常。
> 如果提示 `No devices connected`，检查 USB 线缆（建议 USB 3.0）。

---

## 2. 安装 realsense2_camera ROS2 包装

```bash
# 从 apt 安装（推荐）
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-realsense2-camera

# 验证
ros2 pkg list | grep realsense
# 应输出: realsense2_camera
```

### 备选：源码编译（如需最新版本）

```bash
mkdir -p ~/realsense_ws/src && cd ~/realsense_ws/src
git clone -b ros2-development https://github.com/IntelRealSense/realsense-ros.git

sudo apt install ros-humble-diagnostic-updater ros-humble-nav-msgs
source /opt/ros/humble/setup.bash

cd ~/realsense_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 3. 构建本项目

```bash
cd ~/imu_mocap/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select imu_mocap_node

# 修复可执行权限（setuptools 72.x workaround）
find install/imu_mocap_node/lib/imu_mocap_node -maxdepth 1 -type f -exec chmod +x {} \;
source install/setup.bash
```

> **注意：** 如果系统安装了 miniconda/anaconda，`python3` 可能指向 conda 的 Python 版本（如 3.14），与 ROS2 Humble 的 Python 3.10 不兼容。
> 运行 ROS2 命令前务必退出 conda 环境：`conda deactivate`

---

## 4. 启动并验证 `/imu0`

需要**三个终端**，每个都先执行 `conda deactivate` 退出 conda 环境。

### 终端 1 — 启动 D435i 相机驱动（启用 IMU）

```bash
conda deactivate
source /opt/ros/humble/setup.bash

# 注意：v4.x 必须同时指定 enable_gyro 和 enable_accel，
# 且 unite_imu_method:=2 将 gyro + accel 合并为统一的 IMU 话题
ros2 launch realsense2_camera rs_launch.py \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2
```

启动后检查话题：
```bash
# 在另一个终端查看
ros2 topic list | grep imu
```

正常应看到 IMU 相关话题：
- `/camera/camera/accel/imu_info`
- `/camera/camera/gyro/imu_info`
- **`/camera/camera/imu`**（合并后的 IMU 数据，类型 `sensor_msgs/Imu`）

### 终端 2 — 启动中继节点（/camera/camera/imu → /imu0）

```bash
conda deactivate
cd ~/imu_mocap

source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash

export IMU_MOCAP_DIR="$PWD"

ros2 run imu_mocap_node imu_relay
```

看到日志 `Relaying /camera/camera/imu → /imu0 ... calibrating bias over 200 samples` 即启动成功。

中继节点（[imu_relay.py](../ros_ws/src/imu_mocap_node/imu_mocap_node/imu_relay.py)）功能：
- 订阅 `/camera/camera/imu` → 发布 `/imu0`
- 首 ~1s 自动校准 gyro bias（假设相机静止）
- 内嵌 **Madgwick AHRS** 滤波器，融合 gyro + accel 输出四元数姿态
- EMA 低通滤波抑制高频噪声

### 终端 3 — 验证话题

```bash
conda deactivate
source /opt/ros/humble/setup.bash
ros2 topic echo /imu0
```

正常输出示例（相机静止在桌面）：
```
header:
  stamp:
    sec: 1785309698
    nanosec: 705587456
  frame_id: imu0
orientation:
  x: 0.662  y: 0.008  z: 0.003  w: 0.749
angular_velocity:
  x: -0.0002  y: 0.0004  z: 0.0003    # 静止 ≈ 0
linear_acceleration:
  x: 0.10  y: -9.75  z: -0.30          # 总值 ≈ 9.81 m/s²（重力）
```

> **数据判定：** 静止时陀螺仪三轴 ≈ 0，加速度计模长 ≈ 9.81，姿态连续稳定，即正常工作。

---

## 5. 同时启动 mocap 全栈（可选）

如果 STM32 硬件已连接，可同时启动 6 段身体动作捕捉：

```bash
# 一键启动（默认 ROS2 distro 为 humble）
cd ~/imu_mocap
conda deactivate
./run_ros2_mocap.sh
```

话题对应关系：

| 话题 | 类型 | 数据来源 |
|---|---|---|
| `/imu0` | `sensor_msgs/Imu` | D435i 相机（Madgwick 融合后） |
| `/left_thigh/imu` `/left_shank/imu` ... | `sensor_msgs/Imu` | STM32 6 段身体 IMU |
| `/left_foot/wrench` `/right_foot/wrench` | `geometry_msgs/WrenchStamped` | 足底压力 |

---

## 常见问题

**Q：`realsense-viewer` 找不到相机？**
- 检查 USB 连接（插拔一次），建议使用 USB 3.0 口
- 检查 `lsusb` 是否识别到 Intel RealSense 设备
- 重新加载内核模块：`sudo modprobe -r hid_sensor_hub && sudo modprobe hid_sensor_hub`

**Q：`ros2 launch realsense2_camera rs_launch.py` 报找不到包？**
- 确认 `ros-humble-realsense2-camera` 已安装
- 确认已 `source /opt/ros/humble/setup.bash`

**Q：apt update 报 `NO_PUBKEY FB0B24895113F120`？**
- librealsense 仓库使用新版签名密钥，旧密钥文件里没有它
- 重新获取：`gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys FB0B24895113F120 && gpg --export FB0B24895113F120 | sudo tee -a /usr/share/keyrings/librealsense-keyring.gpg > /dev/null`

**Q：没有 `/camera/camera/imu` 话题？**
- 确认启动参数包含 `unite_imu_method:=2`
- v4.x 不再发布 `/camera/imu`，需要 `unite_imu_method` 合并 gyro + accel
- 用 `ros2 topic list | grep imu` 查看实际话题名

**Q：imu_relay 报错 `QoS incompatibility`？**
- `realsense2_camera` 发布 QoS 为 `BEST_EFFORT`，中继节点需用 `qos_profile_sensor_data` 订阅
- 已修复在代码中（`imu_relay.py` 使用 `qos_profile_sensor_data`），重新构建即可

**Q：imu_relay 报 import 错误 / C 扩展缺失？**
- 通常是因为 conda 的 Python 版本与 ROS2 的 Python 版本不一致
- 先执行 `conda deactivate` 退出 conda 环境再运行
- 检查 `python3 --version` 应输出 `Python 3.10.x`

**Q：`add-apt-repository` 之后 apt update 报重复源？**
- `add-apt-repository` 可能生成重复的 `.list` 文件
- 检查并删除多余文件：`sudo rm /etc/apt/sources.list.d/archive_uri-*.list`
