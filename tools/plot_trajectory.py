#!/usr/bin/env python3
"""
从 rosbag 提取 /orb_odom 相机轨迹并可视化。

用法:
    python3 tools/plot_trajectory.py                      # 处理 bags/ 下最新的包
    python3 tools/plot_trajectory.py <bag目录或名称>       # 处理指定包
    python3 tools/plot_trajectory.py <bag> <输出png>       # 指定输出路径

输出:
    默认保存到 <bag所在目录>/trajectory_<bag时间戳>.png
    例如: bags/trajectory_2026-08-03_14-59-56.png
"""
import sys
import glob
import os
import sqlite3
import struct

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ_DIR = os.path.dirname(SCRIPT_DIR)          # 项目根目录
DEFAULT_BAGS_DIR = os.path.join(PROJ_DIR, 'bags')

# ============ 参数解析 ============
def find_latest_bag(bags_dir):
    """找到 bags/ 下最新的、包含可用数据的 rosbag 目录"""
    bags = sorted(glob.glob(os.path.join(bags_dir, 'rosbag_*')))
    if not bags:
        print(f"错误: {bags_dir} 下没有 rosbag 目录")
        sys.exit(1)
    # 从最新往旧找，跳过不完整（无 .db3 也无 .zstd）的目录
    for b in reversed(bags):
        if not os.path.isdir(b):
            continue
        has_data = glob.glob(os.path.join(b, '*.db3')) or \
                   glob.glob(os.path.join(b, '*.db3.zstd'))
        if has_data:
            return b
        print(f"跳过不完整的包: {b}")
    print(f"错误: {bags_dir} 下没有包含数据的包")
    sys.exit(1)

def parse_time_from_bag(bag_path):
    """从 bag 目录名提取时间戳，如 rosbag_2026-08-03_14-59-56 → 2026-08-03_14-59-56"""
    name = os.path.basename(bag_path)
    # 匹配 rosbag_YYYY-MM-DD_HH-MM-SS
    if name.startswith('rosbag_'):
        return name[len('rosbag_'):]
    return 'unknown'

def main():
    args = sys.argv[1:]
    if not args:
        bag_path = find_latest_bag(DEFAULT_BAGS_DIR)
        out_path = None
    else:
        bag_arg = args[0]
        # 支持相对/绝对路径，或仅包名
        if os.path.isdir(bag_arg):
            bag_path = bag_arg
        elif os.path.isdir(os.path.join(DEFAULT_BAGS_DIR, bag_arg)):
            bag_path = os.path.join(DEFAULT_BAGS_DIR, bag_arg)
        else:
            print(f"错误: 找不到 bag 目录: {bag_arg}")
            sys.exit(1)
        out_path = args[1] if len(args) > 1 else None

    # 默认输出路径: <bag所在目录>/trajectory_<时间戳>.png
    if out_path is None:
        ts = parse_time_from_bag(bag_path)
        bag_dir = os.path.dirname(bag_path) or DEFAULT_BAGS_DIR
        out_path = os.path.join(bag_dir, f'trajectory_{ts}.png')

    # ============ 读取 bag（支持 .db3 或压缩的 .db3.zstd）============
    dbs = glob.glob(os.path.join(bag_path, '*.db3'))
    if not dbs:
        # 没有裸 .db3，尝试解压 .zstd（ros2 bag 压缩存储格式）
        zstds = glob.glob(os.path.join(bag_path, '*.db3.zstd'))
        if not zstds:
            print(f"错误: 在 {bag_path} 中找不到 .db3 或 .db3.zstd 文件")
            sys.exit(1)
        zstd_path = zstds[0]
        db_path = zstd_path[:-len('.zstd')]  # 解压目标: xxx.db3
        print(f"检测到压缩包，解压中: {os.path.basename(zstd_path)}")
        import subprocess as _sp
        # 用系统 zstd 解压（不覆盖已有文件）
        result = _sp.run(['zstd', '-d', '-f', '-o', db_path, zstd_path],
                         capture_output=True, text=True)
        if result.returncode != 0 or not os.path.exists(db_path):
            print(f"解压失败: {result.stderr.strip()}")
            print("请检查是否安装了 zstd: sudo apt install zstd")
            sys.exit(1)
        print(f"解压完成: {os.path.basename(db_path)}")
        dbs = [db_path]
    db_path = dbs[0]

    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    rows = cur.execute(
        "SELECT t.id FROM topics t WHERE t.name = '/orb_odom'"
    ).fetchall()
    if not rows:
        print(f"错误: bag 中没有 /orb_odom 话题")
        sys.exit(1)
    msgs = cur.execute(
        "SELECT m.timestamp, m.data FROM messages m WHERE m.topic_id = ? ORDER BY m.timestamp",
        (rows[0][0],)
    ).fetchall()
    conn.close()
    print(f"处理包: {bag_path}")
    print(f"读取 {len(msgs)} 条 /orb_odom 消息")

    # ============ 解析 position (CDR, 固定偏移 44) ============
    POS_OFFSET = 44
    positions = []
    times = []
    for ts, data in msgs:
        try:
            x, y, z = struct.unpack_from('<ddd', data, POS_OFFSET)
        except Exception:
            continue
        # 过滤异常值
        if abs(x) > 100 or abs(y) > 100 or abs(z) > 100:
            continue
        positions.append((x, y, z))
        times.append(ts)

    if not positions:
        print("错误: 没有有效的轨迹点")
        sys.exit(1)

    arr = np.array(positions)
    times_s = (np.array(times) - times[0]) / 1e9

    # ============ 绘图 ============
    fig = plt.figure(figsize=(18, 6))

    # 1. 俯视图 X-Y
    ax1 = fig.add_subplot(1, 3, 1)
    ax1.plot(arr[:, 0], arr[:, 1], '-o', markersize=3, linewidth=1.2, color='blue', label='Trajectory')
    ax1.scatter(arr[0, 0], arr[0, 1], c='green', s=100, marker='s', label='Start')
    ax1.scatter(arr[-1, 0], arr[-1, 1], c='red', s=100, marker='x', label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title(f'Top View (XY)\n{len(positions)} points, {times_s[-1]:.1f}s')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')

    # 2. 3D 轨迹（三轴等比例，反映真实空间比例）
    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    ax2.plot(arr[:, 0], arr[:, 1], arr[:, 2], '-', linewidth=1.5, color='blue', label='Trajectory')
    ax2.scatter(arr[0, 0], arr[0, 1], arr[0, 2], c='green', s=80, marker='s', label='Start')
    ax2.scatter(arr[-1, 0], arr[-1, 1], arr[-1, 2], c='red', s=80, marker='x', label='End')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    ax2.set_title('3D Trajectory')
    ax2.legend()

    # 三轴等比例：统一所有轴的范围，使每个轴上的 1 米显示为相同长度
    x_span = arr[:, 0].max() - arr[:, 0].min()
    y_span = arr[:, 1].max() - arr[:, 1].min()
    z_span = arr[:, 2].max() - arr[:, 2].min()
    max_span = max(x_span, y_span, z_span, 1e-3)  # 防止全零
    center_x = (arr[:, 0].max() + arr[:, 0].min()) / 2
    center_y = (arr[:, 1].max() + arr[:, 1].min()) / 2
    center_z = (arr[:, 2].max() + arr[:, 2].min()) / 2
    half = max_span / 2 + 0.1  # 加 0.1m 边距，避免轨迹贴到边界
    ax2.set_xlim(center_x - half, center_x + half)
    ax2.set_ylim(center_y - half, center_y + half)
    ax2.set_zlim(center_z - half, center_z + half)
    # 使 3D 盒子本身也是立方体（等比例显示）
    try:
        ax2.set_box_aspect((1, 1, 1))
    except Exception:
        pass  # 旧版 matplotlib 可能不支持 set_box_aspect

    # 3. 各轴坐标随时间变化
    ax3 = fig.add_subplot(1, 3, 3)
    ax3.plot(times_s, arr[:, 0], label='X', color='red')
    ax3.plot(times_s, arr[:, 1], label='Y', color='green')
    ax3.plot(times_s, arr[:, 2], label='Z', color='blue')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (m)')
    ax3.set_title('Position vs Time')
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"轨迹图已保存: {out_path}")

    # 打印轨迹统计
    print(f"\n轨迹统计:")
    print(f"  起点: ({arr[0,0]:.3f}, {arr[0,1]:.3f}, {arr[0,2]:.3f})")
    print(f"  终点: ({arr[-1,0]:.3f}, {arr[-1,1]:.3f}, {arr[-1,2]:.3f})")
    print(f"  X范围: {arr[:,0].min():.3f} ~ {arr[:,0].max():.3f}")
    print(f"  Y范围: {arr[:,1].min():.3f} ~ {arr[:,1].max():.3f}")
    print(f"  Z范围: {arr[:,2].min():.3f} ~ {arr[:,2].max():.3f}")

if __name__ == '__main__':
    main()
