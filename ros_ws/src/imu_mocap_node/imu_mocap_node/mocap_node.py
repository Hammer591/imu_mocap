#!/usr/bin/env python3
"""ROS2 node that uses V2 protocol (MocapClient) to publish 6-segment IMU data."""

import math
import os
import sys
import threading
import time
from typing import Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

# Find jetson_mocap_v2.py in the project root (set by run_ros2_mocap.sh)
_proj_dir = os.environ.get("IMU_MOCAP_DIR", "")
if _proj_dir and os.path.isdir(_proj_dir):
    sys.path.insert(0, _proj_dir)
from jetson_mocap_v2 import (  # noqa: E402
    FLAG_TIMESTAMP,
    MocapClient,
    MocapDataPacket,
    ResponsePacket,
    VALID_LEFT_FOOT,
    VALID_LEFT_SHANK,
    VALID_LEFT_THIGH,
    VALID_RIGHT_FOOT,
    VALID_RIGHT_SHANK,
    VALID_RIGHT_THIGH,
)


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert roll/pitch/yaw (degrees) to quaternion (x, y, z, w)."""
    cr = math.cos(math.radians(roll / 2))
    sr = math.sin(math.radians(roll / 2))
    cp = math.cos(math.radians(pitch / 2))
    sp = math.sin(math.radians(pitch / 2))
    cy = math.cos(math.radians(yaw / 2))
    sy = math.sin(math.radians(yaw / 2))

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


SEGMENTS = [
    ("left_thigh",  VALID_LEFT_THIGH),
    ("left_shank",  VALID_LEFT_SHANK),
    ("right_thigh", VALID_RIGHT_THIGH),
    ("right_shank", VALID_RIGHT_SHANK),
    ("left_foot",   VALID_LEFT_FOOT),
    ("right_foot",  VALID_RIGHT_FOOT),
]


class MocapNode(Node):
    def __init__(self):
        super().__init__("mocap_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("serial_timeout", 0.05)
        self.declare_parameter("heartbeat_timeout", 3.0)

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value
        serial_timeout = self.get_parameter("serial_timeout").value
        self._heartbeat_timeout = self.get_parameter("heartbeat_timeout").value

        # Publishers: one Imu per segment, plus Wrench for foot pressure
        self._imu_pubs = {}
        self._wrench_pubs = {}
        for name, _ in SEGMENTS:
            self._imu_pubs[name] = self.create_publisher(Imu, f"/{name}/imu", 10)
            if "foot" in name:
                self._wrench_pubs[name] = self.create_publisher(
                    WrenchStamped, f"/{name}/wrench", 10
                )

        self._client: Optional[MocapClient] = None
        self._client_lock = threading.Lock()
        self._running = True

        self._serial_thread = threading.Thread(
            target=self._serial_loop,
            args=(port, baudrate, serial_timeout),
            daemon=True,
        )
        self._serial_thread.start()

        self.get_logger().info(f"V2 client connecting to {port} @ {baudrate} baud")

    def _serial_loop(self, port: str, baudrate: int, serial_timeout: float):
        while self._running and rclpy.ok():
            try:
                client = MocapClient(port, baudrate, serial_timeout)
                client.on_data = self._on_data
                client.open()

                self.get_logger().info("Waiting for heartbeat ...")
                hb = client.wait_for_heartbeat(self._heartbeat_timeout)
                self.get_logger().info(
                    f"STM32 detected: state={hb.system_state}, "
                    f"uptime={hb.uptime_ms / 1000:.1f}s, "
                    f"valid=0x{hb.valid_mask:04X}"
                )

                with self._client_lock:
                    self._client = client

                response = client.start_stream()
                self.get_logger().info(
                    f"Stream started: state={response.system_state}, "
                    f"valid=0x{response.valid_mask:04X}"
                )

                while self._running and rclpy.ok():
                    time.sleep(0.1)

            except TimeoutError as exc:
                self.get_logger().error(str(exc))
                if self._running and rclpy.ok():
                    self.get_logger().info("Retrying in 1 s ...")
                    time.sleep(1.0)
            except Exception as exc:
                self.get_logger().error(f"Error: {exc}")
                with self._client_lock:
                    self._client = None
                if self._running and rclpy.ok():
                    self.get_logger().info("Reconnecting in 1 s ...")
                    time.sleep(1.0)
            else:
                break
            finally:
                self._cleanup_client()

        s = self._client.parser.stats if self._client else None
        if s:
            self.get_logger().info(
                f"Stats: ok={s.packets_ok}, crc_err={s.crc_errors}, "
                f"hdr_err={s.header_errors}, lost={s.estimated_lost_packets}"
            )

    def _on_data(self, packet: MocapDataPacket) -> None:
        stamp = rclpy.time.Time()
        if packet.header.flags & FLAG_TIMESTAMP and packet.header.timestamp_us > 0:
            stamp = TimeMsg(
                sec=packet.header.timestamp_us // 1_000_000,
                nanosec=(packet.header.timestamp_us % 1_000_000) * 1000,
            )
        else:
            now = time.monotonic_ns()
            stamp = TimeMsg(
                sec=now // 1_000_000_000,
                nanosec=now % 1_000_000_000,
            )

        for name, valid_flag in SEGMENTS:
            valid = packet.unit_is_valid(valid_flag)
            if "foot" in name:
                sample = getattr(packet, name.replace("_foot", "_foot"))
                imu_msg = self._build_imu_msg(
                    stamp, name,
                    sample.roll, sample.pitch, sample.yaw,
                    sample.gyrox, sample.gyroy, sample.gyroz,
                    sample.ax, sample.ay, sample.az,
                )
                self._imu_pubs[name].publish(imu_msg)

                wrench_msg = WrenchStamped()
                wrench_msg.header.stamp = stamp
                wrench_msg.header.frame_id = name
                # coP → torque (moment around x/y), GRF → force.z
                wrench_msg.wrench.torque.x = float(sample.copx)
                wrench_msg.wrench.torque.y = float(sample.copy)
                wrench_msg.wrench.force.z = float(sample.grf)
                self._wrench_pubs[name].publish(wrench_msg)
            else:
                sample = getattr(packet, name)
                imu_msg = self._build_imu_msg(
                    stamp, name,
                    sample.roll, sample.pitch, sample.yaw,
                    sample.gyrox, sample.gyroy, sample.gyroz,
                    sample.ax, sample.ay, sample.az,
                )
                self._imu_pubs[name].publish(imu_msg)

    def _build_imu_msg(
        self, stamp, frame_id: str,
        roll: float, pitch: float, yaw: float,
        gx: float, gy: float, gz: float,
        ax: float, ay: float, az: float,
    ) -> Imu:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        qx, qy, qz, qw = _rpy_to_quat(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        return msg

    def _cleanup_client(self):
        with self._client_lock:
            if self._client is not None:
                try:
                    self._client.stop_stream()
                except Exception:
                    pass
                self._client.close()
                self._client = None

    def shutdown(self):
        self._running = False
        self._cleanup_client()


def main(args=None):
    rclpy.init(args=args)
    node = MocapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
