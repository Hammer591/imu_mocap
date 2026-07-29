"""IMU relay: /camera/camera/imu → /imu0 with Madgwick AHRS + auto gyro bias calibration.

Subscribes to D435i gyro + accel from realsense2_camera, applies online
gyro bias calibration (first ~1 s averaged as bias), runs Madgwick filter
for orientation, and publishes as /imu0 matching mocap 6-segment format.

Run:
    ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=2
    ros2 run imu_mocap_node imu_relay
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class MadgwickAHRS:
    """Madgwick filter (gyro + accel only, no magnetometer)."""
    def __init__(self, beta: float = 0.1):
        self.beta = beta
        self.q = (1.0, 0.0, 0.0, 0.0)
        self._sample_period = 1.0 / 100.0

    def update(self, gx: float, gy: float, gz: float,
               ax: float, ay: float, az: float, dt: float) -> tuple:
        self._sample_period = dt
        qw, qx, qy, qz = self.q

        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-8:
            return self.q
        ax /= norm
        ay /= norm
        az /= norm

        # Gradient descent (accelerometer only)
        f1 = 2.0 * (qx * qz - qw * qy) - ax
        f2 = 2.0 * (qw * qx + qy * qz) - ay
        f3 = 2.0 * (0.5 - qx * qx - qy * qy) - az

        s0 = -2.0 * qy * f1 + 2.0 * qx * f2
        s1 = 2.0 * qz * f1 + 2.0 * qw * f2 - 4.0 * qx * f3
        s2 = -2.0 * qw * f1 + 2.0 * qz * f2 - 4.0 * qy * f3
        s3 = 2.0 * qx * f1 + 2.0 * qy * f2

        norm_s = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        if norm_s > 1e-10:
            s0 /= norm_s
            s1 /= norm_s
            s2 /= norm_s
            s3 /= norm_s

        # Gyroscope integration
        qw_dot = 0.5 * (-qx * gx - qy * gy - qz * gz)
        qx_dot = 0.5 * (qw * gx + qy * gz - qz * gy)
        qy_dot = 0.5 * (qw * gy - qx * gz + qz * gx)
        qz_dot = 0.5 * (qw * gz + qx * gy - qy * gx)

        qw -= (qw_dot - self.beta * s0) * dt
        qx -= (qx_dot - self.beta * s1) * dt
        qy -= (qy_dot - self.beta * s2) * dt
        qz -= (qz_dot - self.beta * s3) * dt

        norm_q = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if norm_q > 1e-10:
            self.q = (qw / norm_q, qx / norm_q, qy / norm_q, qz / norm_q)
        else:
            self.q = (qw, qx, qy, qz)
        return self.q


class IMURelay(Node):
    def __init__(self):
        super().__init__("imu_relay")

        self.declare_parameter("beta", 0.1)
        self.declare_parameter("calib_samples", 200)  # ~1 s at 200 Hz
        self.declare_parameter("lpf_alpha", 0.15)  # EMA low-pass: lower = smoother

        beta = self.get_parameter("beta").value
        lpf_alpha = self.get_parameter("lpf_alpha").value
        self._filter = MadgwickAHRS(beta)
        self._pub = self.create_publisher(Imu, "/imu0", 10)
        self._sub = self.create_subscription(
            Imu, "/camera/camera/imu", self._on_imu, qos_profile_sensor_data
        )
        self._last_ts = None

        # Gyro bias calibration
        self._calib_count = 0
        self._calib_target = self.get_parameter("calib_samples").value
        self._bias_x = 0.0
        self._bias_y = 0.0
        self._bias_z = 0.0
        self._calibrated = False

        # Low-pass filter state (EMA) — smooths noise from old driver
        self._lpf_alpha = lpf_alpha
        self._lpf_x = 0.0
        self._lpf_y = 0.0
        self._lpf_z = 0.0
        self._lpf_initialized = False

        self.get_logger().info(
            f"Relaying /camera/camera/imu → /imu0 (Madgwick beta={beta}, "
            f"gyro LPF alpha={lpf_alpha}, "
            f"calibrating bias over {self._calib_target} samples)"
        )

    def _on_imu(self, msg: Imu):
        now = msg.header.stamp
        if self._last_ts is not None:
            dt_s = ((now.sec - self._last_ts.sec) +
                    (now.nanosec - self._last_ts.nanosec) / 1e9)
        else:
            dt_s = 1.0 / 100.0
        self._last_ts = now

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Online gyro bias calibration (first N samples, assume stationary)
        if not self._calibrated:
            n = self._calib_count
            self._bias_x = (self._bias_x * n + gx) / (n + 1)
            self._bias_y = (self._bias_y * n + gy) / (n + 1)
            self._bias_z = (self._bias_z * n + gz) / (n + 1)
            self._calib_count += 1
            if self._calib_count >= self._calib_target:
                self._calibrated = True
                self.get_logger().info(
                    f"Gyro bias calibrated: "
                    f"({self._bias_x:.4f}, {self._bias_y:.4f}, {self._bias_z:.4f}) rad/s"
                )
            # During calibration, publish raw (no filtering)
            gx_c = gx
            gy_c = gy
            gz_c = gz
        else:
            gx_c = gx - self._bias_x
            gy_c = gy - self._bias_y
            gz_c = gz - self._bias_z

        # Low-pass filter (EMA) to suppress high-frequency noise from old driver
        if self._calibrated:
            if not self._lpf_initialized:
                self._lpf_x = gx_c
                self._lpf_y = gy_c
                self._lpf_z = gz_c
                self._lpf_initialized = True
            else:
                a = self._lpf_alpha
                self._lpf_x += a * (gx_c - self._lpf_x)
                self._lpf_y += a * (gy_c - self._lpf_y)
                self._lpf_z += a * (gz_c - self._lpf_z)
            gx_c = self._lpf_x
            gy_c = self._lpf_y
            gz_c = self._lpf_z

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        qw, qx, qy, qz = self._filter.update(
            gx_c, gy_c, gz_c, ax, ay, az, max(dt_s, 0.001)
        )

        out = Imu()
        out.header.stamp = now
        out.header.frame_id = "imu0"

        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw

        out.angular_velocity.x = gx_c
        out.angular_velocity.y = gy_c
        out.angular_velocity.z = gz_c
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IMURelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
