from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="imu_mocap_node",
            executable="mocap_node",
            name="mocap_node",
            parameters=[
                {"port": "/dev/ttyACM0"},
                {"baudrate": 921600},
                {"serial_timeout": 0.05},
                {"heartbeat_timeout": 3.0},
            ],
            output="screen",
        )
    ])
