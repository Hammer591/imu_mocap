from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to launch RViz.",
            ),
            DeclareLaunchArgument(
                "sensor_type",
                default_value="imu-monocular",
                description="The mode which ORB_SLAM3 will run in.",
            ),
            DeclareLaunchArgument(
                "use_pangolin",
                default_value="true",
                description="Whether to use Pangolin for visualization.",
            ),
            DeclareLaunchArgument(
                "playback_bag",
                default_value="changeme",
                description="The rosbag to play during execution. If set, the \
                realsense2_camera node will not launch. Otherwise, nothing\
                will happen.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("orb_slam3_ros2"),
                            "launch",
                            "mapping.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "sensor_type": LaunchConfiguration("sensor_type"),
                    "use_pangolin": LaunchConfiguration("use_pangolin"),
                    "playback_bag": LaunchConfiguration("playback_bag"),
                }.items(),
            ),
            Node(
                package="orb_slam3_ros2",
                executable="orb_camera_info_node",
                output="screen",
            ),
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "use_mag": False,
                        "publish_tf": True,
                        "world_frame": "odom",
                        "base_link_frame": "base_link",
                    }
                ],
                remappings=[
                    ("/imu/data_raw", "/orb_camera/imu"),
                    ("/imu/data", "/rtabmap/imu"),
                ],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[
                    {
                        "frame_id": "base_link",
                        # "odom_frame_id": "odom",
                        "subscribe_rgb": True,
                        "subscribe_depth": False,
                        # "subscribe_scan": False,
                        # "approx_sync": True,
                        # "use_sim_time": LaunchConfiguration("use_sim_time"),
                        # "sync_queue_size": 10,
                        # "log_level": "debug",
                        # "stereo": "false",
                        # "odom_tf_linear_variance": 0.0005,
                        # "odom_tf_angular_variance": 0.0005,
                        "Mem/StereoFromMotion": "true",
                        # "initial_pose": "0 0 0 0 0 0", # only for localizing
                    }
                ],
                remappings=[
                    ("rgb/image", "/orb_camera/image"),
                    ("rgb/camera_info", "/orb_camera/info"),
                    ("odom", "/orb_odom"),
                    ("imu", "/rtabmap/imu")
                ],
                arguments=["--delete_db_on_start"],
            ),
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=[
                    {
                        "frame_id": "base_link",
                        "odom_frame_id": "odom",
                        "subscribe_rgb": True,
                        "approx_sync": False,  # False,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                remappings=[
                    ("rgb/image", "/orb_camera/image"),
                    ("rgb/camera_info", "/orb_camera/info"),
                ],
            ),
        ],
    )
