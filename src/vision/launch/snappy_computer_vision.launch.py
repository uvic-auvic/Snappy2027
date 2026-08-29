from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments for serial numbers (with defaults)
    # Serial numbers must be wrapped in single quotes for RealSense parameter type handling
    serial_no_d455_arg = DeclareLaunchArgument(
        "serial_no_d455",
        default_value="'239222301226'",
        description="Serial number of D455 camera",
    )

    serial_no_d405_arg = DeclareLaunchArgument(
        "serial_no_d405",
        default_value="'230422271334'",
        description="Serial number of D405 camera",
    )

    # Get launch configuration
    serial_no_d455 = LaunchConfiguration("serial_no_d455")
    serial_no_d405 = LaunchConfiguration("serial_no_d405")

    # D455 Front Camera
    d455_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "camera_namespace": "d455",
            "camera_name": "d455",
            "serial_no": serial_no_d455,
            "json_file_path": PathJoinSubstitution(
                [FindPackageShare("snappy_computer_vision"), "config", "d455_underwater.json"]
            ),
            # "json_file_path": PathJoinSubstitution([FindPackageShare("snappy_computer_vision"), "config", "test455.json"]),
            "rgb_camera.color_profile": "848x480x30",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "enable_sync": "true",
            "align_depth.enable": "true",
        }.items(),
    )

    # D405 Bottom Camera
    d405_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "camera_namespace": "d405",
            "camera_name": "d405",
            "serial_no": serial_no_d405,
            "json_file_path": PathJoinSubstitution(
                [FindPackageShare("snappy_computer_vision"), "config", "d405_underwater.json"]
            ),
            # "json_file_path": PathJoinSubstitution([FindPackageShare("snappy_computer_vision"), "config", "test405.json"]),
            "rgb_camera.color_profile": "848x480x30",
            "enable_gyro": "false",  # D405 does not include IMU
            "enable_accel": "false",
            "unite_imu_method": "0",
            "enable_sync": "true",
            "align_depth.enable": "true",
        }.items(),
    )

    # Single shared TensorRT inference node serving both cameras (D455 front,
    # D405 bottom). Replaces the former front_cam + bottom_cam processes: one
    # CUDA context / engine for all cameras. Add a camera by appending its
    # namespace to camera_namespaces.
    #
    front_camera_vision = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="snappy_computer_vision",
                executable="front_camera_vision",
                name="front_camera_vision",
                output="screen",
                parameters=[{
                    "engine_path": "/ros2_ws/models/ffc_rs_26.engine",
                    "video_capture_enabled": True,
                    "video_capture_directory": "/home/snappy_data/front_camera",
                    "video_capture_fps": 30.0,
                }],
            )
        ],
    )

    bottom_camera_vision = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="snappy_computer_vision",
                executable="bottom_camera_vision",
                name="bottom_camera_vision",
                output="screen",
                parameters=[{"engine_path": "/ros2_ws/models/dfc_rs_26.engine"}],
            )
        ],
    )

    return LaunchDescription(
        [
            serial_no_d455_arg,
            serial_no_d405_arg,
            d455_launch,
            d405_launch,
            front_camera_vision,
            bottom_camera_vision,
        ]
    )
