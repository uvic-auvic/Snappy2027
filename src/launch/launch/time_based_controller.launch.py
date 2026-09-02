from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    serial_dev_arg = DeclareLaunchArgument(
        "serial_dev",
        default_value="/dev/ttyUSB1",
        description="Serial device for micro-ROS agent",
    )

    micro_ros_agent = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "micro_ros_agent",
            "micro_ros_agent",
            "serial",
            "--dev",
            LaunchConfiguration("serial_dev"),
            "-b",
            "115200",
        ],
        output="screen",
    )

    xsens_parameters_file_path = Path(
        get_package_share_directory("xsens_mti_ros2_driver"),
        "param",
        "xsens_mti_node.yaml",
    )

    state_estimator_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="snappy_estimation",
                executable="state_estimator",
                name="state_estimator",
                output="screen",
            )
        ],
    )

    pressure_sensor_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="snappy_drivers",
                executable="pressureSensor",
                name="pressure_sensor",
                output="screen",
            )
        ],
    )

    dvl_node = Node(
        package="waterlinked_dvl_driver",
        executable="waterlinked_dvl_driver",
        name="waterlinked_dvl_driver",
        output="screen",
    )

    xsens_node = Node(
        package="xsens_mti_ros2_driver",
        executable="xsens_mti_node",
        name="xsens_mti_node",
        output="screen",
        parameters=[str(xsens_parameters_file_path)],
    )

    mission_file = Path(
        get_package_share_directory("snappy_launch"),
        "config",
        "time_based_mission.yaml",
    )

    time_based_controller = Node(
        package="snappy_control",
        executable="time_based_controller",
        name="time_based_controller",
        output="screen",
        parameters=[
            {
                "mission_file": str(mission_file),
                "default_speed": 35.0,
                "state_topic": "/state_estimator/state",
                "command_rate_hz": 20.0,
                "kill_timeout_s": 120.0,
            }
        ],
    )

    bag_recording = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "mkdir -p /tmp/snappy_rosbags && ros2 bag record -o /tmp/snappy_rosbags/time_based_$(date +%Y%m%d_%H%M%S) "
            "/state_estimator/state /depth_data /imu/data /d455/camera/camera/imu "
            "/waterlinked_dvl_driver/odom /motor_cmd /controller/status /time_based_controller/status "
            "/planner/task /controller/task_done",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            serial_dev_arg,
            micro_ros_agent,
            xsens_node,
            pressure_sensor_node,
            dvl_node,
            state_estimator_node,
            time_based_controller,
            bag_recording,
        ]
    )
