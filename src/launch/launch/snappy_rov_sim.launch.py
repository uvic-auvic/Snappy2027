"""Motor-disabled ROV exercise with synthetic Joy, pressure and estimator state."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("snappy_launch")
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="neutral"),
        DeclareLaunchArgument("start_simulator", default_value="true",
                              description="Disable to run and replace fixture scenarios in a separate terminal"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, "launch", "snappy_rov.launch.py"])),
            launch_arguments={
                "extra_params": PathJoinSubstitution([share, "config", "rov_test_params.yaml"]),
                "motor_output_enabled": "false",
                "start_sensors": "false", "start_micro_ros_agent": "false", "start_joy": "false",
            }.items()),
        Node(package="snappy_launch", executable="rov_simulator.py", output="screen",
             arguments=["--scenario", LaunchConfiguration("scenario")],
             condition=IfCondition(LaunchConfiguration("start_simulator"))),
    ])
