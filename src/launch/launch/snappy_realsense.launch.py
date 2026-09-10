"""
Launch file for Snappy ROV with RealSense cameras.
Uses the official RealSense rs_launch.py and launches all Snappy C++ nodes.

Usage:
    ros2 launch snappy_launch snappy_realsense.launch.py
    ros2 launch snappy_launch snappy_realsense.launch.py serial_dev:=/dev/ttyUSB1
"""

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare



def validate_serial_ports(context):
    assignments = {name: os.path.realpath(LaunchConfiguration(name).perform(context))
                   for name in ("serial_dev", "imu_port", "depth_port")}
    if len(set(assignments.values())) != len(assignments):
        raise RuntimeError(f"Motor board, IMU, and depth sensor must use different serial devices: {assignments}")
    return []


def generate_launch_description():

    snappyComputerVision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("snappy_computer_vision"),
                        "launch",
                        "snappy_computer_vision.launch.py",
                    ]
                )
            ]
        ),
    )

    dvl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("waterlinked_dvl_driver"),
                        "launch",
                        "dvl.launch.py",
                    ]
                )
            ]
        ),
        condition=IfCondition(LaunchConfiguration("enable_dvl")),
    )

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

    controller_parameters_file_path = Path(
        get_package_share_directory("snappy_launch"),
        "config",
        "controller_params.yaml",
    )

    xsens_mti_node = Node(
        package="xsens_mti_ros2_driver",
        executable="xsens_mti_node",
        name="xsens_mti_node",
        output="screen",
        parameters=[xsens_parameters_file_path, {
            "scan_for_devices": False,
            "port": LaunchConfiguration("imu_port"),
        }],
        arguments=[],
    )

    controller_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="snappy_control",
                executable="controller",
                name="controller",
                output="screen",
                parameters=[controller_parameters_file_path, {
                    "wait_for_task": ParameterValue(LaunchConfiguration("enable_planner"), value_type=bool),
                }],
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration("enable_controller"), "'.lower() == 'true' or '",
                    LaunchConfiguration("enable_planner"), "'.lower() == 'true'",
                ])),
            )
        ],
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

  #  solenoid_channel_node = TimerAction(
  #      period=3.0,
  #      actions=[
  #          Node(
  #              package="snappy_drivers",
  #              executable="solenoid_channel",
  #              name="solenoid_channel",
  #              output="screen",
  #          )
  #      ],
  #  )

    task_file_path = Path(
        get_package_share_directory("snappy_autonomy"),
        "config",
        "tasks_example.yaml"
        )

    planner_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="snappy_autonomy",
                executable="planner",
                name="planner",
                output="screen",
                parameters=[{"task_file": LaunchConfiguration("task_file")}],
                condition=IfCondition(LaunchConfiguration("enable_planner")),
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
                parameters=[{"serial_port": LaunchConfiguration("depth_port")}],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_controller", default_value="false", choices=["true", "false"], description="Run fixed-target control"),
            DeclareLaunchArgument("enable_planner", default_value="false", choices=["true", "false"], description="Run mission and controller together"),
            DeclareLaunchArgument("enable_dvl", default_value="false", choices=["true", "false"], description="Enable DVL hardware driver"),
            DeclareLaunchArgument("task_file", default_value=str(task_file_path)),
            DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB2"),
            DeclareLaunchArgument("depth_port", default_value="/dev/ttyUSB0"),
            serial_dev_arg,
            OpaqueFunction(function=validate_serial_ports),
            xsens_mti_node,
            micro_ros_agent,
            #snappyComputerVision,
            pressure_sensor_node,
            dvl,
            state_estimator_node,
            controller_node,
            planner_node,
  #          solenoid_channel_node,
        ]
    )
