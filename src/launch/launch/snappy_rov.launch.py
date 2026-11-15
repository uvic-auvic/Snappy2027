"""Dedicated ROV configuration. All hardware access/output is opt-in."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = [FindPackageShare("snappy_launch"), "config"]
    base = PathJoinSubstitution(config_dir + ["controller_params.yaml"])
    rov = PathJoinSubstitution(config_dir + ["rov_params.yaml"])
    arguments = [
        DeclareLaunchArgument("rov_params", default_value=rov,
                              description="ROV mapping and approved depth limits"),
        DeclareLaunchArgument("extra_params", default_value=LaunchConfiguration("rov_params"),
                              description="Optional final parameter overrides"),
        DeclareLaunchArgument("motor_output_enabled", default_value="false"),
        DeclareLaunchArgument("start_sensors", default_value="false",
                              description="Start pressure, Xsens and estimator; leave false if already running"),
        DeclareLaunchArgument("start_micro_ros_agent", default_value="false",
                              description="Start one agent only when no existing agent serves the board"),
        DeclareLaunchArgument("serial_dev", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("start_joy", default_value="false",
                              description="Start Linux joy_node when a real controller is connected"),
    ]
    params = [base, LaunchConfiguration("rov_params"), LaunchConfiguration("extra_params")]
    controller = Node(
        package="snappy_control", executable="rov_depth_controller", name="controller",
        output="screen", parameters=params + [{"motor_output_enabled": ParameterValue(
            LaunchConfiguration("motor_output_enabled"), value_type=bool)}])
    input_node = Node(package="snappy_control", executable="rov_controller",
                      name="rov_controller", output="screen", parameters=params)
    sensors = [
        Node(package="snappy_drivers", executable="pressureSensor", name="pressure_sensor",
             output="screen", condition=IfCondition(LaunchConfiguration("start_sensors"))),
        Node(package="xsens_mti_ros2_driver", executable="xsens_mti_node", name="xsens_mti_node",
             parameters=[PathJoinSubstitution([FindPackageShare("xsens_mti_ros2_driver"),
                                               "param", "xsens_mti_node.yaml"])],
             condition=IfCondition(LaunchConfiguration("start_sensors"))),
        Node(package="snappy_estimation", executable="state_estimator", name="state_estimator",
             output="screen", condition=IfCondition(LaunchConfiguration("start_sensors"))),
    ]
    agent = ExecuteProcess(
        cmd=["ros2", "run", "micro_ros_agent", "micro_ros_agent", "serial", "--dev",
             LaunchConfiguration("serial_dev"), "-b", "115200"],
        condition=IfCondition(LaunchConfiguration("start_micro_ros_agent")), output="screen")
    joy = Node(package="joy", executable="joy_node", name="joy_node",
               parameters=[{"autorepeat_rate": 50.0, "deadzone": 0.0}],
               condition=IfCondition(LaunchConfiguration("start_joy")))
    return LaunchDescription(arguments + sensors + [agent, joy, controller, input_node])
