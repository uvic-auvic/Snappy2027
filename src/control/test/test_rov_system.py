"""Exercise both C++ ROV nodes over DDS, with physical motor output disabled."""

import os
import random
import signal
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from snappy_interfaces.msg import Pose, RovCommand, ThrusterCommand
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger


def generate_test_description():
    domain = random.SystemRandom().randrange(20, 100)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    env = {"ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1"}
    params = {"max_depth_m": 5.0, "motor_output_enabled": False,
              "kill_timeout_s": 90.0, "pid_z": [0.72, 0.08, 0.76]}
    controller = launch_ros.actions.Node(
        package="snappy_control", executable="rov_depth_controller", name="controller",
        parameters=[params], additional_env=env, output="screen")
    input_process = launch_ros.actions.Node(
        package="snappy_control", executable="rov_controller", name="rov_controller",
        parameters=[params], additional_env=env, output="screen")
    return launch.LaunchDescription([
        controller, input_process, launch_testing.actions.ReadyToTest(),
    ]), {"domain": domain, "input_process": input_process}


class TestRovSystem(unittest.TestCase):
    def test_control_and_faults(self, domain, input_process, proc_info):
        rclpy.init(domain_id=domain)
        self.node = rclpy.create_node("rov_test_fixture")
        self.axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.buttons = [0] * 13
        self.depth = 2.0
        self.pressure_value = 2.0
        self.joy_age = 0.0
        self.latest = None
        self.allocated = None
        self.disabled = None
        self.latched = False
        self.joy = self.node.create_publisher(Joy, "/joy", 1)
        self.pressure = self.node.create_publisher(Float32, "/depth_data", 1)
        self.state = self.node.create_publisher(Pose, "/state_estimator/state", 1)
        self.node.create_subscription(RovCommand, "/rov/command", self.on_command, 1)
        self.node.create_subscription(ThrusterCommand, "/rov/allocated_thrusters", self.on_allocated, 1)
        self.node.create_subscription(ThrusterCommand, "/rov/disabled_motor_cmd", self.on_disabled,
                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(Bool, "/rov/emergency_stop", self.on_stop,
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.reset = self.node.create_client(Trigger, "/rov/reset_emergency_stop")
        try:
            proc_info.assertWaitForStartup(input_process, timeout=10)
            self.drive(0.8, sensors=False)
            self.assertIsNotNone(self.allocated)
            self.assertEqual(self.allocated, [0] * 8)
            self.drive(1.0)
            self.assertTrue(self.latest.depth_enabled)
            self.assertEqual(self.latest.depth_target, 2.0)
            self.assertEqual(self.node.count_publishers("/motor_cmd"), 0)
            self.assertEqual(self.node.count_subscribers("/planner/task"), 0)
            self.assertEqual(self.disabled, [0] * 8)

            # Forward, reverse, net triggers, strafe and yaw; depth remains active.
            self.buttons[4] = 1
            self.axes[5] = 0.0
            self.drive()
            self.assertAlmostEqual(self.latest.surge, 0.15)
            self.assertTrue(self.latest.manual_enabled)
            self.assertTrue(any(self.allocated[i] > 0 for i in (2, 6)))
            self.assertTrue(any(self.allocated[i] != 0 for i in (1, 3, 5, 7)))
            self.assertEqual(self.disabled, [0] * 8)
            self.axes[5], self.axes[2] = 1.0, -1.0
            self.drive()
            self.assertAlmostEqual(self.latest.surge, -0.3)
            self.axes[5], self.axes[2] = -0.6, 0.4
            self.drive()
            self.assertAlmostEqual(self.latest.surge, 0.15, places=6)
            self.axes[0], self.axes[3] = -1.0, 1.0
            self.drive()
            self.assertAlmostEqual(self.latest.sway, 0.3)
            self.assertAlmostEqual(self.latest.yaw, -0.3)
            self.assertTrue(all(abs(self.allocated[i]) <= 30 for i in (0, 2, 4, 6)))

            # D-pad hold steps once; release then press steps again.
            self.axes[7] = -1.0
            self.drive(0.7)
            self.assertEqual(self.latest.depth_target, 2.25)
            self.axes[7] = 0.0
            self.drive()
            self.axes[7] = -1.0
            self.drive()
            self.assertEqual(self.latest.depth_target, 2.5)
            self.axes[7] = 0.0
            self.drive()
            self.axes[7] = 1.0
            self.drive()
            self.assertEqual(self.latest.depth_target, 2.25)
            self.axes[7] = 0.0

            # The existing PID responds to depth error alongside manual effort.
            self.depth = 1.0
            self.drive(0.4)
            shallow_thrust = sum(self.allocated[i] for i in (1, 3, 5, 7))
            self.depth = 3.0
            self.drive(0.4)
            self.assertLess(sum(self.allocated[i] for i in (1, 3, 5, 7)), shallow_thrust)
            self.depth = 2.0

            self.buttons[4] = 0
            self.drive()
            self.assert_manual_zero()
            self.assertTrue(self.latest.depth_enabled)
            self.assertTrue(any(self.allocated))

            # Reconnect with L1 held cannot rearm until a fresh release and press.
            self.buttons[4] = 1
            self.drive()
            self.assertTrue(self.latest.manual_enabled)
            self.drive(0.8, joy=False)
            self.assert_manual_zero()
            self.assertTrue(self.latest.depth_enabled)
            self.drive()
            self.assertFalse(self.latest.manual_enabled)
            self.rearm()
            self.assertTrue(self.latest.manual_enabled)

            # Stale and malformed Joy cannot move the depth setpoint or arm.
            target = self.latest.depth_target
            self.joy_age, self.axes[7] = 2.0, -1.0
            self.drive()
            self.assert_manual_zero()
            self.assertEqual(self.latest.depth_target, target)
            self.joy_age, self.axes[7] = 0.0, 0.0
            self.axes = []
            self.drive()
            self.assert_manual_zero()
            self.axes = [0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0]
            self.rearm()

            # Pressure loss stops ALL motors despite fresh estimator predictions.
            self.drive(0.8, pressure=False)
            self.assertEqual(self.allocated, [0] * 8)
            self.assertFalse(self.latest.depth_enabled)
            self.depth, self.pressure_value = 2.7, 2.7
            self.drive()
            self.assertEqual(self.latest.depth_target, 2.7)
            self.assertFalse(self.latest.manual_enabled)
            self.rearm()
            self.pressure_value = float("nan")
            self.drive()
            self.assertEqual(self.allocated, [0] * 8)
            self.pressure_value = 2.7
            self.drive()
            self.rearm()

            # The lower controller watchdog works if the input process hangs.
            pid = proc_info[input_process].pid
            os.kill(pid, signal.SIGSTOP)
            try:
                self.drive(0.85, joy=False)
                self.assertEqual(self.allocated, [0] * 8)
            finally:
                os.kill(pid, signal.SIGCONT)
            self.drive()
            self.assertFalse(self.latest.manual_enabled)
            self.rearm()

            # Circle wins over malformed input and stays latched after release.
            self.buttons[1] = 1
            self.axes = []
            self.drive()
            self.assertTrue(self.latched)
            self.assertEqual(self.allocated, [0] * 8)
            self.axes = [0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0]
            self.buttons[1] = 0
            self.drive(0.7)
            self.assertEqual(self.allocated, [0] * 8)
            self.assertFalse(self.call_reset().success)  # L1 is still held.
            self.buttons[4] = 0
            self.drive()
            self.assertTrue(self.call_reset().success)
            self.drive()
            self.assertFalse(self.latched)
            self.assertTrue(self.latest.depth_enabled)
            self.assert_manual_zero()
            self.rearm()
            self.assertTrue(self.latest.manual_enabled)
            self.assertEqual(self.disabled, [0] * 8)
            self.assertEqual(self.node.count_publishers("/motor_cmd"), 0)
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

    def on_command(self, msg):
        self.latest = msg

    def on_allocated(self, msg):
        self.allocated = list(msg.thrust_pct)

    def on_disabled(self, msg):
        self.disabled = list(msg.thrust_pct)
        self.assertEqual(self.disabled, [0] * 8)

    def on_stop(self, msg):
        self.latched = msg.data

    def assert_manual_zero(self):
        self.assertEqual((self.latest.surge, self.latest.sway, self.latest.yaw), (0.0, 0.0, 0.0))
        self.assertFalse(self.latest.manual_enabled)

    def rearm(self):
        self.buttons[4] = 0
        self.drive()
        self.buttons[4] = 1
        self.drive()

    def call_reset(self):
        self.assertTrue(self.reset.wait_for_service(timeout_sec=2.0))
        future = self.reset.call_async(Trigger.Request())
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            self.drive(0.02)
        self.assertTrue(future.done())
        return future.result()

    def drive(self, duration=0.3, *, joy=True, sensors=True, pressure=True):
        end = time.monotonic() + duration
        next_publish = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_publish:
                next_publish = time.monotonic() + 0.02
                if sensors:
                    if pressure:
                        self.pressure.publish(Float32(data=self.pressure_value))
                    pose = Pose()
                    pose.position.z, pose.orientation.w = self.depth, 1.0
                    self.state.publish(pose)
                if joy:
                    msg = Joy()
                    msg.header.stamp = (self.node.get_clock().now() -
                                        rclpy.duration.Duration(seconds=self.joy_age)).to_msg()
                    msg.axes, msg.buttons = self.axes, self.buttons
                    self.joy.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.005)


@launch_testing.post_shutdown_test()
class TestExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
