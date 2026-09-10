#!/usr/bin/env python3
"""Exercise real ROS nodes with simulated state and remapped motor output only."""
import math
import os
from pathlib import Path
import signal
import pty
import importlib.util
import subprocess
import tempfile
import time
import unittest

# Private DDS domain AND explicit motor-topic remapping: never publish /motor_cmd.
os.environ["ROS_DOMAIN_ID"] = "207"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import Imu
from snappy_interfaces.msg import Pose, Task, ThrusterCommand


class ControlIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node("control_test")
        self.temp = tempfile.TemporaryDirectory(prefix="snappy-control-test-")
        self.processes = []
        self.prefix = "/test_" + str(time.monotonic_ns())
        self.messages = {key: [] for key in ("motor", "target", "wrench", "done", "status", "tasks", "abort", "planner", "estimate", "depth")}
        self.topics = {
            "/motor_cmd": "motor", "/controller/target": "target", "/controller/wrench_requested": "wrench",
            "/controller/wrench_allocated": "allocated", "/controller/task_done": "done",
            "/controller/status": "status", "/planner/task": "tasks", "/planner/abort": "abort",
            "/planner/status": "planner", "/state_estimator/state": "state",
            "/depth_data": "depth", "/imu/data": "imu",
        }
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs = []
        for key, cls, qos in [
            ("motor", ThrusterCommand, QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)),
            ("target", Pose, transient), ("wrench", WrenchStamped, 100), ("done", Int32, 10),
            ("status", String, transient), ("tasks", Task, transient), ("abort", String, transient),
            ("planner", String, transient),
        ]:
            self.subs.append(self.node.create_subscription(cls, self.prefix + "/" + key,
                lambda msg, key=key: self.messages[key].append(msg), qos))
        self.state_pub = self.node.create_publisher(Pose, self.prefix + "/state", 10)
        self.task_pub = self.node.create_publisher(Task, self.prefix + "/tasks", transient)
        self.done_pub = self.node.create_publisher(Int32, self.prefix + "/done", 10)
        self.depth_pub = self.node.create_publisher(Float32, self.prefix + "/depth", 10)
        self.imu_pub = self.node.create_publisher(Imu, self.prefix + "/imu", 10)
        self.subs.append(self.node.create_subscription(Pose, self.prefix + "/state",
                         lambda msg: self.messages["estimate"].append(msg), 10))
        self.subs.append(self.node.create_subscription(Float32, self.prefix + "/depth",
                         lambda msg: self.messages["depth"].append(msg), 10))
        self.state = None
        self.depth_input = None
        self.imu_input = None

    def tearDown(self):
        for process, log in reversed(self.processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            log.close()
        self.node.destroy_node()
        rclpy.shutdown()
        self.temp.cleanup()

    def launch(self, package, executable, params):
        binary = Path(get_package_prefix(package)) / "lib" / package / executable
        args = [str(binary), "--ros-args"]
        for topic, name in self.topics.items():
            args += ["-r", topic + ":=" + self.prefix + "/" + name]
        for key, value in params.items():
            args += ["-p", key + ":=" + str(value)]
        log = (Path(self.temp.name) / (executable + ".log")).open("w+")
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        self.processes.append((process, log))
        return process

    def controller(self, **overrides):
        params = dict(kill_timeout_s=30.0, task_timeout_s=15.0, state_timeout_s=0.5,
                      settle_time_s=0.2, wait_for_task="true", pid_z="[1.0, 0.0, 10.0]",
                      pid_roll="[1.0, 0.0, 1.0]", pid_pitch="[1.0, 0.0, 1.0]",
                      pid_yaw="[1.0, 0.0, 1.725]")
        params.update(overrides)
        return self.launch("snappy_control", "controller", params)

    def spin(self, duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            if self.state is not None:
                self.state_pub.publish(self.state)
            if self.depth_input is not None:
                self.depth_pub.publish(Float32(data=self.depth_input))
            if self.imu_input is not None:
                self.imu_input.header.stamp = self.node.get_clock().now().to_msg()
                self.imu_pub.publish(self.imu_input)
            # Drain all observed topics; one callback per loop lets old motor
            # messages queue behind status, state and wrench callbacks.
            rclpy.spin_once(self.node, timeout_sec=0.001)
            for _ in range(20):
                rclpy.spin_once(self.node, timeout_sec=0)
            time.sleep(0.01)
            for process, log in self.processes:
                if process.poll() is not None:
                    log.flush()
                    log.seek(0)
                    self.fail("ROS process exited: " + log.read())

    def wait_for(self, predicate, timeout=5):
        until = time.monotonic() + timeout
        while not predicate() and time.monotonic() < until:
            self.spin(.05)
        self.assertTrue(predicate(), {key: str(values[-1:]) for key, values in self.messages.items()})

    def pose(self, z, roll=0):
        self.state = Pose()
        self.state.position.z = float(z)
        self.state.orientation.x = math.sin(roll / 2)
        self.state.orientation.w = math.cos(roll / 2)

    def assert_zero_since(self, cutoff):
        recent = [m for m in self.messages["motor"]
                  if m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec >= cutoff]
        self.assertTrue(recent, "No motor commands received after stop")
        self.assertTrue(all(not any(m.thrust_pct) for m in recent), "Stopped controller restarted")

    def task(self, seq, z, **angles):
        message = Task(seq=seq, z=float(z), **angles)
        self.task_pub.publish(message)
        return message

    def test_controller_transitions_and_state_loss(self):
        self.controller()
        self.wait_for(lambda: len(self.messages["motor"]) > 3)
        self.assertTrue(all(not any(m.thrust_pct) for m in self.messages["motor"]))
        self.pose(0)
        self.spin(.3)
        self.assertFalse(self.messages["wrench"], "Controller moved before its first task")
        self.task(0, 2)
        self.wait_for(lambda: self.messages["wrench"])
        self.assertAlmostEqual(self.messages["wrench"][0].wrench.force.z, 7, places=4)
        self.assertEqual(self.messages["target"][-1].position.z, 2)
        self.spin(.15)
        self.assertFalse(self.messages["done"], "Unreached task was acknowledged")
        self.pose(2)
        self.spin(.1)
        self.assertFalse(self.messages["done"], "Task completed before settling")
        self.wait_for(lambda: any(m.data == 0 for m in self.messages["done"]))
        # Same task retransmission should recover an ACK, not restart the task.
        before = len(self.messages["done"])
        self.task(0, 2)
        self.wait_for(lambda: len(self.messages["done"]) > before)
        self.messages["wrench"].clear()
        self.task(1, 3, yaw=math.pi / 2)
        self.wait_for(lambda: self.messages["target"][-1].position.z == 3)
        self.spin(.2)
        self.assertLess(max(abs(m.wrench.torque.z) for m in self.messages["wrench"]), 2,
                        "Planner target produced a yaw derivative spike")
        # At 90 degrees roll, depth-holding force belongs on body Y.
        self.pose(3, math.pi / 2)
        self.task(2, 3, roll=math.pi / 2)
        self.spin(.4)
        wrench = self.messages["wrench"][-1].wrench
        self.assertAlmostEqual(wrench.force.y, 5, places=4)
        self.assertAlmostEqual(wrench.force.z, 0, places=4)
        self.state = None
        self.wait_for(lambda: any("stale" in m.data for m in self.messages["status"]))
        cutoff = self.node.get_clock().now().nanoseconds
        self.pose(3)
        self.task(3, 4)
        self.messages["motor"].clear()
        self.spin(.4)
        self.assert_zero_since(cutoff)

    def test_kill_timer_stays_latched(self):
        self.pose(0)
        self.controller(wait_for_task="false", kill_timeout_s=2.0)
        self.wait_for(lambda: any(any(m.thrust_pct) for m in self.messages["motor"]))
        self.wait_for(lambda: any("kill timer expired" in m.data for m in self.messages["status"]))
        cutoff = self.node.get_clock().now().nanoseconds
        self.messages["motor"].clear()
        self.task(0, 5)
        self.spin(.4)
        self.assert_zero_since(cutoff)

    def mission_file(self, increments):
        path = Path(self.temp.name) / "tasks.yaml"
        path.write_text("tasks:\n" + "".join(
            f"  - {{x: 0, y: 0, z: {z}, roll: 0, pitch: 0, yaw: 0}}\n" for z in increments))
        return str(path)

    def test_planner_accumulation_retries_and_timeout(self):
        self.launch("snappy_autonomy", "planner", dict(task_file=self.mission_file([1, 1, 1, 1]), task_timeout_s=2.0))
        self.wait_for(lambda: len(self.messages["tasks"]) >= 2)
        self.assertTrue(all(m.seq == 0 and m.z == 1 for m in self.messages["tasks"]))
        self.done_pub.publish(Int32(data=99))
        self.spin(.1)
        self.assertEqual(self.messages["tasks"][-1].seq, 0)
        for seq in range(3):
            self.done_pub.publish(Int32(data=seq))
            self.wait_for(lambda: self.messages["tasks"][-1].seq == seq + 1)
            self.assertEqual(self.messages["tasks"][-1].z, seq + 2)
        self.wait_for(lambda: self.messages["abort"])
        self.assertIn("timed out", self.messages["abort"][-1].data)

    def test_real_planner_controller_handshake(self):
        self.pose(0)
        self.controller()
        self.launch("snappy_autonomy", "planner", dict(task_file=self.mission_file([1, .5]), task_timeout_s=10.0))
        self.wait_for(lambda: self.messages["target"] and self.messages["target"][-1].position.z == 1)
        self.wait_for(lambda: self.messages["wrench"])
        self.pose(1)
        self.wait_for(lambda: self.messages["target"][-1].position.z == 1.5)
        self.pose(1.5)
        self.wait_for(lambda: any(m.data.startswith("completed:") for m in self.messages["planner"]))
        self.messages["motor"].clear()
        self.spin(.3)
        self.assertTrue(self.messages["motor"])
        self.assertTrue(all(any(m.thrust_pct) for m in self.messages["motor"]), "Mission completion dropped depth hold")

    def test_invalid_configuration_exits_cleanly(self):
        process = self.controller(pid_z="[1.0, 0.0]")
        self.assertEqual(process.wait(timeout=10), 1)
        self.assertFalse(self.messages["motor"])

    def test_estimator_requires_fresh_depth(self):
        self.launch("snappy_estimation", "state_estimator", dict(sensor_timeout_s=.2))
        self.imu_input = Imu()
        self.imu_input.orientation.w = 1.0
        self.imu_input.linear_acceleration.z = 9.81
        self.spin(.4)
        self.assertFalse(self.messages["estimate"])
        self.depth_input = 1.0
        self.wait_for(lambda: len(self.messages["estimate"]) > 3)
        self.depth_input = None
        self.spin(.5)
        count = len(self.messages["estimate"])
        self.spin(.3)
        self.assertEqual(len(self.messages["estimate"]), count, "Estimator concealed loss of depth feedback")
        self.depth_input = 1.0
        self.wait_for(lambda: len(self.messages["estimate"]) > count)
        self.imu_input.orientation.w = float("nan")
        self.spin(.1)
        count = len(self.messages["estimate"])
        self.spin(.2)
        self.assertEqual(len(self.messages["estimate"]), count)
        self.imu_input.orientation.w = 1.0
        self.wait_for(lambda: len(self.messages["estimate"]) > count)
        self.assertTrue(math.isfinite(self.messages["estimate"][-1].position.z))

    def test_invalid_state_latches_stop(self):
        self.pose(0)
        self.controller(wait_for_task="false")
        self.wait_for(lambda: any(any(m.thrust_pct) for m in self.messages["motor"]))
        self.state.orientation.w = float("nan")
        self.wait_for(lambda: any("invalid estimator state" in m.data for m in self.messages["status"]))
        cutoff = self.node.get_clock().now().nanoseconds
        self.pose(0)
        self.spin(.2)
        self.assert_zero_since(cutoff)

    def test_disabled_xy_task_is_rejected(self):
        self.pose(0)
        self.controller()
        self.wait_for(lambda: self.messages["motor"])
        self.task(0, 1, x=1.0)
        self.wait_for(lambda: any("enable_xy_control is false" in m.data for m in self.messages["status"]))
        self.assertFalse(self.messages["wrench"])

    def test_retries_do_not_extend_controller_task_timeout(self):
        self.pose(0)
        self.controller(task_timeout_s=.6)
        self.wait_for(lambda: self.messages["motor"])
        self.task(0, 2)
        self.wait_for(lambda: self.messages["wrench"])
        for _ in range(8):
            self.task(0, 2)
            self.spin(.1)
        self.wait_for(lambda: any("task timeout" in m.data for m in self.messages["status"]), timeout=1)
        cutoff = self.node.get_clock().now().nanoseconds
        self.task(1, 0)
        self.spin(.2)
        self.assert_zero_since(cutoff)

    def test_depth_serial_partial_lines_and_invalid_values(self):
        master, slave = pty.openpty()
        try:
            self.launch("snappy_drivers", "pressureSensor", dict(serial_port=os.ttyname(slave)))
            self.spin(.5)
            os.write(master, b"D 1.")
            self.spin(.2)
            self.assertFalse(self.messages["depth"])
            os.write(master, b"25\r\n")
            self.wait_for(lambda: self.messages["depth"])
            self.assertEqual(self.messages["depth"][-1].data, 1.25)
            count = len(self.messages["depth"])
            os.write(master, b"D nan\nD inf\nD 1.25garbage\n")
            self.spin(.3)
            self.assertEqual(len(self.messages["depth"]), count)
            os.write(master, b"D -0.06\n")
            self.wait_for(lambda: len(self.messages["depth"]) > count)
            self.assertAlmostEqual(self.messages["depth"][-1].data, -.06, places=5)
        finally:
            os.close(master)
            os.close(slave)

    def test_launch_rejects_serial_alias_collision(self):
        from launch import LaunchContext
        path = Path(__file__).resolve().parents[3] / "src/launch/launch/snappy_realsense.launch.py"
        spec = importlib.util.spec_from_file_location("snappy_launch_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations.update(serial_dev="/dev/ttyUSB1", imu_port="/dev/ttyUSB2", depth_port="/dev/ttyUSB0")
        self.assertEqual(module.validate_serial_ports(context), [])
        alias = Path(self.temp.name) / "imu_alias"
        alias.symlink_to("/dev/ttyUSB1")
        context.launch_configurations["imu_port"] = str(alias)
        with self.assertRaises(RuntimeError):
            module.validate_serial_ports(context)


if __name__ == "__main__":
    unittest.main()
