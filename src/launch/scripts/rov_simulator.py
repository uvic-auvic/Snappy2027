#!/usr/bin/env python3
"""Fixed sensor fixtures and PS4 Joy scenarios; never publishes motor commands."""

import argparse
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Joy
from snappy_interfaces.msg import Pose
from std_msgs.msg import Float32

SCENARIOS = (
    "neutral", "half_forward", "full_reverse", "both_triggers", "strafe_left",
    "strafe_right", "yaw_left", "yaw_right", "depth_up", "depth_down",
    "held_dpad", "deadman_release", "emergency_stop", "disconnect", "malformed",
)


class RovSimulator(Node):
    def __init__(self, scenario, depth):
        super().__init__("rov_simulator")
        self.scenario, self.depth = scenario, depth
        self.started = time.monotonic()
        self.joy = self.create_publisher(Joy, "/joy", 1)
        self.pressure = self.create_publisher(Float32, "/depth_data", 1)
        self.state = self.create_publisher(Pose, "/state_estimator/state", 1)
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info(f"Synthetic depth {depth} m; scenario {scenario}; no motor output")

    def tick(self):
        elapsed = time.monotonic() - self.started
        self.pressure.publish(Float32(data=self.depth))
        pose = Pose()
        pose.position.z, pose.orientation.w = self.depth, 1.0
        self.state.publish(pose)
        if self.scenario == "disconnect" and elapsed > 2.0:
            return
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        msg.buttons = [0] * 13
        # Give discovery and sensors time, then observe L1 released before arming.
        if elapsed > 1.0 and self.scenario != "neutral":
            msg.buttons[4] = 1
            if self.scenario == "half_forward":
                msg.axes[5] = 0.0
            elif self.scenario == "full_reverse":
                msg.axes[2] = -1.0
            elif self.scenario == "both_triggers":
                msg.axes[5], msg.axes[2] = -0.6, 0.4  # 80% - 30% = 50%
            elif self.scenario in ("strafe_left", "strafe_right"):
                msg.axes[0] = 1.0 if self.scenario == "strafe_left" else -1.0
            elif self.scenario in ("yaw_left", "yaw_right"):
                msg.axes[3] = 1.0 if self.scenario == "yaw_left" else -1.0
            elif self.scenario in ("depth_up", "depth_down", "held_dpad"):
                if self.scenario == "held_dpad" or (elapsed % 1.0) < 0.2:
                    msg.axes[7] = 1.0 if self.scenario == "depth_up" else -1.0
            elif self.scenario == "deadman_release":
                msg.axes[5] = -1.0
                msg.buttons[4] = int(elapsed < 2.0)
            elif self.scenario == "emergency_stop":
                msg.axes[5] = -1.0
                msg.buttons[1] = int(2.0 < elapsed < 2.2)
            elif self.scenario == "disconnect":
                msg.axes[5] = -1.0
            elif self.scenario == "malformed":
                msg.axes = []
        self.joy.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, default="neutral")
    parser.add_argument("--depth", type=float, default=2.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = RovSimulator(args.scenario, args.depth)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal signal and launch's forwarded SIGINT can both arrive.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
