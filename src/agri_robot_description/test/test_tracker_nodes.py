#!/usr/bin/env python3

import queue
import sys
import time
import unittest
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from agri_tracker.fake_odom_publisher import FakeOdomPublisher
from agri_tracker.tracker import PurePursuitTracker


class CmdVelProbe(Node):
    def __init__(self):
        super().__init__('cmd_vel_probe')
        self.messages = queue.Queue()
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

    def on_cmd_vel(self, msg):
        self.messages.put(msg)


class TestTrackerIntegration(unittest.TestCase):
    def test_tracker_receives_odom_and_publishes_cmd_vel(self):
        rclpy.init()
        executor = MultiThreadedExecutor()

        fake_odom_publisher = FakeOdomPublisher()
        pure_pursuit_tracker = PurePursuitTracker()
        cmd_vel_probe = CmdVelProbe()

        executor.add_node(fake_odom_publisher)
        executor.add_node(pure_pursuit_tracker)
        executor.add_node(cmd_vel_probe)

        try:
            # Give enough time for the default path to complete at default speed.
            deadline = time.time() + 45.0
            captured_messages = []
            saw_nonzero_command = False

            while time.time() < deadline and not pure_pursuit_tracker.complete:
                executor.spin_once(timeout_sec=0.1)
                while True:
                    try:
                        msg = cmd_vel_probe.messages.get_nowait()
                        captured_messages.append(msg)
                        if abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6:
                            saw_nonzero_command = True
                    except queue.Empty:
                        break

            self.assertTrue(captured_messages, 'Tracker did not publish any /cmd_vel messages')
            self.assertTrue(saw_nonzero_command, 'Tracker only published zero /cmd_vel commands')
            self.assertTrue(pure_pursuit_tracker.complete, 'Tracker did not complete the full waypoint path')

            target_x, target_y = pure_pursuit_tracker.waypoints[-1]
            dx = target_x - pure_pursuit_tracker.robot_x
            dy = target_y - pure_pursuit_tracker.robot_y
            self.assertLessEqual(
                (dx * dx + dy * dy) ** 0.5,
                pure_pursuit_tracker.waypoint_tolerance + 0.2,
                'Final robot pose is not near the final waypoint',
            )
        finally:
            executor.remove_node(cmd_vel_probe)
            executor.remove_node(pure_pursuit_tracker)
            executor.remove_node(fake_odom_publisher)
            cmd_vel_probe.destroy_node()
            pure_pursuit_tracker.destroy_node()
            fake_odom_publisher.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()