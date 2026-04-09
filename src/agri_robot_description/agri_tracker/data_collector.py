#!/usr/bin/env python3

import json
import os
from math import hypot
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class DataCollector(Node):
    """Collect trajectory, control, and cross-track error data for analysis."""

    def __init__(self):
        super().__init__('data_collector')

        self.declare_parameter(
            'log_path',
            'sim_log.json',
            ParameterDescriptor(description='Output JSON file path for logged data'),
        )
        self.declare_parameter(
            'autosave_interval',
            1.0,
            ParameterDescriptor(description='Periodic log flush interval in seconds (0 disables autosave)'),
        )
        self.log_path = str(self.get_parameter('log_path').value)
        self.autosave_interval = float(self.get_parameter('autosave_interval').value)

        # Keep waypoints aligned with the tracker defaults.
        self.waypoints: List[Tuple[float, float]] = [
            (0.0, 0.0),
            (5.0, 0.0),
            (10.0, 5.0),
            (15.0, 5.0),
        ]

        self.trajectory: List[Tuple[float, float]] = []
        self.control_commands: List[Tuple[float, float]] = []
        self.cross_track_errors: List[float] = []

        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self.autosave_timer = None
        if self.autosave_interval > 0.0:
            self.autosave_timer = self.create_timer(self.autosave_interval, self._autosave_timer_callback)

        self.get_logger().info(f'Data collector initialized. Log path: {self.log_path}')

    def _odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.trajectory.append((x, y))
        self.cross_track_errors.append(self._compute_cross_track_error(x, y))

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.control_commands.append((msg.linear.x, msg.angular.z))

    def _compute_cross_track_error(self, robot_x: float, robot_y: float) -> float:
        min_error = float('inf')
        for i in range(len(self.waypoints) - 1):
            x1, y1 = self.waypoints[i]
            x2, y2 = self.waypoints[i + 1]
            error = self._perpendicular_distance_to_segment(robot_x, robot_y, x1, y1, x2, y2)
            min_error = min(min_error, error)
        return min_error

    @staticmethod
    def _perpendicular_distance_to_segment(
        px: float,
        py: float,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
    ) -> float:
        dx_segment = x2 - x1
        dy_segment = y2 - y1
        dx_point = px - x1
        dy_point = py - y1

        segment_length_sq = dx_segment * dx_segment + dy_segment * dy_segment
        if segment_length_sq == 0.0:
            return hypot(dx_point, dy_point)

        t = (dx_point * dx_segment + dy_point * dy_segment) / segment_length_sq
        t = max(0.0, min(1.0, t))

        closest_x = x1 + t * dx_segment
        closest_y = y1 + t * dy_segment
        return hypot(px - closest_x, py - closest_y)

    def _save_data_to_json(self, quiet: bool = False) -> None:
        data = {
            'waypoints': self.waypoints,
            'trajectory': self.trajectory,
            'error': self.cross_track_errors,
            'control': self.control_commands,
        }

        try:
            log_dir = os.path.dirname(self.log_path)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)

            with open(self.log_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)

            if rclpy.ok() and not quiet:
                self.get_logger().info(
                    f'Data saved to {self.log_path}: '
                    f'{len(self.trajectory)} trajectory points, '
                    f'{len(self.control_commands)} control commands'
                )
        except OSError as e:
            if rclpy.ok():
                self.get_logger().error(f'Failed to save data to {self.log_path}: {e}')

    def _autosave_timer_callback(self) -> None:
        # Keep the log file updated while nodes are running.
        self._save_data_to_json(quiet=True)

    def destroy_node(self) -> None:
        if rclpy.ok():
            self.get_logger().info('Shutting down data collector...')
        self._save_data_to_json()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()