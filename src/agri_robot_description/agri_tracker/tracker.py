#!/usr/bin/env python3

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from std_srvs.srv import Trigger

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from math import atan2, hypot, pi, sin, tan
from typing import List, Tuple


class PurePursuitTracker(Node):
    def __init__(self):
        super().__init__('pure_pursuit_tracker')

        # Runtime-tunable tracking parameters.
        self.declare_parameter(
            'lookahead_distance',
            1.5,
            ParameterDescriptor(description='Pure Pursuit lookahead distance in meters'),
        )
        self.declare_parameter(
            'linear_speed',
            0.35,
            ParameterDescriptor(description='Desired linear speed in m/s'),
        )
        self.declare_parameter(
            'control_period',
            0.05,
            ParameterDescriptor(description='Control loop period in seconds (smaller = more control steps)'),
        )
        self.declare_parameter(
            'off_track_speed_scale',
            0.35,
            ParameterDescriptor(description='Linear speed multiplier used when path deviation is high'),
        )
        self.declare_parameter(
            'off_track_error_threshold',
            0.15,
            ParameterDescriptor(description='Path deviation threshold that enables slower off-track speed'),
        )
        self.declare_parameter(
            'max_steering',
            pi / 4.5,
            ParameterDescriptor(description='Maximum steering angle in radians'),
        )
        self.declare_parameter(
            'off_track_angular_gain',
            1.8,
            ParameterDescriptor(description='Angular command gain applied while off-track recovery is active'),
        )
        self.declare_parameter(
            'max_angular_speed',
            1.2,
            ParameterDescriptor(description='Maximum absolute angular velocity command in rad/s'),
        )
        self.declare_parameter(
            'waypoint_tolerance',
            0.3,
            ParameterDescriptor(description='Distance tolerance to consider waypoint reached'),
        )
        self.declare_parameter(
            'shutdown_on_complete',
            False,
            ParameterDescriptor(description='If true, shutdown this node when trajectory is complete'),
        )

        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.control_period = float(self.get_parameter('control_period').value)
        self.off_track_speed_scale = float(self.get_parameter('off_track_speed_scale').value)
        self.off_track_error_threshold = float(self.get_parameter('off_track_error_threshold').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.off_track_angular_gain = float(self.get_parameter('off_track_angular_gain').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.shutdown_on_complete = bool(self.get_parameter('shutdown_on_complete').value)
        self.add_on_set_parameters_callback(self._on_parameter_changed)

        self.waypoints: List[Tuple[float, float]] = [
            (0.0, 0.0),
            (5.0, 0.0),
            (10.0, 5.0),
            (15.0, 5.0),
        ]
        self.current_idx = 0
        self.complete = False
        self.off_track_mode = False

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.reset_srv = self.create_service(Trigger, '/pure_pursuit/reset', self._reset_callback)
        self.timer = self.create_timer(self.control_period, self._control_loop)

        self.get_logger().info('Pure Pursuit Tracker Ready')

    def _on_parameter_changed(self, parameters) -> SetParametersResult:
        for param in parameters:
            if param.name == 'lookahead_distance':
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='lookahead_distance must be > 0.0',
                    )
                self.lookahead_distance = float(param.value)
            elif param.name == 'linear_speed':
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='linear_speed must be > 0.0',
                    )
                self.linear_speed = float(param.value)
            elif param.name == 'control_period':
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='control_period must be > 0.0',
                    )
                self.control_period = float(param.value)
                self.timer.cancel()
                self.timer = self.create_timer(self.control_period, self._control_loop)
            elif param.name == 'off_track_speed_scale':
                if float(param.value) <= 0.0 or float(param.value) > 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason='off_track_speed_scale must be in the range (0.0, 1.0]',
                    )
                self.off_track_speed_scale = float(param.value)
            elif param.name == 'off_track_error_threshold':
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='off_track_error_threshold must be > 0.0',
                    )
                self.off_track_error_threshold = float(param.value)
            elif param.name == 'max_steering':
                if float(param.value) <= 0.0 or float(param.value) >= (pi / 2.0):
                    return SetParametersResult(
                        successful=False,
                        reason='max_steering must be in the range (0.0, pi/2)',
                    )
                self.max_steering = float(param.value)
            elif param.name == 'off_track_angular_gain':
                if float(param.value) < 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason='off_track_angular_gain must be >= 1.0',
                    )
                self.off_track_angular_gain = float(param.value)
            elif param.name == 'max_angular_speed':
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='max_angular_speed must be > 0.0',
                    )
                self.max_angular_speed = float(param.value)
            elif param.name == 'waypoint_tolerance':
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='waypoint_tolerance must be > 0.0',
                    )
                self.waypoint_tolerance = float(param.value)
            elif param.name == 'shutdown_on_complete':
                self.shutdown_on_complete = bool(param.value)

        return SetParametersResult(successful=True)

    def _odom_callback(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)

    @staticmethod
    def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def _control_loop(self) -> None:
        if self.complete:
            self._publish_cmd(0.0, 0.0)
            return

        while self.current_idx < len(self.waypoints):
            target_x, target_y = self.waypoints[self.current_idx]
            distance = hypot(target_x - self.robot_x, target_y - self.robot_y)
            if distance > self.waypoint_tolerance:
                break
            self.get_logger().info(f'Reached waypoint {self.current_idx}')
            self.current_idx += 1
            if self.current_idx >= len(self.waypoints):
                self.complete = True
                self._publish_cmd(0.0, 0.0)
                self.get_logger().info('Trajectory complete')
                if self.shutdown_on_complete and rclpy.ok():
                    self.get_logger().info('shutdown_on_complete enabled; shutting down tracker node')
                    rclpy.shutdown()
                return

        target_x, target_y = self.waypoints[self.current_idx]
        lookahead_x, lookahead_y = self._find_lookahead_point(target_x, target_y)
        path_error = self._path_deviation()
        self._update_speed_mode(path_error)

        angle_to = atan2(lookahead_y - self.robot_y, lookahead_x - self.robot_x)
        alpha = self._normalize_angle(angle_to - self.robot_yaw)

        wheelbase = 1.0
        steering = atan2(2.0 * wheelbase * sin(alpha), max(self.lookahead_distance, 1e-6))

        steering = max(-self.max_steering, min(self.max_steering, steering))

        effective_speed = self.linear_speed
        if self.off_track_mode:
            effective_speed *= self.off_track_speed_scale
        angular = effective_speed * tan(steering) / wheelbase
        if self.off_track_mode:
            angular *= self.off_track_angular_gain
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))
        self._publish_cmd(effective_speed, angular)

    def _path_deviation(self) -> float:
        if self.current_idx <= 0:
            target_x, target_y = self.waypoints[0]
            return hypot(target_x - self.robot_x, target_y - self.robot_y)

        start_x, start_y = self.waypoints[self.current_idx - 1]
        end_x, end_y = self.waypoints[self.current_idx]
        dx_segment = end_x - start_x
        dy_segment = end_y - start_y
        segment_length_sq = dx_segment * dx_segment + dy_segment * dy_segment
        if segment_length_sq == 0.0:
            return hypot(self.robot_x - start_x, self.robot_y - start_y)

        dx_point = self.robot_x - start_x
        dy_point = self.robot_y - start_y
        t = (dx_point * dx_segment + dy_point * dy_segment) / segment_length_sq
        t = max(0.0, min(1.0, t))

        closest_x = start_x + t * dx_segment
        closest_y = start_y + t * dy_segment
        return hypot(self.robot_x - closest_x, self.robot_y - closest_y)

    def _update_speed_mode(self, path_error: float) -> None:
        if self.off_track_mode:
            if path_error <= self.off_track_error_threshold * 0.5:
                self.off_track_mode = False
                self.get_logger().info('Path deviation recovered; returning to nominal speed')
        elif path_error >= self.off_track_error_threshold:
            self.off_track_mode = True
            self.get_logger().info(
                f'Path deviation {path_error:.3f} m exceeded threshold; slowing down to recover'
            )

    def _find_lookahead_point(self, target_x: float, target_y: float) -> Tuple[float, float]:
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = hypot(dx, dy)

        if distance <= self.lookahead_distance:
            return target_x, target_y

        scale = self.lookahead_distance / max(distance, 1e-6)
        return self.robot_x + scale * dx, self.robot_y + scale * dy

    def _publish_cmd(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def _reset_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.current_idx = 0
        self.complete = False
        self._publish_cmd(0.0, 0.0)
        self.get_logger().info('Trajectory reset')
        response.success = True
        response.message = 'Trajectory reset successfully'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitTracker()
    try:
        rclpy.spin(node)
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()