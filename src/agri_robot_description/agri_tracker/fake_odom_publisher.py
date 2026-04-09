#!/usr/bin/env python3

import math

from geometry_msgs.msg import TransformStamped, Twist
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.declare_parameter('snap_to_goal', True)
        self.declare_parameter('target_x', 15.0)
        self.declare_parameter('target_y', 5.0)
        self.declare_parameter('goal_snap_tolerance', 0.35)
        self.declare_parameter('zero_cmd_threshold', 1e-3)
        self.declare_parameter('auto_shutdown_on_target', False)

        self.snap_to_goal = bool(self.get_parameter('snap_to_goal').value)
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.goal_snap_tolerance = float(self.get_parameter('goal_snap_tolerance').value)
        self.zero_cmd_threshold = float(self.get_parameter('zero_cmd_threshold').value)
        self.auto_shutdown_on_target = bool(self.get_parameter('auto_shutdown_on_target').value)

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.linear_speed_cmd = 0.0
        self.angular_speed_cmd = 0.0
        self.goal_snapped = False
        self.get_logger().info('FakeOdomPublisher started')

    def cmd_callback(self, msg: Twist):
        self.linear_speed_cmd = float(msg.linear.x)
        self.angular_speed_cmd = float(msg.angular.z)

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            dt = 0.1

        # Integrate planar differential-drive kinematics from commanded velocities.
        self.x += self.linear_speed_cmd * math.cos(self.yaw) * dt
        self.y += self.linear_speed_cmd * math.sin(self.yaw) * dt
        self.yaw += self.angular_speed_cmd * dt

        self._maybe_snap_to_goal_and_shutdown()

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Planar quaternion uses only z and w for yaw rotation.
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = self.linear_speed_cmd
        msg.twist.twist.angular.z = self.angular_speed_cmd

        self.pub.publish(msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

    def _maybe_snap_to_goal_and_shutdown(self):
        if not self.snap_to_goal or self.goal_snapped:
            return

        dx = self.target_x - self.x
        dy = self.target_y - self.y
        near_goal = math.hypot(dx, dy) <= self.goal_snap_tolerance
        stopped = (
            abs(self.linear_speed_cmd) <= self.zero_cmd_threshold
            and abs(self.angular_speed_cmd) <= self.zero_cmd_threshold
        )
        if not (near_goal and stopped):
            return

        self.x = self.target_x
        self.y = self.target_y
        self.goal_snapped = True
        self.get_logger().info(
            f'Reached target pose ({self.target_x:.2f}, {self.target_y:.2f}); snapping odometry and finishing'
        )
        if self.auto_shutdown_on_target and rclpy.ok():
            self.get_logger().info('auto_shutdown_on_target enabled; shutting down fake odom node')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()