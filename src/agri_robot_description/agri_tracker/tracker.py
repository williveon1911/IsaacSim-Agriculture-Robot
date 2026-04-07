# agri_robot_description/agri_tracker/tracker.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, sin, cos, pi, tan
import numpy as np

class PurePursuitTracker(Node):
    def __init__(self):
        super().__init__('pure_pursuit_tracker')
        
        # Parameters
        self.lookahead_distance = 2.0
        self.linear_speed = 0.5
        self.wheelbase = 0.5
        
        # Waypoints
        self.waypoints = [(0,0), (5,0), (10,5), (15,5)]
        self.current_idx = 0
        self.complete = False
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Pure Pursuit Tracker Ready')
    
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        self.robot_yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    
    def control_loop(self):
        if self.complete:
            self.publish_cmd(0, 0)
            return
        
        target_x, target_y = self.waypoints[self.current_idx]
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = sqrt(dx*dx + dy*dy)
        
        # Check if waypoint reached
        if distance < 0.3:
            self.get_logger().info(f'Reached waypoint {self.current_idx}')
            self.current_idx += 1
            if self.current_idx >= len(self.waypoints):
                self.complete = True
                self.publish_cmd(0, 0)
                return
            target_x, target_y = self.waypoints[self.current_idx]
            dx = target_x - self.robot_x
            dy = target_y - self.robot_y
        
        # Pure pursuit calculation
        if distance <= self.lookahead_distance:
            lookahead = (target_x, target_y)
        else:
            t = self.lookahead_distance / distance
            lookahead = (self.robot_x + t*dx, self.robot_y + t*dy)
        
        # Steering angle
        angle_to = atan2(lookahead[1] - self.robot_y, lookahead[0] - self.robot_x)
        alpha = angle_to - self.robot_yaw
        
        # Normalize alpha
        while alpha > pi: alpha -= 2*pi
        while alpha < -pi: alpha += 2*pi
        
        steering = atan2(2 * self.wheelbase * sin(alpha), self.lookahead_distance)
        steering = np.clip(steering, -pi/6, pi/6)
        
        # Command velocities
        angular = self.linear_speed * tan(steering) / self.wheelbase
        self.publish_cmd(self.linear_speed, angular)
    
    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitTracker()
    rclpy.spin(node)

if __name__ == '__main__':
    main()