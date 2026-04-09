from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agri_robot_description',
            executable='fake_odom_publisher',
            name='fake_odom_publisher',
            output='screen',
            parameters=[
                {
                    'snap_to_goal': True,
                    'target_x': 15.0,
                    'target_y': 5.0,
                    'goal_snap_tolerance': 0.35,
                    'auto_shutdown_on_target': True,
                }
            ],
        ),
        Node(
            package='agri_robot_description',
            executable='pure_pursuit_tracker',
            name='pure_pursuit_tracker',
            output='screen',
            parameters=[
                {
                    'shutdown_on_complete': True,
                }
            ],
        ),
    ])