from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    start_joint_state_publisher = LaunchConfiguration('start_joint_state_publisher')
    use_fake_odom = LaunchConfiguration('use_fake_odom')
    start_data_collector = LaunchConfiguration('start_data_collector')
    run_tracker = LaunchConfiguration('run_tracker')
    start_visualizer = LaunchConfiguration('start_visualizer')
    visualizer_show = LaunchConfiguration('visualizer_show')
    visualizer_delay = LaunchConfiguration('visualizer_delay')
    visualizer_plots_dir = LaunchConfiguration('visualizer_plots_dir')
    collector_log_path = LaunchConfiguration('collector_log_path')
    lookahead_distance = LaunchConfiguration('lookahead_distance')
    linear_speed = LaunchConfiguration('linear_speed')
    max_steering = LaunchConfiguration('max_steering')
    control_period = LaunchConfiguration('control_period')
    off_track_speed_scale = LaunchConfiguration('off_track_speed_scale')
    off_track_error_threshold = LaunchConfiguration('off_track_error_threshold')
    off_track_angular_gain = LaunchConfiguration('off_track_angular_gain')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    waypoint_tolerance = LaunchConfiguration('waypoint_tolerance')
    shutdown_on_complete = LaunchConfiguration('shutdown_on_complete')
    fake_odom_snap_to_goal = LaunchConfiguration('fake_odom_snap_to_goal')
    fake_odom_target_x = LaunchConfiguration('fake_odom_target_x')
    fake_odom_target_y = LaunchConfiguration('fake_odom_target_y')
    fake_odom_goal_snap_tolerance = LaunchConfiguration('fake_odom_goal_snap_tolerance')
    fake_odom_auto_shutdown_on_target = LaunchConfiguration('fake_odom_auto_shutdown_on_target')
    rviz_fixed_frame = LaunchConfiguration('rviz_fixed_frame')

    urdf_path = PathJoinSubstitution(
        [FindPackageShare('agri_robot_description'), 'urdf', 'robot.urdf.xacro']
    )
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    def launch_visualizer_process(context, show_visualizer: bool):
        collector_log_path_value = LaunchConfiguration('collector_log_path').perform(context)
        visualizer_delay_value = LaunchConfiguration('visualizer_delay').perform(context)
        visualizer_plots_dir_value = LaunchConfiguration('visualizer_plots_dir').perform(context)
        show_flag = '--show' if show_visualizer else '--no-show'
        visualizer_command = (
            f'sleep {visualizer_delay_value}; '
            'for i in $(seq 1 120); do [ -f "'
            f'{collector_log_path_value}'
            '" ] && break; sleep 1; done; '
            'if [ -f "'
            f'{collector_log_path_value}'
            '" ]; then ros2 run agri_robot_description tracker_visualizer --run "'
            f'{collector_log_path_value}'
            '" --plots-dir "'
            f'{visualizer_plots_dir_value}'
            f'" {show_flag}; else echo "[full_stack] visualizer skipped: log file not found"; fi'
        )
        return [
            ExecuteProcess(
                cmd=['bash', '-lc', visualizer_command],
                output='screen',
            )
        ]

    pure_pursuit_tracker_node = Node(
        package='agri_robot_description',
        executable='pure_pursuit_tracker',
        name='pure_pursuit_tracker',
        output='screen',
        parameters=[
            {
                'lookahead_distance': lookahead_distance,
                'linear_speed': linear_speed,
                'max_steering': max_steering,
                'control_period': control_period,
                'off_track_speed_scale': off_track_speed_scale,
                'off_track_error_threshold': off_track_error_threshold,
                'off_track_angular_gain': off_track_angular_gain,
                'max_angular_speed': max_angular_speed,
                'waypoint_tolerance': waypoint_tolerance,
                'shutdown_on_complete': shutdown_on_complete,
            }
        ],
        condition=IfCondition(run_tracker),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_rviz',
                default_value='true',
                description='Launch RViz2',
            ),
            DeclareLaunchArgument(
                'start_joint_state_publisher',
                default_value='true',
                description='Launch joint_state_publisher for wheel joint TF',
            ),
            DeclareLaunchArgument(
                'use_fake_odom',
                    default_value='true',
                description='Launch fake odometry publisher (set false when using Isaac Sim odometry)',
            ),
            DeclareLaunchArgument(
                'start_data_collector',
                default_value='true',
                description='Launch data collector logger',
            ),
            DeclareLaunchArgument(
                'start_visualizer',
                default_value='true',
                description='Auto-launch tracker visualizer after a delay',
            ),
            DeclareLaunchArgument(
                'visualizer_show',
                default_value='true',
                description='If true, visualizer opens matplotlib windows',
            ),
            DeclareLaunchArgument(
                'visualizer_delay',
                default_value='120',
                description='Seconds to wait before launching visualizer',
            ),
            DeclareLaunchArgument(
                'visualizer_plots_dir',
                default_value='plots',
                description='Directory for visualizer outputs',
            ),
            DeclareLaunchArgument(
                'collector_log_path',
                default_value='sim_log.json',
                description='Output log path for data collector',
            ),
            DeclareLaunchArgument(
                'lookahead_distance',
                default_value='1.5',
                description='Pure Pursuit lookahead distance',
            ),
            DeclareLaunchArgument(
                'linear_speed',
                default_value='0.35',
                description='Pure Pursuit linear speed',
            ),
            DeclareLaunchArgument(
                'max_steering',
                default_value='0.85',
                description='Maximum steering angle in radians',
            ),
            DeclareLaunchArgument(
                'control_period',
                default_value='0.05',
                description='Tracker control loop period in seconds',
            ),
            DeclareLaunchArgument(
                'off_track_speed_scale',
                default_value='0.35',
                description='Speed multiplier when deviation exceeds threshold',
            ),
            DeclareLaunchArgument(
                'off_track_error_threshold',
                default_value='0.05',
                description='Deviation threshold in meters to trigger slowdown',
            ),
            DeclareLaunchArgument(
                'off_track_angular_gain',
                default_value='1.8',
                description='Angular command gain while recovering from path deviation',
            ),
            DeclareLaunchArgument(
                'max_angular_speed',
                default_value='1.2',
                description='Maximum absolute angular velocity command in rad/s',
            ),
            DeclareLaunchArgument(
                'waypoint_tolerance',
                default_value='0.15',
                description='Distance tolerance in meters to consider waypoint reached',
            ),
            DeclareLaunchArgument(
                'shutdown_on_complete',
                default_value='false',
                description='If true, tracker process exits when final waypoint is reached',
            ),
            DeclareLaunchArgument(
                'fake_odom_snap_to_goal',
                default_value='true',
                description='If true, fake odom snaps final pose to target when robot stops near goal',
            ),
            DeclareLaunchArgument(
                'fake_odom_target_x',
                default_value='15.0',
                description='Fake odom target X for final pose snap',
            ),
            DeclareLaunchArgument(
                'fake_odom_target_y',
                default_value='5.0',
                description='Fake odom target Y for final pose snap',
            ),
            DeclareLaunchArgument(
                'fake_odom_goal_snap_tolerance',
                default_value='0.35',
                description='Distance threshold for fake odom goal snap',
            ),
            DeclareLaunchArgument(
                'fake_odom_auto_shutdown_on_target',
                default_value='false',
                description='If true, fake odom exits after snapping to goal',
            ),
            DeclareLaunchArgument(
                'run_tracker',
                default_value='true',
                description='Launch pure pursuit tracker node',
            ),
            DeclareLaunchArgument(
                'rviz_fixed_frame',
                default_value='base_link',
                description='RViz fixed frame',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_chassis_tf',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'chassis'],
                output='screen',
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
                condition=IfCondition(start_joint_state_publisher),
            ),
            pure_pursuit_tracker_node,
            Node(
                package='agri_robot_description',
                executable='fake_odom_publisher',
                name='fake_odom_publisher',
                output='screen',
                parameters=[
                    {
                        'snap_to_goal': fake_odom_snap_to_goal,
                        'target_x': fake_odom_target_x,
                        'target_y': fake_odom_target_y,
                        'goal_snap_tolerance': fake_odom_goal_snap_tolerance,
                        'auto_shutdown_on_target': fake_odom_auto_shutdown_on_target,
                    }
                ],
                condition=IfCondition(use_fake_odom),
            ),
            Node(
                package='agri_robot_description',
                executable='data_collector',
                name='data_collector',
                output='screen',
                parameters=[{'log_path': collector_log_path}],
                condition=IfCondition(start_data_collector),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-f', rviz_fixed_frame],
                condition=IfCondition(start_rviz),
            ),
            OpaqueFunction(
                function=lambda context, *args, **kwargs: launch_visualizer_process(context, True),
                condition=IfCondition(
                    PythonExpression(["'", start_visualizer, "' == 'true' and '", visualizer_show, "' == 'true'"])
                ),
            ),
            OpaqueFunction(
                function=lambda context, *args, **kwargs: launch_visualizer_process(context, False),
                condition=IfCondition(
                    PythonExpression(["'", start_visualizer, "' == 'true' and '", visualizer_show, "' != 'true'"])
                ),
            ),
        ]
    )