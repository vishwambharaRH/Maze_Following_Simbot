from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        Node(
            package='maze_solver_logic',
            executable='logic_node',
            name='maze_logic_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }],
        ),
    ])
