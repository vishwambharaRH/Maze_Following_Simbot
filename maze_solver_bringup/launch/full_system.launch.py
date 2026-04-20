from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true',
        }.items(),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_maze_robot',
        output='screen',
        arguments=[
            '-entity', 'maze_bot',
            '-file', LaunchConfiguration('robot_model'),
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot_model'), "' != ''"])),
    )

    perception_node = Node(
        package=LaunchConfiguration('perception_package'),
        executable=LaunchConfiguration('perception_executable'),
        name='perception_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('enable_perception')),
    )

    diagnostics_node = Node(
        package=LaunchConfiguration('diagnostics_package'),
        executable=LaunchConfiguration('diagnostics_executable'),
        name='diagnostics_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('enable_diagnostics')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_rviz'),
            "' == 'true' and '",
            LaunchConfiguration('rviz_config'),
            "' != ''",
        ])),
    )

    logic_node = Node(
        package='maze_solver_logic',
        executable='logic_node',
        name='maze_logic_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='/usr/share/gazebo-11/worlds/empty.world'),
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.05'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('enable_perception', default_value='false'),
        DeclareLaunchArgument('perception_package', default_value='maze_perception'),
        DeclareLaunchArgument('perception_executable', default_value='distance_filter_node'),
        DeclareLaunchArgument('enable_diagnostics', default_value='false'),
        DeclareLaunchArgument('diagnostics_package', default_value='maze_diagnostics'),
        DeclareLaunchArgument('diagnostics_executable', default_value='watchdog_node'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=''),
        gazebo,
        spawn_robot,
        perception_node,
        diagnostics_node,
        logic_node,
        rviz,
    ])
