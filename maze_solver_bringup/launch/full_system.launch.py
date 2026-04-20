from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    world = LaunchConfiguration('world')
    rviz_config = LaunchConfiguration('rviz_config')
    robot_xacro = LaunchConfiguration('robot_xacro')
    gui = LaunchConfiguration('gui')
    server = LaunchConfiguration('server')
    gazebo_model_path = PathJoinSubstitution([FindPackageShare('maze_solver_description')])
    gazebo_resource_path = PathJoinSubstitution([FindPackageShare('maze_solver_bringup')])

    robot_description = Command(['xacro', ' ', robot_xacro])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': world,
            'verbose': 'true',
            'gui': gui,
            'server': server,
        }.items(),
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_maze_robot',
        output='screen',
        arguments=[
            '-entity', 'maze_bot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
    )

    perception_node = Node(
        package='maze_solver_perception',
        executable='perception_node',
        name='maze_perception_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    logic_node = Node(
        package='maze_solver_logic',
        executable='logic_node',
        name='maze_logic_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    diagnostics_node = Node(
        package='maze_solver_diagnostics',
        executable='diagnostics_node',
        name='maze_diagnostics_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('maze_solver_bringup'),
                'config',
                'solver.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('maze_solver_bringup'),
                'worlds',
                'maze.world',
            ]),
        ),
        DeclareLaunchArgument(
            'robot_xacro',
            default_value=PathJoinSubstitution([
                FindPackageShare('maze_solver_description'),
                'urdf',
                'maze_bot.urdf.xacro',
            ]),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('maze_solver_description'),
                'rviz',
                'maze_solver.rviz',
            ]),
        ),
        DeclareLaunchArgument('spawn_x', default_value='-0.55'),
        DeclareLaunchArgument('spawn_y', default_value='-0.55'),
        DeclareLaunchArgument('spawn_z', default_value='0.08'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('server', default_value='true'),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                '/usr/share/gazebo-11/models',
                ':',
                gazebo_model_path,
            ],
        ),
        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value=[
                gazebo_resource_path,
                ':',
                EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value=''),
            ],
        ),
        gazebo,
        state_publisher,
        spawn_robot,
        perception_node,
        logic_node,
        diagnostics_node,
        rviz,
    ])
