# Predictive Maze Solver

ROS2 Humble workspace for a predictive wall-following maze robot.

## Packages

- `maze_solver_interfaces`: Custom regional-distance message.
- `maze_solver_perception`: Cleans `/scan`, publishes `/scan_filtered` and `/distance_regions`.
- `maze_solver_logic`: State machine plus predictive PID wall follower.
- `maze_solver_diagnostics`: Collision watchdog and solve-time logger.
- `maze_solver_description`: Xacro robot model and RViz config.
- `maze_solver_bringup`: Gazebo world, params, and launch orchestration.

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Full Demo

```bash
ros2 launch maze_solver_bringup full_system.launch.py
```

## Launch Logic Only

```bash
ros2 launch maze_solver_bringup logic_only.launch.py
```

## Topics

- `/scan`: Raw Gazebo LiDAR
- `/scan_filtered`: Median-filtered LiDAR
- `/distance_regions`: Simplified front-left-right distances with rates
- `/cmd_vel`: Velocity command
- `/odom`: Diff-drive odometry
- `/maze_solver/goal_reached`: Goal flag
- `/maze_solver/status`: Status string
