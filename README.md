# Predictive Maze Solver

ROS2 Humble workspace scaffold for a predictive wall-following maze robot.

## Packages

- `maze_solver_logic`: State machine, predictive wall-follow controller, and `/cmd_vel` output.
- `maze_solver_bringup`: Launch files for logic-only runs or full Gazebo integration.

## Quick Start

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch maze_solver_bringup logic_only.launch.py
```

## Full Simulation Hook Points

```bash
ros2 launch maze_solver_bringup full_system.launch.py \
  world:=/absolute/path/to/maze.world \
  robot_model:=/absolute/path/to/robot.urdf \
  enable_perception:=true \
  perception_package:=maze_perception \
  perception_executable:=distance_filter_node \
  enable_diagnostics:=true \
  diagnostics_package:=maze_diagnostics \
  diagnostics_executable:=watchdog_node
```

The logic node expects a filtered `sensor_msgs/msg/LaserScan` on `/scan_filtered` by default.
